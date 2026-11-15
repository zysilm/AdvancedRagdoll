// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using AdvancedRagdoll.CollapseProfiles;
using AdvancedRagdoll.SurfaceProfiles;
using AdvancedRagdoll.SkeletonProfiles;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.Bepu;
using BepuSimulation = BepuPhysics.Simulation;

namespace AdvancedRagdoll;

/// <summary>
/// The ragdoll of one character (docs/ARCHITECTURE.md). A game makes it on the public host API
/// (<see cref="RagdollHost"/>) and drives it with <see cref="Activate(in RagdollActivation)"/>, <see cref="Step"/>,
/// <see cref="WritePose"/> and <see cref="Deactivate"/> (Ragdoll.Facade.cs). Inside, everything it asks of the game
/// goes through the internal host services (<see cref="RagdollHostServices"/>); trusted hosts (the plugin's
/// RagdollController, which derives from it, and the offline replay) use those directly and drive it with
/// <see cref="OnRenderFrame"/> and <see cref="OnFrameworkTick"/>.
/// </summary>
public unsafe partial class Ragdoll : IDisposable
{
    // Everything the controller asks of the game goes through these host services (Hosting/; docs/ARCHITECTURE.md
    // §2).
    private readonly RagdollHostServices host;
    // The state shared with the components (SimulationContext.cs). The fields that moved there are reached through
    // the forwarding properties below under their old names, until the code using them is split into components too.
    private readonly SimulationContext ctx;
    private IHostSkeletonSource skeletons => ctx.Skeletons;
    // The character models: every mesh the controller uses comes from here, as a finished product.
    private IMeshSource meshes => ctx.Meshes;
    private IHostWorldQuery world => ctx.World;
    private IHostActors actors => ctx.Actors;
    // Who else may collide with the corpse. The extra collision actors are evaluated lazily when
    // collision is built (at Activate), not at construction — the player's persistent controller is
    // created before any companions spawn, so a snapshot would always be empty.
    private readonly IHostColliders colliders;
    private IRagdollClock clock => ctx.Clock;
    private readonly IRagdollRandom random;
    private readonly IRagdollScheduler scheduler;
    private readonly IHostSettingsSource settingsSource;
    // The settings for this frame: refreshed once at the start of every render frame. The physics
    // worker threads read this copy too (never the user's Configuration).
    private HostSettings settings => ctx.Settings;
    private IRagdollLog log => ctx.Log;
    // Physics simulation
    private BufferPool? bufferPool { get => ctx.BufferPool; set => ctx.BufferPool = value; }
    // What each pair of the ragdoll's own bodies overlapped by when it started (see the class).
    private StartOverlapAllowance? startOverlap;
    private BepuSimulation? simulation { get => ctx.Simulation; set => ctx.Simulation = value; }

    // State
    private nint targetCharacterAddress { get => ctx.TargetCharacterAddress; set => ctx.TargetCharacterAddress = value; }
    private Vector3 savedCharacterPosition; // original position before follow moved it
    private bool followWasActive;           // tracks toggle-off to restore position
    private float elapsed;
    private float activationDelay;
    // Decides the moment physics takes over from the death animation (see RagdollTrigger).
    private RagdollTrigger? trigger;
    // The pose the trigger last saw, to tell how fast the body is moving.
    private Vector3[] triggerPosePositions = Array.Empty<Vector3>();
    private float[] triggerPoseMoved = Array.Empty<float>();
    private bool triggerPoseKnown;
    private float triggerStagger;
    private float triggeredAt = -1f;

    // Animation→physics handoff velocity sampling. During the activation-delay countdown the
    // death animation drives the bones; we snapshot each frame's model-space bone poses so that
    // at handoff InitializePhysics can finite-difference the last animated frame and seed the
    // physics bodies with the animation's velocity (instead of starting at rest). Stored by bone
    // INDEX to avoid a name→index map during the countdown.
    private bool handoffSampleValid;
    private float handoffSampleDt;
    private Vector3 handoffSkelPos;
    private Quaternion handoffSkelRot;
    private Vector3 handoffSkelScale = Vector3.One;
    private Vector3[]? handoffPrevPos;
    private Quaternion[]? handoffPrevRot;
    private int handoffPrevCount;
    // Skeleton bone count captured at InitializePhysics. If it changes mid-ragdoll the draw
    // object was rebuilt and our bone indices are stale → deactivate.
    private int initialBoneCount;
    // True once Timeline.OverallSpeed has been zeroed (animation frozen). Gates the restore
    // in Deactivate. NOT the same as physicsStarted: the freeze happens inside
    // InitializePhysics, which can still fail or throw afterwards (e.g. zero qualifying
    // bones), and the restore must run on that path too or the character stays frozen forever.
    private bool animationFrozen;

    // Frame-rate-independent physics timing. The render hook fires once per rendered
    // frame at whatever framerate the game runs (30/60/120/144…). Stepping a fixed
    // 1/60 every call made the ragdoll run in slow-motion below 60fps and fast above it.
    // We measure real wall-clock dt and advance the simulation in fixed-size substeps.
    private long lastFrameTimestamp;
    private float physicsAccumulator;
    private const int MaxSubstepsPerFrame = 4;  // cap catch-up to avoid a spiral of death
    private const float SlowFrameSeconds = 1f / 30f; // slower than this, one substep a frame
    private const float MaxFrameDelta = 0.1f;   // ignore huge gaps (loading screens, hitches)
    private const float BiomechanicalSettleDuration = 1.25f;
    private const float RagdollSleepThreshold = 0.03f;

    // Skeleton world transform (captured at activation from Skeleton.Transform)
    // ModelPose is in skeleton-local space; these convert to/from world space.
    private Vector3 skelWorldPos { get => ctx.RootPosition; set => ctx.RootPosition = value; }
    private Quaternion skelWorldRot { get => ctx.RootRotation; set => ctx.RootRotation = value; }
    private Quaternion skelWorldRotInv { get => ctx.RootRotationInverse; set => ctx.RootRotationInverse = value; }
    private Vector3 skelWorldScale { get => ctx.RootScale; set => ctx.RootScale = value; }

    // Bone-to-body mapping
    private List<RagdollBone> ragdollBones => ctx.Bodies;
    private readonly HashSet<int> softKinematicBodyHandles = new();
    // Soft-tissue (SoftBody) rig bodies — excluded from ALL contact generation (see
    // RagdollNarrowPhaseCallbacks.SoftBodyBodies).
    private readonly HashSet<int> softBodyBodyHandles = new();

    private struct NpcTraversalProxyRequest
    {
        public bool Enabled;
        public Vector3 InitialRoot;
        public Vector3 TargetRoot;
        public float FloorY;
        public float VisualScale;
        public float MoveSpeed;
        public float MaxClimb;
        public float StompResponse;
        public int ConcurrentStompers;
        public bool Locomoting;
    }

    private sealed class NpcTraversalProxy
    {
        public BodyHandle Body;
        public TypedIndex Shape;
        public float Radius;
        public bool Active;
        public Vector3 CachedRoot;
        public bool HasCachedRoot;
        public bool OnCorpseSupport;
        public float Mass;
        public float GaitPhase;
    }

    // True while the active ragdoll was built by the generic (non-humanoid) path.
    // Generic rigs get extra stabilization (more solver iterations + velocity clamp)
    // because their auto-generated constraint network is stiffer/less conditioned
    // than the hand-tuned human profile.
    private bool activeRagdollIsGeneric { get => ctx.IsGeneric; set => ctx.IsGeneric = value; }

    // The skeleton profile (docs/ARCHITECTURE.md §3): which bone plays which part,
    // the bone table, ranges of motion, mass budget and the rest. The named bones (landmarks, bone
    // classes) always come from the humanoid profile, as the literal names did before, whatever rig
    // is active; only the per-class tuning switches to SkeletonProfile.Auto() for a generated rig.
    //
    // Each ragdoll has its own profile (a constructor argument) and its own bone tables built from it. A host may also
    // register a default humanoid profile once per process (UseHumanoidProfile; the plugin does it when its
    // assembly loads, and its own code reads the default bone tables below). A ragdoll on the default profile uses
    // the default tables (the same arrays), as every ragdoll did when the profile was process-wide.
    private static SkeletonProfile? humanoidProfile;
    private static RagdollBoneConfig[]? allBoneDefaults;
    private static RagdollBoneDef[]? defaultBoneDefs;

    // This ragdoll's profile and the bone configurations built from it.
    private SkeletonProfile skeletonProfile => ctx.Profile;
    private RagdollBoneConfig[] boneDefaults => ctx.BoneDefaults;
    // a6-3
    private bool isActive { get => ctx.IsActive; set => ctx.IsActive = value; }
    /// <summary>What the rig actually weighs, summed at InitializePhysics.</summary>
    private float ragdollTotalMass { get => ctx.TotalMass; set => ctx.TotalMass = value; }
    // a6-4
    // True once BEPU has put the complete ragdoll constraint island to sleep (SimulationContext.PrevAllAsleep).
    private bool prevAllAsleep { get => ctx.PrevAllAsleep; set => ctx.PrevAllAsleep = value; }
    // a6-6
    private bool physicsStarted { get => ctx.PhysicsStarted; set => ctx.PhysicsStarted = value; }
    // Short post-wake window that keeps the island awake (SimulationContext.BiomechanicalSettleRemaining).
    private float biomechanicalSettleRemaining
    {
        get => ctx.BiomechanicalSettleRemaining;
        set => ctx.BiomechanicalSettleRemaining = value;
    }
    // a6-7
    // EntityId captured at Activate (SimulationContext.TargetEntityId).
    private uint targetEntityId { get => ctx.TargetEntityId; set => ctx.TargetEntityId = value; }

    /// <summary>Registers the host's default humanoid profile and builds its bone tables (<see cref="AllBoneDefaults"/>,
    /// <see cref="DefaultBoneDefs"/>). Once per process (the same profile again is accepted), before the tables are
    /// read. The tables are built first and assigned only when both were built.</summary>
    internal static void UseHumanoidProfile(SkeletonProfile skeletonProfile)
    {
        ArgumentNullException.ThrowIfNull(skeletonProfile);
        if (humanoidProfile != null)
        {
            if (ReferenceEquals(humanoidProfile, skeletonProfile)) return;
            throw new InvalidOperationException("A different humanoid skeleton profile is already registered.");
        }
        var configs = ToBoneConfigs(skeletonProfile.BoneDefinitions);
        var defs = BuildBoneDefsFromConfigs(skeletonProfile, configs, configs);
        allBoneDefaults = configs;
        defaultBoneDefs = defs;
        humanoidProfile = skeletonProfile;
    }

    private static InvalidOperationException NoHumanoidProfile() =>
        new("No humanoid skeleton profile is registered (Ragdoll.UseHumanoidProfile).");

    /// <summary>The host's default humanoid profile (<see cref="UseHumanoidProfile"/>).</summary>
    internal static SkeletonProfile HumanoidSkeleton => humanoidProfile ?? throw NoHumanoidProfile();
    /// <summary>This ragdoll's profile: humanoid rigs are built from it, and its named bones (landmarks, bone classes)
    /// are the ones the ragdoll asks for, whatever rig is active.</summary>
    internal SkeletonProfile Profile => skeletonProfile;
    /// <summary>The profile whose class tuning applies: the generated rig's, or this ragdoll's profile.</summary>
    private SkeletonProfile ActiveClassProfile => activeRagdollIsGeneric ? SkeletonProfile.Auto() : skeletonProfile;

    // Generic (non-humanoid) ragdoll generation tuning lives in SkeletonProfile.Auto().GenericRig
    // (see BuildGenericSkeletonDefs), its velocity clamp and solver floor in its ClassConstants.
    private const int PartyMinSolverIterations = 8;      // floor while party ragdolls add bodies/statics

    /// <summary>
    /// True when this controller drives the LOCAL PLAYER's corpse rather than an NPC's. Set at
    /// Activate, read at InitializePhysics: it decides soft-tissue coverage and the solver budget,
    /// both of which are spent on a body the camera is very likely not framing.
    /// </summary>
    private bool targetIsLocalPlayer;
    private const float TerrainRaycastStartYOffset = 10.0f;
    private const float TerrainRaycastDistance = 80.0f;
    private const int MaxTerrainPatches = 32;
    private const float SafetyGroundDrop = 0.75f;

    // Water under the corpse (Ragdoll.Water.cs).
    private readonly Water water;

    // Diagnostic frame counter
    private int frameCount;

    // Render interpolation: physics advances in fixed 60Hz ticks but we render every frame
    // (often >60fps). Without interpolation the ragdoll shows the same pose for several
    // frames then jumps on each tick — visible micro-judder. We snapshot the bone world
    // pose just before each physics tick (prev) and after (cur, in cachedWorld*), then blend
    // by the leftover-accumulator fraction so the rendered pose advances smoothly every frame.
    private bool hasPrevPhysicsState;
    // Animation freeze state
    private float savedOverallSpeed = 1.0f;

    // Weapon drop physics is WeaponDrop.WeaponDropSimulation (a separate simulation; the plugin's WeaponDropController).

    // Ancestor bone index — the belly-follow bone must follow the pelvis to prevent mesh tearing
    private int nHaraIndex = -1;

    private readonly struct FaceTraversalLandmark
    {
        public readonly int Id;
        public readonly Vector3 HeadLocalPosition;
        public readonly string Label;

        public FaceTraversalLandmark(int id, Vector3 headLocalPosition, string label)
        {
            Id = id;
            HeadLocalPosition = headLocalPosition;
            Label = label;
        }
    }


    /// <summary>True from <see cref="Activate(in RagdollActivation)"/> until <see cref="Deactivate"/> (or until the ragdoll
    /// deactivates itself because its character is gone).</summary>
    public bool IsActive => isActive;
    /// <summary>True once physics has taken the body over from the death animation.</summary>
    public bool PhysicsStarted => physicsStarted;
    /// <summary>True when physics starts at a moment found from the animation or the motion, not after a known delay.</summary>
    internal bool StartsOnItsOwn => trigger is { Automatic: true };
    internal bool IsSimulationReady => isActive && simulation != null;
    internal nint TargetCharacterAddress => targetCharacterAddress;
    internal uint TargetEntityId => targetEntityId;

    /// <summary>
    /// Complete core bone catalog with skeleton parents, default enabled state, and physics
    /// parameters: the humanoid profile's bone definitions (<see cref="UseHumanoidProfile"/>), as
    /// the per-bone configuration records the bone-def builder and the Advanced UI use. The core body
    /// plus each finger's proximal joint are enabled by default; additional skeleton-specific bones
    /// are discovered by the Advanced UI and default disabled. One array (and one set of records) per process.
    /// </summary>
    internal static RagdollBoneConfig[] AllBoneDefaults => allBoneDefaults ?? throw NoHumanoidProfile();
    /// <summary>Default enabled bones with physics parents resolved through the skeleton tree.</summary>
    internal static RagdollBoneDef[] DefaultBoneDefs => defaultBoneDefs ?? throw NoHumanoidProfile();

    /// <summary>
    /// A ragdoll on the given host services, with the guided-collapse profiles it may play. The caller drives it:
    /// <see cref="OnRenderFrame"/> once per rendered frame, <see cref="OnFrameworkTick"/> once per game tick, and the
    /// public methods. Nothing is subscribed here.
    /// </summary>
    internal Ragdoll(RagdollHostServices host, ICollapseProfileSource collapseProfiles, SkeletonProfile profile)
    {
        ArgumentNullException.ThrowIfNull(profile);
        // The default profile keeps the process-wide table (the same array every ragdoll used to share).
        var profileBoneDefaults = ReferenceEquals(profile, humanoidProfile) ? allBoneDefaults! : ToBoneConfigs(profile.BoneDefinitions);
        this.host = host;
        var hostSkeletons = host.Skeletons;
        // The recorder observes these four from outside (Hosting/RecordingDecorators.cs).
        var recordedWorld = new RecordingWorldQuery(host.World, () => rec);
        // Mesh products are recorded inside a frame or a recorded call only (another thread may ask between frames).
        var recordedMeshes = new RecordingMeshSource(host.Meshes, () => recDepth > 0 ? rec : null, () => recMeshCaches);
        var hostActors = host.Actors;
        colliders = host.Colliders;
        var recordedClock = new RecordingClock(host.Clock, () => rec);
        random = new RecordingRandom(host.Random, () => rec);
        scheduler = host.Scheduler;
        settingsSource = host.Settings;
        ctx = new SimulationContext(new CoreCalls(this), profile, profileBoneDefaults, hostSkeletons, recordedWorld,
            recordedMeshes, hostActors, recordedClock, host.Log);
        water = new Water(ctx);
        weaponContact = new WeaponContactSurfaces(ctx);
        hairRig = new HairRig(ctx);
        // a6-2
        meshVolumes = new MeshVolumes(ctx);
        structuralFit = new StructuralFit(ctx);
        surfaceRuntime = new SurfaceRuntime(ctx);
        // a6-3
        ground = new Ground(ctx);
        standing = new StandingSupport(ctx);
        grab = new Grab(ctx);
        impacts = new Impacts(ctx);
        strike = new WeaponStrike(ctx);
        // a6-4
        knee = new KneePowerLoss(ctx);
        collapse = new GuidedCollapse(ctx, random);
        // a6-5
        external = new ExternalBodies(ctx);
        softTissue = new SoftTissue(ctx);
        // a6-6
        characterColliders = new CharacterColliders(ctx);
        corpseTraversal = new CorpseTraversal(ctx);
        // a6-7
        joints = new JointModel(ctx);
        // a6-8
        skeletonReader = new SkeletonReader(ctx);
        poseWriter = new PoseWriter(ctx);
        // a6-9
        ctx.Recorder = recorder = new Recorder(ctx);
        settingsSource.Refresh(settings);
        collapseProfileBook = collapseProfiles;
    }


    // The guided-collapse profiles this ragdoll may play (the directed collapse, Ragdoll.GuidedCollapse.cs).
    private readonly ICollapseProfileSource collapseProfileBook;

    // Knee power-loss tuning is fixed at the settings class defaults (no longer exposed in the UI).
    private static readonly GuidedCollapseKneePowerLossSettings KneePowerLossDefaults = new();

    // ─────────────────────────────────────────────────────────────────────────

    private BodyHandle? FindBodyHandle(string boneName) => ctx.FindBodyHandle(boneName);

    /// <summary>Deactivates and frees everything; a host that subscribed the entry points unsubscribes after this.</summary>
    public virtual void Dispose()
    {
        Deactivate();
        meshVolumes.DisposeCaches();
        hairRig.Remove();
    }
}

// --- BEPU Callbacks ---

partial struct RagdollNarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    public float Friction;
    // The settings the contact callbacks read (worker threads): a copy refreshed between steps.
    public Hosting.HostSettings? Config;

    // Connected body pairs that should NOT collide (parent-child joints).
    // All other body-body pairs DO collide (arms vs torso, etc.).
    public HashSet<(int, int)>? ConnectedPairs;
    // Overlap the ragdoll's own bodies started with; a contact only resists going deeper than that.
    public StartOverlapAllowance? StartOverlap;
    public HashSet<int>? ExternalDynamicBodies;
    public HashSet<int>? ExternalRigDynamicBodies;
    public HashSet<int>? ExternalRigNoRagdollContactBodies;
    public HashSet<(int, int)>? ExternalRigConnectedPairs;
    public HashSet<int>? SoftKinematicBodies;
    public HashSet<int>? NpcCollisionKinematicBodies;
    public HashSet<int>? NpcTraversalGhostKinematicBodies;
    public HashSet<int>? NpcTraversalDynamicBodies;
    public HashSet<int>? RestrictedStatics;
    public HashSet<int>? AllowedDynamicBodiesForRestrictedStatics;
    // Soft-tissue rig bodies (breast / mod jiggle bones): no body contacts ever; static
    // (ground) contacts only when SoftBodyStaticCollision is on.
    public HashSet<int>? SoftBodyBodies;
    public bool SoftBodyStaticCollision;
    // Weapon-only body surfaces: kinematic twins touch only registered weapons, and those
    // weapons skip the ragdoll volumes the twins stand in for.
    public HashSet<int>? WeaponContactBodies;
    public HashSet<int>? WeaponContactProxyBodies;
    public HashSet<int>? WeaponContactProxiedBodies;
    // The extensions registered on the controller that built this simulation (null for other simulations). An
    // object reference, so extensions registered after the simulation exists are seen.
    public RagdollExtensionSet? Extensions;

    public void Initialize(BepuSimulation simulation) { }

    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        if (WeaponContactProxyBodies is { Count: > 0 } proxies)
        {
            var aProxy = a.Mobility == CollidableMobility.Kinematic && proxies.Contains(a.BodyHandle.Value);
            var bProxy = b.Mobility == CollidableMobility.Kinematic && proxies.Contains(b.BodyHandle.Value);
            if (aProxy || bProxy)
            {
                var other = aProxy ? b : a;
                return other.Mobility == CollidableMobility.Dynamic && WeaponContactBodies!.Contains(other.BodyHandle.Value);
            }
            if (a.Mobility == CollidableMobility.Dynamic && b.Mobility == CollidableMobility.Dynamic &&
                (WeaponContactBodies!.Contains(a.BodyHandle.Value) && WeaponContactProxiedBodies!.Contains(b.BodyHandle.Value) ||
                 WeaponContactBodies.Contains(b.BodyHandle.Value) && WeaponContactProxiedBodies!.Contains(a.BodyHandle.Value)))
                return false;
        }

        // Soft-tissue bodies: no contacts with other BODIES ever — their capsules sit inside
        // the torso volume by design, so body contact is permanent interpenetration fighting
        // the jiggle springs; the joints' SwingLimit is the excursion backstop instead.
        // Ground (static) contact is an opt-in: flesh pressing against the floor reacts,
        // at the cost of extra contact pairs. Must run before the static branch below.
        if (SoftBodyBodies != null)
        {
            var aSoft = a.Mobility != CollidableMobility.Static && SoftBodyBodies.Contains(a.BodyHandle.Value);
            var bSoft = b.Mobility != CollidableMobility.Static && SoftBodyBodies.Contains(b.BodyHandle.Value);
            if (aSoft || bSoft)
            {
                if (!SoftBodyStaticCollision)
                    return false;
                if (a.Mobility != CollidableMobility.Static && b.Mobility != CollidableMobility.Static)
                    return false;
                // soft-vs-static: fall through to the static branch.
            }
        }

        // Always allow body-static collisions (ragdoll vs ground)
        if (a.Mobility == CollidableMobility.Static || b.Mobility == CollidableMobility.Static)
        {
            var dynamicRef = a.Mobility == CollidableMobility.Dynamic ? a :
                b.Mobility == CollidableMobility.Dynamic ? b : default;
            if (dynamicRef.Mobility == CollidableMobility.Dynamic && IsExternalDynamic(dynamicRef.BodyHandle.Value))
                speculativeMargin = MathF.Min(speculativeMargin, 0.016f);

            if (RestrictedStatics != null && AllowedDynamicBodiesForRestrictedStatics != null &&
                ((a.Mobility == CollidableMobility.Static && RestrictedStatics.Contains(a.StaticHandle.Value)) ||
                 (b.Mobility == CollidableMobility.Static && RestrictedStatics.Contains(b.StaticHandle.Value))))
            {
                return dynamicRef.Mobility == CollidableMobility.Dynamic &&
                       AllowedDynamicBodiesForRestrictedStatics.Contains(dynamicRef.BodyHandle.Value);
            }
            return true;
        }

        if ((a.Mobility == CollidableMobility.Dynamic && b.Mobility == CollidableMobility.Kinematic) ||
            (b.Mobility == CollidableMobility.Dynamic && a.Mobility == CollidableMobility.Kinematic))
        {
            var dynamicBody = a.Mobility == CollidableMobility.Dynamic ? a : b;
            // The locomotion carrier represents the follower's support/weight. Letting it also
            // collide with animated NPC kinematics would duplicate the character volume and
            // introduce another infinite-mass feedback path.
            if (IsNpcTraversalDynamic(dynamicBody.BodyHandle.Value))
                return false;
            var kinematic = a.Mobility == CollidableMobility.Kinematic ? a : b;
            if (NpcTraversalGhostKinematicBodies?.Contains(kinematic.BodyHandle.Value) == true)
                return false;

            // Hair strands (and their kinematic follow-anchor) are flagged no-ragdoll-contact: they are
            // driven purely by joints, so they must not collide with any kinematic body — neither their
            // own anchor (which sits on the scalp) nor NPC collision proxies. Without this the anchor
            // punts the strands and they whip the corpse.
            if (IsExternalRigNoRagdollContact(a.BodyHandle.Value) || IsExternalRigNoRagdollContact(b.BodyHandle.Value))
                return false;
            return true;
        }

        // Body-body: allow UNLESS they are directly connected by a joint.
        // Connected pairs would explode because they share a constraint anchor point.
        if (a.Mobility == CollidableMobility.Dynamic && b.Mobility == CollidableMobility.Dynamic)
        {
            var aId = a.BodyHandle.Value;
            var bId = b.BodyHandle.Value;
            var aTraversal = IsNpcTraversalDynamic(aId);
            var bTraversal = IsNpcTraversalDynamic(bId);
            if (aTraversal || bTraversal)
            {
                // Followers do not form a dynamic pile against each other. Each carrier only
                // exchanges finite impulses with the corpse; formation spacing handles peers.
                if (aTraversal && bTraversal)
                    return false;
                var otherId = aTraversal ? bId : aId;
                if (IsExternalDynamic(otherId) || IsExternalRigDynamic(otherId))
                    return false;
                return true;
            }
            var aExternal = IsExternalDynamic(aId);
            var bExternal = IsExternalDynamic(bId);
            var aRig = IsExternalRigDynamic(aId);
            var bRig = IsExternalRigDynamic(bId);
            if (aRig || bRig)
            {
                var rigLo = Math.Min(aId, bId);
                var rigHi = Math.Max(aId, bId);
                if (ExternalRigConnectedPairs != null && ExternalRigConnectedPairs.Contains((rigLo, rigHi)))
                    return false;

                // The garment rig should collide with the corpse ragdoll, but not with legacy
                // single-piece dropped gear. Single gear remains ground-only by design below.
                if ((aRig && bExternal && !bRig) || (bRig && aExternal && !aRig))
                    return false;

                // Body garments spawn around the torso, where the corpse ragdoll capsules overlap the
                // visual clothing volume. Let those rigs keep ground/static contacts but skip corpse
                // dynamic contacts so handoff does not shove the shirt backward in one frame.
                var aNoRagdoll = IsExternalRigNoRagdollContact(aId);
                var bNoRagdoll = IsExternalRigNoRagdollContact(bId);
                if ((aNoRagdoll && !bExternal && !bRig) || (bNoRagdoll && !aExternal && !aRig))
                    return false;

                return true;
            }

            if (aExternal || bExternal)
            {
                // Dropped gear falls independently straight to the ground: it must NOT rest on the
                // corpse's invisible limb capsules or pile on other dropped pieces. That dynamic-vs-
                // dynamic stacking is what left garments hanging in the air (and only dropping when an
                // NPC kick knocked the stack loose). Ground (static) and NPC strikes (kinematic) are
                // handled above, so gear still lands and can still be struck.
                return false;
            }

            if (ConnectedPairs == null)
                return false;

            var lo = Math.Min(aId, bId);
            var hi = Math.Max(aId, bId);
            if (ConnectedPairs.Contains((lo, hi)))
                return false;
            return true;
        }

        return false;
    }

    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        => true;

    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold,
        out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        pairMaterial = default;
        var handled = false;
        if (Extensions is { HasContact: true } extensions) extensions.ConfigureContact(pair, ref manifold, ref pairMaterial, ref handled);
        if (handled) return true;
        pairMaterial.FrictionCoefficient = Friction;

        // Self-collision (limb vs own limb) is dynamic-vs-dynamic. At rest the corpse's
        // capsules overlap (thighs touching, forearms on the torso, hands on legs); a crisp
        // 2 m/s recovery shoves those overlaps apart every step, the joints pull them back,
        // and the velocity never decays below the sleep threshold — so the body writhes
        // forever and never settles. Give self-contacts a gentle, overdamped recovery so
        // resting overlaps stop pumping energy and the rig can sleep. Ground (static) and
        // external strikes (kinematic) keep the firm 2 m/s recovery for crisp response.
        if (UseAdvancedGearContactFriction() && IsGearGroundContact(pair))
        {
            pairMaterial.MaximumRecoveryVelocity = 0.25f;
            pairMaterial.FrictionCoefficient = MathF.Max(Friction, 3.5f);
            pairMaterial.SpringSettings = new SpringSettings(12, 3);
        }
        else if (UseAdvancedGearContactFriction() && IsGearContact(pair))
        {
            pairMaterial.MaximumRecoveryVelocity = 0.08f;
            pairMaterial.FrictionCoefficient = Math.Clamp(Friction, 0.45f, 0.9f);
            pairMaterial.SpringSettings = new SpringSettings(6, 4);
        }
        else if (IsNpcCollisionKinematicContact(pair))
        {
            // A walking NPC is an infinite-mass kinematic body. The default crisp material turns
            // even a small overlap into a hard horizontal recovery impulse and launches the corpse.
            // Traversal lifts the actor root onto the corpse, so this contact only needs to carry
            // and softly deform the body: low friction avoids dragging, while slow overdamped
            // recovery preserves a visible step/weight response without energetic bounce.
            pairMaterial.MaximumRecoveryVelocity = 0.12f;
            pairMaterial.FrictionCoefficient = MathF.Min(Friction, 0.28f);
            pairMaterial.SpringSettings = new SpringSettings(8, 3);
        }
        else if (IsNpcTraversalDynamicContact(pair))
        {
            // A finite-mass carrier can now produce real normal force, friction and torque on the
            // exact body it stands on. Keep penetration recovery overdamped so a newly engaged
            // carrier climbs instead of bouncing, but retain enough friction for credible footing.
            pairMaterial.MaximumRecoveryVelocity = 0.18f;
            pairMaterial.FrictionCoefficient = Math.Clamp(Friction, 0.55f, 0.85f);
            pairMaterial.SpringSettings = new SpringSettings(12, 3);
        }
        else if (pair.A.Mobility == CollidableMobility.Dynamic && pair.B.Mobility == CollidableMobility.Dynamic)
        {
            pairMaterial.MaximumRecoveryVelocity = 0.2f;
            pairMaterial.SpringSettings = new SpringSettings(20, 2);
            StartOverlap?.Apply(pair.A.BodyHandle.Value, pair.B.BodyHandle.Value, ref manifold);
        }
        else if (IsSoftKinematicContact(pair))
        {
            pairMaterial.MaximumRecoveryVelocity = 0.25f;
            pairMaterial.SpringSettings = new SpringSettings(12, 3);
        }
        else
        {
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
        }
        return true;
    }

    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB,
        ref ConvexContactManifold manifold) => true;

    public void Dispose() { }

    private bool IsExternalDynamic(int bodyHandle)
        => ExternalDynamicBodies != null && ExternalDynamicBodies.Contains(bodyHandle);

    private bool IsNpcTraversalDynamic(int bodyHandle)
        => NpcTraversalDynamicBodies != null && NpcTraversalDynamicBodies.Contains(bodyHandle);

    private bool IsNpcTraversalDynamicContact(CollidablePair pair)
        => (pair.A.Mobility == CollidableMobility.Dynamic &&
            IsNpcTraversalDynamic(pair.A.BodyHandle.Value)) ||
           (pair.B.Mobility == CollidableMobility.Dynamic &&
            IsNpcTraversalDynamic(pair.B.BodyHandle.Value));

    private bool IsExternalRigDynamic(int bodyHandle)
        => ExternalRigDynamicBodies != null && ExternalRigDynamicBodies.Contains(bodyHandle);

    private bool IsExternalRigNoRagdollContact(int bodyHandle)
        => ExternalRigNoRagdollContactBodies != null && ExternalRigNoRagdollContactBodies.Contains(bodyHandle);

    private bool IsSoftKinematicContact(CollidablePair pair)
        => SoftKinematicBodies != null &&
           ((pair.A.Mobility == CollidableMobility.Kinematic && SoftKinematicBodies.Contains(pair.A.BodyHandle.Value)) ||
            (pair.B.Mobility == CollidableMobility.Kinematic && SoftKinematicBodies.Contains(pair.B.BodyHandle.Value)));

    private bool IsNpcCollisionKinematicContact(CollidablePair pair)
        => NpcCollisionKinematicBodies != null &&
           ((pair.A.Mobility == CollidableMobility.Kinematic && NpcCollisionKinematicBodies.Contains(pair.A.BodyHandle.Value)) ||
            (pair.B.Mobility == CollidableMobility.Kinematic && NpcCollisionKinematicBodies.Contains(pair.B.BodyHandle.Value)));

    private bool UseAdvancedGearContactFriction()
        => Config?.KoStripPhysicsDropClothing == true;

    // "Gear" for contact-material purposes = single dropped pieces (ExternalDynamic) AND articulated
    // garment-rig bodies (ExternalRigDynamic). In the player-ragdoll host sim a rig body is in BOTH sets,
    // so folding the rig set in changes nothing there; it only matters in the DismembermentController's
    // LOCAL fallback sim, where rig bodies are ExternalRigDynamic-only. Without this, local-sim garments
    // fell through to the firm 2 m/s / (30,1) default material and visibly popped apart on the first step.
    private bool IsGearDynamic(int bodyHandle)
        => IsExternalDynamic(bodyHandle) || IsExternalRigDynamic(bodyHandle);

    private bool IsGearContact(CollidablePair pair)
    {
        var aGear = pair.A.Mobility == CollidableMobility.Dynamic && IsGearDynamic(pair.A.BodyHandle.Value);
        var bGear = pair.B.Mobility == CollidableMobility.Dynamic && IsGearDynamic(pair.B.BodyHandle.Value);
        return aGear || bGear;
    }

    private bool IsGearGroundContact(CollidablePair pair)
    {
        var aGear = pair.A.Mobility == CollidableMobility.Dynamic && IsGearDynamic(pair.A.BodyHandle.Value);
        var bGear = pair.B.Mobility == CollidableMobility.Dynamic && IsGearDynamic(pair.B.BodyHandle.Value);
        return (aGear && pair.B.Mobility == CollidableMobility.Static) ||
               (bGear && pair.A.Mobility == CollidableMobility.Static);
    }
}

/// <summary>
/// Gravity and damping.
///
/// The damping here is a numerical crutch — it exists to bleed energy out of a solver that might
/// otherwise diverge — and applying it flat turned out to be the single biggest reason the ragdoll reads
/// weightless. At the configured 0.97 per 60Hz frame it takes 84% of a body's speed away every second,
/// which pins terminal velocity at
///
///     v = g·dt·d / (1 − d)  ≈  9.8/60 × 0.97/0.03  ≈  5.3 m/s
///
/// A person who falls 1.4 m is already past that. So nothing in this rig has ever been able to fall
/// faster than a brisk walk — every corpse has been sinking through syrup, and the whip a limb should
/// have on impact was drained out of it before it landed. It also explains why raising mass never
/// helped: mass cancels out of free fall entirely (Galileo), and out of the joint forces too, so
/// scaling the whole rig changes nothing at all about how it moves.
///
/// The fix is to charge the crutch only where it is needed. A body that is settling keeps the full
/// damping — every bit of the settle and collapse tuning behaves exactly as before, because below the
/// settle threshold this is bit-for-bit the old code — while a body that is falling keeps almost none
/// of it and accelerates the way it should. A little always survives, so a solver that does start to
/// diverge still bleeds energy rather than parking on the velocity clamp.
/// </summary>
struct RagdollPoseIntegratorCallbacks : IPoseIntegratorCallbacks
{
    /// <summary>At or below this speed, the configured damping applies in full (settle unchanged).</summary>
    private const float SettleSpeed = 3f;        // m/s
    /// <summary>At or above this speed, the body is falling and is left alone.</summary>
    private const float FreeFallSpeed = 6f;      // m/s
    private const float SettleAngularSpeed = 4f; // rad/s
    private const float FreeAngularSpeed = 9f;   // rad/s
    /// <summary>What survives at speed. Not 1: terminal works out around 32 m/s, well past the velocity
    /// clamp, so this is free fall in practice — but a diverging solver still has something pulling it
    /// back down instead of sitting at the ceiling.</summary>
    private const float FreeDamping = 0.995f;

    private Vector3 gravity;
    private float linearDamping;
    private Vector3Wide gravityDt;
    private Vector<float> settleDamping;
    private Vector<float> freeDamping;

    private Vector<float> settleSpeed;
    private Vector<float> freeSpeed;
    private Vector<float> settleAngular;
    private Vector<float> freeAngular;

    public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
    public readonly bool AllowSubstepsForUnconstrainedBodies => false;
    public readonly bool IntegrateVelocityForKinematics => false;

    public RagdollPoseIntegratorCallbacks(Vector3 gravity, float linearDamping)
    {
        this.gravity = gravity;
        this.linearDamping = linearDamping;
        this.gravityDt = default;
        this.settleDamping = default;
        this.freeDamping = default;
        this.settleSpeed = default;
        this.freeSpeed = default;
        this.settleAngular = default;
        this.freeAngular = default;
    }

    public void Initialize(BepuSimulation simulation) { }

    public void PrepareForIntegration(float dt)
    {
        gravityDt.X = new Vector<float>(gravity.X * dt);
        gravityDt.Y = new Vector<float>(gravity.Y * dt);
        gravityDt.Z = new Vector<float>(gravity.Z * dt);

        settleDamping = new Vector<float>(MathF.Pow(linearDamping, dt * 60f));
        freeDamping = new Vector<float>(MathF.Pow(FreeDamping, dt * 60f));

        settleSpeed = new Vector<float>(SettleSpeed);
        freeSpeed = new Vector<float>(FreeFallSpeed);
        settleAngular = new Vector<float>(SettleAngularSpeed);
        freeAngular = new Vector<float>(FreeAngularSpeed);
    }

    public void IntegrateVelocity(
        Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation,
        BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex,
        Vector<float> dt, ref BodyVelocityWide velocity)
    {
        velocity.Linear.X += gravityDt.X;
        velocity.Linear.Y += gravityDt.Y;
        velocity.Linear.Z += gravityDt.Z;

        Vector3Wide.Length(velocity.Linear, out var linearSpeed);
        var linear = Blend(linearSpeed, settleSpeed, freeSpeed, settleDamping, freeDamping);
        velocity.Linear.X *= linear;
        velocity.Linear.Y *= linear;
        velocity.Linear.Z *= linear;

        // Gated on its OWN speed: a limb whipping round a body that is barely translating is still
        // moving fast, and draining it is exactly what robs an impact of its snap.
        Vector3Wide.Length(velocity.Angular, out var angularSpeed);
        var angular = Blend(angularSpeed, settleAngular, freeAngular, settleDamping, freeDamping);
        velocity.Angular.X *= angular;
        velocity.Angular.Y *= angular;
        velocity.Angular.Z *= angular;
    }

    /// <summary>Smoothstep from the settle damping to the free damping across the speed band.</summary>
    private static Vector<float> Blend(
        Vector<float> speed, Vector<float> low, Vector<float> high,
        Vector<float> atLow, Vector<float> atHigh)
    {
        var t = (speed - low) / (high - low);
        t = Vector.Min(Vector<float>.One, Vector.Max(Vector<float>.Zero, t));
        t = t * t * (new Vector<float>(3f) - new Vector<float>(2f) * t);
        return atLow + (atHigh - atLow) * t;
    }
}

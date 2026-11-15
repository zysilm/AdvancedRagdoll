// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
// Synthetic host of the dropped-weapon simulation: skeletons, ground, characters, weapons, settings, clock and models,
// all made up and fully deterministic. Every answer is a pure function of what the scenario set.
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using AdvancedRagdoll;
using AdvancedRagdoll.Hair;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.MeshVolume;
using AdvancedRagdoll.Recording;
using BepuPhysics.Collidables;

namespace AdvancedRagdoll.Tests;

/// <summary>A skeleton with a body part only (no partial skeletons).</summary>
internal sealed class SyntheticSkeleton : IHostSkeleton
{
    public SyntheticSkeleton(nint actor, string[] names, short[] parents, Vector3[] modelPositions)
    {
        Actor = actor;
        Names = names;
        Parents = parents;
        Pose = new BoneQs[modelPositions.Length];
        for (var i = 0; i < Pose.Length; i++) SetBone(i, modelPositions[i]);
    }

    public readonly string[] Names;
    public readonly short[] Parents;
    public readonly BoneQs[] Pose;
    public RootTransform RootTransform = new(Vector3.Zero, Quaternion.Identity, Vector3.One);
    public bool Rooted = true;

    public void SetBone(int i, Vector3 position) => SetBone(i, position, Quaternion.Identity);

    public void SetBone(int i, Vector3 position, Quaternion rotation) => Pose[i] = new BoneQs
    {
        Translation = new Vector4(position, 0f),
        Rotation = rotation,
        Scale = new Vector4(1f, 1f, 1f, 0f),
    };

    public int IndexOf(string name) => Array.IndexOf(Names, name);

    public nint Actor { get; }
    public nint DrawIdentity => Actor + 1;
    public int BoneCount => Pose.Length;
    public int ParentCount => Parents.Length;
    public Span<BoneQs> ModelPose => Pose;
    public short Parent(int bone) => Parents[bone];
    public int BoneNameCount => Names.Length;
    public string? BoneName(int bone) => Names[bone];
    public ReadOnlySpan<BoneQs> ReferencePose => ReadOnlySpan<BoneQs>.Empty;
    public bool HasRoot => Rooted;
    public RootTransform Root => RootTransform;
    public bool TryGetDominantAnimation(out RagdollAnimationSample sample) { sample = default; return false; }
    public int PartCount => 1;
    public bool PartAvailable(int part) => part == 0;
    public bool PartHasPose(int part) => part == 0;
    public bool PartInSync(int part) => true;
    public nint PartIdentity(int part) => part == 0 ? Actor + 2 : 0;
    public nint PartPoseIdentity(int part) => part == 0 ? Actor + 3 : 0;
    public Span<BoneQs> PartModelPose(int part) => part == 0 ? Pose : Span<BoneQs>.Empty;
    public int PartModelPoseLength(int part) => part == 0 ? Pose.Length : 0;
    public int PartBoneNameCount(int part) => part == 0 ? Names.Length : 0;
    public string? PartBoneName(int part, int bone) => part == 0 ? Names[bone] : null;
    public int PartParentCount(int part) => part == 0 ? Parents.Length : 0;
    public short PartParent(int part, int bone) => part == 0 ? Parents[bone] : (short)-1;
    public ReadOnlySpan<BoneQs> PartReferencePose(int part) => ReadOnlySpan<BoneQs>.Empty;
    public int PartReferenceCount(int part) => 0;
    public int PartConnectedBone(int part) => -1;
    public int PartConnectedParentBone(int part) => -1;
    public void ResyncPartModelPose(int part) { }
}

internal sealed class SyntheticSkeletons : IHostSkeletonSource
{
    public readonly Dictionary<nint, SyntheticSkeleton> ByActor = new();
    public readonly HashSet<nint> Vanished = new();

    public IHostSkeleton? TryGetSkeleton(nint actor, PosePurpose site = PosePurpose.Other) =>
        !Vanished.Contains(actor) && ByActor.TryGetValue(actor, out var s) ? s : null;

    public Vector3? GetBoneWorldPosition(nint actor, string boneName) => null;
}

/// <summary>Ground as a height function of (x, z); null = no ground there (rays miss).</summary>
internal sealed class SyntheticWorld(Func<float, float, float?> height) : IHostWorldQuery
{
    public int Rays;

    public bool Raycast(RaycastSite site, Vector3 origin, Vector3 direction, float maxDistance, out RayHit hit)
    {
        Rays++;
        hit = default;
        if (direction != new Vector3(0, -1, 0)) return false;
        var h = height(origin.X, origin.Z);
        if (h is not float y || origin.Y < y || origin.Y - y > maxDistance) return false;
        hit.Point = new Vector3(origin.X, y, origin.Z);
        hit.Normal = Vector3.UnitY;
        hit.Distance = origin.Y - y;
        return true;
    }

    public bool TryCollectGroundTriangles(Vector3 min, Vector3 max, List<Vector3> triangles, int maxTriangles) => false;

    public bool TryFindWaterSurface(Vector3 position, float searchAbove, float searchBelow, out float surfaceY)
    {
        surfaceY = 0;
        return false;
    }
}

internal sealed class SyntheticActors : IHostActors
{
    public nint Local;
    public readonly Dictionary<nint, Vector3> DrawScale = new();

    public bool GameRunning => true;
    public nint LocalPlayer => Local;
    public bool IsAlive(nint actor, uint entityId) => true;
    public Vector3 GetPosition(nint actor, ActorSite site) => Vector3.Zero;
    public float GetFacing(nint actor, ActorSite site) => 0f;
    public uint GetEntityId(nint actor) => (uint)actor;
    public ulong GetGameObjectId(nint actor) => (ulong)actor;
    public int GetObjectIndex(nint actor) => (int)(actor & 0xFF);
    public bool TryGetDrawScale(nint actor, out Vector3 scale) => DrawScale.TryGetValue(actor, out scale);
    public void ReadAppearance(nint actor, Span<byte> customize, out int modelCharaId) { customize.Clear(); modelCharaId = 0; }
    public void ReadModelType(nint actor, IHostSkeleton? skeleton, out int modelType, out int raceSexId) { modelType = -1; raceSexId = -1; }
    public float GetAnimationSpeed(nint actor) => 1f;
    public void SetAnimationSpeed(nint actor, float speed) { }
    public void SetDrawPosition(nint actor, Vector3 position) { }
    public bool SetApproachPosition(nint actor, Vector3 position) => false;
}

internal sealed class NoColliders : IHostColliders
{
    public bool HasSelector => false;
    public bool HasExtraColliders => false;
    public bool HasMovementBlock => false;
    public void GetSelectedNpcs(List<ColliderCandidate> destination) { }
    public IReadOnlyList<nint> GetExtraColliders() => Array.Empty<nint>();
    public bool TryGetWeaponTransform(nint actor, out Vector3 position, out Quaternion rotation)
    {
        position = default;
        rotation = Quaternion.Identity;
        return false;
    }
}

internal sealed class SyntheticWeapons(Output output) : IWeaponDrawSource
{
    public readonly Dictionary<(nint Actor, DrawnWeaponSlot Slot), nint> Slots = new();
    public readonly Dictionary<nint, (Vector3 Position, Quaternion Rotation)> Roots = new();
    public readonly Dictionary<nint, SyntheticSkeleton> Skeletons = new();
    public readonly Dictionary<nint, Vector3> Scales = new();

    public nint GetWeapon(nint actor, DrawnWeaponSlot slot) => Slots.TryGetValue((actor, slot), out var w) ? w : 0;

    public bool TryGetRoot(nint weapon, out Vector3 position, out Quaternion rotation)
    {
        if (Roots.TryGetValue(weapon, out var root)) { (position, rotation) = root; return true; }
        position = default;
        rotation = default;
        return false;
    }

    public IHostSkeleton? TryGetSkeleton(nint weapon) => Skeletons.TryGetValue(weapon, out var s) ? s : null;

    public Vector3 GetDrawScale(nint weapon) => Scales.TryGetValue(weapon, out var s) ? s : Vector3.One;

    public void SetTransform(nint weapon, Vector3 position, Quaternion rotation)
    {
        Roots[weapon] = (position, rotation);
        output.Line($"T 0x{weapon:X} p={Output.V(position)} q={Output.Q(rotation)}");
    }
}

internal sealed class SyntheticSettings : IHostSettingsSource
{
    public bool FineDetail, WeaponMesh, BodyMesh;

    public bool Refresh(HostSettings settings)
    {
        settings.NpcRagdollFineDetail = FineDetail;
        settings.WeaponDropMeshCollision = WeaponMesh;
        settings.WeaponBodyMeshContacts = BodyMesh;
        return false;
    }
}

/// <summary>A clock the scenario advances by hand (one tick = one microsecond).</summary>
internal sealed class ManualClock : IRagdollClock
{
    public long Now = 1_000_000;
    public long Frequency => 1_000_000;
    public long GetTimestamp(ClockSite site) => Now;
    public TimeSpan GetElapsedTime(ClockSite site, long since) => TimeSpan.FromTicks((Now - since) * 10);
    public TimeSpan GetElapsedTime(ClockSite site, long since, long until) => TimeSpan.FromTicks((until - since) * 10);
    public long GetTickCount64(ClockSite site) => Now / 1000;
}

internal sealed class OutputLog(Output output, string prefix) : IRagdollLog
{
    public void Info(string message) => output.Line($"{prefix}I {message}");
    public void Warning(string message) => output.Line($"{prefix}W {message}");
    public void Warning(Exception exception, string message) => output.Line($"{prefix}W {message} [{exception.GetType().Name}: {exception.Message}]");
    public void Error(string message) => output.Line($"{prefix}E {message}");
    public void Error(Exception exception, string message) => output.Line($"{prefix}E {message} [{exception.GetType().Name}: {exception.Message}]");
}

/// <summary>Models: a box per listed bone as the frozen body surface (bone-local), and fitted hulls for the weapons
/// that have triangles. Reduced surfaces are never built (no background work).</summary>
internal sealed class SyntheticMeshes : IMeshSource
{
    public readonly Dictionary<nint, List<(string Bone, Vector3 Center, Vector3 Half)>> BodySurfaces = new();
    public readonly Dictionary<nint, List<Triangle>> WeaponTriangles = new();

    public nint ModelIdentity(nint actor) => actor + 1;

    public FrozenMeshGeometryCache? ProvideBodySurface(nint actor, BodySurfaceUse use, Func<FrozenMeshGeometryCache?> capture)
    {
        if (!BodySurfaces.TryGetValue(actor, out var regions)) return null;
        var cache = new FrozenMeshGeometryCache();
        foreach (var (bone, center, half) in regions) cache.AddLocal(bone, Box(center, half));
        return cache;
    }

    public RenderedGeometrySnapshot CaptureRenderedGeometry(nint actor, IHostSkeleton skeleton) => new();
    public void ConfigureLod(FrozenMeshGeometryCache cache, bool enabled, float tolerance) { }
    public bool PollLod(FrozenMeshGeometryCache cache, out Exception? error) { error = null; return false; }

    public bool TryCreateLod(FrozenMeshGeometryCache cache, CachedLodUse use, string bone, BodyMeshGeometry local,
        out MeshCollisionLodResult lod) { lod = null!; return false; }

    public bool TryGetCollisionGeometry(FrozenMeshGeometryCache cache, string bone, out BodyMeshGeometry local)
        => cache.TryGetSurfaceGeometry(bone, out local);

    public Task<Ragdoll.StructuralFitResult>? StartStructuralFit(nint actor, IHostSkeleton skeleton) => null;

    public Dictionary<int, Ragdoll.SoftTissueMeshFit> SoftTissueFits(nint actor, IHostSkeleton skeleton,
        HashSet<int> candidates, bool verboseLog) => new();

    public bool TryCollectActorTriangles(nint actor, IHostSkeleton skeleton, string label, bool verboseLog,
        List<Triangle> triangles, out int loadedModels, out int processedMeshes)
    {
        loadedModels = processedMeshes = 0;
        return false;
    }

    public void PollHairMesh() { }
    public HairMeshBinding? HairMesh => null;
    public bool TakeHairMeshRebuild() => false;

    public WeaponMeshGeometry? BuildWeaponMesh(nint weaponModel, bool verboseLog) =>
        WeaponTriangles.TryGetValue(weaponModel, out var triangles) ? WeaponMeshGeometry.Fit(triangles) : null;

    public static BodyMeshGeometry Box(Vector3 center, Vector3 half)
    {
        var v = new Vector3[8];
        for (var i = 0; i < 8; i++)
            v[i] = center + new Vector3((i & 1) != 0 ? half.X : -half.X, (i & 2) != 0 ? half.Y : -half.Y, (i & 4) != 0 ? half.Z : -half.Z);
        int[] indices =
        [
            0, 2, 1, 1, 2, 3, // -z
            4, 5, 6, 5, 7, 6, // +z
            0, 1, 4, 1, 5, 4, // -y
            2, 6, 3, 3, 6, 7, // +y
            0, 4, 2, 2, 4, 6, // -x
            1, 3, 5, 3, 7, 5, // +x
        ];
        return new BodyMeshGeometry(v, indices);
    }

    public static void AddBoxTriangles(List<Triangle> triangles, Vector3 center, Vector3 half)
    {
        var box = Box(center, half);
        for (var i = 0; i < box.Indices.Length; i += 3)
            triangles.Add(new Triangle(box.Vertices[box.Indices[i]], box.Vertices[box.Indices[i + 1]], box.Vertices[box.Indices[i + 2]]));
    }
}

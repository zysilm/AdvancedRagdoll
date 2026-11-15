// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Collections.Generic;
using System.Numerics;
using AdvancedRagdoll.MeshVolume;

namespace AdvancedRagdoll;

// Entry points for the offline replay (Tests/RagdollReplay, stages A2a and A2b). A recording starts in the middle of
// a controller's life: what earlier activations left behind is in the Activate record and, since format version 3,
// in the MeshProduct "carried" record after it (the body surfaces); a version 2 recording does not list
// followWasActive, which its first FrameBegin record shows. Whether the local player's live hair rig was running is
// shown by its first live hair frame (a rig build always writes a HairMesh record). The replay puts it back before
// it calls Activate.
// Nothing here runs in the game.
public unsafe partial class Ragdoll
{
    /// <summary>One request of the walking-support proxy, as the Activate record lists it.</summary>
    internal readonly record struct ReplayTraversalRequest(
        nint Actor, bool Enabled, Vector3 InitialRoot, Vector3 TargetRoot, float FloorY, float VisualScale,
        float MoveSpeed, float MaxClimb, float StompResponse, int ConcurrentStompers, bool Locomoting);

    /// <summary>State a controller carries from one activation to the next (Activate record, format version 2).</summary>
    internal sealed class ReplayCarriedState
    {
        public bool PendingDirectedCollapse;
        public string PendingProfile = string.Empty;
        public float PendingDuration, PendingFootForce, PendingPelvisForce, PendingDrop, PendingForward, PendingPitchDegrees;
        public bool PendingEntryConditionedKneePowerLoss;
        public bool PendingKneePowerLoss;
        public bool UseLocalSeveredBones;
        public List<string> LocalSeveredBones = new();
        public List<nint> LightweightColliders = new();
        public uint ImpactRandomState;
        public int NextGrabSlotId;
        public int ExternalBodyGeneration;
        public List<int> StruckThisWindow = new();
        public List<nint> StrikeColliders = new();
        public List<ReplayTraversalRequest> TraversalRequests = new();
        /// <summary>Whether the previous activation's last frame wrote a follow position (public builds never reset it).</summary>
        public bool FollowWasActive;
        /// <summary>The body surfaces carried over from earlier activations (format version 3, MeshProduct "carried").</summary>
        public bool HasBodySurfaces;
        public nint FrozenMeshActor;
        public uint FrozenMeshEntity;
        public bool ObservedMeshEnabled;
        public int ObservedMeshCacheRevision = -1;
        public bool CachedMeshLodEnabled;
        public float CachedMeshLodTolerance;
        public long NextMeshCacheAttempt;
        public FrozenMeshGeometryCache? FrozenMeshCache;
        public List<(nint Actor, uint Entity, nint Draw, FrozenMeshGeometryCache? Cache)> WeaponContactMeshes = new();
        /// <summary>Whether the local player's live hair rig was already running, and current, when the activation
        /// started (the main controller runs it all the time the player is alive). The game did not rebuild it on the
        /// first live hair frame of the recording; the identities are what the replay host reports for that frame.</summary>
        public bool LiveHairRunning;
        public nint LiveHairDraw;
        public ulong LiveHairObject;
    }

    internal void RestoreCarriedState(ReplayCarriedState s)
    {
        // Each component puts back its own fields, in the order the writes always had.
        collapse.RestoreCarried(s);
        knee.RestoreCarried(s.PendingKneePowerLoss);
        skeletonReader.RestoreCarried(s.UseLocalSeveredBones, s.LocalSeveredBones);
        characterColliders.RestoreCarried(s.LightweightColliders);
        impacts.RestoreCarried(s.ImpactRandomState);
        grab.RestoreCarried(s.NextGrabSlotId);
        external.RestoreCarried(s.ExternalBodyGeneration);
        strike.RestoreCarried(s.StruckThisWindow, s.StrikeColliders);
        corpseTraversal.RestoreCarried(s.TraversalRequests);
        followWasActive = s.FollowWasActive;
        // Only what decides whether the first live hair frame rebuilds the rig (and so reads the hair model); the
        // solver state of the game's live rig is not recorded, and its poses before the ragdoll are not outputs.
        hairRig.RestoreLiveHairState(s.LiveHairRunning, s.LiveHairDraw, s.LiveHairObject);
        if (s.HasBodySurfaces)
            meshVolumes.RestoreCarried(s);
    }

    /// <summary>One render frame, as the render hook runs it.</summary>
    internal void ReplayRenderFrame() => OnRenderFrame();

    /// <summary>True while the controller is armed or simulating.</summary>
    internal bool ReplayActive => isActive;
}

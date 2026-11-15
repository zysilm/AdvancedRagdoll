// This Source Code Form is subject to the Mozilla Public License, v. 2.0.
using System;
using System.Collections.Generic;
using System.Numerics;
using AdvancedRagdoll.MeshVolume;
using AdvancedRagdoll.Hosting;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // The cached body surfaces this controller carries over activations (the component's fields; the recording and
    // the replay read and restore them).
    private FrozenMeshGeometryCache? frozenMeshCache { get => meshVolumes.frozenMeshCache; set => meshVolumes.frozenMeshCache = value; }
    private nint frozenMeshActor { get => meshVolumes.frozenMeshActor; set => meshVolumes.frozenMeshActor = value; }
    private uint frozenMeshEntity { get => meshVolumes.frozenMeshEntity; set => meshVolumes.frozenMeshEntity = value; }
    private int observedMeshCacheRevision { get => meshVolumes.observedMeshCacheRevision; set => meshVolumes.observedMeshCacheRevision = value; }
    private int activeMeshCacheRevision { get => meshVolumes.activeMeshCacheRevision; set => meshVolumes.activeMeshCacheRevision = value; }
    private bool observedMeshEnabled { get => meshVolumes.observedMeshEnabled; set => meshVolumes.observedMeshEnabled = value; }
    private bool cachedMeshLodEnabled { get => meshVolumes.cachedMeshLodEnabled; set => meshVolumes.cachedMeshLodEnabled = value; }
    private float cachedMeshLodTolerance { get => meshVolumes.cachedMeshLodTolerance; set => meshVolumes.cachedMeshLodTolerance = value; }
    private long nextMeshCacheAttempt { get => meshVolumes.nextMeshCacheAttempt; set => meshVolumes.nextMeshCacheAttempt = value; }
    private Dictionary<nint, (uint Entity, nint Draw, FrozenMeshGeometryCache? Cache)> weaponContactMeshes => meshVolumes.weaponContactMeshes;
    private List<nint> weaponContactMeshOrder => meshVolumes.weaponContactMeshOrder;

    /// <summary>The rendered surface already cached for this actor; never captures for hair.</summary>
    private FrozenMeshGeometryCache? CachedSurfaceFor(nint address) => meshVolumes.CachedSurfaceFor(address);

    private sealed partial class MeshVolumes
    {
        internal FrozenMeshGeometryCache? frozenMeshCache;
        internal nint frozenMeshActor;
        internal uint frozenMeshEntity;
        internal int observedMeshCacheRevision = -1, activeMeshCacheRevision;
        internal bool observedMeshEnabled, cachedMeshLodEnabled;
        internal float cachedMeshLodTolerance;
        internal long nextMeshCacheAttempt;

        // Other actors' surfaces for weapon contacts only. A failed capture is kept as null so a
        // non-humanoid or unreadable model is not retried on every weapon drop.
        private const int MaxWeaponContactMeshes = 8;
        internal readonly Dictionary<nint, (uint Entity, nint Draw, FrozenMeshGeometryCache? Cache)> weaponContactMeshes = new();
        internal readonly List<nint> weaponContactMeshOrder = new();

        /// <summary>The rendered surface already cached for this actor; never captures for hair.</summary>
        internal FrozenMeshGeometryCache? CachedSurfaceFor(nint address)
        {
            if (address == 0) return null;
            var entity = ctx.Actors.GetEntityId(address);
            if (frozenMeshCache != null && frozenMeshActor == address && frozenMeshEntity == entity) return frozenMeshCache;
            return weaponContactMeshes.TryGetValue(address, out var known) && known.Entity == entity ? known.Cache : null;
        }

        // Weapon and hair contacts need the cache on the main controller only; body mesh volumes need it everywhere.
        private bool FrozenMeshCacheWanted => ctx.Settings.RagdollMeshVolumes ||
            (ctx.Settings.WeaponBodyMeshContacts || ctx.Settings.RagdollHairPhysics && ctx.Settings.RagdollHairBodyMesh) && ctx.Core.RunAnimatedHair;

        // Called from the render owner even while inactive. Only the main controller preloads
        // the local player; other controllers capture their own target on first activation.
        internal void UpdateFrozenMeshCache()
        {
            var settings = ctx.Settings;
            var wanted = FrozenMeshCacheWanted;
            if (observedMeshEnabled != wanted || observedMeshCacheRevision != settings.RagdollMeshCacheRevision)
            {
                frozenMeshCache?.Dispose(); frozenMeshCache = null;
                frozenMeshActor = 0; nextMeshCacheAttempt = 0;
                ClearWeaponContactMeshes();
                observedMeshEnabled = wanted;
                observedMeshCacheRevision = settings.RagdollMeshCacheRevision;
            }
            if (!wanted) return;
            // Body LOD follows the user's setting; weapon contacts always use the coarse LOD (hair queries the full surface).
            var lodEnabled = settings.RagdollMeshCollisionLod || settings.WeaponBodyMeshContacts;
            var tolerance = settings.RagdollMeshCollisionLod
                ? (float.IsFinite(settings.RagdollMeshLodToleranceMm) ? Math.Clamp(settings.RagdollMeshLodToleranceMm, .5f, 50f) : 3f) * .001f
                : RagdollTuning.WeaponBodyMeshLodToleranceMm * .001f;
            PollWeaponContactMeshes();
            var localPlayer = ctx.Actors.LocalPlayer;
            nint address;
            if (settings.RagdollMeshVolumes)
            {
                if (ctx.Core.IsActive && !ctx.Core.TargetObjectAlive()) return;
                address = ctx.Core.IsActive ? ctx.TargetCharacterAddress : ctx.Core.RunAnimatedHair ? localPlayer : 0;
            }
            else address = localPlayer; // Weapon-only: other actors are captured on demand by GetWeaponContactMesh.
            if (address == 0) return;
            var entity = ctx.Actors.GetEntityId(address);
            if (frozenMeshActor != address || frozenMeshEntity != entity)
            {
                frozenMeshCache?.Dispose(); frozenMeshCache = null;
                frozenMeshActor = address; frozenMeshEntity = entity; nextMeshCacheAttempt = 0;
            }
            if (frozenMeshCache == null && ctx.Clock.GetTickCount64(ClockSite.MeshCacheRetry) >= nextMeshCacheAttempt)
            {
                nextMeshCacheAttempt = ctx.Clock.GetTickCount64(ClockSite.MeshCacheRetry) + 2000;
                frozenMeshCache = ctx.Meshes.ProvideBodySurface(address, BodySurfaceUse.Frozen, () => ctx.Core.CaptureFrozenMeshCache(address));
                cachedMeshLodTolerance = -1; // Queue LOD once for this capture, including prewarming.
            }
            if (frozenMeshCache == null) return;
            if (cachedMeshLodEnabled != lodEnabled || cachedMeshLodTolerance != tolerance)
            {
                cachedMeshLodEnabled = lodEnabled; cachedMeshLodTolerance = tolerance;
                ctx.Meshes.ConfigureLod(frozenMeshCache, cachedMeshLodEnabled, tolerance);
            }
            if (ctx.Meshes.PollLod(frozenMeshCache, out var error) && error != null)
                ctx.Log.Warning(error, "Frozen collision cache LOD failed; full cached surface retained.");
        }

        /// <summary>Cached bone-local surface of an actor for weapon contacts. Reuses this controller's
        /// own cache when it holds that actor; otherwise captures once (synchronously) per actor.</summary>
        internal FrozenMeshGeometryCache? GetWeaponContactMesh(nint address)
        {
            if (!ctx.Settings.WeaponBodyMeshContacts || address == 0) return null;
            var entity = ctx.Actors.GetEntityId(address);
            if (frozenMeshCache != null && frozenMeshActor == address && frozenMeshEntity == entity) return frozenMeshCache;
            // Client-spawned actors can reuse an address and entity id; a new draw object means a new model.
            var draw = ctx.Meshes.ModelIdentity(address);
            if (weaponContactMeshes.TryGetValue(address, out var known) && known.Entity == entity && known.Draw == draw) return known.Cache;
            known.Cache?.Dispose();
            weaponContactMeshOrder.Remove(address);
            while (weaponContactMeshOrder.Count >= MaxWeaponContactMeshes)
            {
                var oldest = weaponContactMeshOrder[0];
                weaponContactMeshOrder.RemoveAt(0);
                if (weaponContactMeshes.Remove(oldest, out var evicted)) evicted.Cache?.Dispose();
            }
            // Body status text belongs to this controller's own ragdoll, not to another actor's capture.
            var status = Status;
            var cache = ctx.Meshes.ProvideBodySurface(address, BodySurfaceUse.WeaponContact, () => ctx.Core.CaptureFrozenMeshCache(address));
            Status = status;
            if (cache != null) ctx.Meshes.ConfigureLod(cache, true, RagdollTuning.WeaponBodyMeshLodToleranceMm * .001f);
            weaponContactMeshes[address] = (entity, draw, cache);
            weaponContactMeshOrder.Add(address);
            return cache;
        }

        private void PollWeaponContactMeshes()
        {
            foreach (var entry in weaponContactMeshes.Values)
                if (entry.Cache != null && ctx.Meshes.PollLod(entry.Cache, out var error) && error != null)
                    ctx.Log.Warning(error, "Weapon contact mesh LOD failed; full cached surface retained.");
        }

        private void ClearWeaponContactMeshes()
        {
            foreach (var entry in weaponContactMeshes.Values) entry.Cache?.Dispose();
            weaponContactMeshes.Clear();
            weaponContactMeshOrder.Clear();
        }

        /// <summary>The ragdoll is disposed: the carried surfaces go with it.</summary>
        internal void DisposeCaches()
        {
            frozenMeshCache?.Dispose(); frozenMeshCache = null;
            ClearWeaponContactMeshes();
        }
    }

    /// <summary>The rendered body of a character, partitioned into the rig's regions and frozen bone-local. The
    /// mesh source runs this (<see cref="IMeshSource.ProvideBodySurface"/>); the models come from it too.</summary>
    private FrozenMeshGeometryCache? CaptureFrozenMeshCache(nint address)
    {
        var skel = skeletons.TryGetSkeleton(address);
        if (skel == null) return null;
        if (!skel.HasRoot) return null;
        var defs = GetBoneDefs();
        var names = new Dictionary<string, int>();
        var byName = new Dictionary<string, RagdollBoneDef>();
        foreach (var def in defs)
        {
            var index = ResolveBoneIndex(skel, def.Name);
            if (index < 0) continue;
            names[def.Name] = index; byName[def.Name] = def;
        }
        if (!IsHumanoidSkeleton(names)) return null;
        FrozenMeshGeometryCache? next = null;
        try
        {
            if (settings.RagdollSoftTissueModBones && address == actors.LocalPlayer)
                AppendModSoftTissueDefs(skel, defs, names, byName);
            var worldParts = CaptureBodyMeshVolumes(address, skel, byName, names);
            if (worldParts.Count == 0) return null;
            next = new();
            var root = skel.Root;
            var rootPosition = new Vector3(root.Position.X, root.Position.Y, root.Position.Z);
            var rootRotation = Quaternion.Normalize(new(root.Rotation.X, root.Rotation.Y, root.Rotation.Z, root.Rotation.W));
            var scale = GetCharacterScale(address, skel);
            foreach (var entry in names)
            {
                if (!worldParts.TryGetValue(entry.Value, out var geometry)) continue;
                ref var pose = ref skel.ModelPose[entry.Value];
                var origin = rootPosition + Vector3.Transform(new Vector3(pose.Translation.X, pose.Translation.Y, pose.Translation.Z) * scale, rootRotation);
                var rotation = Quaternion.Normalize(rootRotation * new Quaternion(pose.Rotation.X, pose.Rotation.Y, pose.Rotation.Z, pose.Rotation.W));
                next.Add(entry.Key, geometry, origin, rotation);
            }
            var result = next; next = null;
            log.Info($"Frozen collision cache loaded for 0x{address:X}: {result.Count} regions; reused until explicit refresh or actor change.");
            return result;
        }
        finally { next?.Dispose(); }
    }
}

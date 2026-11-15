// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using AdvancedRagdoll.MeshVolume;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // Weapon-only body surfaces (Ragdoll.WeaponContactMesh.cs).
    private readonly WeaponContactSurfaces weaponContact;

    /// <summary>A weapon rests on the cached body surface instead of the ragdoll volumes. The first weapon makes the
    /// surface twins; their surface comes from the recorded <see cref="GetWeaponContactMesh"/>, asked here.</summary>
    private void AttachWeaponContactBodyCore(BodyHandle weapon)
    {
        var step = weaponContact.PrepareAttach();
        if (step == WeaponContactSurfaces.AttachStep.Skip) return;
        if (step == WeaponContactSurfaces.AttachStep.CreateSurfaces)
            weaponContact.CreateProxies(GetWeaponContactMesh(targetCharacterAddress));
        weaponContact.AddWeapon(weapon);
    }

    /// <summary>
    /// Weapon-only body surfaces. Ragdoll bodies keep their lightweight volumes; each body with a
    /// cached surface gets a kinematic mesh twin that follows it and collides only with registered
    /// weapons. The weapon stops colliding with the volumes those twins replace. Weapons therefore
    /// rest on the rendered body without the cost of full mesh ragdoll contacts. Twins have infinite
    /// mass, so a weapon does not push the body. A component (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class WeaponContactSurfaces
    {
        private struct WeaponContactProxy
        {
            public int BoneListIndex;
            public BodyHandle RagdollBody;
            public BodyHandle Body;
            public TypedIndex Shape;
        }

        private static readonly Vector3 WeaponContactProxyParking = new(0, -9999, 0);

        private readonly SimulationContext ctx;
        private readonly List<WeaponContactProxy> weaponContactProxies = new();
        private readonly HashSet<int> weaponContactBodyHandles = new();
        private readonly HashSet<int> weaponContactProxyHandles = new();
        private readonly HashSet<int> weaponContactProxiedBodyHandles = new();

        internal WeaponContactSurfaces(SimulationContext ctx) { this.ctx = ctx; }

        // The contact filter sets the narrow-phase callbacks hold (the same sets for the ragdoll's lifetime): the
        // registered weapons, the twins, and the ragdoll bodies a twin stands in for.
        internal HashSet<int> WeaponBodies => weaponContactBodyHandles;
        internal HashSet<int> ProxyBodies => weaponContactProxyHandles;
        internal HashSet<int> ProxiedBodies => weaponContactProxiedBodyHandles;

        /// <summary>What attaching a weapon takes.</summary>
        internal enum AttachStep
        {
            /// <summary>Weapon body contacts are off (or there is no simulation, or the rig is generated).</summary>
            Skip,
            /// <summary>No twins yet: the body surface is needed to make them, then the weapon is added.</summary>
            CreateSurfaces,
            /// <summary>The weapon is added (to the twins there are).</summary>
            AddWeapon,
        }

        /// <summary>Weapon body contacts use the cached body surface instead of the ragdoll volumes: whether they are
        /// on, and whether the twins must be made first (the ragdoll asks the recorded surface for them).</summary>
        internal AttachStep PrepareAttach()
        {
            if (ctx.Simulation == null || !ctx.Settings.WeaponBodyMeshContacts || ctx.IsGeneric) return AttachStep.Skip;
            if (weaponContactProxies.Count == 0 && ctx.BufferPool != null) return AttachStep.CreateSurfaces;
            return AttachStep.AddWeapon;
        }

        /// <summary>The weapon collides with the twins (when there are any).</summary>
        internal void AddWeapon(BodyHandle weapon)
        {
            if (weaponContactProxies.Count > 0) weaponContactBodyHandles.Add(weapon.Value);
        }

        internal void Detach(BodyHandle weapon)
        {
            if (!weaponContactBodyHandles.Remove(weapon.Value) || weaponContactBodyHandles.Count > 0) return;
            RemoveWeaponContactProxies();
        }

        /// <summary>The twins, on the body surface of the target (<paramref name="cache"/>, null when it has none). The
        /// simulation and the buffer pool exist (<see cref="PrepareAttach"/>).</summary>
        internal void CreateProxies(FrozenMeshGeometryCache? cache)
        {
            if (ctx.Recorder.IsRecording) ctx.Recorder.MeshCache(cache, "weaponContact");
            if (cache == null) return;
            var ragdollBones = ctx.Bodies;
            for (int i = 0; i < ragdollBones.Count; i++)
            {
                var rb = ragdollBones[i];
                // Bodies that already collide through their rendered mesh need no twin.
                if (rb.BoneIndex < 0 || rb.MeshGeometry != null || !ctx.Meshes.TryGetCollisionGeometry(cache, rb.Name, out var local)) continue;
                Mesh mesh;
                try { mesh = TwoSidedMesh.Create(local, ctx.BufferPool!); }
                catch (Exception ex) { ctx.Log.Warning(ex, $"Weapon contact surface rejected for {rb.Name}; weapon uses its volume."); continue; }
                TypedIndex shape;
                try { shape = ctx.Simulation!.Shapes.Add(mesh); }
                catch { mesh.Dispose(ctx.BufferPool!); throw; }
                ctx.GetBonePoseFromBody(rb, out var origin, out var rotation, out _, out _);
                if (ctx.Recorder.IsRecording) ctx.Recorder.MeshUse("weaponContactProxy", rb.Name, local);
                var body = ctx.Simulation.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(origin, rotation), default,
                    new CollidableDescription(shape, 0.04f), new BodyActivityDescription(0.01f)));
                weaponContactProxies.Add(new WeaponContactProxy { BoneListIndex = i, RagdollBody = rb.BodyHandle, Body = body, Shape = shape });
                weaponContactProxyHandles.Add(body.Value);
                weaponContactProxiedBodyHandles.Add(rb.BodyHandle.Value);
            }
            if (weaponContactProxies.Count > 0)
                ctx.Log.Info($"Weapon contact surfaces: {weaponContactProxies.Count}/{ragdollBones.Count} ragdoll bodies.");
        }

        private void RemoveWeaponContactProxies()
        {
            if (ctx.Simulation != null && ctx.BufferPool != null)
                foreach (var proxy in weaponContactProxies)
                {
                    ctx.RemoveConstraintsOnBody(proxy.Body);
                    try { ctx.Simulation.Bodies.Remove(proxy.Body); }
                    catch (Exception ex) { ctx.Log.Warning(ex, $"Ragdoll: failed to remove weapon contact surface {proxy.Body.Value}"); }
                    try { ctx.Simulation.Shapes.RemoveAndDispose(proxy.Shape, ctx.BufferPool); } catch { }
                }
            ClearState();
        }

        // The simulation owns the native data; on teardown only the bookkeeping is dropped.
        internal void ClearState()
        {
            weaponContactProxies.Clear();
            weaponContactBodyHandles.Clear();
            weaponContactProxyHandles.Clear();
            weaponContactProxiedBodyHandles.Clear();
        }

        /// <summary>Before each step: twins start at their body's bone pose with the same rigid motion,
        /// so both integrate to the same place over the step.</summary>
        internal void Sync()
        {
            if (ctx.Simulation == null || weaponContactProxies.Count == 0) return;
            foreach (var proxy in weaponContactProxies)
            {
                var twin = ctx.Simulation.Bodies.GetBodyReference(proxy.Body);
                if (!TryFindRagdollBone(proxy.RagdollBody, proxy.BoneListIndex, out var rb))
                {
                    // Its body was removed (e.g. a severed limb): retire the twin without a launch velocity.
                    twin.Pose = new RigidPose(ParkingSpot(WeaponContactProxyParking, proxy.Body.Value), Quaternion.Identity);
                    twin.Velocity = default;
                    continue;
                }
                ctx.GetBonePoseFromBody(rb, out var origin, out var rotation, out var bodyPosition, out var velocity);
                twin.Pose = new RigidPose(origin, rotation);
                twin.Velocity = new BodyVelocity(velocity.Linear + Vector3.Cross(velocity.Angular, origin - bodyPosition), velocity.Angular);
                if (velocity.Linear.LengthSquared() + velocity.Angular.LengthSquared() > 1e-10f) twin.Awake = true;
            }
        }

        private bool TryFindRagdollBone(BodyHandle body, int hint, out RagdollBone bone)
        {
            var ragdollBones = ctx.Bodies;
            if (hint < ragdollBones.Count && ragdollBones[hint].BodyHandle.Value == body.Value) { bone = ragdollBones[hint]; return true; }
            foreach (var candidate in ragdollBones)
                if (candidate.BodyHandle.Value == body.Value) { bone = candidate; return true; }
            bone = default;
            return false;
        }
    }

    /// <summary>A dropped weapon's collision hulls from the model it is drawn with (the mesh source reads it), or
    /// null when mesh collision for weapons is off or the model is unsupported. <paramref name="draw"/> is the weapon's
    /// draw object, an opaque id handed to the mesh source (0 = none).</summary>
    internal WeaponMeshGeometry? TryBuildWeaponMesh(nint draw)
    {
        if (!settings.WeaponDropMeshCollision || draw == 0) return null;
        var result = meshes.BuildWeaponMesh(draw, settings.RagdollVerboseLog);
        if (result == null) log.Warning("Weapon mesh: geometry unavailable or unsupported; using simple collision for this drop.");
        return result;
    }
}

// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
// The scenarios of the contact golden test. Each one activates the synthetic humanoid and makes its calls on fixed
// frames, or on frames counted from an event it saw (a landing); see README.md for what each one covers.
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Reflection;
using AdvancedRagdoll;
using AdvancedRagdoll.Tests;

internal static class Scenarios
{
    private static float? Flat(float x, float z) => 0f;
    private static float? Slope(float x, float z) => 0.1f * x - 0.07f * z;
    private static float? Steps(float x, float z) => MathF.Floor(x / 0.4f) * 0.08f;

    private static readonly Vector3 Up = Vector3.UnitY;

    private static Ragdoll.ExternalShapePart Box(float hx, float hy, float hz, Vector3 at = default, Quaternion? rotation = null)
        => new(new Vector3(hx, hy, hz), at, rotation ?? Quaternion.Identity);

    private static Quaternion Yaw(float degrees) => Quaternion.CreateFromAxisAngle(Vector3.UnitY, degrees * MathF.PI / 180f);

    private static string P(Vector3 v) => Output.V(v);

    /// <summary>Writes an external body's pose as the ragdoll reports it.</summary>
    private static void PoseOf(Rig rig, string label, Ragdoll.ExternalBodyHandle? handle)
    {
        var ok = rig.Ragdoll.TryGetExternalBodyPose(handle, out var p, out var q, out var v, out var w);
        rig.Output.Line($"X {label} ok={ok} p={P(p)} q={Output.Q(q)} v={P(v)} w={P(w)}");
    }

    private static void RigPoseOf(Rig rig, string label, Ragdoll.ExternalRigHandle? handle, int index)
    {
        var ok = rig.Ragdoll.TryGetExternalRigBodyPose(handle, index, out var p, out var q, out var v, out var w);
        rig.Output.Line($"X {label}[{index}] ok={ok} p={P(p)} q={Output.Q(q)} v={P(v)} w={P(w)}");
    }

    /// <summary>A three-panel garment worn on the front of the torso (pelvis, spine_a, spine_b): two ball-socket joints
    /// (a swing limit, a pose guide on the first, a joint friction on the second) and a distance limit end to end.</summary>
    private static Ragdoll.ExternalRigHandle? WearGarment(Rig rig, string label, bool collide)
    {
        var forward = rig.Forward;
        var rotation = rig.Skeleton.RootTransform.Rotation;
        var bones = new[] { "pelvis", "spine_a", "spine_b" };
        var bodies = new List<Ragdoll.ExternalRigBodySpec>();
        foreach (var bone in bones)
            bodies.Add(new Ragdoll.ExternalRigBodySpec($"{label}-{bone}", [Box(0.11f, 0.07f, 0.012f)], 0.35f,
                rig.Body(bone) + forward * 0.16f, rotation, Vector3.Zero, Vector3.Zero));
        var joints = new List<Ragdoll.ExternalRigJointSpec>
        {
            new(0, 1, (bodies[0].Position + bodies[1].Position) * 0.5f, 0.8f, 0.28f, 30f, 5f),
            new(1, 2, (bodies[1].Position + bodies[2].Position) * 0.5f, 0.6f) { JointFriction = 0.3f },
            Ragdoll.ExternalRigJointSpec.MakeDistanceLimit(2, 0, 0.1f, 0.3f),
        };
        Ragdoll.ExternalRigHandle? handle = null;
        rig.Call($"TryCreateExternalRig {label} collide={collide}",
            () => rig.Ragdoll.TryCreateExternalRig(bodies, joints, out handle, collide));
        return handle;
    }

    /// <summary>A two-panel garment lying on the ground beside the feet.</summary>
    private static Ragdoll.ExternalRigHandle? DropGarment(Rig rig, string label, bool collide)
    {
        var side = rig.Right * 0.45f;
        var ground = rig.Body("leg_d_l") with { Y = rig.Skeleton.RootTransform.Position.Y + 0.03f };
        var bodies = new List<Ragdoll.ExternalRigBodySpec>
        {
            new($"{label}-a", [Box(0.12f, 0.012f, 0.1f)], 0.3f, ground + side, Quaternion.Identity, Vector3.Zero, Vector3.Zero),
            new($"{label}-b", [Box(0.12f, 0.012f, 0.1f)], 0.3f, ground + side + rig.Forward * 0.22f, Quaternion.Identity,
                Vector3.Zero, Vector3.Zero),
        };
        var joints = new List<Ragdoll.ExternalRigJointSpec>
        {
            new(0, 1, ground + side + rig.Forward * 0.11f, 0.9f),
        };
        Ragdoll.ExternalRigHandle? handle = null;
        rig.Call($"TryCreateExternalRig {label} collide={collide}",
            () => rig.Ragdoll.TryCreateExternalRig(bodies, joints, out handle, collide));
        return handle;
    }

    /// <summary>Applies the same velocity to every simulated body (a throw of the whole corpse).</summary>
    private static void Launch(Rig rig, Vector3 velocity)
    {
        foreach (var bone in rig.Ragdoll.GetSimulatedBoneNames())
            rig.Call($"ApplyImpulse {bone} {P(velocity)}", () => rig.Ragdoll.ApplyImpulse(bone, velocity));
    }

    // ------------------------------------------------------------------ control

    /// <summary>A plain activation on flat ground, nothing else: the control.</summary>
    public static void ControlFall(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0f, 0f, 0f, seed: 1);
        rig.Activate(0.05f);
        for (var f = 0; f < 240; f++) rig.RenderFrame();
        output.Summary = rig.Finals();
    }

    // ------------------------------------------------------------------ standing support

    /// <summary>The standing support with worn and dropped garments and the wrist constraints: create (refused before
    /// physics), updates of the anchor that move the rig (the worn rigs carried along, by the owner's decision), wrist
    /// targets moving, garment edits during the hold, a poke during the hold, removal and a second hold that lifts the
    /// settled corpse.</summary>
    public static void StandingWornRigsWrists(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.WideBent, 0.2f, -0.4f, 25f, seed: 11);
        rig.Activate(0.05f);
        Ragdoll.ExternalRigHandle? worn = null, puddled = null;
        Ragdoll.ExternalBodyHandle? pack = null;
        var anchor = Vector3.Zero;
        var upright = rig.Skeleton.RootTransform.Rotation;
        var carried = new List<string>();
        // The owner's test whether a rig rides along with the hold (asked before every anchor correction).
        bool WornOnly(Ragdoll.ExternalRigHandle h)
        {
            var carry = ReferenceEquals(h, worn);
            output.Line($"carry? {(carry ? "worn" : ReferenceEquals(h, puddled) ? "puddled" : "other")} -> {carry}");
            return carry;
        }
        bool EveryRig(Ragdoll.ExternalRigHandle h)
        {
            carried.Add($"{rig.Frame}");
            return true;
        }
        for (var f = 0; f < 320; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 1)
                    rig.Call("CreateStandingSupport before physics", () => rig.Ragdoll.CreateStandingSupport(Vector3.Zero, Quaternion.Identity));
                if (f == 6)
                {
                    worn = WearGarment(rig, "worn", collide: true);
                    puddled = DropGarment(rig, "puddled", collide: false);
                    rig.Call("TryCreateExternalDynamicBody pack", () => rig.Ragdoll.TryCreateExternalDynamicBody(
                        [Box(0.14f, 0.1f, 0.08f)], 3f, rig.Body("leg_d_r") - rig.Right * 0.4f + Up * 0.12f, Yaw(10f),
                        Vector3.Zero, Vector3.Zero, out pack));
                    rig.Call("CarryExternalRigOnAnchor = worn only", () => { rig.Ragdoll.CarryExternalRigOnAnchor = WornOnly; });
                }
                if (f == 8)
                {
                    anchor = rig.Body("pelvis") + Up * 0.05f;
                    rig.Call($"CreateStandingSupport {P(anchor)}", () => rig.Ragdoll.CreateStandingSupport(anchor, upright));
                }
                if (f == 10)
                    rig.Call("CreateWristConstraints", () => rig.Ragdoll.CreateWristConstraints(
                        rig.Body("hand_l") + Up * 0.25f + rig.Forward * 0.1f, rig.Body("hand_r") + Up * 0.25f + rig.Forward * 0.1f, 400f, 60f));
                if (f >= 20 && f <= 90 && f % 10 == 0)
                {
                    var t = (f - 20) / 70f;
                    var lift = Up * (0.25f + 0.3f * t) + rig.Forward * (0.1f + 0.15f * MathF.Sin(t * 3f));
                    rig.Call($"UpdateWristConstraints t={Output.F(t)}", () => rig.Ragdoll.UpdateWristConstraints(
                        anchor + rig.Right * 0.3f + lift, anchor - rig.Right * 0.3f + lift));
                }
                if (f == 40)
                {
                    anchor += rig.Right * 0.25f + Up * 0.05f;
                    upright = Quaternion.Concatenate(upright, Yaw(10f));
                    rig.Call($"UpdateStandingSupport {P(anchor)}", () => rig.Ragdoll.UpdateStandingSupport(anchor, upright));
                }
                if (f == 55)
                {
                    anchor = rig.Body("spine_b") + rig.Forward * 0.1f;
                    rig.Call($"UpdateStandingSupport spine_b {P(anchor)}", () => rig.Ragdoll.UpdateStandingSupport(anchor, upright, "spine_b"));
                }
                if (f == 60)
                {
                    rig.Call("TrySetExternalRigBodyPose worn 0", () => rig.Ragdoll.TrySetExternalRigBodyPose(worn, 0,
                        rig.Body("pelvis") + rig.Forward * 0.17f, upright, new Vector3(0.1f, 0f, 0f), Vector3.Zero));
                    rig.Call("RelaxExternalRigSwingLimits worn 0.6", () => rig.Ragdoll.RelaxExternalRigSwingLimits(worn, 0.6f));
                    rig.Call("ApplyExternalRigPoseGuidance worn 0.5", () => rig.Ragdoll.ApplyExternalRigPoseGuidance(worn, 0.5f));
                }
                if (f == 70) rig.Call("ApplyImpulse arm_a_l", () => rig.Ragdoll.ApplyImpulse("arm_a_l", new Vector3(0.5f, 2f, -1f)));
                if (f == 80)
                {
                    rig.Call("CarryExternalRigOnAnchor = null", () => { rig.Ragdoll.CarryExternalRigOnAnchor = null; });
                    anchor -= rig.Right * 0.15f;
                    rig.Call($"UpdateStandingSupport spine_b {P(anchor)}", () => rig.Ragdoll.UpdateStandingSupport(anchor, upright, "spine_b"));
                }
                if (f == 85)
                    rig.Call("CarryExternalRigOnAnchor = every rig", () => { rig.Ragdoll.CarryExternalRigOnAnchor = EveryRig; });
                if (f == 100)
                    rig.Call("CreateWristConstraints again", () => rig.Ragdoll.CreateWristConstraints(
                        anchor + Up * 0.4f + rig.Right * 0.2f, anchor + Up * 0.4f - rig.Right * 0.2f));
                if (f == 120)
                {
                    rig.Call("RemoveWristConstraints", () => rig.Ragdoll.RemoveWristConstraints());
                    rig.Call("UpdateWristConstraints after removal", () => rig.Ragdoll.UpdateWristConstraints(anchor, anchor));
                }
                if (f == 140)
                {
                    rig.Call("RemoveStandingSupport", () => rig.Ragdoll.RemoveStandingSupport());
                    rig.Call("RemoveStandingSupport again", () => rig.Ragdoll.RemoveStandingSupport());
                    rig.Call("UpdateStandingSupport after removal", () => rig.Ragdoll.UpdateStandingSupport(anchor, upright));
                }
                if (f == 150)
                {
                    rig.Call("SetExternalRigBodyRagdollContact puddled 0 true", () => rig.Ragdoll.SetExternalRigBodyRagdollContact(puddled, 0, true));
                    rig.Call("SetExternalRigBodyRagdollContact puddled 5 true", () => rig.Ragdoll.SetExternalRigBodyRagdollContact(puddled, 5, true));
                    rig.Call("SetExternalRigBodyRagdollContact worn 1 false", () => rig.Ragdoll.SetExternalRigBodyRagdollContact(worn, 1, false));
                }
                if (f == 200)
                {
                    rig.Call("RemoveExternalRig worn", () => rig.Ragdoll.RemoveExternalRig(worn));
                    rig.Call("RemoveExternalRig worn again", () => rig.Ragdoll.RemoveExternalRig(worn));
                    rig.Call("TrySetExternalRigBodyPose removed", () => rig.Ragdoll.TrySetExternalRigBodyPose(worn, 0,
                        Vector3.Zero, Quaternion.Identity, Vector3.Zero, Vector3.Zero));
                }
                if (f == 220)
                {
                    anchor = rig.Body("pelvis") + Up * 0.3f;
                    rig.Call($"CreateStandingSupport on the settled corpse {P(anchor)}", () => rig.Ragdoll.CreateStandingSupport(anchor, upright));
                }
                if (f == 260) rig.Call("RemoveStandingSupport", () => rig.Ragdoll.RemoveStandingSupport());
                if (f == 270) rig.Call("RemoveExternalRig puddled", () => rig.Ragdoll.RemoveExternalRig(puddled));
            });
            if (f % 20 == 0)
            {
                RigPoseOf(rig, "worn", worn, 1);
                RigPoseOf(rig, "puddled", puddled, 0);
                PoseOf(rig, "pack", pack);
            }
        }
        output.Line($"carry asked {carried.Count} times after frame 85");
        output.Summary = $"{rig.Finals()}; carry asked {carried.Count} times after f85";
    }

    // ------------------------------------------------------------------ external bodies and rigs

    /// <summary>External bodies and rigs on a slope: creation (refused before physics, refused and cleaned up for a bad
    /// rig), every edit, shape changes, pose reads, removal (and calls on removed handles), weapon-contact attach and
    /// detach, then a deactivation with bodies alive and calls on the stale handles after reactivating.</summary>
    public static void ExternalBodiesRigs(Output output)
    {
        using var rig = new Rig(output, Slope, Stance.Staggered, -0.3f, 0.6f, -40f, seed: 12);
        rig.Activate(0.05f);
        Ragdoll.ExternalBodyHandle? a = null, b = null, c = null, d = null;
        Ragdoll.ExternalRigHandle? cape = null, bad = null;
        for (var f = 0; f < 280; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 1)
                    rig.Call("TryCreateExternalDynamicBody before physics", () => rig.Ragdoll.TryCreateExternalDynamicBody(
                        [Box(0.1f, 0.1f, 0.1f)], 1f, Vector3.Zero, Quaternion.Identity, Vector3.Zero, Vector3.Zero, out a));
                if (f == 6)
                {
                    rig.Call("TryCreateExternalDynamicBody a (falls on the chest)", () => rig.Ragdoll.TryCreateExternalDynamicBody(
                        [Box(0.15f, 0.1f, 0.2f)], 4f, rig.Body("spine_b") + rig.Forward * 0.35f + Up * 0.5f, Yaw(30f),
                        new Vector3(0f, -1f, 0f), Vector3.Zero, out a));
                    rig.Call("TryCreateExternalDynamicBody b (compound, spinning)", () => rig.Ragdoll.TryCreateExternalDynamicBody(
                        [Box(0.2f, 0.03f, 0.03f), Box(0.03f, 0.03f, 0.15f, new Vector3(0.17f, 0f, 0.1f), Yaw(20f))], 2.5f,
                        rig.Body("leg_c_l") + rig.Right * 0.35f + Up * 0.3f, Quaternion.Identity, new Vector3(-1.2f, 0f, 0.4f),
                        new Vector3(0f, 6f, 2f), out b));
                    rig.Call("TryCreateExternalDynamicBody c (empty parts)", () => rig.Ragdoll.TryCreateExternalDynamicBody(
                        Array.Empty<Ragdoll.ExternalShapePart>(), 1f, Vector3.Zero, Quaternion.Identity, Vector3.Zero, Vector3.Zero, out c));
                    rig.Call("TryCreateExternalDynamicBody c", () => rig.Ragdoll.TryCreateExternalDynamicBody(
                        [Box(0.05f, 0.3f, 0.02f)], 1.5f, rig.Body("hand_r") + Up * 0.2f, Quaternion.CreateFromAxisAngle(Vector3.UnitX, 1.2f),
                        Vector3.Zero, Vector3.Zero, out c));
                    rig.Call("AttachWeaponContactBody c", () => rig.Ragdoll.AttachWeaponContactBody(c!.Body));
                }
                if (f == 10)
                {
                    var back = -rig.Forward * 0.17f;
                    var bodies = new List<Ragdoll.ExternalRigBodySpec>();
                    var names = new[] { "neck", "spine_c", "spine_b", "spine_a" };
                    for (var i = 0; i < names.Length; i++)
                        bodies.Add(new Ragdoll.ExternalRigBodySpec($"cape-{i}", [Box(0.16f, 0.07f, 0.01f)], 0.25f,
                            rig.Body(names[i]) + back - Up * 0.02f * i, rig.Skeleton.RootTransform.Rotation, Vector3.Zero, Vector3.Zero));
                    var joints = new List<Ragdoll.ExternalRigJointSpec>();
                    for (var i = 0; i + 1 < bodies.Count; i++)
                        joints.Add(new(i, i + 1, (bodies[i].Position + bodies[i + 1].Position) * 0.5f, 0.7f, 0.4f,
                            i == 0 ? 40f : 0f, 6f) { JointFriction = i == 2 ? 0.2f : 0f });
                    joints.Add(Ragdoll.ExternalRigJointSpec.MakeDistanceLimit(3, 0, 0.2f, 0.5f, 12f, 1.2f));
                    joints.Add(new(1, 1, Vector3.Zero, 0.5f)); // parent == child: skipped
                    joints.Add(new(0, 7, Vector3.Zero, 0.5f)); // out of range: skipped
                    rig.Call("TryCreateExternalRig cape", () => rig.Ragdoll.TryCreateExternalRig(bodies, joints, out cape));
                }
                if (f == 12)
                {
                    rig.Call("TryCreateExternalRig empty", () => rig.Ragdoll.TryCreateExternalRig(
                        new List<Ragdoll.ExternalRigBodySpec>(), new List<Ragdoll.ExternalRigJointSpec>(), out bad));
                    var bodies = new List<Ragdoll.ExternalRigBodySpec>
                    {
                        new("bad-0", [Box(0.05f, 0.05f, 0.05f)], 0.2f, rig.Body("head") + Up * 0.5f, Quaternion.Identity, Vector3.Zero, Vector3.Zero),
                        new("bad-1", Array.Empty<Ragdoll.ExternalShapePart>(), 0.2f, rig.Body("head") + Up * 0.7f, Quaternion.Identity, Vector3.Zero, Vector3.Zero),
                    };
                    rig.Call("TryCreateExternalRig with a body without shapes", () => rig.Ragdoll.TryCreateExternalRig(
                        bodies, new List<Ragdoll.ExternalRigJointSpec>(), out bad));
                }
                if (f == 30)
                {
                    rig.Call("TrySetExternalBodyShape a", () => rig.Ragdoll.TrySetExternalBodyShape(a,
                        [Box(0.12f, 0.08f, 0.2f), Box(0.05f, 0.05f, 0.05f, new Vector3(0f, 0.12f, 0f))], 6f));
                    rig.Call("TrySetExternalBodyRotation a", () => rig.Ragdoll.TrySetExternalBodyRotation(a, Yaw(-50f), 0.5f));
                    rig.Call("TryApplyExternalVelocityDelta b", () => rig.Ragdoll.TryApplyExternalVelocityDelta(b, new Vector3(0f, 2f, 0.5f)));
                    rig.Call("TryDampenExternalBodyVelocity b", () => rig.Ragdoll.TryDampenExternalBodyVelocity(b, 0.5f, 0.8f, 0.3f));
                }
                if (f == 40)
                {
                    rig.Call("TrySetExternalRigBodyShape cape 1", () => rig.Ragdoll.TrySetExternalRigBodyShape(cape, 1,
                        [Box(0.16f, 0.07f, 0.004f)], 0.2f));
                    rig.Call("TrySetExternalRigBodyShape cape 9", () => rig.Ragdoll.TrySetExternalRigBodyShape(cape, 9,
                        [Box(0.16f, 0.07f, 0.004f)], 0.2f));
                    rig.Call("TryApplyExternalRigVelocityDelta cape", () => rig.Ragdoll.TryApplyExternalRigVelocityDelta(cape, new Vector3(0.3f, 1f, 0f)));
                    rig.Call("TryDampenExternalRigVelocity cape", () => rig.Ragdoll.TryDampenExternalRigVelocity(cape, 0.7f, 0.9f, 0.5f));
                    rig.Call("RelaxExternalRigSwingLimits cape 1", () => rig.Ragdoll.RelaxExternalRigSwingLimits(cape, 1f));
                    rig.Call("ApplyExternalRigPoseGuidance cape 0", () => rig.Ragdoll.ApplyExternalRigPoseGuidance(cape, 0f));
                }
                if (f == 60)
                {
                    rig.Call("RemoveExternalBody a", () => rig.Ragdoll.RemoveExternalBody(a));
                    rig.Call("TrySetExternalBodyShape removed a", () => rig.Ragdoll.TrySetExternalBodyShape(a, [Box(0.1f, 0.1f, 0.1f)], 1f));
                    rig.Call("TryApplyExternalVelocityDelta removed a", () => rig.Ragdoll.TryApplyExternalVelocityDelta(a, Up));
                    rig.Call("RemoveExternalBody a again", () => rig.Ragdoll.RemoveExternalBody(a));
                    rig.Call("RemoveExternalBody null", () => rig.Ragdoll.RemoveExternalBody(null));
                    rig.Call("RemoveExternalRig null", () => rig.Ragdoll.RemoveExternalRig(null));
                }
                if (f == 80) rig.Call("DetachWeaponContactBody c", () => rig.Ragdoll.DetachWeaponContactBody(c!.Body));
                if (f == 120) rig.Call("RemoveExternalRig cape", () => rig.Ragdoll.RemoveExternalRig(cape));
                if (f == 125) rig.Call("TryDampenExternalRigVelocity removed cape", () => rig.Ragdoll.TryDampenExternalRigVelocity(cape, 0.5f, 0.5f, 0.5f));
                if (f == 150) rig.Call("Deactivate (b and c alive)", () => rig.Ragdoll.Deactivate());
                if (f == 152) rig.Activate(0.05f);
                if (f == 165)
                {
                    rig.Call("TryApplyExternalVelocityDelta stale b", () => rig.Ragdoll.TryApplyExternalVelocityDelta(b, Up));
                    rig.Call("TrySetExternalBodyShape stale c", () => rig.Ragdoll.TrySetExternalBodyShape(c, [Box(0.1f, 0.1f, 0.1f)], 1f));
                    rig.Call("RemoveExternalBody stale b", () => rig.Ragdoll.RemoveExternalBody(b));
                    rig.Call("TryCreateExternalDynamicBody d", () => rig.Ragdoll.TryCreateExternalDynamicBody(
                        [Box(0.2f, 0.05f, 0.2f)], 5f, rig.Body("spine_a") + Up * 0.6f, Yaw(45f), Vector3.Zero, new Vector3(1f, 0f, 0f), out d));
                }
                if (f == 230) rig.Call("RemoveExternalBody d", () => rig.Ragdoll.RemoveExternalBody(d));
            });
            if (f % 10 == 0)
            {
                PoseOf(rig, "a", a);
                PoseOf(rig, "b", b);
                PoseOf(rig, "d", d);
                RigPoseOf(rig, "cape", cape, 3);
                RigPoseOf(rig, "cape", cape, 4);
            }
        }
        output.Summary = rig.Finals();
    }

    // ------------------------------------------------------------------ grab

    /// <summary>Grabs by a second character that walks backward: refused before physics; a rigid grab and a soft grab
    /// (the standing support they take over is removed; a new one is refused while held), a grab on a bone without a
    /// body (carried by its parent at an offset), a grab on a missing bone, retuning, release one by one (the grabber's
    /// collision is parked until the last slot goes), then a second grab session without a grabber.</summary>
    public static void GrabSlotsParking(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0f, 0f, 0f, seed: 13);
        rig.Settings.NpcCollisionAutoSize = true;
        var attacker = rig.AddAttacker(Stance.WideBent, 0.9f);
        var attackerStart = attacker.RootTransform.Position;
        var hand = attacker.Index("hand_r");
        rig.Activate(0.05f);
        int? neck = null, pelvis = null, ear = null, leg = null;
        var legGrip = Vector3.Zero;
        var slots = new List<(string Name, Func<int?> Slot, Vector3 Offset)>
        {
            ("neck", () => neck, Vector3.Zero),
            ("pelvis", () => pelvis, Up * 0.2f),
            ("ear", () => ear, -Up * 0.1f),
        };
        for (var f = 0; f < 260; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f >= 20 && f <= 120)
                {
                    // The grabber walks backward, away from the corpse, and lifts its hand.
                    var t = (f - 20) / 100f;
                    var back = Vector3.Transform(-Vector3.UnitZ, attacker.RootTransform.Rotation);
                    attacker.RootTransform = attacker.RootTransform with { Position = attackerStart + back * (0.6f * t) };
                    attacker.SetBone(hand, attacker.ModelPosition(hand) with { Y = 0.95f + 0.5f * t });
                }
                if (f == 1) rig.Call("CreateGrabConstraint before physics", () => rig.Ragdoll.CreateGrabConstraint("neck", Vector3.Zero));
                if (f == 8)
                {
                    rig.Call("AddLiveCollider attacker", () => rig.Ragdoll.AddLiveCollider(Rig.Attacker));
                    rig.Call("AddLiveCollider attacker again", () => rig.Ragdoll.AddLiveCollider(Rig.Attacker));
                    rig.Call("AddLiveCollider zero", () => rig.Ragdoll.AddLiveCollider(0));
                    rig.Call("AddLiveCollider self", () => rig.Ragdoll.AddLiveCollider(Rig.Actor));
                }
                if (f == 12)
                    rig.Call("CreateStandingSupport", () => rig.Ragdoll.CreateStandingSupport(rig.Body("pelvis"), rig.Skeleton.RootTransform.Rotation));
                if (f == 20)
                {
                    var grip = attacker.World(hand);
                    // The grab without a grabber first: every grab sets the parked grabber (one for all slots), so the
                    // last one wins.
                    ear = rig.Call("CreateGrabConstraint head_end", () => rig.Ragdoll.CreateGrabConstraint("head_end", grip - Up * 0.1f));
                    rig.Call("GrabRigid = true", () => { rig.Ragdoll.GrabRigid = true; });
                    neck = rig.Call($"CreateGrabConstraint neck {P(grip)}", () => rig.Ragdoll.CreateGrabConstraint("neck", grip, Rig.Attacker, 1500f, 8f, 90f));
                    rig.Call("GrabRigid = false", () => { rig.Ragdoll.GrabRigid = false; });
                    pelvis = rig.Call("CreateGrabConstraint pelvis", () => rig.Ragdoll.CreateGrabConstraint("pelvis", grip + Up * 0.2f, Rig.Attacker));
                    rig.Call("CreateGrabConstraint missing", () => rig.Ragdoll.CreateGrabConstraint("missing_bone", grip));
                    rig.Call("TryResolveGrabAttachment hand_end_l", () =>
                    {
                        var ok = rig.Ragdoll.TryResolveGrabAttachment("hand_end_l", out var body, out var offset);
                        return $"{ok} body={body.Value} offset={P(offset)}";
                    });
                }
                if (f == 22)
                {
                    rig.Call("CreateStandingSupport while held", () => rig.Ragdoll.CreateStandingSupport(rig.Body("pelvis"), Quaternion.Identity));
                    rig.Call("CreateWristConstraints while held", () => rig.Ragdoll.CreateWristConstraints(Vector3.Zero, Vector3.Zero));
                }
                if (f > 20 && f <= 140)
                {
                    var grip = attacker.World(hand);
                    foreach (var (name, slot, offset) in slots)
                        if (slot() is { } id) rig.Ragdoll.UpdateGrabTarget(id, grip + offset);
                }
                if (f == 50)
                {
                    rig.Call("SetGrabTuning pelvis", () => rig.Ragdoll.SetGrabTuning(pelvis!.Value, 4000f, 5f, 40f));
                    rig.Call("SetGrabTuning neck (non-finite)", () => rig.Ragdoll.SetGrabTuning(neck!.Value, float.NaN, float.PositiveInfinity, 500f));
                    rig.Call("SetGrabTuning unknown", () => rig.Ragdoll.SetGrabTuning(999, 1f, 1f, 1f));
                    rig.Call("UpdateGrabTarget unknown", () => rig.Ragdoll.UpdateGrabTarget(999, Vector3.Zero));
                }
                if (f == 90)
                {
                    rig.Call("RemoveGrabConstraint pelvis", () => rig.Ragdoll.RemoveGrabConstraint(pelvis!.Value));
                    rig.Call("RemoveGrabConstraint pelvis again", () => rig.Ragdoll.RemoveGrabConstraint(pelvis!.Value));
                    pelvis = null;
                }
                if (f == 110)
                {
                    rig.Call("RemoveGrabConstraint ear", () => rig.Ragdoll.RemoveGrabConstraint(ear!.Value));
                    ear = null;
                }
                if (f == 140)
                {
                    rig.Call("RemoveGrabConstraint neck (last)", () => rig.Ragdoll.RemoveGrabConstraint(neck!.Value));
                    neck = null;
                }
                if (f == 170) rig.Call("RemoveLiveCollider attacker", () => rig.Ragdoll.RemoveLiveCollider(Rig.Attacker));
                if (f == 180)
                {
                    rig.Call("GrabRigid = true", () => { rig.Ragdoll.GrabRigid = true; });
                    legGrip = rig.Body("leg_c_r") + Up * 0.5f;
                    leg = rig.Call("CreateGrabConstraint leg_c_r", () => rig.Ragdoll.CreateGrabConstraint("leg_c_r",
                        legGrip, 0, 800f, 6f, 30f));
                }
                if (f > 180 && f <= 220 && leg is { } l)
                    rig.Ragdoll.UpdateGrabTarget(l, legGrip + rig.Forward * (0.01f * (f - 180)));
                if (f == 220) rig.Call("RemoveGrabConstraint leg", () => rig.Ragdoll.RemoveGrabConstraint(leg!.Value));
            });
        }
        output.Summary = rig.Finals();
    }

    // ------------------------------------------------------------------ anchored grab

    /// <summary>A held anchored grab as the test sees it: the slot's body and grip offset (read from the private grab
    /// slot, so the line shows what the library stored), and the body's orientation when it was caught.</summary>
    private sealed class AnchorTrack
    {
        public required string Label;
        public required int Slot;
        public required Quaternion CaughtOrientation;
        public int Frames;
        public readonly List<string> Samples = new();
    }

    private static readonly int[] AnchorSampleFrames = [0, 5, 10, 20, 40, 80];

    private static object? GrabSlotOf(Rig rig, int slot)
    {
        var grab = typeof(Ragdoll).GetField("grab", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(rig.Ragdoll)!;
        var slots = (System.Collections.IDictionary)grab.GetType().GetProperty("Slots", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(grab)!;
        return slots.Contains(slot) ? slots[slot] : null;
    }

    private static T SlotField<T>(object slot, string name) => (T)slot.GetType().GetField(name)!.GetValue(slot)!;

    /// <summary>Writes the anchored point of a held grab against its target: <c>M label anchor= target= d= centre=
    /// angle=</c> (the grip point on the body, the grab's live target, their distance, the body centre's distance from
    /// the target, how far the body has turned since it was caught, in radians).</summary>
    private static void Measure(Rig rig, AnchorTrack track)
    {
        if (GrabSlotOf(rig, track.Slot) is not { } slot || rig.Simulation is not { } sim) return;
        var body = sim.Bodies.GetBodyReference(SlotField<BepuPhysics.BodyHandle>(slot, "BodyHandle"));
        var pose = body.Pose;
        var anchor = pose.Position + Vector3.Transform(SlotField<Vector3>(slot, "LocalOffset"), pose.Orientation);
        var target = SlotField<Vector3>(slot, "TargetPosition");
        var d = Vector3.Distance(anchor, target);
        var centre = Vector3.Distance(pose.Position, target);
        var angle = 2f * MathF.Acos(MathF.Min(1f, MathF.Abs(Quaternion.Dot(pose.Orientation, track.CaughtOrientation))));
        rig.Output.Line($"M {track.Label} anchor={P(anchor)} target={P(target)} d={Output.F(d)} centre={Output.F(centre)} angle={Output.F(angle)}");
        if (Array.IndexOf(AnchorSampleFrames, track.Frames) >= 0)
            track.Samples.Add($"+{track.Frames}: d {d:F3} centre {centre:F3} turned {angle * 180f / MathF.PI:F0}deg");
        track.Frames++;
    }

    /// <summary>Anchored grabs (<c>CreateGrabConstraintAt</c>): refused before physics, for a non-finite grip point and
    /// a missing bone; a forearm (a bone with its own body) held by a point on its surface off the bone's axis, the
    /// target lifted and swept sideways; then, rigid and without a grabber, a bone without a body (the right hand's
    /// end, carried by the hand) held by a point of the hand's surface, the target lifted. Each held frame writes the
    /// anchored point against the target (see <see cref="Measure"/>); the summary gives the distance at fixed frames
    /// after the grab.</summary>
    public static void GrabAnchored(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0f, 0f, 0f, seed: 17);
        rig.Settings.NpcCollisionAutoSize = true;
        rig.AddAttacker(Stance.WideBent, 0.9f);
        rig.Activate(0.05f);
        int? fore = null, tip = null;
        var foreTarget = Vector3.Zero;
        var tipTarget = Vector3.Zero;
        AnchorTrack? foreTrack = null, tipTrack = null;
        var tracks = new List<AnchorTrack>();

        AnchorTrack? Track(string label, int? slot, Quaternion caught)
        {
            if (slot is not { } id) return null;
            var t = new AnchorTrack { Label = label, Slot = id, CaughtOrientation = caught };
            tracks.Add(t);
            return t;
        }

        for (var f = 0; f < 300; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 1)
                    rig.Call("CreateGrabConstraintAt before physics", () => rig.Ragdoll.CreateGrabConstraintAt("arm_b_l", Vector3.Zero, Vector3.Zero));
                if (f == 8) rig.Call("AddLiveCollider attacker", () => rig.Ragdoll.AddLiveCollider(Rig.Attacker));
                if (f == 40)
                {
                    rig.Ragdoll.TryGetBoneCapsule("arm_b_l", out var c);
                    // On the forearm's surface (a radius off its axis), a little toward the hand.
                    var grip = c.Center + Vector3.Transform(new Vector3(c.Radius, 0.05f, 0f), c.Orientation);
                    rig.Call("CreateGrabConstraintAt arm_b_l NaN", () => rig.Ragdoll.CreateGrabConstraintAt("arm_b_l",
                        new Vector3(float.NaN, 0f, 0f), grip));
                    rig.Call("CreateGrabConstraintAt missing", () => rig.Ragdoll.CreateGrabConstraintAt("missing_bone", grip, grip));
                    foreTarget = grip + Up * 0.5f + rig.Forward * 0.3f;
                    fore = rig.Call($"CreateGrabConstraintAt arm_b_l {P(grip)}", () => rig.Ragdoll.CreateGrabConstraintAt("arm_b_l",
                        grip, foreTarget, Rig.Attacker));
                    foreTrack = Track("forearm", fore, c.Orientation);
                }
                if (f > 40 && f <= 140 && fore is { } fs)
                {
                    // Held still for a second, then swept 0.4 m sideways.
                    var sweep = f <= 100 ? 0f : 0.4f * (f - 100) / 40f;
                    rig.Ragdoll.UpdateGrabTarget(fs, foreTarget + rig.Right * sweep);
                }
                if (f == 150)
                {
                    rig.Call("RemoveGrabConstraint forearm", () => rig.Ragdoll.RemoveGrabConstraint(fore!.Value));
                    fore = null;
                }
                if (f == 170)
                {
                    rig.Ragdoll.TryGetBoneCapsule("hand_r", out var c);
                    var grip = c.Center + Vector3.Transform(new Vector3(c.Radius, 0.02f, 0.01f), c.Orientation);
                    tipTarget = grip + Up * 0.6f;
                    rig.Call("GrabRigid = true", () => { rig.Ragdoll.GrabRigid = true; });
                    tip = rig.Call($"CreateGrabConstraintAt hand_end_r {P(grip)}", () => rig.Ragdoll.CreateGrabConstraintAt("hand_end_r",
                        grip, tipTarget, 0, 1200f, 8f, 60f));
                    rig.Call("GrabRigid = false", () => { rig.Ragdoll.GrabRigid = false; });
                    tipTrack = Track("hand_end", tip, c.Orientation);
                }
                if (f > 170 && f <= 260 && tip is { } ts)
                    rig.Ragdoll.UpdateGrabTarget(ts, tipTarget + rig.Forward * (0.003f * (f - 170)));
                if (f == 270)
                {
                    rig.Call("RemoveGrabConstraint hand_end", () => rig.Ragdoll.RemoveGrabConstraint(tip!.Value));
                    tip = null;
                }
            });
            if (fore != null && foreTrack != null) Measure(rig, foreTrack);
            if (tip != null && tipTrack != null) Measure(rig, tipTrack);
        }
        output.Summary = rig.Finals() + "; " + string.Join("; ", tracks.Select(t => $"{t.Label} {string.Join(", ", t.Samples)}"));
    }

    // ------------------------------------------------------------------ landings

    /// <summary>A fall from 1 m, lying flat, onto flat ground: the landing (hitstop, follow-through release, the event);
    /// a punt during the hitstop; a throw up that comes down inside the cooldown (its arrest is no landing); a bigger
    /// throw after the cooldown (a second landing).</summary>
    public static void LandingCooldown(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0.5f, 0.5f, 70f, seed: 14, lift: 1.0f, pitchDegrees: -80f);
        var landings = new List<int>();
        rig.Ragdoll.HardLanding += _ => landings.Add(rig.Frame);
        rig.Activate(0.05f);
        int first = -1;
        for (var f = 0; f < 300; f++)
        {
            rig.RenderFrame(() =>
            {
                if (first < 0 && landings.Count > 0) first = landings[0];
                if (first < 0) return;
                if (f == first + 1)
                    rig.Call("PuntNearest during the hitstop", () => rig.Ragdoll.PuntNearest(rig.Body("pelvis") + rig.Forward * 0.2f, 0.8f, 1.5f, 0.2f));
                if (f == first + 2) Launch(rig, Up * 3.6f);
                if (f == first + 100) Launch(rig, Up * 5f + rig.Right * 0.5f);
            });
        }
        output.Line($"landings at frames {string.Join(",", landings)}");
        output.Summary = $"{rig.Finals()}; landings at frames {string.Join(",", landings)}";
    }

    /// <summary>A fall from 3.5 m, lying flat, onto raised ground: a slam (the heave); then the standing support lifts the corpse
    /// (a held body does not land) and lets go of it (it lands again).</summary>
    public static void LandingSlamRaised(Output output)
    {
        static float? Raised(float x, float z) => 0.6f;
        using var rig = new Rig(output, Raised, Stance.WideBent, -1f, 1.5f, 120f, seed: 15, lift: 3.5f, pitchDegrees: 85f);
        var landings = new List<int>();
        rig.Ragdoll.HardLanding += _ => landings.Add(rig.Frame);
        rig.Activate(0.05f);
        int first = -1;
        for (var f = 0; f < 320; f++)
        {
            rig.RenderFrame(() =>
            {
                if (first < 0 && landings.Count > 0) first = landings[0];
                if (first < 0) return;
                if (f == first + 60)
                    rig.Call("CreateStandingSupport 1.2 m up", () => rig.Ragdoll.CreateStandingSupport(
                        rig.Body("pelvis") with { Y = 0.6f + 1.2f }, rig.Skeleton.RootTransform.Rotation));
                if (f == first + 110) rig.Call("RemoveStandingSupport", () => rig.Ragdoll.RemoveStandingSupport());
            });
        }
        output.Line($"landings at frames {string.Join(",", landings)}");
        output.Summary = $"{rig.Finals()}; landings at frames {string.Join(",", landings)}";
    }

    // ------------------------------------------------------------------ pokes

    /// <summary>Direct pokes on stepped ground: ignored before physics; impulses (one on a missing bone); angular kicks
    /// that relax their joint for the recoil window (twice on the same bone, one on a missing bone); punts (a hit and a
    /// miss); shakes (a random direction each); then the same pokes once the corpse sleeps.</summary>
    public static void PokesRecoilShake(Output output)
    {
        using var rig = new Rig(output, Steps, Stance.Staggered, 0.1f, -0.2f, 200f, seed: 16);
        rig.Activate(0.05f);
        var asleepAt = -1;
        for (var f = 0; f < 520; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 1)
                {
                    rig.Call("ApplyImpulse before physics", () => rig.Ragdoll.ApplyImpulse("pelvis", Up));
                    rig.Call("ApplyAngularVelocity before physics", () => rig.Ragdoll.ApplyAngularVelocity("pelvis", Up));
                    rig.Call("ApplyShake before physics", () => rig.Ragdoll.ApplyShake(1f));
                    rig.Call("PuntNearest before physics", () => rig.Ragdoll.PuntNearest(Vector3.Zero, 10f, 1f));
                }
                if (f == 10)
                {
                    rig.Call("ApplyImpulse hand_l", () => rig.Ragdoll.ApplyImpulse("hand_l", new Vector3(1f, 2f, 0f)));
                    rig.Call("ApplyImpulse missing", () => rig.Ragdoll.ApplyImpulse("missing", Up));
                }
                if (f == 20)
                {
                    rig.Call("ApplyAngularVelocity arm_b_r", () => rig.Ragdoll.ApplyAngularVelocity("arm_b_r", new Vector3(0f, 0f, 12f)));
                    rig.Call("ApplyAngularVelocity leg_c_l", () => rig.Ragdoll.ApplyAngularVelocity("leg_c_l", new Vector3(8f, 0f, 0f)));
                    rig.Call("ApplyAngularVelocity pelvis", () => rig.Ragdoll.ApplyAngularVelocity("pelvis", new Vector3(0f, 3f, 0f)));
                    rig.Call("ApplyAngularVelocity missing", () => rig.Ragdoll.ApplyAngularVelocity("missing", Up));
                }
                if (f == 32) rig.Call("ApplyAngularVelocity arm_b_r again", () => rig.Ragdoll.ApplyAngularVelocity("arm_b_r", new Vector3(0f, -6f, 6f)));
                if (f == 40)
                {
                    rig.Call("PuntNearest hit", () => rig.Ragdoll.PuntNearest(rig.Body("spine_b") + rig.Forward * 0.3f, 1f, 3f));
                    rig.Call("PuntNearest miss", () => rig.Ragdoll.PuntNearest(rig.Body("spine_b") + Up * 5f, 0.5f, 3f));
                    rig.Call("PuntNearest straight above", () => rig.Ragdoll.PuntNearest(rig.Body("head") + Up * 0.05f, 0.5f, 2f, 0f));
                    rig.Call("NearestBodyDistance", () => rig.Ragdoll.NearestBodyDistance(rig.Body("head") + Up));
                }
                if (f == 50 || f == 52 || f == 54) rig.Call("ApplyShake 2", () => rig.Ragdoll.ApplyShake(2f));
                if (asleepAt < 0 && f > 120 && rig.AllAsleep)
                {
                    asleepAt = f;
                    output.Line($"asleep at frame {f}");
                }
                if (asleepAt < 0 && f == 400)
                {
                    asleepAt = f;
                    output.Line("not asleep by frame 400");
                }
                if (asleepAt >= 0 && f == asleepAt + 1)
                {
                    rig.Call("ApplyImpulse head (asleep)", () => rig.Ragdoll.ApplyImpulse("head", new Vector3(0f, 1.5f, 0.5f)));
                    rig.Call("ApplyAngularVelocity arm_a_l (asleep)", () => rig.Ragdoll.ApplyAngularVelocity("arm_a_l", new Vector3(5f, 0f, 0f)));
                }
                if (asleepAt >= 0 && f == asleepAt + 40)
                {
                    rig.Call("PuntNearest (rig settling)", () => rig.Ragdoll.PuntNearest(rig.Body("pelvis") - rig.Right * 0.4f, 1f, 2.5f, 0.8f));
                    rig.Call("ApplyShake 4", () => rig.Ragdoll.ApplyShake(4f));
                }
            });
        }
        output.Summary = $"{rig.Finals()}; asleep at frame {asleepAt}";
    }

    // ------------------------------------------------------------------ strikes

    /// <summary>A second character next to the corpse, its right arm swinging through it, with a weapon in that hand.
    /// No internal call opens the strike window in the library (only Activate and CancelAttackStrike write it), so the
    /// scenario writes the private window state itself (see README.md). The window: bone and weapon strikes (each body
    /// at most once per window), cancelled; again with the collider in lightweight mode; during the standing support
    /// (no strike); with the collider a traversal ghost; after the collider is removed; a deactivation resets it.</summary>
    public static void StrikeWindowForced(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0f, 0f, -15f, seed: 17);
        var attacker = rig.AddAttacker(Stance.Staggered, 0.65f);
        int shoulder = attacker.Index("arm_a_r"), elbow = attacker.Index("arm_b_r"), wrist = attacker.Index("hand_r"),
            tip = attacker.Index("hand_end_r");
        Vector3 ArmDirection(int f)
        {
            var theta = f is >= 20 and <= 230 ? 0.9f - 0.9f * MathF.Cos(2f * MathF.PI * (f - 20) / 40f) : 0f;
            return new Vector3(0f, -MathF.Cos(theta), MathF.Sin(theta));
        }
        rig.Colliders.Weapon = actor =>
        {
            if (actor != Rig.Attacker) return null;
            var direction = Vector3.Normalize(Vector3.Transform(ArmDirection(rig.Frame), attacker.RootTransform.Rotation));
            var rotation = Quaternion.Normalize(Quaternion.CreateFromAxisAngle(
                Vector3.Normalize(Vector3.Cross(Vector3.UnitY, direction) + new Vector3(1e-4f, 0f, 0f)),
                MathF.Acos(Math.Clamp(Vector3.Dot(Vector3.UnitY, direction), -1f, 1f))));
            return (attacker.World(wrist) + direction * 0.4f, rotation);
        };
        rig.Activate(0.05f);
        for (var f = 0; f < 270; f++)
        {
            rig.RenderFrame(() =>
            {
                var direction = ArmDirection(f);
                var s = attacker.ModelPosition(shoulder);
                attacker.SetBone(elbow, s + direction * 0.28f);
                attacker.SetBone(wrist, s + direction * 0.52f);
                attacker.SetBone(tip, s + direction * 0.6f);
                if (f == 8) rig.Call("AddLiveCollider attacker", () => rig.Ragdoll.AddLiveCollider(Rig.Attacker));
                if (f == 30)
                {
                    rig.Force("strikePower", 0.3f);
                    rig.Force("attackStrikeTimer", 0.6f);
                }
                if (f == 80) rig.Call("CancelAttackStrike", () => rig.Ragdoll.CancelAttackStrike());
                if (f == 100)
                {
                    rig.Call("SetLiveColliderLightweight attacker true", () => rig.Ragdoll.SetLiveColliderLightweight(Rig.Attacker, true));
                    rig.Force("strikePower", 0.2f);
                    rig.Force("attackStrikeTimer", 0.4f);
                }
                if (f == 130)
                {
                    rig.Call("CreateStandingSupport", () => rig.Ragdoll.CreateStandingSupport(rig.Body("pelvis") + Up * 0.1f, rig.Skeleton.RootTransform.Rotation));
                    rig.Call("CancelAttackStrike", () => rig.Ragdoll.CancelAttackStrike());
                    rig.Force("strikePower", 0.3f);
                    rig.Force("attackStrikeTimer", 0.3f);
                }
                if (f == 160) rig.Call("RemoveStandingSupport", () => rig.Ragdoll.RemoveStandingSupport());
                if (f == 170)
                {
                    rig.Call("SetLiveColliderLightweight attacker false", () => rig.Ragdoll.SetLiveColliderLightweight(Rig.Attacker, false));
                    rig.Call("SetLiveColliderTraversalGhost attacker true", () => rig.Ragdoll.SetLiveColliderTraversalGhost(Rig.Attacker, true));
                    rig.Call("CancelAttackStrike", () => rig.Ragdoll.CancelAttackStrike());
                    rig.Force("strikePower", 0.25f);
                    rig.Force("attackStrikeTimer", 0.5f);
                }
                if (f == 200) rig.Call("SetLiveColliderTraversalGhost attacker false", () => rig.Ragdoll.SetLiveColliderTraversalGhost(Rig.Attacker, false));
                if (f == 215)
                {
                    rig.Call("RemoveLiveCollider attacker", () => rig.Ragdoll.RemoveLiveCollider(Rig.Attacker));
                    rig.Call("CancelAttackStrike", () => rig.Ragdoll.CancelAttackStrike());
                    rig.Force("strikePower", 0.25f);
                    rig.Force("attackStrikeTimer", 0.5f);
                }
                if (f == 250) rig.Call("Deactivate", () => rig.Ragdoll.Deactivate());
            });
        }
        output.Summary = rig.Finals();
    }

    // ------------------------------------------------------------------ teardown

    /// <summary>Deactivations in the middle of everything: the standing support with the wrist constraints, a worn rig
    /// carried by the hold, a recoil relax and an open strike window; then, after reactivating, a strike window (the
    /// second character stands still: no strike) and grabs by the second character (its collision parked); a
    /// deactivation while held; a grab after reactivating (slot ids carry over) and a release of a slot of the earlier
    /// activation.</summary>
    public static void TeardownMidContact(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.Staggered, -0.4f, 0.3f, 75f, seed: 18);
        var attacker = rig.AddAttacker(Stance.NarrowStraight, 0.8f);
        var hand = attacker.Index("hand_r");
        rig.Activate(0.05f);
        int? neck = null, pelvis = null, later = null;
        for (var f = 0; f < 220; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 8)
                {
                    rig.Call("AddLiveCollider attacker", () => rig.Ragdoll.AddLiveCollider(Rig.Attacker));
                    WearGarment(rig, "worn", collide: true);
                    rig.Call("CarryExternalRigOnAnchor = every rig", () => { rig.Ragdoll.CarryExternalRigOnAnchor = _ => true; });
                }
                if (f == 10)
                {
                    rig.Call("CreateStandingSupport", () => rig.Ragdoll.CreateStandingSupport(rig.Body("pelvis") + Up * 0.08f,
                        rig.Skeleton.RootTransform.Rotation));
                    rig.Call("CreateWristConstraints", () => rig.Ragdoll.CreateWristConstraints(
                        rig.Body("hand_l") + Up * 0.3f, rig.Body("hand_r") + Up * 0.3f));
                    rig.Call("ApplyAngularVelocity arm_b_l", () => rig.Ragdoll.ApplyAngularVelocity("arm_b_l", new Vector3(0f, 0f, -9f)));
                    rig.Force("strikePower", 0.2f);
                    rig.Force("attackStrikeTimer", 1f);
                }
                if (f == 20) rig.Call("Deactivate (holding)", () => rig.Ragdoll.Deactivate());
                if (f == 22) rig.Activate(0.05f);
                if (f == 30)
                {
                    rig.Call("AddLiveCollider attacker", () => rig.Ragdoll.AddLiveCollider(Rig.Attacker));
                    rig.Force("strikePower", 0.25f);
                    rig.Force("attackStrikeTimer", 0.3f);
                }
                if (f == 34)
                {
                    var grip = attacker.World(hand);
                    rig.Call("GrabRigid = true", () => { rig.Ragdoll.GrabRigid = true; });
                    neck = rig.Call("CreateGrabConstraint neck", () => rig.Ragdoll.CreateGrabConstraint("neck", grip, Rig.Attacker));
                    pelvis = rig.Call("CreateGrabConstraint pelvis", () => rig.Ragdoll.CreateGrabConstraint("pelvis", grip - Up * 0.3f, Rig.Attacker));
                }
                if (f > 34 && f < 60)
                {
                    if (neck is { } n) rig.Ragdoll.UpdateGrabTarget(n, attacker.World(hand) + Up * 0.01f * (f - 34));
                    if (pelvis is { } p) rig.Ragdoll.UpdateGrabTarget(p, attacker.World(hand) - Up * 0.3f);
                }
                if (f == 60) rig.Call("Deactivate (grabbed)", () => rig.Ragdoll.Deactivate());
                if (f == 62)
                {
                    rig.Call("GrabRigid = false", () => { rig.Ragdoll.GrabRigid = false; });
                    rig.Activate(0.05f);
                }
                if (f == 80)
                    later = rig.Call("CreateGrabConstraint spine_c", () => rig.Ragdoll.CreateGrabConstraint("spine_c",
                        rig.Body("spine_c") + Up * 0.4f));
                if (f == 120)
                {
                    rig.Call("RemoveGrabConstraint neck (earlier activation)", () => rig.Ragdoll.RemoveGrabConstraint(neck!.Value));
                    rig.Call("RemoveGrabConstraint spine_c", () => rig.Ragdoll.RemoveGrabConstraint(later!.Value));
                }
            });
        }
        output.Summary = rig.Finals();
    }
}

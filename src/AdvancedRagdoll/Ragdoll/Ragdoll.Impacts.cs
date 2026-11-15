// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Collections.Immutable;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // === Impact weight ===
    //
    // A body that hits the ground hard should land like it weighs something. Physics alone will not do
    // it, and turning up restitution is the wrong instinct: a real coefficient of restitution produces a
    // decaying patter of little hops, which is what a beach ball does — the more of it you add, the
    // LIGHTER the corpse reads. (BEPU has no restitution anyway; it approximates elasticity through
    // contact springs and a recovery-velocity cap, which the ground contacts hold at 2 m/s so the rig
    // can settle at all.)
    //
    // What a fighting game does instead is stage the landing:
    //   1. a freeze — the corpse stops dead for a few dozen milliseconds,
    //   2. the limbs carry on through, because a body's mass is legible in what keeps moving AFTER the
    //      part that stopped,
    //   3. on a genuinely hard blow, and only then, a heave that folds the body about its hips,
    //   4. a camera shake, so the impact is felt outside the body as well.
    //
    // The freeze is not merely a trick, either. A real body arrests over 50-100ms of soft tissue
    // crushing — flesh deforming, absorbing the energy — and a rigid capsule has no way to do that: it
    // simply stops, in one step. The hitstop stands in for the crush, and its duration lands in the same
    // 20-55ms band by no coincidence at all. It is the cheapest possible model of the one thing this
    // engine genuinely cannot do.
    //
    // The impact is measured at the TORSO, not across the rig. A body falls about a pivot: the hips and
    // chest are what arrive, while the feet have been on the floor the whole way down. Averaging the
    // whole rig let the stationary feet drag the number under any plausible threshold, so an ordinary
    // knockdown never registered as a landing at all — only a fall from a height did.
    private const float ImpactSpeed = 2.5f;        // m/s of TORSO descent that counts as a landing
    private const float ImpactHardSpeed = 8f;      // ...and where a landing has become a slam
    private const float ImpactArrestRatio = 0.45f; // how much of it the ground must have taken
    private const float ImpactFreezeMin = 0.020f;  // a knockdown gets a beat
    private const float ImpactFreezeMax = 0.055f;  // a slam gets a full one
    private const float ImpactBounceFraction = 0.30f;
    private const float ImpactMaxBounce = 3.2f;    // m/s — a heave, never a trampoline
    private const float ImpactLimbBounceShare = 0.12f;
    private const float ImpactFollowThroughSeconds = 0.35f;
    private const float ImpactCooldown = 0.7f;     // one landing, not a stutter of them

    /// <summary>The bodies that actually arrive when a body falls. Everything else is along for the
    /// ride — and the feet, in an ordinary collapse, never left the floor.</summary>
    private ImmutableArray<string> TorsoBones => skeletonProfile.ImpactTorsoBones;

    /// <summary>
    /// The joints released on impact so the limbs can follow through.
    ///
    /// This is the part of a landing that actually reads as mass, and it is the twelve principles'
    /// "overlapping action": what tells you a thing is heavy is not the part that stops, it is
    /// everything that keeps going afterwards. An arm that was travelling down keeps travelling down
    /// after the shoulder has arrived; the head whips (the same mechanism that gives a real fall its
    /// secondary head strike).
    ///
    /// And the engine would do all of this by itself. The joint motors were stopping it. So rather than
    /// invent motion — the old pass scattered random angular velocity across every body, which reads as
    /// flailing, not as weight, because it points nowhere in particular — simply take the brakes off and
    /// let the momentum the limbs already have express itself. The swing limits stay on, so a limb
    /// follows through within its anatomy instead of hyperextending.
    /// </summary>
    private ImmutableArray<string> FollowThroughBones => skeletonProfile.ImpactFollowThroughBones;

    /// <summary>Fired when the corpse lands, with the speed it came down at (m/s). The camera hangs off
    /// this — an impact you only see in the body is half an impact.</summary>
    internal event Action<float>? HardLanding;

    /// <summary>Where this landing sits between "a knockdown" and "a slam", smoothed.</summary>
    private static float ImpactSeverity(float speed)
    {
        var t = Math.Clamp((speed - ImpactSpeed) / (ImpactHardSpeed - ImpactSpeed), 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    // The landing weight (Ragdoll.Impacts.cs).
    private readonly Impacts impacts;

    private float impactFreezeRemaining { get => impacts.FreezeRemaining; set => impacts.FreezeRemaining = value; }
    private float impactCooldownRemaining { get => impacts.CooldownRemaining; set => impacts.CooldownRemaining = value; }
    private float previousDescentSpeed { get => impacts.PreviousDescentSpeed; set => impacts.PreviousDescentSpeed = value; }
    private List<int> torsoBodyIndices => impacts.TorsoBodyIndices;
    private uint impactRandomState { get => impacts.RandomState; set => impacts.RandomState = value; }

    /// <summary>
    /// Watch the torso's descent collapse. A landing is simply a fall that the ground took most of in a
    /// single step — no contact bookkeeping needed, and it works whether the corpse lands on terrain, on
    /// a monster, or on its own dropped gear.
    /// </summary>
    private void DetectImpact(float dt)
    {
        // A held/posed body (a grab or the standing support) is not landing.
        if (impacts.Detect(dt, grabSlots.Count > 0 || standingActive, out var speed))
            StageImpact(speed);
    }

    private void StageImpact(float speed)
    {
        if (simulation == null) return;

        impacts.Stage(speed, out var severity, out var heave);

        // Take the brakes off, and the limbs' own momentum carries them through. See FollowThroughBones.
        foreach (var bone in FollowThroughBones)
            ReleaseJointForFollowThrough(bone, ImpactFollowThroughSeconds);

        // The rig is emphatically not at rest any more.
        BeginBiomechanicalSettle();

        log.Info($"Ragdoll: landing at {speed:F1} m/s (severity {severity:F2}) — " +
                 $"{impactFreezeRemaining * 1000f:F0}ms freeze, {heave:F2} m/s heave, follow-through released");
        if (rec != null) RecEvent("HardLanding", speed);
        HardLanding?.Invoke(speed);
    }

    /// <summary>
    /// The landing weight: watches the torso's descent for a landing, and stages one — the hitstop, and on a hard
    /// blow the heave. The joint release, the event and the camera stay with the ragdoll. A component
    /// (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class Impacts
    {
        private readonly SimulationContext ctx;
        internal Impacts(SimulationContext ctx) { this.ctx = ctx; }

        internal float FreezeRemaining;
        internal float CooldownRemaining;
        internal float PreviousDescentSpeed;
        // The bodies that actually arrive when this rig lands (indices into the body list).
        internal readonly List<int> TorsoBodyIndices = new();
        internal uint RandomState = 0x9E3779B9u;

        private bool IsTorsoBody(int index) => TorsoBodyIndices.Count == 0 || TorsoBodyIndices.Contains(index);

        /// <summary>True when the torso's descent was just arrested (a landing), with the speed it came down at.
        /// <paramref name="held"/>: a grab or the standing support holds the rig.</summary>
        internal bool Detect(float dt, bool held, out float speed)
        {
            speed = 0f;
            var simulation = ctx.Simulation;
            var ragdollBones = ctx.Bodies;
            if (simulation == null || ragdollBones.Count == 0) return false;

            // A held/posed body can move downward rapidly without entering a free landing. Do not
            // inject hitstop, heave or joint releases into an active external controller.
            if (held)
            {
                PreviousDescentSpeed = 0f;
                return false;
            }

            if (CooldownRemaining > 0f)
                CooldownRemaining = MathF.Max(0f, CooldownRemaining - dt);

            var descent = 0f;
            var counted = 0;
            for (int i = 0; i < ragdollBones.Count; i++)
            {
                if (!IsTorsoBody(i)) continue;
                descent -= simulation.Bodies.GetBodyReference(ragdollBones[i].BodyHandle).Velocity.Linear.Y;
                counted++;
            }
            if (counted == 0) return false;
            descent /= counted;

            var before = PreviousDescentSpeed;
            PreviousDescentSpeed = descent;

            if (CooldownRemaining > 0f) return false;
            if (before < ImpactSpeed) return false;                    // was not coming down
            if (descent > before * ImpactArrestRatio) return false;    // ground has not taken it yet

            speed = before;
            return true;
        }

        /// <summary>The hitstop and, on a hard blow, the heave.</summary>
        internal void Stage(float speed, out float severity, out float heave)
        {
            var simulation = ctx.Simulation!;
            var ragdollBones = ctx.Bodies;
            severity = ImpactSeverity(speed);

            CooldownRemaining = ImpactCooldown;
            FreezeRemaining = ImpactFreezeMin + (ImpactFreezeMax - ImpactFreezeMin) * severity;

            // Real bodies do not bounce. Flesh is a lousy spring: a coefficient of restitution of maybe 0.1,
            // which is why a dropped body goes "thud" and stays there. The heave is therefore NOT a physical
            // bounce — it is the fighting game's deliberate stylisation of one, and like theirs it earns its
            // place only on a genuinely hard blow. So it scales in from nothing: an ordinary knockdown gets
            // no heave at all, just the beat and the follow-through, which is what a real body does.
            heave = MathF.Min(speed * ImpactBounceFraction, ImpactMaxBounce) * severity;

            if (heave > 0.01f)
            {
                for (int i = 0; i < ragdollBones.Count; i++)
                {
                    var body = simulation.Bodies.GetBodyReference(ragdollBones[i].BodyHandle);

                    // The torso takes the heave and the limbs barely do, so the body FOLDS about its hips on
                    // the way up instead of rising like a plank. A uniform lift is neither physical nor
                    // cinematic; it is a third thing, and it looks like a board.
                    var share = IsTorsoBody(i) ? 1f : ImpactLimbBounceShare;
                    var velocity = body.Velocity.Linear;
                    velocity.Y = MathF.Max(velocity.Y, heave * share);
                    body.Velocity.Linear = velocity;
                    body.Awake = true;
                }
            }
        }

        internal float NextJitter()
        {
            // xorshift — deterministic per corpse, and no allocation on a per-substep path.
            RandomState ^= RandomState << 13;
            RandomState ^= RandomState >> 17;
            RandomState ^= RandomState << 5;
            return (RandomState & 0xFFFFFF) / (float)0x1000000;
        }
    }
}

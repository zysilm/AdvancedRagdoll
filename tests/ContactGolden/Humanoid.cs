// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
// The synthetic humanoid of the contact golden test, a copy of the collapse golden test's (kept separate so neither
// golden output depends on the other): a skeleton profile that builds a full humanoid rig (bone table, ranges of
// motion, mass budget, roles) on the plain bone names of tests/Fixtures/TestHumanoid.cs, and a standing skeleton in a
// few stances. The bone table follows the shape of a real humanoid table (spine chain, clavicles, arms
// with an elbow hinge, legs with two knee hinges and a foot), so the ragdoll finds every landmark it asks for.
using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Numerics;
using AdvancedRagdoll.SkeletonProfiles;

namespace AdvancedRagdoll.Tests;

internal enum Stance
{
    /// <summary>Feet close together, knees straight: the entry conditioning is needed.</summary>
    NarrowStraight,
    /// <summary>Feet apart, knees bent forward: the entry conditioning is skipped.</summary>
    WideBent,
    /// <summary>One foot forward, a little lean: asymmetric legs.</summary>
    Staggered,
}

internal static class GoldenHumanoid
{
    public static SkeletonProfile Profile { get; } = Build();

    private static float D2R(float degrees) => degrees * (MathF.PI / 180f);

    private static BoneRule<T> Rule<T>(BoneNameClass match, T value) => new(match, value);

    private static SkeletonProfile Build()
    {
        var t = TestHumanoid.Profile;
        var classes = t.Classes;
        return new SkeletonProfile
        {
            Name = "contact-golden-humanoid",
            Forward = t.Forward,
            SignatureBones = t.SignatureBones,
            Landmarks = t.Landmarks,
            Classes = classes,
            SideSuffixes = t.SideSuffixes,
            RoleRules = ImmutableArray.Create(
                Rule(classes.SoftTissue, SkeletonBoneRole.SoftBody),
                Rule(classes.Cloth, SkeletonBoneRole.Cloth),
                Rule(classes.Weapon, SkeletonBoneRole.Weapon),
                Rule(BoneNameClass.Exact("pelvis"), SkeletonBoneRole.Pelvis),
                Rule(BoneNameClass.Prefix("spine_").Or(BoneNameClass.Exact("neck")), SkeletonBoneRole.Spine),
                Rule(BoneNameClass.Exact("head"), SkeletonBoneRole.Head),
                Rule(classes.Clavicle.Or(classes.UpperArm), SkeletonBoneRole.Shoulder),
                Rule(classes.Forearm, SkeletonBoneRole.Elbow),
                Rule(classes.Hand, SkeletonBoneRole.Hand),
                Rule(classes.Thigh, SkeletonBoneRole.Hip),
                Rule(classes.Shin, SkeletonBoneRole.Knee),
                Rule(classes.Calf, SkeletonBoneRole.Knee),
                Rule(classes.Foot.Or(classes.Toe), SkeletonBoneRole.Foot)),
            BoneDefinitions = BoneDefinitions(),
            RangeOfMotion = new Dictionary<string, JointRom>
            {
                { "leg_b", new JointRom(D2R(60f), D2R(5f), 0f, 0f, D2R(-10f), D2R(10f)) },
                { "leg_c", new JointRom(D2R(100f), D2R(5f), 0f, 0f, D2R(-10f), D2R(10f)) },
                { "arm_b", new JointRom(D2R(145f), D2R(5f), 0f, 0f, D2R(-80f), D2R(80f)) },
                { "leg_a", new JointRom(D2R(95f), D2R(20f), D2R(45f), D2R(25f), D2R(-40f), D2R(40f)) },
                { "arm_a", new JointRom(D2R(170f), D2R(50f), D2R(170f), D2R(40f), D2R(-90f), D2R(90f)) },
                { "clavicle", new JointRom(D2R(170f), D2R(50f), D2R(170f), D2R(40f), D2R(-90f), D2R(90f)) },
                { "leg_d", new JointRom(D2R(20f), D2R(45f), 0f, 0f, D2R(-20f), D2R(20f)) },
                { "neck", new JointRom(D2R(45f), D2R(45f), D2R(40f), D2R(40f), D2R(-70f), D2R(70f)) },
                { "spine_a", new JointRom(D2R(15f), D2R(15f), D2R(15f), D2R(15f), D2R(-10f), D2R(10f)) },
                { "spine_b", new JointRom(D2R(15f), D2R(15f), D2R(15f), D2R(15f), D2R(-10f), D2R(10f)) },
                { "spine_c", new JointRom(D2R(15f), D2R(15f), D2R(15f), D2R(15f), D2R(-10f), D2R(10f)) },
            }.ToImmutableDictionary(),
            MassBudget = new SkeletonMassBudget
            {
                Fractions = new Dictionary<string, float>
                {
                    { "leg_a", 0.100f }, { "leg_b", 0.0065f }, { "leg_c", 0.0400f }, { "leg_d", 0.0145f },
                    { "arm_a", 0.028f }, { "arm_b", 0.016f },
                    { "head", 0.036f }, { "neck", 0.045f },
                    { "pelvis", 0.142f }, { "spine_a", 0.139f }, { "spine_b", 0.110f }, { "spine_c", 0.104f },
                    { "clavicle", 0.001f },
                }.ToImmutableDictionary(),
                ProximalFingers = ImmutableArray<SidedBone>.Empty,
                HandGroupFraction = 0.006f,
                PalmShare = 0.75f,
                FingersShare = 0.25f,
                FootGroupFraction = 0.0145f,
                FootShare = 0.85f,
                ToesShare = 0.15f,
            },
            HingeFrameTargets = ImmutableArray<HingeFrameTarget>.Empty,
            BuoyancyDensities = t.BuoyancyDensities,
            StompLandmarks = ImmutableArray<StompLandmark>.Empty,
            ImpactTorsoBones = ImmutableArray.Create("pelvis", "spine_a", "spine_b", "spine_c"),
            ImpactFollowThroughBones = ImmutableArray.Create("clavicle_l", "clavicle_r", "arm_a_l", "arm_a_r", "arm_b_l",
                "arm_b_r", "hand_l", "hand_r", "neck", "head"),
            StandingSupportSpine = ImmutableArray.Create("spine_a", "spine_b", "spine_c"),
            LightCollisionAttackScores = BoneScoreTable.Empty,
            LightCollisionFollowScores = BoneScoreTable.Empty,
            FaceLandmarks = FaceLandmarkVocabulary.Empty,
            HairBones = HairBoneNaming.None,
            Digits = DigitNaming.None,
            ClassConstants = new SkeletonClassConstants(MaxLinearVelocity: 18f, MaxAngularVelocity: 14f, MinSolverIterations: 0),
        };
    }

    private static SkeletonBoneDefinition Bone(string name, string? parent, float radius, float halfLength, float mass,
        float swing, int joint, float twistMin, float twistMax, bool enabled = true) => new()
    {
        Name = name,
        SkeletonParent = parent,
        Enabled = enabled,
        CapsuleRadius = radius,
        CapsuleHalfLength = halfLength,
        Mass = mass,
        SwingLimit = swing,
        JointType = joint,
        TwistMinAngle = twistMin,
        TwistMaxAngle = twistMax,
    };

    private static ImmutableArray<SkeletonBoneDefinition> BoneDefinitions()
    {
        var list = new List<SkeletonBoneDefinition>
        {
            Bone("pelvis", null, 0.105f, 0.06f, 8.0f, 0.2f, 0, 0f, 0f),
            Bone("spine_a", "pelvis", 0.10f, 0.05f, 10.0f, 0.2f, 0, -0.1f, 0.1f),
            Bone("spine_b", "spine_a", 0.09f, 0.05f, 5.0f, 0.15f, 0, -0.15f, 0.15f),
            Bone("spine_c", "spine_b", 0.09f, 0.05f, 6.0f, 0.15f, 0, -0.2f, 0.2f),
            Bone("neck", "spine_c", 0.04f, 0.03f, 2.0f, 0.5f, 0, -0.3f, 0.3f),
            Bone("head", "neck", 0.08f, 0.04f, 3.5f, 0.35f, 0, -0.1f, 0.1f),
        };
        foreach (var s in new[] { "_l", "_r" })
        {
            list.Add(Bone("clavicle" + s, "spine_c", 0.025f, 0.04f, 0.5f, 0.35f, 0, -0.15f, 0.15f));
            list.Add(Bone("arm_a" + s, "clavicle" + s, 0.025f, 0.08f, 2.0f, 1.35f, 0, -0.65f, 0.65f));
            list.Add(Bone("arm_b" + s, "arm_a" + s, 0.025f, 0.07f, 1.2f, MathF.PI / 2, 1, -1.25f, 1.25f));
            list.Add(Bone("hand" + s, "arm_b" + s, 0.02f, 0.03f, 0.5f, 0.4f, 0, -0.15f, 0.15f));
        }
        foreach (var s in new[] { "_l", "_r" })
        {
            list.Add(Bone("leg_a" + s, "pelvis", 0.045f, 0.12f, 10.0f, 1.3f, 0, -0.5f, 0.5f));
            list.Add(Bone("leg_b" + s, "leg_a" + s, 0.035f, 0.03f, 1.0f, MathF.PI / 2, 1, -0.1f, 0.1f));
            list.Add(Bone("leg_c" + s, "leg_b" + s, 0.035f, 0.19f, 3.0f, MathF.PI / 2, 1, -0.1f, 0.1f));
            list.Add(Bone("leg_d" + s, "leg_c" + s, 0.01f, 0.0f, 1.0f, 0.3f, 0, -0.64f, 0.65f));
            list.Add(Bone("leg_e" + s, "leg_d" + s, 0.02f, 0.02f, 0.2f, 0.3f, 0, -0.1f, 0.1f, enabled: false));
        }
        return list.ToImmutableArray();
    }

    // Skeleton order: tip bones (head_end, hand_end) give the head and hands a child direction.
    private static readonly string[] Names =
    [
        "root", "pelvis", "spine_a", "spine_b", "spine_c", "neck", "head", "head_end",
        "clavicle_l", "arm_a_l", "arm_b_l", "hand_l", "hand_end_l",
        "clavicle_r", "arm_a_r", "arm_b_r", "hand_r", "hand_end_r",
        "leg_a_l", "leg_b_l", "leg_c_l", "leg_d_l", "leg_e_l",
        "leg_a_r", "leg_b_r", "leg_c_r", "leg_d_r", "leg_e_r",
    ];

    private static readonly short[] Parents =
    [
        -1, 0, 1, 2, 3, 4, 5, 6,
        4, 8, 9, 10, 11,
        4, 13, 14, 15, 16,
        1, 18, 19, 20, 21,
        1, 23, 24, 25, 26,
    ];

    /// <summary>A standing skeleton (model space, Y up, facing +Z) in the given stance.</summary>
    public static SyntheticSkeleton Skeleton(nint actor, Stance stance)
    {
        var p = new Vector3[Names.Length];
        p[0] = Vector3.Zero;
        p[1] = new(0f, 1.00f, 0f);
        p[2] = new(0f, 1.05f, -0.01f);
        p[3] = new(0f, 1.20f, -0.02f);
        p[4] = new(0f, 1.35f, -0.02f);
        p[5] = new(0f, 1.52f, 0f);
        p[6] = new(0f, 1.60f, 0.01f);
        p[7] = new(0f, 1.80f, 0.02f);
        for (var side = 0; side < 2; side++)
        {
            var sx = side == 0 ? 1f : -1f;
            var b = side == 0 ? 8 : 13;
            p[b] = new(0.04f * sx, 1.47f, 0f);
            p[b + 1] = new(0.18f * sx, 1.45f, -0.01f);
            p[b + 2] = new(0.21f * sx, 1.18f, -0.02f);
            p[b + 3] = new(0.22f * sx, 0.95f, 0.03f);
            p[b + 4] = new(0.22f * sx, 0.87f, 0.05f);
        }
        for (var side = 0; side < 2; side++)
        {
            var sx = side == 0 ? 1f : -1f;
            var b = side == 0 ? 18 : 23;
            float footX, kneeZ, footZ;
            switch (stance)
            {
                case Stance.NarrowStraight:
                    footX = 0.06f; kneeZ = 0.01f; footZ = 0f;
                    break;
                case Stance.WideBent:
                    footX = 0.19f; kneeZ = 0.11f; footZ = 0f;
                    break;
                default:
                    footX = 0.09f; kneeZ = side == 0 ? 0.08f : 0.02f; footZ = side == 0 ? 0.18f : -0.12f;
                    break;
            }
            var hipY = stance == Stance.WideBent ? 0.93f : 0.95f;
            p[b] = new(0.10f * sx, hipY, 0f);
            p[b + 1] = new((0.10f + footX) * 0.5f * sx, 0.54f, kneeZ);
            p[b + 2] = new((0.10f + footX) * 0.5f * sx, 0.49f, kneeZ * 0.9f + footZ * 0.1f);
            p[b + 3] = new(footX * sx, 0.09f, footZ);
            p[b + 4] = new(footX * sx, 0.02f, footZ + 0.12f);
        }
        if (stance == Stance.WideBent)
            for (var i = 1; i < 18; i++) p[i].Y -= 0.04f;
        return new SyntheticSkeleton(actor, Names, Parents, p);
    }
}

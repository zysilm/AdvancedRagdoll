// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
using System.Collections.Immutable;
using System.Numerics;
using AdvancedRagdoll.SkeletonProfiles;

namespace AdvancedRagdoll.Tests;

/// <summary>
/// A small humanoid skeleton profile for the tests that ask a profile for bone names: the landmarks, the bone
/// classes and the buoyancy densities, with plain bone names. The densities are those of the tuned humanoid the
/// library was developed with. The bone table, ranges of motion, mass budget and the rest are empty: no test that
/// links this file builds a rig.
/// </summary>
internal static class TestHumanoid
{
    public static SkeletonProfile Profile { get; } = Build();

    private static SidedBone Sided(string root) => new(root, root + "_l", root + "_r");

    private static BoneRule<T> Rule<T>(BoneNameClass match, T value) => new(match, value);

    private static SkeletonProfile Build()
    {
        var landmarks = new SkeletonLandmarks
        {
            Pelvis = "pelvis",
            BellyFollow = "belly",
            LowerSpine = "spine_a",
            MidSpine = "spine_b",
            Chest = "spine_c",
            Neck = "neck",
            Head = "head",
            Breast = Sided("breast"),
            Clavicle = Sided("clavicle"),
            UpperArm = Sided("arm_a"),
            Forearm = Sided("arm_b"),
            Hand = Sided("hand"),
            ArmpitHelper = Sided("armpit"),
            Thigh = Sided("leg_a"),
            Shin = Sided("leg_b"),
            Calf = Sided("leg_c"),
            Foot = Sided("leg_d"),
            Toes = Sided("leg_e"),
        };
        var classes = new SkeletonBoneClasses
        {
            Cloth = BoneNameClass.Prefix("skirt_"),
            SoftTissue = BoneNameClass.Prefix("breast"),
            Weapon = BoneNameClass.Prefix("weapon"),
            Clavicle = BoneNameClass.Prefix("clavicle_"),
            UpperArm = BoneNameClass.Prefix("arm_a_"),
            Forearm = BoneNameClass.Prefix("arm_b_"),
            Hand = BoneNameClass.Prefix("hand_"),
            Thigh = BoneNameClass.Prefix("leg_a_"),
            Shin = BoneNameClass.Prefix("leg_b_"),
            Calf = BoneNameClass.Prefix("leg_c_"),
            Foot = BoneNameClass.Prefix("leg_d_"),
            Toe = BoneNameClass.Prefix("leg_e_"),
            LongLimbRoots = BoneNameClass.Exact("arm_a", "arm_b", "leg_a", "leg_b"),
            HairContact = BoneNameClass.Exact("head", "neck", "pelvis").Or(BoneNameClass.Prefix("spine_", "arm_")),
        };
        return new SkeletonProfile
        {
            Name = "test-humanoid",
            Forward = Vector3.UnitZ,
            SignatureBones = ImmutableArray.Create("pelvis", "spine_a", "neck", "arm_a_l", "arm_a_r", "leg_a_l", "leg_a_r"),
            Landmarks = landmarks,
            Classes = classes,
            SideSuffixes = ImmutableArray.Create("_l", "_r"),
            RoleRules = ImmutableArray<BoneRule<SkeletonBoneRole>>.Empty,
            BoneDefinitions = ImmutableArray<SkeletonBoneDefinition>.Empty,
            RangeOfMotion = ImmutableDictionary<string, JointRom>.Empty,
            MassBudget = new SkeletonMassBudget
            {
                Fractions = ImmutableDictionary<string, float>.Empty,
                ProximalFingers = ImmutableArray<SidedBone>.Empty,
                HandGroupFraction = 0f, PalmShare = 0f, FingersShare = 0f,
                FootGroupFraction = 0f, FootShare = 0f, ToesShare = 0f,
            },
            HingeFrameTargets = ImmutableArray<HingeFrameTarget>.Empty,
            // First match wins: the thigh before the rest of the leg.
            BuoyancyDensities = ImmutableArray.Create(
                Rule(BoneNameClass.Prefix("spine", "breast"), 0.78f),
                Rule(BoneNameClass.Exact("pelvis"), 0.96f),
                Rule(BoneNameClass.Prefix("neck", "head"), 1.02f),
                Rule(BoneNameClass.Prefix("leg_a"), 1.03f),
                Rule(BoneNameClass.Prefix("leg"), 1.09f),
                Rule(BoneNameClass.Prefix("clavicle", "arm_"), 1.04f),
                Rule(BoneNameClass.Prefix("hand"), 1.08f),
                Rule(BoneNameClass.Prefix("skirt_"), 1.0f)),
            StompLandmarks = ImmutableArray<StompLandmark>.Empty,
            ImpactTorsoBones = ImmutableArray<string>.Empty,
            ImpactFollowThroughBones = ImmutableArray<string>.Empty,
            StandingSupportSpine = ImmutableArray<string>.Empty,
            LightCollisionAttackScores = BoneScoreTable.Empty,
            LightCollisionFollowScores = BoneScoreTable.Empty,
            FaceLandmarks = FaceLandmarkVocabulary.Empty,
            HairBones = HairBoneNaming.None,
            Digits = DigitNaming.None,
            ClassConstants = new SkeletonClassConstants(MaxLinearVelocity: 12f, MaxAngularVelocity: 16f, MinSolverIterations: 8),
        };
    }
}

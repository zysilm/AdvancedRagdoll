// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
// Synthetic host of the collapse golden test: skeleton, ground, character, settings, clock, random source, scheduler
// and collapse profiles, all made up and fully deterministic. Every answer is a pure function of what the scenario set.
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using AdvancedRagdoll;
using AdvancedRagdoll.CollapseProfiles;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.Recording;

namespace AdvancedRagdoll.Tests;

/// <summary>A skeleton with a body part only (no partial skeletons, no reference pose).</summary>
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

    public void SetBone(int i, Vector3 position) => Pose[i] = new BoneQs
    {
        Translation = new Vector4(position, 0f),
        Rotation = Quaternion.Identity,
        Scale = new Vector4(1f, 1f, 1f, 0f),
    };

    public nint Actor { get; }
    public nint DrawIdentity => Actor + 1;
    public int BoneCount => Pose.Length;
    public int ParentCount => Parents.Length;
    public Span<BoneQs> ModelPose => Pose;
    public short Parent(int bone) => Parents[bone];
    public int BoneNameCount => Names.Length;
    public string? BoneName(int bone) => Names[bone];
    public ReadOnlySpan<BoneQs> ReferencePose => ReadOnlySpan<BoneQs>.Empty;
    public bool HasRoot => true;
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

    public IHostSkeleton? TryGetSkeleton(nint actor, PosePurpose site = PosePurpose.Other) =>
        ByActor.TryGetValue(actor, out var s) ? s : null;

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

/// <summary>One NPC (object index 1, so not the local player); everything else is absent.</summary>
internal sealed class SyntheticActors : IHostActors
{
    public bool GameRunning => true;
    public nint LocalPlayer => 0;
    public bool IsAlive(nint actor, uint entityId) => true;
    public Vector3 GetPosition(nint actor, ActorSite site) => Vector3.Zero;
    public float GetFacing(nint actor, ActorSite site) => 0f;
    public uint GetEntityId(nint actor) => (uint)actor;
    public ulong GetGameObjectId(nint actor) => (ulong)actor;
    public int GetObjectIndex(nint actor) => 1;
    public bool TryGetDrawScale(nint actor, out Vector3 scale) { scale = Vector3.One; return false; }
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

/// <summary>The settings: everything off except the guided collapse switch the scenario sets.</summary>
internal sealed class SyntheticSettings : IHostSettingsSource
{
    public bool GuidedCollapse;

    public bool Refresh(HostSettings settings)
    {
        settings.GuidedCollapse.Enabled = GuidedCollapse;
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

/// <summary>A seeded random source that writes every draw into the output.</summary>
internal sealed class SeededRandom(Output output, int seed) : IRagdollRandom
{
    private readonly Random random = new(seed);

    public double NextDouble(RandomSite site)
    {
        var value = random.NextDouble();
        output.Line($"Z {site} d={BitConverter.DoubleToUInt64Bits(value):X16}");
        return value;
    }

    public int Next(RandomSite site, int maxExclusive)
    {
        var value = random.Next(maxExclusive);
        output.Line($"Z {site} {maxExclusive}->{value}");
        return value;
    }
}

/// <summary>Runs background work inline, so nothing depends on thread timing.</summary>
internal sealed class InlineScheduler : IRagdollScheduler
{
    public Task<T> Run<T>(Func<T> work) => Task.FromResult(work());

    public bool TryWait<T>(Task<T> task, int milliseconds, out T result)
    {
        result = task.Result;
        return true;
    }
}

internal sealed class OutputLog(Output output) : IRagdollLog
{
    public void Info(string message) => output.Line($"RI {message}");
    public void Warning(string message) => output.Line($"RW {message}");
    public void Warning(Exception exception, string message) => output.Line($"RW {message} [{exception.GetType().Name}: {exception.Message}]");
    public void Error(string message) => output.Line($"RE {message}");
    public void Error(Exception exception, string message) => output.Line($"RE {message} [{exception.GetType().Name}: {exception.Message}]");
}

/// <summary>The collapse profiles the scenarios use, looked up by id ignoring case (as the plugin's book does).</summary>
internal sealed class ProfileBook : ICollapseProfileSource
{
    private readonly Dictionary<string, CollapseProfile> profiles = new(StringComparer.OrdinalIgnoreCase);

    public ProfileBook(params CollapseProfile[] list)
    {
        foreach (var profile in list) profiles[profile.Id] = profile;
    }

    public CollapseProfile? FindById(string id) => profiles.TryGetValue(id, out var p) ? p : null;
}

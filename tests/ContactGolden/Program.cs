// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
// Golden-output test of the contact paths of the ragdoll: the standing support and the wrist constraints, external
// bodies and rigs (and carrying the worn rigs with the standing hold), the grab (and the grabber's parked collision),
// the landing weight (hitstop, follow-through release, heave, cooldown, the HardLanding event), the strike window,
// and the direct pokes (impulse, angular kick with its recoil relax, punt, shake). Each scenario drives Ragdoll
// through its internal host interfaces (Host.cs) with a synthetic humanoid (Humanoid.cs) for a fixed number of render
// frames and writes everything it does: log lines, random draws, events, the results of the calls it makes, the
// ragdoll's contact state, and the state of every body of its simulation, floats as their exact bits. The output must
// be byte for byte the file in Golden/. See README.md.
//
//   dotnet run -c Release                 compare with Golden/ (exit code 0 = identical)
//   dotnet run -c Release -- --write      rewrite Golden/ from the current code
//   dotnet run -c Release -- --out <dir>  also write this run's output to <dir>
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Text;
using AdvancedRagdoll;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.Tests;
using BepuPhysics;

var write = args.Contains("--write");
var outIndex = Array.IndexOf(args, "--out");
var outDir = outIndex >= 0 && outIndex + 1 < args.Length ? args[outIndex + 1] : null;
var goldenDir = FindGoldenDir();
if (goldenDir == null)
{
    Console.WriteLine("FAIL: tests/ContactGolden/Golden not found");
    return 2;
}
Console.WriteLine($"Vector<float>.Count={Vector<float>.Count}, runtime {Environment.Version}, {System.Runtime.InteropServices.RuntimeInformation.OSDescription}");

var scenarios = new (string Name, Action<Output> Run)[]
{
    ("control-fall", Scenarios.ControlFall),
    ("standing-worn-rigs-wrists", Scenarios.StandingWornRigsWrists),
    ("external-bodies-rigs", Scenarios.ExternalBodiesRigs),
    ("grab-slots-parking", Scenarios.GrabSlotsParking),
    ("landing-cooldown", Scenarios.LandingCooldown),
    ("landing-slam-raised", Scenarios.LandingSlamRaised),
    ("pokes-recoil-shake", Scenarios.PokesRecoilShake),
    ("strike-window-forced", Scenarios.StrikeWindowForced),
    ("teardown-mid-contact", Scenarios.TeardownMidContact),
    ("grab-anchored", Scenarios.GrabAnchored),
};

var failed = 0;
foreach (var (name, run) in scenarios)
{
    var output = new Output();
    run(output);
    var bytes = output.ToBytes();
    var path = Path.Combine(goldenDir, name + ".txt");
    if (outDir != null)
    {
        Directory.CreateDirectory(outDir);
        File.WriteAllBytes(Path.Combine(outDir, name + ".txt"), bytes);
    }
    if (write)
    {
        File.WriteAllBytes(path, bytes);
        Console.WriteLine($"WROTE {name}: {output.Frames} frames, {output.LineCount} lines, {bytes.Length} bytes; {output.Summary}");
        continue;
    }
    if (!File.Exists(path))
    {
        failed++;
        Console.WriteLine($"FAIL {name}: no golden file {path}");
        continue;
    }
    var golden = File.ReadAllBytes(path);
    if (golden.AsSpan().SequenceEqual(bytes))
    {
        Console.WriteLine($"IDENTICAL {name}: {output.Frames} frames, {output.LineCount} lines, {bytes.Length} bytes; {output.Summary}");
        continue;
    }
    failed++;
    var expected = Encoding.UTF8.GetString(golden).Split('\n');
    var actual = Encoding.UTF8.GetString(bytes).Split('\n');
    var n = Math.Min(expected.Length, actual.Length);
    var line = 0;
    while (line < n && expected[line] == actual[line]) line++;
    Console.WriteLine($"DIFFERS {name}: first difference at line {line + 1} (golden {expected.Length} lines, now {actual.Length})");
    var frame = expected.Take(line).LastOrDefault(l => l.StartsWith("F ", StringComparison.Ordinal));
    if (frame != null) Console.WriteLine($"  in frame: {frame}");
    Console.WriteLine($"  golden: {(line < expected.Length ? expected[line] : "<end of file>")}");
    Console.WriteLine($"  now:    {(line < actual.Length ? actual[line] : "<end of file>")}");
}
if (write) return 0;
Console.WriteLine(failed == 0 ? $"PASS: {scenarios.Length} scenarios identical to the golden output"
    : $"FAIL: {failed} of {scenarios.Length} scenarios differ from the golden output");
return failed == 0 ? 0 : 1;

static string? FindGoldenDir()
{
    foreach (var start in new[] { AppContext.BaseDirectory, Directory.GetCurrentDirectory() })
        for (var dir = new DirectoryInfo(start); dir != null; dir = dir.Parent)
        {
            var candidate = Path.Combine(dir.FullName, "tests", "ContactGolden", "Golden");
            if (Directory.Exists(candidate)) return candidate;
            if (dir.Name == "ContactGolden" && Directory.Exists(Path.Combine(dir.FullName, "Golden")))
                return Path.Combine(dir.FullName, "Golden");
        }
    return null;
}

/// <summary>The canonical text output: '\n' line ends, floats as the hex of their bits.</summary>
internal sealed class Output
{
    private readonly StringBuilder text = new();
    public readonly List<string> Log = new();
    public int LineCount { get; private set; }
    public int Frames;
    public string Summary = "";

    public void Line(string line)
    {
        text.Append(line).Append('\n');
        LineCount++;
        if (line.StartsWith('R')) Log.Add(line);
    }

    public byte[] ToBytes() => new UTF8Encoding(false).GetBytes(text.ToString());

    public static string F(float v) => BitConverter.SingleToUInt32Bits(v).ToString("X8");
    public static string V(Vector3 v) => $"{F(v.X)},{F(v.Y)},{F(v.Z)}";
    public static string Q(Quaternion q) => $"{F(q.X)},{F(q.Y)},{F(q.Z)},{F(q.W)}";
}

/// <summary>One scenario's world: the host, the character, its ragdoll, and optionally a second character standing
/// next to it (it grabs and strikes the corpse).</summary>
internal sealed class Rig : IDisposable
{
    public const nint Actor = 0x1001;
    public const nint Attacker = 0x2002;

    public readonly Output Output;
    public readonly SyntheticSkeletons Skeletons = new();
    public readonly SyntheticWorld World;
    public readonly SyntheticSettings Settings = new();
    public readonly SyntheticColliders Colliders;
    public readonly ManualClock Clock = new();
    public readonly SyntheticSkeleton Skeleton;
    public readonly Ragdoll Ragdoll;
    public SyntheticSkeleton? AttackerSkeleton;
    public int Frame;
    private string lastState = "";
    private string lastPrivate = "";
    private int awakeBodies;
    private int lastCallFrame = -1000;

    // The simulation is a private member in every version of the library (it moved into the shared context in A6-1;
    // the property kept its name).
    private static readonly PropertyInfo SimulationProperty = typeof(Ragdoll)
        .GetProperty("simulation", BindingFlags.Instance | BindingFlags.NonPublic)!;

    /// <summary>Private state of the ragdoll the output watches: the name the output prints (the ragdoll's field before
    /// stage A6-2) and where it lives since the split (a component's member, or the shared context's, under the
    /// ragdoll's private field of that component). The value is read at the path; on a library without that path
    /// (before stage A6-2) it is read from the ragdoll's field of the printed name (see README.md).</summary>
    private static readonly (string Name, string Path)[] PrivateMembers =
    [
        ("impactFreezeRemaining", "impacts.FreezeRemaining"), ("impactCooldownRemaining", "impacts.CooldownRemaining"),
        ("previousDescentSpeed", "impacts.PreviousDescentSpeed"), ("impactRandomState", "impacts.RandomState"),
        ("attackStrikeTimer", "strike.AttackStrikeTimer"), ("strikePower", "strike.StrikePower"),
        ("strikeColliderAddress", "strike.ColliderAddress"), ("weaponPrevValid", "strike.WeaponPrevValid"),
        ("struckThisWindow", "strike.StruckThisWindow"), ("strikeColliderAddresses", "strike.ColliderAddresses"),
        ("grabSlots", "grab.Slots"), ("nextGrabSlotId", "grab.NextSlotId"),
        ("suspendedNpcAddress", "grab.SuspendedNpcAddress"), ("recoilRelaxers", "joints.recoilRelaxers"),
        ("standingActive", "standing.standingActive"), ("standingConstraints", "standing.standingConstraints"),
        ("wristConstraints", "standing.wristConstraints"), ("carriedRigs", "external.carriedRigs"),
        ("externalBodies", "external.BodyList"), ("externalRigs", "external.Rigs"),
        ("externalBodyGeneration", "external.Generation"),
        ("biomechanicalSettleRemaining", "ctx.BiomechanicalSettleRemaining"), ("prevAllAsleep", "ctx.PrevAllAsleep"),
    ];

    private static readonly string[] PrivateNames = PrivateMembers.Select(m => m.Name).ToArray();
    private static readonly Func<object, object?>[] PrivateGetters = PrivateMembers.Select(m => Getter(m.Name, m.Path)).ToArray();

    private const BindingFlags MemberFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

    private static MemberInfo Member(Type type, string name)
        => (MemberInfo?)type.GetField(name, MemberFlags) ?? type.GetProperty(name, MemberFlags)
           ?? throw new MissingMemberException(type.Name, name);

    private static object? Read(MemberInfo member, object target)
        => member is FieldInfo field ? field.GetValue(target) : ((PropertyInfo)member).GetValue(target);

    /// <summary>The owner of <paramref name="path"/>'s last member (the ragdoll's component or context) and that member;
    /// null when the ragdoll has no field of the path's first name (a library before stage A6-2).</summary>
    private static (Func<object, object> Owner, MemberInfo Member)? Resolve(string path)
    {
        var parts = path.Split('.');
        var first = typeof(Ragdoll).GetField(parts[0], MemberFlags | BindingFlags.DeclaredOnly);
        if (first == null) return null;
        var member = Member(first.FieldType, parts[1]);
        return (o => first.GetValue(o)!, member);
    }

    private static Func<object, object?> Getter(string name, string path)
    {
        if (Resolve(path) is { } at) return o => Read(at.Member, at.Owner(o));
        var field = Member(typeof(Ragdoll), name);
        return o => Read(field, o);
    }

    /// <summary>Writes a private member (by its printed name; at its path since the split). Only for state no internal
    /// call can reach (the strike window, see README.md).</summary>
    public void Force(string name, object value)
    {
        Output.Line($"A force {name}={Format(value)}");
        lastCallFrame = Frame;
        var path = PrivateMembers.First(m => m.Name == name).Path;
        if (Resolve(path) is { } at) Write(at.Member, at.Owner(Ragdoll), value);
        else Write(Member(typeof(Ragdoll), name), Ragdoll, value);
    }

    private static void Write(MemberInfo member, object target, object value)
    {
        if (member is FieldInfo field) field.SetValue(target, value);
        else ((PropertyInfo)member).SetValue(target, value);
    }

    private static string Format(object? value) => value switch
    {
        null => "null",
        float f => Output.F(f),
        bool b => b ? "1" : "0",
        nint p => $"0x{p:X}",
        uint u => $"{u:X8}",
        // Sets of body handles or addresses: their members, sorted; other collections: their size.
        IEnumerable<int> handles => $"[{string.Join(",", handles.Order())}]",
        IEnumerable<nint> addresses => $"[{string.Join(",", addresses.Order().Select(a => $"0x{a:X}"))}]",
        ICollection c => $"#{c.Count}",
        _ => value.ToString() ?? "?",
    };

    public Rig(Output output, Func<float, float, float?> ground, Stance stance, float x, float z, float yawDegrees,
        int seed, float lift = 0f, float pitchDegrees = 0f)
    {
        Output = output;
        World = new SyntheticWorld(ground);
        Colliders = new SyntheticColliders(output);
        Skeleton = GoldenHumanoid.Skeleton(Actor, stance);
        Skeleton.RootTransform = new RootTransform(new Vector3(x, (ground(x, z) ?? 0f) + lift, z),
            Quaternion.Concatenate(Quaternion.CreateFromAxisAngle(Vector3.UnitX, pitchDegrees * MathF.PI / 180f),
                Quaternion.CreateFromAxisAngle(Vector3.UnitY, yawDegrees * MathF.PI / 180f)), Vector3.One);
        Skeletons.ByActor[Actor] = Skeleton;
        Ragdoll = new Ragdoll(new RagdollHostServices
        {
            Skeletons = Skeletons,
            World = World,
            Actors = new SyntheticActors(Skeletons),
            Colliders = Colliders,
            Settings = Settings,
            Log = new OutputLog(output),
            Clock = Clock,
            Random = new SeededRandom(output, seed),
            Scheduler = new InlineScheduler(),
            Build = new RagdollBuildInfo(false, true, "golden"),
        }, NoCollapseProfiles.Instance, GoldenHumanoid.Profile);
        Ragdoll.HardLanding += speed => output.Line($"E HardLanding speed={Output.F(speed)}");
        output.Line($"seed {seed} stance {stance} at {Output.V(Skeleton.RootTransform.Position)} yaw {Output.F(yawDegrees)} pitch {Output.F(pitchDegrees)}");
    }

    /// <summary>A second character, <paramref name="distance"/> in front of the corpse and facing it.</summary>
    public SyntheticSkeleton AddAttacker(Stance stance, float distance)
    {
        var root = Skeleton.RootTransform;
        var forward = Vector3.Transform(Vector3.UnitZ, root.Rotation);
        var rotation = Quaternion.Concatenate(root.Rotation, Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI));
        var s = GoldenHumanoid.Skeleton(Attacker, stance);
        s.RootTransform = new RootTransform(root.Position + forward * distance, rotation, Vector3.One);
        Skeletons.ByActor[Attacker] = s;
        AttackerSkeleton = s;
        Output.Line($"attacker {stance} at {Output.V(s.RootTransform.Position)}");
        return s;
    }

    public BepuPhysics.Simulation? Simulation => SimulationProperty.GetValue(Ragdoll) as BepuPhysics.Simulation;

    /// <summary>A call the scenario makes, with its result.</summary>
    public T Call<T>(string what, Func<T> call)
    {
        Output.Line($"A {what}");
        lastCallFrame = Frame;
        var result = call();
        Output.Line($"A {what} -> {Describe(result)}");
        return result;
    }

    public void Call(string what, Action call) => Call(what, () => { call(); return "done"; });

    private static string Describe(object? result) => result switch
    {
        bool b => b ? "true" : "false",
        null => "null",
        Vector3 v => Output.V(v),
        float f => Output.F(f),
        _ => result.ToString() ?? "null",
    };

    public void Activate(float delay) =>
        Call($"Activate delay={Output.F(delay)}", () => Ragdoll.Activate(Actor, delay, RagdollTriggerMode.FixedDelay));

    public Vector3 Body(string bone) => Ragdoll.GetBodyWorldPosition(bone) ?? Vector3.Zero;

    /// <summary>The corpse's facing (its root rotation), for placing things around it.</summary>
    public Vector3 Forward => Vector3.Transform(Vector3.UnitZ, Skeleton.RootTransform.Rotation);
    public Vector3 Right => Vector3.Transform(Vector3.UnitX, Skeleton.RootTransform.Rotation);

    public bool AllAsleep => Simulation != null && awakeBodies == 0;

    /// <summary>One render frame of 1/60 s: the clock advances, the scenario acts, the ragdoll runs, the output gets
    /// the frame's events, the ragdoll's state when it changed, and the bodies (see <see cref="DumpBodies"/>).</summary>
    public void RenderFrame(Action? before = null)
    {
        Output.Line($"F {Frame}");
        Clock.Now += (long)MathF.Round(Clock.Frequency / 60f);
        before?.Invoke();
        var rays = World.Rays;
        Ragdoll.OnRenderFrame();
        if (World.Rays != rays) Output.Line($"rays {World.Rays - rays}");
        var state = $"C ready={Ragdoll.IsSimulationReady} standing={Ragdoll.IsStandingSupportActive}";
        if (state != lastState)
        {
            Output.Line(state);
            lastState = state;
        }
        var values = new StringBuilder("P");
        for (var i = 0; i < PrivateNames.Length; i++)
            values.Append(' ').Append(PrivateNames[i]).Append('=').Append(Format(PrivateGetters[i](Ragdoll)));
        var privateState = values.ToString();
        if (privateState != lastPrivate)
        {
            Output.Line(privateState);
            lastPrivate = privateState;
        }
        // Every awake dynamic body in full for 30 frames after any call, otherwise every tenth frame. All bodies every
        // twentieth frame. Every frame, the hash of the exact bits of every body.
        DumpBodies(Frame % 20 == 0, Frame - lastCallFrame < 30 || Frame % 10 == 0);
        Frame++;
        Output.Frames = Frame;
    }

    private void DumpBodies(bool all, bool awakeOnes)
    {
        awakeBodies = 0;
        if (Simulation is not { } sim) return;
        var bodies = sim.Bodies;
        var highest = bodies.HandlePool.HighestPossiblyClaimedId;
        if (all) Output.Line($"S bodies<={highest + 1} active={bodies.ActiveSet.Count} statics={sim.Statics.Count}");
        var hash = 14695981039346656037UL;
        void Mix(uint bits) => hash = (hash ^ bits) * 1099511628211UL;
        void MixV(Vector3 v) { Mix(BitConverter.SingleToUInt32Bits(v.X)); Mix(BitConverter.SingleToUInt32Bits(v.Y)); Mix(BitConverter.SingleToUInt32Bits(v.Z)); }
        for (var h = 0; h <= highest; h++)
        {
            var handle = new BodyHandle(h);
            if (!bodies.BodyExists(handle)) continue;
            var body = bodies.GetBodyReference(handle);
            var kinematic = body.Kinematic;
            var awake = body.Awake;
            if (awake && !kinematic) awakeBodies++;
            var pose = body.Pose;
            var velocity = body.Velocity;
            Mix((uint)h);
            Mix((kinematic ? 2u : 0u) | (awake ? 1u : 0u));
            MixV(pose.Position);
            Mix(BitConverter.SingleToUInt32Bits(pose.Orientation.X)); Mix(BitConverter.SingleToUInt32Bits(pose.Orientation.Y));
            Mix(BitConverter.SingleToUInt32Bits(pose.Orientation.Z)); Mix(BitConverter.SingleToUInt32Bits(pose.Orientation.W));
            MixV(velocity.Linear);
            MixV(velocity.Angular);
            if (!all && !(awakeOnes && awake && !kinematic)) continue;
            Output.Line($"B {h} {(kinematic ? 'k' : 'd')}{(awake ? 'a' : 's')} p={Output.V(pose.Position)} q={Output.Q(pose.Orientation)} " +
                        $"v={Output.V(velocity.Linear)} w={Output.V(velocity.Angular)}");
        }
        Output.Line($"H {hash:X16} awake={awakeBodies} constraints={sim.Solver.CountConstraints()}");
    }

    /// <summary>Readable pelvis and head heights (for the console; the bits are in the output).</summary>
    public string Finals()
    {
        var pelvis = Ragdoll.GetBodyWorldPosition("pelvis");
        var head = Ragdoll.GetBodyWorldPosition("head");
        return pelvis is { } p && head is { } h ? $"pelvis y {p.Y:F3}, head y {h.Y:F3}" : "no bodies";
    }

    public void Dispose() => Ragdoll.Dispose();
}

// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
// Golden-output test of the guided collapse (directed collapse: kneel-pitch and collapse profiles; the entry
// conditioning) and the knee power loss. Each scenario drives Ragdoll through its internal host interfaces (Host.cs)
// with a synthetic humanoid (Humanoid.cs) for a fixed number of render frames and writes everything it does: log
// lines, random draws, the results of the calls it makes, the collapse state, and the state of every body of its
// simulation, floats as their exact bits. The output must be byte for byte the file in Golden/. See README.md.
//
//   dotnet run -c Release                 compare with Golden/ (exit code 0 = identical)
//   dotnet run -c Release -- --write      rewrite Golden/ from the current code
//   dotnet run -c Release -- --out <dir>  also write this run's output to <dir>
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Text;
using AdvancedRagdoll;
using AdvancedRagdoll.CollapseProfiles;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.Tests;
using BepuPhysics;

var write = args.Contains("--write");
var outIndex = Array.IndexOf(args, "--out");
var outDir = outIndex >= 0 && outIndex + 1 < args.Length ? args[outIndex + 1] : null;
var goldenDir = FindGoldenDir();
if (goldenDir == null)
{
    Console.WriteLine("FAIL: tests/CollapseGolden/Golden not found");
    return 2;
}
Console.WriteLine($"Vector<float>.Count={Vector<float>.Count}, runtime {Environment.Version}, {System.Runtime.InteropServices.RuntimeInformation.OSDescription}");

var scenarios = new (string Name, Action<Output> Run)[]
{
    ("control-plain", Scenarios.ControlPlain),
    ("entry-knee-settings", Scenarios.EntryKneeSettings),
    ("entry-skipped-raised", Scenarios.EntrySkippedRaised),
    ("entry-grab-knee-stops", Scenarios.EntryGrabKneeStops),
    ("knee-pending-reactivate", Scenarios.KneePendingReactivate),
    ("profile-sideways-seed1", Scenarios.ProfileSidewaysSeed1),
    ("profile-sideways-seed2", Scenarios.ProfileSidewaysSeed2),
    ("profile-directions-stop", Scenarios.ProfileDirectionsStop),
    ("kneel-pitch-chain", Scenarios.KneelPitchChain),
    ("asleep-wake-external", Scenarios.AsleepWakeExternal),
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
            var candidate = Path.Combine(dir.FullName, "tests", "CollapseGolden", "Golden");
            if (Directory.Exists(candidate)) return candidate;
            if (dir.Name == "CollapseGolden" && Directory.Exists(Path.Combine(dir.FullName, "Golden")))
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

/// <summary>One scenario's world: the host, the character, its ragdoll.</summary>
internal sealed class Rig : IDisposable
{
    public const nint Actor = 0x1001;

    public readonly Output Output;
    public readonly SyntheticSkeletons Skeletons = new();
    public readonly SyntheticWorld World;
    public readonly SyntheticSettings Settings = new();
    public readonly ManualClock Clock = new();
    public readonly SyntheticSkeleton Skeleton;
    public readonly Ragdoll Ragdoll;
    public int Frame;
    private string lastState = "";
    private int awakeBodies;

    // The simulation is a private member in every version of the library (it moved into the shared context in A6-1;
    // the property kept its name).
    private static readonly PropertyInfo SimulationProperty = typeof(Ragdoll)
        .GetProperty("simulation", BindingFlags.Instance | BindingFlags.NonPublic)!;

    public Rig(Output output, Func<float, float, float?> ground, Stance stance, float x, float z, float yawDegrees,
        int seed, bool guidedCollapse = false)
    {
        Output = output;
        World = new SyntheticWorld(ground);
        Settings.GuidedCollapse = guidedCollapse;
        Skeleton = GoldenHumanoid.Skeleton(Actor, stance);
        Skeleton.RootTransform = new RootTransform(new Vector3(x, ground(x, z) ?? 0f, z),
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, yawDegrees * MathF.PI / 180f), Vector3.One);
        Skeletons.ByActor[Actor] = Skeleton;
        Ragdoll = new Ragdoll(new RagdollHostServices
        {
            Skeletons = Skeletons,
            World = World,
            Actors = new SyntheticActors(),
            Colliders = new NoColliders(),
            Settings = Settings,
            Log = new OutputLog(output),
            Clock = Clock,
            Random = new SeededRandom(output, seed),
            Scheduler = new InlineScheduler(),
            Build = new RagdollBuildInfo(false, true, "golden"),
        }, Profiles.Book, GoldenHumanoid.Profile);
        output.Line($"seed {seed} stance {stance} at {Output.V(Skeleton.RootTransform.Position)} yaw {Output.F(yawDegrees)}");
    }

    public BepuPhysics.Simulation? Simulation => SimulationProperty.GetValue(Ragdoll) as BepuPhysics.Simulation;

    /// <summary>A call the scenario makes, with its result.</summary>
    public void Call(string what, Func<object?> call)
    {
        Output.Line($"A {what}");
        lastCallFrame = Frame;
        var result = call();
        Output.Line($"A {what} -> {(result is bool b ? (b ? "true" : "false") : result?.ToString() ?? "null")}");
    }

    public void Call(string what, Action call) => Call(what, () => { call(); return "done"; });

    public void Activate(float delay) =>
        Call($"Activate delay={Output.F(delay)}", () => Ragdoll.Activate(Actor, delay, RagdollTriggerMode.FixedDelay));

    /// <summary>True when the ragdoll logged a line containing <paramref name="text"/> at index <paramref name="since"/> or later.</summary>
    public bool Logged(string text, int since = 0)
    {
        for (var i = since; i < Output.Log.Count; i++)
            if (Output.Log[i].Contains(text, StringComparison.Ordinal)) return true;
        return false;
    }

    public bool AllAsleep => Simulation != null && awakeBodies == 0;

    /// <summary>One render frame of 1/60 s: the clock advances, the ragdoll runs, the output gets the frame's events,
    /// the collapse state when it changed, and the bodies (see <see cref="DumpBodies"/>).</summary>
    public void RenderFrame(Action? before = null)
    {
        Output.Line($"F {Frame}");
        Clock.Now += (long)MathF.Round(Clock.Frequency / 60f);
        before?.Invoke();
        var rays = World.Rays;
        Ragdoll.OnRenderFrame();
        if (World.Rays != rays) Output.Line($"rays {World.Rays - rays}");
        var directed = Ragdoll.DirectedCollapseSpikeActive;
        var knee = Ragdoll.KneePowerLossPatternActive;
        var state = $"C ready={Ragdoll.IsSimulationReady} directed={directed} knee={knee}";
        if (state != lastState)
        {
            Output.Line(state);
            lastState = state;
        }
        // Every awake body in full while a collapse runs and for 45 frames after any call (the entry conditioning
        // lasts at most 0.42 s); otherwise every tenth frame. All bodies every twentieth frame. Every frame, the hash
        // of the exact bits of every body.
        DumpBodies(Frame % 20 == 0, directed || knee || Frame - lastCallFrame < 45 || Frame % 10 == 0);
        Frame++;
        Output.Frames = Frame;
    }

    private int lastCallFrame = -1000;

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

/// <summary>The collapse profiles of the scenarios.</summary>
internal static class Profiles
{
    private static CollapseControllerProfile C(string type, Action<CollapseControllerProfile>? set = null)
    {
        var c = new CollapseControllerProfile { Type = type };
        set?.Invoke(c);
        return c;
    }

    private static CollapsePhaseProfile Phase(string name, float duration, params CollapseControllerProfile[] controllers)
        => new() { Name = name, Duration = duration, Controllers = controllers.ToList() };

    /// <summary>Feet planted, the pelvis drops sideways (a random side every frame), the torso folds forward, then
    /// the rig is released to passive physics (a controller after the release still runs).</summary>
    public static readonly CollapseProfile Sideways = new()
    {
        Id = "golden-sideways",
        DisplayName = "Golden sideways",
        Phases =
        [
            Phase("plant", 0.35f,
                C("footPlant", c => { c.Bones = ["leg_d_l", "leg_d_r"]; c.Force = 900f; c.StrengthFrom = 1f; c.StrengthTo = 0.7f; }),
                C("pelvisDrop", c => { c.Direction = "sideways"; c.Force = 650f; c.DropFrom = 0f; c.Drop = 0.18f; c.DistanceFrom = 0f; c.Distance = 0.12f; c.PitchDegreesFrom = 0f; c.PitchDegrees = 12f; })),
            Phase("fold", 0.45f,
                C("footPlant", c => { c.Bones = ["leg_d_l", "leg_d_r"]; c.Force = 700f; c.Frequency = 60f; c.StrengthFrom = 0.7f; c.StrengthTo = 0.3f; }),
                C("pelvisDrop", c => { c.Direction = "forward"; c.Force = 500f; c.Frequency = 30f; c.DropFrom = 0.18f; c.Drop = 0.42f; c.DistanceFrom = 0.12f; c.Distance = 0.25f; c.PelvisPitchScale = 0.5f; }),
                C("torsoPitch", c => { c.Force = 320f; c.Frequency = 20f; c.PitchDegreesFrom = 0f; c.PitchDegrees = 38f; c.StartTime = 0.2f; c.EndTime = 0.9f; })),
            Phase("release", 0.3f,
                C("footPlant", c => { c.Bones = ["leg_d_l", "leg_d_r"]; c.Force = 500f; c.StrengthFrom = 0.3f; c.StrengthTo = 0f; }),
                C("releaseToPassive"),
                C("pelvisDrop", c => { c.Direction = "sideways"; c.Force = 200f; c.Drop = 0.5f; })),
        ],
    };

    /// <summary>Every direction, bone overrides, a foot list with a missing and a side-less bone, an empty phase, an
    /// unknown controller type, a controller whose window is inverted, and a phase shorter than a frame; it ends when
    /// its phases run out (no release controller).</summary>
    public static readonly CollapseProfile Directions = new()
    {
        Id = "golden-directions",
        DisplayName = "Golden directions",
        Phases =
        [
            Phase("back", 0.25f,
                C("pelvisDrop", c => { c.Direction = "backward"; c.Bone = "spine_a"; c.Force = 400f; c.Drop = 0.1f; c.Distance = 0.08f; }),
                C("footPlant", c => { c.Bones = ["leg_d_l", "leg_d_r", "pelvis", "missing_l"]; c.Force = 800f; })),
            Phase("empty", 0.1f),
            Phase("left", 0.2f,
                C("pelvisDrop", c => { c.Direction = "LEFT"; c.Force = 450f; c.DropFrom = 0.1f; c.Drop = 0.25f; c.Distance = 0.1f; }),
                C("torsoPitch", c => { c.Bone = "spine_b"; c.Force = 250f; c.PitchDegreesFrom = 5f; c.PitchDegrees = -20f; c.PelvisPitchScale = 0.5f; })),
            Phase("right", 0.2f,
                C("pelvisDrop", c => { c.Direction = "right"; c.Force = 450f; c.DropFrom = 0.25f; c.Drop = 0.35f; c.Distance = 0.15f; c.StrengthFrom = 1f; c.StrengthTo = 0.2f; }),
                C("wobble", c => { c.Force = 999f; })),
            Phase("tiny", 0.005f,
                C("torsoPitch", c => { c.Force = 100f; c.PitchDegrees = 10f; c.StartTime = 0.9f; c.EndTime = 0.1f; })),
        ],
    };

    /// <summary>A profile without phases: it cannot begin.</summary>
    public static readonly CollapseProfile Empty = new() { Id = "golden-empty", DisplayName = "Golden empty" };

    public static readonly ProfileBook Book = new(Sideways, Directions, Empty);
}

internal static class Scenarios
{
    private static float? Flat(float x, float z) => 0f;

    /// <summary>A plain activation on flat ground, no collapse: the control.</summary>
    public static void ControlPlain(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0f, 0f, 0f, seed: 1);
        rig.Activate(0.05f);
        for (var f = 0; f < 300; f++) rig.RenderFrame();
        output.Summary = rig.Finals();
    }

    /// <summary>The guided collapse switch on: the activation arms the entry-conditioned knee power loss; the narrow
    /// straight stance needs the entry conditioning, which hands over to the knee power loss when ready; the knee
    /// power loss runs buckle, torso loss and release.</summary>
    public static void EntryKneeSettings(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0.3f, -0.2f, 30f, seed: 2, guidedCollapse: true);
        rig.Activate(0.05f);
        for (var f = 0; f < 300; f++) rig.RenderFrame();
        output.Summary = rig.Finals();
    }

    /// <summary>The switch on, on raised ground, in a wide stance with bent knees: the entry conditioning is skipped
    /// and the knee power loss begins at once.</summary>
    public static void EntrySkippedRaised(Output output)
    {
        static float? Raised(float x, float z) => 0.5f;
        using var rig = new Rig(output, Raised, Stance.WideBent, -1.2f, 2.0f, -60f, seed: 3, guidedCollapse: true);
        rig.Activate(0.05f);
        for (var f = 0; f < 300; f++) rig.RenderFrame();
        output.Summary = rig.Finals();
    }

    /// <summary>On a slope: the entry-conditioned knee power loss requested while physics runs (begins at once); a
    /// grab cuts the entry conditioning short; the knee power loss is requested while ready, stopped in the buckle,
    /// begun again and stopped in the torso loss.</summary>
    public static void EntryGrabKneeStops(Output output)
    {
        static float? Slope(float x, float z) => 0.12f * x + 0.06f * z;
        using var rig = new Rig(output, Slope, Stance.Staggered, 0.4f, -0.3f, 145f, seed: 4);
        rig.Activate(0.05f);
        int? slot = null;
        var torsoSeenAt = -1;
        var secondBeginLog = 0;
        for (var f = 0; f < 300; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 8) rig.Call("RequestEntryConditionedKneePowerLossOnReady", () => rig.Ragdoll.RequestEntryConditionedKneePowerLossOnReady());
                if (f == 14)
                {
                    var hand = rig.Ragdoll.GetBodyWorldPosition("hand_r") ?? Vector3.Zero;
                    rig.Call("CreateGrabConstraint hand_r", () => slot = rig.Ragdoll.CreateGrabConstraint("hand_r", hand + new Vector3(0f, 0.3f, 0f)));
                }
                if (f == 30 && slot is { } s) rig.Call("RemoveGrabConstraint", () => rig.Ragdoll.RemoveGrabConstraint(s));
                if (f == 40) rig.Call("RequestKneePowerLossForwardOnReady", () => rig.Ragdoll.RequestKneePowerLossForwardOnReady());
                if (f == 50) rig.Call("StopKneePowerLossPattern", () => rig.Ragdoll.StopKneePowerLossPattern());
                if (f == 60)
                {
                    secondBeginLog = output.Log.Count;
                    rig.Call("BeginKneePowerLossForwardPattern", () => rig.Ragdoll.BeginKneePowerLossForwardPattern());
                }
                if (f > 60 && torsoSeenAt < 0 && rig.Logged("phase -> TorsoLoss", secondBeginLog)) torsoSeenAt = f;
                if (torsoSeenAt >= 0 && f == torsoSeenAt + 6) rig.Call("StopKneePowerLossPattern", () => rig.Ragdoll.StopKneePowerLossPattern());
            });
        }
        output.Line($"torso loss seen at frame {torsoSeenAt}");
        output.Summary = $"{rig.Finals()}; torso loss at frame {torsoSeenAt}";
    }

    /// <summary>On stepped ground: the knee power loss requested while the ragdoll still waits for its start (armed,
    /// begun when physics is ready), run to its end; then deactivated and activated again without it.</summary>
    public static void KneePendingReactivate(Output output)
    {
        static float? Steps(float x, float z) => MathF.Floor(z / 0.5f) * 0.1f;
        using var rig = new Rig(output, Steps, Stance.WideBent, 0f, 0.2f, 90f, seed: 5);
        rig.Activate(0.1f);
        for (var f = 0; f < 300; f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 2) rig.Call("RequestKneePowerLossForwardOnReady", () => rig.Ragdoll.RequestKneePowerLossForwardOnReady());
                if (f == 200) rig.Call("Deactivate", () => rig.Ragdoll.Deactivate());
                if (f == 202) rig.Activate(0.05f);
            });
        }
        output.Summary = rig.Finals();
    }

    /// <summary>The sideways profile requested while waiting (armed under an id in other case), every phase, the
    /// release to passive; seed 1, flat ground.</summary>
    public static void ProfileSidewaysSeed1(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, 0f, 0f, 0f, seed: 1);
        rig.Activate(0.05f);
        for (var f = 0; f < 300; f++)
            rig.RenderFrame(() =>
            {
                if (f == 1) rig.Call("RequestProfileDirectedCollapseOnReady GOLDEN-SIDEWAYS", () => rig.Ragdoll.RequestProfileDirectedCollapseOnReady("GOLDEN-SIDEWAYS"));
            });
        output.Summary = rig.Finals();
    }

    /// <summary>The sideways profile requested while physics runs (begins at once); seed 2, staggered stance on a
    /// slope, facing away.</summary>
    public static void ProfileSidewaysSeed2(Output output)
    {
        static float? Slope(float x, float z) => -0.1f * x + 0.08f * z;
        using var rig = new Rig(output, Slope, Stance.Staggered, 0.7f, 0.5f, -100f, seed: 2);
        rig.Activate(0.05f);
        for (var f = 0; f < 300; f++)
            rig.RenderFrame(() =>
            {
                if (f == 10) rig.Call("RequestProfileDirectedCollapseOnReady golden-sideways", () => rig.Ragdoll.RequestProfileDirectedCollapseOnReady("golden-sideways"));
            });
        output.Summary = rig.Finals();
    }

    /// <summary>The directions profile begun while physics runs and stopped in its third phase; a missing profile
    /// and one without phases do not begin; the directions profile again, to its natural end; then the sideways
    /// profile, cut short by the knee power loss (which stops the directed collapse).</summary>
    public static void ProfileDirectionsStop(Output output)
    {
        static float? Raised(float x, float z) => 0.3f;
        using var rig = new Rig(output, Raised, Stance.WideBent, 2f, -1f, 200f, seed: 3);
        rig.Activate(0.05f);
        for (var f = 0; f < 300; f++)
            rig.RenderFrame(() =>
            {
                if (f == 8) rig.Call("BeginProfileDirectedCollapseSpike golden-directions", () => rig.Ragdoll.BeginProfileDirectedCollapseSpike("golden-directions"));
                if (f == 38) rig.Call("StopDirectedCollapseSpike", () => rig.Ragdoll.StopDirectedCollapseSpike());
                if (f == 45) rig.Call("BeginProfileDirectedCollapseSpike missing", () => rig.Ragdoll.BeginProfileDirectedCollapseSpike("missing"));
                if (f == 46) rig.Call("BeginProfileDirectedCollapseSpike golden-empty", () => rig.Ragdoll.BeginProfileDirectedCollapseSpike("golden-empty"));
                if (f == 50) rig.Call("BeginProfileDirectedCollapseSpike golden-directions", () => rig.Ragdoll.BeginProfileDirectedCollapseSpike("golden-directions"));
                if (f == 150) rig.Call("RequestProfileDirectedCollapseOnReady golden-sideways", () => rig.Ragdoll.RequestProfileDirectedCollapseOnReady("golden-sideways"));
                if (f == 160) rig.Call("BeginKneePowerLossForwardPattern", () => rig.Ragdoll.BeginKneePowerLossForwardPattern());
            });
        output.Summary = rig.Finals();
    }

    /// <summary>The kneel-pitch armed while waiting and run to its end; begun again with out-of-range values
    /// (clamped); the knee power loss stops it; a profile then runs alongside the knee power loss; deactivated in the
    /// middle of both; activated again with the guided collapse switch on.</summary>
    public static void KneelPitchChain(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.NarrowStraight, -0.5f, 0.5f, 60f, seed: 6);
        rig.Activate(0.05f);
        for (var f = 0; f < 300; f++)
            rig.RenderFrame(() =>
            {
                if (f == 1) rig.Call("RequestDirectedKneelPitchOnReady", () => rig.Ragdoll.RequestDirectedKneelPitchOnReady(0.8f, 6000f, 900f, 0.45f, 0.3f, 40f));
                if (f == 100) rig.Call("BeginDirectedKneelPitchSpike clamped", () => rig.Ragdoll.BeginDirectedKneelPitchSpike(5f, 20f, 12000f, 2f, 3f, -120f));
                if (f == 130) rig.Call("BeginKneePowerLossForwardPattern", () => rig.Ragdoll.BeginKneePowerLossForwardPattern());
                if (f == 140) rig.Call("BeginProfileDirectedCollapseSpike golden-sideways", () => rig.Ragdoll.BeginProfileDirectedCollapseSpike("golden-sideways"));
                if (f == 170) rig.Call("Deactivate", () => rig.Ragdoll.Deactivate());
                if (f == 172) rig.Settings.GuidedCollapse = true;
                if (f == 175) rig.Activate(0.05f);
            });
        output.Summary = rig.Finals();
    }

    /// <summary>A plain fall next to an external dynamic body until everything sleeps; then the knee power loss, the
    /// entry-conditioned knee power loss, a profile and the kneel-pitch begin one after the other, each when the rig
    /// is asleep again or 150 frames after the one before (the first one always on the sleeping rig, which it wakes).</summary>
    public static void AsleepWakeExternal(Output output)
    {
        using var rig = new Rig(output, Flat, Stance.WideBent, 0f, 0f, -20f, seed: 7);
        rig.Activate(0.05f);
        var starts = new (string What, Func<object?> Call)[]
        {
            ("BeginKneePowerLossForwardPattern", () => rig.Ragdoll.BeginKneePowerLossForwardPattern()),
            ("RequestEntryConditionedKneePowerLossOnReady", () => { rig.Ragdoll.RequestEntryConditionedKneePowerLossOnReady(); return "done"; }),
            ("BeginProfileDirectedCollapseSpike golden-sideways", () => rig.Ragdoll.BeginProfileDirectedCollapseSpike("golden-sideways")),
            ("BeginDirectedKneelPitchSpike", () => rig.Ragdoll.BeginDirectedKneelPitchSpike(0.6f, 3000f, 700f, 0.3f, 0.2f, 25f)),
        };
        var next = 0;
        var lastStart = -1;
        var startedAt = new List<string>();
        for (var f = 0; f < 900 && (next < starts.Length || f - lastStart < 150); f++)
        {
            rig.RenderFrame(() =>
            {
                if (f == 12)
                {
                    var parts = new[] { new Ragdoll.ExternalShapePart(new Vector3(0.15f, 0.1f, 0.2f), Vector3.Zero, Quaternion.Identity) };
                    rig.Call("TryCreateExternalDynamicBody", () =>
                    {
                        var ok = rig.Ragdoll.TryCreateExternalDynamicBody(parts, 4f, new Vector3(0.6f, 0.6f, 0.3f),
                            Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.4f), new Vector3(-0.3f, 0f, 0f), Vector3.Zero, out var handle);
                        return ok && handle != null;
                    });
                }
                if (next >= starts.Length || f <= 30) return;
                var asleep = rig.AllAsleep;
                if (asleep || (lastStart >= 0 && f - lastStart >= 150))
                {
                    startedAt.Add($"{f}{(asleep ? " asleep" : "")}");
                    output.Line($"start {next} at frame {f} asleep={asleep}");
                    var (what, call) = starts[next];
                    rig.Call(what, call);
                    next++;
                    lastStart = f;
                }
            });
        }
        output.Line($"starts at frames {string.Join(", ", startedAt)}");
        output.Summary = $"{rig.Finals()}; starts at frames {string.Join(", ", startedAt)}";
    }
}

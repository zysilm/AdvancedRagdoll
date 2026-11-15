// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
// Golden-output test of the dropped-weapon simulation. Each scenario drives WeaponDropSimulation through its internal
// host interfaces (Host.cs) for a fixed number of render frames and writes everything it does: log lines, every weapon
// transform it sets, and the state of every body of its simulation, floats as their exact bits. The output must be
// byte for byte the file in Golden/. See README.md.
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
using AdvancedRagdoll.WeaponDrop;
using BepuPhysics;
using BepuPhysics.Collidables;

var write = args.Contains("--write");
var outIndex = Array.IndexOf(args, "--out");
var outDir = outIndex >= 0 && outIndex + 1 < args.Length ? args[outIndex + 1] : null;
var goldenDir = FindGoldenDir();
if (goldenDir == null)
{
    Console.WriteLine("FAIL: tests/WeaponDropGolden/Golden not found");
    return 2;
}
Console.WriteLine($"Vector<float>.Count={Vector<float>.Count}, runtime {Environment.Version}, {System.Runtime.InteropServices.RuntimeInformation.OSDescription}");

var scenarios = new (string Name, Action<Output> Run)[]
{
    ("flat-resting-body", Scenarios.FlatRestingBody),
    ("slope-steps-bare-ground", Scenarios.SlopeStepsBareGround),
    ("mesh-weapon-mesh-body", Scenarios.MeshWeaponMeshBody),
    ("far-away-despawn", Scenarios.FarAwayDespawn),
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
        Console.WriteLine($"WROTE {name}: {output.LineCount} lines, {bytes.Length} bytes");
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
        Console.WriteLine($"IDENTICAL {name}: {output.LineCount} lines, {bytes.Length} bytes; {output.Summary}");
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
            var candidate = Path.Combine(dir.FullName, "tests", "WeaponDropGolden", "Golden");
            if (Directory.Exists(candidate)) return candidate;
            if (dir.Name == "WeaponDropGolden" && Directory.Exists(Path.Combine(dir.FullName, "Golden")))
                return Path.Combine(dir.FullName, "Golden");
        }
    return null;
}

/// <summary>The canonical text output: '\n' line ends, floats as the hex of their bits.</summary>
internal sealed class Output
{
    private readonly StringBuilder text = new();
    public int LineCount { get; private set; }
    public string Summary = "";

    public void Line(string line)
    {
        text.Append(line).Append('\n');
        LineCount++;
    }

    public byte[] ToBytes() => new UTF8Encoding(false).GetBytes(text.ToString());

    public static string F(float v) => BitConverter.SingleToUInt32Bits(v).ToString("X8");
    public static string V(Vector3 v) => $"{F(v.X)},{F(v.Y)},{F(v.Z)}";
    public static string Q(Quaternion q) => $"{F(q.X)},{F(q.Y)},{F(q.Z)},{F(q.W)}";
}

/// <summary>One scenario's world: the host, the character ragdoll the simulation borrows models from, the simulation.</summary>
internal sealed class Rig : IDisposable
{
    public readonly Output Output;
    public readonly SyntheticSkeletons Skeletons = new();
    public readonly SyntheticWorld World;
    public readonly SyntheticActors Actors = new();
    public readonly SyntheticWeapons Weapons;
    public readonly SyntheticSettings Settings = new();
    public readonly SyntheticMeshes Meshes = new();
    public readonly ManualClock Clock = new();
    public readonly Ragdoll Ragdoll;
    public readonly WeaponDropSimulation Simulation;
    public int Frame;

    private static readonly FieldInfo SimulationField = typeof(WeaponDropSimulation)
        .GetFields(BindingFlags.Instance | BindingFlags.NonPublic)
        .Single(f => f.FieldType == typeof(BepuPhysics.Simulation));

    public Rig(Output output, Func<float, float, float?> ground, bool fineDetail, bool weaponMesh, bool bodyMesh)
    {
        Output = output;
        World = new SyntheticWorld(ground);
        Weapons = new SyntheticWeapons(output);
        Settings.FineDetail = fineDetail;
        Settings.WeaponMesh = weaponMesh;
        Settings.BodyMesh = bodyMesh;
        Ragdoll = new Ragdoll(new RagdollHostServices
        {
            Skeletons = Skeletons,
            World = World,
            Actors = Actors,
            Colliders = new NoColliders(),
            Settings = Settings,
            Log = new OutputLog(output, "R"),
            Clock = Clock,
            Meshes = Meshes,
            Build = new RagdollBuildInfo(false, true, "golden"),
        }, NoCollapseProfiles.Instance, TestHumanoid.Profile);
        Simulation = new WeaponDropSimulation(new WeaponDropHost
        {
            Skeletons = Skeletons,
            World = World,
            Actors = Actors,
            Weapons = Weapons,
            Settings = Settings,
            Log = new OutputLog(output, "W"),
            Profile = TestHumanoid.Profile,
            Clock = Clock,
        }, Ragdoll);
    }

    /// <summary>One render frame of <paramref name="seconds"/>: the clock advances, the simulation runs, the output
    /// gets the frame's events and (every frame for dynamic bodies, every <paramref name="allBodiesEvery"/> frames for
    /// all of them) the body states.</summary>
    public void RenderFrame(float seconds = 1f / 60f, int allBodiesEvery = 4)
    {
        Output.Line($"F {Frame}");
        Clock.Now += (long)MathF.Round(seconds * Clock.Frequency);
        var rays = World.Rays;
        Simulation.OnRenderFrame();
        if (World.Rays != rays) Output.Line($"rays {World.Rays - rays}");
        DumpBodies(Frame % allBodiesEvery == 0);
        Frame++;
    }

    private void DumpBodies(bool all)
    {
        if (SimulationField.GetValue(Simulation) is not BepuPhysics.Simulation sim) return;
        var bodies = sim.Bodies;
        var highest = bodies.HandlePool.HighestPossiblyClaimedId;
        var active = bodies.ActiveSet.Count;
        if (all) Output.Line($"S bodies<={highest + 1} active={active} statics={sim.Statics.Count}");
        for (var h = 0; h <= highest; h++)
        {
            var handle = new BodyHandle(h);
            if (!bodies.BodyExists(handle)) continue;
            var body = bodies.GetBodyReference(handle);
            var kinematic = body.Kinematic;
            if (kinematic && !all) continue;
            var pose = body.Pose;
            var velocity = body.Velocity;
            Output.Line($"B {h} {(kinematic ? 'k' : 'd')}{(body.Awake ? 'a' : 's')} p={Output.V(pose.Position)} q={Output.Q(pose.Orientation)} " +
                        $"v={Output.V(velocity.Linear)} w={Output.V(velocity.Angular)}");
        }
    }

    /// <summary>Readable final weapon positions (for the console; the bits are in the output).</summary>
    public string Finals(params nint[] weapons) => string.Join(", ", weapons.Select(w =>
        Weapons.Roots.TryGetValue(w, out var r) ? $"0x{w:X} at ({r.Position.X:F3}, {r.Position.Y:F3}, {r.Position.Z:F3})" : $"0x{w:X} -"));

    public void Dispose()
    {
        Simulation.Dispose();
        Ragdoll.Dispose();
    }
}

internal static class Scenarios
{
    // A 23-bone humanoid (about 1.7 m), model space, Y up, facing +Z.
    private static readonly string[] Names =
    [
        "root", "pelvis", "spine_a", "spine_b", "spine_c", "neck", "head",
        "clavicle_l", "arm_a_l", "arm_b_l", "hand_l", "clavicle_r", "arm_a_r", "arm_b_r", "hand_r",
        "leg_a_l", "leg_b_l", "leg_d_l", "leg_e_l", "leg_a_r", "leg_b_r", "leg_d_r", "leg_e_r",
    ];
    private static readonly short[] Parents = [-1, 0, 1, 2, 3, 4, 5, 4, 7, 8, 9, 4, 11, 12, 13, 1, 15, 16, 17, 1, 19, 20, 21];
    private static readonly Vector3[] Standing =
    [
        new(0, 0, 0), new(0, 1.0f, 0), new(0, 1.12f, 0), new(0, 1.28f, 0), new(0, 1.42f, 0), new(0, 1.55f, 0), new(0, 1.65f, 0),
        new(0.05f, 1.48f, 0), new(0.18f, 1.45f, 0), new(0.45f, 1.42f, 0.02f), new(0.70f, 1.40f, 0.05f),
        new(-0.05f, 1.48f, 0), new(-0.18f, 1.45f, 0), new(-0.45f, 1.42f, 0.02f), new(-0.70f, 1.40f, 0.05f),
        new(0.1f, 0.95f, 0), new(0.1f, 0.52f, 0.02f), new(0.1f, 0.08f, 0), new(0.1f, 0.02f, 0.12f),
        new(-0.1f, 0.95f, 0), new(-0.1f, 0.52f, 0.02f), new(-0.1f, 0.08f, 0), new(-0.1f, 0.02f, 0.12f),
    ];

    private static SyntheticSkeleton Humanoid(Rig rig, nint actor, Vector3 position, Quaternion rotation)
    {
        var skeleton = new SyntheticSkeleton(actor, Names, Parents, Standing)
        {
            RootTransform = new RootTransform(position, rotation, Vector3.One),
        };
        rig.Skeletons.ByActor[actor] = skeleton;
        return skeleton;
    }

    /// <summary>A weapon's own skeleton: bones along +Y spanning <paramref name="length"/>.</summary>
    private static void WeaponSkeleton(Rig rig, nint weapon, int bones, float length)
    {
        var names = new string[bones];
        var parents = new short[bones];
        var positions = new Vector3[bones];
        for (var i = 0; i < bones; i++)
        {
            names[i] = "w" + i;
            parents[i] = (short)(i - 1);
            positions[i] = new Vector3(0.01f * i, bones == 1 ? 0 : length * i / (bones - 1), 0);
        }
        rig.Weapons.Skeletons[weapon] = new SyntheticSkeleton(weapon, names, parents, positions);
    }

    private static void Weapon(Rig rig, nint actor, DrawnWeaponSlot slot, nint weapon, Vector3 position, Quaternion rotation)
    {
        rig.Weapons.Slots[(actor, slot)] = weapon;
        rig.Weapons.Roots[weapon] = (position, rotation);
    }

    private static Quaternion Axis(float x, float y, float z, float degrees) =>
        Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(x, y, z)), degrees * MathF.PI / 180f);

    /// <summary>Flat ground; an NPC lies on its back; a long sword and a one-bone off-hand weapon are released above
    /// its chest and fall onto the segment capsules of its body, then onto the ground.</summary>
    public static void FlatRestingBody(Output output)
    {
        using var rig = new Rig(output, (x, z) => 0f, fineDetail: false, weaponMesh: false, bodyMesh: false);
        const nint actor = 0x1000;
        Humanoid(rig, actor, new Vector3(0, 0.12f, 0), Axis(1, 0, 0, -90));
        Weapon(rig, actor, DrawnWeaponSlot.MainHand, 0x2001, new Vector3(0.15f, 0.7f, -1.3f), Axis(1, 0, 0.3f, 70));
        WeaponSkeleton(rig, 0x2001, 3, 1.1f);
        Weapon(rig, actor, DrawnWeaponSlot.OffHand, 0x2002, new Vector3(-0.4f, 0.5f, -0.8f), Axis(0, 0, 1, 30));
        WeaponSkeleton(rig, 0x2002, 1, 0f);
        for (var f = 0; f < 200; f++)
        {
            if (f == 3) rig.Simulation.SpawnFor(actor);
            if (f == 4) rig.Simulation.SpawnFor(actor); // already spawned: ignored
            rig.RenderFrame();
        }
        output.Summary = rig.Finals(0x2001, 0x2002);
    }

    /// <summary>Sloped and stepped ground; a standing NPC shuffles sideways and drops its arms while its weapons fall
    /// clear of it onto bare ground: one without a skeleton of its own, one with a tiny scaled skeleton.</summary>
    public static void SlopeStepsBareGround(Output output)
    {
        static float? Ground(float x, float z) => 0.3f * x + MathF.Floor(z / 0.75f) * 0.12f;
        using var rig = new Rig(output, Ground, fineDetail: false, weaponMesh: false, bodyMesh: false);
        const nint actor = 0x1100;
        var skeleton = Humanoid(rig, actor, new Vector3(0, Ground(0, 0)!.Value, 0), Axis(0, 1, 0, 40));
        Weapon(rig, actor, DrawnWeaponSlot.MainHand, 0x2101, new Vector3(1.6f, 1.4f, 0.9f), Axis(0.2f, 1, 0.5f, 110));
        Weapon(rig, actor, DrawnWeaponSlot.OffHand, 0x2102, new Vector3(-1.4f, 1.0f, -1.2f), Axis(1, 0.1f, 0, 15));
        WeaponSkeleton(rig, 0x2102, 4, 0.12f);
        rig.Weapons.Scales[0x2102] = new Vector3(0.7f);
        var hand = skeleton.IndexOf("hand_r");
        var forearm = skeleton.IndexOf("arm_b_r");
        for (var f = 0; f < 240; f++)
        {
            if (f == 1) rig.Simulation.SpawnFor(actor);
            if (f is >= 10 and < 40)
            {
                var t = (f - 9) / 30f;
                skeleton.RootTransform = skeleton.RootTransform with { Position = new Vector3(-0.4f * t, Ground(-0.4f * t, 0)!.Value, 0.05f * t) };
                skeleton.SetBone(forearm, Vector3.Lerp(Standing[forearm], new Vector3(-0.25f, 1.15f, 0.05f), t));
                skeleton.SetBone(hand, Vector3.Lerp(Standing[hand], new Vector3(-0.28f, 0.9f, 0.1f), t));
            }
            rig.RenderFrame();
        }
        output.Summary = rig.Finals(0x2101, 0x2102);
    }

    /// <summary>The local player (detailed contacts): a mesh-fitted sword and a weapon whose model cannot be read
    /// (falls back to a box) drop onto a raised floor and onto the mesh surface of the body lying there; the right arm
    /// pulls away from the sword over the first frames; every fifth frame is slow (one half step only).</summary>
    public static void MeshWeaponMeshBody(Output output)
    {
        using var rig = new Rig(output, (x, z) => 0.5f, fineDetail: false, weaponMesh: true, bodyMesh: true);
        const nint actor = 0x1200;
        rig.Actors.Local = actor;
        rig.Actors.DrawScale[actor] = new Vector3(1.05f);
        var lying = Quaternion.Concatenate(Axis(1, 0, 0, -90), Axis(0, 1, 0, 20));
        var skeleton = Humanoid(rig, actor, new Vector3(0.2f, 0.62f, 0.1f), lying);
        rig.Meshes.BodySurfaces[actor] =
        [
            ("pelvis", new Vector3(0, 0.05f, 0), new Vector3(0.16f, 0.1f, 0.11f)),
            ("spine_b", new Vector3(0, 0.07f, 0), new Vector3(0.15f, 0.08f, 0.1f)),
            ("spine_c", new Vector3(0, 0.06f, 0), new Vector3(0.17f, 0.08f, 0.11f)),
            ("head", new Vector3(0, 0.1f, 0.01f), new Vector3(0.09f, 0.11f, 0.1f)),
            ("arm_a_r", new Vector3(-0.13f, 0, 0), new Vector3(0.13f, 0.05f, 0.05f)),
            ("arm_b_r", new Vector3(-0.12f, 0, 0), new Vector3(0.12f, 0.04f, 0.04f)),
            ("hand_r", new Vector3(-0.06f, 0, 0), new Vector3(0.06f, 0.02f, 0.045f)),
            ("leg_a_l", new Vector3(0, -0.2f, 0), new Vector3(0.08f, 0.21f, 0.08f)),
            ("leg_a_r", new Vector3(0, -0.2f, 0), new Vector3(0.08f, 0.21f, 0.08f)),
        ];
        var sword = new List<Triangle>();
        SyntheticMeshes.AddBoxTriangles(sword, new Vector3(0, 0.6f, 0), new Vector3(0.025f, 0.45f, 0.006f));
        SyntheticMeshes.AddBoxTriangles(sword, new Vector3(0, 0.14f, 0), new Vector3(0.12f, 0.02f, 0.02f));
        SyntheticMeshes.AddBoxTriangles(sword, new Vector3(0, 0.05f, 0), new Vector3(0.015f, 0.08f, 0.015f));
        rig.Meshes.WeaponTriangles[0x2201] = sword;
        Weapon(rig, actor, DrawnWeaponSlot.MainHand, 0x2201, new Vector3(-0.35f, 1.0f, -1.1f), Axis(1, 0.2f, 0, -80));
        Weapon(rig, actor, DrawnWeaponSlot.OffHand, 0x2202, new Vector3(0.45f, 0.95f, -0.9f), Axis(0, 0, 1, 60));
        WeaponSkeleton(rig, 0x2202, 2, 0.5f);
        var hand = skeleton.IndexOf("hand_r");
        var forearm = skeleton.IndexOf("arm_b_r");
        for (var f = 0; f < 200; f++)
        {
            if (f == 2) rig.Simulation.SpawnFor(actor);
            if (f is >= 3 and < 18)
            {
                var t = (f - 2) / 15f;
                skeleton.SetBone(forearm, Vector3.Lerp(Standing[forearm], new Vector3(-0.42f, 1.2f, 0.25f), t), Axis(0, 0, 1, 25 * t));
                skeleton.SetBone(hand, Vector3.Lerp(Standing[hand], new Vector3(-0.5f, 1.0f, 0.45f), t), Axis(0, 0, 1, 40 * t));
            }
            rig.RenderFrame(f % 5 == 4 ? 1f / 20f : 1f / 60f);
        }
        output.Summary = rig.Finals(0x2201, 0x2202);
    }

    /// <summary>Far from the origin, where no ground ray hits (flat fallback ground). The owner vanishes while its
    /// sword falls (its body colliders are parked below the world), then its weapons are removed; a second character
    /// drops a sword next to it; characters without weapons or without a skeleton are released too; another
    /// controller takes the second character over; at the end everything is removed.</summary>
    public static void FarAwayDespawn(Output output)
    {
        using var rig = new Rig(output, (x, z) => null, fineDetail: false, weaponMesh: false, bodyMesh: false);
        var origin = new Vector3(1800f, 40f, -2600f);
        Humanoid(rig, 0x1300, origin, Axis(0, 1, 0, -120));
        Weapon(rig, 0x1300, DrawnWeaponSlot.MainHand, 0x2301, origin + new Vector3(0.6f, 1.3f, 0.2f), Axis(0.3f, 0.2f, 1, 50));
        WeaponSkeleton(rig, 0x2301, 3, 0.9f);
        Humanoid(rig, 0x1400, origin + new Vector3(1.0f, 0, 0.5f), Axis(0, 1, 0, 15));
        Weapon(rig, 0x1400, DrawnWeaponSlot.MainHand, 0x2401, origin + new Vector3(1.5f, 1.2f, 0.4f), Axis(1, 0, 0, 85));
        WeaponSkeleton(rig, 0x2401, 2, 1.3f);
        Humanoid(rig, 0x1500, origin + new Vector3(-2f, 0, 0), Quaternion.Identity);
        IDisposable? external = null;
        for (var f = 0; f < 220; f++)
        {
            if (f == 2) rig.Simulation.SpawnFor(0x1300);
            if (f == 50) rig.Skeletons.Vanished.Add(0x1300);
            if (f == 120) rig.Simulation.RemoveFor(0x1300);
            if (f == 125) rig.Simulation.SpawnFor(0x1400);
            if (f == 126) { rig.Simulation.SpawnFor(0x1500); rig.Simulation.SpawnFor(0x1600); rig.Simulation.SpawnFor(0); }
            if (f == 170)
            {
                external = rig.Simulation.AcquireExternalControl(0x1400);
                output.Line($"X acquired={external != null} again={rig.Simulation.AcquireExternalControl(0x1400) != null}");
                rig.Simulation.SpawnFor(0x1400); // owned by the other controller: ignored
            }
            if (f == 180) { external?.Dispose(); rig.Simulation.SpawnFor(0x1400); }
            if (f == 210) rig.Simulation.RemoveAll();
            rig.RenderFrame();
            if (f is 0 or 60 or 130 or 175 or 215)
                output.Line($"H 0x1300={rig.Simulation.HasEntry(0x1300)} 0x1400={rig.Simulation.HasEntry(0x1400)} 0x1500={rig.Simulation.HasEntry(0x1500)}");
        }
        output.Summary = rig.Finals(0x2301, 0x2401);
    }
}

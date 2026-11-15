# Guided collapse and knee power loss: golden output

The guided collapse (`Ragdoll.GuidedCollapse.cs`: the directed collapse, as a kneel-pitch or as a collapse profile, and
the entry conditioning) and the knee power loss (`Ragdoll.KneePowerLoss.cs`) are not exercised by any of the ragdoll
recordings. This test pins their behaviour: it compiles every library source file (so it sees internal types), drives
`Ragdoll` through its internal host interfaces with a synthetic host (`Host.cs`) and a synthetic humanoid
(`Humanoid.cs`), and compares the complete output with the files in `Golden/` byte for byte.

```
dotnet run -c Release                 # compare with Golden/ (exit code 0 = every scenario identical)
dotnet run -c Release -- --out <dir>  # also write this run's output to <dir>
dotnet run -c Release -- --write      # rewrite Golden/ from the current code
```

Rewrite the golden files only for an intended change of the collapse behaviour, and say so in the commit. The golden
files in this directory were written from the library at 932fa5d, before the guided collapse and the knee power loss
became components (stage A6-4).

The harness calls only members that existed before that stage (the recorded entry points in `Ragdoll.Recording.cs`,
`DirectedCollapseSpikeActive`, `KneePowerLossPatternActive`, `IsSimulationReady`, `GetBodyWorldPosition`) and reads the
simulation through the private `simulation` property by reflection, so the same harness runs on both versions.

## The humanoid

A 28-bone skeleton (22 bodies) on the plain bone names of `tests/Fixtures/TestHumanoid.cs`, standing in model space
(Y up, facing +Z) in one of three stances: narrow with straight knees (the entry conditioning is needed), wide with
bent knees (it is skipped), and staggered. Its profile has a full bone table in the shape of a real humanoid table:
a spine chain, clavicles, arms with an elbow hinge, legs with two knee hinges and a foot, ranges of motion, a mass
budget and anatomical roles. Nothing is read from a model (no mesh source); the ground is a height function; the
random source is seeded and writes every draw; background work runs inline.

## Scenarios

Each runs a fixed number of render frames of 1/60 s on a manual clock, the character activated with a 0.05 s delay:

- `control-plain`: a plain activation on flat ground, no collapse.
- `entry-knee-settings`: the guided collapse switch on: the activation arms the entry-conditioned knee power loss
  (pending until physics is ready); the entry conditioning runs and hands over; the knee power loss runs its buckle,
  torso loss and release.
- `entry-skipped-raised`: the switch on, raised ground, wide stance: the entry conditioning is skipped.
- `entry-grab-knee-stops`: on a slope: the entry-conditioned knee power loss requested while ready; a grab cuts the
  entry conditioning short; the knee power loss requested while ready, stopped in its buckle, begun again and stopped
  six frames into its torso loss.
- `knee-pending-reactivate`: on stepped ground: the knee power loss requested while waiting (armed), run to its end;
  deactivated and activated again without it.
- `profile-sideways-seed1`, `profile-sideways-seed2`: a three-phase profile (foot plant, sideways pelvis drop with a
  random side every frame, forward fold, torso pitch, release to passive with a controller after the release),
  requested while waiting (under an id in another case) and while ready; two seeds, two grounds, two stances.
- `profile-directions-stop`: on raised ground: a profile with every direction, bone overrides, a foot list with a
  missing and a side-less bone, an empty phase, an unknown controller type, an inverted window and a phase shorter than
  a frame: stopped in its third phase; a missing profile and one without phases do not begin; the profile again, to
  its natural end; another profile, cut short by the knee power loss.
- `kneel-pitch-chain`: the kneel-pitch armed while waiting and run to its end; begun again with out-of-range values
  (clamped); the knee power loss stops it; a profile then runs alongside the knee power loss; deactivated in the middle
  of both; activated again with the switch on.
- `asleep-wake-external`: a plain fall next to an external dynamic body until every body sleeps; the knee power loss
  begins on the sleeping rig (and wakes it); then the entry-conditioned knee power loss, a profile and the kneel-pitch,
  each 150 frames after the one before.

## Output

One line per event, `\n` line ends, every float as the hex of its bits:

- `F <n>`: a render frame begins;
- `RI|RW|RE <text>`: a log line of the ragdoll (info, warning, error);
- `A <call>` and `A <call> -> <result>`: a call the scenario makes and its result;
- `Z <site> ...`: a random draw;
- `rays <n>`: ground rays asked in the frame;
- `C ready= directed= knee=`: the collapse state, when it changed;
- `B <handle> <d|k><a|s> p= q= v= w=`: a body (dynamic or kinematic, awake or asleep; pose, linear and angular
  velocity): every awake body every frame while a collapse runs and for 45 frames after any call, every tenth frame
  otherwise; all bodies every twentieth frame, after an `S bodies<= active= statics=` line;
- `H <hash> awake=<n> constraints=<n>`: every frame, a hash of the exact bits of every body (handle, kind, sleep
  state, pose, velocities), the awake bodies and the solver's constraint count.

On a difference the test prints the scenario, the first differing line with its frame, and both versions of the line.

## Determinism

Two runs give the same bytes. The output is bit-exact only on the same OS family, .NET runtime and BepuPhysics version:
the golden files were written on Windows x64 with .NET 10, and CI runs this test on Windows. `.gitattributes` keeps
the golden files' line ends untouched.

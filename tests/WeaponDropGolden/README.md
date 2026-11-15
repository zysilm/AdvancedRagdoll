# Weapon drop: golden output

The dropped-weapon simulation (`src/AdvancedRagdoll/WeaponDrop/`) is not covered by the ragdoll recordings. This test
pins its behaviour: it compiles every library source file (so it sees internal types), drives
`WeaponDropSimulation` through its internal host interfaces with a synthetic host (`Host.cs`), and compares the
complete output with the files in `Golden/` byte for byte.

```
dotnet run -c Release                 # compare with Golden/ (exit code 0 = every scenario identical)
dotnet run -c Release -- --out <dir>  # also write this run's output to <dir>
dotnet run -c Release -- --write      # rewrite Golden/ from the current code
```

Rewrite the golden files only for an intended change of the simulation's behaviour, and say so in the commit.

## Scenarios

Each runs a fixed number of render frames on a manual clock (60 Hz unless noted):

- `flat-resting-body`: flat ground; a character lies on its back; a long sword (its own three-bone skeleton sizes the
  box) and a one-bone off-hand weapon are released above its chest, fall onto its segment capsules and the ground. A
  second release of the same character is ignored.
- `slope-steps-bare-ground`: ground sloped along x and stepped along z; a standing character shuffles sideways and
  drops its arm (moving body colliders) while its weapons fall clear of it onto bare ground: one with no skeleton of
  its own (default length), one with a tiny skeleton drawn at 0.7 scale.
- `mesh-weapon-mesh-body`: the local player, so the detailed contacts apply: a sword whose hulls are fitted to its
  mesh (`WeaponMeshGeometry.Fit` on synthetic triangles), a weapon whose model cannot be read (box fallback, with a
  warning), a raised floor, a body lying on it with box surfaces as its mesh colliders, the arm pulling away from the
  sword (release contacts); every fifth frame is slow (one half step instead of two).
- `far-away-despawn`: far from the origin, where no ground ray hits (flat fallback ground). The owner vanishes while
  its sword falls (its body colliders are parked below the world), then its weapons are removed; a second character
  drops a sword; characters with no weapon, with no skeleton, and the id 0 are released; another controller takes the
  second character over (its release is ignored), gives it back, and at the end everything is removed.

## Output

One line per event, `\n` line ends, every float as the hex of its bits:

- `F <n>`: a render frame begins;
- `WI|WW|WE <text>`: a log line of the simulation (info, warning, error); `RI|RW|RE`: one of the character ragdoll it
  borrows models from;
- `T <weapon> p=<x,y,z> q=<x,y,z,w>`: the simulation moved a drawn weapon;
- `rays <n>`: ground rays asked in the frame;
- `S bodies<=<n> active=<n> statics=<n>` and `B <handle> <d|k><a|s> p= q= v= w=`: the simulation's bodies (dynamic or
  kinematic, awake or asleep; pose, linear and angular velocity). Dynamic bodies every frame, all bodies every fourth
  frame;
- `X`, `H`: results of external control and of `HasEntry` at fixed frames.

On a difference the test prints the scenario, the first differing line with its frame, and both versions of the line.

## Determinism

Two runs give the same bytes. The output is bit-exact only on the same OS family, .NET runtime and BepuPhysics version:
the golden files were written on Windows x64 with .NET 10, and CI runs this test on Windows. `.gitattributes` keeps
the golden files' line ends untouched.

# Contact paths: golden output

Most contact paths of the ragdoll are exercised by few ragdoll recordings or none: the standing support and the wrist
constraints, external bodies and rigs (and carrying the worn rigs with the standing hold), the grab with several slots
and the grabber's parked collision, the landing weight (`Ragdoll.Impacts.cs`), the strike window and the direct pokes
(`Ragdoll.Strikes.cs`). This test pins their behaviour: it compiles every library source file (so it sees internal
types), drives `Ragdoll` through its internal host interfaces with a synthetic host (`Host.cs`) and a synthetic
humanoid (`Humanoid.cs`, a copy of the collapse golden test's), and compares the complete output with the files in
`Golden/` byte for byte.

```
dotnet run -c Release                 # compare with Golden/ (exit code 0 = every scenario identical)
dotnet run -c Release -- --out <dir>  # also write this run's output to <dir>
dotnet run -c Release -- --write      # rewrite Golden/ from the current code
```

Rewrite the golden files only for an intended change of one of these paths, and say so in the commit. The golden files
in this directory were written from the library at 96ceef1 (stage A6-1), before the standing support, the grab, the
impacts, the strikes and the external bodies became components (stages A6-2 to A6-5).

The harness calls only internal members that existed before those stages (the recorded entry points in
`Ragdoll.Recording.cs`, `GrabRigid`, `CarryExternalRigOnAnchor`, `HardLanding`, `TryGetExternalBodyPose`,
`TryGetExternalRigBodyPose`, `TryResolveGrabAttachment`, `GetSimulatedBoneNames`, `NearestBodyDistance`,
`IsStandingSupportActive`, `IsSimulationReady`, `GetBodyWorldPosition`). It reads the simulation through the private
`simulation` property, and a few private members: the impact timers and descent speed, the impact jitter state, the
strike window, the grab slots and the parked grabber, the recoil relaxers, the standing and wrist constraints, the
carried rigs, the external bodies and rigs, the settle timer and the resting flag. The output prints each under the
name of the ragdoll's field before stage A6-2; the harness reads it where it lives since the split (a member of a
component, or of the shared context, under the ragdoll's private field of that component, for example
`impacts.FreezeRemaining`), and from the ragdoll's field of that name on a library without that field (before stage
A6-2). So the same harness runs on both versions.

## The strike window is forced

No internal call opens the strike window: `attackStrikeTimer` and `strikePower` are only ever reset (on activation and
by `CancelAttackStrike`), never set above zero, in the library. `strike-window-forced` writes those two private members
itself, so the bone strikes and the weapon sweep that read them run; everything else in that scenario goes through the
internal calls. The multi-source strike set (`strikeColliderAddresses`) is only filled by the replay entry point
`RestoreCarriedState` and is not covered. The impact jitter generator (`NextImpactJitter`, later `Impacts.NextJitter`)
has no caller; its state is written in the output but never changes.

## The host

A 28-bone skeleton (22 bodies) on the plain bone names of `tests/Fixtures/TestHumanoid.cs` (see the collapse golden
test), activated with a 0.05 s delay; a second character of the same skeleton stands in front of it in the grab and
strike scenarios (registered with `AddLiveCollider`; the scenario moves its root and its right arm). The host answers a
bone's world position from the skeleton (a grab on a bone without a body asks for it) and the second character's
weapon transform from its right hand. The ground is a height function; the random source is seeded and writes every
draw; background work runs inline; the settings are all off except the character-collision size.

## Scenarios

Each runs a fixed number of render frames of 1/60 s on a manual clock:

- `control-fall`: a plain activation on flat ground.
- `standing-worn-rigs-wrists`: a worn garment rig (ball-socket joints with swing limits, a pose guide and a joint
  friction, a distance limit), a dropped garment rig that does not touch the ragdoll, and a dynamic body; the standing
  support (refused before physics), its anchor moved (pelvis, then `spine_b`) with the owner's carry test choosing the
  worn rig, then no test, then every rig; the wrist constraints created, moved, replaced and removed; garment edits and
  an impulse during the hold; removal; contact switched on and off for single rig bodies; a rig removed twice; a second
  hold that lifts the settled corpse.
- `external-bodies-rigs`: on a slope: bodies created (refused before physics and with no parts), a compound spinning
  body, a weapon-contact body attached and detached; a four-panel cape (skipped bad joints) and a rig refused with its
  partial bodies cleaned up; every shape, rotation, velocity and damping edit on bodies and rig bodies, poses read;
  removal, and calls on removed and null handles; a deactivation with bodies alive, then calls on the stale handles and
  a new body after reactivating.
- `grab-slots-parking`: a grab before physics (refused); the second character registered as a live collider; the
  standing support, taken over by the grabs; a grab on a bone without a body (carried by its parent at an offset), a
  rigid grab and a soft grab by the second character (its collision parked while held), a grab on a missing bone; the
  standing support and the wrist constraints refused while held; the grip followed as the grabber walks back; retuning
  (also with non-finite values and an unknown slot); release one slot at a time; the live collider removed; a second
  rigid grab without a grabber.
- `landing-cooldown`: a fall from 1 m lying flat: the landing (hitstop, follow-through release, event), a punt during
  the hitstop, a throw that comes down inside the cooldown (no landing), a bigger throw after it (a second landing).
- `landing-slam-raised`: a fall from 3.5 m onto raised ground: a slam (the heave); the standing support lifts the
  corpse 1.2 m (a held body does not land; the cooldown does not run) and lets it go.
- `pokes-recoil-shake`: on stepped ground: pokes before physics (ignored); impulses, angular kicks with their recoil
  relax (the same bone twice; a bone without a motor; a missing bone), punts (a hit, a miss, one straight above),
  shakes; the same once every body sleeps.
- `strike-window-forced`: the second character's arm swings through the corpse with a weapon in its hand; strike
  windows (forced, see above): plain, cancelled; with the collider lightweight; during the standing support (no
  strike); with the collider a traversal ghost; after the collider is removed; a deactivation.
- `teardown-mid-contact`: deactivations in the middle of the standing support with wrist constraints, a carried worn
  rig, a recoil relax and an open strike window, and later in the middle of two grabs (the grabber parked); after each
  reactivation the state that carries over (grab slot ids, a release of a slot of the earlier activation).

## Output

One line per event, `\n` line ends, every float as the hex of its bits:

- `F <n>`: a render frame begins;
- `RI|RW|RE <text>`: a log line of the ragdoll (info, warning, error);
- `A <call>` and `A <call> -> <result>`: a call the scenario makes and its result (`A force <member>=<value>`: a
  private member the scenario writes);
- `E HardLanding speed=`: the landing event;
- `Z <site> ...`: a random draw;
- `W <actor> p= q=`: a weapon transform the ragdoll asked for;
- `carry? <rig> -> <answer>`: the owner's carry test, asked by the ragdoll;
- `X <label> ok= p= q= v= w=`: an external body's pose, as the ragdoll reports it;
- `rays <n>`: ground rays asked in the frame;
- `C ready= standing=`: the ragdoll's state, when it changed;
- `P <member>=<value> ...`: the private members listed above, when any changed (sets of handles and addresses by
  member, other collections by size);
- `B <handle> <d|k><a|s> p= q= v= w=`: a body (dynamic or kinematic, awake or asleep; pose, linear and angular
  velocity): every awake dynamic body for 30 frames after any call and every tenth frame; all bodies every twentieth
  frame, after an `S bodies<= active= statics=` line;
- `H <hash> awake=<n> constraints=<n>`: every frame, a hash of the exact bits of every body (handle, kind, sleep
  state, pose, velocities), the awake bodies and the solver's constraint count.

On a difference the test prints the scenario, the first differing line with its frame, and both versions of the line.

## Determinism

Two runs give the same bytes. The output is bit-exact only on the same OS family, .NET runtime and BepuPhysics version:
the golden files were written on Windows x64 with .NET 10, and CI runs this test on Windows. `.gitattributes` keeps
the golden files' line ends untouched.

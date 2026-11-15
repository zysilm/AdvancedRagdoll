// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Numerics;
using System.Reflection;
using System.Runtime.InteropServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using AdvancedRagdoll.Hair;
using AdvancedRagdoll.MeshVolume;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.Recording;
using BepuSimulation = BepuPhysics.Simulation;

namespace AdvancedRagdoll;

// Read-only recorder for one ragdoll activation (docs/ARCHITECTURE.md §6).
//
// Everything here only observes. The recorder's writer is null unless a recording session was on when the ragdoll
// was activated, so each call site in the controller costs one null check. The helpers below read
// the same game memory the surrounding code reads at that moment, never throw into the caller, and
// write records in the order things happen. The format is described in
// Recording/RagdollRecordingFormat.cs.
//
// The recording session and the records that only need what is handed in (or the context) belong to the Recorder
// component, reachable from the context (SimulationContext.Recorder). The records that read the state of other
// subsystems (the Activate record, the frame and rig records) and the recorded API calls stay in the ragdoll's own
// methods below, which write them through the recorder's writer.
public unsafe partial class Ragdoll
{
    // a6-9: the recording session (Recorder, below).
    private readonly Recorder recorder;
    // The recording session's state, under its old names, for the ragdoll's recorded layer (and the recording
    // decorators made in the constructor).
    private RagdollRecordingWriter? rec => recorder.Writer;
    private int recDepth => recorder.Depth;
    private Dictionary<object, int> recMeshCaches => recorder.MeshCaches;

    /// <summary>
    /// The recording session of this ragdoll: the file and writer of the activation being recorded (null when none),
    /// the nesting depth of recorded calls, the frame and step counters, the structural fit outcome, the configuration
    /// watches, and what was already written (skeleton topologies, the last poses, mesh caches). It writes the
    /// records that need only what is handed in and the context; it never reads another component's state.
    /// </summary>
    private sealed class Recorder
    {
        private static int nextControllerId;
        private static readonly ConcurrentDictionary<Type, FieldInfo[]> FieldCache = new();

        private readonly SimulationContext ctx;
        internal Recorder(SimulationContext ctx) { this.ctx = ctx; }

        internal readonly int ControllerId = System.Threading.Interlocked.Increment(ref nextControllerId);
        /// <summary>The writer of the activation being recorded; null when nothing is recorded.</summary>
        internal RagdollRecordingWriter? Writer;
        internal RagdollRecordingSession.RecordingFile? File;
        // 0 = a call from the rest of the plugin; more = made while a frame or another recorded call runs.
        internal int Depth;
        private int renderFrame;
        internal int PhysicsFrame;
        internal int Tick;
        /// <summary>The recorded outcome of the structural fit (1 none, 2 not ready, 3 fitted, 4 failed).</summary>
        internal int FitOutcome { get; private set; }
        /// <summary>The structural fit outcome of this rig build (0 at its start).</summary>
        internal void NoteFitOutcome(int outcome) => FitOutcome = outcome;
        private bool faultLogged;
        // One watch per entry of ConfigNames. The bound getters are shared by every controller (built with
        // the first recording); each recorded activation gets its own copies, which remember what it last saw.
        private static RagdollRecordingWatch?[]? configWatchPrototypes;
        private RagdollRecordingWatch?[] configWatches = Array.Empty<RagdollRecordingWatch?>();
        // Created with the first recording, so a controller that is never recorded carries nothing.
        private Dictionary<(ulong Actor, int Partial), (nint Skeleton, int Bones)> topologies = null!;
        private Dictionary<(ulong Actor, int Purpose, int Partial), byte[]> lastPose = null!;
        internal Dictionary<object, int> MeshCaches = null!;
        private int[] handles = Array.Empty<int>();
        private byte[] flags = Array.Empty<byte>();
        private float[] states = Array.Empty<float>();

        /// <summary>True while a recording session writes this activation.</summary>
        internal bool IsRecording => Writer != null;

        /// <summary>Every configuration property the ragdoll code reads (Ragdoll*.cs), by name.</summary>
        private static readonly string[] ConfigNames =
        {
            nameof(HostSettings.DismemberPocBones),
            nameof(HostSettings.EnableCombatCompanions),
            nameof(HostSettings.EnableNpcDeathRagdoll),
            nameof(HostSettings.ExtendTerrainDetection),
            nameof(HostSettings.GuidedCollapse),
            nameof(HostSettings.KoStripPhysicsDropClothing),
            nameof(HostSettings.NpcCollisionActive),
            nameof(HostSettings.NpcRagdollFineDetail),
            nameof(HostSettings.PartyCompanionDeathRagdoll),
            nameof(HostSettings.RagdollActivationDelay),
            nameof(HostSettings.RagdollAnatomicalRom),
            nameof(HostSettings.RagdollExactGroundCollision),
            nameof(HostSettings.RagdollHairBodyMesh),
            nameof(HostSettings.RagdollHairPhysics),
            nameof(HostSettings.RagdollMeshCacheRevision),
            nameof(HostSettings.RagdollMeshCollisionLod),
            nameof(HostSettings.RagdollMeshContactQuality),
            nameof(HostSettings.RagdollMeshLodToleranceMm),
            nameof(HostSettings.RagdollMeshVolumes),
            nameof(HostSettings.RagdollNpcCollisionAutoSize),
            nameof(HostSettings.RagdollNpcCollisionMode),
            nameof(HostSettings.RagdollNpcCollisionScale),
            nameof(HostSettings.RagdollSoftTissueBonePrefixes),
            nameof(HostSettings.RagdollSoftTissueCollision),
            nameof(HostSettings.RagdollSoftTissueModBones),
            nameof(HostSettings.RagdollSoftTissueScope),
            nameof(HostSettings.RagdollSquashIntensity),
            nameof(HostSettings.RagdollSquashStretch),
            nameof(HostSettings.RagdollTriggerMode),
            nameof(HostSettings.RagdollVerboseLog),
            nameof(HostSettings.RagdollWaterBuoyancy),
            nameof(HostSettings.RagdollWaterLiquid),
            nameof(HostSettings.WeaponBodyMeshContacts),
            nameof(HostSettings.WeaponDropMeshCollision),
        };

        private static readonly PropertyInfo?[] ConfigProperties = Array.ConvertAll(ConfigNames,
            name => typeof(HostSettings).GetProperty(name, BindingFlags.Public | BindingFlags.Instance));

        // ------------------------------------------------------------------ lifetime

        /// <summary>Takes over an opened file for a new activation: resets the session's counters and tables.</summary>
        internal RagdollRecordingWriter Open(RagdollRecordingSession.RecordingFile file)
        {
            renderFrame = PhysicsFrame = Tick = 0;
            FitOutcome = 0;
            faultLogged = false;
            Depth = 0;
            topologies = new();
            lastPose = new();
            MeshCaches = new(ReferenceEqualityComparer.Instance);

            // From here on the file is this controller's: a failure below still closes it at deactivation.
            var w = file.Writer;
            File = file;
            Writer = w;
            return w;
        }

        internal void WriteHeader(RagdollRecordingWriter w, RagdollRecordingSession.RecordingFile file,
            RagdollBuildInfo build, bool targetIsLocalPlayer)
        {
            using (w.Begin(RecordType.Header))
            {
                w.I32(RagdollRecordingFormat.Version);
                w.Str(build.HostVersion ?? "?");
                w.Bool(build.RestoresFollowPosition);
                w.Bool(build.PublicBuild);
                w.Str(RagdollRecordingBepu.BepuVersion());
                w.Str(RuntimeInformation.FrameworkDescription);
                w.Str(RuntimeInformation.OSDescription);
                w.Str(RuntimeInformation.ProcessArchitecture.ToString());
                w.I32(Environment.ProcessorCount);
                w.I32(Vector<float>.Count);
                w.Str($"Vector.IsHardwareAccelerated={Vector.IsHardwareAccelerated} " +
                      $"Sse42={System.Runtime.Intrinsics.X86.Sse42.IsSupported} " +
                      $"Avx={System.Runtime.Intrinsics.X86.Avx.IsSupported} " +
                      $"Avx2={System.Runtime.Intrinsics.X86.Avx2.IsSupported} " +
                      $"Fma={System.Runtime.Intrinsics.X86.Fma.IsSupported} " +
                      $"Avx512F={System.Runtime.Intrinsics.X86.Avx512F.IsSupported}");
                w.Bool(BitConverter.IsLittleEndian);
                w.I64(DateTime.UtcNow.Ticks);
                w.I64(System.Diagnostics.Stopwatch.Frequency);
                w.Str(RagdollRecordingSession.SessionId);
                w.I32(file.Index);
                w.I32(ControllerId);
                w.Bool(ctx.RunAnimatedHair);
                w.U64((ulong)ctx.TargetCharacterAddress);
                w.U32(ctx.TargetEntityId);
                w.I32(ctx.Actors.GetObjectIndex(ctx.TargetCharacterAddress));
                w.Bool(targetIsLocalPlayer);
            }
        }

        /// <summary>Closes this activation's file, if any.</summary>
        internal void EndActivation(string reason)
        {
            var file = File;
            Writer = null;
            File = null;
            if (file == null) return;
            try
            {
                RagdollRecordingSession.Close(file, reason);
            }
            catch (Exception ex)
            {
                ctx.Log.Warning(ex, "RagdollRec: closing the recording failed.");
            }
        }

        internal void Fault(Exception ex)
        {
            if (!faultLogged)
            {
                faultLogged = true;
                ctx.Log.Warning(ex, "RagdollRec: a recorder helper failed; the recording continues without that record.");
            }
            try
            {
                var w = Writer;
                if (w == null) return;
                using (w.Begin(RecordType.Note))
                    w.Str("recorder fault: " + ex.GetType().Name + ": " + ex.Message);
            }
            catch
            {
                // Nothing more to do; never let the recorder disturb the frame.
            }
        }

        /// <summary>
        /// Exception filter of the recorder's own catch blocks. A filter runs before the using-scopes
        /// between the throw and the catch are disposed, so the record that was being built at that moment
        /// (if any) can still be marked: its scope then writes a note in its place instead of a record
        /// that stops half-way. Always lets the catch block run.
        /// </summary>
        internal bool AbortPartial()
        {
            try { Writer?.AbortRecord(); }
            catch { /* never let the recorder disturb the frame */ }
            return true;
        }

        /// <summary>A recorded call (or a frame) starts: what it does is nested in it.</summary>
        internal void Enter() => Depth++;

        internal void Leave()
        {
            if (Depth > 0) Depth--;
        }

        // ------------------------------------------------------------------ configuration

        private object? ConfigValue(int index)
        {
            var property = ConfigProperties[index];
            if (property == null) return "<missing property>";
            try
            {
                var value = property.GetValue(ctx.Settings);
                return value switch
                {
                    null => null,
                    bool or int or uint or long or float or double or string => value,
                    Enum e => Convert.ToInt32(e),
                    System.Collections.IEnumerable list when value is not string => Join(list),
                    _ => System.Text.Json.JsonSerializer.Serialize(value, value.GetType()),
                };
            }
            catch (Exception ex)
            {
                return "<unreadable: " + ex.GetType().Name + ">";
            }
        }

        private static string Join(System.Collections.IEnumerable list)
        {
            var text = new System.Text.StringBuilder("[");
            var first = true;
            foreach (var item in list)
            {
                if (!first) text.Append(',');
                first = false;
                text.Append(System.Text.Json.JsonSerializer.Serialize(item?.ToString()));
            }
            return text.Append(']').ToString();
        }

        internal void ConfigSnapshot(RagdollRecordingWriter w)
        {
            var prototypes = configWatchPrototypes ??= Array.ConvertAll(ConfigProperties,
                property => property == null ? null : RagdollRecordingWatch.For(property));
            var values = new object?[ConfigNames.Length];
            configWatches = new RagdollRecordingWatch?[ConfigNames.Length];
            for (var i = 0; i < ConfigNames.Length; i++)
            {
                values[i] = ConfigValue(i);
                // The first look only remembers the value the snapshot holds.
                var watch = configWatches[i] = prototypes[i]?.Clone();
                try { watch?.Changed(ctx.Settings); }
                catch { configWatches[i] = null; /* unreadable: the snapshot says so; nothing to watch */ }
            }
            using (w.Begin(RecordType.ConfigSnapshot))
            {
                w.List(ConfigNames.Length * 2);
                for (var i = 0; i < ConfigNames.Length; i++)
                {
                    w.Str(ConfigNames[i]);
                    Any(w, values[i]);
                }
            }
        }

        /// <summary>
        /// Once per render frame. While nothing changed this allocates nothing: each watch reads its
        /// property through a bound getter and compares it in its own type (see RagdollRecordingWatch).
        /// The value is read the slow way, and written, only for a property that did change.
        /// </summary>
        internal void ConfigChanges(RagdollRecordingWriter w)
        {
            var watches = configWatches;
            for (var i = 0; i < watches.Length; i++)
            {
                var watch = watches[i];
                if (watch == null) continue;
                bool changed;
                try { changed = watch.Changed(ctx.Settings); }
                catch { changed = false; }
                if (!changed) continue;
                var now = ConfigValue(i);
                using (w.Begin(RecordType.ConfigChange))
                {
                    w.Str(ConfigNames[i]);
                    Any(w, now);
                }
            }
        }

        // ------------------------------------------------------------------ generic values

        internal static void Any(RagdollRecordingWriter w, object? value)
        {
            switch (value)
            {
                case null: w.Null(); break;
                case bool v: w.Bool(v); break;
                case int v: w.I32(v); break;
                case uint v: w.U32(v); break;
                case short v: w.I32(v); break;
                case byte v: w.I32(v); break;
                case long v: w.I64(v); break;
                case ulong v: w.U64(v); break;
                case nint v: w.U64((ulong)v); break;
                case float v: w.F32(v); break;
                case double v: w.F64(v); break;
                case string v: w.Str(v); break;
                case Vector2 v: w.V2(v); break;
                case Vector3 v: w.V3(v); break;
                case Quaternion v: w.Quat(v); break;
                case Enum v: w.I32(Convert.ToInt32(v)); break;
                case float[] v: w.Floats(v); break;
                case int[] v: w.Ints(v); break;
                case Vector3[] v: w.V3s(v); break;
                case Quaternion[] v: w.Quats(v); break;
                case BodyHandle v: w.I32(v.Value); break;
                case TypedIndex v: w.U32(v.Packed); break;
                // A reference to something recorded elsewhere (a mesh, a LOD): only its presence.
                default: w.Str("<" + value.GetType().Name + ">"); break;
            }
        }

        /// <summary>Writes every public field of a struct or class as one list of (name, value) pairs.</summary>
        internal static void Fields(RagdollRecordingWriter w, object boxed)
        {
            var fields = FieldCache.GetOrAdd(boxed.GetType(),
                static type => type.GetFields(BindingFlags.Public | BindingFlags.Instance));
            w.List(fields.Length * 2);
            foreach (var field in fields)
            {
                w.Str(field.Name);
                Any(w, field.GetValue(boxed));
            }
        }

        // ------------------------------------------------------------------ skeletons and actors

        /// <param name="skeleton">The target's skeleton as the surrounding code obtained it, or null. The model
        /// type is asked of the draw object only when the ragdoll code itself would ask at this point: once a
        /// skeleton with models and a Havok skeleton was obtained (the condition in StructuralFit.Start).
        /// Otherwise the record says -1.</param>
        internal void Identity(IHostSkeleton? skeleton)
        {
            var w = Writer;
            if (w == null || ctx.TargetCharacterAddress == nint.Zero) return;
            try
            {
                Span<byte> customize = stackalloc byte[0x1A];
                ctx.Actors.ReadAppearance(ctx.TargetCharacterAddress, customize, out var modelCharaId);
                ctx.Actors.ReadModelType(ctx.TargetCharacterAddress, skeleton, out var modelType, out var raceSex);
                using (w.Begin(RecordType.Identity))
                {
                    w.U64((ulong)ctx.TargetCharacterAddress);
                    w.Bytes(customize);
                    w.I32(modelCharaId);
                    w.I32(modelType);
                    w.I32(raceSex);
                    w.F32(ctx.Actors.GetFacing(ctx.TargetCharacterAddress, ActorSite.Facing));
                    w.V3(ctx.Actors.GetPosition(ctx.TargetCharacterAddress, ActorSite.Identity));
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        private void Topology(RagdollRecordingWriter w, ulong actor, IHostSkeleton skeleton, int partial)
        {
            if (!skeleton.PartAvailable(partial)) return;
            var havok = skeleton.PartIdentity(partial);
            var bones = skeleton.PartModelPoseLength(partial);
            if (topologies.TryGetValue((actor, partial), out var seen) && seen == (havok, bones)) return;
            topologies[(actor, partial)] = (havok, bones);

            var nameCount = skeleton.PartBoneNameCount(partial);
            var parentCount = skeleton.PartParentCount(partial);
            var parents = new int[parentCount];
            for (var i = 0; i < parentCount; i++) parents[i] = skeleton.PartParent(partial, i);
            using (w.Begin(RecordType.SkeletonTopology))
            {
                w.U64(actor);
                w.I32(partial);
                w.I32(bones);
                w.I32(parentCount);
                w.I32(skeleton.PartConnectedBone(partial));
                w.I32(skeleton.PartConnectedParentBone(partial));
                w.List(nameCount);
                for (var i = 0; i < nameCount; i++) w.Str(skeleton.PartBoneName(partial, i));
                w.Ints(parents);
                w.Bytes(MemoryMarshal.AsBytes(skeleton.PartReferencePose(partial)));
            }
        }

        /// <summary>
        /// Captures a skeleton as the surrounding code is about to read it (or, for PoseOut, has just
        /// written it): the root transform, the scale sources and the model pose of the body skeleton
        /// (and of every attached partial skeleton when asked).
        /// </summary>
        internal void Skeleton(RecordType type, PosePurpose purpose, nint address, IHostSkeleton? access, bool allPartials)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                var actor = (ulong)address;
                if (access == null || !access.HasRoot)
                {
                    using (w.Begin(type))
                    {
                        w.U64(actor);
                        w.I32((int)purpose);
                        w.Bool(false);
                    }
                    return;
                }

                var skel = access;
                var partials = allPartials
                    ? Math.Clamp(skel.PartCount, 1, 64)
                    : 1;
                for (var p = 0; p < partials; p++)
                    Topology(w, actor, skel, p);

                var drawScaleValid = ctx.TryGetVisualScale(address, out var drawScale);
                var root = skel.Root;
                using (w.Begin(type))
                {
                    w.U64(actor);
                    w.I32((int)purpose);
                    w.Bool(true);
                    w.V3(root.Position);
                    w.Quat(root.Rotation);
                    w.V3(root.Scale);
                    w.Bool(drawScaleValid);
                    w.V3(drawScale);
                    w.I32(partials);
                    for (var p = 0; p < partials; p++)
                    {
                        w.I32(p);
                        if (!skel.PartHasPose(p))
                        {
                            w.I32(2);
                            continue;
                        }
                        var notInSync = skel.PartInSync(p) ? 0 : 4;
                        var bytes = MemoryMarshal.AsBytes((ReadOnlySpan<BoneQs>)skel.PartModelPose(p));
                        var key = (actor, (int)purpose, p);
                        if (lastPose.TryGetValue(key, out var last) && bytes.SequenceEqual(last))
                        {
                            w.I32(1 | notInSync);
                            continue;
                        }
                        if (last == null || last.Length != bytes.Length)
                            lastPose[key] = last = new byte[bytes.Length];
                        bytes.CopyTo(last);
                        w.I32(notInSync);
                        w.Bytes(bytes);
                    }
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        internal void ActorPosition(ActorSite site, nint address, Vector3 position)
        {
            var w = Writer;
            if (w == null) return;
            using (w.Begin(RecordType.ActorPosition))
            {
                w.I32((int)site);
                w.U64((ulong)address);
                w.V3(position);
            }
        }

        internal void ActorTransform(ActorSite site, nint address, Vector3 position, Quaternion rotation, bool scaleValid, Vector3 scale)
        {
            var w = Writer;
            if (w == null) return;
            using (w.Begin(RecordType.ActorTransform))
            {
                w.I32((int)site);
                w.U64((ulong)address);
                w.V3(position);
                w.Quat(rotation);
                w.Bool(scaleValid);
                w.V3(scale);
            }
        }

        internal void ActorList(string site, IReadOnlyList<nint> actors)
        {
            var w = Writer;
            if (w == null) return;
            using (w.Begin(RecordType.ActorList))
            {
                w.Str(site);
                w.List(actors.Count);
                for (var i = 0; i < actors.Count; i++) w.U64((ulong)actors[i]);
            }
        }

        internal void WeaponSample(Vector3 position, Quaternion rotation)
        {
            var w = Writer;
            if (w == null) return;
            using (w.Begin(RecordType.WeaponSample))
            {
                w.Bool(true);
                w.V3(position);
                w.Quat(rotation);
            }
        }

        internal void BoneWorldPosition(string site, nint address, string bone, Vector3? position)
        {
            var w = Writer;
            if (w == null) return;
            using (w.Begin(RecordType.BoneWorldPosition))
            {
                w.Str(site);
                w.U64((ulong)address);
                w.Str(bone);
                w.Bool(position.HasValue);
                w.V3(position ?? Vector3.Zero);
            }
        }

        // World queries, random draws and clock stamps are recorded by the decorators in
        // Hosting/RecordingDecorators.cs.

        // ------------------------------------------------------------------ decisions

        internal bool Decision(string site, bool value)
        {
            var w = Writer;
            if (w == null) return value;
            using (w.Begin(RecordType.Decision))
            {
                w.Str(site);
                w.Bool(value);
            }
            return value;
        }

        internal void Event(string name, float value)
        {
            var w = Writer;
            if (w == null) return;
            using (w.Begin(RecordType.Event))
            {
                w.Str(name);
                w.F32(value);
            }
        }

        // ------------------------------------------------------------------ frames

        /// <summary>
        /// The checks at the start of a render frame: closes the file when the session stopped, the disk failed or a
        /// limit was reached. Returns the writer to write this frame's records with, or null.
        /// </summary>
        internal RagdollRecordingWriter? BeginRenderFrame()
        {
            var w = Writer;
            var file = File;
            if (w == null || file == null) return null;
            if (!w.IsOpen)
            {
                // The session was stopped (or the disk failed): drop the reference so the call sites go quiet.
                EndActivation(w.Failed ? "write failed" : "session stopped");
                return null;
            }
            // The byte limits are enforced by the writer for every record (it refuses the record that would
            // cross one, and everything after it). Here the refusal is noticed and the file is closed.
            if (RagdollRecordingSession.SessionBytesExhausted)
            {
                RagdollRecordingSession.CloseAllAtSessionLimit();
                EndActivation(RagdollRecordingSession.SessionBytesReason);
                return null;
            }
            if (file.Seconds > RagdollRecordingSession.SecondsLimit || w.FileLimitReached)
            {
                EndActivation(w.FileLimitReached
                    ? $"size limit of {RagdollRecordingSession.MaxBytesPerFile / (1024 * 1024)} MiB reached"
                    : $"time limit of {RagdollRecordingSession.MaxSecondsPerFile:F0} s reached");
                return null;
            }
            return w;
        }

        /// <summary>Counts a render frame (in the file too) and returns its number.</summary>
        internal int NextRenderFrame(RagdollRecordingSession.RecordingFile file) => file.RenderFrames = ++renderFrame;

        internal void PreFrame()
        {
            if (Writer == null) return;
            try
            {
                Skeleton(RecordType.SkeletonPose, PosePurpose.PreFrame, ctx.TargetCharacterAddress,
                    ctx.Skeletons.TryGetSkeleton(ctx.TargetCharacterAddress, PosePurpose.PreFrame), false);
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        internal void TriggerInputs(RagdollAnimationSample? animation, float? middleHeight, float? motion)
        {
            var w = Writer;
            if (w == null) return;
            using (w.Begin(RecordType.TriggerInputs))
            {
                w.Bool(animation.HasValue);
                w.U64((ulong)(animation?.Id ?? 0));
                w.F32(animation?.Time ?? 0f);
                w.F32(animation?.Duration ?? 0f);
                w.Bool(middleHeight.HasValue);
                w.F32(middleHeight ?? 0f);
                w.Bool(motion.HasValue);
                w.F32(motion ?? 0f);
            }
        }

        /// <summary>After each fixed step (and everything the controller does around it): every body's state.</summary>
        internal void Step(int substep, bool steppedExternally)
        {
            var w = Writer;
            var file = File;
            if (w == null || file == null || ctx.Simulation == null) return;
            try
            {
                file.FixedSteps = ++Tick;
                BodyStates(w, ctx.Simulation, Tick, substep, steppedExternally);
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        /// <summary>Every body's state, as of this physics frame.</summary>
        internal void BodyStates(RagdollRecordingWriter w, BepuSimulation simulation, int tick, int substep,
            bool steppedExternally)
            => RagdollRecordingBepu.WriteBodyStates(w, simulation, tick, PhysicsFrame, substep, steppedExternally,
                ref handles, ref flags, ref states);

        // ------------------------------------------------------------------ InitializePhysics

        internal void BoneDefs(RagdollBoneDef[] defs, Dictionary<string, int> nameToIndex, bool generic)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                using (w.Begin(RecordType.BoneDefs))
                {
                    w.Bool(generic);
                    w.I32(defs.Length);
                    foreach (var def in defs)
                    {
                        Fields(w, def);
                        w.I32(nameToIndex.TryGetValue(def.Name, out var index) ? index : -1);
                    }
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        internal void StructuralFits(Dictionary<string, StructuralMeshFitter.Fit> fits)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                using (w.Begin(RecordType.StructuralFits))
                {
                    w.I32(FitOutcome);
                    w.I32(fits.Count);
                    foreach (var entry in fits)
                    {
                        w.Str(entry.Key);
                        Fields(w, entry.Value);
                    }
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        /// <summary>The cached bone-local surface of a character, the first time this recording sees that cache.</summary>
        internal void MeshCache(FrozenMeshGeometryCache? cache, string use)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                using (w.Begin(RecordType.MeshCache))
                {
                    w.Str(use);
                    RecordingMeshSource.WriteCache(w, cache, MeshCaches, true);
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        internal void MeshLod(string site, string bone, MeshCollisionLodResult lod)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                using (w.Begin(RecordType.MeshLod))
                {
                    w.Str(site);
                    w.Str(bone);
                    w.V3s(lod.Geometry.Vertices);
                    w.Ints(lod.Geometry.Indices);
                    w.Ints(lod.SourceVertexIndices);
                    w.I32(lod.SourceFaceCount);
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        internal void MeshUse(string site, string bone, BodyMeshGeometry geometry)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                using (w.Begin(RecordType.MeshLod))
                {
                    w.Str(site);
                    w.Str(bone);
                    w.V3s(geometry.Vertices);
                    w.Ints(geometry.Indices);
                    w.Ints(default);
                    w.I32(geometry.Indices.Length / 3);
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        internal void Triangles(string use, List<Triangle> triangles, Vector3 scale)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                using (w.Begin(RecordType.Triangles))
                {
                    w.Str(use);
                    w.V3(scale);
                    w.V3s(MemoryMarshal.Cast<Triangle, Vector3>(CollectionsMarshal.AsSpan(triangles)));
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        /// <param name="animatedHair">Whether the hair rig runs the live (animated) hair, as the hair rig holds it.</param>
        internal void HairMesh(HairMeshBinding? binding, bool animatedHair)
        {
            var w = Writer;
            if (w == null) return;
            try
            {
                using (w.Begin(RecordType.HairMesh))
                {
                    w.Bool(binding != null);
                    w.Bool(animatedHair);
                    w.I32(binding?.VisibleVertices ?? 0);
                    w.I32(binding?.Samples.Count ?? 0);
                    if (binding == null) return;
                    foreach (var region in binding.Samples)
                    {
                        var samples = region.Value;
                        var positions = new Vector3[samples.Length];
                        var counts = new int[samples.Length];
                        var total = 0;
                        for (var i = 0; i < samples.Length; i++)
                        {
                            positions[i] = samples[i].Position;
                            counts[i] = samples[i].Influences.Length;
                            total += counts[i];
                        }
                        var weights = new float[total];
                        w.Str(region.Key);
                        w.V3s(positions);
                        w.Ints(counts);
                        w.List(total);
                        var at = 0;
                        foreach (var sample in samples)
                            foreach (var influence in sample.Influences)
                            {
                                w.Str(influence.Bone);
                                weights[at++] = influence.Weight;
                            }
                        w.Floats(weights);
                    }
                }
            }
            catch (Exception ex) when (AbortPartial())
            {
                Fault(ex);
            }
        }

        // ------------------------------------------------------------------ calls from the rest of the plugin

        internal RagdollRecordingWriter.Scope Call(RagdollRecordingWriter w, string method)
        {
            var scope = w.Begin(RecordType.ApiCall);
            try
            {
                w.I32(Depth);
                w.Str(method);
                return scope;
            }
            catch
            {
                // The caller never receives the scope, so it could not finish it: do that here (as a
                // discarded record), or the writer's lock would stay held.
                w.AbortRecord();
                scope.Dispose();
                throw;
            }
        }

        internal static bool Return(RagdollRecordingWriter w, string method, bool value)
        {
            using (w.Begin(RecordType.ApiReturn))
            {
                w.Str(method);
                w.Bool(value);
            }
            return value;
        }

        // The three helpers below write an argument inside a call record. They read objects handed in by
        // the caller, so they are the only part of such a record that can throw; when one does, the record
        // is marked incomplete (the writer puts a note in its place) and the call itself goes ahead.

        internal static void Body(RagdollRecordingWriter w, ExternalBodyHandle? handle)
        {
            if (handle == null) { w.Null(); return; }
            try
            {
                w.List(3);
                w.I32(handle.Body.Value);
                w.I32(handle.Generation);
                w.Bool(handle.Removed);
            }
            catch { w.AbortRecord(); }
        }

        internal static void Rig(RagdollRecordingWriter w, ExternalRigHandle? handle)
        {
            if (handle == null) { w.Null(); return; }
            try
            {
                var bodies = new int[handle.Bodies.Count];
                for (var i = 0; i < bodies.Length; i++) bodies[i] = handle.Bodies[i].Body.Value;
                w.List(3);
                w.I32(handle.Generation);
                w.Bool(handle.Removed);
                w.Ints(bodies);
            }
            catch { w.AbortRecord(); }
        }

        internal static void Parts(RagdollRecordingWriter w, IReadOnlyList<ExternalShapePart>? parts)
        {
            if (parts == null) { w.Null(); return; }
            try
            {
                w.List(parts.Count * 3);
                for (var i = 0; i < parts.Count; i++)
                {
                    w.V3(parts[i].HalfExtents);
                    w.V3(parts[i].LocalPosition);
                    w.Quat(parts[i].LocalOrientation);
                }
            }
            catch { w.AbortRecord(); }
        }

        /// <summary>Hands a sequence on unchanged and notes each item as the consumer takes it.</summary>
        internal static IEnumerable<string> Tee(IEnumerable<string> source, List<string?> seen)
        {
            foreach (var item in source)
            {
                seen.Add(item);
                yield return item;
            }
        }
    }

    // ------------------------------------------------------------------ the ragdoll's recording calls
    //
    // Each Rec* method of the ragdoll either forwards to the recorder or, where the record reads the state of other
    // subsystems, writes that record itself through the recorder's writer, reading the same values at the same
    // moments as before.

    /// <summary>Called from Activate once the target is known. Opens this activation's file when a session is on.</summary>
    private void RecBeginActivation(float delay, RagdollTriggerMode mode, float stagger)
    {
        if (!RagdollRecordingSession.Enabled) return;
        try
        {
            var file = RagdollRecordingSession.TryOpen(targetIsLocalPlayer, targetEntityId);
            if (file == null) return;

            // From here on the file is this controller's: a failure below still closes it at deactivation.
            var w = recorder.Open(file);
            recorder.WriteHeader(w, file, host.Build, targetIsLocalPlayer);

            recorder.ConfigSnapshot(w);
            using (w.Begin(RecordType.Activate))
            {
                w.U64((ulong)targetCharacterAddress);
                w.F32(delay);
                w.I32((int)mode);
                w.F32(stagger);
                w.V3(savedCharacterPosition);
                w.U32(targetEntityId);
                w.I32(actors.GetObjectIndex(targetCharacterAddress));
                w.Bool(targetIsLocalPlayer);
                w.Bool(colliders.HasSelector);
                w.Bool(colliders.HasMovementBlock);
                w.Bool(colliders.HasExtraColliders);
                w.Bool(RunAnimatedHair);
                w.Bool(GrabRigid);
                w.Bool(CarryExternalRigOnAnchor != null);
                // State that calls made before this activation may have left behind.
                w.Bool(pendingDirectedCollapseSpike);
                w.Str(pendingDirectedProfileId);
                w.F32(pendingDirectedDuration);
                w.F32(pendingDirectedFootForce);
                w.F32(pendingDirectedPelvisForce);
                w.F32(pendingDirectedDrop);
                w.F32(pendingDirectedForward);
                w.F32(pendingDirectedPitchDegrees);
                w.Bool(pendingEntryConditionedKneePowerLoss);
                w.Bool(pendingKneePowerLossPattern);
                w.Bool(useLocalDismemberBones);
                w.List(localDismemberBones.Count);
                foreach (var bone in localDismemberBones) w.Str(bone);
                w.List(lightweightNpcCollisionAddresses.Count);
                foreach (var address in lightweightNpcCollisionAddresses) w.U64((ulong)address);
                // State this controller carries over from its earlier activations: neither Activate nor the
                // teardown resets it, so a replay that starts here has to be given it.
                w.U32(impactRandomState);
                w.I32(nextGrabSlotId);
                w.I32(externalBodyGeneration);
                w.List(struckThisWindow.Count);
                foreach (var handle in struckThisWindow) w.I32(handle);
                w.List(strikeColliderAddresses.Count);
                foreach (var address in strikeColliderAddresses) w.U64((ulong)address);
                lock (npcTraversalProxySync)
                {
                    w.I32(npcTraversalProxyRequests.Count);
                    foreach (var entry in npcTraversalProxyRequests)
                    {
                        var request = entry.Value;
                        w.U64((ulong)entry.Key);
                        w.Bool(request.Enabled);
                        w.V3(request.InitialRoot);
                        w.V3(request.TargetRoot);
                        w.F32(request.FloorY);
                        w.F32(request.VisualScale);
                        w.F32(request.MoveSpeed);
                        w.F32(request.MaxClimb);
                        w.F32(request.StompResponse);
                        w.I32(request.ConcurrentStompers);
                        w.Bool(request.Locomoting);
                    }
                }
                // Format version 3: whether the previous activation's last frame wrote a follow position
                // (public builds never reset it).
                w.Bool(followWasActive);
            }
            // Format version 3: the body surfaces this controller carries over from earlier activations (the main
            // controller captures the local player's long before it dies), with the bookkeeping that decides
            // whether they are captured again.
            using (w.Begin(RecordType.MeshProduct))
            {
                w.Str(MeshProductKind.Carried);
                w.U64((ulong)frozenMeshActor);
                w.U32(frozenMeshEntity);
                w.Bool(observedMeshEnabled);
                w.I32(observedMeshCacheRevision);
                w.Bool(cachedMeshLodEnabled);
                w.F32(cachedMeshLodTolerance);
                w.I64(nextMeshCacheAttempt);
                RecordingMeshSource.WriteCache(w, frozenMeshCache, recorder.MeshCaches, false);
                w.I32(weaponContactMeshOrder.Count);
                foreach (var address in weaponContactMeshOrder)
                {
                    var entry = weaponContactMeshes[address];
                    w.U64((ulong)address);
                    w.U32(entry.Entity);
                    w.U64((ulong)entry.Draw);
                    RecordingMeshSource.WriteCache(w, entry.Cache, recorder.MeshCaches, false);
                }
            }
            recorder.Identity(skeletons.TryGetSkeleton(targetCharacterAddress));
        }
        catch (Exception ex) when (RecAbortPartial())
        {
            RecFault(ex);
        }
    }

    /// <summary>Closes this activation's file, if any.</summary>
    private void RecEndActivation(string reason) => recorder.EndActivation(reason);
    private void RecFault(Exception ex) => recorder.Fault(ex);
    private bool RecAbortPartial() => recorder.AbortPartial();
    private void RecEnter() => recorder.Enter();
    private void RecLeave() => recorder.Leave();

    /// <summary>Once per rendered frame (the plugin subscribes it to the render hook): steps the ragdoll and writes the pose.</summary>
    internal void OnRenderFrame()
    {
        if (rec == null)
        {
            OnRenderFrameCore();
            return;
        }
        RecEnter();
        try { OnRenderFrameCore(); }
        finally { RecLeave(); }
    }

    private void RecSkeleton(RecordType type, PosePurpose purpose, nint address, IHostSkeleton? access, bool allPartials)
        => recorder.Skeleton(type, purpose, address, access, allPartials);
    private void RecActorPosition(ActorSite site, nint address, Vector3 position) => recorder.ActorPosition(site, address, position);
    private void RecActorTransform(ActorSite site, nint address, Vector3 position, Quaternion rotation, bool scaleValid, Vector3 scale)
        => recorder.ActorTransform(site, address, position, rotation, scaleValid, scale);
    private void RecActorList(string site, IReadOnlyList<nint> actors) => recorder.ActorList(site, actors);

    /// <summary>The selected NPCs in the order the collision build walks them (those without a character last, flagged).</summary>
    private void RecSelectedNpcs(string site, List<ColliderCandidate> selected)
    {
        if (rec == null || !colliders.HasSelector) return;
        try
        {
            var present = new List<nint>();
            var missing = new List<nint>();
            foreach (var npc in selected)
                (!npc.HasCharacter ? missing : present).Add(npc.Address);
            recorder.ActorList(site, present);
            if (missing.Count > 0) recorder.ActorList(site + ".withoutCharacter", missing);
        }
        catch (Exception ex) when (RecAbortPartial())
        {
            RecFault(ex);
        }
    }

    private void RecWeaponSample(Vector3 position, Quaternion rotation) => recorder.WeaponSample(position, rotation);
    private void RecBoneWorldPosition(string site, nint address, string bone, Vector3? position)
        => recorder.BoneWorldPosition(site, address, bone, position);
    private bool RecDecision(string site, bool value) => recorder.Decision(site, value);
    private void RecEvent(string name, float value) => recorder.Event(name, value);

    /// <summary>Once per render frame while the ragdoll is active, right after the frame's dt is known.</summary>
    private void RecRenderFrame(float dt)
    {
        var w = recorder.BeginRenderFrame();
        if (w == null) return;
        try
        {
            recorder.ConfigChanges(w);
            var frame = recorder.NextRenderFrame(recorder.File!);
            using (w.Begin(RecordType.RenderFrame))
            {
                w.I32(frame);
                w.F32(dt);
                w.F32(elapsed);
                w.Bool(physicsStarted);
                w.F32(attackStrikeTimer);
                w.F32(strikePower);
                w.U64((ulong)strikeColliderAddress);
            }
        }
        catch (Exception ex) when (RecAbortPartial())
        {
            RecFault(ex);
        }
    }

    private void RecPreFrame() => recorder.PreFrame();
    private void RecTriggerInputs(RagdollAnimationSample? animation, float? middleHeight, float? motion)
        => recorder.TriggerInputs(animation, middleHeight, motion);

    private void RecFrameBegin(float dt, IHostSkeleton skel)
    {
        var w = rec;
        var file = recorder.File;
        if (w == null || file == null) return;
        try
        {
            file.PhysicsFrames = ++recorder.PhysicsFrame;
            using (w.Begin(RecordType.FrameBegin))
            {
                w.I32(recorder.PhysicsFrame);
                w.F32(dt);
                w.F32(physicsAccumulator);
                w.Bool(prevAllAsleep);
                w.U32(actors.GetEntityId(targetCharacterAddress));
                w.I32(actors.GetObjectIndex(targetCharacterAddress));
                w.Bool(followWasActive);
                w.F32(impactFreezeRemaining);
                w.F32(biomechanicalSettleRemaining);
                w.I32(grabSlots.Count);
                w.I32(externalBodies.Count);
                w.I32(externalRigs.Count);
            }
            recorder.Skeleton(RecordType.SkeletonPose, PosePurpose.FrameIn, targetCharacterAddress, skel, true);
        }
        catch (Exception ex) when (RecAbortPartial())
        {
            RecFault(ex);
        }
    }

    /// <summary>After each fixed step (and everything the controller does around it): every body's state.</summary>
    private void RecStep(int substep, bool steppedExternally) => recorder.Step(substep, steppedExternally);

    private void RecFrameEnd(int substeps, bool resting, float renderAlpha)
    {
        var w = rec;
        if (w == null) return;
        using (w.Begin(RecordType.FrameEnd))
        {
            w.I32(recorder.PhysicsFrame);
            w.I32(substeps);
            w.Bool(resting);
            w.F32(physicsAccumulator);
            w.F32(renderAlpha);
            w.I32(recorder.Tick);
        }
    }

    private void RecInitBegin(IHostSkeleton skel)
    {
        var w = rec;
        if (w == null) return;
        try
        {
            recorder.NoteFitOutcome(0);
            var count = handoffSampleValid && handoffPrevPos != null && handoffPrevRot != null
                ? Math.Min(handoffPrevCount, Math.Min(handoffPrevPos.Length, handoffPrevRot.Length))
                : 0;
            using (w.Begin(RecordType.InitBegin))
            {
                w.Bool(handoffSampleValid);
                w.F32(handoffSampleDt);
                w.V3(handoffSkelPos);
                w.Quat(handoffSkelRot);
                w.V3(handoffSkelScale);
                w.I32(handoffPrevCount);
                w.V3s(count > 0 ? handoffPrevPos.AsSpan(0, count) : default);
                w.Quats(count > 0 ? handoffPrevRot.AsSpan(0, count) : default);
            }
            recorder.Identity(skel);
            recorder.Skeleton(RecordType.SkeletonPose, PosePurpose.Init, targetCharacterAddress, skel, true);
        }
        catch (Exception ex) when (RecAbortPartial())
        {
            RecFault(ex);
        }
    }

    private void RecBoneDefs(RagdollBoneDef[] defs, Dictionary<string, int> nameToIndex, bool generic)
        => recorder.BoneDefs(defs, nameToIndex, generic);
    private void RecStructuralFits(Dictionary<string, StructuralMeshFitter.Fit> fits) => recorder.StructuralFits(fits);
    /// <summary>The cached bone-local surface of a character, the first time this recording sees that cache.</summary>
    private void RecMeshCache(FrozenMeshGeometryCache? cache, string use) => recorder.MeshCache(cache, use);
    private void RecTriangles(string use, List<Triangle> triangles, Vector3 scale) => recorder.Triangles(use, triangles, scale);

    private void RecInitEnd(bool ok, int solverIterations, int solverSubsteps, bool generic)
    {
        var w = rec;
        if (w == null) return;
        try
        {
            using (w.Begin(RecordType.RigBodies))
            {
                w.I32(ragdollBones.Count);
                foreach (var rb in ragdollBones)
                {
                    w.Str(rb.Name);
                    w.I32(rb.BoneIndex);
                    w.I32(rb.BodyHandle.Value);
                    Recorder.Fields(w, rb);
                }
            }
            using (w.Begin(RecordType.InitEnd))
            {
                w.Bool(ok);
                w.I32(ragdollBones.Count);
                w.F32(groundY);
                w.F32(realGroundY);
                w.V3(skelWorldPos);
                w.Quat(skelWorldRot);
                w.V3(skelWorldScale);
                w.Bool(generic);
                w.I32(solverIterations);
                w.I32(solverSubsteps);
                w.I32(simulation?.Bodies.CountBodies() ?? 0);
                w.I32(simulation?.Statics.Count ?? 0);
                w.F32(savedOverallSpeed);
                w.I32(npcCollisionStates.Count);
                w.I32(hairRig.ChainCount);
            }
            if (simulation != null)
                recorder.BodyStates(w, simulation, 0, 0, false);
        }
        catch (Exception ex) when (RecAbortPartial())
        {
            RecFault(ex);
        }
    }

    // ------------------------------------------------------------------ calls from the rest of the plugin

    private RagdollRecordingWriter.Scope RecCall(RagdollRecordingWriter w, string method) => recorder.Call(w, method);
    private static bool RecReturn(RagdollRecordingWriter w, string method, bool value) => Recorder.Return(w, method, value);
    private static void RecBody(RagdollRecordingWriter w, ExternalBodyHandle? handle) => Recorder.Body(w, handle);
    private static void RecRig(RagdollRecordingWriter w, ExternalRigHandle? handle) => Recorder.Rig(w, handle);
    private static void RecParts(RagdollRecordingWriter w, IReadOnlyList<ExternalShapePart>? parts) => Recorder.Parts(w, parts);
    /// <summary>Hands a sequence on unchanged and notes each item as the consumer takes it.</summary>
    private static IEnumerable<string> RecTee(IEnumerable<string> source, List<string?> seen) => Recorder.Tee(source, seen);

    internal bool TryCreateExternalDynamicBody(IReadOnlyList<ExternalShapePart> parts, float mass,
        Vector3 position, Quaternion orientation, Vector3 linearVelocity, Vector3 angularVelocity,
        out ExternalBodyHandle? handle)
    {
        var w = rec;
        if (w == null)
            return TryCreateExternalDynamicBodyCore(parts, mass, position, orientation, linearVelocity, angularVelocity, out handle);
        using (RecCall(w, nameof(TryCreateExternalDynamicBody)))
        {
            RecParts(w, parts); w.F32(mass); w.V3(position); w.Quat(orientation); w.V3(linearVelocity); w.V3(angularVelocity);
        }
        RecEnter();
        try
        {
            var ok = TryCreateExternalDynamicBodyCore(parts, mass, position, orientation, linearVelocity, angularVelocity, out handle);
            using (w.Begin(RecordType.ApiReturn)) { w.Str(nameof(TryCreateExternalDynamicBody)); w.Bool(ok); RecBody(w, handle); }
            return ok;
        }
        finally { RecLeave(); }
    }

    internal bool TryCreateExternalRig(
        IReadOnlyList<ExternalRigBodySpec> bodySpecs,
        IReadOnlyList<ExternalRigJointSpec> jointSpecs,
        out ExternalRigHandle? handle,
        bool collideWithRagdoll = true)
    {
        var w = rec;
        if (w == null) return TryCreateExternalRigCore(bodySpecs, jointSpecs, out handle, collideWithRagdoll);
        try
        {
            using (RecCall(w, nameof(TryCreateExternalRig)))
            {
                w.I32(bodySpecs?.Count ?? -1);
                for (var i = 0; bodySpecs != null && i < bodySpecs.Count; i++)
                {
                    var spec = bodySpecs[i];
                    w.Str(spec.Name); RecParts(w, spec.Parts); w.F32(spec.Mass); w.V3(spec.Position); w.Quat(spec.Orientation);
                    w.V3(spec.LinearVelocity); w.V3(spec.AngularVelocity);
                }
                w.I32(jointSpecs?.Count ?? -1);
                for (var i = 0; jointSpecs != null && i < jointSpecs.Count; i++)
                {
                    var joint = jointSpecs[i];
                    w.I32((int)joint.Kind); w.I32(joint.ParentIndex); w.I32(joint.ChildIndex); w.V3(joint.AnchorWorld);
                    w.F32(joint.SwingLimit); w.F32(joint.InitialSwingFactor); w.F32(joint.PoseGuideMaxForce);
                    w.F32(joint.PoseGuideFrequency); w.V3(joint.LocalOffsetChild); w.V3(joint.LocalOffsetParent);
                    w.F32(joint.MinDistance); w.F32(joint.MaxDistance); w.F32(joint.DistanceFrequency);
                    w.F32(joint.DistanceDamping); w.F32(joint.JointFriction);
                }
                w.Bool(collideWithRagdoll);
            }
        }
        catch (Exception ex) when (RecAbortPartial()) { RecFault(ex); }
        RecEnter();
        try
        {
            var ok = TryCreateExternalRigCore(bodySpecs!, jointSpecs!, out handle, collideWithRagdoll);
            using (w.Begin(RecordType.ApiReturn)) { w.Str(nameof(TryCreateExternalRig)); w.Bool(ok); RecRig(w, handle); }
            return ok;
        }
        finally { RecLeave(); }
    }

    internal bool TrySetExternalBodyShape(ExternalBodyHandle? handle,
        IReadOnlyList<ExternalShapePart> parts, float mass)
    {
        var w = rec;
        if (w == null) return TrySetExternalBodyShapeCore(handle, parts, mass);
        using (RecCall(w, nameof(TrySetExternalBodyShape))) { RecBody(w, handle); RecParts(w, parts); w.F32(mass); }
        RecEnter();
        try { return RecReturn(w, nameof(TrySetExternalBodyShape), TrySetExternalBodyShapeCore(handle, parts, mass)); }
        finally { RecLeave(); }
    }

    internal bool TrySetExternalRigBodyPose(ExternalRigHandle? handle, int bodyIndex,
        Vector3 position, Quaternion orientation, Vector3 linearVelocity, Vector3 angularVelocity)
    {
        var w = rec;
        if (w == null) return TrySetExternalRigBodyPoseCore(handle, bodyIndex, position, orientation, linearVelocity, angularVelocity);
        using (RecCall(w, nameof(TrySetExternalRigBodyPose)))
        {
            RecRig(w, handle); w.I32(bodyIndex); w.V3(position); w.Quat(orientation); w.V3(linearVelocity); w.V3(angularVelocity);
        }
        RecEnter();
        try
        {
            return RecReturn(w, nameof(TrySetExternalRigBodyPose),
                TrySetExternalRigBodyPoseCore(handle, bodyIndex, position, orientation, linearVelocity, angularVelocity));
        }
        finally { RecLeave(); }
    }

    internal bool SetExternalRigBodyRagdollContact(ExternalRigHandle? handle, int bodyIndex, bool collide)
    {
        var w = rec;
        if (w == null) return SetExternalRigBodyRagdollContactCore(handle, bodyIndex, collide);
        using (RecCall(w, nameof(SetExternalRigBodyRagdollContact))) { RecRig(w, handle); w.I32(bodyIndex); w.Bool(collide); }
        RecEnter();
        try { return RecReturn(w, nameof(SetExternalRigBodyRagdollContact), SetExternalRigBodyRagdollContactCore(handle, bodyIndex, collide)); }
        finally { RecLeave(); }
    }

    internal bool TrySetExternalRigBodyShape(ExternalRigHandle? handle, int bodyIndex,
        IReadOnlyList<ExternalShapePart> parts, float mass)
    {
        var w = rec;
        if (w == null) return TrySetExternalRigBodyShapeCore(handle, bodyIndex, parts, mass);
        using (RecCall(w, nameof(TrySetExternalRigBodyShape))) { RecRig(w, handle); w.I32(bodyIndex); RecParts(w, parts); w.F32(mass); }
        RecEnter();
        try { return RecReturn(w, nameof(TrySetExternalRigBodyShape), TrySetExternalRigBodyShapeCore(handle, bodyIndex, parts, mass)); }
        finally { RecLeave(); }
    }

    internal bool TryApplyExternalRigVelocityDelta(ExternalRigHandle? handle, Vector3 velocityDelta)
    {
        var w = rec;
        if (w == null) return TryApplyExternalRigVelocityDeltaCore(handle, velocityDelta);
        using (RecCall(w, nameof(TryApplyExternalRigVelocityDelta))) { RecRig(w, handle); w.V3(velocityDelta); }
        RecEnter();
        try { return RecReturn(w, nameof(TryApplyExternalRigVelocityDelta), TryApplyExternalRigVelocityDeltaCore(handle, velocityDelta)); }
        finally { RecLeave(); }
    }

    internal bool TryDampenExternalRigVelocity(ExternalRigHandle? handle,
        float horizontalScale, float verticalScale, float angularScale)
    {
        var w = rec;
        if (w == null) return TryDampenExternalRigVelocityCore(handle, horizontalScale, verticalScale, angularScale);
        using (RecCall(w, nameof(TryDampenExternalRigVelocity))) { RecRig(w, handle); w.F32(horizontalScale); w.F32(verticalScale); w.F32(angularScale); }
        RecEnter();
        try { return RecReturn(w, nameof(TryDampenExternalRigVelocity), TryDampenExternalRigVelocityCore(handle, horizontalScale, verticalScale, angularScale)); }
        finally { RecLeave(); }
    }

    internal bool RelaxExternalRigSwingLimits(ExternalRigHandle? handle, float factor)
    {
        var w = rec;
        if (w == null) return RelaxExternalRigSwingLimitsCore(handle, factor);
        using (RecCall(w, nameof(RelaxExternalRigSwingLimits))) { RecRig(w, handle); w.F32(factor); }
        RecEnter();
        try { return RecReturn(w, nameof(RelaxExternalRigSwingLimits), RelaxExternalRigSwingLimitsCore(handle, factor)); }
        finally { RecLeave(); }
    }

    internal bool ApplyExternalRigPoseGuidance(ExternalRigHandle? handle, float strength)
    {
        var w = rec;
        if (w == null) return ApplyExternalRigPoseGuidanceCore(handle, strength);
        using (RecCall(w, nameof(ApplyExternalRigPoseGuidance))) { RecRig(w, handle); w.F32(strength); }
        RecEnter();
        try { return RecReturn(w, nameof(ApplyExternalRigPoseGuidance), ApplyExternalRigPoseGuidanceCore(handle, strength)); }
        finally { RecLeave(); }
    }

    internal bool TrySetExternalBodyRotation(ExternalBodyHandle? handle, Quaternion orientation, float angularVelocityScale)
    {
        var w = rec;
        if (w == null) return TrySetExternalBodyRotationCore(handle, orientation, angularVelocityScale);
        using (RecCall(w, nameof(TrySetExternalBodyRotation))) { RecBody(w, handle); w.Quat(orientation); w.F32(angularVelocityScale); }
        RecEnter();
        try { return RecReturn(w, nameof(TrySetExternalBodyRotation), TrySetExternalBodyRotationCore(handle, orientation, angularVelocityScale)); }
        finally { RecLeave(); }
    }

    internal bool TryApplyExternalVelocityDelta(ExternalBodyHandle? handle, Vector3 velocityDelta)
    {
        var w = rec;
        if (w == null) return TryApplyExternalVelocityDeltaCore(handle, velocityDelta);
        using (RecCall(w, nameof(TryApplyExternalVelocityDelta))) { RecBody(w, handle); w.V3(velocityDelta); }
        RecEnter();
        try { return RecReturn(w, nameof(TryApplyExternalVelocityDelta), TryApplyExternalVelocityDeltaCore(handle, velocityDelta)); }
        finally { RecLeave(); }
    }

    internal bool TryDampenExternalBodyVelocity(ExternalBodyHandle? handle,
        float horizontalScale, float verticalScale, float angularScale)
    {
        var w = rec;
        if (w == null) return TryDampenExternalBodyVelocityCore(handle, horizontalScale, verticalScale, angularScale);
        using (RecCall(w, nameof(TryDampenExternalBodyVelocity))) { RecBody(w, handle); w.F32(horizontalScale); w.F32(verticalScale); w.F32(angularScale); }
        RecEnter();
        try { return RecReturn(w, nameof(TryDampenExternalBodyVelocity), TryDampenExternalBodyVelocityCore(handle, horizontalScale, verticalScale, angularScale)); }
        finally { RecLeave(); }
    }

    internal void RemoveExternalBody(ExternalBodyHandle? handle)
    {
        var w = rec;
        if (w == null) { RemoveExternalBodyCore(handle); return; }
        using (RecCall(w, nameof(RemoveExternalBody))) RecBody(w, handle);
        RecEnter();
        try { RemoveExternalBodyCore(handle); }
        finally { RecLeave(); }
    }

    internal void RemoveExternalRig(ExternalRigHandle? handle)
    {
        var w = rec;
        if (w == null) { RemoveExternalRigCore(handle); return; }
        using (RecCall(w, nameof(RemoveExternalRig))) RecRig(w, handle);
        RecEnter();
        try { RemoveExternalRigCore(handle); }
        finally { RecLeave(); }
    }

    internal void SetDismemberedBones(IEnumerable<string> bones)
    {
        var w = rec;
        if (w == null) { SetDismemberedBonesCore(bones); return; }
        // Same order of effects as with recording off: the core clears its table first and only then
        // walks the caller's sequence, once (the sequence may be lazy, or may read that very table).
        // The items are noted as the core takes them, so this is the one call whose record is written
        // after the call ran. It makes no other record, so nothing can come between the two.
        var seen = new List<string?>();
        RecEnter();
        try { SetDismemberedBonesCore(bones == null ? null! : RecTee(bones, seen)); }
        finally
        {
            RecLeave();
            try
            {
                using (RecCall(w, nameof(SetDismemberedBones)))
                {
                    if (bones == null) w.Null();
                    else
                    {
                        w.List(seen.Count);
                        foreach (var bone in seen) w.Str(bone);
                    }
                }
            }
            catch (Exception ex) when (RecAbortPartial())
            {
                RecFault(ex);
            }
        }
    }

    /// <summary>Stops the ragdoll and gives the body back (<see cref="IRagdollActor.OnDeactivated"/> when physics had
    /// started). The physics is torn down; <see cref="Activate(in RagdollActivation)"/> may arm it again.</summary>
    public void Deactivate()
    {
        var w = rec;
        if (w == null) { DeactivateCore(); return; }
        using (RecCall(w, nameof(Deactivate))) { }
        RecEnter();
        try { DeactivateCore(); }
        finally
        {
            RecLeave();
            RecEndActivation("deactivated");
        }
    }

    internal void SetTraversalMassMultiplier(float multiplier)
    {
        var w = rec;
        if (w == null) { SetTraversalMassMultiplierCore(multiplier); return; }
        using (RecCall(w, nameof(SetTraversalMassMultiplier))) w.F32(multiplier);
        RecEnter();
        try { SetTraversalMassMultiplierCore(multiplier); }
        finally { RecLeave(); }
    }

    internal int? CreateGrabConstraint(string boneName, Vector3 initialTarget, nint grabbingNpcAddress = 0, float maxForce = 1000f, float maxSpeed = 50f, float springFreq = 120f)
    {
        var w = rec;
        if (w == null) return CreateGrabConstraintCore(boneName, initialTarget, grabbingNpcAddress, maxForce, maxSpeed, springFreq);
        using (RecCall(w, nameof(CreateGrabConstraint)))
        {
            w.Str(boneName); w.V3(initialTarget); w.U64((ulong)grabbingNpcAddress); w.F32(maxForce); w.F32(maxSpeed); w.F32(springFreq);
            w.Bool(GrabRigid); // read by the call, set through the property
        }
        RecEnter();
        try
        {
            var slot = CreateGrabConstraintCore(boneName, initialTarget, grabbingNpcAddress, maxForce, maxSpeed, springFreq);
            using (w.Begin(RecordType.ApiReturn)) { w.Str(nameof(CreateGrabConstraint)); w.Bool(slot.HasValue); w.I32(slot ?? 0); }
            return slot;
        }
        finally { RecLeave(); }
    }

    /// <summary>
    /// Work out what a grab on this bone should actually take hold of: the bone's own body, or the
    /// body of the nearest ancestor that has one, with the offset of the requested bone in that
    /// body's frame (see TryResolveGrabAttachmentCore).
    ///
    /// It changes nothing, but it reads a bone's world position from the game, so it is recorded as a
    /// call: the BoneWorldPosition record it makes then always belongs to a call record (this one when
    /// called from outside, nested in CreateGrabConstraint's otherwise).
    /// </summary>
    internal bool TryResolveGrabAttachment(string boneName, out BodyHandle body, out Vector3 localOffset)
    {
        var w = rec;
        if (w == null) return TryResolveGrabAttachmentCore(boneName, out body, out localOffset);
        using (RecCall(w, nameof(TryResolveGrabAttachment))) w.Str(boneName);
        RecEnter();
        try
        {
            var ok = TryResolveGrabAttachmentCore(boneName, out body, out localOffset);
            using (w.Begin(RecordType.ApiReturn)) { w.Str(nameof(TryResolveGrabAttachment)); w.Bool(ok); w.I32(body.Value); w.V3(localOffset); }
            return ok;
        }
        finally { RecLeave(); }
    }

    internal void SetGrabTuning(int slotId, float maxForce, float maxSpeed, float springFreq)
    {
        var w = rec;
        if (w == null) { SetGrabTuningCore(slotId, maxForce, maxSpeed, springFreq); return; }
        using (RecCall(w, nameof(SetGrabTuning))) { w.I32(slotId); w.F32(maxForce); w.F32(maxSpeed); w.F32(springFreq); }
        RecEnter();
        try { SetGrabTuningCore(slotId, maxForce, maxSpeed, springFreq); }
        finally { RecLeave(); }
    }

    internal void UpdateGrabTarget(int slotId, Vector3 worldTarget)
    {
        var w = rec;
        if (w == null) { UpdateGrabTargetCore(slotId, worldTarget); return; }
        using (RecCall(w, nameof(UpdateGrabTarget))) { w.I32(slotId); w.V3(worldTarget); }
        RecEnter();
        try { UpdateGrabTargetCore(slotId, worldTarget); }
        finally { RecLeave(); }
    }

    internal void RemoveGrabConstraint(int slotId)
    {
        var w = rec;
        if (w == null) { RemoveGrabConstraintCore(slotId); return; }
        using (RecCall(w, nameof(RemoveGrabConstraint))) w.I32(slotId);
        RecEnter();
        try { RemoveGrabConstraintCore(slotId); }
        finally { RecLeave(); }
    }

    internal void RequestDirectedKneelPitchOnReady(float duration, float footForce, float pelvisForce,
        float drop, float forward, float pitchDegrees)
    {
        var w = rec;
        if (w == null) { RequestDirectedKneelPitchOnReadyCore(duration, footForce, pelvisForce, drop, forward, pitchDegrees); return; }
        using (RecCall(w, nameof(RequestDirectedKneelPitchOnReady)))
        {
            w.F32(duration); w.F32(footForce); w.F32(pelvisForce); w.F32(drop); w.F32(forward); w.F32(pitchDegrees);
        }
        RecEnter();
        try { RequestDirectedKneelPitchOnReadyCore(duration, footForce, pelvisForce, drop, forward, pitchDegrees); }
        finally { RecLeave(); }
    }

    internal void RequestProfileDirectedCollapseOnReady(string profileId)
    {
        var w = rec;
        if (w == null) { RequestProfileDirectedCollapseOnReadyCore(profileId); return; }
        using (RecCall(w, nameof(RequestProfileDirectedCollapseOnReady))) w.Str(profileId);
        RecEnter();
        try { RequestProfileDirectedCollapseOnReadyCore(profileId); }
        finally { RecLeave(); }
    }

    internal bool BeginProfileDirectedCollapseSpike(string profileId)
    {
        var w = rec;
        if (w == null) return BeginProfileDirectedCollapseSpikeCore(profileId);
        using (RecCall(w, nameof(BeginProfileDirectedCollapseSpike))) w.Str(profileId);
        RecEnter();
        try { return RecReturn(w, nameof(BeginProfileDirectedCollapseSpike), BeginProfileDirectedCollapseSpikeCore(profileId)); }
        finally { RecLeave(); }
    }

    internal bool BeginDirectedKneelPitchSpike(float duration, float footForce, float pelvisForce,
        float drop, float forward, float pitchDegrees)
    {
        var w = rec;
        if (w == null) return BeginDirectedKneelPitchSpikeCore(duration, footForce, pelvisForce, drop, forward, pitchDegrees);
        using (RecCall(w, nameof(BeginDirectedKneelPitchSpike)))
        {
            w.F32(duration); w.F32(footForce); w.F32(pelvisForce); w.F32(drop); w.F32(forward); w.F32(pitchDegrees);
        }
        RecEnter();
        try
        {
            return RecReturn(w, nameof(BeginDirectedKneelPitchSpike),
                BeginDirectedKneelPitchSpikeCore(duration, footForce, pelvisForce, drop, forward, pitchDegrees));
        }
        finally { RecLeave(); }
    }

    internal void StopDirectedCollapseSpike()
    {
        var w = rec;
        if (w == null) { StopDirectedCollapseSpikeCore(); return; }
        using (RecCall(w, nameof(StopDirectedCollapseSpike))) { }
        RecEnter();
        try { StopDirectedCollapseSpikeCore(); }
        finally { RecLeave(); }
    }

    internal void RequestEntryConditionedKneePowerLossOnReady()
    {
        var w = rec;
        if (w == null) { RequestEntryConditionedKneePowerLossOnReadyCore(); return; }
        using (RecCall(w, nameof(RequestEntryConditionedKneePowerLossOnReady))) { }
        RecEnter();
        try { RequestEntryConditionedKneePowerLossOnReadyCore(); }
        finally { RecLeave(); }
    }

    internal void RequestKneePowerLossForwardOnReady()
    {
        var w = rec;
        if (w == null) { RequestKneePowerLossForwardOnReadyCore(); return; }
        using (RecCall(w, nameof(RequestKneePowerLossForwardOnReady))) { }
        RecEnter();
        try { RequestKneePowerLossForwardOnReadyCore(); }
        finally { RecLeave(); }
    }

    internal bool BeginKneePowerLossForwardPattern()
    {
        var w = rec;
        if (w == null) return BeginKneePowerLossForwardPatternCore();
        using (RecCall(w, nameof(BeginKneePowerLossForwardPattern))) { }
        RecEnter();
        try { return RecReturn(w, nameof(BeginKneePowerLossForwardPattern), BeginKneePowerLossForwardPatternCore()); }
        finally { RecLeave(); }
    }

    internal void StopKneePowerLossPattern()
    {
        var w = rec;
        if (w == null) { StopKneePowerLossPatternCore(); return; }
        using (RecCall(w, nameof(StopKneePowerLossPattern))) { }
        RecEnter();
        try { StopKneePowerLossPatternCore(); }
        finally { RecLeave(); }
    }

    /// <param name="anchorBoneName">The anchor bone; null = the humanoid profile's pelvis.</param>
    internal bool CreateStandingSupport(Vector3 anchorWorldPos, Quaternion uprightRot, string? anchorBoneName = null)
    {
        anchorBoneName ??= skeletonProfile.Landmarks.Pelvis;
        var w = rec;
        if (w == null) return CreateStandingSupportCore(anchorWorldPos, uprightRot, anchorBoneName);
        using (RecCall(w, nameof(CreateStandingSupport))) { w.V3(anchorWorldPos); w.Quat(uprightRot); w.Str(anchorBoneName); }
        RecEnter();
        try { return RecReturn(w, nameof(CreateStandingSupport), CreateStandingSupportCore(anchorWorldPos, uprightRot, anchorBoneName)); }
        finally { RecLeave(); }
    }

    /// <param name="anchorBoneName">The anchor bone; null = the humanoid profile's pelvis.</param>
    internal bool UpdateStandingSupport(Vector3 anchorWorldPos, Quaternion uprightRot, string? anchorBoneName = null)
    {
        anchorBoneName ??= skeletonProfile.Landmarks.Pelvis;
        var w = rec;
        if (w == null) return UpdateStandingSupportCore(anchorWorldPos, uprightRot, anchorBoneName);
        using (RecCall(w, nameof(UpdateStandingSupport))) { w.V3(anchorWorldPos); w.Quat(uprightRot); w.Str(anchorBoneName); }
        RecEnter();
        try { return RecReturn(w, nameof(UpdateStandingSupport), UpdateStandingSupportCore(anchorWorldPos, uprightRot, anchorBoneName)); }
        finally { RecLeave(); }
    }

    internal void RemoveStandingSupport()
    {
        var w = rec;
        if (w == null) { RemoveStandingSupportCore(); return; }
        using (RecCall(w, nameof(RemoveStandingSupport))) { }
        RecEnter();
        try { RemoveStandingSupportCore(); }
        finally { RecLeave(); }
    }

    internal bool CreateWristConstraints(Vector3 leftTarget, Vector3 rightTarget,
        float force = 500f, float freq = 80f)
    {
        var w = rec;
        if (w == null) return CreateWristConstraintsCore(leftTarget, rightTarget, force, freq);
        using (RecCall(w, nameof(CreateWristConstraints))) { w.V3(leftTarget); w.V3(rightTarget); w.F32(force); w.F32(freq); }
        RecEnter();
        try { return RecReturn(w, nameof(CreateWristConstraints), CreateWristConstraintsCore(leftTarget, rightTarget, force, freq)); }
        finally { RecLeave(); }
    }

    internal bool UpdateWristConstraints(Vector3 leftTarget, Vector3 rightTarget)
    {
        var w = rec;
        if (w == null) return UpdateWristConstraintsCore(leftTarget, rightTarget);
        using (RecCall(w, nameof(UpdateWristConstraints))) { w.V3(leftTarget); w.V3(rightTarget); }
        RecEnter();
        try { return RecReturn(w, nameof(UpdateWristConstraints), UpdateWristConstraintsCore(leftTarget, rightTarget)); }
        finally { RecLeave(); }
    }

    internal void RemoveWristConstraints()
    {
        var w = rec;
        if (w == null) { RemoveWristConstraintsCore(); return; }
        using (RecCall(w, nameof(RemoveWristConstraints))) { }
        RecEnter();
        try { RemoveWristConstraintsCore(); }
        finally { RecLeave(); }
    }

    internal void ApplyImpulse(string boneName, Vector3 velocityDelta)
    {
        var w = rec;
        if (w == null) { ApplyImpulseCore(boneName, velocityDelta); return; }
        using (RecCall(w, nameof(ApplyImpulse))) { w.Str(boneName); w.V3(velocityDelta); }
        RecEnter();
        try { ApplyImpulseCore(boneName, velocityDelta); }
        finally { RecLeave(); }
    }

    internal void ApplyAngularVelocity(string boneName, Vector3 angularVelocityDelta)
    {
        var w = rec;
        if (w == null) { ApplyAngularVelocityCore(boneName, angularVelocityDelta); return; }
        using (RecCall(w, nameof(ApplyAngularVelocity))) { w.Str(boneName); w.V3(angularVelocityDelta); }
        RecEnter();
        try { ApplyAngularVelocityCore(boneName, angularVelocityDelta); }
        finally { RecLeave(); }
    }

    internal bool PuntNearest(Vector3 from, float maxDist, float strength, float upBias = 0.5f)
    {
        var w = rec;
        if (w == null) return PuntNearestCore(from, maxDist, strength, upBias);
        using (RecCall(w, nameof(PuntNearest))) { w.V3(from); w.F32(maxDist); w.F32(strength); w.F32(upBias); }
        RecEnter();
        try { return RecReturn(w, nameof(PuntNearest), PuntNearestCore(from, maxDist, strength, upBias)); }
        finally { RecLeave(); }
    }

    internal void ClearNpcTraversalSupport(nint npcAddress)
    {
        var w = rec;
        if (w == null) { ClearNpcTraversalSupportCore(npcAddress); return; }
        using (RecCall(w, nameof(ClearNpcTraversalSupport))) w.U64((ulong)npcAddress);
        RecEnter();
        try { ClearNpcTraversalSupportCore(npcAddress); }
        finally { RecLeave(); }
    }

    internal bool TryGetNpcTraversalRootHeight(
        nint npcAddress,
        Vector3 proposedRootPosition,
        float terrainY,
        float maxClimb,
        out float rootY,
        int preferredCorpseBodyHandle = -1)
    {
        var w = rec;
        if (w == null)
            return TryGetNpcTraversalRootHeightCore(npcAddress, proposedRootPosition, terrainY, maxClimb, out rootY, preferredCorpseBodyHandle);
        using (RecCall(w, nameof(TryGetNpcTraversalRootHeight)))
        {
            w.U64((ulong)npcAddress); w.V3(proposedRootPosition); w.F32(terrainY); w.F32(maxClimb); w.I32(preferredCorpseBodyHandle);
        }
        RecEnter();
        try
        {
            var ok = TryGetNpcTraversalRootHeightCore(npcAddress, proposedRootPosition, terrainY, maxClimb, out rootY, preferredCorpseBodyHandle);
            using (w.Begin(RecordType.ApiReturn)) { w.Str(nameof(TryGetNpcTraversalRootHeight)); w.Bool(ok); w.F32(rootY); }
            return ok;
        }
        finally { RecLeave(); }
    }

    internal void DriveNpcTraversalProxy(
        nint address,
        Vector3 initialRoot,
        Vector3 targetRoot,
        float floorY,
        float visualScale,
        float moveSpeed,
        float maxClimb,
        float stompResponse,
        int concurrentStompers,
        bool locomoting)
    {
        var w = rec;
        if (w == null)
        {
            DriveNpcTraversalProxyCore(address, initialRoot, targetRoot, floorY, visualScale, moveSpeed, maxClimb,
                stompResponse, concurrentStompers, locomoting);
            return;
        }
        using (RecCall(w, nameof(DriveNpcTraversalProxy)))
        {
            w.U64((ulong)address); w.V3(initialRoot); w.V3(targetRoot); w.F32(floorY); w.F32(visualScale); w.F32(moveSpeed);
            w.F32(maxClimb); w.F32(stompResponse); w.I32(concurrentStompers); w.Bool(locomoting);
        }
        RecEnter();
        try
        {
            DriveNpcTraversalProxyCore(address, initialRoot, targetRoot, floorY, visualScale, moveSpeed, maxClimb,
                stompResponse, concurrentStompers, locomoting);
        }
        finally { RecLeave(); }
    }

    internal void ReleaseNpcTraversalProxy(nint address)
    {
        var w = rec;
        if (w == null) { ReleaseNpcTraversalProxyCore(address); return; }
        using (RecCall(w, nameof(ReleaseNpcTraversalProxy))) w.U64((ulong)address);
        RecEnter();
        try { ReleaseNpcTraversalProxyCore(address); }
        finally { RecLeave(); }
    }

    internal bool AddLiveCollider(nint address)
    {
        var w = rec;
        if (w == null) return AddLiveColliderCore(address);
        using (RecCall(w, nameof(AddLiveCollider))) w.U64((ulong)address);
        RecEnter();
        try { return RecReturn(w, nameof(AddLiveCollider), AddLiveColliderCore(address)); }
        finally { RecLeave(); }
    }

    internal void SetLiveColliderLightweight(nint address, bool lightweight)
    {
        var w = rec;
        if (w == null) { SetLiveColliderLightweightCore(address, lightweight); return; }
        using (RecCall(w, nameof(SetLiveColliderLightweight))) { w.U64((ulong)address); w.Bool(lightweight); }
        RecEnter();
        try { SetLiveColliderLightweightCore(address, lightweight); }
        finally { RecLeave(); }
    }

    internal void SetLiveColliderTraversalGhost(nint address, bool ghost)
    {
        var w = rec;
        if (w == null) { SetLiveColliderTraversalGhostCore(address, ghost); return; }
        using (RecCall(w, nameof(SetLiveColliderTraversalGhost))) { w.U64((ulong)address); w.Bool(ghost); }
        RecEnter();
        try { SetLiveColliderTraversalGhostCore(address, ghost); }
        finally { RecLeave(); }
    }

    internal void RemoveLiveCollider(nint address)
    {
        var w = rec;
        if (w == null) { RemoveLiveColliderCore(address); return; }
        using (RecCall(w, nameof(RemoveLiveCollider))) w.U64((ulong)address);
        RecEnter();
        try { RemoveLiveColliderCore(address); }
        finally { RecLeave(); }
    }

    internal void CancelAttackStrike()
    {
        var w = rec;
        if (w == null) { CancelAttackStrikeCore(); return; }
        using (RecCall(w, nameof(CancelAttackStrike))) { }
        RecEnter();
        try { CancelAttackStrikeCore(); }
        finally { RecLeave(); }
    }

    internal void ApplyShake(float intensity)
    {
        var w = rec;
        if (w == null) { ApplyShakeCore(intensity); return; }
        using (RecCall(w, nameof(ApplyShake))) w.F32(intensity);
        RecEnter();
        try { ApplyShakeCore(intensity); }
        finally { RecLeave(); }
    }

    internal void AttachWeaponContactBody(BodyHandle weapon)
    {
        var w = rec;
        if (w == null) { AttachWeaponContactBodyCore(weapon); return; }
        using (RecCall(w, nameof(AttachWeaponContactBody))) w.I32(weapon.Value);
        RecEnter();
        try { AttachWeaponContactBodyCore(weapon); }
        finally { RecLeave(); }
    }

    internal void DetachWeaponContactBody(BodyHandle weapon)
    {
        var w = rec;
        if (w == null) { weaponContact.Detach(weapon); return; }
        using (RecCall(w, nameof(DetachWeaponContactBody))) w.I32(weapon.Value);
        RecEnter();
        try { weaponContact.Detach(weapon); }
        finally { RecLeave(); }
    }

    /// <summary>Recorded since stage A2b: the weapon-drop simulation asks it from outside, and a capture it makes
    /// stays in this controller (its hair can rest on that surface).</summary>
    internal FrozenMeshGeometryCache? GetWeaponContactMesh(nint address)
    {
        var w = rec;
        if (w == null) return meshVolumes.GetWeaponContactMesh(address);
        using (RecCall(w, nameof(GetWeaponContactMesh))) w.U64((ulong)address);
        RecEnter();
        try
        {
            var cache = meshVolumes.GetWeaponContactMesh(address);
            using (w.Begin(RecordType.ApiReturn)) { w.Str(nameof(GetWeaponContactMesh)); w.Bool(cache != null); }
            return cache;
        }
        finally { RecLeave(); }
    }
}

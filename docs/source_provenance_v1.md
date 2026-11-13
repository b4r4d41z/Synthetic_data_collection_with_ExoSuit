# Exact Gateway input provenance, Decoupled V4

MCAP log time is recorder receipt time. The Gateway and recorder independently
receive operator inputs, so the recorder's latest source at Action receipt need
not be the source buffered when the Gateway built that Action. The reported
one-encoder-count discrepancy is not evidence for an acceptable target tolerance;
its exact runtime cause is not established by this patch or replayed here.

## Action and evidence protocol

The existing 26 target dimensions, order, units, producer/action identity and
15 Hz simulation schedule are unchanged. Two metadata fields are appended to
`dynamic_biped/msg/BfmAction`; existing fields retain their ordering:

| Field | Meaning |
|---|---|
| `uint32 source_provenance_version` | `1` for the Gateway operator-input protocol; `0` means no operator evidence claimed |
| `string source_provenance_sha256` | Lowercase SHA-256 of the matching trace's exact UTF-8 `String.data` bytes; empty when version is zero |

The Gateway publishes one `std_msgs/msg/String` JSON bundle per Action on
`/bfm/source/input_provenance`. Its QoS is best-effort/keep-last/depth 100;
no ACK, subscriber wait, controller change or freshness gate is introduced.
The recorder includes this topic; custom BFM collection topic lists must retain it.
Missing evidence due to loss remains a verification failure, never a reason to
infer the input from nearby timestamps. Action/trace delivery order is irrelevant.

The complete JSON v1 schema has exactly these fields:

| Object | Fields |
|---|---|
| Root | `protocol="synthetic.gateway_input"`, integer `version=1`, `producer_session_id`, positive uint64 `action_id`, `decision_stamp`, `arm`, `hand` |
| `decision_stamp` | `sec`, `nanosec`, exactly equal to Action.header.stamp (Isaac simulation clock) |
| `arm` | `kind="arm"`, positive uint64 `input_sequence`, `topic`, `header`, `joint_names[14]`, `position_rad[14]`, `processing="per_joint_saturation_rad"`, `bounds_source_sha256` |
| `hand` | `kind="hand"`, positive uint64 `input_sequence`, `topic`, `header`, `left_hand_position[6]`, `right_hand_position[6]` |
| Each source `header` | Original `stamp_sec`, `stamp_nanosec`, `frame_id` from the selected source message |

Arm topic is `/bfm/source/arm_joint_target_rad`; names/positions preserve the
received order and **pre-clamp** requested values. The bounds hash is the
existing authoritative model hash in episode `bfm.arm_target_processing`.
Hand topic is the actual configured `HAND_INPUT_TOPIC`; values precede finger
interpolation. Values are finite binary64 numbers, with hands in 0..100. Headers
are preserved source-native fields, not invented IDs or a common clock.

The producer session is the Gateway's existing UUID. Independent arm/hand
sequences increment on each valid input callback, even for identical values and
repeated timestamps. STOP, episode boundaries and clock rollback clear buffers
but never reset these counters or action IDs. A new Gateway process gets a new
session. Each sample identity is `(producer_session_id, kind, input_sequence)`.
Each Action references its selected pair through the hashed trace. Input updates
not selected by any Action need no bundle, so gaps in sequence numbers are valid.
Parse uint64 identities as exact integers, never rounded binary64 numbers.
Full-rate source/controller topics continue recording independently.

The selected records repeat in every Action bundle. Thus an unchanged old input
remains verifiable even if recording began after its original callback. Trace
generation freezes values/IDs before publication; later callbacks cannot change
that Action. JSON emission uses Python `json.dumps(sort_keys=True,
separators=(',', ':'), ensure_ascii=False, allow_nan=False)` with numeric targets
normalized to float. Hash the **recorded bytes**, not a reserialized JSON object.

These identities prove which inputs the Gateway reports selecting. They do not
identify a unique upstream ROS1 publication: upstream has no supplied unique
sample ID. Unused source fields (arm velocity/effort) are not target inputs and
are not copied. SHA-256 provides consistency/integrity, not publisher authenticity
or evidence of physical execution/attainment.

## Converter compatibility contract

New metadata adds `bfm.operator_input_provenance` (protocol, version, topic,
identity scope, digest and failure rules); contract_version remains 4 and
control_architecture remains `decoupled_teleoperation_v1`. Clock metadata gains
the trace clock/identity distinction. Old recordings are not rewritten.

1. Decode each MCAP channel with its embedded schema. Index all trace records
   before matching Actions; do not require trace-before-Action recorder ordering.
2. Match `(producer_session_id, action_id)` and verify the Action's nonempty
   SHA-256 against the exact `String.data` UTF-8 bytes. Require protocol/version 1,
   valid shapes, finite values, source identities and equal decision stamp.
3. Use the **recorded** `bfm.arm_target_processing` limits and model hash. Saturate
   arm values by name; flatten left_arm7/left_hand6/right_arm7/right_hand6. Compare
   all target binary64 values exactly, without encoder-count tolerance. Original
   source order is not the canonical order; headers/log times do not choose inputs.
4. An older selected source is valid when its explicit trace verifies. Repeated
   identical traces may be deduplicated; conflicting Action keys or inconsistent
   reuse of an input identity must fail. Missing, corrupt, unsupported or mismatched
   evidence rejects the affected Action/recording under the converter's strict
   admission policy. Never fall back to the latest causal MCAP input.
   A valid unreferenced trace may be an orphan at a recording boundary; retain
   it for audit but never synthesize an Action or transition from it.
5. Policy producers using this updated message leave version `0` and empty digest;
   these are explicitly outside operator provenance and must not get fabricated
   input IDs. Old schemas also lack this proof: use a separate legacy/unverified
   profile or reject them, never silently label them verified v1. A new operator
   demonstration requiring v1 cannot accept unclaimed/version-zero Actions.

`isaac_eff_control/source_provenance.py` provides a pure-Python reference verifier:
instantiate `InputProvenanceIndex(recorded_arm_target_processing)`, feed decoded
`String.data` to `add_trace()`, then call `verify(decoded_action)`. Keep one index
across the bag/producer sessions. It accepts out-of-order evidence, deduplicates
identical traces and fails closed on conflicts. It does not change observation
sampling, transition synchronization, state, success or controller semantics.
No changes were made to the separate BFM repository; its converter must adopt
this protocol rather than the recorder-latest-input equality heuristic.

## Build and validation

Rebuild the authoritative ROS2 interface and Python package, then restart Isaac,
Gateway, recorder and any external policy process using BfmAction:

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit/exo2isaacsim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dynamic_biped isaac_eff_control --symlink-install
source install/setup.bash
```

All live BfmAction producers/consumers must use the same rebuilt type definition;
appending fields changes ROS2 wire/type identity. No new CMake dependency is
needed (builtin integer/string plus already-declared std_msgs). ROS1 bridge and
its hand/arm message types are unchanged; no Docker bridge rebuild is needed.
Standard launch commands, manual PLAY and recording lifecycle remain unchanged.

Check `ros2 topic info /bfm/source/input_provenance --verbose`, inspect a trace
with `ros2 topic echo`, and confirm nonzero trace counts in a short new bag plus
v1 metadata in episode.json. Exercise STOP/PLAY, clock reset, reused inputs and
verify the recorded Action/trace pairs offline. Host tests include real MCAP/CDR
encoding/decoding with reordered receipt times, but do not claim live DDS/Isaac
execution or replay of the reported failed recording.

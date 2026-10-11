# Parameter changes (T0-18 / H5)

## Core transactions

The owner approved requiring a recent, explicitly disarmed autopilot heartbeat before
parameter writes. `Core::write_params(target, edits, snapshot_path)` implements this for
ordinary parameters. Its fixed freshness limit is **1.5 seconds**, independent of the
displayed link state or `CoreConfig::link_stale`. The target must be a specific system
and its autopilot component (component 1).

The core downloads a complete baseline, validates the whole edit batch, and creates a
new snapshot file before sending the first `PARAM_SET`. Existing files are never
overwritten. The file contents are synced on all platforms; its directory is also synced
on Unix. A download, validation or storage failure sends no parameter sets.

Every set, retry and explicit read request checks the current disarmed heartbeat. During
actual writes, armed reports and heartbeat gaps latch an interruption even if a later
heartbeat is disarmed again. Read-only baseline preparation may experience gaps; writes
start only after state is checked afresh. Once interrupted, the write transaction does
not resume by itself. A matching set echo
starts `PARAM_REQUEST_READ`; success requires matching name, value and type from the
target component. Retries stop after **10 seconds per parameter**. MAVLink has no request
identifier for parameter responses, so the core cannot distinguish a delayed identical
echo from the explicit read response; it always sends the explicit request before it
accepts the subsequent matching response.

`ParamReport::verified()` is true only when every planned edit is verified and the batch
was not interrupted. On the first mismatch, timeout or interruption the batch stops;
`results` identifies attempted parameters and `planned` also counts those not attempted.
The baseline remains available for recovery. Already-transmitted sets may still reach
the vehicle; stopping further writes does not undo them.

`Core::rollback_params(original_snapshot, names, recovery_snapshot)` restores only the
named parameters from the original baseline. Pass names attempted by the original
report, including a timed-out write whose outcome is unknown. The core re-downloads
current parameters, refuses missing or changed types, persists a new recovery baseline,
and uses the same fresh-disarmed, risk, lifetime and explicit-read checks. Rollback is
operator-requested; it never starts automatically while the vehicle is armed or unknown.
Write/rollback transactions are serialized across core clones, and concurrent requests
are refused as busy rather than queued. Flight-command delivery remains independent.

`parameter_risk` marks `FS_*`, names containing `_FS_` (including battery and RC
failsafes), `FENCE_*`, `ARMING_*` and `BRD_SAFETY*` as dangerous. This conservative list
is not a complete metadata catalog. The initial core API refuses dangerous edits entirely
until a dedicated typed-confirmation path is implemented; being disarmed does not grant
permission to disable a failsafe or fence.

The last finite, nonnegative `FS_EKF_THRESH` value reported by each autopilot component
sets that vehicle's `NavThresholds.ekf_fail`. Zero retains ArduPilot's disabled-variance-
failsafe semantics. Before a value is reported, the configured threshold is used.

## Offline previews and storage

`protocols::params::snapshot` provides an offline baseline and batch preview for H5.
It performs no vehicle I/O and grants no write or rollback approval.

1. Complete the parameter download for a specific system and component.
2. Call `ParamSnapshot::capture(target, announced_count, downloaded_params)`. Use the
   vehicle's announced count; a partial map's length does not establish completeness.
   The snapshot rejects missing/duplicate indices, broadcast targets, invalid names,
   unsupported types and non-finite or unrepresentable values.
3. Parse an edit file with `parse_param_file`, then call `snapshot.plan(&edits)`.
   The whole batch is validated before a preview is returned. Unknown names are errors;
   unchanged wire values are omitted. Integer values must fit their type and the classic
   protocol's C-cast `f32` representation without fractional truncation or precision loss.
4. Inspect `plan.changes()`: each entry carries the original value, proposed value,
   MAVLink type and index. The snapshot remains unchanged.
5. `plan.rollback()` previews restoration of only this batch's edited parameters,
   retaining the original target and types. It does not restore unrelated parameters,
   send writes, verify a read-back, or report vehicle success.
6. `plan.compare(&later_snapshot)` reports missing parameters, clamped/different values
   and changed types in a later complete download. A different target is an error.
   An empty difference list means the supplied data agrees; the transaction driver
   must establish that the download follows the actual write operation.

`snapshot.to_text()` and `ParamSnapshot::from_text()` preserve the target, parameter
count, original float bits, types and indices across restarts. The versioned text format
has a SHA-256 trailer to detect accidental corruption; it is not a Mission Planner
`.param` file, an authentication signature, or permission to restore a vehicle. Loading
also revalidates completeness and value/type constraints.

Validation:

Stop a documentation server on port 8000 before the full SITL suite (or serve docs on
port 18000). SITL instance 224 uses TCP port 8000; a conflict can prevent a simulator
from starting even though the application code is correct.

```sh
cargo test -p protocols --locked --test param_snapshot
cargo test -p dc-core --locked --test parameters
DC_SITL_REQUIRED=1 cargo test -p sitl-harness --locked --test scenarios \
  parameter_write_and_rollback_are_verified_through_30_percent_loss -- --nocapture
```

These tests exercise baseline completeness, name and type validity, integer limits and
wire precision, whole-batch rejection, no-op edits, and restoration of edited values
while an unrelated parameter retains a concurrent change. Core tests exercise refusal,
interruption, snapshot-before-write ordering, explicit reads, mismatches, timeout,
source filtering and selected restoration. The SITL scenario changes `WP_SPD` on the
pinned simulator and verifies its restoration with 30% bidirectional frame loss; it
requests 5 Hz heartbeats rather than weakening the 1.5 s freshness rule.

# Parameter changes (T0-18 groundwork)

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
   An empty difference list means the supplied data agrees; the future write driver
   must establish that the download is fresh and follows the actual write operation.

`snapshot.to_text()` and `ParamSnapshot::from_text()` preserve the target, parameter
count, original float bits, types and indices across restarts. The versioned text format
has a SHA-256 trailer to detect accidental corruption; it is not a Mission Planner
`.param` file, an authentication signature, or permission to restore a vehicle. Loading
also revalidates completeness and value/type constraints.

This groundwork does **not** complete T0-18.
Before enabling writes, the core still needs a persisted pre-write snapshot, serialized
transactions, target/component checks, explicit read-back verification, honest mismatch
and timeout reports, and restoration verified through the same write path. The existing
`ParamWrite` state machine is not yet exposed through the core.

The owner's armed-state rule is pending, as required by the handover. Failsafe/fence risk
labels, dangerous-parameter confirmation and following the vehicle's `FS_EKF_THRESH`
also remain outstanding. No write, rollback, interlock or navigation policy is changed
by this offline module.

Validation:

```sh
cargo test -p protocols --locked --test param_snapshot
```

These tests exercise baseline completeness, name and type validity, integer limits and
wire precision, whole-batch rejection, no-op edits, and restoration of edited values
while an unrelated parameter retains a concurrent change. They are not SITL write tests.

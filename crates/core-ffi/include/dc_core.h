/*
 * dc_core.h — C ABI for the DrorControl headless core (crate `core-ffi`).
 *
 * Ownership: dc_core_new() returns a handle the caller owns; release it with
 * dc_core_free(). Strings passed in are borrowed for the duration of the call.
 *
 * Threading: every function may be called from any thread. The core runs its own worker
 * threads; dc_core_telemetry() reads a lock-free snapshot and never blocks, so it is safe
 * to call once per rendered frame from a UI thread.
 *
 * Versioning: check dc_abi_version() == DC_ABI_VERSION at load time. Structs are only
 * extended at the end; `struct_size` tells the caller how much the library filled in.
 */
#ifndef DC_CORE_H
#define DC_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DC_ABI_VERSION 1u

/* Error codes (negative return values). */
#define DC_ERR_NULL        (-1) /* a required pointer was NULL            */
#define DC_ERR_ARGUMENT    (-2) /* malformed argument (e.g. link spec)     */
#define DC_ERR_IO          (-3) /* could not open the link                 */
#define DC_ERR_NOT_FOUND   (-4) /* no such vehicle                         */

/* DcTelemetry.flags */
#define DC_TELEM_HEARTBEAT (1u << 0)
#define DC_TELEM_ATTITUDE  (1u << 1)
#define DC_TELEM_POSITION  (1u << 2)
#define DC_TELEM_BATTERY   (1u << 3)
#define DC_TELEM_ARMED     (1u << 4)

/* Latest state of one vehicle. A group is valid only if its flag is set; *_age_ms is
 * the time since it was last updated — UIs must flag stale data rather than show it
 * as live. Units: radians, degrees (lat/lon), metres, volts. */
typedef struct DcTelemetry {
    uint32_t struct_size;
    uint8_t  sysid;
    uint8_t  flags;
    uint8_t  mav_type;
    uint8_t  reserved0;
    uint32_t custom_mode;      /* ArduPilot flight mode number */
    uint32_t heartbeat_age_ms;
    float    roll_rad;
    float    pitch_rad;
    float    yaw_rad;
    uint32_t attitude_age_ms;
    double   lat_deg;
    double   lon_deg;
    float    alt_rel_m;
    uint32_t position_age_ms;
    float    battery_v;
    uint32_t battery_age_ms;
} DcTelemetry;

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(DcTelemetry) == 64, "DcTelemetry layout");
#elif defined(__cplusplus)
static_assert(sizeof(DcTelemetry) == 64, "DcTelemetry layout");
#endif

typedef struct DcCore DcCore; /* opaque */

uint32_t dc_abi_version(void);

/* Start a core with our MAVLink ids (255/190 are the GCS defaults). NULL on failure. */
DcCore *dc_core_new(uint8_t sysid, uint8_t compid);

/* Stop the core and release the handle. NULL is ignored. */
void dc_core_free(DcCore *core);

/* Open a link, e.g. "tcp:127.0.0.1:5760", "udp:0.0.0.0:14550",
 * "serial:/dev/cu.usbmodem1101:115200". Returns the link id (>= 0) or DC_ERR_*. */
int32_t dc_core_connect(DcCore *core, const char *spec);

/* Write up to `cap` system ids of vehicles heard into `out`; returns how many vehicles
 * exist (may exceed `cap`), or DC_ERR_NULL. */
int32_t dc_core_vehicles(const DcCore *core, uint8_t *out, size_t cap);

/* Fill `out` with the latest state of `sysid`. Returns 0 or DC_ERR_*. */
int32_t dc_core_telemetry(const DcCore *core, uint8_t sysid, DcTelemetry *out);

#ifdef __cplusplus
}
#endif

#endif /* DC_CORE_H */

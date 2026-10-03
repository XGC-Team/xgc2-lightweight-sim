#ifndef XGC_LIGHTWEIGHT_SIMULATION_RECORDS_V1_H
#define XGC_LIGHTWEIGHT_SIMULATION_RECORDS_V1_H

#include <stdint.h>

/* "xgc.fcu_request/2": simulation FCU request with result correlation.
 * /1 remains the unchanged physical service-client boundary.
 * flags bit 0 preserves forced disarm; a model may explicitly refuse it. */
typedef struct xgc_fcu_request_v2 {
  double stamp;
  uint32_t kind;
  uint32_t arm;
  char mode[32];
  uint64_t request_id;
  uint32_t flags;
  uint32_t reserved;
} xgc_fcu_request_v2;

/* "xgc.fcu_result/1": event, one executed request in a six-robot batch.
 * result is MAV_RESULT: 0 accepted, 1 temporary rejection, 2 denied,
 * 3 unsupported, 4 failed. No successful result is produced on mere enqueue. */
typedef struct xgc_fcu_result_v1 {
  double stamp;
  double request_stamp;
  uint64_t request_id;
  uint32_t robot_index;
  uint32_t kind;
  uint32_t result;
  uint32_t reserved;
} xgc_fcu_result_v1;

/* "xgc.fcu_extended_state/1": one same-stamp batch snapshot.
 * landed_state uses MAV_LANDED_STATE; vtol_state=0 for these quadrotors. */
typedef struct xgc_fcu_extended_state_v1 {
  double stamp;
  uint32_t count;
  uint32_t reserved;
  uint8_t landed_state[6];
  uint8_t vtol_state[6];
  uint8_t reserved_tail[4];
} xgc_fcu_extended_state_v1;

/* World-owned simulation provider lifecycle. Actions: 0 observe, 1 start CAS, 2 stop.
 * Start inactive requires expected current generation; active retry accepts
 * current or its predecessor. Stop is fenced by current generation. Results
 * always report current generation/enabled, including rejected old requests.
 * Re-start after stop resets only this body at a model advance boundary. */
typedef struct xgc_sim_provider_request_v1 {
  double stamp;
  uint64_t request_id;
  uint64_t generation;
  uint32_t robot_index;
  uint32_t action;
} xgc_sim_provider_request_v1;

typedef struct xgc_sim_provider_result_v1 {
  double stamp;
  uint64_t request_id;
  uint64_t generation;
  uint32_t robot_index;
  uint32_t accepted;
  uint32_t enabled;
  uint32_t reason; /* 0 accepted, 1 stale generation, 2 invalid action/slot */
} xgc_sim_provider_result_v1;

#ifdef __cplusplus
static_assert(sizeof(xgc_fcu_request_v2) == 64, "xgc_fcu_request_v2");
static_assert(sizeof(xgc_fcu_result_v1) == 40, "xgc_fcu_result_v1");
static_assert(sizeof(xgc_fcu_extended_state_v1) == 32, "xgc_fcu_extended_state_v1");
static_assert(sizeof(xgc_sim_provider_request_v1) == 32, "xgc_sim_provider_request_v1");
static_assert(sizeof(xgc_sim_provider_result_v1) == 40, "xgc_sim_provider_result_v1");
#else
_Static_assert(sizeof(xgc_fcu_request_v2) == 64, "xgc_fcu_request_v2");
_Static_assert(sizeof(xgc_fcu_result_v1) == 40, "xgc_fcu_result_v1");
_Static_assert(sizeof(xgc_fcu_extended_state_v1) == 32, "xgc_fcu_extended_state_v1");
_Static_assert(sizeof(xgc_sim_provider_request_v1) == 32, "xgc_sim_provider_request_v1");
_Static_assert(sizeof(xgc_sim_provider_result_v1) == 40, "xgc_sim_provider_result_v1");
#endif

#endif

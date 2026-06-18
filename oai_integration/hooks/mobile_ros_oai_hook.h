#ifndef MOBILE_ROS_OAI_HOOK_H
#define MOBILE_ROS_OAI_HOOK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mobile_ros_oai_metrics_s {
  double timestamp;
  uint16_t rnti;
  double sinr_db;
  double rsrp_dbm;
  double rsrq_db;
  double prb_util;
  int mcs;
  double throughput_mbps;
  double latency_ms;
  double packet_loss_pct;
  double jitter_ms;
} mobile_ros_oai_metrics_t;

void mobile_ros_oai_publish_metrics(const mobile_ros_oai_metrics_t *metrics);
void mobile_ros_oai_apply_prb_weight(uint16_t rnti, double weight, int min_prb, int max_prb);

#ifdef __cplusplus
}
#endif

#endif

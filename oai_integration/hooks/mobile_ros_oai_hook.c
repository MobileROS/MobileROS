#include "mobile_ros_oai_hook.h"

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static int mobile_ros_sock = -1;
static struct sockaddr_in mobile_ros_addr;

static void mobile_ros_init_udp(void)
{
  if (mobile_ros_sock >= 0)
    return;

  const char *host = getenv("MOBILEROS_METRIC_HOST");
  const char *port_text = getenv("MOBILEROS_METRIC_PORT");
  if (!host)
    host = "127.0.0.1";
  int port = port_text ? atoi(port_text) : 62000;

  mobile_ros_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (mobile_ros_sock < 0)
    return;

  memset(&mobile_ros_addr, 0, sizeof(mobile_ros_addr));
  mobile_ros_addr.sin_family = AF_INET;
  mobile_ros_addr.sin_port = htons((uint16_t)port);
  inet_pton(AF_INET, host, &mobile_ros_addr.sin_addr);
}

void mobile_ros_oai_publish_metrics(const mobile_ros_oai_metrics_t *metrics)
{
  if (!metrics)
    return;
  mobile_ros_init_udp();
  if (mobile_ros_sock < 0)
    return;

  char payload[1024];
  int n = snprintf(payload,
                   sizeof(payload),
                   "{\"timestamp\":%.6f,\"rnti\":\"0x%04x\",\"sinr_db\":%.3f,"
                   "\"rsrp_dbm\":%.3f,\"rsrq_db\":%.3f,\"prb_util\":%.6f,"
                   "\"mcs\":%d,\"throughput_mbps\":%.6f,\"latency_ms\":%.6f,"
                   "\"packet_loss_pct\":%.6f,\"jitter_ms\":%.6f}",
                   metrics->timestamp,
                   metrics->rnti,
                   metrics->sinr_db,
                   metrics->rsrp_dbm,
                   metrics->rsrq_db,
                   metrics->prb_util,
                   metrics->mcs,
                   metrics->throughput_mbps,
                   metrics->latency_ms,
                   metrics->packet_loss_pct,
                   metrics->jitter_ms);
  if (n > 0)
    sendto(mobile_ros_sock, payload, (size_t)n, 0, (struct sockaddr *)&mobile_ros_addr, sizeof(mobile_ros_addr));
}

void mobile_ros_oai_apply_prb_weight(uint16_t rnti, double weight, int min_prb, int max_prb)
{
  (void)rnti;
  (void)weight;
  (void)min_prb;
  (void)max_prb;
}

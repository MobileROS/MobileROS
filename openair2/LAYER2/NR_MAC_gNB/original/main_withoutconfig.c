/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file main.c
 * \brief top init of Layer 2
 * \author  Navid Nikaein and Raymond Knopp, WEI-TAI CHEN
 * \date 2010 - 2014, 2018
 * \version 1.0
 * \company Eurecom, NTUST
 * \email: navid.nikaein@eurecom.fr, kroempa@gmail.com
 * @ingroup _mac

 */

#include "NR_MAC_gNB/mac_proto.h"
#include "NR_MAC_COMMON/nr_mac_extern.h"
#include "assertions.h"
#include "nr_pdcp/nr_pdcp_oai_api.h"

#include "RRC/NR/nr_rrc_defs.h"
#include "common/utils/LOG/log.h"
#include "nr_rlc/nr_rlc_oai_api.h"
#include "RRC/NR/MESSAGES/asn1_msg.h"
//#include "RRC/L2_INTERFACE/openair_rrc_L2_interface.h"

#include "common/ran_context.h"
#include "executables/softmodem-common.h"

#include <time.h>
#include <stdio.h>
#include "curl/curl.h"
extern RAN_CONTEXT_t RC;


#define MACSTATSSTRLEN 16000

//influxdb related
#define INFLUXDB_URL "http://localhost:8086/api/v2/write?org=my_org&bucket=my_bucket&precision=ms"
#define INFLUXDB_TOKEN "LbUOm57nYdJp0trGuaI4TcSTvzeo5yJYsj5GtEC-EvkNmEFC35t4HyrMwWBanDhVRZmVW7iTqqXGTGB82DmG2g=="

// 发送数据到 InfluxDB
void send_to_influxdb(const char *data) {
  CURL *curl = curl_easy_init();
  if (!curl) {
      fprintf(stderr, "Failed to initialize libcurl\n");
      return;
  }

  struct curl_slist *headers = NULL;
  headers = curl_slist_append(headers, "Content-Type: text/plain");
  headers = curl_slist_append(headers, "Accept: application/json");

  // 认证
  char auth_header[512];
  snprintf(auth_header, sizeof(auth_header), "Authorization: Token %s", INFLUXDB_TOKEN);
  headers = curl_slist_append(headers, auth_header);

  curl_easy_setopt(curl, CURLOPT_URL, INFLUXDB_URL);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)strlen(data));

  // 发送请求
  CURLcode res = curl_easy_perform(curl);
  if (res != CURLE_OK) {
      fprintf(stderr, "Failed to send data to InfluxDB: %s\n", curl_easy_strerror(res));
  }

  // 释放资源
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
}

long get_time_milliseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    long ms = ts.tv_sec * 1000 + round(ts.tv_nsec / 1.0e6);
    return ms;
}

/* msleep(): Sleep for the requested number of milliseconds. */
int msleep(long msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

size_t o1_copy_mac_stats_csv(gNB_MAC_INST *gNB, char *output, size_t strlen, bool reset_rsrp)
{
  int num = 1;
  const char *begin = output;
  const char *end = output + strlen;

  pthread_mutex_lock(&gNB->UE_info.mutex);

  UE_iterator(gNB->UE_info.list, UE) {
    NR_UE_sched_ctrl_t *sched_ctrl = &UE->UE_sched_ctrl;
    NR_mac_stats_t *stats = &UE->mac_stats;
    const int avg_rsrp = stats->num_rsrp_meas > 0 ? stats->cumul_rsrp / stats->num_rsrp_meas : 0;


    char ue_imsi[] = "0010100000107XX";
    long timestamp = get_time_milliseconds();
    output += snprintf(output,
            end - output,
            "%ld,%d,%d,%s,%04x,%d,%d,%f,%f, ,%d,%d,%d,%d,%d,%d,%u,%u,%d,%d,%"PRIu32",%"PRIu16", ,%.5f,%d,%"PRIu64",%"PRIu64",%"PRIu32",%"PRIu32",%"PRIu32",%"PRIu32",%"PRIu64",%d,%d,%02d%02d%02d,%"PRIu64",%d,%d,%02d%02d%02d, ,%.5f,%d,%"PRIu64",%"PRIu64",%"PRIu32",%"PRIu32",%"PRIu32",%"PRIu32",%d, ,\n",
            timestamp,
            gNB->frame,
            gNB->slot,
            ue_imsi,
            UE->rnti,
            UE->uid,
            num++,
            UE->dl_thr_ue,
            UE->ul_thr_ue,
            // UE sched ctrl
            sched_ctrl->ph,
            sched_ctrl->pcmax,
            avg_rsrp,
            sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb,
            sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.ri+1,
            sched_ctrl->raw_rssi,
            sched_ctrl->ul_rssi,
            sched_ctrl->dl_max_mcs,
            sched_ctrl->sched_ul_bytes,
            sched_ctrl->estimated_ul_buffer,
            sched_ctrl->num_total_bytes,
            sched_ctrl->dl_pdus_total,
            // DL stats
            sched_ctrl->dl_bler_stats.bler,
            sched_ctrl->dl_bler_stats.mcs,
            stats->dl.errors,
            stats->dl.total_bytes,
            stats->dl.current_bytes,
            stats->dl.total_rbs,
            stats->dl.current_rbs,
            stats->dl.num_mac_sdu,
            stats->dl.lc_bytes[4],
            sched_ctrl->dl_sl_info[4].id,
            sched_ctrl->dl_sl_info[4].nssai_config.sST,
            sched_ctrl->dl_sl_info[4].nssai_config.sD[0],
            sched_ctrl->dl_sl_info[4].nssai_config.sD[1],
            sched_ctrl->dl_sl_info[4].nssai_config.sD[2],
            stats->dl.lc_bytes[5],
            sched_ctrl->dl_sl_info[5].id,
            sched_ctrl->dl_sl_info[5].nssai_config.sST,
            sched_ctrl->dl_sl_info[5].nssai_config.sD[0],
            sched_ctrl->dl_sl_info[5].nssai_config.sD[1],
            sched_ctrl->dl_sl_info[5].nssai_config.sD[2],
            // UL stats
            sched_ctrl->ul_bler_stats.bler,
            sched_ctrl->ul_bler_stats.mcs,
            stats->ul.errors,
            stats->ul.total_bytes,
            stats->ul.current_bytes,
            stats->ul.total_rbs,
            stats->ul.current_rbs,
            stats->ul.num_mac_sdu,
            sched_ctrl->pusch_snrx10  //新加的
            );

    
    //发送influxdb数据的代码放到这里，看起来是ms级别的了, 因为这个函数只会被后边的mac_stats线程调用一次
    //还是有很多信息没有补齐的，暂时用到哪些放哪些
    //总体就是在sched_ctrl stats UE gNB这四个里边找
    char influx_data[1024];
    snprintf(influx_data, sizeof(influx_data),
          "ue_metrics,rnti=%04x in_sync=%d,frame=%d,slot=%d,DL_Thr=%f,UL_Thr=%f,"
          "ph=%d,pcmax=%d,avg_rsrp=%d,num_rsrp_meas=%u,cumul_rsrp=%d,cqi=%d,ri=%d,raw_rssi=%d,"
          "ul_rssi=%u,dl_max_mcs=%u,sched_ul_bytes=%d,estimated_ul_buffer=%d,num_total_bytes=%"PRIu32","
          "dl_pdus_total=%"PRIu16",snr_pusch=%d,dl_bler=%.5f,dl_mcs=%d,dlsch_errors=%"PRIu64","
          "dlsch_total_byte=%"PRIu64",dlsch_current_bytes=%"PRIu32",dl_total_rbs=%"PRIu32",dl_current_rbs=%"PRIu32","
          "dl_num_mac_sdu=%"PRIu32",ul_bler=%.5f,ul_mcs=%d,ulsch_errors=%"PRIu64",ulsch_total_byte=%"PRIu64","
          "ulsch_current_bytes=%"PRIu32",ul_total_rbs=%"PRIu32",ul_current_rbs=%"PRIu32",ul_num_mac_sdu=%"PRIu32",timestamp=%ld\n",
          //通用信息
          UE->rnti,
          !sched_ctrl->ul_failure,
          gNB->frame,
          gNB->slot,
          UE->dl_thr_ue,
          UE->ul_thr_ue,
          //ue schedule ctrl
          sched_ctrl->ph,
          sched_ctrl->pcmax,
          // stats->num_rsrp_meas > 0 ? stats->cumul_rsrp / stats->num_rsrp_meas : 0, //aver_rsrp
          avg_rsrp,
          stats->num_rsrp_meas,  
          stats->cumul_rsrp,
          sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb, //cqi也一直是0
          sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.ri+1,
          sched_ctrl->raw_rssi,
          sched_ctrl->ul_rssi,
          sched_ctrl->dl_max_mcs,
          sched_ctrl->sched_ul_bytes,
          sched_ctrl->estimated_ul_buffer,
          sched_ctrl->num_total_bytes,
          sched_ctrl->dl_pdus_total,
          sched_ctrl->pusch_snrx10,          //放大了10倍的结果
          // sched_ctrl->pusch_snrx10/10,  //SNR CQI之前一直为0，avgr_rsrp也是，不同版本的OAI这一点不同
          // sched_ctrl->pusch_snrx10%10,
          //dl stats dl_sl_info的没有加上，切片的可能会有用吧
          sched_ctrl->dl_bler_stats.bler,
          sched_ctrl->dl_bler_stats.mcs,
          stats->dl.errors,
          stats->dl.total_bytes,
          stats->dl.current_bytes,
          stats->dl.total_rbs,
          stats->dl.current_rbs,
          stats->dl.num_mac_sdu,
          //ul stats        
          sched_ctrl->ul_bler_stats.bler,
          sched_ctrl->ul_bler_stats.mcs,
          stats->ul.errors,
          stats->ul.total_bytes,
          stats->ul.current_bytes,
          stats->ul.total_rbs,
          stats->ul.current_rbs,
          stats->ul.num_mac_sdu,
          timestamp
          );

    //注意，rnti之后没有逗号
    //这样差不多每隔1ms或者2ms上传一次，若是单独写一个发送函数，然后在gNB_dlsch_ulsch_scheduler()调用是否可以做到间隔1ms呢？
    // snprintf(influx_data, sizeof(influx_data),
    //          "ue_metrics,rnti=%04x pusch_snr=%d,timestamp=%ld\n",
    //          UE->rnti,
    //          sched_ctrl->pusch_snrx10,
    //          timestamp);

    // 发送数据到 InfluxDB
    // send_to_influxdb(influx_data);

    //用作测试
    // char influx_data[1024];

    // 格式化 InfluxDB 数据
    // snprintf(influx_data, sizeof(influx_data),
    //         "ue_metrics,rnti=%04x in_sync=%d,ph=%d,pcmax=%d,avg_rsrp=%d,num_rsrp_meas=%d,"
    //         "cqi=%d,ri=%d,pmi_x1=%d,pmi_x2=%d,"
    //         "dlsch_errors=%"PRIu64",ulsch_errors=%"PRIu64",bler=%.5f,"
    //         "tx_bytes=%"PRIu64",rx_bytes=%"PRIu64"\n",
    //         UE->rnti,
    //         !sched_ctrl->ul_failure,
    //         sched_ctrl->ph,
    //         sched_ctrl->pcmax,
    //         stats->num_rsrp_meas > 0 ? stats->cumul_rsrp / stats->num_rsrp_meas : 0,
    //         stats->num_rsrp_meas,
    //         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb,
    //         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.ri + 1,
    //         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.pmi_x1,
    //         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.pmi_x2,
    //         stats->dl.errors,
    //         stats->ul.errors,
    //         sched_ctrl->dl_bler_stats.bler,
    //         stats->dl.total_bytes,
    //         stats->ul.total_bytes);

    // 发送数据到 InfluxDB
    send_to_influxdb(influx_data);
    
    //应该和下边的没什么关系吧
    if (reset_rsrp) {
      stats->num_rsrp_meas = 0;
      stats->cumul_rsrp = 0;
    }
  }

  pthread_mutex_unlock(&gNB->UE_info.mutex);
  return output - begin;
}



void *nrmac_stats_thread(void *arg)
{

  gNB_MAC_INST *gNB = (gNB_MAC_INST *)arg;

  int report_interval = 1;

  char ue_metrics_csv_filepath [] = "ue_metrics_multi_pdu.csv";

  char csv_header[] = "Timestamp, Frame, Slot, IMSI, RNTI, UID, UE_No, DL_THR, UL_THR, ,PH_dB, PCMAX_dBm, AVG_RSRP, CQI, RI, raw_rssi, ul_rssi, dl_max_mcs, sched_ul_bytes, estimated_ul_buffer, num_total_bytes, dl_pdus_total, ,dl_BLER, dl_MCS, dlsch_errors, dlsch_total_bytes, dlsch_current_bytes, dlsch_total_rbs, dlsch_current_rbs, dl_num_mac_sdu, dl_lc4_bytes, dl_lc4_id, dl_lc4_sst,dl_lc4_sd, dl_lc5_bytes, dl_lc5_id, dl_lc5_sst,dl_lc5_sd, ,ul_BLER, ul_MCS, ulsch_errors, ulsch_total_bytes, ulsch_current_bytes, ulsch_total_rbs, ulsch_current_rbs, ul_num_mac_sdu, SNR, ,\n";

  int size = 0;
  char output[MACSTATSSTRLEN] = {0};
  const char *end = output + MACSTATSSTRLEN;

  FILE *file = fopen(ue_metrics_csv_filepath,"a+");

  AssertFatal(file!=NULL,"Cannot open %s, error %s\n",ue_metrics_csv_filepath, strerror(errno));

  if (NULL !=file){
    fseek(file,0,SEEK_END);
    size = ftell(file);
  }
  if (size == 0){
    fprintf(file, "%s", csv_header);
  }
  while (oai_exit == 0) {
    char *p = output;
    NR_SCHED_LOCK(&gNB->sched_lock);
    p += dump_mac_stats(gNB, p, end - p, false);
    NR_SCHED_UNLOCK(&gNB->sched_lock);
    p += snprintf(p, end - p, "\n");
    p += print_meas_log(&gNB->eNB_scheduler, "DL & UL scheduling timing", NULL, NULL, p, end - p);
    p += print_meas_log(&gNB->schedule_dlsch, "dlsch scheduler", NULL, NULL, p, end - p);
    p += print_meas_log(&gNB->rlc_data_req, "rlc_data_req", NULL, NULL, p, end - p);
    p += print_meas_log(&gNB->rlc_status_ind, "rlc_status_ind", NULL, NULL, p, end - p);
    // p += o1_copy_mac_stats_csv(gNB, p, end - p, true);  //原始的
    p += o1_copy_mac_stats_csv(gNB, p, end - p, false);  //这里是否会有影响呢？这样修改后就有rsrp了 但cqi仍为0
    fwrite(output, p - output, 1, file);
    fflush(file);
    msleep(report_interval);
    fseek(file,0,SEEK_SET);
  }
  fclose(file);
  return NULL;
}


// void *nrmac_stats_thread(void *arg) {

//   gNB_MAC_INST *gNB = (gNB_MAC_INST *)arg;

//   char output[MACSTATSSTRLEN] = {0};
//   const char *end = output + MACSTATSSTRLEN;
//   FILE *file = fopen("nrMAC_stats.log","w");
//   AssertFatal(file!=NULL,"Cannot open nrMAC_stats.log, error %s\n",strerror(errno));

//   while (oai_exit == 0) {
//     char *p = output;
//     p += dump_mac_stats(gNB, p, end - p, false);
//     p += snprintf(p, end - p, "\n");
//     p += print_meas_log(&gNB->eNB_scheduler, "DL & UL scheduling timing", NULL, NULL, p, end - p);
//     p += print_meas_log(&gNB->schedule_dlsch, "dlsch scheduler", NULL, NULL, p, end - p);
//     p += print_meas_log(&gNB->rlc_data_req, "rlc_data_req", NULL, NULL, p, end - p);
//     p += print_meas_log(&gNB->rlc_status_ind, "rlc_status_ind", NULL, NULL, p, end - p);
//     fwrite(output, p - output, 1, file);
//     fflush(file);
//     sleep(1);
//     fseek(file,0,SEEK_SET);
//   }
//   fclose(file);
//   return NULL;
// }

void clear_mac_stats(gNB_MAC_INST *gNB) {
  UE_iterator(gNB->UE_info.list, UE) {
    memset(&UE->mac_stats,0,sizeof(UE->mac_stats));
  }
}

size_t dump_mac_stats(gNB_MAC_INST *gNB, char *output, size_t strlen, bool reset_rsrp)
{
  int num = 1;
  const char *begin = output;
  const char *end = output + strlen;

  /* this function is called from gNB_dlsch_ulsch_scheduler(), so assumes the
   * scheduler to be locked*/
  NR_SCHED_ENSURE_LOCKED(&gNB->sched_lock);

  NR_SCHED_LOCK(&gNB->UE_info.mutex);
  UE_iterator(gNB->UE_info.list, UE) {
    NR_UE_sched_ctrl_t *sched_ctrl = &UE->UE_sched_ctrl;
    NR_mac_stats_t *stats = &UE->mac_stats;
    const int avg_rsrp = stats->num_rsrp_meas > 0 ? stats->cumul_rsrp / stats->num_rsrp_meas : 0;

    output += snprintf(output,
                       end - output,
                       "UE RNTI %04x (%d) PH %d dB PCMAX %d dBm, average RSRP %d (%d meas)\n",
                       UE->rnti,
                       num++,
                       sched_ctrl->ph,
                       sched_ctrl->pcmax,
                       avg_rsrp,
                       stats->num_rsrp_meas);

    if(sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.print_report)  //输出就一直是0 不用注释掉
      output += snprintf(output,
                         end - output,
                         "UE %04x: CQI %d, RI %d, PMI (%d,%d)\n",
                         UE->rnti,
                         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb,
                         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.ri+1,
                         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.pmi_x1,
                         sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.pmi_x2);

    if (stats->srs_stats[0] != '\0') {
      output += snprintf(output, end - output, "UE %04x: %s\n", UE->rnti, stats->srs_stats);
    }

    output += snprintf(output,
                       end - output,
                       "UE %04x: dlsch_rounds ", UE->rnti);
    output += snprintf(output, end - output, "%"PRIu64, stats->dl.rounds[0]);
    for (int i = 1; i < gNB->dl_bler.harq_round_max; i++)
      output += snprintf(output, end - output, "/%"PRIu64, stats->dl.rounds[i]);

    output += snprintf(output,
                       end - output,
                       ", dlsch_errors %"PRIu64", pucch0_DTX %d, BLER %.5f MCS %d\n",
                       stats->dl.errors,
                       stats->pucch0_DTX,
                       sched_ctrl->dl_bler_stats.bler,
                       sched_ctrl->dl_bler_stats.mcs);
    if (reset_rsrp) {
      stats->num_rsrp_meas = 0;
      stats->cumul_rsrp = 0;
    }
    output += snprintf(output,
                       end - output,
                       "UE %04x: dlsch_total_bytes %"PRIu64"\n",
                       UE->rnti, stats->dl.total_bytes);
    output += snprintf(output,
                       end - output,
                       "UE %04x: ulsch_rounds ", UE->rnti);
    output += snprintf(output, end - output, "%"PRIu64, stats->ul.rounds[0]);
    for (int i = 1; i < gNB->ul_bler.harq_round_max; i++)
      output += snprintf(output, end - output, "/%"PRIu64, stats->ul.rounds[i]);

    output += snprintf(output,
                       end - output,
                       ", ulsch_DTX %d, ulsch_errors %"PRIu64", BLER %.5f MCS %d\n",
                       stats->ulsch_DTX,
                       stats->ul.errors,
                       sched_ctrl->ul_bler_stats.bler,
                       sched_ctrl->ul_bler_stats.mcs);
    output += snprintf(output,
                       end - output,
                       "UE %04x: ulsch_total_bytes_scheduled %"PRIu64", ulsch_total_bytes_received %"PRIu64"\n",
                       UE->rnti,
                       stats->ulsch_total_bytes_scheduled, stats->ul.total_bytes);

    for (int lc_id = 0; lc_id < 63; lc_id++) {  
      if (stats->dl.lc_bytes[lc_id] > 0)
        output += snprintf(output,
                           end - output,
                           "UE %04x: LCID %d: %"PRIu64" bytes TX\n",
                           UE->rnti,
                           lc_id,
                           stats->dl.lc_bytes[lc_id]);
      if (stats->ul.lc_bytes[lc_id] > 0)
        output += snprintf(output,
                           end - output,
                           "UE %04x: LCID %d: %"PRIu64" bytes RX\n",
                           UE->rnti,
                           lc_id,
                           stats->ul.lc_bytes[lc_id]);     
    }
  }
  NR_SCHED_UNLOCK(&gNB->UE_info.mutex);
  return output - begin;
}


//这个函数用来实现每2个slot调度一次数据库写入指令，2slot应该是1ms，在gNB_dlsch_ulsch_scheduler()中被调用
//目前不太稳定
void influxdb_mac_stats(gNB_MAC_INST *gNB)
{
  int num = 1;
  /* this function is called from gNB_dlsch_ulsch_scheduler(), so assumes the
   * scheduler to be locked*/
  NR_SCHED_ENSURE_LOCKED(&gNB->sched_lock);

  NR_SCHED_LOCK(&gNB->UE_info.mutex);
  UE_iterator(gNB->UE_info.list, UE) {
    NR_UE_sched_ctrl_t *sched_ctrl = &UE->UE_sched_ctrl;
    NR_mac_stats_t *stats = &UE->mac_stats;
    long timestamp = get_time_milliseconds();
    const int avg_rsrp = stats->num_rsrp_meas > 0 ? stats->cumul_rsrp / stats->num_rsrp_meas : 0;

    char influx_data[1024];
    snprintf(influx_data, sizeof(influx_data),
      "ue_metrics,rnti=%04x in_sync=%d,frame=%d,slot=%d,DL_Thr=%f,UL_Thr=%f,"
      "ph=%d,pcmax=%d,avg_rsrp=%d,num_rsrp_meas=%u,cumul_rsrp=%d,cqi=%d,ri=%d,raw_rssi=%d,"
      "ul_rssi=%u,dl_max_mcs=%u,sched_ul_bytes=%d,estimated_ul_buffer=%d,num_total_bytes=%"PRIu32","
      "dl_pdus_total=%"PRIu16",snr_pusch=%d,dl_bler=%.5f,dl_mcs=%d,dlsch_errors=%"PRIu64","
      "dlsch_total_byte=%"PRIu64",dlsch_current_bytes=%"PRIu32",dl_total_rbs=%"PRIu32",dl_current_rbs=%"PRIu32","
      "dl_num_mac_sdu=%"PRIu32",ul_bler=%.5f,ul_mcs=%d,ulsch_errors=%"PRIu64",ulsch_total_byte=%"PRIu64","
      "ulsch_current_bytes=%"PRIu32",ul_total_rbs=%"PRIu32",ul_current_rbs=%"PRIu32",ul_num_mac_sdu=%"PRIu32",timestamp=%ld\n",
      //通用信息
      UE->rnti,
      !sched_ctrl->ul_failure,
      gNB->frame,
      gNB->slot,
      UE->dl_thr_ue,
      UE->ul_thr_ue,
      //ue schedule ctrl
      sched_ctrl->ph,
      sched_ctrl->pcmax,
      // stats->num_rsrp_meas > 0 ? stats->cumul_rsrp / stats->num_rsrp_meas : 0, //aver_rsrp
      avg_rsrp,
      stats->num_rsrp_meas,  //这两个rsrp主要是为了看看为什么会出现0
      stats->cumul_rsrp,
      sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb, //cqi也一直是0
      sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.ri+1,
      sched_ctrl->raw_rssi,
      sched_ctrl->ul_rssi,
      sched_ctrl->dl_max_mcs,
      sched_ctrl->sched_ul_bytes,
      sched_ctrl->estimated_ul_buffer,
      sched_ctrl->num_total_bytes,
      sched_ctrl->dl_pdus_total,
      sched_ctrl->pusch_snrx10,          //放大了10倍的结果
      // sched_ctrl->pusch_snrx10/10,  //SNR CQI之前一直为0，avgr_rsrp也是，不同版本的OAI这一点不同
      // sched_ctrl->pusch_snrx10%10,
      //dl stats dl_sl_info的没有加上，切片的可能会有用吧
      sched_ctrl->dl_bler_stats.bler,
      sched_ctrl->dl_bler_stats.mcs,
      stats->dl.errors,
      stats->dl.total_bytes,
      stats->dl.current_bytes,
      stats->dl.total_rbs,
      stats->dl.current_rbs,
      stats->dl.num_mac_sdu,
      //ul stats        
      sched_ctrl->ul_bler_stats.bler,
      sched_ctrl->ul_bler_stats.mcs,
      stats->ul.errors,
      stats->ul.total_bytes,
      stats->ul.current_bytes,
      stats->ul.total_rbs,
      stats->ul.current_rbs,
      stats->ul.num_mac_sdu,
      timestamp
      );
        // snprintf(influx_data, sizeof(influx_data),
        //      "ue_metrics,rnti=%04x pusch_snr=%d,timestamp=%ld\n",
        //      UE->rnti,
        //      sched_ctrl->pusch_snrx10,
        //      timestamp);

    // 发送数据到 InfluxDB
    send_to_influxdb(influx_data);
    }
  NR_SCHED_UNLOCK(&gNB->UE_info.mutex);
}

static void mac_rrc_init(gNB_MAC_INST *mac, ngran_node_t node_type)
{
  switch (node_type) {
    case ngran_gNB_CU:
      AssertFatal(1 == 0, "nothing to do for CU\n");
      break;
    case ngran_gNB_DU:
      mac_rrc_ul_f1ap_init(&mac->mac_rrc);
      break;
    case ngran_gNB:
      mac_rrc_ul_direct_init(&mac->mac_rrc);
      break;
    default:
      AssertFatal(0 == 1, "Unknown node type %d\n", node_type);
      break;
  }
}

void mac_top_init_gNB(ngran_node_t node_type)
{
  module_id_t     i;
  gNB_MAC_INST    *nrmac;

  LOG_I(MAC, "[MAIN] Init function start:nb_nr_macrlc_inst=%d\n",RC.nb_nr_macrlc_inst);

  if (RC.nb_nr_macrlc_inst > 0) {

    RC.nrmac = (gNB_MAC_INST **) malloc16(RC.nb_nr_macrlc_inst *sizeof(gNB_MAC_INST *));
    
    AssertFatal(RC.nrmac != NULL,"can't ALLOCATE %zu Bytes for %d gNB_MAC_INST with size %zu \n",
                RC.nb_nr_macrlc_inst * sizeof(gNB_MAC_INST *),
                RC.nb_nr_macrlc_inst, sizeof(gNB_MAC_INST));

    for (i = 0; i < RC.nb_nr_macrlc_inst; i++) {

      RC.nrmac[i] = (gNB_MAC_INST *) malloc16(sizeof(gNB_MAC_INST));
      
      AssertFatal(RC.nrmac != NULL,"can't ALLOCATE %zu Bytes for %d gNB_MAC_INST with size %zu \n",
                  RC.nb_nr_macrlc_inst * sizeof(gNB_MAC_INST *),
                  RC.nb_nr_macrlc_inst, sizeof(gNB_MAC_INST));
      
      LOG_D(MAC,"[MAIN] ALLOCATE %zu Bytes for %d gNB_MAC_INST @ %p\n",sizeof(gNB_MAC_INST), RC.nb_nr_macrlc_inst, RC.mac);
      
      bzero(RC.nrmac[i], sizeof(gNB_MAC_INST));
      
      RC.nrmac[i]->Mod_id = i;

      RC.nrmac[i]->tag = (NR_TAG_t*)malloc(sizeof(NR_TAG_t));
      memset((void*)RC.nrmac[i]->tag,0,sizeof(NR_TAG_t));
        
      RC.nrmac[i]->ul_handle = 0;

      RC.nrmac[i]->first_MIB = true;

      pthread_mutex_init(&RC.nrmac[i]->sched_lock, NULL);

      pthread_mutex_init(&RC.nrmac[i]->UE_info.mutex, NULL);
      uid_linear_allocator_init(&RC.nrmac[i]->UE_info.uid_allocator);

      if (get_softmodem_params()->phy_test) {
        RC.nrmac[i]->pre_processor_dl = nr_preprocessor_phytest;
        RC.nrmac[i]->pre_processor_ul = nr_ul_preprocessor_phytest;
      } else {
        RC.nrmac[i]->pre_processor_dl = nr_init_fr1_dlsch_preprocessor(0);
        RC.nrmac[i]->pre_processor_ul = nr_init_fr1_ulsch_preprocessor(0);
      }

      if (!IS_SOFTMODEM_NOSTATS_BIT)
         threadCreate(&RC.nrmac[i]->stats_thread, nrmac_stats_thread, (void*)RC.nrmac[i], "MAC_STATS", -1,     sched_get_priority_min(SCHED_OAI)+1 );
      mac_rrc_init(RC.nrmac[i], node_type);
    }//END for (i = 0; i < RC.nb_nr_macrlc_inst; i++)

    AssertFatal(rlc_module_init(1) == 0,"Could not initialize RLC layer\n");

    // These should be out of here later
    if (get_softmodem_params()->usim_test == 0 ) nr_pdcp_layer_init();

    if(IS_SOFTMODEM_NOS1 && get_softmodem_params()->phy_test) {
      // get default noS1 configuration
      NR_RadioBearerConfig_t *rbconfig = NULL;
      NR_RLC_BearerConfig_t *rlc_rbconfig = NULL;
      fill_nr_noS1_bearer_config(&rbconfig, &rlc_rbconfig);
      NR_RLC_BearerConfig_t *rlc_rbconfig_list[1] = {rlc_rbconfig};
      struct NR_CellGroupConfig__rlc_BearerToAddModList rlc_bearer_list = {
        .list = { .array = rlc_rbconfig_list, .count = 1, .size = 1, }
      };

      /* Note! previously, in nr_DRB_preconfiguration(), we passed ENB_FLAG_NO
       * if ENB_NAS_USE_TUN was *not* set. It seems to me that we could not set
       * this flag anywhere in the code, hence we would always configure PDCP
       * with ENB_FLAG_NO in nr_DRB_preconfiguration(). This makes sense for
       * noS1, because the result of passing ENB_FLAG_NO to PDCP is that PDCP
       * will output the packets at a local interface, which is in line with
       * the noS1 mode.  Hence, below, we simply hardcode ENB_FLAG_NO */
      // setup PDCP, RLC
      nr_pdcp_add_drbs(ENB_FLAG_NO, 0x1234, 0, rbconfig->drb_ToAddModList, 0, NULL, NULL, &rlc_bearer_list);
      nr_rlc_add_drb(0x1234, rbconfig->drb_ToAddModList->list.array[0]->drb_Identity, rlc_rbconfig, NULL);

      // free memory
      free_nr_noS1_bearer_config(&rbconfig, &rlc_rbconfig);
    }

  } else {
    RC.nrmac = NULL;
  }

  // Initialize Linked-List for Active UEs
  for (i = 0; i < RC.nb_nr_macrlc_inst; i++) {
    nrmac = RC.nrmac[i];
    nrmac->if_inst = NR_IF_Module_init(i);
    memset(&nrmac->UE_info, 0, sizeof(nrmac->UE_info));
  }

  srand48(0);
}

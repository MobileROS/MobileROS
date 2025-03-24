#!/usr/bin/env python3
import rospy
import requests
import json
import time
from datetime import datetime, timedelta
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from wireless_ros.msgs.msg import ChannelState

class RadioInformationEngine:
    def __init__(self):
        rospy.init_node('radio_information_engine')
        
        # InfluxDB配置
        self.influxdb_url = rospy.get_param('~influxdb_url', 'http://localhost:8086')
        self.influxdb_token = rospy.get_param('~influxdb_token', 'LbUOm57nYdJp0trGuaI4TcSTvzeo5yJYsj5GtEC-EvkNmEFC35t4HyrMwWBanDhVRZmVW7iTqqXGTGB82DmG2g==')
        self.influxdb_org = rospy.get_param('~influxdb_org', 'my_org')
        self.influxdb_bucket = rospy.get_param('~influxdb_bucket', 'my_bucket')
        
        # 发布者
        self.channel_state_pub = rospy.Publisher('/wireless_ros/channel_state', ChannelState, queue_size=10)
        
        # InfluxDB客户端
        self.client = InfluxDBClient(
            url=self.influxdb_url,
            token=self.influxdb_token,
            org=self.influxdb_org
        )
        self.query_api = self.client.query_api()
        
        # 参数
        self.query_interval = rospy.get_param('~query_interval', 0.1)  # 100ms
        self.rnti_list = []  # 动态发现的RNTI列表
        
        rospy.loginfo(f"RadioInformationEngine initialized. Connecting to InfluxDB at {self.influxdb_url}")
        self.discover_rntis()
        
        # 启动主循环
        self.timer = rospy.Timer(rospy.Duration(self.query_interval), self.query_and_publish_data)
        rospy.on_shutdown(self.shutdown_hook)

    def discover_rntis(self):
        """发现当前活跃的RNTI列表"""
        try:
            query = f'from(bucket: "{self.influxdb_bucket}") \
                    |> range(start: -1m) \
                    |> filter(fn: (r) => r._measurement == "glasses_18") \
                    |> group(columns: ["rnti"]) \
                    |> distinct(column: "rnti")'
            
            result = self.query_api.query(query=query)
            
            rnti_list = []
            for table in result:
                for record in table.records:
                    rnti_list.append(record.values.get("rnti"))
            
            if rnti_list:
                self.rnti_list = rnti_list
                rospy.loginfo(f"Discovered RNTIs: {self.rnti_list}")
            else:
                rospy.logwarn("No RNTIs discovered. Will retry...")
        except Exception as e:
            rospy.logerr(f"Error discovering RNTIs: {e}")

    def query_and_publish_data(self, event=None):
        """查询InfluxDB并发布数据"""
        current_time = datetime.utcnow()
        start_time = (current_time - timedelta(seconds=1)).isoformat() + "Z"
        
        # 如果没有发现RNTI，尝试重新发现
        if not self.rnti_list:
            self.discover_rntis()
            return
        
        for rnti in self.rnti_list:
            try:
                query = f'from(bucket: "{self.influxdb_bucket}") \
                        |> range(start: {start_time}) \
                        |> filter(fn: (r) => r._measurement == "glasses_18" and r.rnti == "{rnti}") \
                        |> last()'
                
                result = self.query_api.query(query=query)
                
                if not result or len(result) == 0:
                    continue
                
                # 从查询结果构造消息
                channel_state = ChannelState()
                channel_state.header.stamp = rospy.Time.now()
                channel_state.rnti = int(rnti, 16)  # 将十六进制RNTI转换为整数
                
                for table in result:
                    for record in table.records:
                        field = record.get_field()
                        value = record.get_value()
                        
                        # 将InfluxDB字段映射到ROS消息
                        if field == "in_sync":
                            channel_state.in_sync = bool(value)
                        elif field == "frame":
                            channel_state.frame = int(value)
                        elif field == "slot":
                            channel_state.slot = int(value)
                        elif field == "DL_Thr":
                            channel_state.dl_thr = float(value)
                        elif field == "UL_Thr":
                            channel_state.ul_thr = float(value)
                        elif field == "ph":
                            channel_state.ph = int(value)
                        elif field == "pcmax":
                            channel_state.pcmax = int(value)
                        elif field == "avg_rsrp":
                            channel_state.avg_rsrp = int(value)
                        elif field == "num_rsrp_meas":
                            channel_state.num_rsrp_meas = int(value)
                        elif field == "cqi":
                            channel_state.cqi = int(value)
                        elif field == "ri":
                            channel_state.ri = int(value)
                        elif field == "raw_rssi":
                            channel_state.raw_rssi = int(value)
                        elif field == "ul_rssi":
                            channel_state.ul_rssi = int(value)
                        elif field == "dl_max_mcs":
                            channel_state.dl_max_mcs = int(value)
                        elif field == "sched_ul_bytes":
                            channel_state.sched_ul_bytes = int(value)
                        elif field == "estimated_ul_buffer":
                            channel_state.estimated_ul_buffer = int(value)
                        elif field == "num_total_bytes":
                            channel_state.num_total_bytes = int(value)
                        elif field == "dl_pdus_total":
                            channel_state.dl_pdus_total = int(value)
                        elif field == "snr_pusch":
                            channel_state.pusch_snr = int(value)
                        elif field == "dl_bler":
                            channel_state.dl_bler = float(value)
                        elif field == "dl_mcs":
                            channel_state.dl_mcs = int(value)
                        elif field == "dlsch_errors":
                            channel_state.dlsch_errors = int(value)
                        elif field == "dlsch_total_byte":
                            channel_state.dlsch_total_bytes = int(value)
                        elif field == "dlsch_current_bytes":
                            channel_state.dlsch_current_bytes = int(value)
                        elif field == "dl_total_rbs":
                            channel_state.dl_total_rbs = int(value)
                        elif field == "dl_current_rbs":
                            channel_state.dl_current_rbs = int(value)
                        elif field == "dl_num_mac_sdu":
                            channel_state.dl_num_mac_sdu = int(value)
                        elif field == "ul_bler":
                            channel_state.ul_bler = float(value)
                        elif field == "ul_mcs":
                            channel_state.ul_mcs = int(value)
                        elif field == "ulsch_errors":
                            channel_state.ulsch_errors = int(value)
                        elif field == "ulsch_total_byte":
                            channel_state.ulsch_total_bytes = int(value)
                        elif field == "ulsch_current_bytes":
                            channel_state.ulsch_current_bytes = int(value)
                        elif field == "ul_total_rbs":
                            channel_state.ul_total_rbs = int(value)
                        elif field == "ul_current_rbs":
                            channel_state.ul_current_rbs = int(value)
                        elif field == "ul_num_mac_sdu":
                            channel_state.ul_num_mac_sdu = int(value)
                
                # 发布数据
                self.channel_state_pub.publish(channel_state)
                rospy.logdebug(f"Published channel state for RNTI {rnti}")
            
            except Exception as e:
                rospy.logerr(f"Error querying data for RNTI {rnti}: {e}")

    def shutdown_hook(self):
        """关闭时的处理"""
        self.client.close()
        rospy.loginfo("RadioInformationEngine shutting down")

if __name__ == '__main__':
    try:
        engine = RadioInformationEngine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
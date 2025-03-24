#!/usr/bin/env python3
import rospy
import requests
import json
import time
import numpy as np
import threading
from datetime import datetime, timedelta
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from wireless_ros.msgs.msg import ChannelState, PhyLayerMetrics, SpectrumAnalysis
from std_msgs.msg import String, Float32, Int32MultiArray
from wireless_ros.core import ChannelObserver

class RadioInformationEngine(ChannelObserver):
    """
    无线电信息引擎 - 提取PHY和MAC层指标，将它们转换为语义信息
    
    依据服务网格原则和领域驱动设计方法，该引擎负责：
    1. 从物理层获取无线信道信息
    2. 分析通信质量与稳定性
    3. 预测未来带宽和延迟变化
    4. 提供频谱感知和资源利用分析
    """
    
    def __init__(self):
        super(RadioInformationEngine, self).__init__()
        rospy.init_node('radio_information_engine')
        
        # InfluxDB配置
        self.influxdb_url = rospy.get_param('~influxdb_url', 'http://localhost:8086')
        self.influxdb_token = rospy.get_param('~influxdb_token', 'LbUOm57nYdJp0trGuaI4TcSTvzeo5yJYsj5GtEC-EvkNmEFC35t4HyrMwWBanDhVRZmVW7iTqqXGTGB82DmG2g==')
        self.influxdb_org = rospy.get_param('~influxdb_org', 'my_org')
        self.influxdb_bucket = rospy.get_param('~influxdb_bucket', 'my_bucket')
        
        # 高级参数
        self.prediction_horizon = rospy.get_param('~prediction_horizon', 10)  # 预测未来多少个采样点
        self.history_window_size = rospy.get_param('~history_window_size', 50)  # 历史窗口大小
        self.prediction_enabled = rospy.get_param('~prediction_enabled', True)  # 启用预测
        self.anomaly_detection_enabled = rospy.get_param('~anomaly_detection_enabled', True)  # 启用异常检测
        self.measurement_interval = rospy.get_param('~measurement_interval', 0.1)  # 100ms
        
        # 发布者 - 基本通道状态
        self.channel_state_pub = rospy.Publisher('/wireless_ros/channel_state', ChannelState, queue_size=10)
        
        # 发布者 - 高级指标
        self.phy_metrics_pub = rospy.Publisher('/wireless_ros/phy_layer_metrics', PhyLayerMetrics, queue_size=10)
        self.spectrum_analysis_pub = rospy.Publisher('/wireless_ros/spectrum_analysis', SpectrumAnalysis, queue_size=10)
        self.bandwidth_forecast_pub = rospy.Publisher('/wireless_ros/bandwidth_forecast', Float32, queue_size=10)
        self.latency_forecast_pub = rospy.Publisher('/wireless_ros/latency_forecast', Float32, queue_size=10)
        self.resource_utilization_pub = rospy.Publisher('/wireless_ros/resource_utilization', Float32, queue_size=10)
        self.interference_zone_pub = rospy.Publisher('/wireless_ros/interference_zone', String, queue_size=10)
        self.prb_utilization_pub = rospy.Publisher('/wireless_ros/prb_utilization', Int32MultiArray, queue_size=10)
        
        # InfluxDB客户端
        self.client = InfluxDBClient(
            url=self.influxdb_url,
            token=self.influxdb_token,
            org=self.influxdb_org
        )
        self.query_api = self.client.query_api()
        
        # 状态跟踪
        self.rnti_list = []  # 动态发现的RNTI列表
        self.channel_history = {}  # RNTI -> 历史数据
        self.bandwidth_history = {}  # RNTI -> 带宽历史
        self.latency_history = {}  # RNTI -> 延迟历史
        self.snr_history = {}  # RNTI -> SNR历史
        self.bler_history = {}  # RNTI -> BLER历史
        self.resource_usage_history = {}  # RNTI -> 资源使用历史
        
        # 预测模型状态
        self.prediction_models = {}  # RNTI -> 预测模型
        self.last_prediction_time = {}  # RNTI -> 上次预测时间
        self.prediction_accuracy = {}  # RNTI -> 预测准确性
        
        # 频谱分析
        self.interference_patterns = {}  # 干扰模式检测
        self.spectral_efficiency = {}  # 频谱效率分析
        
        # 异常检测
        self.anomaly_thresholds = {
            'snr_drop': 5.0,  # SNR骤降阈值(dB)
            'bler_spike': 0.15,  # BLER突增阈值
            'latency_spike': 30.0,  # 延迟突增阈值(ms)
        }
        
        # 初始化
        rospy.loginfo(f"RadioInformationEngine initialized. Connecting to InfluxDB at {self.influxdb_url}")
        
        # 启动发现RNTI线程
        self.discovery_thread = threading.Thread(target=self.discover_rntis_periodically)
        self.discovery_thread.daemon = True
        self.discovery_thread.start()
        
        # 启动主循环
        self.timer = rospy.Timer(rospy.Duration(self.measurement_interval), self.query_and_publish_data)
        
        # 启动高级分析线程
        self.analysis_thread = threading.Thread(target=self.run_advanced_analysis)
        self.analysis_thread.daemon = True
        self.analysis_thread.start()
        
        rospy.on_shutdown(self.shutdown_hook)

    def discover_rntis_periodically(self):
        """定期发现活跃的RNTI"""
        while not rospy.is_shutdown():
            self.discover_rntis()
            time.sleep(5.0)  # 每5秒扫描一次新的RNTI

    def discover_rntis(self):
        """发现当前活跃的RNTI列表"""
        try:
            # 查询过去1分钟的活跃RNTI
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
            
            # 检查是否发现了新的RNTI
            new_rntis = [rnti for rnti in rnti_list if rnti not in self.rnti_list]
            if new_rntis:
                rospy.loginfo(f"Discovered new RNTIs: {new_rntis}")
                self.rnti_list = rnti_list
                
                # 为新的RNTI初始化历史数据结构
                for rnti in new_rntis:
                    self.channel_history[rnti] = []
                    self.bandwidth_history[rnti] = []
                    self.latency_history[rnti] = []
                    self.snr_history[rnti] = []
                    self.bler_history[rnti] = []
                    self.resource_usage_history[rnti] = []
                    self.prediction_models[rnti] = self.initialize_prediction_model()
                    self.last_prediction_time[rnti] = 0
                    self.prediction_accuracy[rnti] = {
                        'bandwidth': 0.0,
                        'latency': 0.0,
                        'snr': 0.0
                    }
            
            # 检查是否有RNTI变为非活跃
            inactive_rntis = [rnti for rnti in self.rnti_list if rnti not in rnti_list]
            if inactive_rntis:
                rospy.loginfo(f"RNTIs became inactive: {inactive_rntis}")
                self.rnti_list = rnti_list
        
        except Exception as e:
            rospy.logerr(f"Error discovering RNTIs: {e}")

    def initialize_prediction_model(self):
        """初始化预测模型"""
        # 简单的线性预测模型
        # 在实际实现中，可能会使用更复杂的机器学习模型
        return {
            'type': 'linear',
            'coefficients': {
                'bandwidth': [0, 0],  # [斜率, 截距]
                'latency': [0, 0],
                'snr': [0, 0]
            },
            'last_update': 0
        }

    def query_and_publish_data(self, event=None):
        """查询InfluxDB并发布数据"""
        current_time = datetime.utcnow()
        start_time = (current_time - timedelta(seconds=1)).isoformat() + "Z"
        
        # 如果没有发现RNTI，无需查询
        if not self.rnti_list:
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
                
                # 从查询结果构造基础消息
                channel_state = self.build_channel_state_from_results(result, rnti)
                
                # 更新历史数据
                self.update_history(rnti, channel_state)
                
                # 发布基础通道状态
                self.channel_state_pub.publish(channel_state)
                
                # 生成并发布高级PHY层指标
                phy_metrics = self.generate_phy_metrics(rnti, channel_state)
                self.phy_metrics_pub.publish(phy_metrics)
                
                # 运行频谱分析并发布结果
                spectrum_analysis = self.analyze_spectrum(rnti, channel_state)
                self.spectrum_analysis_pub.publish(spectrum_analysis)
                
                # 预测带宽和延迟
                if self.prediction_enabled and len(self.bandwidth_history[rnti]) >= 10:
                    bandwidth_forecast, latency_forecast = self.predict_network_conditions(rnti)
                    self.bandwidth_forecast_pub.publish(bandwidth_forecast)
                    self.latency_forecast_pub.publish(latency_forecast)
                
                # 发布资源利用率
                resource_utilization = self.calculate_resource_utilization(rnti, channel_state)
                self.resource_utilization_pub.publish(resource_utilization)
                
                # 检测并发布干扰区域信息
                interference_zone = self.detect_interference_zone(rnti, channel_state)
                if interference_zone:
                    self.interference_zone_pub.publish(interference_zone)
                
                # 发布PRB利用情况
                prb_utilization = self.analyze_prb_utilization(rnti, channel_state)
                self.prb_utilization_pub.publish(prb_utilization)
                
                # 通知观察者
                self.notify_observers(channel_state)
                
                rospy.logdebug(f"Published complete radio information for RNTI {rnti}")
            
            except Exception as e:
                rospy.logerr(f"Error processing data for RNTI {rnti}: {e}")

    def build_channel_state_from_results(self, result, rnti):
        """从查询结果构建通道状态消息"""
        channel_state = ChannelState()
        channel_state.header.stamp = rospy.Time.now()
        
        try:
            # 将十六进制RNTI转换为整数
            channel_state.rnti = int(rnti, 16)
        except ValueError:
            # 如果转换失败，尝试直接使用
            channel_state.rnti = int(rnti)
        
        # 提取所有字段值
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
        
        return channel_state

    def update_history(self, rnti, channel_state):
        """更新历史数据"""
        # 添加通道状态到历史记录
        self.channel_history[rnti].append(channel_state)
        
        # 提取关键指标并更新历史
        self.bandwidth_history[rnti].append(channel_state.dl_thr)
        self.latency_history[rnti].append(20.0)  # 假设单位为ms，实际应从测量获取
        self.snr_history[rnti].append(channel_state.pusch_snr)
        self.bler_history[rnti].append(channel_state.dl_bler)
        
        # 计算资源使用率 (PRB利用率)
        if channel_state.dl_total_rbs > 0:
            resource_usage = channel_state.dl_current_rbs / channel_state.dl_total_rbs
        else:
            resource_usage = 0.0
        self.resource_usage_history[rnti].append(resource_usage)
        
        # 保持历史记录在合理大小
        if len(self.channel_history[rnti]) > self.history_window_size:
            self.channel_history[rnti] = self.channel_history[rnti][-self.history_window_size:]
            self.bandwidth_history[rnti] = self.bandwidth_history[rnti][-self.history_window_size:]
            self.latency_history[rnti] = self.latency_history[rnti][-self.history_window_size:]
            self.snr_history[rnti] = self.snr_history[rnti][-self.history_window_size:]
            self.bler_history[rnti] = self.bler_history[rnti][-self.history_window_size:]
            self.resource_usage_history[rnti] = self.resource_usage_history[rnti][-self.history_window_size:]

    def generate_phy_metrics(self, rnti, channel_state):
        """生成高级PHY层指标"""
        phy_metrics = PhyLayerMetrics()
        phy_metrics.header.stamp = rospy.Time.now()
        phy_metrics.rnti = channel_state.rnti
        
        # 计算链路质量指标
        phy_metrics.link_quality = self.calculate_link_quality(rnti)
        
        # 计算信道稳定性
        phy_metrics.channel_stability = self.calculate_channel_stability(rnti)
        
        # 计算通信可靠性
        phy_metrics.reliability = self.calculate_reliability(rnti)
        
        # 计算频谱效率
        phy_metrics.spectral_efficiency = self.calculate_spectral_efficiency(rnti, channel_state)
        
        # 计算信道容量
        phy_metrics.channel_capacity = self.calculate_channel_capacity(rnti, channel_state)
        
        # 计算信道响应时间
        phy_metrics.channel_response_time = self.calculate_channel_response_time(rnti)
        
        return phy_metrics

    def calculate_link_quality(self, rnti):
        """计算链路质量，综合考虑SNR、BLER等因素"""
        if not self.snr_history[rnti] or not self.bler_history[rnti]:
            return 0.5  # 默认中等质量
        
        # 使用最近10个样本
        recent_snr = self.snr_history[rnti][-10:] if len(self.snr_history[rnti]) >= 10 else self.snr_history[rnti]
        recent_bler = self.bler_history[rnti][-10:] if len(self.bler_history[rnti]) >= 10 else self.bler_history[rnti]
        
        # 归一化SNR (0-30dB -> 0-1)
        avg_snr = np.mean(recent_snr)
        norm_snr = min(1.0, max(0.0, avg_snr / 30.0))
        
        # 归一化BLER (0-1, 越低越好)
        avg_bler = np.mean(recent_bler)
        norm_bler = 1.0 - min(1.0, avg_bler * 5.0)  # BLER>0.2视为非常差
        
        # 综合评分 (加权平均)
        link_quality = norm_snr * 0.6 + norm_bler * 0.4
        
        return link_quality

    def calculate_channel_stability(self, rnti):
        """计算信道稳定性，基于SNR和BLER的波动"""
        if len(self.snr_history[rnti]) < 10 or len(self.bler_history[rnti]) < 10:
            return 0.5  # 默认中等稳定性
        
        # 计算SNR标准差的归一化值
        snr_std = np.std(self.snr_history[rnti][-10:])
        norm_snr_std = max(0.0, 1.0 - (snr_std / 10.0))  # 标准差越小，稳定性越高
        
        # 计算BLER标准差的归一化值
        bler_std = np.std(self.bler_history[rnti][-10:])
        norm_bler_std = max(0.0, 1.0 - (bler_std * 20.0))  # 标准差越小，稳定性越高
        
        # 综合评分
        stability = norm_snr_std * 0.5 + norm_bler_std * 0.5
        
        return min(1.0, max(0.0, stability))

    def calculate_reliability(self, rnti):
        """计算通信可靠性，基于BLER和错误率"""
        if not self.bler_history[rnti]:
            return 0.5  # 默认中等可靠性
        
        recent_bler = self.bler_history[rnti][-10:] if len(self.bler_history[rnti]) >= 10 else self.bler_history[rnti]
        
        # 平均BLER
        avg_bler = np.mean(recent_bler)
        
        # 归一化可靠性 (BLER越低越可靠)
        reliability = max(0.0, 1.0 - avg_bler * 5.0)  # BLER>0.2视为非常不可靠
        
        return min(1.0, reliability)

    def calculate_spectral_efficiency(self, rnti, channel_state):
        """计算频谱效率 (bits/s/Hz)"""
        # 使用MCS和调制阶数估算频谱效率
        mcs = channel_state.dl_mcs
        
        # 根据MCS估算调制阶数和编码率
        if mcs <= 9:
            # QPSK
            modulation_order = 2
            coding_rate = (mcs + 1) / 10.0
        elif mcs <= 16:
            # 16QAM
            modulation_order = 4
            coding_rate = (mcs - 9) / 7.0
        elif mcs <= 28:
            # 64QAM
            modulation_order = 6
            coding_rate = (mcs - 16) / 12.0
        else:
            # 256QAM (更高阶)
            modulation_order = 8
            coding_rate = 0.8
        
        # 频谱效率 = 调制阶数 * 编码率 * (1 - BLER)
        spectral_efficiency = modulation_order * coding_rate * (1 - channel_state.dl_bler)
        
        return spectral_efficiency

    def calculate_channel_capacity(self, rnti, channel_state):
        """根据Shannon公式计算信道容量 (bits/s/Hz)"""
        # C = B * log2(1 + SNR)
        # 这里我们简化为单位带宽下的容量
        
        # 将dB转换为线性比例
        snr_linear = 10 ** (channel_state.pusch_snr / 10.0)
        
        # Shannon公式
        capacity = np.log2(1 + snr_linear)
        
        return capacity

    def calculate_channel_response_time(self, rnti):
        """估计信道响应时间 (ms)"""
        # 简单返回平均延迟
        if not self.latency_history[rnti]:
            return 20.0  # 默认值
        
        return np.mean(self.latency_history[rnti][-5:])

    def analyze_spectrum(self, rnti, channel_state):
        """进行频谱分析"""
        spectrum_analysis = SpectrumAnalysis()
        spectrum_analysis.header.stamp = rospy.Time.now()
        spectrum_analysis.rnti = channel_state.rnti
        
        # 频谱占用率
        if channel_state.dl_total_rbs > 0:
            spectrum_analysis.utilization = float(channel_state.dl_current_rbs) / channel_state.dl_total_rbs
        else:
            spectrum_analysis.utilization = 0.0
        
        # 干扰水平估计 (使用RSSI和RSRP的差值作为估计)
        interference_level = channel_state.raw_rssi - channel_state.avg_rsrp
        spectrum_analysis.interference_level = max(0, interference_level)
        
        # 估计信噪比
        spectrum_analysis.estimated_snr = float(channel_state.pusch_snr)
        
        # 频谱效率
        spectrum_analysis.spectral_efficiency = self.calculate_spectral_efficiency(rnti, channel_state)
        
        # 检测异常模式
        if self.anomaly_detection_enabled:
            # 检查SNR急剧下降
            if len(self.snr_history[rnti]) > 5:
                snr_diff = np.mean(self.snr_history[rnti][-5:-1]) - channel_state.pusch_snr
                if snr_diff > self.anomaly_thresholds['snr_drop']:
                    spectrum_analysis.anomaly_detected = True
                    spectrum_analysis.anomaly_type = "SNR_DROP"
            
            # 检查BLER突升
            if len(self.bler_history[rnti]) > 5:
                bler_diff = channel_state.dl_bler - np.mean(self.bler_history[rnti][-5:-1])
                if bler_diff > self.anomaly_thresholds['bler_spike']:
                    spectrum_analysis.anomaly_detected = True
                    spectrum_analysis.anomaly_type = "BLER_SPIKE"
        
        return spectrum_analysis

    def predict_network_conditions(self, rnti):
        """预测未来网络条件"""
        # 更新预测模型
        self.update_prediction_model(rnti)
        
        # 使用模型预测带宽
        bandwidth_forecast = self.predict_bandwidth(rnti)
        
        # 使用模型预测延迟
        latency_forecast = self.predict_latency(rnti)
        
        return bandwidth_forecast, latency_forecast

    def update_prediction_model(self, rnti):
        """更新预测模型参数"""
        current_time = time.time()
        
        # 每5秒更新一次模型
        if current_time - self.prediction_models[rnti]['last_update'] < 5.0:
            return
        
        # 更新时间戳
        self.prediction_models[rnti]['last_update'] = current_time
        
        # 带宽预测模型更新 (简单线性回归)
        if len(self.bandwidth_history[rnti]) >= 10:
            x = np.array(range(len(self.bandwidth_history[rnti][-10:])))
            y = np.array(self.bandwidth_history[rnti][-10:])
            
            try:
                # 计算趋势线
                slope, intercept = np.polyfit(x, y, 1)
                self.prediction_models[rnti]['coefficients']['bandwidth'] = [slope, intercept]
            except:
                rospy.logwarn(f"Failed to update bandwidth prediction model for RNTI {rnti}")
        
        # 延迟预测模型更新
        if len(self.latency_history[rnti]) >= 10:
            x = np.array(range(len(self.latency_history[rnti][-10:])))
            y = np.array(self.latency_history[rnti][-10:])
            
            try:
                # 计算趋势线
                slope, intercept = np.polyfit(x, y, 1)
                self.prediction_models[rnti]['coefficients']['latency'] = [slope, intercept]
            except:
                rospy.logwarn(f"Failed to update latency prediction model for RNTI {rnti}")
        
        # SNR预测模型更新
        if len(self.snr_history[rnti]) >= 10:
            x = np.array(range(len(self.snr_history[rnti][-10:])))
            y = np.array(self.snr_history[rnti][-10:])
            
            try:
                # 计算趋势线
                slope, intercept = np.polyfit(x, y, 1)
                self.prediction_models[rnti]['coefficients']['snr'] = [slope, intercept]
            except:
                rospy.logwarn(f"Failed to update SNR prediction model for RNTI {rnti}")

    def predict_bandwidth(self, rnti):
        """预测未来带宽"""
        if not self.bandwidth_history[rnti]:
            return 0.0
        
        try:
            # 获取预测模型参数
            slope, intercept = self.prediction_models[rnti]['coefficients']['bandwidth']
            
            # 预测下一个时间点的值
            next_x = len(self.bandwidth_history[rnti])
            predicted_bw = slope * next_x + intercept
            
            # 确保预测值合理
            predicted_bw = max(0.1, predicted_bw)  # 最小0.1 Mbps
            
            return predicted_bw
        except:
            # 如果预测失败，返回最后一个已知值
            return self.bandwidth_history[rnti][-1]

    def predict_latency(self, rnti):
        """预测未来延迟"""
        if not self.latency_history[rnti]:
            return 20.0  # 默认值
        
        try:
            # 获取预测模型参数
            slope, intercept = self.prediction_models[rnti]['coefficients']['latency']
            
            # 预测下一个时间点的值
            next_x = len(self.latency_history[rnti])
            predicted_latency = slope * next_x + intercept
            
            # 确保预测值合理
            predicted_latency = max(1.0, predicted_latency)  # 最小1ms
            
            return predicted_latency
        except:
            # 如果预测失败，返回最后一个已知值
            return self.latency_history[rnti][-1]

    def calculate_resource_utilization(self, rnti, channel_state):
        """计算资源利用率"""
        # PRB利用率
        if channel_state.dl_total_rbs > 0:
            prb_utilization = float(channel_state.dl_current_rbs) / channel_state.dl_total_rbs
        else:
            prb_utilization = 0.0
        
        return prb_utilization

    def detect_interference_zone(self, rnti, channel_state):
        """检测干扰区域"""
        # 如果RSRP低但RSSI高，可能存在干扰
        if channel_state.avg_rsrp < -100 and channel_state.raw_rssi > -80:
            return "HIGH_INTERFERENCE_ZONE"
        
        # 如果SNR低但功率高，可能存在干扰
        if channel_state.pusch_snr < 10 and channel_state.raw_rssi > -70:
            return "SIGNAL_DEGRADATION_ZONE"
        
        # 如果BLER高但MCS低，可能存在干扰
        if channel_state.dl_bler > 0.1 and channel_state.dl_mcs < 10:
            return "POOR_CHANNEL_ZONE"
        
        return ""

    def analyze_prb_utilization(self, rnti, channel_state):
        """分析PRB利用情况"""
        # 创建PRB利用率消息
        prb_util = Int32MultiArray()
        
        # 简单返回当前PRB和总PRB
        prb_util.data = [channel_state.dl_current_rbs, channel_state.dl_total_rbs]
        
        return prb_util

    def run_advanced_analysis(self):
        """运行高级分析任务"""
        while not rospy.is_shutdown():
            # 分析所有活跃RNTI的通信模式
            for rnti in self.rnti_list:
                # 跳过历史数据不足的RNTI
                if rnti not in self.channel_history or len(self.channel_history[rnti]) < 20:
                    continue
                
                # 分析频谱稳定性
                self.analyze_spectral_stability(rnti)
                
                # 检测长期干扰模式
                self.detect_interference_patterns(rnti)
                
                # 评估预测准确性
                self.evaluate_prediction_accuracy(rnti)
            
            # 休眠一段时间
            time.sleep(5.0)

    def analyze_spectral_stability(self, rnti):
        """分析频谱稳定性"""
        if len(self.snr_history[rnti]) < 20:
            return
        
        # 计算SNR的滚动标准差
        rolling_std = []
        for i in range(len(self.snr_history[rnti]) - 10):
            window = self.snr_history[rnti][i:i+10]
            rolling_std.append(np.std(window))
        
        # 判断频谱稳定性
        avg_std = np.mean(rolling_std)
        
        # 更新频谱效率分析
        self.spectral_efficiency[rnti] = {
            'stability': 1.0 / (1.0 + avg_std),  # 标准差越小，稳定性越高
            'trend': 'stable' if avg_std < 2.0 else 'unstable',
            'last_update': time.time()
        }

    def detect_interference_patterns(self, rnti):
        """检测长期干扰模式"""
        if len(self.snr_history[rnti]) < 30 or len(self.bler_history[rnti]) < 30:
            return
        
        # 分析SNR和BLER的相关性
        snr_data = np.array(self.snr_history[rnti][-30:])
        bler_data = np.array(self.bler_history[rnti][-30:])
        
        # 计算相关系数
        try:
            correlation = np.corrcoef(snr_data, bler_data)[0, 1]
            
            # 强负相关表示正常行为 (SNR高，BLER低)
            # 弱相关或正相关可能表示存在干扰
            if correlation > -0.5:
                self.interference_patterns[rnti] = {
                    'type': 'POTENTIAL_INTERFERENCE',
                    'correlation': correlation,
                    'detection_time': time.time()
                }
                rospy.loginfo(f"Detected potential interference pattern for RNTI {rnti}")
        except:
            pass

    def evaluate_prediction_accuracy(self, rnti):
        """评估预测准确性"""
        # 只有在有足够的历史数据时才评估
        if len(self.bandwidth_history[rnti]) < 20 or len(self.latency_history[rnti]) < 20:
            return
        
        # 上次预测时间超过5秒，可以评估
        current_time = time.time()
        if current_time - self.last_prediction_time.get(rnti, 0) < 5.0:
            return
        
        try:
            # 获取10个采样点前的预测模型
            old_data_index = -15
            x_old = np.array(range(len(self.bandwidth_history[rnti][:old_data_index])))
            y_bw_old = np.array(self.bandwidth_history[rnti][:old_data_index])
            
            # 计算当时的趋势
            slope_bw, intercept_bw = np.polyfit(x_old, y_bw_old, 1)
            
            # 预测现在的值
            x_now = len(self.bandwidth_history[rnti][:old_data_index]) + 5
            predicted_bw = slope_bw * x_now + intercept_bw
            
            # 获取实际值
            actual_bw = np.mean(self.bandwidth_history[rnti][-5:])
            
            # 计算预测误差
            error_bw = abs((predicted_bw - actual_bw) / max(0.1, actual_bw))
            
            # 更新预测准确性
            self.prediction_accuracy[rnti]['bandwidth'] = max(0.0, 1.0 - min(1.0, error_bw))
            
            # 更新最后预测时间
            self.last_prediction_time[rnti] = current_time
            
            rospy.logdebug(f"Prediction accuracy for RNTI {rnti}: {self.prediction_accuracy[rnti]['bandwidth']:.2f}")
        except:
            rospy.logwarn(f"Failed to evaluate prediction accuracy for RNTI {rnti}")

    def shutdown_hook(self):
        """关闭时的处理"""
        if hasattr(self, 'client'):
            self.client.close()
        rospy.loginfo("RadioInformationEngine shutting down")

if __name__ == '__main__':
    try:
        engine = RadioInformationEngine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
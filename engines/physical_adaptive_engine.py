#!/usr/bin/env python3
import rospy
import numpy as np
import threading
import time
from collections import deque
from wireless_ros.msgs.msg import ChannelState, TransmissionStrategy, Constraint, PhyLayerMetrics
from std_msgs.msg import Int32, Float32, String, Bool
from sensor_msgs.msg import Image, PointCloud2

class PhysicalAdaptiveEngine:
    """
    物理自适应引擎 - 基于任务需求和网络状况调整物理层参数
    
    该引擎负责：
    1. 动态MCS和调制方案调整
    2. 自适应PRB分配
    3. 数据压缩和传输策略优化
    4. 传输优先级和HARQ配置
    5. 基于环境的无线适应性学习
    """
    
    def __init__(self):
        rospy.init_node('physical_adaptive_engine')
        
        # 高级参数
        self.update_interval = rospy.get_param('~update_interval', 0.1)  # 100ms
        self.mcs_table = self.initialize_mcs_table()
        self.learning_rate = rospy.get_param('~learning_rate', 0.1)
        self.exploration_rate = rospy.get_param('~exploration_rate', 0.2)
        self.min_exploration_rate = rospy.get_param('~min_exploration_rate', 0.05)
        self.max_harq_retx = rospy.get_param('~max_harq_retx', 4)
        self.enable_rtt_measurement = rospy.get_param('~enable_rtt_measurement', True)
        self.enable_adaptive_harq = rospy.get_param('~enable_adaptive_harq', True)
        self.enable_predictive_coding = rospy.get_param('~enable_predictive_coding', True)
        self.use_reinforcement_learning = rospy.get_param('~use_reinforcement_learning', True)
        
        # 订阅者 - 信道和约束
        rospy.Subscriber('/wireless_ros/channel_state', ChannelState, self.channel_state_callback)
        rospy.Subscriber('/wireless_ros/constraints', Constraint, self.constraint_callback)
        rospy.Subscriber('/wireless_ros/phy_layer_metrics', PhyLayerMetrics, self.phy_metrics_callback)
        
        # 订阅者 - 传感器和数据流
        rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        rospy.Subscriber('/lidar/points', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/wireless_ros/adaptation_recommendation', String, self.adaptation_recommendation_callback)
        rospy.Subscriber('/wireless_ros/task_criticality', Float32, self.task_criticality_callback)
        
        # 发布者 - 传输策略
        self.strategy_pub = rospy.Publisher('/wireless_ros/transmission_strategy', TransmissionStrategy, queue_size=10)
        self.target_mcs_pub = rospy.Publisher('/wireless_ros/target_mcs', Int32, queue_size=10)
        self.target_prb_pub = rospy.Publisher('/wireless_ros/target_prb', Int32, queue_size=10)
        self.compression_ratio_pub = rospy.Publisher('/wireless_ros/compression_ratio', Float32, queue_size=10)
        
        # 发布者 - 传输质量和调试信息
        self.packet_success_rate_pub = rospy.Publisher('/wireless_ros/packet_success_rate', Float32, queue_size=10)
        self.adaptive_action_pub = rospy.Publisher('/wireless_ros/adaptive_action', String, queue_size=10)
        self.learning_status_pub = rospy.Publisher('/wireless_ros/learning_status', String, queue_size=10)
        self.resource_allocation_pub = rospy.Publisher('/wireless_ros/resource_allocation', String, queue_size=10)
        
        # 状态
        self.channel_states = {}  # 按RNTI分类的通道状态
        self.constraints = {}     # 按RNTI分类的约束
        self.phy_metrics = {}     # 按RNTI分类的PHY层指标
        self.strategy_history = {}  # 历史策略及其效果
        self.snr_history = {}     # SNR历史数据
        self.bler_history = {}    # BLER历史数据
        self.rtt_history = {}     # RTT历史数据
        self.buffer_history = {}  # 缓冲区历史数据
        
        # 策略状态
        self.current_strategies = {}  # 当前应用的策略
        self.strategy_performance = {}  # 策略性能评估
        
        # 数据流信息
        self.camera_data_rate = 0.0  # 相机数据速率 (MB/s)
        self.lidar_data_rate = 0.0   # LiDAR数据速率 (MB/s)
        self.total_data_rate = 0.0   # 总数据速率 (MB/s)
        self.camera_timestamp = 0    # 上次相机数据时间戳
        self.lidar_timestamp = 0     # 上次LiDAR数据时间戳
        
        # 强化学习状态
        self.rl_models = {}          # RNTI -> RL模型
        self.state_actions = {}      # 状态-动作历史
        self.q_values = {}           # Q值表
        self.adaptation_recommendation = "NORMAL_OPERATION"  # 当前适应建议
        self.task_criticality = 0.5  # 任务关键度 (0-1)
        
        # 约束映射表
        self.constraint_type_mapping = {
            "HIGH_QUALITY": {"priority": 5, "retx": 4, "strategy": "HIGH_THROUGHPUT"},
            "STANDARD": {"priority": 4, "retx": 3, "strategy": "BALANCED"},
            "BALANCED": {"priority": 3, "retx": 2, "strategy": "RELIABILITY"},
            "REDUCED": {"priority": 2, "retx": 1, "strategy": "LOW_LATENCY"},
            "MINIMUM": {"priority": 1, "retx": 0, "strategy": "EMERGENCY"},
            "CRITICAL_HIGH_QUALITY": {"priority": 5, "retx": 4, "strategy": "CRITICAL_THROUGHPUT"},
            "CRITICAL_STANDARD": {"priority": 5, "retx": 4, "strategy": "CRITICAL_BALANCED"},
            "CRITICAL_BALANCED": {"priority": 5, "retx": 3, "strategy": "CRITICAL_RELIABILITY"},
            "CRITICAL_REDUCED": {"priority": 4, "retx": 2, "strategy": "CRITICAL_LOW_LATENCY"},
            "CRITICAL_MINIMUM": {"priority": 3, "retx": 1, "strategy": "CRITICAL_EMERGENCY"},
            "EMERGENCY_HIGH_QUALITY": {"priority": 5, "retx": 3, "strategy": "EMERGENCY_THROUGHPUT"},
            "EMERGENCY_STANDARD": {"priority": 4, "retx": 2, "strategy": "EMERGENCY_BALANCED"},
            "EMERGENCY_BALANCED": {"priority": 3, "retx": 2, "strategy": "EMERGENCY_RELIABILITY"},
            "EMERGENCY_REDUCED": {"priority": 2, "retx": 1, "strategy": "EMERGENCY_LOW_LATENCY"},
            "EMERGENCY_MINIMUM": {"priority": 1, "retx": 0, "strategy": "CRITICAL_EMERGENCY"}
        }
        
        # 启动定时更新
        self.timer = rospy.Timer(rospy.Duration(self.update_interval), self.update_transmission_strategy)
        
        # 启动RTT测量线程
        if self.enable_rtt_measurement:
            self.rtt_thread = threading.Thread(target=self.measure_rtt_periodically)
            self.rtt_thread.daemon = True
            self.rtt_thread.start()
        
        # 启动学习线程
        if self.use_reinforcement_learning:
            self.learning_thread = threading.Thread(target=self.run_reinforcement_learning)
            self.learning_thread.daemon = True
            self.learning_thread.start()
        
        rospy.loginfo("PhysicalAdaptiveEngine initialized")

    def initialize_mcs_table(self):
        """初始化MCS表"""
        # 详细的MCS表，遵循5G NR标准
        # 格式: MCS索引 -> (调制方案, 编码率, 频谱效率, 最小所需SNR)
        mcs_table = {}
        
        # QPSK (调制阶数=2)
        for i in range(10):
            coding_rate = (i + 1) / 10.0
            spectral_efficiency = 2 * coding_rate * 0.8
            min_snr = -6.7 + i * 1.0  # 估算的最小SNR
            mcs_table[i] = {
                'modulation': 'QPSK',
                'coding_rate': coding_rate,
                'spectral_efficiency': spectral_efficiency,
                'min_snr': min_snr
            }
        
        # 16QAM (调制阶数=4)
        for i in range(10, 17):
            idx = i - 10
            coding_rate = (idx + 1) / 8.0
            spectral_efficiency = 4 * coding_rate * 0.8
            min_snr = 3.0 + idx * 1.0  # 估算的最小SNR
            mcs_table[i] = {
                'modulation': '16QAM',
                'coding_rate': coding_rate,
                'spectral_efficiency': spectral_efficiency,
                'min_snr': min_snr
            }
        
        # 64QAM (调制阶数=6)
        for i in range(17, 29):
            idx = i - 17
            coding_rate = (idx + 1) / 12.0
            spectral_efficiency = 6 * coding_rate * 0.8
            min_snr = 10.0 + idx * 1.0  # 估算的最小SNR
            mcs_table[i] = {
                'modulation': '64QAM',
                'coding_rate': coding_rate,
                'spectral_efficiency': spectral_efficiency,
                'min_snr': min_snr
            }
        
        # 256QAM (调制阶数=8) - 扩展MCS
        for i in range(29, 34):
            idx = i - 29
            coding_rate = (idx + 1) / 6.0
            spectral_efficiency = 8 * coding_rate * 0.8
            min_snr = 20.0 + idx * 1.5  # 估算的最小SNR
            mcs_table[i] = {
                'modulation': '256QAM',
                'coding_rate': coding_rate,
                'spectral_efficiency': spectral_efficiency,
                'min_snr': min_snr
            }
        
        return mcs_table

    def channel_state_callback(self, msg):
        """处理通道状态更新"""
        rnti = msg.rnti
        self.channel_states[rnti] = msg
        
        # 更新历史记录
        if rnti not in self.snr_history:
            self.snr_history[rnti] = deque(maxlen=100)
            self.bler_history[rnti] = deque(maxlen=100)
            self.buffer_history[rnti] = deque(maxlen=100)
        
        self.snr_history[rnti].append(msg.pusch_snr)
        self.bler_history[rnti].append(msg.dl_bler)
        self.buffer_history[rnti].append(msg.estimated_ul_buffer)
        
        # 初始化RL模型（如果不存在）
        if self.use_reinforcement_learning and rnti not in self.rl_models:
            self.initialize_rl_model(rnti)

    def constraint_callback(self, msg):
        """处理约束更新"""
        rnti = msg.rnti
        self.constraints[rnti] = msg

    def phy_metrics_callback(self, msg):
        """处理PHY层指标更新"""
        rnti = msg.rnti
        self.phy_metrics[rnti] = msg

    def camera_callback(self, msg):
        """处理相机数据并计算数据速率"""
        now = time.time()
        if self.camera_timestamp > 0:
            # 计算时间间隔
            dt = now - self.camera_timestamp
            
            # 计算数据大小 (字节)
            data_size = len(msg.data)
            
            # 更新数据速率 (MB/s)
            self.camera_data_rate = (data_size / (1024 * 1024)) / dt
            self.total_data_rate = self.camera_data_rate + self.lidar_data_rate
        
        self.camera_timestamp = now

    def lidar_callback(self, msg):
        """处理LiDAR数据并计算数据速率"""
        now = time.time()
        if self.lidar_timestamp > 0:
            # 计算时间间隔
            dt = now - self.lidar_timestamp
            
            # 计算数据大小 (字节)
            data_size = msg.row_step * msg.height
            
            # 更新数据速率 (MB/s)
            self.lidar_data_rate = (data_size / (1024 * 1024)) / dt
            self.total_data_rate = self.camera_data_rate + self.lidar_data_rate
        
        self.lidar_timestamp = now

    def adaptation_recommendation_callback(self, msg):
        """处理适应建议更新"""
        self.adaptation_recommendation = msg.data
        def task_criticality_callback(self, msg):
        """处理任务关键度更新"""
        self.task_criticality = msg.data

    def update_transmission_strategy(self, event=None):
        """根据当前通道状态和约束更新传输策略"""
        for rnti, state in self.channel_states.items():
            if rnti not in self.constraints:
                continue  # 没有对应的约束
            
            constraint = self.constraints[rnti]
            
            # 获取强化学习推荐（如果启用）
            rl_recommendation = None
            if self.use_reinforcement_learning and rnti in self.rl_models:
                rl_recommendation = self.get_rl_recommendation(rnti, state)
            
            # 生成新的传输策略
            strategy = self.optimize_transmission_parameters(rnti, state, constraint, rl_recommendation)
            
            # 评估并记录策略性能
            if rnti in self.current_strategies:
                self.evaluate_strategy_performance(rnti, self.current_strategies[rnti], state)
            
            # 记录策略历史
            if rnti not in self.strategy_history:
                self.strategy_history[rnti] = deque(maxlen=50)
            
            self.strategy_history[rnti].append(strategy)
            
            # 保存当前策略
            self.current_strategies[rnti] = strategy
            
            # 发布策略
            self.strategy_pub.publish(strategy)
            self.target_mcs_pub.publish(strategy.target_mcs)
            self.target_prb_pub.publish(strategy.target_prb)
            self.compression_ratio_pub.publish(strategy.compression_ratio)
            
            # 发布资源分配信息
            allocation_info = f"RNTI: {rnti}, MCS: {strategy.target_mcs}, PRB: {strategy.target_prb}, Compression: {strategy.compression_ratio:.2f}"
            self.resource_allocation_pub.publish(allocation_info)
            
            rospy.loginfo(f"Updated transmission strategy for RNTI {rnti}")

    def optimize_transmission_parameters(self, rnti, state, constraint, rl_recommendation=None):
        """优化传输参数"""
        strategy = TransmissionStrategy()
        strategy.header.stamp = rospy.Time.now()
        strategy.rnti = rnti
        
        # 获取历史数据
        snr_values = list(self.snr_history[rnti])
        bler_values = list(self.bler_history[rnti])
        
        # 平均SNR和稳定性
        recent_snr = snr_values[-10:] if len(snr_values) >= 10 else snr_values
        recent_bler = bler_values[-10:] if len(bler_values) >= 10 else bler_values
        
        avg_snr = np.mean(recent_snr)
        avg_bler = np.mean(recent_bler)
        
        snr_stability = 1.0 - min(1.0, np.std(recent_snr) / 5.0) if len(recent_snr) >= 5 else 0.5
        bler_stability = 1.0 - min(1.0, np.std(recent_bler) * 10) if len(recent_bler) >= 5 else 0.5
        
        # 决定是利用还是探索
        use_rl = self.use_reinforcement_learning and rl_recommendation is not None
        exploit = np.random.random() > self.get_current_exploration_rate(rnti)
        
        # 确定MCS和PRB
        if use_rl and exploit:
            # 使用强化学习推荐
            target_mcs, target_prb, compression_ratio = rl_recommendation
            strategy.target_mcs = target_mcs
            strategy.target_prb = target_prb
            strategy.compression_ratio = compression_ratio
            
            self.adaptive_action_pub.publish("USING_RL_RECOMMENDATION")
            
        elif exploit and rnti in self.current_strategies:
            # 利用现有策略，根据反馈微调
            prev_strategy = self.current_strategies[rnti]
            
            # 自适应MCS调整
            target_mcs = self.adapt_mcs_based_on_feedback(rnti, prev_strategy.target_mcs, avg_snr, avg_bler, snr_stability)
            
            # 根据约束调整PRB
            target_prb, compression_ratio = self.calculate_resource_needs(rnti, constraint, target_mcs)
            
            strategy.target_mcs = target_mcs
            strategy.target_prb = target_prb
            strategy.compression_ratio = compression_ratio
            
            self.adaptive_action_pub.publish("EXPLOITING_PREVIOUS_STRATEGY")
            
        else:
            # 探索新策略
            # 基于SNR估计合适的MCS
            target_mcs = self.select_mcs_for_snr(avg_snr, constraint.min_quality / 100.0)
            
            # 根据约束计算PRB需求
            target_prb, compression_ratio = self.calculate_resource_needs(rnti, constraint, target_mcs)
            
            strategy.target_mcs = target_mcs
            strategy.target_prb = target_prb
            strategy.compression_ratio = compression_ratio
            
            self.adaptive_action_pub.publish("EXPLORING_NEW_STRATEGY")
        
        # 根据约束类型和映射表设置其他参数
        constraint_type = constraint.constraint_type
        if constraint_type in self.constraint_type_mapping:
            mapping = self.constraint_type_mapping[constraint_type]
            strategy.priority = mapping["priority"]
            strategy.retransmission_enabled = mapping["retx"] > 0
            strategy.max_harq_retx = mapping["retx"]
            strategy.strategy_type = mapping["strategy"]
        else:
            # 默认值
            strategy.priority = 3
            strategy.retransmission_enabled = True
            strategy.max_harq_retx = 2
            strategy.strategy_type = "BALANCED"
        
        # 自适应HARQ策略
        if self.enable_adaptive_harq:
            self.adapt_harq_strategy(rnti, strategy, avg_bler, bler_stability)
        
        # 根据适应建议微调策略
        self.adjust_strategy_based_on_recommendation(strategy)
        
        # 应用预测性编码（如果启用）
        if self.enable_predictive_coding and self.predict_congestion(rnti):
            strategy.compression_ratio = min(0.95, strategy.compression_ratio * 1.2)  # 增加压缩
            strategy.priority = max(1, strategy.priority - 1)  # 降低优先级
        
        return strategy

    def adapt_mcs_based_on_feedback(self, rnti, current_mcs, avg_snr, avg_bler, snr_stability):
        """基于BLER和SNR反馈调整MCS"""
        # BLER目标范围
        bler_target_min = 0.01
        bler_target_max = 0.1
        
        # MCS调整步长
        mcs_step = 1
        
        # 获取当前MCS的理论最小SNR要求
        min_snr_required = self.mcs_table.get(current_mcs, {}).get('min_snr', 0)
        
        # 自适应MCS调整逻辑
        if avg_bler > bler_target_max * 1.5:
            # BLER远高于目标，大幅降低MCS
            target_mcs = max(0, current_mcs - 2 * mcs_step)
            
        elif avg_bler > bler_target_max:
            # BLER高于目标，降低MCS
            target_mcs = max(0, current_mcs - mcs_step)
            
        elif avg_bler < bler_target_min and snr_stability > 0.7:
            # BLER低于目标且SNR稳定，可以尝试提高MCS
            
            # 检查SNR是否足够支持下一级MCS
            next_mcs = current_mcs + mcs_step
            if next_mcs in self.mcs_table:
                next_min_snr = self.mcs_table[next_mcs]['min_snr']
                if avg_snr > next_min_snr + 3.0:  # 添加3dB余量
                    target_mcs = next_mcs
                else:
                    target_mcs = current_mcs  # 保持当前MCS
            else:
                target_mcs = current_mcs  # 保持当前MCS
                
        else:
            # BLER在目标范围内，保持当前MCS
            target_mcs = current_mcs
        
        return target_mcs

    def select_mcs_for_snr(self, snr, reliability_factor=0.9):
        """为给定SNR选择合适的MCS"""
        # 添加裕度，reliability_factor越高，选择越保守
        margin_db = 10.0 * (1.0 - reliability_factor)  # 0-10dB的裕度
        effective_snr = snr - margin_db
        
        # 找到所有SNR要求小于effective_snr的MCS
        valid_mcs = []
        for mcs_idx, mcs_info in self.mcs_table.items():
            if mcs_info['min_snr'] <= effective_snr:
                valid_mcs.append((mcs_idx, mcs_info['spectral_efficiency']))
        
        if not valid_mcs:
            # 如果没有合适的MCS，使用最低MCS
            return 0
        
        # 按频谱效率排序
        valid_mcs.sort(key=lambda x: x[1], reverse=True)
        
        # 返回频谱效率最高的MCS
        return valid_mcs[0][0]

    def calculate_resource_needs(self, rnti, constraint, target_mcs):
        """计算满足约束所需的资源"""
        # 获取MCS对应的频谱效率
        spectral_efficiency = self.mcs_table.get(target_mcs, {}).get('spectral_efficiency', 1.0)
        
        # 计算所需数据速率（考虑压缩）
        required_data_rate = constraint.max_data_rate  # Mbps
        
        # 计算所需PRB数量
        # 假设每个PRB的带宽为0.18MHz (5G NR, 15kHz子载波间隔)
        resource_blocks_needed = int(required_data_rate / (spectral_efficiency * 0.18))
        
        # 根据可用资源确定是否需要压缩
        # 假设最大可用PRB为100
        max_available_prbs = 100
        
        if resource_blocks_needed <= max_available_prbs:
            # 资源充足，不需要压缩
            target_prb = resource_blocks_needed
            compression_ratio = 1.0
        else:
            # 资源不足，需要压缩
            target_prb = max_available_prbs
            compression_needed = resource_blocks_needed / max_available_prbs
            compression_ratio = 1.0 / compression_needed
        
        # 确保compression_ratio在有效范围内（0.05-1.0）
        compression_ratio = max(0.05, min(1.0, compression_ratio))
        
        return target_prb, compression_ratio

    def adapt_harq_strategy(self, rnti, strategy, avg_bler, bler_stability):
        """自适应HARQ策略"""
        if not strategy.retransmission_enabled:
            return  # 如果重传被禁用，不做调整
        
        # 基于BLER和稳定性调整HARQ参数
        if avg_bler > 0.15:
            # 高BLER，增加重传次数
            strategy.max_harq_retx = min(self.max_harq_retx, strategy.max_harq_retx + 1)
        elif avg_bler < 0.01 and bler_stability > 0.8:
            # 非常低的BLER且稳定，可以减少重传
            strategy.max_harq_retx = max(0, strategy.max_harq_retx - 1)
        
        # 考虑任务关键度
        if self.task_criticality > 0.8:
            # 高关键度任务需要更多重传保证可靠性
            strategy.max_harq_retx = min(self.max_harq_retx, strategy.max_harq_retx + 1)

    def adjust_strategy_based_on_recommendation(self, strategy):
        """根据跨域引擎的适应建议调整策略"""
        recommendation = self.adaptation_recommendation
        
        if recommendation == "MAXIMIZE_DATA_COLLECTION":
            # 最大化数据收集，提高MCS和PRB
            strategy.target_mcs = min(28, strategy.target_mcs + 1)
            strategy.target_prb = min(100, strategy.target_prb + 10)
            strategy.compression_ratio = min(1.0, strategy.compression_ratio + 0.1)
            
        elif recommendation == "MODERATE_CONSERVATION":
            # 中度保守，稍微降低MCS，增加重传
            strategy.target_mcs = max(0, strategy.target_mcs - 1)
            strategy.max_harq_retx = min(self.max_harq_retx, strategy.max_harq_retx + 1)
            
        elif recommendation == "SIGNIFICANT_REDUCTION":
            # 显著降低资源使用
            strategy.target_mcs = max(0, strategy.target_mcs - 2)
            strategy.target_prb = max(10, strategy.target_prb - 20)
            strategy.compression_ratio = max(0.3, strategy.compression_ratio - 0.2)
            
        elif recommendation == "EMERGENCY_CONSERVATION":
            # 紧急情况，最小化资源使用
            strategy.target_mcs = max(0, strategy.target_mcs - 4)
            strategy.target_prb = max(5, strategy.target_prb - 30)
            strategy.compression_ratio = max(0.2, strategy.compression_ratio - 0.3)
            strategy.priority = max(1, strategy.priority - 2)  # 降低优先级

    def measure_rtt_periodically(self):
        """定期测量RTT（往返时间）"""
        while not rospy.is_shutdown():
            for rnti in self.channel_states.keys():
                if rnti not in self.rtt_history:
                    self.rtt_history[rnti] = deque(maxlen=100)
                
                # 在实际实现中，这里会进行真正的RTT测量
                # 现在简单模拟
                simulated_rtt = 20.0  # 基础RTT (ms)
                
                # 根据当前BLER模拟RTT波动
                if rnti in self.bler_history and self.bler_history[rnti]:
                    recent_bler = self.bler_history[rnti][-1]
                    # BLER增加会导致RTT增加（由于重传）
                    rtt_increase = recent_bler * 100.0  # BLER为0.1时增加10ms
                    simulated_rtt += rtt_increase
                
                # 添加一些随机波动
                simulated_rtt += np.random.normal(0, 2.0)
                
                # 确保RTT为正值
                simulated_rtt = max(1.0, simulated_rtt)
                
                # 记录RTT
                self.rtt_history[rnti].append(simulated_rtt)
            
            time.sleep(0.5)  # 每500ms测量一次

    def predict_congestion(self, rnti):
        """预测网络拥塞"""
        # 检查缓冲区增长
        if rnti in self.buffer_history and len(self.buffer_history[rnti]) >= 10:
            recent_buffers = list(self.buffer_history[rnti])[-10:]
            
            # 计算缓冲区增长率
            if np.mean(recent_buffers[-3:]) > np.mean(recent_buffers[:7]) * 1.5:
                # 缓冲区快速增长
                return True
        
        # 检查RTT增长
        if rnti in self.rtt_history and len(self.rtt_history[rnti]) >= 10:
            recent_rtts = list(self.rtt_history[rnti])[-10:]
            
            # 计算RTT增长率
            if np.mean(recent_rtts[-3:]) > np.mean(recent_rtts[:7]) * 1.3:
                # RTT快速增长
                return True
        
        return False

    def evaluate_strategy_performance(self, rnti, strategy, current_state):
        """评估策略性能"""
        if rnti not in self.strategy_performance:
            self.strategy_performance[rnti] = []
        
        # 计算性能指标
        bler = current_state.dl_bler
        spectral_efficiency = self.mcs_table.get(strategy.target_mcs, {}).get('spectral_efficiency', 0)
        throughput = current_state.dl_thr
        
        # 计算综合性能得分
        # 低BLER表示高可靠性，高频谱效率和高吞吐量表示高效率
        reliability_score = 1.0 - min(1.0, bler * 10.0)  # BLER 0.1及以上得分为0
        efficiency_score = min(1.0, spectral_efficiency / 5.0)  # 频谱效率为5及以上得分为1
        throughput_score = min(1.0, throughput / 20.0)  # 吞吐量为20Mbps及以上得分为1
        
        # 综合评分 (0-1)
        performance_score = reliability_score * 0.4 + efficiency_score * 0.3 + throughput_score * 0.3
        
        # 记录性能
        self.strategy_performance[rnti].append({
            'strategy': strategy,
            'bler': bler,
            'spectral_efficiency': spectral_efficiency,
            'throughput': throughput,
            'score': performance_score,
            'timestamp': time.time()
        })
        
        # 只保留最近50条记录
        if len(self.strategy_performance[rnti]) > 50:
            self.strategy_performance[rnti] = self.strategy_performance[rnti][-50:]
        
        # 发布成功率
        packet_success_rate = 1.0 - bler
        self.packet_success_rate_pub.publish(packet_success_rate)
        
        # 如果使用强化学习，更新模型
        if self.use_reinforcement_learning and rnti in self.rl_models:
            self.update_rl_model(rnti, strategy, performance_score)

    def initialize_rl_model(self, rnti):
        """初始化RL模型"""
        self.rl_models[rnti] = {
            'state_size': 4,  # SNR, BLER, Buffer Size, Task Criticality
            'action_size': 3,  # MCS, PRB, Compression Ratio
            'q_table': {},
            'learning_rate': self.learning_rate,
            'discount_factor': 0.9,
            'last_state': None,
            'last_action': None,
            'episodes': 0
        }
        
        self.q_values[rnti] = {}
        self.state_actions[rnti] = []
        
        rospy.loginfo(f"Initialized RL model for RNTI {rnti}")

    def get_current_exploration_rate(self, rnti):
        """获取当前探索率（随时间衰减）"""
        if rnti not in self.rl_models:
            return self.exploration_rate
        
        # 随着经验的积累，逐渐减少探索率
        episodes = self.rl_models[rnti]['episodes']
        return max(self.min_exploration_rate, self.exploration_rate * np.exp(-0.01 * episodes))

    def get_rl_recommendation(self, rnti, state):
        """获取RL推荐的传输策略"""
        # 将当前状态离散化
        current_state = self.discretize_state(rnti, state)
        
        # 检查状态是否已在Q表中
        q_table = self.rl_models[rnti]['q_table']
        if current_state not in q_table:
            q_table[current_state] = np.zeros((29, 10, 5))  # MCS(0-28), PRB(10-100,步长10), 压缩比(0.2-1.0,步长0.2)
        
        # 选择最佳动作
        q_values = q_table[current_state]
        best_action_idx = np.unravel_index(np.argmax(q_values), q_values.shape)
        
        # 将索引转换为实际动作值
        mcs = best_action_idx[0]
        prb = (best_action_idx[1] + 1) * 10  # 10-100
        compression = 0.2 + best_action_idx[2] * 0.2  # 0.2-1.0
        
        # 保存当前状态和动作
        self.rl_models[rnti]['last_state'] = current_state
        self.rl_models[rnti]['last_action'] = best_action_idx
        
        return mcs, prb, compression

    def discretize_state(self, rnti, state):
        """将连续状态离散化"""
        # 获取当前值
        snr = state.pusch_snr
        bler = state.dl_bler
        buffer = state.estimated_ul_buffer
        
        # 离散化SNR (-10 to 30 dB, 5个区间)
        snr_idx = min(4, max(0, int((snr + 10) / 8)))
        
        # 离散化BLER (0 to 0.5, 5个区间)
        bler_idx = min(4, max(0, int(bler * 10)))
        
        # 离散化Buffer (0 to 100000, 5个区间)
        buffer_idx = min(4, max(0, int(buffer / 20000)))
        
        # 离散化任务关键度 (0 to 1, 3个区间)
        criticality_idx = int(self.task_criticality * 3)
        
        # 组合为状态元组
        return (snr_idx, bler_idx, buffer_idx, criticality_idx)

    def update_rl_model(self, rnti, strategy, reward):
        """更新RL模型"""
        model = self.rl_models[rnti]
        
        # 如果没有前一个状态-动作对，无法更新
        if model['last_state'] is None or model['last_action'] is None:
            return
        
        # 获取前一个状态、动作和当前状态
        last_state = model['last_state']
        last_action = model['last_action']
        
        # 从Q表获取前一个状态-动作对的Q值
        q_table = model['q_table']
        old_q_value = q_table[last_state][last_action]
        
        # 更新Q值 (Q-learning公式)
        # Q(s,a) = Q(s,a) + alpha * (r + gamma * max(Q(s',a')) - Q(s,a))
        max_next_q = np.max(q_table[last_state])
        new_q_value = old_q_value + model['learning_rate'] * (reward + model['discount_factor'] * max_next_q - old_q_value)
        
        # 更新Q表
        q_table[last_state][last_action] = new_q_value
        
        # 记录Q值
        if rnti not in self.q_values:
            self.q_values[rnti] = {}
        action_key = f"{last_action[0]}_{last_action[1]}_{last_action[2]}"
        if action_key not in self.q_values[rnti]:
            self.q_values[rnti][action_key] = []
        self.q_values[rnti][action_key].append(new_q_value)
        
        # 记录状态-动作对
        self.state_actions[rnti].append((last_state, last_action, reward))
        
        # 增加经验计数
        model['episodes'] += 1
        
        # 每100次更新输出一次学习状态
        if model['episodes'] % 100 == 0:
            learning_status = f"RL Model for RNTI {rnti}: Episodes={model['episodes']}, Exploration_rate={self.get_current_exploration_rate(rnti):.3f}"
            self.learning_status_pub.publish(learning_status)

    def run_reinforcement_learning(self):
        """运行强化学习模型的后台任务"""
        while not rospy.is_shutdown():
            # 对每个RNTI进行模型分析和优化
            for rnti, model in self.rl_models.items():
                # 执行周期性模型分析
                if model['episodes'] > 0 and model['episodes'] % 1000 == 0:
                    self.analyze_model_performance(rnti)
            
            time.sleep(60.0)  # 每分钟运行一次

    def analyze_model_performance(self, rnti):
        """分析RL模型性能"""
        if rnti not in self.state_actions or not self.state_actions[rnti]:
            return
        
        # 获取最近1000个状态-动作对
        recent_actions = self.state_actions[rnti][-1000:]
        
        # 计算平均奖励
        avg_reward = np.mean([r for _, _, r in recent_actions])
        
        # 计算状态覆盖率
        unique_states = set([s for s, _, _ in recent_actions])
        state_coverage = len(unique_states)
        
        # 输出分析结果
        analysis = f"RL Analysis for RNTI {rnti}: Avg_reward={avg_reward:.3f}, State_coverage={state_coverage}"
        rospy.loginfo(analysis)
        
        # 根据分析结果调整学习参数
        if avg_reward < 0.3:
            # 如果平均奖励低，增加学习率以加快收敛
            self.rl_models[rnti]['learning_rate'] = min(0.5, self.rl_models[rnti]['learning_rate'] * 1.2)
        elif avg_reward > 0.7:
            # 如果平均奖励高，减小学习率以提高稳定性
            self.rl_models[rnti]['learning_rate'] = max(0.01, self.rl_models[rnti]['learning_rate'] * 0.8)

if __name__ == '__main__':
    try:
        engine = PhysicalAdaptiveEngine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
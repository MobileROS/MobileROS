#!/usr/bin/env python3
import rospy
import numpy as np
import threading
import time
from collections import deque
from wireless_ros.msgs.msg import ChannelState, Constraint, PhyLayerMetrics, SpectrumAnalysis
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray

class CrossDomainEngine:
    """
    跨域引擎 - 融合通信和机器人领域的知识
    
    该引擎将通信层的见解转化为机器人任务的可操作输入，实现：
    1. 通信感知的地图增强
    2. 网络状况到机器人约束的转换
    3. 任务关键度分析
    4. 资源和任务优先级动态分配
    """
    
    def __init__(self):
        rospy.init_node('cross_domain_engine')
        
        # 高级参数
        self.update_interval = rospy.get_param('~update_interval', 0.2)  # 200ms
        self.fusion_window_size = rospy.get_param('~fusion_window_size', 10)
        self.enable_map_enhancement = rospy.get_param('~enable_map_enhancement', True)
        self.enable_path_optimization = rospy.get_param('~enable_path_optimization', True)
        self.enable_sensor_adaptation = rospy.get_param('~enable_sensor_adaptation', True)
        self.enable_task_criticality = rospy.get_param('~enable_task_criticality', True)
        
        # 订阅者 - 通信域
        rospy.Subscriber('/wireless_ros/channel_state', ChannelState, self.channel_state_callback)
        rospy.Subscriber('/wireless_ros/phy_layer_metrics', PhyLayerMetrics, self.phy_metrics_callback)
        rospy.Subscriber('/wireless_ros/spectrum_analysis', SpectrumAnalysis, self.spectrum_analysis_callback)
        rospy.Subscriber('/wireless_ros/bandwidth_forecast', Float32, self.bandwidth_forecast_callback)
        rospy.Subscriber('/wireless_ros/latency_forecast', Float32, self.latency_forecast_callback)
        rospy.Subscriber('/wireless_ros/interference_zone', String, self.interference_zone_callback)
        
        # 订阅者 - 机器人域
        rospy.Subscriber('/robot/pose', PoseStamped, self.robot_pose_callback)
        rospy.Subscriber('/robot/velocity', TwistStamped, self.robot_velocity_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/planned_path', Path, self.planned_path_callback)
        rospy.Subscriber('/active_sensors', MarkerArray, self.active_sensors_callback)
        rospy.Subscriber('/task_status', String, self.task_status_callback)
        
        # 发布者 - 约束和融合结果
        self.constraint_pub = rospy.Publisher('/wireless_ros/constraints', Constraint, queue_size=10)
        self.network_quality_pub = rospy.Publisher('/wireless_ros/network_quality', String, queue_size=10)
        self.latency_estimate_pub = rospy.Publisher('/wireless_ros/latency_estimate', Float32, queue_size=10)
        self.bandwidth_estimate_pub = rospy.Publisher('/wireless_ros/bandwidth_estimate', Float32, queue_size=10)
        self.reliability_score_pub = rospy.Publisher('/wireless_ros/reliability_score', Float32, queue_size=10)
        
        # 发布者 - 跨域融合结果
        self.enhanced_map_pub = rospy.Publisher('/wireless_ros/enhanced_map', OccupancyGrid, queue_size=2)
        self.communication_aware_path_pub = rospy.Publisher('/wireless_ros/comm_aware_path', Path, queue_size=2)
        self.sensor_mode_recommendation_pub = rospy.Publisher('/wireless_ros/sensor_mode', String, queue_size=10)
        self.task_criticality_pub = rospy.Publisher('/wireless_ros/task_criticality', Float32, queue_size=10)
        self.adaptation_recommendation_pub = rospy.Publisher('/wireless_ros/adaptation_recommendation', String, queue_size=10)
        self.data_compression_level_pub = rospy.Publisher('/wireless_ros/compression_level', Float32, queue_size=10)
        
        # 状态变量
        self.channel_states = {}  # RNTI -> 最新通道状态
        self.phy_metrics = {}     # RNTI -> 最新PHY层指标
        self.spectrum_data = {}   # RNTI -> 最新频谱分析
        self.bandwidth_forecast = 0.0  # 带宽预测
        self.latency_forecast = 0.0    # 延迟预测
        self.interference_zones = []   # 干扰区域列表
        
        self.robot_pose = None        # 机器人位置
        self.robot_velocity = None    # 机器人速度
        self.current_map = None       # 当前地图
        self.planned_path = None      # 规划路径
        self.active_sensors = []      # 活跃传感器
        self.task_status = "IDLE"     # 任务状态
        
        # 历史记录
        self.channel_history = {}     # RNTI -> 通道历史
        self.robot_pose_history = deque(maxlen=100)  # 位置历史
        self.network_quality_history = deque(maxlen=100)  # 网络质量历史
        
        # 融合状态
        self.network_quality_map = None  # 网络质量地图
        self.last_map_update = 0       # 上次地图更新时间
        self.comm_enhanced_paths = {}  # 通信增强的路径
        self.sensor_recommendations = {}  # 传感器推荐
        self.task_criticality = 0.5    # 任务关键度 (0-1)
        
        # 启动定时更新
        self.timer = rospy.Timer(rospy.Duration(self.update_interval), self.update_cross_domain_fusion)
        
        # 启动后台处理线程
        self.map_enhancement_thread = None
        self.path_optimization_thread = None
        if self.enable_map_enhancement:
            self.map_enhancement_thread = threading.Thread(target=self.run_map_enhancement)
            self.map_enhancement_thread.daemon = True
            self.map_enhancement_thread.start()
        
        if self.enable_path_optimization:
            self.path_optimization_thread = threading.Thread(target=self.run_path_optimization)
            self.path_optimization_thread.daemon = True
            self.path_optimization_thread.start()
        
        rospy.loginfo("CrossDomainEngine initialized")

    def channel_state_callback(self, msg):
        """处理通道状态更新"""
        rnti = msg.rnti
        self.channel_states[rnti] = msg
        
        # 更新历史记录
        if rnti not in self.channel_history:
            self.channel_history[rnti] = deque(maxlen=self.fusion_window_size)
        
        self.channel_history[rnti].append(msg)

    def phy_metrics_callback(self, msg):
        """处理PHY层指标更新"""
        rnti = msg.rnti
        self.phy_metrics[rnti] = msg

    def spectrum_analysis_callback(self, msg):
        """处理频谱分析更新"""
        rnti = msg.rnti
        self.spectrum_data[rnti] = msg

    def bandwidth_forecast_callback(self, msg):
        """处理带宽预测更新"""
        self.bandwidth_forecast = msg.data

    def latency_forecast_callback(self, msg):
        """处理延迟预测更新"""
        self.latency_forecast = msg.data

    def interference_zone_callback(self, msg):
        """处理干扰区域通知"""
        zone_type = msg.data
        if zone_type and zone_type not in self.interference_zones:
            self.interference_zones.append(zone_type)
            rospy.loginfo(f"New interference zone detected: {zone_type}")

    def robot_pose_callback(self, msg):
        """处理机器人位置更新"""
        self.robot_pose = msg
        self.robot_pose_history.append(msg)
        
        # 更新位置与通信质量的关联
        if self.robot_pose and self.channel_states:
            # 为当前位置记录通信质量
            for rnti, state in self.channel_states.items():
                if rnti in self.phy_metrics:
                    pos = (msg.pose.position.x, msg.pose.position.y)
                    quality = self.phy_metrics[rnti].link_quality
                    self.update_network_quality_at_position(pos, quality)

    def robot_velocity_callback(self, msg):
        """处理机器人速度更新"""
        self.robot_velocity = msg

    def map_callback(self, msg):
        """处理地图更新"""
        self.current_map = msg
        
        # 初始化网络质量地图
        if self.network_quality_map is None and self.enable_map_enhancement:
            self.initialize_network_quality_map(msg)

    def planned_path_callback(self, msg):
        """处理路径规划更新"""
        self.planned_path = msg
        
        # 分析规划路径的通信质量
        if self.enable_path_optimization and self.network_quality_map is not None:
            self.analyze_path_communication_quality(msg)

    def active_sensors_callback(self, msg):
        """处理活跃传感器更新"""
        self.active_sensors = msg

    def task_status_callback(self, msg):
        """处理任务状态更新"""
        self.task_status = msg.data
        
        # 更新任务关键度
        if self.enable_task_criticality:
            self.evaluate_task_criticality()

    def update_cross_domain_fusion(self, event=None):
        """更新跨域融合和约束"""
        if not self.channel_states:
            return  # 没有通道数据
        
        # 对每个RNTI处理
        for rnti, state in self.channel_states.items():
            if rnti not in self.phy_metrics:
                continue  # 缺少PHY层指标
            
            # 计算并发布网络质量指标
            network_quality = self.calculate_network_quality(rnti)
            self.network_quality_pub.publish(network_quality)
            
            # 存储网络质量历史
            self.network_quality_history.append(network_quality)
            
            # 估计延迟和带宽
            latency_estimate = self.estimate_latency(rnti)
            bandwidth_estimate = self.estimate_bandwidth(rnti)
            reliability_score = self.calculate_reliability_score(rnti)
            
            # 发布估计
            self.latency_estimate_pub.publish(latency_estimate)
            self.bandwidth_estimate_pub.publish(bandwidth_estimate)
            self.reliability_score_pub.publish(reliability_score)
            
            # 生成约束
            constraint = self.generate_constraint_from_network(
                rnti,
                network_quality,
                latency_estimate,
                bandwidth_estimate,
                reliability_score
            )
            
            # 发布约束
            self.constraint_pub.publish(constraint)
            
            # 基于当前状态生成传感器模式建议
            if self.enable_sensor_adaptation:
                sensor_mode = self.recommend_sensor_mode(rnti, network_quality, bandwidth_estimate)
                self.sensor_mode_recommendation_pub.publish(sensor_mode)
            
            # 计算并发布数据压缩级别
            compression_level = self.calculate_data_compression_level(rnti)
            self.data_compression_level_pub.publish(compression_level)
            
            rospy.loginfo(f"Updated cross-domain fusion for RNTI {rnti}: {network_quality}")
        
        # 发布通信感知适应建议
        adaptation = self.generate_adaptation_recommendation()
        self.adaptation_recommendation_pub.publish(adaptation)

    def calculate_network_quality(self, rnti):
        """计算网络质量级别"""
        # 使用PHY层指标
        link_quality = self.phy_metrics[rnti].link_quality
        reliability = self.phy_metrics[rnti].reliability
        
        # 综合评估
        overall_quality = link_quality * 0.6 + reliability * 0.4
        
        # 分级
        if overall_quality > 0.8:
            return "EXCELLENT"
        elif overall_quality > 0.6:
            return "GOOD"
        elif overall_quality > 0.4:
            return "FAIR"
        elif overall_quality > 0.2:
            return "POOR"
        else:
            return "BAD"

    def estimate_latency(self, rnti):
        """估计当前延迟"""
        # 如果有预测，使用预测值
        if self.latency_forecast > 0:
            return self.latency_forecast
        
        # 否则使用PHY层指标中的响应时间
        return self.phy_metrics[rnti].channel_response_time

    def estimate_bandwidth(self, rnti):
        """估计当前带宽"""
        # 如果有预测，使用预测值
        if self.bandwidth_forecast > 0:
            return self.bandwidth_forecast
        
        # 否则使用通道状态中的下行吞吐量
        return self.channel_states[rnti].dl_thr

    def calculate_reliability_score(self, rnti):
        """计算可靠性评分"""
        # 使用PHY层指标中的可靠性
        return self.phy_metrics[rnti].reliability

    def generate_constraint_from_network(self, rnti, network_quality, latency, bandwidth, reliability):
        """根据网络状态生成应用约束"""
        constraint = Constraint()
        constraint.header.stamp = rospy.Time.now()
        constraint.rnti = rnti
        
        # 根据网络质量设置约束
        if network_quality == "EXCELLENT":
            constraint.max_data_rate = bandwidth
            constraint.min_quality = 90
            constraint.max_latency = latency
            constraint.drop_non_essential = False
            constraint.frame_rate = 30
            constraint.resolution_scale = 100
            constraint.constraint_type = "HIGH_QUALITY"
            
        elif network_quality == "GOOD":
            constraint.max_data_rate = bandwidth * 0.8
            constraint.min_quality = 80
            constraint.max_latency = latency * 1.2
            constraint.drop_non_essential = False
            constraint.frame_rate = 30
            constraint.resolution_scale = 80
            constraint.constraint_type = "STANDARD"
            
        elif network_quality == "FAIR":
            constraint.max_data_rate = bandwidth * 0.6
            constraint.min_quality = 70
            constraint.max_latency = latency * 1.5
            constraint.drop_non_essential = False
            constraint.frame_rate = 20
            constraint.resolution_scale = 70
            constraint.constraint_type = "BALANCED"
            
        elif network_quality == "POOR":
            constraint.max_data_rate = bandwidth * 0.4
            constraint.min_quality = 50
            constraint.max_latency = latency * 2.0
            constraint.drop_non_essential = True
            constraint.frame_rate = 15
            constraint.resolution_scale = 50
            constraint.constraint_type = "REDUCED"
            
        else:  # "BAD"
            constraint.max_data_rate = bandwidth * 0.2
            constraint.min_quality = 30
            constraint.max_latency = latency * 3.0
            constraint.drop_non_essential = True
            constraint.frame_rate = 10
            constraint.resolution_scale = 30
            constraint.constraint_type = "MINIMUM"
        
        # 根据任务关键度调整约束
        if self.task_criticality > 0.8:
            # 高关键度任务需要更可靠的通信
            constraint.constraint_type = "CRITICAL_" + constraint.constraint_type
            constraint.min_quality = min(95, constraint.min_quality + 10)
            constraint.drop_non_essential = False  # 关键任务不丢弃数据
        
        # 根据可靠性调整约束
        if reliability < 0.3:
            # 低可靠性时更激进地降低要求
            constraint.drop_non_essential = True
            constraint.frame_rate = max(5, constraint.frame_rate // 2)
            constraint.constraint_type = "EMERGENCY_" + constraint.constraint_type
        
        return constraint

    def initialize_network_quality_map(self, map_msg):
        """初始化网络质量地图"""
        if not map_msg:
            return
        
        # 创建与地图相同尺寸的网络质量地图
        self.network_quality_map = {
            'width': map_msg.info.width,
            'height': map_msg.info.height,
            'resolution': map_msg.info.resolution,
            'origin_x': map_msg.info.origin.position.x,
            'origin_y': map_msg.info.origin.position.y,
            'data': np.full((map_msg.info.height, map_msg.info.width), -1.0)  # -1表示未知
        }
        
        rospy.loginfo(f"Initialized network quality map: {map_msg.info.width}x{map_msg.info.height}")

    def update_network_quality_at_position(self, position, quality):
        """更新特定位置的网络质量"""
        if not self.network_quality_map:
            return
        
        # 将位置转换为地图坐标
        map_x = int((position[0] - self.network_quality_map['origin_x']) / self.network_quality_map['resolution'])
        map_y = int((position[1] - self.network_quality_map['origin_y']) / self.network_quality_map['resolution'])
        
        # 检查坐标是否在地图范围内
        if (0 <= map_x < self.network_quality_map['width'] and 
            0 <= map_y < self.network_quality_map['height']):
            
            # 指数移动平均更新
            alpha = 0.3  # 更新权重
            current_val = self.network_quality_map['data'][map_y, map_x]
            
            if current_val < 0:
                # 首次更新
                self.network_quality_map['data'][map_y, map_x] = quality
            else:
                # 增量更新
                self.network_quality_map['data'][map_y, map_x] = alpha * quality + (1 - alpha) * current_val

    def run_map_enhancement(self):
        """运行地图增强"""
        while not rospy.is_shutdown():
            if not self.network_quality_map or not self.current_map:
                time.sleep(1.0)
                continue
            
            current_time = time.time()
            # 每5秒更新一次地图
            if current_time - self.last_map_update > 5.0:
                self.last_map_update = current_time
                self.enhance_map_with_network_quality()
            
            time.sleep(1.0)

    def enhance_map_with_network_quality(self):
        """用网络质量增强地图"""
        if not self.network_quality_map or not self.current_map:
            return
        
        # 创建一个新的地图消息
        enhanced_map = OccupancyGrid()
        enhanced_map.header.stamp = rospy.Time.now()
        enhanced_map.header.frame_id = self.current_map.header.frame_id
        enhanced_map.info = self.current_map.info
        
        # 复制原始地图数据
        original_data = np.array(self.current_map.data).reshape(self.current_map.info.height, self.current_map.info.width)
        
        # 创建增强数据
        enhanced_data = np.copy(original_data).astype(np.int8)
        
        # 对有网络质量数据的单元格进行增强
        quality_data = self.network_quality_map['data']
        for y in range(self.network_quality_map['height']):
            for x in range(self.network_quality_map['width']):
                if quality_data[y, x] >= 0.0:
                    # 只增强自由空间 (值为0)
                    if original_data[y, x] == 0:
                        # 根据网络质量设置增强值 (1-49为自定义值)
                        # 网络质量越低，值越高 (更倾向于避开)
                        quality_cost = int((1.0 - quality_data[y, x]) * 49)
                        enhanced_data[y, x] = max(1, quality_cost)
        
        # 更新地图数据
        enhanced_map.data = enhanced_data.flatten().tolist()
        
        # 发布增强地图
        self.enhanced_map_pub.publish(enhanced_map)
        
        rospy.loginfo("Published network-enhanced map")

    def run_path_optimization(self):
        """运行路径优化"""
        while not rospy.is_shutdown():
            if not self.planned_path or not self.network_quality_map:
                time.sleep(1.0)
                continue
            
            # 优化路径考虑网络质量
            self.optimize_path_for_communication()
            
            time.sleep(5.0)  # 每5秒运行一次

    def analyze_path_communication_quality(self, path_msg):
        """分析路径上的通信质量"""
        if not self.network_quality_map:
            return
        
        # 提取路径点
        path_points = []
        for pose in path_msg.poses:
            path_points.append((pose.pose.position.x, pose.pose.position.y))
        
        # 计算路径上的通信质量
        quality_values = []
        for x, y in path_points:
            # 转换为地图坐标
            map_x = int((x - self.network_quality_map['origin_x']) / self.network_quality_map['resolution'])
            map_y = int((y - self.network_quality_map['origin_y']) / self.network_quality_map['resolution'])
            
            # 检查坐标是否在地图范围内
            if (0 <= map_x < self.network_quality_map['width'] and 
                0 <= map_y < self.network_quality_map['height']):
                
                quality = self.network_quality_map['data'][map_y, map_x]
                if quality >= 0.0:
                    quality_values.append(quality)
                else:
                    quality_values.append(0.5)  # 未知区域假设为中等质量
        
        # 计算平均通信质量
        if quality_values:
            avg_quality = np.mean(quality_values)
            min_quality = np.min(quality_values)
            
            rospy.loginfo(f"Path communication quality: avg={avg_quality:.2f}, min={min_quality:.2f}")

    def optimize_path_for_communication(self):
        """优化路径考虑通信质量"""
        if not self.planned_path or not self.network_quality_map:
            return
        
        # 简化方法：如果路径上有通信质量差的区域，生成警告
        # 在实际实现中，这可能会包含路径重规划
        
        # 提取路径点
        path_points = []
        for pose in self.planned_path.poses:
            path_points.append((pose.pose.position.x, pose.pose.position.y))
        
        # 检查路径上的通信质量
        poor_quality_segments = []
        current_segment = []
        
        for i, (x, y) in enumerate(path_points):
            # 转换为地图坐标
            map_x = int((x - self.network_quality_map['origin_x']) / self.network_quality_map['resolution'])
            map_y = int((y - self.network_quality_map['origin_y']) / self.network_quality_map['resolution'])
            
            # 检查坐标是否在地图范围内
            if (0 <= map_x < self.network_quality_map['width'] and 
                0 <= map_y < self.network_quality_map['height']):
                
                quality = self.network_quality_map['data'][map_y, map_x]
                if quality >= 0.0 and quality < 0.3:  # 通信质量差
                    current_segment.append(i)
                else:
                    if current_segment:
                        poor_quality_segments.append(current_segment)
                        current_segment = []
        
        if current_segment:
            poor_quality_segments.append(current_segment)
        
        # 发出通信质量差的路径段警告
        if poor_quality_segments:
            total_segments = len(poor_quality_segments)
            total_points = sum(len(segment) for segment in poor_quality_segments)
            warning = f"WARNING: {total_segments} path segments with poor communication quality, total {total_points} points"
            rospy.logwarn(warning)
            
            # 生成通信感知路径建议
            self.generate_communication_aware_path()
    
    def generate_communication_aware_path(self):
        """生成通信感知的路径"""
        if not self.planned_path:
            return
        
        # 在实际实现中，这里会包含更复杂的路径规划算法
        # 现在简单地发布现有路径作为示例
        self.communication_aware_path_pub.publish(self.planned_path)
        
        rospy.loginfo("Published communication-aware path")

    def recommend_sensor_mode(self, rnti, network_quality, bandwidth):
        """推荐传感器模式"""
        sensor_mode = ""
        
        # 根据网络质量和带宽确定传感器模式
        if network_quality == "EXCELLENT" or network_quality == "GOOD":
            if bandwidth > 10.0:  # 高带宽
                sensor_mode = "HIGH_RESOLUTION"
            else:
                sensor_mode = "STANDARD"
        elif network_quality == "FAIR":
            if bandwidth > 5.0:
                sensor_mode = "STANDARD"
            else:
                sensor_mode = "REDUCED"
        else:  # POOR or BAD
            if bandwidth > 2.0:
                sensor_mode = "REDUCED"
            else:
                sensor_mode = "MINIMAL"
        
        # 根据任务关键度调整
        if self.task_criticality > 0.8:
            # 高关键任务需要更高质量
            if sensor_mode == "REDUCED":
                sensor_mode = "STANDARD"
            elif sensor_mode == "MINIMAL":
                sensor_mode = "REDUCED"
        
        # 存储传感器推荐
        self.sensor_recommendations[rnti] = {
            'mode': sensor_mode,
            'timestamp': rospy.Time.now()
        }
        
        return sensor_mode

    def evaluate_task_criticality(self):
        """评估任务关键度"""
        # 根据任务状态设置基础关键度
        if self.task_status == "EMERGENCY":
            base_criticality = 1.0
        elif self.task_status == "CRITICAL":
            base_criticality = 0.9
        elif self.task_status == "NORMAL":
            base_criticality = 0.5
        elif self.task_status == "LOW_PRIORITY":
            base_criticality = 0.3
        else:
            base_criticality = 0.1
        
        # 考虑网络状况
        network_factor = 1.0
        if self.network_quality_history:
            recent_qualities = list(self.network_quality_history)[-5:]
            poor_count = sum(1 for q in recent_qualities if q in ["POOR", "BAD"])
            
            if poor_count >= 3:
                # 网络条件差会增加任务关键度
                network_factor = 1.2
        
        # 最终关键度
        self.task_criticality = min(1.0, base_criticality * network_factor)
        
        # 发布任务关键度
        self.task_criticality_pub.publish(self.task_criticality)

    def calculate_data_compression_level(self, rnti):
        """计算数据压缩级别"""
        if rnti not in self.channel_states or rnti not in self.phy_metrics:
            return 0.0
        
        # 获取通道状态
        state = self.channel_states[rnti]
        metrics = self.phy_metrics[rnti]
        
        # 基于带宽和链路质量计算压缩级别
        bandwidth = state.dl_thr  # Mbps
        link_quality = metrics.link_quality
        
        # 基础压缩级别 (0-1，0表示不压缩，1表示最大压缩)
        if bandwidth > 10.0 and link_quality > 0.8:
            # 高带宽高质量，最小压缩
            compression_level = 0.0
        elif bandwidth > 5.0 and link_quality > 0.6:
            # 中等带宽和质量，轻度压缩
            compression_level = 0.3
        elif bandwidth > 2.0 and link_quality > 0.4:
            # 低带宽和质量，中度压缩
            compression_level = 0.6
        else:
            # 非常低的带宽或质量，高度压缩
            compression_level = 0.9
        
        # 调整压缩级别基于任务关键度
        if self.task_criticality > 0.8:
            # 高关键度任务需要更低的压缩
            compression_level = max(0.0, compression_level - 0.2)
        
        return compression_level

    def generate_adaptation_recommendation(self):
        """生成通信感知适应建议"""
        # 如果没有通道数据，无法生成建议
        if not self.channel_states:
            return "MAINTAIN_CURRENT"
        
        # 获取最新的网络质量
        latest_quality = None
        if self.network_quality_history:
            latest_quality = list(self.network_quality_history)[-1]
        else:
            return "MAINTAIN_CURRENT"
        
        # 基于网络质量生成建议
        if latest_quality == "EXCELLENT":
            return "MAXIMIZE_DATA_COLLECTION"
        elif latest_quality == "GOOD":
            return "NORMAL_OPERATION"
        elif latest_quality == "FAIR":
            return "MODERATE_CONSERVATION"
        elif latest_quality == "POOR":
            return "SIGNIFICANT_REDUCTION"
        else:  # BAD
            return "EMERGENCY_CONSERVATION"

if __name__ == '__main__':
    try:
        engine = CrossDomainEngine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
import subprocess
import threading
import time
import numpy as np
from collections import deque
from wireless_ros.msgs.msg import ChannelState, TransmissionStrategy, Constraint, PhyLayerMetrics, SpectrumAnalysis
from std_msgs.msg import String, Float32, Int32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Hub:
    """
    WirelessROS Hub - 整个系统的协调中心
    
    依据Hub-Engines-Cells架构，Hub负责：
    1. 全局资源分配与策略协调
    2. 引擎和Cell的生命周期管理
    3. 系统健康监控与故障恢复
    4. 跨域优化和策略执行
    5. 资源冲突解决
    """
    
    def __init__(self):
        rospy.init_node('wireless_ros_hub')
        
        # 高级参数
        self.check_interval = rospy.get_param('~check_interval', 1.0)
        self.recovery_enabled = rospy.get_param('~recovery_enabled', True)
        self.priority_management_enabled = rospy.get_param('~priority_management_enabled', True)
        self.resource_conflict_resolution = rospy.get_param('~resource_conflict_resolution', True)
        self.global_optimization_interval = rospy.get_param('~global_optimization_interval', 10.0)  # 10秒
        
        # 引擎配置
        self.engine_nodes = {
            'radio_info_engine': {
                'script': 'radio_information_engine.py',
                'package': 'wireless_ros',
                'priority': 9,  # 高优先级
                'required': True  # 必需组件
            },
            'cross_domain_engine': {
                'script': 'cross_domain_engine.py',
                'package': 'wireless_ros',
                'priority': 8,
                'required': True
            },
            'physical_adaptive_engine': {
                'script': 'physical_adaptive_engine.py',
                'package': 'wireless_ros',
                'priority': 8,
                'required': True
            }
        }
        
        # Cell配置
        self.cell_nodes = {}  # 动态发现或从参数加载
        
        # 订阅所有相关话题
        self.channel_state_sub = rospy.Subscriber('/wireless_ros/channel_state', ChannelState, self.channel_state_callback)
        self.phy_metrics_sub = rospy.Subscriber('/wireless_ros/phy_layer_metrics', PhyLayerMetrics, self.phy_metrics_callback)
        self.spectrum_analysis_sub = rospy.Subscriber('/wireless_ros/spectrum_analysis', SpectrumAnalysis, self.spectrum_analysis_callback)
        self.constraint_sub = rospy.Subscriber('/wireless_ros/constraints', Constraint, self.constraint_callback)
        self.strategy_sub = rospy.Subscriber('/wireless_ros/transmission_strategy', TransmissionStrategy, self.strategy_callback)
        
        # 系统监控话题
        self.resource_allocation_sub = rospy.Subscriber('/wireless_ros/resource_allocation', String, self.resource_allocation_callback)
        self.learning_status_sub = rospy.Subscriber('/wireless_ros/learning_status', String, self.learning_status_callback)
        self.adaptation_recommendation_sub = rospy.Subscriber('/wireless_ros/adaptation_recommendation', String, self.adaptation_recommendation_callback)
        
        # 发布诊断信息
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        
        # 发布系统状态和控制信息
        self.system_status_pub = rospy.Publisher('/wireless_ros/system_status', String, queue_size=10)
        self.resource_overview_pub = rospy.Publisher('/wireless_ros/resource_overview', String, queue_size=10)
        self.conflict_resolution_pub = rospy.Publisher('/wireless_ros/conflict_resolution', String, queue_size=10)
        self.global_policy_pub = rospy.Publisher('/wireless_ros/global_policy', String, queue_size=10)
        
        # 状态跟踪
        self.channel_states = {}  # RNTI -> 最新通道状态
        self.phy_metrics = {}     # RNTI -> 最新PHY层指标
        self.spectrum_data = {}   # RNTI -> 最新频谱分析
        self.constraints = {}     # RNTI -> 最新约束
        self.strategies = {}      # RNTI -> 最新传输策略
        
        # 系统监控状态
        self.engine_status = {}   # 引擎状态
        self.cell_status = {}     # Cell状态
        self.resource_allocations = {}  # 资源分配信息
        self.learning_status = {}  # 学习状态
        self.adaptation_recommendations = []  # 适应建议
        
        # 引擎和Cell启动时间
        self.engine_start_times = {}
        self.cell_start_times = {}
        
        # 系统事件与错误日志
        self.system_events = deque(maxlen=1000)
        self.error_logs = deque(maxlen=1000)
        
        # 全局策略状态
        self.current_global_policy = "BALANCED_OPERATION"
        self.global_policy_timestamp = 0
        self.resource_constraints = {
            'max_total_prbs': 100,
            'max_total_bandwidth': 1000.0,  # Mbps
            'max_users': 20
        }
        
        # 性能指标
        self.performance_metrics = {
            'avg_bler': 0.0,
            'avg_throughput': 0.0,
            'avg_latency': 0.0,
            'resource_utilization': 0.0,
            'successful_transmissions': 0,
            'failed_transmissions': 0
        }
        
        # 启动监控线程
        self.stop_event = threading.Event()
        self.monitor_thread = threading.Thread(target=self.monitor_nodes)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # 启动全局优化线程
        self.optimization_thread = threading.Thread(target=self.run_global_optimization)
        self.optimization_thread.daemon = True
        self.optimization_thread.start()
        
        # 启动定时状态检查和发布
        self.timer = rospy.Timer(rospy.Duration(self.check_interval), self.publish_diagnostics)
        
        # 启动资源分配监控
        if self.resource_conflict_resolution:
            self.resource_monitor_thread = threading.Thread(target=self.monitor_resource_conflicts)
            self.resource_monitor_thread.daemon = True
            self.resource_monitor_thread.start()
        
        rospy.loginfo("WirelessROS Hub initialized")
        self.log_system_event("Hub initialized and ready for operation")
        
        # 注册关闭回调
        rospy.on_shutdown(self.shutdown_hook)

    def channel_state_callback(self, msg):
        """处理通道状态更新"""
        rnti = msg.rnti
        self.channel_states[rnti] = msg
        self.update_engine_status('radio_info_engine', 'active')

    def phy_metrics_callback(self, msg):
        """处理PHY层指标更新"""
        rnti = msg.rnti
        self.phy_metrics[rnti] = msg
        self.update_engine_status('radio_info_engine', 'active')

    def spectrum_analysis_callback(self, msg):
        """处理频谱分析更新"""
        rnti = msg.rnti
        self.spectrum_data[rnti] = msg
        self.update_engine_status('radio_info_engine', 'active')

    def constraint_callback(self, msg):
        """处理约束更新"""
        rnti = msg.rnti
        self.constraints[rnti] = msg
        self.update_engine_status('cross_domain_engine', 'active')

    def strategy_callback(self, msg):
        """处理传输策略更新"""
        rnti = msg.rnti
        self.strategies[rnti] = msg
        self.update_engine_status('physical_adaptive_engine', 'active')

    def resource_allocation_callback(self, msg):
        """处理资源分配信息"""
        allocation_info = msg.data
        # 提取RNTI（假设格式为"RNTI: xxx, ..."）
        parts = allocation_info.split(',')
        if parts and "RNTI:" in parts[0]:
            rnti_str = parts[0].split("RNTI:")[1].strip()
            try:
                rnti = int(rnti_str)
                self.resource_allocations[rnti] = {
                    'info': allocation_info,
                    'timestamp': rospy.Time.now()
                }
            except ValueError:
                rospy.logwarn(f"Invalid RNTI format in resource allocation: {rnti_str}")

    def learning_status_callback(self, msg):
        """处理学习状态信息"""
        status_info = msg.data
        # 提取RNTI（假设格式为"RL Model for RNTI xxx: ..."）
        if "RNTI" in status_info:
            try:
                rnti_str = status_info.split("RNTI")[1].split(":")[0].strip()
                rnti = int(rnti_str)
                self.learning_status[rnti] = {
                    'info': status_info,
                    'timestamp': rospy.Time.now()
                }
            except (ValueError, IndexError):
                rospy.logwarn(f"Invalid RNTI format in learning status: {status_info}")

    def adaptation_recommendation_callback(self, msg):
        """处理适应建议"""
        recommendation = msg.data
        self.adaptation_recommendations.append({
            'recommendation': recommendation,
            'timestamp': rospy.Time.now()
        })
        
        # 只保留最近10条建议
        if len(self.adaptation_recommendations) > 10:
            self.adaptation_recommendations = self.adaptation_recommendations[-10:]

    def update_engine_status(self, engine_name, status):
        """更新引擎状态"""
        self.engine_status[engine_name] = status
        self.engine_status[f"{engine_name}_timestamp"] = rospy.get_time()

    def monitor_nodes(self):
        """监控节点状态并在必要时重启"""
        while not self.stop_event.is_set():
            try:
                # 获取当前运行的节点列表
                nodes_list = subprocess.check_output(["rosnode", "list"]).decode('utf-8').split('\n')
                
                # 检查所有引擎节点
                for name, config in self.engine_nodes.items():
                    node_name = f"/wireless_ros/{name}"
                    
                    # 检查节点是否活跃
                    if name not in self.engine_status or rospy.get_time() - self.engine_status.get(f"{name}_timestamp", 0) > 5.0:
                        # 节点可能不活跃，检查是否运行
                        if node_name not in nodes_list:
                            if self.recovery_enabled:
                                if config['required']:
                                    self.log_system_event(f"Required engine {name} not running. Attempting to start...")
                                    self.start_engine(name, config)
                                else:
                                    self.log_system_event(f"Optional engine {name} not running")
                            self.engine_status[name] = 'starting' if self.recovery_enabled else 'inactive'
                        else:
                            # 节点存在但没有收到消息
                            self.engine_status[name] = 'inactive'
                            if config['required']:
                                self.log_system_event(f"Engine {name} is inactive, not sending messages")
                
                # 同样检查Cell节点
                for name, config in self.cell_nodes.items():
                    node_name = f"/wireless_ros/cells/{name}"
                    
                    if node_name not in nodes_list:
                        if self.recovery_enabled and config.get('required', False):
                            self.log_system_event(f"Required cell {name} not running. Attempting to start...")
                            self.start_cell(name, config)
                        self.cell_status[name] = 'starting' if self.recovery_enabled else 'inactive'
                
                # 短暂休眠避免过度使用CPU
                time.sleep(2.0)
            except Exception as e:
                error_msg = f"Error in monitor thread: {e}"
                rospy.logerr(error_msg)
                self.log_error(error_msg)
                time.sleep(2.0)

    def start_engine(self, name, config):
        """启动引擎节点"""
        try:
            script = config['script']
            package = config.get('package', 'wireless_ros')
            
            # 记录启动时间
            self.engine_start_times[name] = rospy.get_time()
            
            # 启动节点
            subprocess.Popen(["rosrun", package, script])
            
            self.log_system_event(f"Started engine {name}")
            return True
        except Exception as e:
            error_msg = f"Failed to start engine {name}: {e}"
            rospy.logerr(error_msg)
            self.log_error(error_msg)
            return False

    def start_cell(self, name, config):
        """启动Cell节点"""
        try:
            script = config['script']
            package = config.get('package', 'wireless_ros')
            
            # 记录启动时间
            self.cell_start_times[name] = rospy.get_time()
            
            # 启动节点
            subprocess.Popen(["rosrun", package, script])
            
            self.log_system_event(f"Started cell {name}")
            return True
        except Exception as e:
            error_msg = f"Failed to start cell {name}: {e}"
            rospy.logerr(error_msg)
            self.log_error(error_msg)
            return False

    def publish_diagnostics(self, event=None):
        """发布系统诊断信息"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        
        # 引擎诊断
        for name, config in self.engine_nodes.items():
            status = DiagnosticStatus()
            status.name = f"WirelessROS Engine: {name}"
            
            if name in self.engine_status and self.engine_status[name] == 'active':
                status.level = DiagnosticStatus.OK
                status.message = "Engine running normally"
            elif name in self.engine_status and self.engine_status[name] == 'starting':
                status.level = DiagnosticStatus.WARN
                status.message = "Engine starting"
            elif name in self.engine_status and self.engine_status[name] == 'inactive':
                if config['required']:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Required engine inactive"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Optional engine inactive"
            else:
                if config['required']:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Required engine not running"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Optional engine not running"
            
            # 添加详细信息
            if name == 'radio_info_engine':
                status.values.append(KeyValue(key="Active RNTIs", value=str(len(self.channel_states))))
                if self.phy_metrics:
                    avg_link_quality = np.mean([m.link_quality for m in self.phy_metrics.values()])
                    status.values.append(KeyValue(key="Avg Link Quality", value=f"{avg_link_quality:.3f}"))
            elif name == 'cross_domain_engine':
                status.values.append(KeyValue(key="Active Constraints", value=str(len(self.constraints))))
                status.values.append(KeyValue(key="Latest Adaptation", value=self.adaptation_recommendations[-1]['recommendation'] if self.adaptation_recommendations else "NONE"))
            elif name == 'physical_adaptive_engine':
                status.values.append(KeyValue(key="Active Strategies", value=str(len(self.strategies))))
                status.values.append(KeyValue(key="Resource Allocations", value=str(len(self.resource_allocations))))
            
            diag_array.status.append(status)
        
        # Cell诊断
        for name, config in self.cell_nodes.items():
            status = DiagnosticStatus()
            status.name = f"WirelessROS Cell: {name}"
            
            if name in self.cell_status and self.cell_status[name] == 'active':
                status.level = DiagnosticStatus.OK
                status.message = "Cell running normally"
            else:
                if config.get('required', False):
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Required cell not running"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Optional cell not running"
            
            diag_array.status.append(status)
        
        # 系统整体状态
        overall_status = DiagnosticStatus()
        overall_status.name = "WirelessROS Hub"
        
        # 判断系统整体状态
        required_engines_active = all(
            self.engine_status.get(name, '') == 'active' 
            for name, config in self.engine_nodes.items() 
            if config['required']
        )
        
        required_cells_active = all(
            self.cell_status.get(name, '') == 'active' 
            for name, config in self.cell_nodes.items() 
            if config.get('required', False)
        )
        
        if required_engines_active and required_cells_active:
            overall_status.level = DiagnosticStatus.OK
            overall_status.message = "All systems operational"
        elif not required_engines_active:
            overall_status.level = DiagnosticStatus.ERROR
            overall_status.message = "Critical engines inactive"
        elif not required_cells_active:
            overall_status.level = DiagnosticStatus.ERROR
            overall_status.message = "Critical cells inactive"
        else:
            overall_status.level = DiagnosticStatus.WARN
            overall_status.message = "Some components inactive"
        
        # 添加详细信息
        overall_status.values.append(KeyValue(key="Active Engines", 
                                       value=str(sum(1 for s in self.engine_status.values() 
                                                  if isinstance(s, str) and s == 'active'))))
        overall_status.values.append(KeyValue(key="Total RNTIs", value=str(len(self.channel_states))))
        overall_status.values.append(KeyValue(key="Global Policy", value=self.current_global_policy))
        overall_status.values.append(KeyValue(key="Resource Utilization", value=f"{self.performance_metrics['resource_utilization']:.2f}"))
        
        diag_array.status.append(overall_status)
        
        # 发布诊断信息
        self.diagnostics_pub.publish(diag_array)
        
        # 发布系统状态
        self.publish_system_status()
        
        # 发布资源概览
        self.publish_resource_overview()

    def publish_system_status(self):
        """发布系统状态信息"""
        # 构建状态消息
        status_msg = (
            f"System Status: {self.current_global_policy}\n"
            f"Active RNTIs: {len(self.channel_states)}\n"
            f"Active Engines: {sum(1 for s in self.engine_status.values() if isinstance(s, str) and s == 'active')}/{len(self.engine_nodes)}\n"
            f"Active Cells: {sum(1 for s in self.cell_status.values() if isinstance(s, str) and s == 'active')}/{len(self.cell_nodes)}\n"
            f"Avg BLER: {self.performance_metrics['avg_bler']:.3f}\n"
            f"Avg Throughput: {self.performance_metrics['avg_throughput']:.2f} Mbps\n"
            f"Resource Utilization: {self.performance_metrics['resource_utilization'] * 100:.1f}%"
        )
        
        self.system_status_pub.publish(status_msg)

    def publish_resource_overview(self):
        """发布资源使用概览"""
        # 计算PRB和带宽使用情况
        total_prbs_used = 0
        total_bandwidth_used = 0.0
        
        for rnti, strategy in self.strategies.items():
            total_prbs_used += strategy.target_prb
            
            # 估算带宽使用
            if rnti in self.channel_states:
                total_bandwidth_used += self.channel_states[rnti].dl_thr
        
        # 构建概览消息
        overview_msg = (
            f"Resource Overview:\n"
            f"PRBs Used: {total_prbs_used}/{self.resource_constraints['max_total_prbs']} ({total_prbs_used/self.resource_constraints['max_total_prbs']*100:.1f}%)\n"
            f"Bandwidth Used: {total_bandwidth_used:.2f}/{self.resource_constraints['max_total_bandwidth']} Mbps ({total_bandwidth_used/self.resource_constraints['max_total_bandwidth']*100:.1f}%)\n"
            f"Active Users: {len(self.channel_states)}/{self.resource_constraints['max_users']}"
        )
        
        self.resource_overview_pub.publish(overview_msg)
        
        # 更新性能指标
        self.performance_metrics['resource_utilization'] = total_prbs_used / self.resource_constraints['max_total_prbs']
        
        # 计算平均BLER和吞吐量
        if self.channel_states:
            self.performance_metrics['avg_bler'] = np.mean([state.dl_bler for state in self.channel_states.values()])
            self.performance_metrics['avg_throughput'] = np.mean([state.dl_thr for state in self.channel_states.values()])

    def monitor_resource_conflicts(self):
        """监控和解决资源冲突"""
        while not self.stop_event.is_set():
            try:
                # 检查PRB分配冲突
                total_prbs_allocated = sum(strategy.target_prb for strategy in self.strategies.values())
                
                if total_prbs_allocated > self.resource_constraints['max_total_prbs']:
                    # 检测到PRB分配冲突
                    self.log_system_event(f"PRB allocation conflict detected: {total_prbs_allocated}/{self.resource_constraints['max_total_prbs']}")
                    
                    # 解决冲突
                    self.resolve_prb_conflict()
                
                # 检查优先级冲突
                if self.priority_management_enabled:
                    self.manage_priorities()
                
                time.sleep(2.0)
            except Exception as e:
                error_msg = f"Error in resource monitor: {e}"
                rospy.logerr(error_msg)
                self.log_error(error_msg)
                time.sleep(2.0)

    def resolve_prb_conflict(self):
        """解决PRB分配冲突"""
        # 按优先级排序策略
        prioritized_strategies = sorted(
            [(rnti, strategy) for rnti, strategy in self.strategies.items()],
            key=lambda x: x[1].priority,
            reverse=True  # 高优先级优先
        )
        
        # 分配PRB，优先满足高优先级用户
        remaining_prbs = self.resource_constraints['max_total_prbs']
        prb_allocations = {}
        
        for rnti, strategy in prioritized_strategies:
            # 高优先级获取请求的PRB
            if strategy.priority >= 4:
                allocated_prbs = min(remaining_prbs, strategy.target_prb)
            # 中优先级获取请求的一部分
            elif strategy.priority >= 2:
                allocated_prbs = min(remaining_prbs, int(strategy.target_prb * 0.8))
            # 低优先级获取最少资源
            else:
                allocated_prbs = min(remaining_prbs, max(5, int(strategy.target_prb * 0.5)))
            
            prb_allocations[rnti] = allocated_prbs
            remaining_prbs -= allocated_prbs
            
            if remaining_prbs <= 0:
                break
        
        # 记录和报告冲突解决方案
        resolution_msg = f"PRB Conflict Resolution: Total requested={sum(s.target_prb for s in self.strategies.values())}, Allocated={self.resource_constraints['max_total_prbs']-remaining_prbs}"
        for rnti, prbs in prb_allocations.items():
            original = self.strategies[rnti].target_prb
            resolution_msg += f"\nRNTI {rnti}: {original}->{prbs} ({int(prbs/original*100)}%)"
        
        self.conflict_resolution_pub.publish(resolution_msg)
        self.log_system_event(f"PRB conflict resolved: {len(prb_allocations)} users adjusted")

    def manage_priorities(self):
        """管理用户优先级"""
        # 基于任务类型、数据负载和通信质量动态调整优先级
        
        # 在实际实现中，这里会执行更复杂的优先级管理策略
        # 作为示例，这里简单地平衡系统负载
        
        # 计算系统负载和资源使用情况
        total_prbs = sum(strategy.target_prb for strategy in self.strategies.values())
        system_load = total_prbs / self.resource_constraints['max_total_prbs']
        
        # 根据系统负载调整优先级
        if system_load > 0.9:  # 高负载
            # 降级非关键任务，确保关键任务资源
            for rnti, strategy in self.strategies.items():
                if strategy.priority < 4 and strategy.strategy_type != "CRITICAL_EMERGENCY":
                    # 降低低优先级用户的PRB分配
                    new_prb = int(strategy.target_prb * 0.8)
                    if new_prb != strategy.target_prb:
                        strategy.target_prb = new_prb
                        self.log_system_event(f"High load: Reduced PRB for RNTI {rnti} to {new_prb}")

    def run_global_optimization(self):
        """运行全局优化策略"""
        while not self.stop_event.is_set():
            try:
                # 每次运行时间
                next_run_time = rospy.get_time() + self.global_optimization_interval
                
                # 在有足够数据时运行优化
                if len(self.channel_states) > 0 and len(self.strategies) > 0:
                    self.optimize_global_resource_allocation()
                
                # 等待到下次运行时间
                sleep_time = max(0.1, next_run_time - rospy.get_time())
                time.sleep(sleep_time)
            except Exception as e:
                error_msg = f"Error in global optimization: {e}"
                rospy.logerr(error_msg)
                self.log_error(error_msg)
                time.sleep(5.0)  # 错误后等待更长时间

    def optimize_global_resource_allocation(self):
        """优化全局资源分配"""
        # 分析当前系统状态
        user_count = len(self.channel_states)
        total_prbs = sum(strategy.target_prb for strategy in self.strategies.values())
        avg_bler = np.mean([state.dl_bler for state in self.channel_states.values()]) if self.channel_states else 0
        
        # 分析链路质量分布
        if self.phy_metrics:
            link_qualities = [metrics.link_quality for metrics in self.phy_metrics.values()]
            avg_link_quality = np.mean(link_qualities)
            min_link_quality = np.min(link_qualities)
        else:
            avg_link_quality = 0.5
            min_link_quality = 0.5
        
        # 确定全局策略
        new_policy = self.determine_global_policy(user_count, total_prbs, avg_bler, avg_link_quality, min_link_quality)
        
        # 如果策略变化，通知系统
        if new_policy != self.current_global_policy:
            self.current_global_policy = new_policy
            self.global_policy_timestamp = rospy.get_time()
            
            policy_msg = f"Global Policy Updated: {new_policy}"
            self.global_policy_pub.publish(policy_msg)
            self.log_system_event(policy_msg)

    def determine_global_policy(self, user_count, total_prbs, avg_bler, avg_link_quality, min_link_quality):
        """确定全局策略"""
        # 根据系统状态参数确定最优全局策略
        
        # 高负载情况
        load_factor = total_prbs / self.resource_constraints['max_total_prbs']
        if load_factor > 0.9:
            if avg_bler > 0.1 or min_link_quality < 0.3:
                return "CONGESTION_CONTROL"
            else:
                return "HIGH_EFFICIENCY"
        
        # 中等负载情况
        elif load_factor > 0.6:
            if avg_link_quality > 0.7:
                return "BALANCED_OPERATION"
            else:
                return "RELIABILITY_FOCUS"
        
        # 低负载情况
        else:
            if user_count < 3:
                return "PERFORMANCE_FOCUS"
            else:
                return "BALANCED_OPERATION"

    def log_system_event(self, event):
        """记录系统事件"""
        timestamp = rospy.get_time()
        self.system_events.append({
            'timestamp': timestamp,
            'event': event
        })
        rospy.loginfo(f"System Event: {event}")

    def log_error(self, error):
        """记录错误"""
        timestamp = rospy.get_time()
        self.error_logs.append({
            'timestamp': timestamp,
            'error': error
        })

    def attach_engine(self, engine, mode='realtime'):
        """动态附加一个Engine"""
        engine_name = engine.__class__.__name__.lower()
        
        # 检查引擎是否已注册
        if engine_name not in self.engine_nodes:
            # 注册新引擎
            self.engine_nodes[engine_name] = {
                'script': f"{engine_name}.py",
                'package': 'wireless_ros',
                'priority': 5,
                'required': False,
                'mode': mode
            }
            
            self.log_system_event(f"Attached new engine: {engine_name} in {mode} mode")
        
        # 配置引擎模式
        if mode == 'realtime':
            # 实时处理所有数据
            pass
        elif mode == 'burst':
            # 批量处理数据
            pass
        elif mode == 'optimized':
            # 优化处理性能敏感数据
            pass
        
        self.update_engine_status(engine_name, 'active')
        
        # 返回配置后的引擎
        return engine

    def deploy_cell(self, cell_class, name, **kwargs):
        """部署一个Cell"""
        # 创建Cell实例
        cell = cell_class(name, **kwargs)
        
        # 注册Cell
        if name not in self.cell_nodes:
            self.cell_nodes[name] = {
                'script': f"{name}_cell.py",
                'package': 'wireless_ros',
                'required': kwargs.get('required', False),
                'parameters': kwargs
            }
            
            self.log_system_event(f"Deployed new cell: {name}")
        
        # 更新Cell状态
        self.cell_status[name] = 'active'
        
        return cell

    def shutdown_hook(self):
        """关闭时清理"""
        self.stop_event.set()
        
        # 等待线程结束
        if hasattr(self, 'monitor_thread') and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)
            
        if hasattr(self, 'optimization_thread') and self.optimization_thread.is_alive():
            self.optimization_thread.join(timeout=1.0)
            
        if hasattr(self, 'resource_monitor_thread') and self.resource_monitor_thread.is_alive():
            self.resource_monitor_thread.join(timeout=1.0)
        
        self.log_system_event("Hub shutting down")
        rospy.loginfo("WirelessROS Hub shutting down")

if __name__ == '__main__':
    try:
        hub = Hub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
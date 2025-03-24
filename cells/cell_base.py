#!/usr/bin/env python3
import rospy
import threading
import time
from wireless_ros.core import ChannelObserver

class Cell:
    """
    Cell基类 - 所有功能单元的基础
    
    Cell是WirelessROS架构中的基本执行单元，提供:
    1. 通信感知能力
    2. 自适应行为
    3. 标准化接口
    4. 生命周期管理
    """
    
    def __init__(self, name, cell_type="generic", required=False):
        """初始化Cell"""
        self.name = name
        self.cell_type = cell_type
        self.required = required
        self.active = False
        self.last_active_time = 0
        self.constraints = None
        self.channel_state = None
        self.observers = []
        
        # 参数和配置
        self.parameters = {}
        self.config = {}
        
        # 状态和指标
        self.status = "INITIALIZED"
        self.performance_metrics = {}
        self.execution_count = 0
        self.error_count = 0
        
        # 资源使用
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.bandwidth_usage = 0.0
        
        # 启动监控线程
        self.stop_monitor = threading.Event()
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        rospy.loginfo(f"Cell {name} ({cell_type}) initialized")

    def start(self):
        """启动Cell"""
        if not self.active:
            self.active = True
            self.status = "ACTIVE"
            self._on_start()
            rospy.loginfo(f"Cell {self.name} started")
        
        return self.active

    def stop(self):
        """停止Cell"""
        if self.active:
            self.active = False
            self.status = "STOPPED"
            self._on_stop()
            rospy.loginfo(f"Cell {self.name} stopped")

    def configure(self, config):
        """配置Cell"""
        self.config.update(config)
        self._on_configure(config)
        rospy.loginfo(f"Cell {self.name} configured")
        return True

    def set_parameter(self, key, value):
        """设置参数"""
        self.parameters[key] = value
        return True

    def get_parameter(self, key, default=None):
        """获取参数"""
        return self.parameters.get(key, default)

    def get_status(self):
        """获取状态"""
        return {
            "name": self.name,
            "type": self.cell_type,
            "status": self.status,
            "active": self.active,
            "execution_count": self.execution_count,
            "error_count": self.error_count,
            "performance": self.performance_metrics,
            "resources": {
                "cpu": self.cpu_usage,
                "memory": self.memory_usage,
                "bandwidth": self.bandwidth_usage
            }
        }

    def apply_constraints(self, constraints):
        """应用约束"""
        self.constraints = constraints
        return self._on_constraints_applied(constraints)

    def bind_parameter(self, param_name, value_provider):
        """动态绑定参数到提供者函数"""
        if not callable(value_provider):
            rospy.logwarn(f"Cell {self.name}: Attempted to bind non-callable to parameter {param_name}")
            return False
        
        self.parameters[param_name] = {'bound_provider': value_provider}
        rospy.loginfo(f"Cell {self.name}: Parameter {param_name} bound to dynamic provider")
        return True

    def register_observer(self, observer):
        """注册状态观察者"""
        if observer not in self.observers:
            self.observers.append(observer)
            return True
        return False

    def unregister_observer(self, observer):
        """注销状态观察者"""
        if observer in self.observers:
            self.observers.remove(observer)
            return True
        return False

    def notify_observers(self, event_type, data=None):
        """通知所有观察者"""
        for observer in self.observers:
            if hasattr(observer, 'on_cell_event'):
                observer.on_cell_event(self.name, event_type, data)

    def on_channel_update(self, channel_state):
        """接收通道状态更新"""
        self.channel_state = channel_state
        self._on_channel_state_changed(channel_state)

    def execute(self, *args, **kwargs):
        """执行Cell功能"""
        if not self.active:
            rospy.logwarn(f"Cell {self.name} not active, execution skipped")
            return None
        
        try:
            self.last_active_time = time.time()
            self.execution_count += 1
            
            # 获取所有动态绑定的参数的当前值
            for param_name, param_value in self.parameters.items():
                if isinstance(param_value, dict) and 'bound_provider' in param_value:
                    # 调用提供者函数获取当前值
                    try:
                        self.parameters[param_name + '_value'] = param_value['bound_provider']()
                    except Exception as e:
                        rospy.logwarn(f"Cell {self.name}: Error getting bound parameter {param_name}: {e}")
            
            # 调用子类实现的执行方法
            result = self._execute(*args, **kwargs)
            
            # 通知观察者执行完成
            self.notify_observers('executed', {'success': True, 'result': result})
            
            return result
            
        except Exception as e:
            self.error_count += 1
            error_msg = f"Cell {self.name} execution error: {e}"
            rospy.logerr(error_msg)
            
            # 通知观察者执行失败
            self.notify_observers('error', {'error': str(e)})
            
            return None

    def _monitor_loop(self):
        """监控资源使用和性能"""
        while not self.stop_monitor.is_set():
            if self.active:
                # 在实际实现中，这里会收集真实的资源使用情况
                # 现在使用模拟值
                self.cpu_usage = 10.0 + (self.execution_count % 10)  # 模拟10-20%的CPU使用
                self.memory_usage = 50.0 + (self.execution_count % 50)  # 模拟50-100MB内存使用
                
                # 更新性能指标
                self._update_performance_metrics()
                
                # 检查是否长时间无活动
                if self.last_active_time > 0 and time.time() - self.last_active_time > 60.0:
                    rospy.logwarn(f"Cell {self.name} inactive for >60s")
            
            time.sleep(5.0)  # 每5秒更新一次

    def _update_performance_metrics(self):
        """更新性能指标"""
        # 子类可重写以提供特定指标
        self.performance_metrics.update({
            'execution_rate': self.execution_count / max(1, (time.time() - self.last_active_time)),
            'error_rate': self.error_count / max(1, self.execution_count),
            'last_updated': time.time()
        })

    # 以下方法由子类重写
    def _on_start(self):
        """启动时调用"""
        pass

    def _on_stop(self):
        """停止时调用"""
        pass

    def _on_configure(self, config):
        """配置时调用"""
        pass

    def _on_constraints_applied(self, constraints):
        """应用约束时调用"""
        return True

    def _on_channel_state_changed(self, channel_state):
        """通道状态变化时调用"""
        pass

    def _execute(self, *args, **kwargs):
        """执行Cell功能，子类必须实现"""
        raise NotImplementedError(f"Cell {self.name} (_execute not implemented)")

    def __del__(self):
        """析构函数"""
        self.stop_monitor.set()
        if hasattr(self, 'monitor_thread') and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)
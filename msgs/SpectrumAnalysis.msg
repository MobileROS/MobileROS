# 频谱分析消息
Header header
uint32 rnti                # 用户RNTI

float32 utilization        # 频谱占用率 (0-1)
float32 interference_level # 干扰水平
float32 estimated_snr      # 估计的信噪比
float32 spectral_efficiency # 频谱效率

bool anomaly_detected      # 是否检测到异常
string anomaly_type        # 异常类型
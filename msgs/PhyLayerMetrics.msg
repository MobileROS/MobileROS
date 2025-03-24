# 高级物理层指标消息
Header header
uint32 rnti                  # 用户RNTI

float32 link_quality         # 链路质量 (0-1)
float32 channel_stability    # 信道稳定性 (0-1)
float32 reliability          # 通信可靠性 (0-1)
float32 spectral_efficiency  # 频谱效率 (bits/s/Hz)
float32 channel_capacity     # 信道容量 (bits/s/Hz)
float32 channel_response_time # 信道响应时间 (ms)
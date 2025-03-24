import json
import time
import random
from datetime import datetime

class RMMPolicyController:
    def __init__(self, update_interval: int = 300, max_slices: int = 5, last_n_ues: int = 2):
        self.rmmpolicy_path = "rrmPolicy_sub.json"
        self.update_interval = update_interval
        self.max_slices = max_slices
        self.last_n_ues = last_n_ues

    def load_json(self, file_path):
        """加载JSON文件"""
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"读取文件错误 {file_path}: {str(e)}")
            return None

    def save_json(self, data, file_path):
        """保存JSON文件"""
        try:
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=4)
            print(f"成功保存文件: {file_path}")
        except Exception as e:
            print(f"保存文件错误 {file_path}: {str(e)}")

    def generate_new_slice(self):
        """生成新的切片配置"""
        return {
            "sST": 1,
            "sD_flag": 0,
            "sD": 1,
            "min_ratio": random.randint(0, 40),
            "max_ratio": random.randint(60, 100)
        }

    def update_rmmpolicy(self):
        """更新RMM策略文件"""
        data = self.load_json(self.rmmpolicy_path)
        if not data:
            return

        # 获取可用的子切片ID列表
        available_sub_slices = [slice["sub_slice_id"] for slice in data["subSlicePolicy"]["subSlices"]]
        if not available_sub_slices:
            return

        # 获取上行切片的数量
        up_slices_count = len(data.get("up_rrmPolicyRatio", []))
        if up_slices_count == 0:
            return

        # 获取当前Slice_Config的值
        slice_config = str(data.get("Slice_Config", "1234"))
        if len(slice_config) >= 4:
            current_target_sub_slice = int(slice_config[-2])
            current_up_target_sub_slice = int(slice_config[-1])
        else:
            current_target_sub_slice = 1
            current_up_target_sub_slice = 1

        # 计算下一个target_sub_slice（在available_sub_slices中循环）
        current_index = available_sub_slices.index(current_target_sub_slice) if current_target_sub_slice in available_sub_slices else -1
        next_index = (current_index + 1) % len(available_sub_slices)
        next_target_sub_slice = available_sub_slices[next_index]

        # 计算下一个up_target_sub_slice（在1到up_slices_count之间循环）
        next_up_target_sub_slice = (current_up_target_sub_slice % up_slices_count) + 1

        # 更新最后N个UE的映射
        ue_keys = list(data["ueSliceMapping"].keys())
        if len(ue_keys) >= self.last_n_ues:
            for ue_key in ue_keys[-self.last_n_ues:]:
                mapping = data["ueSliceMapping"][ue_key]
                mapping["target_sub_slice"] = next_target_sub_slice
                mapping["up_target_sub_slice"] = next_up_target_sub_slice

        # 更新Slice_Config
        slice_config_list = list(slice_config)
        if len(slice_config_list) >= 2:
            slice_config_list[-2] = str(next_target_sub_slice)
            slice_config_list[-1] = str(next_up_target_sub_slice)
            data["Slice_Config"] = int("".join(slice_config_list))

        self.save_json(data, self.rmmpolicy_path)

    def run(self):
        """运行配置控制器"""
        print(f"RMMPolicy控制器启动...")
        print(f"更新间隔: {self.update_interval}秒")
        
        while True:
            try:
                print(f"\n开始更新RMMPolicy - {datetime.now()}")
                self.update_rmmpolicy()
                print(f"等待 {self.update_interval} 秒进行下一次更新...")
                time.sleep(self.update_interval)
            except KeyboardInterrupt:
                print("\nRMMPolicy控制器终止")
                break
            except Exception as e:
                print(f"发生错误: {str(e)}")
                time.sleep(60)

if __name__ == "__main__":
    controller = RMMPolicyController(
        update_interval=3,
        max_slices=10,
        last_n_ues=1
    )
    controller.run()
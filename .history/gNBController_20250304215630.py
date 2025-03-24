import json
import time
import random
from datetime import datetime

class RMMPolicyController:
    def __init__(self, update_interval: int = 300, max_slices: int = 5, last_n_ues: int = 2):
        self.rmmpolicy_path = "/home/lyg/oais_slice_final/rmmpolicy_sub.json"
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

        current_slices = len(data["rrmPolicyRatio"])
        if current_slices < self.max_slices:
            new_slice = self.generate_new_slice()
            data["rrmPolicyRatio"].append(new_slice)
            data["up_rrmPolicyRatio"].append(new_slice.copy())

        for slice_config in data["rrmPolicyRatio"][3:]:
            slice_config["min_ratio"] = random.randint(0, 40)
            slice_config["max_ratio"] = random.randint(60, 100)

        ue_keys = list(data["ueSliceMapping"].keys())
        if len(ue_keys) >= self.last_n_ues:
            for ue_key in ue_keys[-self.last_n_ues:]:
                data["ueSliceMapping"][ue_key].update({
                    "target_sub_slice": random.randint(1, len(data["subSlicePolicy"]["subSlices"])),
                    "up_target_sub_slice": random.randint(1, len(data["subSlicePolicy"]["subSlices"]))
                })

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
        update_interval=30,
        max_slices=10,
        last_n_ues=1
    )
    controller.run()

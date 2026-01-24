import json

class SequenceManager:
    def __init__(self):
        self.steps = []         # Danh sách các dict: {pos, rpy, dur, grip}
        self.is_running = False
        self.current_idx = 0
        self.timer_count = 0.0

    def add_step(self, pos, rpy, dur, grip):
        # Lưu bản sao (list) để tránh tham chiếu biến động
        step = {
            "pos": list(pos),
            "rpy": list(rpy),
            "dur": float(dur),
            "grip": float(grip)
        }
        self.steps.append(step)
        return step

    def save_to_json(self, file_path):
        try:
            with open(file_path, 'w') as f:
                json.dump(self.steps, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving JSON: {e}")
            return False

    def load_from_json(self, file_path):
        try:
            with open(file_path, 'r') as f:
                self.steps = json.load(f)
            return True
        except Exception as e:
            print(f"Error loading JSON: {e}")
            return False

    def clear(self):
        self.steps = []
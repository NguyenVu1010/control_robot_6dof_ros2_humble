import json

class SequenceManager:
    def __init__(self):
        self.steps = []
        self.is_running = False
        self.current_idx = 0
        self.timer_count = 0

    def add_step(self, pos, rpy, dur, grip):
        step = {
            "pos": list(pos),
            "rpy": list(rpy),
            "dur": dur,
            "grip": grip
        }
        self.steps.append(step)

    def save_to_file(self, filename):
        with open(filename, 'w') as f:
            json.dump(self.steps, f)

    def load_from_file(self, filename):
        with open(filename, 'r') as f:
            self.steps = json.load(f)
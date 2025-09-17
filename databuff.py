class DataBuffer:
    def __init__(self, max_size=1000):
        self.max_size = max_size
        self.data = []

    def add(self, value):
        self.data.append(value)
        if len(self.data) > self.max_size:
            self.data.pop(0)
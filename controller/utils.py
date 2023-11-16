import numpy as np


class CircularBuffer:
    """
        Auxilary class of circular buffer. Used for calculations of average power and average consumed energy.
    """
    def __init__(self,
                 size):
        self.size = size
        self.data = np.zeros((size,),)
        self.index = 0
        self.current_mean_value = 0.
        self.current_sum_value = 0.

    def add(self, element):
        self.data[self.index] = element
        self.index = (self.index + 1) % self.size

        self.current_mean_value = np.mean(self.data)
        self.current_sum_value = np.sum(self.data)

    def current_sum(self):
        return self.current_sum_value

    def current_mean(self):
        return self.current_mean_value

    def sum_except_oldest(self):
        return self.current_sum_value - self.data[self.index]

    def put_data(self, new_data):
        self.data = new_data
        self.index = 0
        self.current_mean_value = np.mean(self.data)
        self.current_sum_value = np.sum(self.data)

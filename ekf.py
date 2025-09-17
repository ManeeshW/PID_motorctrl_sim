import numpy as np

class KalmanFilter:
    def __init__(self, a=0.1, g=10.0, dt=0.01, Q=0.01, R=4.0, x0=0.0, P0=100.0):
        self.a = a
        self.g = g
        self.dt = dt
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0
        self.F = np.exp(-self.a * self.dt)
        self.G = self.g * (1 - self.F) / self.a if self.a != 0 else self.g * self.dt

    def predict(self, u):
        self.x = self.F * self.x + self.G * u
        self.P = self.F * self.P * self.F + self.Q

    def update(self, z):
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P = (1 - K) * self.P
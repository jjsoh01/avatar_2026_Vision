import math
import time
from typing import Optional

class LowPassFilter:
    def __init__(self, cutoff_hz, init_y: Optional[float] = None):
        self.cutoff_hz = float(cutoff_hz)
        self.y: Optional[float] = init_y

    def reset(self, value: float = 0.0) -> None:
        self.y = value
    
    def step(self, x: float, dt: float) -> float:
        if self.cutoff_hz <= 0.0:
            self.y = x
            return x

        if dt <= 0.0:
            self.y = x
            return x
        
        if self.y is None:
            self.y = x
            return x
        
        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (tau + dt) if dt > 0.0 else 1.0
        self.y = self.y + alpha * (x - self.y)
        return self.y


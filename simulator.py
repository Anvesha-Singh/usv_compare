# simulator_corrected.py
# FULLY CORRECTED - Simplified and tested USV model

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class SimulationConfig:
    """Configuration for simulation parameters"""
    dt: float = 0.1
    max_speed: float = 1.5
    max_yaw_rate: float = math.radians(30)
    max_accel: float = 0.8
    goal_threshold: float = 0.6
    collision_buffer: float = 0.15


class Obstacle:
    """Stationary circular obstacle"""
    def __init__(self, x: float, y: float, r: float, name: str = ""):
        self.x = x
        self.y = y
        self.r = r
        self.name = name

    def distance_to_point(self, px: float, py: float) -> float:
        """Returns signed distance to point (negative if inside)"""
        return math.hypot(px - self.x, py - self.y) - self.r

    def __repr__(self):
        return f"Obstacle({self.x:.2f}, {self.y:.2f}, r={self.r:.2f})"


class DynamicObstacle(Obstacle):
    """Moving circular obstacle"""
    def __init__(self, x: float, y: float, r: float, vx: float = 0.0, vy: float = 0.0, name: str = ""):
        super().__init__(x, y, r, name)
        self.vx = vx
        self.vy = vy

    def step(self, dt: float):
        """Update obstacle position"""
        self.x += self.vx * dt
        self.y += self.vy * dt

    def get_speed(self) -> float:
        """Get the obstacle's current speed"""
        return math.hypot(self.vx, self.vy)

    def get_heading(self) -> float:
        """Get the obstacle's current heading in radians"""
        speed = self.get_speed()
        if speed < 1e-4:  # Avoid atan2(0, 0)
            return 0.0
        return math.atan2(self.vy, self.vx)

    def __repr__(self):
        return f"DynamicObstacle({self.x:.2f}, {self.y:.2f}, r={self.r:.2f}, v=({self.vx:.2f},{self.vy:.2f}))"


class USV:
    """USV with SIMPLE, TESTED kinematic model"""
    
    def __init__(self, x: float, y: float, theta: float = 0.0, config: Optional[SimulationConfig] = None):
        self.config = config or SimulationConfig()
        
        # State
        self.x = x
        self.y = y
        self.theta = self._normalize_angle(theta)
        self.v = 0.0  # Current speed
        self.omega = 0.0  # Current yaw rate
        
        # Commands
        self.v_cmd = 0.0  # Speed command
        self.omega_cmd = 0.0  # Yaw rate command
        
        # History
        self.position_history: List[Tuple[float, float]] = [(x, y)]
        self.heading_history: List[float] = [self.theta]
        self.velocity_history: List[float] = [0.0]

    def set_control(self, throttle: float, yaw_rate: float):
        """Set control - throttle in [-1,1], yaw_rate in rad/s"""
        # Throttle to speed command
        self.v_cmd = np.clip(throttle, -1.0, 1.0) * self.config.max_speed
        
        # Yaw rate command
        self.omega_cmd = np.clip(yaw_rate, -self.config.max_yaw_rate, self.config.max_yaw_rate)

    def step(self, dt: Optional[float] = None):
        """Simple kinematic update"""
        dt = dt or self.config.dt
        
        # Speed acceleration
        max_dv = self.config.max_accel * dt
        dv = np.clip(self.v_cmd - self.v, -max_dv, max_dv)
        self.v += dv
        
        # Yaw rate
        self.omega = self.omega_cmd
        
        # Update heading
        self.theta += self.omega * dt
        self.theta = self._normalize_angle(self.theta)
        
        # Update position - THIS IS THE KEY PART
        dx = self.v * math.cos(self.theta) * dt
        dy = self.v * math.sin(self.theta) * dt
        self.x += dx
        self.y += dy
        
        # Store history
        self.position_history.append((self.x, self.y))
        self.heading_history.append(self.theta)
        self.velocity_history.append(self.v)

    def get_position(self) -> Tuple[float, float]:
        """Current position"""
        return self.x, self.y

    def get_heading(self) -> float:
        """Current heading in radians"""
        return self.theta

    def get_velocity(self) -> float:
        """Current speed"""
        return self.v

    def distance_to(self, x: float, y: float) -> float:
        """Euclidean distance"""
        return math.hypot(self.x - x, self.y - y)

    def check_collision(self, obstacles: List[Obstacle]) -> Tuple[bool, Optional[Obstacle]]:
        """Check collision"""
        for obs in obstacles:
            if obs.distance_to_point(self.x, self.y) <= self.config.collision_buffer:
                return True, obs
        return False, None

    def get_trajectory(self) -> List[Tuple[float, float]]:
        """Position history"""
        return self.position_history.copy()

    def reset(self, x: float, y: float, theta: float = 0.0):
        """Reset to initial state"""
        self.x = x
        self.y = y
        self.theta = self._normalize_angle(theta)
        self.v = 0.0
        self.omega = 0.0
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.position_history = [(x, y)]
        self.heading_history = [self.theta]
        self.velocity_history = [0.0]

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Shortest angle difference"""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


# Metric functions
def calculate_path_length(trajectory: List[Tuple[float, float]]) -> float:
    """Total path length"""
    if len(trajectory) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(trajectory)):
        total += math.hypot(trajectory[i][0] - trajectory[i-1][0],
                           trajectory[i][1] - trajectory[i-1][1])
    return total


def calculate_path_smoothness(trajectory: List[Tuple[float, float]]) -> float:
    """Path smoothness (sum of heading changes)"""
    if len(trajectory) < 3:
        return 0.0
    smoothness = 0.0
    for i in range(1, len(trajectory) - 1):
        v1 = (trajectory[i][0] - trajectory[i-1][0], trajectory[i][1] - trajectory[i-1][1])
        v2 = (trajectory[i+1][0] - trajectory[i][0], trajectory[i+1][1] - trajectory[i][1])
        
        mag1 = math.hypot(v1[0], v1[1]) + 1e-6
        mag2 = math.hypot(v2[0], v2[1]) + 1e-6
        
        dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (mag1 * mag2)
        dot = np.clip(dot, -1.0, 1.0)
        smoothness += abs(math.acos(dot))
    
    return smoothness


def calculate_clearance(trajectory: List[Tuple[float, float]], 
                       obstacles: List[Obstacle]) -> float:
    """Minimum clearance"""
    if not obstacles:
        return float('inf')
    min_clearance = float('inf')
    for px, py in trajectory:
        for obs in obstacles:
            clearance = obs.distance_to_point(px, py)
            min_clearance = min(min_clearance, clearance)
    return min_clearance if min_clearance != float('inf') else 0.0


def calculate_heading_stability(heading_history: List[float]) -> float:
    """Heading stability"""
    if len(heading_history) < 2:
        return 0.0
    changes = []
    for i in range(1, len(heading_history)):
        diff = abs(USV._angle_diff(heading_history[i], heading_history[i-1]))
        changes.append(diff)
    return np.mean(changes) if changes else 0.0

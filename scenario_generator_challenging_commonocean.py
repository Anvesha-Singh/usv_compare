import random
import math
import numpy as np
from simulator import Obstacle, DynamicObstacle

class ChallengeScenarioBuilder:
    """Generate scenarios with varying difficulty to find algorithm limits"""
    
    @staticmethod
    def create_scenario(name: str, difficulty: str = 'medium') -> tuple:
        """
        Create scenarios with difficulty levels
        
        Args:
            name: 'open', 'channel', 'clutter', 'dynamic', 'maze', 'crowded'
            difficulty: 'easy', 'medium', 'hard', 'extreme'
        """
        if name == 'open':
            return ChallengeScenarioBuilder._open_scenario(difficulty)
        elif name == 'channel':
            return ChallengeScenarioBuilder._channel_scenario(difficulty)
        elif name == 'clutter':
            return ChallengeScenarioBuilder._clutter_scenario(difficulty)
        elif name == 'dynamic':
            return ChallengeScenarioBuilder._dynamic_scenario(difficulty)
        elif name == 'maze':
            return ChallengeScenarioBuilder._maze_scenario(difficulty)
        elif name == 'crowded':
            return ChallengeScenarioBuilder._crowded_scenario(difficulty)
        elif name == 'colregs_challenge':
            return ChallengeScenarioBuilder._colregs_challenge_scenario(difficulty)
        else:
            raise ValueError(f"Unknown scenario: {name}")
    
    @staticmethod
    def _open_scenario(difficulty: str) -> tuple:
        """Open space with increasing complexity"""
        start = (0.0, 0.0)
        goal = (12.0, 0.0)
        
        if difficulty == 'easy':
            static_obs = [
                Obstacle(4.0, 1.5, 0.7),
                Obstacle(7.0, -1.2, 0.9)
            ]
        elif difficulty == 'medium':
            static_obs = [
                Obstacle(2.0, 1.0, 0.6),
                Obstacle(4.5, -1.5, 0.7),
                Obstacle(7.0, 1.2, 0.8),
                Obstacle(9.5, -1.0, 0.6)
            ]
        elif difficulty == 'hard':
            static_obs = [
                Obstacle(1.5, 1.0, 0.7),
                Obstacle(3.0, -1.5, 0.8),
                Obstacle(5.0, 0.8, 0.7),
                Obstacle(7.0, -1.0, 0.9),
                Obstacle(9.0, 1.5, 0.6),
                Obstacle(11.0, -0.8, 0.7)
            ]
        else:  # extreme
            static_obs = [
                Obstacle(1.5, 1.0, 0.8),
                Obstacle(2.0, -1.5, 0.8),
                Obstacle(3.5, 0.5, 0.7),
                Obstacle(4.5, -1.0, 0.8),
                Obstacle(6.0, 1.0, 0.9),
                Obstacle(7.0, -1.5, 0.8),
                Obstacle(8.5, 0.8, 0.7),
                Obstacle(10.0, -1.2, 0.8),
                Obstacle(11.0, 0.5, 0.7)
            ]
        
        return start, goal, static_obs, []
    
    @staticmethod
    def _channel_scenario(difficulty: str) -> tuple:
        """Narrow channel with increasing width constraints"""
        start = (0.0, 0.0)
        goal = (12.0, 0.0)
        static_obs = []
        
        if difficulty == 'easy':
            for x in np.linspace(1, 11, 20):
                static_obs.append(Obstacle(x, 2.5, 0.25))
                static_obs.append(Obstacle(x, -2.5, 0.25))
                
        elif difficulty == 'medium':
            for x in np.linspace(1, 11, 20):
                static_obs.append(Obstacle(x, 1.75, 0.25))
                static_obs.append(Obstacle(x, -1.75, 0.25))
                
        elif difficulty == 'hard':
            for x in np.linspace(1, 11, 20):
                static_obs.append(Obstacle(x, 1.0, 0.25))
                static_obs.append(Obstacle(x, -1.0, 0.25))
                
        else:  # extreme
            for x in np.linspace(1, 11, 20):
                static_obs.append(Obstacle(x, 0.7, 0.25))
                static_obs.append(Obstacle(x, -0.7, 0.25))
            
            for x in np.linspace(2, 10, 9):
                static_obs.append(Obstacle(x, 0.0, 0.3))
        
        return start, goal, static_obs, []
    
    @staticmethod
    def _clutter_scenario(difficulty: str) -> tuple:
        """Dense obstacle field"""
        start = (0.0, 0.0)
        goal = (12.0, 0.0)
        static_obs = []
        
        random.seed(42)
        
        if difficulty == 'easy':
            num_obs = 12
            radius_range = (0.2, 0.4)
            x_range = (2, 10)
            y_range = (-2.5, 2.5)
            
        elif difficulty == 'medium':
            num_obs = 25
            radius_range = (0.3, 0.6)
            x_range = (1.5, 10.5)
            y_range = (-3, 3)
            
        elif difficulty == 'hard':
            num_obs = 35
            radius_range = (0.4, 0.7)
            x_range = (1, 11)
            y_range = (-3, 3)
            
        else:  # extreme
            num_obs = 50
            radius_range = (0.3, 0.8)
            x_range = (0, 12)
            y_range = (-3, 3)
        
        for _ in range(num_obs):
            x = random.uniform(*x_range)
            y = random.uniform(*y_range)
            r = random.uniform(*radius_range)
            static_obs.append(Obstacle(x, y, r))
        
        return start, goal, static_obs, []
    
    @staticmethod
    def _dynamic_scenario(difficulty: str) -> tuple:
        """Moving obstacles"""
        start = (0.0, 0.0)
        goal = (12.0, 0.0)
        static_obs = []
        dynamic_obs = []
        
        if difficulty == 'easy':
            static_obs = [Obstacle(6.0, 1.5, 0.6)]
            dynamic_obs = [
                DynamicObstacle(5.0, -1.5, 0.4, vx=0.12, vy=0.05)
            ]
            
        elif difficulty == 'medium':
            static_obs = [
                Obstacle(3.0, 1.5, 0.5),
                Obstacle(9.0, -1.5, 0.5)
            ]
            dynamic_obs = [
                DynamicObstacle(2.0, -1.0, 0.4, vx=0.15, vy=0.0),
                DynamicObstacle(6.0, 1.5, 0.4, vx=-0.12, vy=0.08)
            ]
            
        elif difficulty == 'hard':
            static_obs = [
                Obstacle(3.0, 1.5, 0.5),
                Obstacle(9.0, -1.5, 0.5),
                Obstacle(6.0, 0.0, 0.4)
            ]
            dynamic_obs = [
                DynamicObstacle(2.0, -1.0, 0.4, vx=0.20, vy=0.0),
                DynamicObstacle(6.0, 1.5, 0.4, vx=-0.15, vy=0.10),
                DynamicObstacle(10.0, 0.5, 0.3, vx=-0.18, vy=0.0)
            ]
            
        else:  # extreme
            static_obs = [
                Obstacle(2.0, 1.5, 0.6),
                Obstacle(4.0, -1.5, 0.5),
                Obstacle(8.0, 1.0, 0.5),
                Obstacle(10.0, -1.5, 0.6)
            ]
            dynamic_obs = [
                DynamicObstacle(1.0, -0.5, 0.4, vx=0.25, vy=0.05),
                DynamicObstacle(5.0, 1.5, 0.4, vx=-0.20, vy=-0.10),
                DynamicObstacle(7.0, -1.0, 0.4, vx=0.15, vy=0.15),
                DynamicObstacle(11.0, 0.5, 0.3, vx=-0.25, vy=0.0)
            ]
        
        return start, goal, static_obs, dynamic_obs
    
    @staticmethod
    def _maze_scenario(difficulty: str) -> tuple:
        """Maze-like scenario with forced path"""
        start = (0.0, 0.0)
        goal = (12.0, 0.0)
        static_obs = []
        
        if difficulty == 'easy':
            # Simple S-curve
            for x in np.linspace(2, 4, 5):
                static_obs.append(Obstacle(x, 1.5, 0.4))
            for x in np.linspace(5, 7, 5):
                static_obs.append(Obstacle(x, -1.5, 0.4))
            for x in np.linspace(8, 10, 5):
                static_obs.append(Obstacle(x, 1.5, 0.4))
                
        elif difficulty == 'medium':
            # Complex maze
            for x in np.linspace(1, 4, 6):
                static_obs.append(Obstacle(x, -2.0, 0.3))
            for x in np.linspace(5, 7, 5):
                static_obs.append(Obstacle(x, 2.0, 0.3))
            for x in np.linspace(8, 10, 5):
                static_obs.append(Obstacle(x, -1.5, 0.3))
            for y in np.linspace(-1.5, 1.5, 5):
                static_obs.append(Obstacle(4.5, y, 0.3))
                
        elif difficulty == 'hard':
            # Very complex maze requiring backtracking
            for x in np.linspace(1, 3, 4):
                static_obs.append(Obstacle(x, 1.5, 0.35))
            for x in np.linspace(2, 4, 4):
                static_obs.append(Obstacle(x, -1.5, 0.35))
            for x in np.linspace(5, 7, 4):
                static_obs.append(Obstacle(x, 1.0, 0.35))
            for x in np.linspace(6, 8, 4):
                static_obs.append(Obstacle(x, -1.5, 0.35))
            for x in np.linspace(9, 11, 4):
                static_obs.append(Obstacle(x, 0.5, 0.35))
            for y in np.linspace(-1.5, 1.5, 5):
                static_obs.append(Obstacle(4.0, y, 0.25))
                static_obs.append(Obstacle(8.5, y, 0.25))
                
        else:  # extreme
            # Maze that might be unsolvable
            for x in np.linspace(1, 2, 3):
                static_obs.append(Obstacle(x, 1.5, 0.4))
            for x in np.linspace(1.5, 3, 3):
                static_obs.append(Obstacle(x, -1.5, 0.4))
            for x in np.linspace(4, 5.5, 3):
                static_obs.append(Obstacle(x, 1.0, 0.4))
            for x in np.linspace(4.5, 6, 3):
                static_obs.append(Obstacle(x, -1.0, 0.4))
            for x in np.linspace(7, 8.5, 3):
                static_obs.append(Obstacle(x, 1.5, 0.4))
            for x in np.linspace(7.5, 9, 3):
                static_obs.append(Obstacle(x, -1.5, 0.4))
            for y in np.linspace(-2, 2, 7):
                static_obs.append(Obstacle(3.5, y, 0.3))
                static_obs.append(Obstacle(6.5, y, 0.3))
                static_obs.append(Obstacle(9.5, y, 0.3))
        
        return start, goal, static_obs, []
    
    @staticmethod
    def _crowded_scenario(difficulty: str) -> tuple:
        """Many obstacles simulating crowded maritime traffic"""
        start = (0.0, 0.0)
        goal = (12.0, 0.0)
        static_obs = []
        
        random.seed(42)
        
        if difficulty == 'easy':
            num_obs = 15
            size_range = (0.3, 0.5)
        elif difficulty == 'medium':
            num_obs = 30
            size_range = (0.35, 0.6)
        elif difficulty == 'hard':
            num_obs = 50
            size_range = (0.4, 0.7)
        else:  # extreme
            num_obs = 80
            size_range = (0.35, 0.8)
        
        for _ in range(num_obs):
            x = random.uniform(0.5, 11.5)
            if random.random() < 0.7:
                y = random.gauss(0, 1.0)  # Gaussian around centerline
            else:
                y = random.uniform(-3, 3)
            r = random.uniform(*size_range)
            static_obs.append(Obstacle(x, y, r))
        
        return start, goal, static_obs, []

    @staticmethod
    def _colregs_challenge_scenario(difficulty: str) -> tuple:
        start = (0.0, 0.0)
        goal = (20.0, 0.0)
        
        static_obs = []
        dynamic_obs = []
        
        # Narrow passage
        static_obs += [Obstacle(x, 1.5, 0.5) for x in np.linspace(5, 15, 5)]
        static_obs += [Obstacle(x, -1.5, 0.5) for x in np.linspace(5, 15, 5)]
        
        if difficulty == 'medium':
            dynamic_obs.append(DynamicObstacle(10, 5, 0.5, vx=0, vy=-0.15))  # Crossing starboard
            dynamic_obs.append(DynamicObstacle(5, -4, 0.4, vx=0.25, vy=0))   # Overtaking from behind
            
        elif difficulty == 'hard':
            dynamic_obs.append(DynamicObstacle(12, 6, 0.5, vx=0, vy=-0.2))
            dynamic_obs.append(DynamicObstacle(7, -5, 0.5, vx=0.3, vy=0))
            dynamic_obs.append(DynamicObstacle(9, 0, 0.6, vx=-0.2, vy=0.1))
            dynamic_obs.append(DynamicObstacle(15, -3, 0.5, vx=-0.25, vy=0))
            dynamic_obs.append(DynamicObstacle(3, 4, 0.4, vx=0.15, vy=-0.1))
            
        elif difficulty == 'extreme':
            for i in range(8):
                x = 5 + np.random.uniform(-2, 2)
                y = np.random.uniform(-5, 5)
                vx = np.random.uniform(-0.3, 0.3)
                vy = np.random.uniform(-0.3, 0.3)
                dynamic_obs.append(DynamicObstacle(x, y, 0.4 + np.random.uniform(0, 0.3), vx=vx, vy=vy))
                
        return start, goal, static_obs, dynamic_obs
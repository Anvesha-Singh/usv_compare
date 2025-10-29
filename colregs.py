# colregs.py
# COLREGs (International Regulations for Preventing Collisions at Sea) implementation
# References: IMO COLREG 1972 Rules 4-19

import math
import numpy as np
from typing import Tuple, List, Optional


class EncounterType:
    """Classify vessel encounter scenarios per COLREGs"""
    HEAD_ON = "head_on"           # Rule 14
    CROSSING_STARBOARD = "crossing_starboard"  # Rule 15 - give way
    CROSSING_PORT = "crossing_port"  # Rule 15 - stand on
    OVERTAKING = "overtaking"     # Rule 13
    SAFE = "safe"                 # No risk of collision


class COLREGsChecker:
    """Detect COLREGs violations and compute compliance metrics"""
    
    def __init__(self, 
                 collision_distance_threshold: float = 2.0,
                 encounter_angle_tolerance: float = math.radians(5)):
        """
        Args:
            collision_distance_threshold: Distance at which collision is imminent (meters)
            encounter_angle_tolerance: Angle tolerance for encounter classification (radians)
        """
        self.collision_distance_threshold = collision_distance_threshold
        self.encounter_angle_tolerance = encounter_angle_tolerance
        self.colregs_events: List[dict] = []
    
    def classify_encounter(self, 
                          own_pos: Tuple[float, float], 
                          own_heading: float,
                          own_speed: float,
                          other_pos: Tuple[float, float],
                          other_heading: float,
                          other_speed: float) -> str:
        """
        Classify the type of encounter between two vessels
        
        Args:
            own_pos: (x, y) position of own vessel
            own_heading: Heading angle of own vessel (radians)
            own_speed: Speed of own vessel (m/s)
            other_pos: (x, y) position of other vessel
            other_heading: Heading angle of other vessel (radians)
            other_speed: Speed of other vessel (m/s)
            
        Returns:
            EncounterType constant
        """
        # Calculate relative position
        dx = other_pos[0] - own_pos[0]
        dy = other_pos[1] - own_pos[1]
        distance = math.hypot(dx, dy)
        
        # If too far, no immediate risk
        if distance > self.collision_distance_threshold * 2:
            return EncounterType.SAFE
        
        # Calculate bearing to other vessel (relative to own heading)
        bearing_to_other = math.atan2(dy, dx)
        relative_bearing = self._normalize_angle(bearing_to_other - own_heading)
        
        # Calculate relative heading (difference between headings)
        heading_diff = self._normalize_angle(other_heading - own_heading)
        
        # HEAD-ON: Other vessel heading toward you (±10° tolerance)
        if abs(self._normalize_angle(heading_diff + math.pi)) < math.radians(10):
            return EncounterType.HEAD_ON
        
        # OVERTAKING: Other vessel behind you, catching up
        if abs(relative_bearing) < math.radians(67.5) and abs(heading_diff) < math.radians(45):
            if distance < self.collision_distance_threshold:
                return EncounterType.OVERTAKING
        
        # CROSSING: Other vessel approaching from side
        # Starboard (right) crossing: Relative bearing 0-90°
        if 0 < relative_bearing < math.radians(90):
            return EncounterType.CROSSING_STARBOARD  # We give way
        
        # Port (left) crossing: Relative bearing 270-360° (-90 to 0°)
        if -math.radians(90) < relative_bearing < 0:
            return EncounterType.CROSSING_PORT  # Other gives way
        
        return EncounterType.SAFE
    
    def check_colregs_violation(self,
                               own_pos: Tuple[float, float],
                               own_heading: float,
                               own_speed: float,
                               planned_heading: float,  # Next heading command
                               other_pos: Tuple[float, float],
                               other_heading: float,
                               other_speed: float) -> Tuple[bool, Optional[str]]:
        """
        Check if planned maneuver violates COLREGs
        
        Returns:
            (is_violation, violation_type)
        """
        encounter = self.classify_encounter(
            own_pos, own_heading, own_speed,
            other_pos, other_heading, other_speed
        )
        
        if encounter == EncounterType.SAFE:
            return False, None
        
        # Check if planned heading violates COLREGs
        heading_change = self._normalize_angle(planned_heading - own_heading)
        
        # Rule 14: Head-on - both must turn to starboard (right)
        if encounter == EncounterType.HEAD_ON:
            if heading_change < math.radians(10):  # Not turning right enough
                return True, "HEAD_ON_NOT_TURNING_STARBOARD"
        
        # Rule 15: Crossing - starboard vessel gives way (turns right/away)
        elif encounter == EncounterType.CROSSING_STARBOARD:
            # Should turn right (positive heading change)
            if heading_change < math.radians(5):
                return True, "CROSSING_STARBOARD_NOT_GIVING_WAY"
        
        # Rule 13: Overtaking - overtaking vessel gives way (turns away)
        elif encounter == EncounterType.OVERTAKING:
            dx = other_pos[0] - own_pos[0]
            dy = other_pos[1] - own_pos[1]
            other_bearing = math.atan2(dy, dx)
            
            # Try to stay away from other vessel's path
            away_heading = other_bearing + math.pi  # Go opposite direction
            heading_to_away = self._normalize_angle(away_heading - planned_heading)
            
            if abs(heading_to_away) > math.radians(30):  # Not turning away enough
                return True, "OVERTAKING_NOT_GIVING_WAY"
        
        return False, None
    
    def get_colregs_correction_heading(self,
                                      own_pos: Tuple[float, float],
                                      own_heading: float,
                                      other_pos: Tuple[float, float],
                                      encounter: str  
                                      ) -> float:
        """
        Return a heading that complies with COLREGs given current situation

        Args:
            own_pos: (x, y) position of own vessel
            own_heading: Heading angle of own vessel (radians)
            other_pos: (x, y) position of other vessel
            encounter: The EncounterType constant
        
        Returns:
            Recommended heading change (radians) to avoid collision and comply with COLREGs
        """
        
        dx = other_pos[0] - own_pos[0]
        dy = other_pos[1] - own_pos[1]
        bearing_to_other = math.atan2(dy, dx)
        
        # Rule 14: Head-on - both turn starboard (90° right)
        if encounter == EncounterType.HEAD_ON:
            return self._normalize_angle(own_heading + math.radians(90))
        
        # Rule 15: Crossing starboard - turn right and slow down
        elif encounter == EncounterType.CROSSING_STARBOARD:
            return self._normalize_angle(own_heading + math.radians(60))
        
        # Rule 13: Overtaking - give way (turn away)
        elif encounter == EncounterType.OVERTAKING:
            return self._normalize_angle(bearing_to_other + math.pi)
        
        # Crossing port - stand on (maintain course)
        elif encounter == EncounterType.CROSSING_PORT:
            return own_heading
        
        return own_heading
    
    def calculate_colregs_compliance(self, 
                                    trajectory: List[Tuple[float, float]],
                                    other_vessel_trajectory: List[Tuple[float, float]],
                                    own_headings: List[float],
                                    other_headings: List[float]) -> Tuple[float, int]:
        """
        Calculate COLREGs compliance score for a completed trajectory
        
        Returns:
            (compliance_percentage, violation_count)
        """
        violations = 0
        total_steps = min(len(trajectory), len(other_vessel_trajectory))
        
        for i in range(total_steps):
            if i >= len(own_headings) or i >= len(other_headings):
                break
            
            is_violation, _ = self.check_colregs_violation(
                trajectory[i],
                own_headings[i],
                0.0,  # Speed not tracked in this version
                own_headings[i],
                other_vessel_trajectory[i],
                other_headings[i],
                0.0
            )
            
            if is_violation:
                violations += 1
        
        compliance = (1.0 - violations / max(total_steps, 1)) * 100
        return compliance, violations
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class COLREGsMetrics:
    """Track COLREGs compliance metrics"""
    
    def __init__(self):
        self.violations: List[str] = []
        self.compliance_score: float = 100.0
        self.encounters_detected: int = 0
        self.evasive_maneuvers: int = 0

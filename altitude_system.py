import time
import logging
import random
import threading
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Union, Any


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("aircraft_control.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger("AircraftControl")


class AltitudeMonitor:
    """Module for monitoring aircraft altitude and comparing to target."""
    
    def __init__(self, tolerance: float = 10.0):
        """
        Initialize altitude monitor.
        
        Args:
            tolerance: Acceptable deviation from target altitude in meters.
        """
        self.altitude_tolerance = tolerance
        logger.info(f"AltitudeMonitor initialized with tolerance of ±{self.altitude_tolerance}m")
    
    def compare_altitude(self, current_altitude: float, target_altitude: float) -> Dict:
        """
        Compare current altitude to target and determine if within tolerance.
        
        Args:
            current_altitude: Current aircraft altitude in meters.
            target_altitude: Target altitude in meters.
            
        Returns:
            Dictionary with deviation and status information.
        """
        deviation = current_altitude - target_altitude
        result = {
            "deviation": deviation,
            "within_tolerance": abs(deviation) <= self.altitude_tolerance,
            "above_target": deviation > 0,
            "below_target": deviation < 0,
            "current_altitude": current_altitude,
            "target_altitude": target_altitude
        }
        
        logger.debug(f"Altitude comparison: current={current_altitude:.1f}m, " 
                    f"target={target_altitude:.1f}m, deviation={deviation:.1f}m, "
                    f"within_tolerance={abs(deviation) <= self.altitude_tolerance}")
        
        return result


class PitchController:
    """Module for calculating and applying pitch adjustments."""
    
    def __init__(self, 
                max_pitch_adjustment: float = 2.0,
                min_pitch: float = -15.0, 
                max_pitch: float = 15.0,
                stabilization_timeout: float = 5.0):
        """
        Initialize pitch controller.
        
        Args:
            max_pitch_adjustment: Maximum pitch change in degrees per adjustment.
            min_pitch: Minimum allowed pitch angle in degrees.
            max_pitch: Maximum allowed pitch angle in degrees.
            stabilization_timeout: Maximum time allowed for stabilization in seconds.
        """
        self.max_pitch_adjustment = max_pitch_adjustment
        self.min_pitch = min_pitch
        self.max_pitch = max_pitch
        self.stabilization_timeout = stabilization_timeout
        
        # For tracking pitch commands and oscillation detection
        self.command_history = []
        self.stabilization_start_time = None
        self.oscillation_threshold = 3  # Number of direction changes to detect oscillation
        
        logger.info(f"PitchController initialized with max adjustment={max_pitch_adjustment}°, "
                   f"pitch range=[{min_pitch}°, {max_pitch}°], "
                   f"stabilization timeout={stabilization_timeout}s")
    
    def calculate_pitch_adjustment(self, altitude_deviation: float, current_altitude: float) -> float:
        """
        Calculate required pitch adjustment based on altitude deviation.
        
        Args:
            altitude_deviation: Difference between current and target altitude in meters.
            current_altitude: Current altitude in meters.
            
        Returns:
            Required pitch adjustment in degrees.
        """
        # Intentional logic issue: Above 5000m, the adjustment is too aggressive,
        # causing oscillation and potential instability
        if current_altitude >= 5000:
            # More aggressive adjustment at high altitudes (logic issue)
            adjustment = (altitude_deviation / 50) * -1.5
            logger.debug(f"High altitude pitch calculation (intentional logic issue): "
                        f"deviation={altitude_deviation:.1f}m, adjustment={adjustment:.2f}°")
        else:
            # Normal adjustment at lower altitudes
            adjustment = (altitude_deviation / 100) * -1
            logger.debug(f"Normal altitude pitch calculation: "
                        f"deviation={altitude_deviation:.1f}m, adjustment={adjustment:.2f}°")
        
        # Limit adjustment to maximum allowed
        clamped_adjustment = max(min(adjustment, self.max_pitch_adjustment), -self.max_pitch_adjustment)
        
        if clamped_adjustment != adjustment:
            logger.debug(f"Pitch adjustment clamped from {adjustment:.2f}° to {clamped_adjustment:.2f}°")
        
        return clamped_adjustment
    
    def adjust_pitch(self, current_pitch: float, adjustment: float) -> float:
        """
        Apply pitch adjustment to current pitch while respecting limits.
        
        Args:
            current_pitch: Current pitch angle in degrees.
            adjustment: Pitch adjustment to apply in degrees.
            
        Returns:
            New pitch angle in degrees.
        """
        new_pitch = current_pitch + adjustment
        
        # Apply pitch limits
        clamped_pitch = max(min(new_pitch, self.max_pitch), self.min_pitch)
        
        # Record command for oscillation detection
        self._record_command(adjustment)
        
        if clamped_pitch != new_pitch:
            logger.debug(f"Pitch clamped from {new_pitch:.2f}° to {clamped_pitch:.2f}°")
        
        return clamped_pitch
    
    def _record_command(self, adjustment: float) -> None:
        """
        Record pitch adjustment command for history and oscillation detection.
        
        Args:
            adjustment: Pitch adjustment in degrees.
        """
        command = {
            "adjustment": adjustment,
            "timestamp": time.time(),
            "direction": 1 if adjustment > 0 else (-1 if adjustment < 0 else 0)
        }
        
        self.command_history.append(command)
        
        # Keep only the last 10 commands for efficiency
        if len(self.command_history) > 10:
            self.command_history.pop(0)
    
    def detect_oscillation(self) -> bool:
        """
        Detect oscillating pitch adjustments.
        
        Returns:
            True if oscillation is detected, False otherwise.
        """
        if len(self.command_history) < 6:
            return False
        
        oscillation_count = 0
        previous_direction = self.command_history[0]["direction"]
        
        for command in self.command_history[1:]:
            current_direction = command["direction"]
            if current_direction != 0 and current_direction != previous_direction:
                oscillation_count += 1
                previous_direction = current_direction
        
        oscillating = oscillation_count >= self.oscillation_threshold
        
        if oscillating:
            logger.debug(f"Oscillation detected with {oscillation_count} direction changes")
        
        return oscillating
    
    def validate_pitch_command(self, current_pitch: float, new_pitch: float, adjustment: float) -> Dict:
        """
        Validate if a pitch adjustment is safe and within limits.
        
        Args:
            current_pitch: Current pitch angle in degrees.
            new_pitch: Resulting pitch angle in degrees.
            adjustment: Pitch adjustment in degrees.
            
        Returns:
            Dictionary with validation result and reason if invalid.
        """
        # Check if adjustment exceeds maximum allowed
        if abs(adjustment) > self.max_pitch_adjustment:
            return {
                "valid": False,
                "reason": f"Pitch adjustment {adjustment:.2f}° exceeds maximum allowed ({self.max_pitch_adjustment}°)"
            }
        
        # Check if new pitch is within allowed range
        if new_pitch < self.min_pitch or new_pitch > self.max_pitch:
            return {
                "valid": False,
                "reason": f"Resulting pitch {new_pitch:.2f}° outside allowed range ({self.min_pitch}° to {self.max_pitch}°)"
            }
        
        return {"valid": True}


class Autopilot:
    """Module for managing autopilot engagement and operation."""
    
    def __init__(self, engagement_altitude: float = 5000.0):
        """
        Initialize autopilot.
        
        Args:
            engagement_altitude: Altitude in meters at which autopilot engages.
        """
        self.engaged = False
        self.engagement_altitude = engagement_altitude
        logger.info(f"Autopilot initialized with engagement altitude of {engagement_altitude}m")
    
    def check_engagement(self, current_altitude: float, previous_altitude: float) -> Dict:
        """
        Check if autopilot should be engaged based on current altitude.
        
        Args:
            current_altitude: Current altitude in meters.
            previous_altitude: Previous altitude in meters.
            
        Returns:
            Dictionary with engagement status and information.
        """
        # Only engage if ascending past the threshold
        if (not self.engaged and 
            current_altitude >= self.engagement_altitude and 
            previous_altitude < self.engagement_altitude):
            
            self.engaged = True
            logger.info(f"AUTOPILOT ENGAGED at {current_altitude:.1f}m")
            
            return {
                "status": True,
                "action": "engaged",
                "message": f"Autopilot engaged at {current_altitude:.1f}m"
            }
        
        return {
            "status": self.engaged,
            "action": "unchanged"
        }


class SensorHandler:
    """Module for validating and handling sensor data."""
    
    def __init__(self, max_sudden_change: float = 50.0, max_consecutive_errors: int = 3):
        """
        Initialize sensor handler.
        
        Args:
            max_sudden_change: Maximum allowed sudden change in altitude in meters.
            max_consecutive_errors: Maximum allowed consecutive sensor errors.
        """
        self.last_valid_altitude = None
        self.last_valid_pitch = None
        self.consecutive_errors = 0
        self.max_consecutive_errors = max_consecutive_errors
        self.max_sudden_change = max_sudden_change
        
        logger.info(f"SensorHandler initialized with max sudden change={max_sudden_change}m, "
                   f"max consecutive errors={max_consecutive_errors}")
    
    def validate_sensor_data(self, altitude: Optional[float], pitch: Optional[float]) -> Dict:
        """
        Validate sensor data and handle invalid values.
        
        Args:
            altitude: Reported altitude in meters.
            pitch: Reported pitch in degrees.
            
        Returns:
            Dictionary with validated values and issue information.
        """
        issues = []
        validated_altitude = altitude
        validated_pitch = pitch
        
        # Check for None or NaN values
        if altitude is None:
            issues.append("Missing altitude data")
            validated_altitude = self.last_valid_altitude if self.last_valid_altitude is not None else 0.0
        
        if pitch is None:
            issues.append("Missing pitch data")
            validated_pitch = self.last_valid_pitch if self.last_valid_pitch is not None else 0.0
        
        # Check for invalid values
        if altitude is not None:
            try:
                altitude_float = float(altitude)
                if altitude_float < 0:
                    issues.append(f"Invalid negative altitude: {altitude_float}m")
                    validated_altitude = self.last_valid_altitude if self.last_valid_altitude is not None else 0.0
            except (ValueError, TypeError):
                issues.append(f"Non-numeric altitude value: {altitude}")
                validated_altitude = self.last_valid_altitude if self.last_valid_altitude is not None else 0.0
        
        if pitch is not None:
            try:
                pitch_float = float(pitch)
                if pitch_float < -90 or pitch_float > 90:
                    issues.append(f"Invalid pitch value outside range [-90, 90]: {pitch_float}°")
                    validated_pitch = self.last_valid_pitch if self.last_valid_pitch is not None else 0.0
            except (ValueError, TypeError):
                issues.append(f"Non-numeric pitch value: {pitch}")
                validated_pitch = self.last_valid_pitch if self.last_valid_pitch is not None else 0.0
        
        # Check for sudden unexpected changes
        if (self.last_valid_altitude is not None and validated_altitude is not None and
            abs(validated_altitude - self.last_valid_altitude) > self.max_sudden_change):
            issues.append(f"Suspicious altitude change: {validated_altitude - self.last_valid_altitude:.1f}m")
            validated_altitude = self.last_valid_altitude
        
        # Update error counter
        if issues:
            self.consecutive_errors += 1
            logger.warning(f"Sensor validation issues: {', '.join(issues)}")
        else:
            self.consecutive_errors = 0
            self.last_valid_altitude = validated_altitude
            self.last_valid_pitch = validated_pitch
        
        return {
            "original_altitude": altitude,
            "original_pitch": pitch,
            "altitude": validated_altitude,
            "pitch": validated_pitch,
            "issues": issues,
            "has_issues": len(issues) > 0,
            "critical_failure": self.consecutive_errors >= self.max_consecutive_errors
        }


class DataLogger:
    """Module for comprehensive logging of system operation."""
    
    def __init__(self, log_file: str = "aircraft_data.log"):
        """
        Initialize data logger.
        
        Args:
            log_file: Path to data log file.
        """
        self.log_file = log_file
        self.entries = []
        
        # Create a specific logger for data
        self.data_logger = logging.getLogger("AircraftData")
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        self.data_logger.addHandler(file_handler)
        self.data_logger.setLevel(logging.INFO)
        
        logger.info(f"DataLogger initialized with log file: {log_file}")
    
    def log_altitude_comparison(self, comparison_data: Dict) -> None:
        """
        Log altitude comparison data.
        
        Args:
            comparison_data: Altitude comparison results.
        """
        self.data_logger.info(
            f"ALTITUDE_COMPARISON,{comparison_data['current_altitude']:.2f},"
            f"{comparison_data['target_altitude']:.2f},{comparison_data['deviation']:.2f},"
            f"{comparison_data['within_tolerance']}"
        )
        self.entries.append({
            "type": "altitude_comparison",
            "timestamp": time.time(),
            "data": comparison_data
        })
    
    def log_pitch_adjustment(self, current_pitch: float, new_pitch: float, 
                           adjustment: float, altitude_deviation: float) -> None:
        """
        Log pitch adjustment data.
        
        Args:
            current_pitch: Current pitch in degrees.
            new_pitch: New pitch in degrees.
            adjustment: Pitch adjustment in degrees.
            altitude_deviation: Altitude deviation in meters.
        """
        self.data_logger.info(
            f"PITCH_ADJUSTMENT,{current_pitch:.2f},{new_pitch:.2f},"
            f"{adjustment:.2f},{altitude_deviation:.2f}"
        )
        self.entries.append({
            "type": "pitch_adjustment",
            "timestamp": time.time(),
            "data": {
                "current_pitch": current_pitch,
                "new_pitch": new_pitch,
                "adjustment": adjustment,
                "altitude_deviation": altitude_deviation
            }
        })
    
    def log_autopilot_event(self, event_type: str, altitude: float) -> None:
        """
        Log autopilot event.
        
        Args:
            event_type: Type of autopilot event.
            altitude: Current altitude in meters.
        """
        self.data_logger.info(f"AUTOPILOT_EVENT,{event_type},{altitude:.2f}")
        self.entries.append({
            "type": "autopilot_event",
            "timestamp": time.time(),
            "data": {
                "event": event_type,
                "altitude": altitude
            }
        })
    
    def log_error(self, error_type: str, message: str, data: Dict = None) -> None:
        """
        Log error event.
        
        Args:
            error_type: Type of error.
            message: Error message.
            data: Additional error data.
        """
        data_str = ",".join([f"{k}={v}" for k, v in (data or {}).items()])
        self.data_logger.error(f"ERROR,{error_type},{message},{data_str}")
        self.entries.append({
            "type": "error",
            "timestamp": time.time(),
            "data": {
                "error_type": error_type,
                "message": message,
                "details": data
            }
        })
    
    def log_sensor_issue(self, issues: List[str], original_data: Dict, corrected_data: Dict) -> None:
        """
        Log sensor data issues.
        
        Args:
            issues: List of sensor issues.
            original_data: Original sensor data.
            corrected_data: Corrected sensor data.
        """
        issues_str = ";".join(issues)
        self.data_logger.warning(
            f"SENSOR_ISSUE,{issues_str},"
            f"orig_alt={original_data.get('altitude')},orig_pitch={original_data.get('pitch')},"
            f"corr_alt={corrected_data.get('altitude')},corr_pitch={corrected_data.get('pitch')}"
        )
        self.entries.append({
            "type": "sensor_issue",
            "timestamp": time.time(),
            "data": {
                "issues": issues,
                "original_data": original_data,
                "corrected_data": corrected_data
            }
        })


class AircraftAltitudeControl:
    """Main class for aircraft altitude control system."""
    
    def __init__(self, target_altitude: float = 6000.0, initial_altitude: float = 3000.0, 
                initial_pitch: float = 5.0):
        """
        Initialize aircraft altitude control system.
        
        Args:
            target_altitude: Target altitude in meters.
            initial_altitude: Initial altitude in meters.
            initial_pitch: Initial pitch in degrees.
        """
        # Initialize components
        self.altitude_monitor = AltitudeMonitor()
        self.pitch_controller = PitchController()
        self.autopilot = Autopilot()
        self.sensor_handler = SensorHandler()
        self.data_logger = DataLogger()
        
        # Initialize state
        self.target_altitude = target_altitude
        self.current_altitude = initial_altitude
        self.previous_altitude = initial_altitude
        self.current_pitch = initial_pitch
        
        # Control state
        self.running = False
        self.stabilized = False
        self.stabilization_start_time = None
        
        logger.info(f"AircraftAltitudeControl initialized with target altitude={target_altitude}m, "
                   f"initial altitude={initial_altitude}m, initial pitch={initial_pitch}°")
    
    def start(self) -> None:
        """Start the altitude control system."""
        if self.running:
            logger.warning("Altitude control system already running")
            return
        
        self.running = True
        logger.info("Altitude control system started")
        
        # Start control loop in a separate thread
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
    
    def stop(self) -> None:
        """Stop the altitude control system."""
        if not self.running:
            logger.warning("Altitude control system already stopped")
            return
        
        self.running = False
        logger.info("Altitude control system stopped")
    
    def _control_loop(self) -> None:
        """Main control loop for altitude management."""
        last_time = time.time()
        
        while self.running:
            # Calculate time step
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            try:
                # Process sensor data and update aircraft state
                self._update_aircraft_state(dt)
                
                # Check for autopilot engagement
                self._check_autopilot()
                
                # If autopilot is engaged, control altitude
                if self.autopilot.engaged:
                    self._control_altitude()
                
                # Sleep a bit to avoid high CPU usage
                time.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in control loop: {str(e)}", exc_info=True)
                self.data_logger.log_error("control_loop", str(e))
                time.sleep(1)  # Sleep longer on error
    
    def _update_aircraft_state(self, dt: float) -> None:
        """
        Update aircraft state based on current flight parameters.
        
        Args:
            dt: Time step in seconds.
        """
        # Store previous altitude
        self.previous_altitude = self.current_altitude
        
        # Simple physics model: altitude change proportional to pitch
        # (simplified for simulation purposes)
        altitude_change = self.current_pitch * 5 * dt
        raw_new_altitude = self.current_altitude + altitude_change
        
        # Randomly inject sensor fault (for testing) - 2% chance based on altitude being above 5000m
        if self.current_altitude > 5000 and random.random() < 0.02:
            self._inject_sensor_fault(raw_new_altitude)
            return
            
        # Validate sensor data
        sensor_data = self.sensor_handler.validate_sensor_data(
            raw_new_altitude, self.current_pitch)
        
        if sensor_data["has_issues"]:
            self.data_logger.log_sensor_issue(
                sensor_data["issues"],
                {"altitude": sensor_data["original_altitude"], "pitch": sensor_data["original_pitch"]},
                {"altitude": sensor_data["altitude"], "pitch": sensor_data["pitch"]}
            )
            
            if sensor_data["critical_failure"]:
                logger.critical("CRITICAL FAILURE: Multiple consecutive sensor errors")
                self.data_logger.log_error(
                    "sensor_critical", 
                    "Multiple consecutive sensor failures",
                    {"consecutive_errors": self.sensor_handler.consecutive_errors}
                )
                # In a real system, this would trigger emergency protocols or be reset by watchdog timer
                # For this simulation, we'll just continue with the validated data
        
        # Update aircraft state with validated data
        self.current_altitude = sensor_data["altitude"]
        logger.info(f"Altitude {self.current_altitude}f")
    
    def _inject_sensor_fault(self, current_altitude: float) -> None:
        """
        Inject a simulated sensor fault for testing purposes where a sensor is misreporting values.
        
        Args:
            current_altitude: Current true altitude before fault injection.
        """
        fault_types = ["null_reading", "invalid_value", "sudden_spike"]
        fault_type = random.choice(fault_types)
        
        if fault_type == "null_reading":
            self.current_altitude = 0
            logger.warning("INJECTED FAULT: low altitude reading")
        elif fault_type == "invalid_value":
            self.current_altitude = -5
            logger.warning("INJECTED FAULT: Invalid altitude value")
        elif fault_type == "sudden_spike":
            spike = 50 if random.random() > 0.5 else -50
            self.current_altitude = current_altitude + spike
            logger.warning(f"INJECTED FAULT: Sudden altitude spike ({spike}m)")
    
    def _check_autopilot(self) -> None:
        """Check and handle autopilot engagement."""
        autopilot_status = self.autopilot.check_engagement(
            self.current_altitude, self.previous_altitude)
        
        if autopilot_status["action"] == "engaged":
            self.data_logger.log_autopilot_event("engage", self.current_altitude)
    
    def _control_altitude(self) -> None:
        """Control altitude when autopilot is engaged."""
        # Compare current altitude to target
        comparison = self.altitude_monitor.compare_altitude(
            self.current_altitude, self.target_altitude)
        
        # Log the comparison
        self.data_logger.log_altitude_comparison(comparison)
        
        # Check if we need to adjust pitch
        if not comparison["within_tolerance"]:
            # If we were stabilized, we're no longer
            if self.stabilized:
                self.stabilized = False
                self.stabilization_start_time = None
                logger.warning(
                    f"Altitude destabilized: current={self.current_altitude:.1f}m, "
                    f"target={self.target_altitude:.1f}m, deviation={comparison['deviation']:.1f}m"
                )
            
            # Calculate pitch adjustment
            adjustment = self.pitch_controller.calculate_pitch_adjustment(
                comparison["deviation"], self.current_altitude)
            
            # Calculate new pitch
            new_pitch = self.pitch_controller.adjust_pitch(self.current_pitch, adjustment)
            
            # Validate the pitch command
            validation = self.pitch_controller.validate_pitch_command(
                self.current_pitch, new_pitch, adjustment)
            
            if validation["valid"]:
                # Apply the new pitch
                previous_pitch = self.current_pitch
                self.current_pitch = new_pitch
                
                logger.info(
                    f"Pitch adjusted by {adjustment:.2f}° (from {previous_pitch:.2f}° to {new_pitch:.2f}°) "
                    f"to correct altitude deviation of {comparison['deviation']:.1f}m"
                )
                
                # Log the adjustment
                self.data_logger.log_pitch_adjustment(
                    previous_pitch, new_pitch, adjustment, comparison["deviation"])
                
                # Start stabilization timer if not already running
                if not self.stabilization_start_time:
                    self.stabilization_start_time = time.time()
                
                # Check for pitch oscillation at high altitude
                if (self.current_altitude >= 5000 and 
                    self.pitch_controller.detect_oscillation()):
                    oscillation_msg = "Pitch oscillation detected at high altitude"
                    logger.warning(f"WARNING: {oscillation_msg}")
                    self.data_logger.log_error(
                        "oscillation", 
                        oscillation_msg,
                        {
                            "altitude": self.current_altitude,
                            "oscillation_count": len(self.pitch_controller.command_history)
                        }
                    )
                
                # Check if we've been trying to stabilize too long
                if (self.stabilization_start_time and 
                    time.time() - self.stabilization_start_time > self.pitch_controller.stabilization_timeout):
                    stabilization_msg = "Failed to stabilize altitude within timeout period"
                    logger.error(f"ERROR: {stabilization_msg}")
                    self.data_logger.log_error(
                        "stabilization_timeout", 
                        stabilization_msg,
                        {
                            "current_altitude": self.current_altitude,
                            "target_altitude": self.target_altitude,
                            "deviation": comparison["deviation"],
                            "timeout_seconds": self.pitch_controller.stabilization_timeout
                        }
                    )
                    # Reset the timer to prevent continuous logging
                    self.stabilization_start_time = time.time()
            else:
                # Log the invalid command
                logger.error(f"Invalid pitch command rejected: {validation['reason']}")
                self.data_logger.log_error(
                    "invalid_pitch", 
                    validation["reason"],
                    {
                        "current_pitch": self.current_pitch,
                        "requested_pitch": new_pitch,
                        "adjustment": adjustment
                    }
                )
        elif not self.stabilized:
            # We're within tolerance, mark as stabilized
            self.stabilized = True
            self.stabilization_start_time = None
            logger.info(
                f"Altitude stabilized within tolerance: current={self.current_altitude:.1f}m, "
                f"target={self.target_altitude:.1f}m, deviation={comparison['deviation']:.1f}m"
            )
            self.data_logger.log_autopilot_event("stabilized", self.current_altitude)


def simulate_flight() -> None:
    """Run a flight simulation."""
    # Create the aircraft control system
    aircraft = AircraftAltitudeControl(
        target_altitude=6000.0,  # Target altitude in meters
        initial_altitude=3000.0,  # Starting altitude in meters
        initial_pitch=5.0        # Starting pitch in degrees
    )
    
    # Start the control system
    aircraft.start()
    
    try:
        # Simulate for 5 minutes (real time)
        logger.info("Starting flight simulation for 5 minutes...")
        time.sleep(300)
    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user")
    finally:
        # Stop the control system
        aircraft.stop()
        logger.info("Flight simulation completed")


if __name__ == "__main__":
    logger.info("Aircraft Altitude Control System")
    logger.info("===============================")
    simulate_flight()


"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Tolerance for altitude adjustment (meters)
HEIGHT_TOLERANCE = 0.5
# Tolerance for yaw adjustment (degrees)
ANGLE_TOLERANCE = 5.0
# Speed for altitude change (m/s)
Z_SPEED = 1.0
# Turning speed for yaw change (deg/s)
TURNING_SPEED = 5.0


class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.

        Parameters:
            connection: MAVLink connection to the drone.
            target: Target position to face and match altitude.
            local_logger: Logger instance for logging messages.

        Returns:
            A tuple containing success status and the Command object (or None on failure).
        """
        if connection is None:
            return False, None

        if target is None:
            return False, None

        if local_logger is None:
            return False, None

        return True, Command(cls.__private_key, connection, target, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        self.__connection = connection
        self.__target = target
        self.__logger = local_logger

        # For calculating average velocity
        self.__total_velocity_x = 0.0
        self.__total_velocity_y = 0.0
        self.__total_velocity_z = 0.0
        self.__sample_count = 0

    def run(
        self,
        telemetry_data: telemetry.TelemetryData,
    ) -> "tuple[True, str] | tuple[False, None]":
        """
        Make a decision based on received telemetry data.

        Parameters:
            telemetry_data: Current telemetry data from the drone.

        Returns:
            A tuple (success, status_string) describing the command sent.
        """
        if telemetry_data is None:
            self.__logger.error("Received None telemetry data")
            return False, None

        # Update average velocity calculation
        if (
            telemetry_data.x_velocity is not None
            and telemetry_data.y_velocity is not None
            and telemetry_data.z_velocity is not None
        ):
            self.__total_velocity_x += telemetry_data.x_velocity
            self.__total_velocity_y += telemetry_data.y_velocity
            self.__total_velocity_z += telemetry_data.z_velocity
            self.__sample_count += 1

            # Calculate and log average velocity
            avg_vx = self.__total_velocity_x / self.__sample_count
            avg_vy = self.__total_velocity_y / self.__sample_count
            avg_vz = self.__total_velocity_z / self.__sample_count
            self.__logger.info(f"Average velocity: ({avg_vx:.3f}, {avg_vy:.3f}, {avg_vz:.3f}) m/s")

        # Check if altitude adjustment is needed
        if telemetry_data.z is not None:
            delta_z = self.__target.z - telemetry_data.z
            if abs(delta_z) > HEIGHT_TOLERANCE:
                # Send altitude change command
                self.__connection.mav.command_long_send(
                    1,  # target_system
                    0,  # target_component
                    mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,  # command (113)
                    0,  # confirmation
                    Z_SPEED,  # param1: descent/ascend rate (m/s)
                    0,  # param2: empty
                    0,  # param3: empty
                    0,  # param4: empty
                    0,  # param5: empty
                    0,  # param6: empty
                    self.__target.z,  # param7: target altitude
                )
                self.__logger.info(f"Sent altitude change command: delta_z={delta_z:.2f}m")
                return True, f"CHANGE ALTITUDE: {delta_z}"

        # Check if yaw adjustment is needed
        if (
            telemetry_data.x is not None
            and telemetry_data.y is not None
            and telemetry_data.yaw is not None
        ):
            # Calculate the angle to the target from the drone's position
            dx = self.__target.x - telemetry_data.x
            dy = self.__target.y - telemetry_data.y

            # Calculate desired yaw (angle to target)
            desired_yaw = math.atan2(dy, dx)

            # Calculate the difference between current yaw and desired yaw
            delta_yaw = desired_yaw - telemetry_data.yaw

            # Normalize to [-pi, pi]
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            # Convert to degrees for comparison
            delta_yaw_deg = math.degrees(delta_yaw)

            if abs(delta_yaw_deg) > ANGLE_TOLERANCE:
                # Determine direction: 1 = CCW (positive), -1 = CW (negative)
                direction = 1.0 if delta_yaw_deg >= 0 else -1.0

                # Send yaw change command
                self.__connection.mav.command_long_send(
                    1,  # target_system
                    0,  # target_component
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command (115)
                    0,  # confirmation
                    abs(delta_yaw_deg),  # param1: target angle (degrees)
                    TURNING_SPEED,  # param2: angular speed (deg/s)
                    direction,  # param3: direction (1=CCW, -1=CW)
                    1.0,  # param4: relative (1=relative to current)
                    0,  # param5: empty
                    0,  # param6: empty
                    0,  # param7: empty
                )
                self.__logger.info(f"Sent yaw change command: delta_yaw={delta_yaw_deg:.2f}deg")
                return True, f"CHANGE YAW: {delta_yaw_deg}"

        # No command needed
        return False, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

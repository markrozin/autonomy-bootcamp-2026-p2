"""
Telemetry gathering logic.
"""

import time

from pymavlink import mavutil

from ..common.modules.logger import logger


class TelemetryData:  # pylint: disable=too-many-instance-attributes
    """
    Python struct to represent Telemtry Data. Contains the most recent attitude and position reading.
    """

    def __init__(
        self,
        time_since_boot: int | None = None,  # ms
        x: float | None = None,  # m
        y: float | None = None,  # m
        z: float | None = None,  # m
        x_velocity: float | None = None,  # m/s
        y_velocity: float | None = None,  # m/s
        z_velocity: float | None = None,  # m/s
        roll: float | None = None,  # rad
        pitch: float | None = None,  # rad
        yaw: float | None = None,  # rad
        roll_speed: float | None = None,  # rad/s
        pitch_speed: float | None = None,  # rad/s
        yaw_speed: float | None = None,  # rad/s
    ) -> None:
        self.time_since_boot = time_since_boot
        self.x = x
        self.y = y
        self.z = z
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.z_velocity = z_velocity
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_speed = roll_speed
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed

    def __str__(self) -> str:
        return f"""{{
            time_since_boot: {self.time_since_boot},
            x: {self.x},
            y: {self.y},
            z: {self.z},
            x_velocity: {self.x_velocity},
            y_velocity: {self.y_velocity},
            z_velocity: {self.z_velocity},
            roll: {self.roll},
            pitch: {self.pitch},
            yaw: {self.yaw},
            roll_speed: {self.roll_speed},
            pitch_speed: {self.pitch_speed},
            yaw_speed: {self.yaw_speed}
        }}"""


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Timeout in seconds for receiving both messages
TELEMETRY_TIMEOUT = 1.0


class Telemetry:
    """
    Telemetry class to read position and attitude (orientation).

    Receives ATTITUDE (30) and LOCAL_POSITION_NED (32) messages and combines them
    into a single TelemetryData object.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, Telemetry] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Telemetry object.

        Parameters:
            connection: MAVLink connection to the drone.
            local_logger: Logger instance for logging messages.

        Returns:
            A tuple containing success status and the Telemetry object (or None on failure).
        """
        if connection is None:
            return False, None

        if local_logger is None:
            return False, None

        return True, Telemetry(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Telemetry.__private_key, "Use create() method"

        self.__connection = connection
        self.__logger = local_logger

    def run(self) -> "tuple[True, TelemetryData] | tuple[False, None]":
        """
        Receive LOCAL_POSITION_NED and ATTITUDE messages from the drone,
        combining them together to form a single TelemetryData object.

        Returns:
            A tuple (success, TelemetryData) or (False, None) on timeout.
        """
        start_time = time.time()
        remaining_timeout = TELEMETRY_TIMEOUT

        attitude_data = None
        position_data = None
        attitude_time = 0
        position_time = 0

        # Keep trying to receive both messages within the timeout
        while remaining_timeout > 0:
            # Try to receive any telemetry message
            msg = self.__connection.recv_match(
                type=["ATTITUDE", "LOCAL_POSITION_NED"],
                blocking=True,
                timeout=remaining_timeout,
            )

            if msg is not None:
                msg_type = msg.get_type()

                if msg_type == "ATTITUDE":
                    attitude_data = msg
                    attitude_time = msg.time_boot_ms
                    self.__logger.debug(f"Received ATTITUDE: yaw={msg.yaw}")

                elif msg_type == "LOCAL_POSITION_NED":
                    position_data = msg
                    position_time = msg.time_boot_ms
                    self.__logger.debug(f"Received LOCAL_POSITION_NED: x={msg.x}, y={msg.y}, z={msg.z}")

            # Check if we have both messages
            if attitude_data is not None and position_data is not None:
                # Use the most recent timestamp
                most_recent_time = max(attitude_time, position_time)

                # Create TelemetryData with combined data
                # Note: LOCAL_POSITION_NED uses NED coordinates, but we're treating it as x-y-z
                # as per bootcamp instructions (x=right, y=forward, z=up)
                telemetry_data = TelemetryData(
                    time_since_boot=most_recent_time,
                    x=position_data.x,
                    y=position_data.y,
                    z=position_data.z,
                    x_velocity=position_data.vx,
                    y_velocity=position_data.vy,
                    z_velocity=position_data.vz,
                    roll=attitude_data.roll,
                    pitch=attitude_data.pitch,
                    yaw=attitude_data.yaw,
                    roll_speed=attitude_data.rollspeed,
                    pitch_speed=attitude_data.pitchspeed,
                    yaw_speed=attitude_data.yawspeed,
                )

                self.__logger.info(f"Telemetry data received: {telemetry_data}")
                return True, telemetry_data

            # Update remaining timeout
            elapsed = time.time() - start_time
            remaining_timeout = TELEMETRY_TIMEOUT - elapsed

        # Timeout: did not receive both messages in time
        self.__logger.error("Telemetry timeout: did not receive both ATTITUDE and LOCAL_POSITION_NED")
        return False, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

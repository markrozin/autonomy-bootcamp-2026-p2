"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Heartbeat period in seconds
HEARTBEAT_PERIOD = 1.0
# Number of missed heartbeats before considered disconnected
DISCONNECT_THRESHOLD = 5


class HeartbeatReceiver:
    """
    HeartbeatReceiver class to receive heartbeats from the drone and track connection status.

    Receives HEARTBEAT messages at 1Hz. If connected, becomes disconnected after
    missing DISCONNECT_THRESHOLD consecutive heartbeats.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.

        Parameters:
            connection: MAVLink connection to the drone.
            local_logger: Logger instance for logging messages.

        Returns:
            A tuple containing success status and the HeartbeatReceiver object (or None on failure).
        """
        if connection is None:
            return False, None

        if local_logger is None:
            return False, None

        return True, HeartbeatReceiver(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        self.__connection = connection
        self.__logger = local_logger
        self.__missed_heartbeats = 0
        self.__is_connected = False

    def run(self) -> tuple[bool, str]:
        """
        Attempt to receive a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.

        Returns:
            A tuple (success, status_string) where status_string is "connected" or "disconnected".
        """
        try:
            # Try to receive a heartbeat message with timeout
            msg = self.__connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=HEARTBEAT_PERIOD,
            )

            if msg is not None and msg.get_type() == "HEARTBEAT":
                # Successfully received a heartbeat
                self.__missed_heartbeats = 0
                self.__is_connected = True
                self.__logger.info("Received heartbeat from drone")
            else:
                # Did not receive a heartbeat in time
                self.__missed_heartbeats += 1
                self.__logger.warning(
                    f"Missed heartbeat ({self.__missed_heartbeats}/{DISCONNECT_THRESHOLD})"
                )

                # Check if we have exceeded the disconnect threshold
                if self.__missed_heartbeats >= DISCONNECT_THRESHOLD:
                    self.__is_connected = False
                    self.__logger.warning("Connection lost: too many missed heartbeats")

        except Exception as e:  # pylint: disable=broad-except
            self.__missed_heartbeats += 1
            self.__logger.error(f"Error receiving heartbeat: {e}")

            if self.__missed_heartbeats >= DISCONNECT_THRESHOLD:
                self.__is_connected = False

        # Return current connection status
        status = "connected" if self.__is_connected else "disconnected"
        return True, status


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

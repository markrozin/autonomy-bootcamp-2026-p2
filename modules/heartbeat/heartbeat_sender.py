"""
Heartbeat sending logic.
"""

from pymavlink import mavutil


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatSender:
    """
    HeartbeatSender class to send a heartbeat message to the drone.
    Sends HEARTBEAT messages at a regular interval (once per second).
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
    ) -> "tuple[True, HeartbeatSender] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatSender object.

        Parameters:
            connection: MAVLink connection to the drone.

        Returns:
            A tuple containing success status and the HeartbeatSender object (or None on failure).
        """
        if connection is None:
            return False, None

        return True, HeartbeatSender(cls.__private_key, connection)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
    ):
        assert key is HeartbeatSender.__private_key, "Use create() method"

        self.__connection = connection

    def run(self) -> tuple[bool, None]:
        """
        Attempt to send a heartbeat message.

        Sends a MAVLink HEARTBEAT message with:
        - type: MAV_TYPE_GCS (Ground Control Station)
        - autopilot: MAV_AUTOPILOT_INVALID (not an autopilot)

        Returns:
            A tuple (success, None) where success indicates if the heartbeat was sent.
        """
        try:
            self.__connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # type: Ground Control Station
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # autopilot: not an autopilot
                0,  # base_mode
                0,  # custom_mode
                0,  # system_status
            )
            return True, None
        except Exception:  # pylint: disable=broad-except
            return False, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

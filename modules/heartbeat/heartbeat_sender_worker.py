"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib
import time

from pymavlink import mavutil

from utilities.workers import worker_controller
from . import heartbeat_sender
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Heartbeat period in seconds
HEARTBEAT_PERIOD = 1


def heartbeat_sender_worker(
    connection: mavutil.mavfile,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process that sends heartbeat messages to the drone at 1Hz.

    Parameters:
        connection: MAVLink connection to the drone.
        controller: WorkerController for managing pause/exit requests.
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (heartbeat_sender.HeartbeatSender)
    result, sender = heartbeat_sender.HeartbeatSender.create(connection)
    if not result:
        local_logger.error("Failed to create HeartbeatSender")
        return

    # Get Pylance to stop complaining
    assert sender is not None

    local_logger.info("HeartbeatSender created successfully")

    # Main loop: do work.
    next_heartbeat_time = time.time()

    while not controller.is_exit_requested():
        # Check if pause has been requested
        controller.check_pause()

        # Send heartbeat
        result, _ = sender.run()
        if result:
            local_logger.info("Heartbeat sent")
        else:
            local_logger.error("Failed to send heartbeat")

        # Schedule next heartbeat exactly 1 second from the last scheduled time
        next_heartbeat_time += HEARTBEAT_PERIOD
        sleep_time = next_heartbeat_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

    local_logger.info("Heartbeat sender worker exiting")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import heartbeat_receiver
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_receiver_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process that receives heartbeat messages from the drone and tracks connection status.

    Parameters:
        connection: MAVLink connection to the drone.
        output_queue: Queue to output connection status strings ("connected" or "disconnected").
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
    # Instantiate class object (heartbeat_receiver.HeartbeatReceiver)
    result, receiver = heartbeat_receiver.HeartbeatReceiver.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create HeartbeatReceiver")
        return

    # Get Pylance to stop complaining
    assert receiver is not None

    local_logger.info("HeartbeatReceiver created successfully")

    # Main loop: do work.
    while not controller.is_exit_requested():
        # Check if pause has been requested
        controller.check_pause()

        # Receive heartbeat and check connection status
        result, status = receiver.run()

        if result:
            # Output the connection status to the queue
            output_queue.queue.put(status)
            local_logger.info(f"Connection status: {status}")

    local_logger.info("Heartbeat receiver worker exiting")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

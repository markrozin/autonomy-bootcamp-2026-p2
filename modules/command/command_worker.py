"""
Command worker to make decisions based on Telemetry Data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import command
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def command_worker(
    connection: mavutil.mavfile,
    target: command.Position,
    input_queue: queue_proxy_wrapper.QueueProxyWrapper,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process that makes decisions based on telemetry data and sends commands.

    Parameters:
        connection: MAVLink connection to the drone.
        target: Target position to face and match altitude.
        input_queue: Queue to receive TelemetryData from.
        output_queue: Queue to output command status strings.
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
    # Instantiate class object (command.Command)
    result, command_instance = command.Command.create(connection, target, local_logger)
    if not result:
        local_logger.error("Failed to create Command")
        return

    # Get Pylance to stop complaining
    assert command_instance is not None

    local_logger.info("Command created successfully")

    # Main loop: do work.
    while not controller.is_exit_requested():
        # Check if pause has been requested
        controller.check_pause()

        try:
            # Get telemetry data from input queue with timeout
            telemetry_data = input_queue.queue.get(timeout=0.5)

            # Check for sentinel value
            if telemetry_data is None:
                local_logger.info("Received sentinel value, exiting")
                break

            local_logger.info(f"Received telemetry data: {telemetry_data}")

            # Process the telemetry data and send commands
            result, status = command_instance.run(telemetry_data)

            if result and status is not None:
                # Output the command status to the queue
                output_queue.queue.put(status)
                local_logger.info(f"Command status sent to queue: {status}")

        except Exception:  # pylint: disable=broad-except
            # Queue.get with timeout raises Empty exception if no data
            continue

    local_logger.info("Command worker exiting")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

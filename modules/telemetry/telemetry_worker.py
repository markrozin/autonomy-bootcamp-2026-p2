"""
Telemtry worker that gathers GPS data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import telemetry
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process that gathers telemetry data (position and attitude) from the drone.

    Parameters:
        connection: MAVLink connection to the drone.
        output_queue: Queue to output TelemetryData objects.
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
    # Instantiate class object (telemetry.Telemetry)
    result, telemetry_instance = telemetry.Telemetry.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create Telemetry")
        return

    # Get Pylance to stop complaining
    assert telemetry_instance is not None

    local_logger.info("Telemetry created successfully")

    # Main loop: do work.
    while not controller.is_exit_requested():
        # Check if pause has been requested
        controller.check_pause()

        # Receive telemetry data
        result, telemetry_data = telemetry_instance.run()

        if result and telemetry_data is not None:
            # Output the telemetry data to the queue
            output_queue.queue.put(telemetry_data)
            local_logger.info(f"Telemetry data sent to queue: {telemetry_data}")

    local_logger.info("Telemetry worker exiting")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================

"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 5 — Step 2: Estimate Range and Approach
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

# -- Course setup: makes the shared `neo_lab` helper importable.
#    You don't need to read or change this block. --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

# -- Constants --------------------------------------------------------------
FOCAL_PX       = 320.0      # camera focal length in pixels (approx calibration)
REAL_TAG_SIZE  = 0.30       # physical corner-tag side length, meters (approx)
COL_CENTER     = 320
STOP_DIST      = 2.0        # meters: close enough
APPROACH_PITCH = 0.15
MAX_YAW        = 0.2
SEARCH_PITCH   = 0.1        # creep forward; ArUco tags only resolve up close

# -- Module-level state -----------------------------------------------------
_done = False

def reset():
    global _done
    _done = False


def update(drone):
    global _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: fly toward the gate, estimating distance from its apparent tag size, and stop
    # once distance <= STOP_DIST.
    #
    # Tools: drone.camera.get_color_image(); neo_lab.detect_gate(image) -> Gate or None;
    #        a Gate has .cx (center column) and .tag_px (tag size in px); uav_utils.clamp(...);
    #        drone.flight.send_pcmd(pitch, roll, yaw, throttle), drone.flight.stop().
    #
    # No gate -> creep forward (SEARCH_PITCH) until tags resolve. With a gate, invert the
    # Module 1 projection: distance = FOCAL_PX * REAL_TAG_SIZE / tag_px. Yaw to keep .cx on
    # COL_CENTER and add APPROACH_PITCH forward. Stop and finish once distance <= STOP_DIST.

    image = drone.camera.get_color_image()
    gate = neo_lab.largest_cyan_gate(image, MIN_AREA)
    if gate is None:
        drone.flight.send_pcmd(0.0, 0.0, SEARCH_YAW, 0.0)
        return False
    x, y, w, h = cv2.boundingRect(gate)
    distance = (FOCAL_PX * REAL_GATE_WIDTH) / w
    err = (x + w / 2.0 - COL_CENTER) / COL_CENTER
    yaw = uav_utils.clamp(err * MAX_YAW, -MAX_YAW, MAX_YAW)
    if distance <= STOP_DIST:
        drone.flight.stop()
        print(f"Gate reached, distance: {distance:.2f} m")
        _done = True
    else:
        drone.flight.send_pcmd(APPROACH_PITCH, 0.0, yaw, 0.0)

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Estimate Range and Approach")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()

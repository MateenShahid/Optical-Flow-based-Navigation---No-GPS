import sys
sys.path.append('c:\\users\\mmsha\\anaconda3\\lib\\site-packages')

import time
from pymavlink import mavutil

alt = 10

def main():
    # Create a connection to the autopilot
    master = mavutil.mavlink_connection("com9", baud=57600)
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (
        master.target_system, master.target_component))

    # Arm and takeoff
    arm_and_takeoff(master, alt)
    time.sleep(5)
    #
    # Waypoints (North, East, Down)
    waypoints = [
        # (North, East, Down) 
        (-60, -40, -alt), # To go 10m in North 
        (-100, 0, -alt), # To go towards East from current Positon)
        (-60, 40, -alt), # To go to 10 meter East from Initial position
        (0, 0, -alt)    # To go to initial position

        # (-50, 87, -alt),
        # (0, 0, -alt)
    ]
    #
    # Navigate to waypoints
    for waypoint in waypoints:
        navigate_to(master, waypoint)
        # time.sleep(5)
    # time.sleep(5)

    # # Land
    # master.mav.command_long_send(
    #     master.target_system,
    #     master.target_component,
    #     mavutil.mavlink.MAV_CMD_NAV_LAND,
    #     0, 0, 0, 0, 0, 0, 0, 0)


def arm_and_takeoff(master, target_altitude):
    # Set mode to GUIDED
    master.set_mode('GUIDED')

    # Arm the vehicle
    master.arducopter_arm()
    time.sleep(5)
    # master.arducopter_disarm()

    # Send takeoff command
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)

    ## Wait for the drone to reach the target altitude
    # while True:
    #     altitude = master.messages['GLOBAL_POSITION_INT'].alt / 1000
    #     if altitude >= target_altitude * 0.95:
    #         break
    time.sleep(5)


def navigate_to(master, waypoint):
    north, east, down = waypoint
    set_position_target_local_ned(master, north, east, down)
    time.sleep(10) ####


def set_position_target_local_ned(master, north, east, down):
    # Set type mask to enable position control (disable velocity and acceleration control)
    # type_mask = 0b000011111000 #OLD MASK

    # Create target position message
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        3576,  # type_mask
        north,  # X (north)
        east,  # Y (east)
        down,  # Z (down)
        0, 0, 0,  # vx, vy, vz (not used)
        0, 0, 0,  # afx, afy, afz (not used)
        0,  # yaw (not used)
        0  # yaw_rate (not used)
    )

if __name__ == '__main__':
    main()

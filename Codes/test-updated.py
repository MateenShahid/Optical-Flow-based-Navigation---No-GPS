import sys
sys.path.append('c:\\users\\mmsha\\anaconda3\\lib\\site-packages')

import time
from pymavlink import mavutil

alt = 10

def main():
    # Create a connection to the autopilot
    master = mavutil.mavlink_connection("com4", baud=115200)
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (
        master.target_system, master.target_component))

    # Set EKF origin manually (latitude, longitude, altitude in degrees and meters)
     ###########################
    lat = 31.4687054 
    long = 74.4121069
    alt_h = 0
    set_ekf_origin(master, lat, long, alt_h)
    ###########################
    # set_ekf_origin(master, 42.12345, -73.54321, alt)

    # Set AHRS_EKF_TYPE to EKF2 (2: Use EKF2 estimator without GPS)
    set_ekf_type(master, 3)

    # Arm and takeoff
    print("Arming!!")
    arm_and_takeoff(master, alt)
    time.sleep(5)

    # Waypoints (North, East, Down)
    waypoints = [
        # (North, East, Down) 
        (-60, -40, -alt), # To go 10m in North 
        (-100, 0, -alt),  # To go towards East from current Positon)
        (-60, 40, -alt),  # To go to 10 meters East from Initial position
        (0, 0, -alt)      # To go to initial position
    ]
    
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




def set_ekf_origin(master, latitude, longitude, altitude):
    # Send MAV_CMD_SET_HOME_POSITION command to set the EKF origin
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,                  # Confirmation
        0,                  # Use current position (1 to set, 0 to use specified values)
        0,                  # 1: Use current altitude, 0: Use specified altitude
        0,                  # 1: Use current direction, 0: Use specified direction
        0,
        latitude,           # Latitude (in degrees)
        longitude,          # Longitude (in degrees)
        altitude            # Altitude (in meters)
    )
    print('Home Location:', latitude, longitude, altitude)
    time.sleep(2)  # Allow some time for the EKF origin to be set

def set_ekf_type(master, ekf_type):
    # Send MAV_CMD_PARAM_SET command to set AHRS_EKF_TYPE parameter
    param_id = "AHRS_EKF_TYPE"
    param_value = ekf_type
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8
    )

    time.sleep(2)  # Allow some time for the parameter to be set

def arm_and_takeoff(master, target_altitude):
    # Set mode to GUIDED
    master.set_mode('GUIDED')

    # Arm the vehicle
    master.arducopter_arm()
    time.sleep(5)

    # Send takeoff command
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)

    time.sleep(5)

def navigate_to(master, waypoint):
    north, east, down = waypoint
    set_position_target_local_ned(master, north, east, down)
    time.sleep(10)

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

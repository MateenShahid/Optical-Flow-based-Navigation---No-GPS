import sys
sys.path.append('c:\\users\\mmsha\\anaconda3\\lib\\site-packages')

# from builtins import *
from pymavlink import mavutil
import time
import dronekit
from dronekit import connect, VehicleMode
#############################
# Connect to SITL simulator
# master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
master = mavutil.mavlink_connection("com9", baud=57600)
master.wait_heartbeat()

print('\nConnection to SITL established!!!!')
# time.sleep(5)

print(master.target_system)

alt = 5

# Arm the drone and take off to a height of 50 meters
master.arducopter_arm()
print('\nArming the drone!!')

master.mav.command_long_send(
    master.target_system,           # target system ID
    mavutil.mavlink.MAV_COMP_ID_USER1, # target component ID
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command ID
    0, 0, 0, 0, 0, 0, 0, alt)        # parameters: lat, lon, alt, yaw, pitch, roll, vx, vy, vz, delay
time.sleep(5) # Wait for the drone to take off

print('\nTaking off!!')

# Move forward 20 meters in the x-axis
master.mav.set_position_target_local_ned_send(
    0,            # time_boot_ms (not used)
    master.target_system,  # target system ID
    mavutil.mavlink.MAV_COMP_ID_USER1, # target component ID
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame ID
    0b0000111111000111,   # bitmask for position and velocity only
    5, 0, -alt,   # x, y, and z positions (relative to the home position)
     0, 0, 0,   # x, y, and z velocity
     0, 0, 0,   # x, y, and z acceleration
     0, 0)      # yaw and yaw rate (not used)

time.sleep(10)

# Return to the home position
master.mav.set_position_target_local_ned_send(
    0,            # time_boot_ms (not used)
    master.target_system,  # target system ID
    mavutil.mavlink.MAV_COMP_ID_USER1, # target component ID
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame ID
    0b0000111111000111,   # bitmask for position and velocity only
    0, 0, -alt,   # x, y, and z positions (relative to the home position)
     0, 0, 0,   # x, y, and z velocity
     0, 0, 0,   # x, y, and z acceleration
     0, 0)      # yaw and yaw rate (not used)

print('\nRetuning to Launch!!')

# Disarm the drone
master.arducopter_disarm()
###################################################################





# from dronekit import connect, VehicleMode
# import time

# # Connect to the SITL simulator
# connection_string = "udp:127.0.0.1:14550"
# print(f"Connecting to vehicle on: {connection_string}")
# vehicle = connect(connection_string, wait_ready=True)
# print(connection_string.target_system)

# # Get some vehicle attributes
# print(f"Get some vehicle attribute values:")
# print(f" GPS: {vehicle.gps_0}")
# print(f" Battery: {vehicle.battery}")
# print(f" Last Heartbeat: {vehicle.last_heartbeat}")
# print(f" Is Armable?: {vehicle.is_armable}")
# print(f" System status: {vehicle.system_status.state}")
# print(f" Mode: {vehicle.mode.name}")

# # Change vehicle mode to "GUIDED"
# vehicle.mode = VehicleMode("GUIDED")
# while not vehicle.mode.name == "GUIDED":
#     print(f" Waiting for mode change ...")
#     time.sleep(1)

# print(f"Mode changed to {vehicle.mode.name}")

# # Close vehicle object before exiting script
# vehicle.close()
# print("Vehicle object closed.")


#################################################

# from pymavlink import mavutil
# import time

# # Connect to the SITL simulator
# master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
# print('Connected!!!!')

# # Arm the drone and take off to a height of 5 meters
# master.arducopter_arm()
# print('Arming!!!!')
# master.mav.command_long_send(
#     master.target_system, master.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#     0, 0, 0, 0, 0, 0, 0, 5)
# print('Taking off!!!!')

# # Wait for the drone to reach a height of 5 meters
# while True:
#     msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#     if msg.alt / 1000.0 > 4.5:
#         break
#     time.sleep(0.1)

# # Fly forward at a speed of 5 meters/second for 10 seconds
# master.mav.command_long_send(
#     master.target_system, master.target_component,
#     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
#     0, 5, 0, 0, 0, 0, 10, 0)
# print('Going Straight!!!!')

# # Land the drone
# master.mav.command_long_send(
#     master.target_system, master.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_LAND,
#     0, 0, 0, 0, 0, 0, 0, 0)
# print('Landing!!!!')

# # Wait for the drone to land
# while True:
#     msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#     if msg.alt == 0:
#         break
#     time.sleep(0.1)

# # Disarm the drone
# master.arducopter_disarm()


#############################################
# connection_string = 'tcp:127.0.0.1:5760'
# vehicle = connect(connection_string, wait_ready=True)
# print('Connected!!!!')

# vehicle.mode = VehicleMode("GUIDED")
# vehicle.armed = True
# vehicle.simple_takeoff(10)
# print('Taking Off!!!')

# vehicle.close()

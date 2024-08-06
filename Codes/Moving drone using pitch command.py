import sys
sys.path.append('c:\\users\\mmsha\\anaconda3\\lib\\site-packages')

################################################# Moving based on Pitch Command ##########################
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Connect to the ArduPilot drone using com9
vehicle = connect('com9', wait_ready=True)

alt_thresh = 5  ## Define the Target Altitude

# Arm and takeoff the drone
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
vehicle.simple_takeoff(alt_thresh)

# Wait for the drone to reach the desired altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= alt_thresh * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

# Move the drone forward using the Pitch command
pitch = 0.5
msg = vehicle.message_factory.set_attitude_target_encode(
    0,   # Time boot ms
    1,   # Target system
    1,   # Target component
    0b00000000, #Type mask: bit 1 is LSB, indiates attitude only
    mavutil.mavlink_euler_to_quaternion(0, pitch, 0),  #Quaternion
    0,  # Body roll rate in radian
    0,  # Body Pitch rate in radian
    0,  # Body yaw rate in radian
    0b00000100  # Mask for ignoring body rates
)
vehicle.send_mavlink(msg)
vehicle.flush()

# Wait for 5 seconds before stopping the drone
time.sleep(5)

################
# Set the Pitch command to stop the drone
pitch = 0 # Set the pitch to 0 to stop the drone
msg = vehicle.message_factory.set_attitude_target_encode(
    0, # Time boot ms
    1, # Target system
    1, # Target component
    0b00000000, # Type mask: bit 1 is LSB, indicates attitude only
    mavutil.mavlink_euler_to_quaternion(0, pitch, 0), # Quaternion
    0, # Body roll rate in radian
    0, # Body pitch rate in radian
    0, # Body yaw rate in radian
    0b00000100 # Mask for ignoring body rates
)
vehicle.send_mavlink(msg)
vehicle.flush()
################

# Stop the drone and disconnect
vehicle.mode = VehicleMode("LAND")
time.sleep(5)
vehicle.armed = False
vehicle.disconnect()

#####################
# Close the connection
vehicle.close()
#####################

#################################################

################################################# Moving based on Velocity Values ##########################
# from dronekit import connect, VehicleMode
# import time

# # Connect to the ArduPilot drone using com9
# vehicle = connect('com9', wait_ready=True)

# alt_thresh = 5 ## Define the target altitude.

# # Arm and takeoff the drone
# vehicle.mode = VehicleMode("GUIDED")
# vehicle.armed = True
# vehicle.simple_takeoff(alt_thresh)

# # Wait for the drone to reach the desired altitude
# while True:
#     print("Altitude: ", vehicle.location.global_relative_frame.alt)
#     if vehicle.location.global_relative_frame.alt >= alt_thresh * 0.95:
#         print("Reached target altitude")
#         break
#     time.sleep(1)

# # Move the drone forward using set_velocity_body()
# vx = 1 # The forward velocity in m/s
# vy = 0 # The lateral velocity in m/s
# vz = 0 # The vertical velocity in m/s
# duration = 5 # The duration of the movement in seconds
# msg = vehicle.message_factory.set_velocity_body_encode(
#     0, # Timestamp in milliseconds, or 0 for unknown
#     vx, # Velocity in X in meters/second
#     vy, # Velocity in Y in meters/second
#     vz, # Velocity in Z in meters/second
#     0  # Heading in radians, or 0 for unknown
# )
# vehicle.send_mavlink(msg)
# vehicle.flush()
# time.sleep(duration)

# # Stop the drone and disconnect
# vehicle.mode = VehicleMode("LAND")
# time.sleep(5)
# vehicle.armed = False
# vehicle.disconnect()
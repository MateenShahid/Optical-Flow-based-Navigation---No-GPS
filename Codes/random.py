import sys
sys.path.append('c:\\users\\mmsha\\anaconda3\\lib\\site-packages')
from dronekit import connect, VehicleMode
import time

connection_string = 'tcp:127.0.0.1:5760'
vehicle = connect(connection_string, wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
vehicle.simple_takeoff(10)

while True:
    print("Position: ", vehicle.location.global_frame)
    time.sleep(1)
    vehicle.close()

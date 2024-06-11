from pymavlink import mavutil
import time

# Connect to the Pixhawk via serial port
connection = mavutil.mavlink_connection('COM7', baud=115200)

# Wait for the heartbeat from Pixhawk
connection.wait_heartbeat()
print("Heartbeat received")

# Function to send arm command
def arm_vehicle():
    # Create the command to arm the vehicle
    # connection.mav.command_long_send(
    #     connection.target_system, 
    #     connection.target_component,
    #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    #     0,
    #     1, 0, 0, 0, 0, 0, 0
    # )
    connection.arducopter_arm()
    connection.arducopter_arm()
    # connection.arducopter_arm()
    # Wait until the vehicle is armed
    msg = connection.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)
    connection.motors_armed_wait()
    print("Vehicle armed")

# Function to send disarm command
def disarm_vehicle():
    # Create the command to disarm the vehicle
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    # Wait until the vehicle is disarmed
    connection.motors_disarmed_wait()
    print("Vehicle disarmed")

# Function to send manual control commands
def send_manual_control(x, y, z, r):
    
    connection.mav.manual_control_send(
        connection.target_system,
        x, #-1000 - 1000
        y, #-1000 - 1000
        z, # 0 - 1000
        r, #-1000 - 1000
        0)

# Example usage: arming the vehicle and running the thruster
try:
    arm_vehicle()
    
    print("hore")
    while True:
        # Run thruster at half power forward (adjust the values as needed)
        send_manual_control(500, 0, 500, 0)
        print("test")
        time.sleep(1)
        
        # Optionally, stop the thruster
        # send_manual_control(0, 0, 500, 0)
        # time.sleep(1)

except KeyboardInterrupt:
    print("Manual control interrupted by user")

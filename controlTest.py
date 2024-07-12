import time
from pymavlink import mavutil

# Connect to the Pixhawk via serial port
connection = mavutil.mavlink_connection('COM7', baud=115200)

# Wait for the heartbeat from Pixhawk
connection.wait_heartbeat()
print("Heartbeat received")

# Function to send arm command
def arm_vehicle():
    connection.arducopter_arm()
    connection.arducopter_arm()
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    connection.motors_armed_wait()
    print("Vehicle armed")

# Function to send disarm command
def disarm_vehicle():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    connection.motors_disarmed_wait()
    print("Vehicle disarmed")

# Function to send manual control commands
def send_manual_control(x, y, z, r):
    connection.mav.manual_control_send(
        connection.target_system,
        x,  # -1000 to 1000
        y,  # -1000 to 1000
        z,  # 0 to 1000
        r,  # -1000 to 1000
        0
    )

# Example usage: arming the vehicle and running the thruster
try:
    arm_vehicle()

    z_value = 200  # Initial throttle value (adjust as needed)

    # # Timing control
    # start_time = time.time()
    
    # while True:
    #     elapsed_time = time.time() - start_time

    #     if elapsed_time < 3:
    #          send_manual_control(0, 0, z_value, 0)
    #     elif elapsed_time < 8:
    #         # Send x = -500 for the first 5 seconds
    #         send_manual_control(-500, 0, z_value, 0)
    #         print(f"Elapsed Time: {elapsed_time:.2f} - Sending x = -500")
    #     elif elapsed_time < 10:
    #         # Send y = -500 for the next 2 seconds
    #         send_manual_control(0, -500, z_value, 0)
    #         print(f"Elapsed Time: {elapsed_time:.2f} - Sending y = -500")
    #     elif elapsed_time < 13:
    #         # Send r = 700 for the next 3 seconds
    #         send_manual_control(0, 0, z_value, 700)
    #         print(f"Elapsed Time: {elapsed_time:.2f} - Sending r = 700")
    #     else:
    #         # Stop sending commands after 10 seconds
    #         send_manual_control(0, 0, 0, 0)
    #         print("Elapsed Time: 10.00 - Stopping manual control commands")
    #         break

    #     time.sleep(0.1)  # Adjust sleep time for smoother control




except KeyboardInterrupt:
    print("Manual control interrupted by user")


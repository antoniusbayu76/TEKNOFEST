from pymavlink import mavutil

fullVoltage = 16.80

def get_battery_status():
    # Connect to the drone
    master = mavutil.mavlink_connection('COM7',baud=115200)

    # Wait for the heartbeat message to find the system ID
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    # Request the battery status message
    while True:
        # Wait for a message
        message = master.recv_match(type='BATTERY_STATUS', blocking=True)
        
        if message is not None:
            # Print the entire BATTERY_STATUS message for diagnostics
            print(message)
            
            voltages = message.voltages  # battery voltages in millivolts
            current_battery = message.current_battery  # battery current in 10*milliamperes (1 = 10 mA)
            battery_remaining = message.battery_remaining  # Remaining battery energy in percentage (0-100)
            
            # Print the voltages of each cell
            print("Battery Cell Voltages (in volts):")
            for i, voltage in enumerate(voltages):
                if voltage == 65535:
                    break  # Stop at the first invalid value
                print(f" Cell {i+1}: {voltage / 1000.0:.2f} V")
                percent = (voltage / 1000.0/fullVoltage)*80
                print("Battery Remaining: ", round(percent,2), "%")

            
            # Print battery current and remaining percentage
            current_amperes = current_battery / 10.0
            if current_battery == -1:
                print("Current Invalid")
            else :
                print(f"Battery Current: {current_amperes:.2f} A")
            
            
            
            
            break  # Exit after receiving the battery status

if __name__ == '__main__':
    get_battery_status()

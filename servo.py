from pymavlink import mavutil
import cv2
import time

# Hubungkan ke perangkat MAVLink
connection = mavutil.mavlink_connection('COM7', baud=115200)

# Tunggu hingga koneksi terjalin
connection.wait_heartbeat()

# Fungsi untuk menggerakkan continuous servo
def move_continuous_servo(channel, pwm_value):
    # Kirim perintah servo output raw
    connection.mav.command_long_send(
        connection.target_system,  # target_system
        connection.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        channel,  # servo number (servo channel)
        pwm_value,  # pwm value (1000 to 2000 microseconds)
        0, 0, 0, 0, 0)  # unused parameters

# Contoh untuk menggerakkan continuous servo di channel AUX 1 (yang biasanya adalah channel 9 pada banyak perangkat)
# Nilai di bawah 1500 membuat servo berputar ke satu arah
# move_continuous_servo(9, 1300)

# # Tunggu beberapa saat agar servo dapat berputar
# time.sleep(5)
try :
    while True :
        # move_continuous_servo(9,1300)
        move_continuous_servo(11,1900)
        print("1300")
        time.sleep(1)

except KeyboardInterrupt :
# Berhentikan servo dengan mengirimkan nilai tengah
    # move_continuous_servo(9, 1500)
    move_continuous_servo(11, 1500)
    print("stop")

# # Tunggu beberapa saat agar servo berhenti
# time.sleep(1)

# # Nilai di atas 1500 membuat servo berputar ke arah berlawanan
# move_continuous_servo(9, 1700)

# # Tunggu beberapa saat agar servo dapat berputar
# time.sleep(5)

# # Berhentikan servo dengan mengirimkan nilai tengah
# move_continuous_servo(9, 1500)

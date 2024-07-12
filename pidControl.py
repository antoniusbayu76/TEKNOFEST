from pymavlink import mavutil
import time

port = 'COM7'
baudrate = 115200
# Connect to the Pixhawk via serial port
connection = mavutil.mavlink_connection(port, baudrate)

Kp = 0.1
Ki = 0.1
Kd = 0.1

nyelam = 0 #state nyelam

xPrev = 0
yPrev = 0
zPrev = 0

prevTime = 0
integral_errx = 0
integral_erry = 0
integral_errz = 0

# Wait for the heartbeat from Pixhawk
connection.wait_heartbeat()
print("Heartbeat received")

# Function to send arm command
def arm_vehicle():
    connection.arducopter_arm()
    connection.arducopter_arm()
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

def pidSys(area,cx,cy,setx,sety,setz,integralx,integraly,integralz,xPrev,yPrev,zPrev,dt):
    x = area
    y = cx
    z = cy

    ex = setx - x
    ey = sety - y
    ez = setz - z

    integralx += ex * dt
    derivativex = (ex - xPrev) / dt
    ux = Kp * ex + Ki * integralx + Kd * derivativex

    integraly += ey * dt
    derivativey = (ey - yPrev) / dt
    uy = Kp * ey + Ki * integraly + Kd * derivativey

    integralz += ez * dt
    derivativez = (ez - zPrev) / dt
    uz = Kp * ez + Ki * integralz + Kd * derivativez

    
    return ux, integralx, ex, uy, integraly, ey, uz, integralz, ez

def mapping(num, inMin, inMax, outMin, outMax):
  return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax
                  - outMin))



# Function to send manual control commands
def send_manual_control(area, cx, cy, setx, sety, setz, minArea, maxArea, minFramex, maxFramex, minFramey, maxFramey):
    global curTime, prevTime, xPrev, yPrev, zPrev, integral_errx, integral_erry, integral_errz
    curTime = time.time()

    dt = curTime - prevTime

    ux, integralx, ex, uy, integraly, ey, uz, integralz, ez = pidSys(area,cx,cy, setx, sety, setz, integral_errx,integral_erry,integral_errz, xPrev, yPrev, zPrev, dt)

    integral_errx = integralx
    xPrev= ex

    integral_erry = integraly
    yPrev = ey

    integral_errz = integralz
    zPrev = ez

    prevTime = curTime

    ux = mapping(ux,minArea,maxArea,-1000,1000)
    uy = mapping(uy,minFramex,maxFramex,-1000,1000)    
    uz = mapping(uz,minFramey,maxFramey,0,1000)    

    connection.mav.manual_control_send(
        connection.target_system,
        ux, #-1000 - 1000
        uy, #-1000 - 1000
        nyelam + uz, # 0 - 1000
        0, #-1000 - 1000
        0)
    
    
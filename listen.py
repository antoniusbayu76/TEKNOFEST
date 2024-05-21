from pymavlink import mavutil

theConnection = mavutil.mavlink_connection("COM7", baud=115200)

theConnection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)"%
      (theConnection.target_system,theConnection.target_component))

theConnection.mav.command_long_send(theConnection.target_system, theConnection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
# theConnection.mav.command_long_send(theConnection.target_system, theConnection.target_component,
#                                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,10)
# theConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,theConnection.target_system,
#                                                                                      theConnection.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,int(0b100111111000),10,0,-10,0,0,0,0,0,0,0,0))


# msg = theConnection.recv_match(blocking= True)
msg = theConnection.recv_match(type='COMMAND_ACK',blocking=True)
# msg = theConnection.recv_match(type='LOCAL_PISITION_NED',blocking=True)
print(msg)
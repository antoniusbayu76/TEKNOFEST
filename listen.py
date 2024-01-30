from pymavlink import mavutil

theConnection = mavutil.mavlink_connection('udpin:localhost:14551')

theConnection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)"%
      (theConnection.target_system,theConnection.target.component))

theConnection.mav.command_long_send(theConnection.target_system, theConnection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
theConnection.mav.command_long_send(theConnection.target_system, theConnection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,10)


#     msg = theConnection.recv_match(blocking= True)
msg = theConnection.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)
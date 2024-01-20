from pymavlink import mavutil

theConnection = mavutil.mavlink_connection('udpin:localhost:14551')

theConnection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)"%
      (theConnection.target_system,theConnection.target.component))
from pymavlink import mavutil

# Example for UDP connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the first heartbeat 
master.wait_heartbeat()

MAVLink = dict()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue
    print("Received message: %s" % msg)
    print("Received message get_type: %s" % msg.get_type())


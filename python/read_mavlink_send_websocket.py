# Purpose : Read  MAVLink data and send it to Grafana  by using WebSocket
# Created : 2025/05/26
# To Run :  clear;python3  .\src\read_mavlink_send_websocket.py

import asyncio
import websockets
import json
from pymavlink import mavutil
import time

# MAVLink connection string (adjust as needed)
MAVLINK_URL = 'udp:0.0.0.0:14550'  # Change for your setup

# WebSocket server settings
WS_HOST = '0.0.0.0'
WS_PORT = 8765

# Declare Dictionaries
MAVLink = dict()
SCALED_IMU = dict()
HEARTBEAT= dict()
LOCAL_POSITION_NED = dict()
ATTITUDE = dict()
RC_CHANNELS = dict()



def get_utc_timestamp_millis():
    return int(time.time() * 1000)

async def stream_mavlink(websocket):
    """
    WebSocket handler that streams MAVLink messages to connected clients.
    """
    print(f"Grafana or client connected: {websocket.remote_address}")
    master = mavutil.mavlink_connection(MAVLINK_URL)
    print("Waiting for MAVLink heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received from system %u component %u" % (master.target_system, master.target_component))
    try:
        while True:
            msg = master.recv_match(blocking=True)
            if msg and msg.get_type() != "BAD_DATA":
                if msg.get_type() == "SCALED_IMU":
                    SCALED_IMU = msg.to_dict()
                    
                    # Assign UTC Timestamp to  IMU  value
                    SCALED_IMU["time_boot_ms"] = get_utc_timestamp_millis()
                    MAVLink["SCALED_IMU"] = SCALED_IMU

                if msg.get_type() == "HEARTBEAT":
                    HEARTBEAT = msg.to_dict()
                    MAVLink["HEARTBEAT"] = HEARTBEAT

                if msg.get_type() == "RC_CHANNELS":
                    RC_CHANNELS = msg.to_dict()
                    MAVLink["RC_CHANNELS"] = RC_CHANNELS

                print(MAVLink)
                await websocket.send(json.dumps(MAVLink))
            await asyncio.sleep(0.5)  # Limit send rate
    except websockets.ConnectionClosed:
        print("Client disconnected from WebSocket.")

async def main():
    print(f"Starting WebSocket server at ws://{WS_HOST}:{WS_PORT}")
    async with websockets.serve(stream_mavlink, WS_HOST, WS_PORT):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())

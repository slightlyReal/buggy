#ifndef CONF_H
#define CONF_H

#define MAVLINK_BAUD 2000000

#define LED_GREEN 16
#define LED_RED 17

#define LOCAL_IP 192, 168, 137, 5
#define SUBNET 255, 255, 255, 0
#define GATEWAY 192, 168, 137, 1
#define LOCAL_PORT 14550

#define GROUNGSTATION_IP "192.168.137.255"
#define GROUNDSTATION_PORT 14550

bool armed = false;

int16_t system_mode = MAV_MODE_PREFLIGHT;
int16_t control_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

unsigned long previousHeartbeatMillis = 0;
unsigned long previousTimeoutMillis = 0;

const int16_t system_id = 1;
const int16_t component_id = 1;

const unsigned long heartbeat_interval = 50;
const unsigned long timeout_interval = 5000;

const char* groundstation_host = GROUNGSTATION_IP;
const uint16_t groundstation_port = GROUNDSTATION_PORT;
const uint16_t udp_mavlink_port = LOCAL_PORT;

IPAddress ip(LOCAL_IP);
IPAddress subnet(SUBNET);
IPAddress gateway(GATEWAY);

WiFiUDP mavlink_udp;

#endif /* MATH_CONSTANTS_H */
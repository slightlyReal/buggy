void init_wifi()
{
  if (!WiFi.config(ip, gateway, subnet))
  {
    Serial.println("Static IP configuration failed!");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\nConnecting to ");
  Serial.print(WIFI_SSID);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(500);
  }

  Serial.print("\nConnected with IP address: ");
  Serial.println(WiFi.localIP());

  mavlink_udp.begin(udp_mavlink_port);
}

#include <sys/time.h>
#include <stdint.h>
// Returns UTC timestamp in milliseconds
uint64_t getUTCTimestampMillis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
}


void send_mavlink(mavlink_message_t *mavlink_message)
{
  uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, mavlink_message);

  mavlink_udp.beginPacket(groundstation_host, groundstation_port);
  mavlink_udp.write(mavlink_message_buffer, mavlink_message_length);
  mavlink_udp.endPacket();

  Serial.write(mavlink_message_buffer, mavlink_message_length);
}

void send_heartbeat()
{
  mavlink_message_t mvl_tx_message;

  const int16_t mavlink_type = MAV_TYPE_GROUND_ROVER;
  const int16_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  uint8_t system_status = MAV_STATE_ACTIVE;
  uint32_t custom_mode = control_mode;

  if (armed)
  {
    base_mode |= MAV_MODE_MANUAL_ARMED;
  }
  else
  {
    base_mode |= MAV_MODE_MANUAL_DISARMED;
  }

  mavlink_msg_heartbeat_pack(system_id, component_id, &mvl_tx_message, mavlink_type, autopilot_type, base_mode, custom_mode, system_status);
  send_mavlink(&mvl_tx_message);
}

void send_systemstatus()
{
  mavlink_sys_status_t sys_status;
  mavlink_message_t mvl_tx_message;

  sys_status.onboard_control_sensors_present = 0;
  sys_status.onboard_control_sensors_enabled = 0;
  sys_status.onboard_control_sensors_health = 0;

  // generate random values for system status
  sys_status.load = random(500, 600);
  sys_status.voltage_battery = random(11000, 13000);
  sys_status.current_battery = random(1, 3);
  sys_status.battery_remaining = random(80, 100);

  sys_status.drop_rate_comm = 0;
  sys_status.errors_comm = 0;
  sys_status.errors_count1 = 0;
  sys_status.errors_count2 = 0;
  sys_status.errors_count3 = 0;
  sys_status.errors_count4 = 0;

  mavlink_msg_sys_status_encode(system_id, component_id, &mvl_tx_message, &sys_status);
  send_mavlink(&mvl_tx_message);
}

void send_radiostatus()
{
  mavlink_radio_status_t radio_status;
  mavlink_message_t mvl_tx_message;

  radio_status.remrssi = WiFi.RSSI();
  radio_status.rssi = WiFi.RSSI();

  mavlink_msg_radio_status_encode(system_id, component_id, &mvl_tx_message, &radio_status);
  send_mavlink(&mvl_tx_message);
}

void send_position()
{
  mavlink_global_position_int_t position;
  mavlink_message_t mvl_tx_message;

  // generate spoofed coordinates
  position.time_boot_ms = millis();
  position.lat = random(557515000, 557515100);
  position.lon = random(376158000, 376158100);
  position.alt = random(602900, 602950);
  position.hdg = random(0, 36000);

  mavlink_msg_global_position_int_encode(system_id, component_id, &mvl_tx_message, &position);
  send_mavlink(&mvl_tx_message);
}

void send_mavlink_msg_rc_channels(uint16_t elapsed_us)
{

  ESP_LOGI("LOGGER_TAG", "UDP transmission took %d microseconds", static_cast<uint16_t>(elapsed_us)); // Warning level

  mavlink_rc_channels_t rc_channels = {0};
  mavlink_message_t mvl_tx_message;

  // generate spoofed coordinates
  rc_channels.time_boot_ms = millis();
  rc_channels.chancount = 10;
  rc_channels.chan1_raw = random(1000, 1100);
  rc_channels.chan2_raw = random(1100, 1200);
  rc_channels.chan3_raw = random(1200, 1300);
  rc_channels.chan4_raw = random(1300, 1400);
  rc_channels.chan5_raw = random(1400, 1500);
  rc_channels.chan6_raw = elapsed_us;

  mavlink_msg_rc_channels_encode(system_id, component_id, &mvl_tx_message, &rc_channels);
  send_mavlink(&mvl_tx_message);
}

void send_mavlink_attitude(uint32_t time_boot_ms, float roll, float pitch, float yaw)
{
  // https://mavlink.io/en/messages/common.html#ATTITUDE
  ESP_LOGI("LOGGER_TAG", "time_boot_ms : %d,roll %.2f,pitch %.2f,yaw: %.2f took %d microseconds", time_boot_ms, roll, pitch, yaw); // Warning level

  mavlink_attitude_t attitude = {0};
  mavlink_message_t mvl_tx_message;

  attitude.time_boot_ms = millis();
  attitude.roll = roll;
  attitude.pitch = pitch;
  attitude.yaw = yaw;

  mavlink_msg_attitude_encode(system_id, component_id, &mvl_tx_message, &attitude);
  send_mavlink(&mvl_tx_message);
}

void send_mavlink_local_position_ned(uint32_t time_boot_ms, float x, float y, float z)
{
  // https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED
  ESP_LOGI("LOGGER_TAG", "time_boot_ms : %d,North[m] %.2f,East[m] %.2f,Down[m] %.2f ", time_boot_ms, x, y, z); // Info level

  mavlink_local_position_ned_t local_position_ned = {0};
  mavlink_message_t mvl_tx_message;

  // generate spoofed coordinates
  local_position_ned.time_boot_ms = millis();
  local_position_ned.x = x ;
  local_position_ned.y = y ;
  local_position_ned.z = z ;

  mavlink_msg_local_position_ned_encode(system_id, component_id, &mvl_tx_message, &local_position_ned);
  send_mavlink(&mvl_tx_message);
}
 

void send_mavlink_SCALED_IMU(uint32_t time_boot_ms, int16_t xacc, int16_t xgyro, int16_t xmag)
{
  // https://mavlink.io/en/messages/common.html#SCALED_IMU
  ESP_LOGI("LOGGER_TAG", "time_boot_ms : %d,xacc %.2f,xgyro %.2f,xmag: %.2f", time_boot_ms, xacc, xgyro, xmag); // Warning level

  // Zeroize message
  mavlink_scaled_imu_t scaled_imu = {0};
  mavlink_message_t mvl_tx_message;

  // generate spoofed coordinates
  scaled_imu.time_boot_ms = millis();
  scaled_imu.xacc = xacc;     /*< [mG] X acceleration*/
  scaled_imu.yacc = xacc * 2; /*< [mG] Y acceleration*/
  scaled_imu.zacc = xacc * 3; /*< [mG] Z acceleration*/

  scaled_imu.xgyro = xgyro;     /*< [mrad/s] Angular speed around X axis*/
  scaled_imu.ygyro = xgyro * 2; /*< [mrad/s] Angular speed around Y axis*/
  scaled_imu.zgyro = xgyro * 3; /*< [mrad/s] Angular speed around Z axis*/

  scaled_imu.xmag = xmag;     /*< [mT] X Magnetic field*/
  scaled_imu.ymag = xmag * 2; /*< [mT] Y Magnetic field*/
  scaled_imu.zmag = xmag * 3; /*< [mT] Z Magnetic field*/
  // scaled_imu.temperature = random(20, 25);

  mavlink_msg_scaled_imu_encode(system_id, component_id, &mvl_tx_message, &scaled_imu);
  send_mavlink(&mvl_tx_message);
}

void handle_message_command_long(mavlink_message_t *msg)
{
  mavlink_command_long_t command_long;
  mavlink_command_ack_t mvl_command_ack;
  mavlink_message_t mvl_tx_message;

  mvl_command_ack.result = MAV_RESULT_FAILED;
  uint8_t result = MAV_RESULT_UNSUPPORTED;

  mavlink_msg_command_long_decode(msg, &command_long);

  switch (command_long.command)
  {
  case MAV_CMD_COMPONENT_ARM_DISARM: // LONG CMD ID 400 - https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
    mvl_command_ack.command = MAV_CMD_COMPONENT_ARM_DISARM;
    mvl_command_ack.result = MAV_RESULT_ACCEPTED;

    (command_long.param1 == 1) ? armed = 1 : armed = 0;

    mavlink_msg_command_ack_encode(system_id, component_id, &mvl_tx_message, &mvl_command_ack);
    send_mavlink(&mvl_tx_message);
    break;

  default:
    break;
  }
}

void handle_mission_manual_control(mavlink_message_t *msg)
{
  mavlink_manual_control_t manual_control;

  mavlink_msg_manual_control_decode(msg, &manual_control);
  // pass &manual_control to motor control logic

  // uncomment below if you want to reply using the manual control message back to the groundstation (for debugging)
  // mavlink_message_t mvl_tx_message;
  // mavlink_msg_manual_control_encode(system_id, component_id, &mvl_tx_message, &manual_control);
  // send_mavlink(&mvl_tx_message);
}

void parse_mavlink(uint8_t parsing_byte)
{
  mavlink_message_t mvl_rx_message;
  mavlink_status_t status;

  if (mavlink_parse_char(MAVLINK_COMM_0, parsing_byte, &mvl_rx_message, &status))
  {
    switch (mvl_rx_message.msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT: // MSG ID 1 - https://mavlink.io/en/messages/common.html#HEARTBEAT
      previousTimeoutMillis = millis();

      if (armed)
      {
        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
        digitalWrite(LED_RED, LOW);
      }
      else
      {
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        digitalWrite(LED_GREEN, LOW);
      }
      break;

    case MAVLINK_MSG_ID_MANUAL_CONTROL: // MSG ID 69 - https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
      handle_mission_manual_control(&mvl_rx_message);
      break;

    case MAVLINK_MSG_ID_COMMAND_LONG: // MSG ID 76 - https://mavlink.io/en/messages/common.html#COMMAND_LONG
      handle_message_command_long(&mvl_rx_message);
      break;

    default:
      break;
    }
  }
}
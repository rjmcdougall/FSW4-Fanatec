#include <Arduino.h>
#include "esp_err.h"
#include "esp_log.h"
#include "BluetoothSerial.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define BT_NAME "FSW4"
#define BUF_SIZE 128

char simhub_message_buf[BUF_SIZE];
BluetoothSerial bt_serial;

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_16, GPIO_NUM_17, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Install TWAI driver

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    Serial.printf("CAN Driver installed\n");
  }
  else
  {
    Serial.printf("Failed to install CAN driver\n");
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    Serial.printf("CAN Driver started\n");
  }
  else
  {
    Serial.printf("Failed to start CAN driver\n");
  }

  memset(simhub_message_buf, 0x0, BUF_SIZE);
  bt_serial.begin(BT_NAME);
  Serial.println("Init Bluetooth");
}

void process_message()
{
  unsigned int rpm;
  char gear_char;
  int gear_num;
  unsigned int speed_kmh;
  unsigned int fb_psi;
  unsigned int rb_psi;
  unsigned int tps_pct;
  float oil_temperature_degC;
  unsigned int oil_psi;
  unsigned int fuel_psi;
  float water_temperature_degC;
  float air_temperature_degC;
  float lap_time;
  float lap_time_h;
  float lap_time_m;
  float lap_time_s;
  float predictive_time;
  float rolling_time;
  float best_time;
  float delta_time;
  float fl_psi;
  float fr_psi;
  float rl_psi;
  float rr_psi;
  float batt_volts = 14.4;
  float brake_bias_pct = 55;

  sscanf(simhub_message_buf, "%d&%c&%u&%f&%f&%f&%f&%f&%f&%f&%f&%f&%f&%f&%f&%f&%f&%f&%f",
         &rpm,
         &gear_char,
         &speed_kmh,
         &tps_pct,
         &rb_psi,
         &fb_psi,
         &brake_bias_pct,
         &water_temperature_degC,
         &oil_temperature_degC,
         &oil_psi,
         &fl_psi,
         &fr_psi,
         &rl_psi,
         &rr_psi,
         &lap_time,
         &predictive_time,
         &rolling_time,
         &best_time,
         &delta_time);

  Serial.println(best_time);

  switch (gear_char)
  {
  case 'R':
    gear_num = 1;
    break;
  case 'N':
    gear_num = 2;
    break;
  case '1':
    gear_num = 3;
    break;
  case '2':
    gear_num = 4;
    break;
  case '3':
    gear_num = 5;
    break;
  case '4':
    gear_num = 6;
    break;
  case '5':
    gear_num = 7;
    break;
  case '6':
    gear_num = 8;
    break;
  default:
    break;
  }

  twai_message_t can_message;
  can_message.data_length_code = 8;
  can_message.extd = 1;

  can_message.identifier = 0x0400;
  can_message.data[0] = fb_psi & 0xFF;
  can_message.data[1] = fb_psi >> 8;
  can_message.data[2] = rb_psi & 0xFF;
  can_message.data[3] = rb_psi >> 8;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0401;
  can_message.data[0] = rpm & 0xFF;
  can_message.data[1] = rpm >> 8;
  can_message.data[2] = (uint32_t)(tps_pct * 100) & 0xFF;
  can_message.data[3] = (uint32_t)(tps_pct * 100) >> 8;
  can_message.data[4] = (uint32_t)(batt_volts * 10) & 0xFF;
  can_message.data[5] = (uint32_t)(batt_volts * 10) >> 8;
  can_message.data[6] = fuel_psi & 0xff;
  can_message.data[7] = fuel_psi >> 8;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0402;
  can_message.data[0] = 0;
  can_message.data[1] = 0;
  can_message.data[2] = (uint32_t)(air_temperature_degC)&0xFF;
  can_message.data[3] = (uint32_t)(air_temperature_degC) >> 8;
  can_message.data[4] = (uint32_t)(water_temperature_degC)&0xFF;
  can_message.data[5] = (uint32_t)(water_temperature_degC) >> 8;
  can_message.data[6] = oil_psi & 0xff;
  can_message.data[7] = oil_psi >> 8;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0403;
  can_message.data[0] = (uint32_t)(oil_temperature_degC)&0xFF;
  can_message.data[1] = (uint32_t)(oil_temperature_degC) >> 8;
  can_message.data[2] = 0;
  can_message.data[3] = 0;
  can_message.data[4] = 0;
  can_message.data[5] = 0;
  can_message.data[6] = 0;
  can_message.data[7] = 0;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0404;
  can_message.data[0] = 100;
  can_message.data[1] = 0;
  can_message.data[2] = gear_num;
  can_message.data[3] = 0;
  can_message.data[4] = 0;
  can_message.data[5] = 0;
  can_message.data[6] = 0;
  can_message.data[7] = 0;
  can_message.data[6] = speed_kmh & 0xff;
  can_message.data[7] = speed_kmh >> 8;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0405;
  can_message.data[0] = (uint32_t)(lap_time * 1000) & 0xFF;
  can_message.data[1] = ((uint32_t)(lap_time * 1000) >> 8) & 0xFF;
  can_message.data[2] = ((uint32_t)(lap_time * 1000) >> 16) & 0xFF;
  can_message.data[3] = ((uint32_t)(lap_time * 1000) >> 24) & 0xFF;
  can_message.data[4] = (uint32_t)(predictive_time * 1000) & 0xFF;
  can_message.data[5] = ((uint32_t)(predictive_time * 1000) >> 8) & 0xFF;
  can_message.data[6] = ((uint32_t)(predictive_time * 1000) >> 16) & 0xFF;
  can_message.data[7] = ((uint32_t)(predictive_time * 1000) >> 24) & 0xFF;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0406;
  can_message.data[0] = (uint32_t)(rolling_time * 1000) & 0xFF;
  can_message.data[1] = ((uint32_t)(rolling_time * 1000) >> 8) & 0xFF;
  can_message.data[2] = ((uint32_t)(rolling_time * 1000) >> 16) & 0xFF;
  can_message.data[3] = ((uint32_t)(rolling_time * 1000) >> 24) & 0xFF;
  can_message.data[4] = speed_kmh & 0xff;
  can_message.data[5] = speed_kmh >> 8;
  can_message.data[6] = (uint32_t)(brake_bias_pct * 10)&0xFF;
  can_message.data[7] = (uint32_t)(brake_bias_pct * 10) >> 8;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0409;
  can_message.data[0] = (uint32_t)(best_time * 1000) & 0xFF;
  can_message.data[1] = ((uint32_t)(best_time * 1000) >> 8) & 0xFF;
  can_message.data[2] = ((uint32_t)(best_time * 1000) >> 16) & 0xFF;
  can_message.data[3] = ((uint32_t)(best_time * 1000) >> 24) & 0xFF;
  can_message.data[4] = (uint32_t)(rolling_time * 1000) & 0xFF;
  can_message.data[5] = ((uint32_t)(rolling_time * 1000) >> 8) & 0xFF;
  can_message.data[6] = ((uint32_t)(rolling_time * 1000) >> 16) & 0xFF;
  can_message.data[7] = ((uint32_t)(rolling_time * 1000) >> 24) & 0xFF;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0408;
  can_message.data[0] = (uint32_t)(fr_psi * 10) & 0xFF;
  can_message.data[1] = (uint32_t)(fr_psi)*10 >> 8;
  can_message.data[2] = (uint32_t)(fl_psi * 10) & 0xFF;
  can_message.data[3] = (uint32_t)(fl_psi * 10) >> 8;
  can_message.data[4] = (uint32_t)(rr_psi * 10) & 0xFF;
  can_message.data[5] = (uint32_t)(rr_psi * 10) >> 8;
  can_message.data[6] = (uint32_t)(rl_psi * 10) & 0xFF;
  can_message.data[7] = (uint32_t)(rl_psi * 10) >> 8;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));

  can_message.identifier = 0x0410;
  can_message.data[0] = (uint32_t)(delta_time * 1000) & 0xFF;
  can_message.data[1] = ((uint32_t)(delta_time * 1000) >> 8) & 0xFF;
  can_message.data[2] = ((uint32_t)(delta_time * 1000) >> 16) & 0xFF;
  can_message.data[3] = ((uint32_t)(delta_time * 1000) >> 24) & 0xFF;
  can_message.data[4] = 0;
  can_message.data[5] = 0;
  can_message.data[6] = 0;
  can_message.data[7] = 0;
  twai_transmit(&can_message, pdMS_TO_TICKS(10));
  
  }



void loop()
{
  if (bt_serial.connected())
  {
    // Serial.println("Connected");
  }
  else
  {
    Serial.println("BT Disconnected");
    delay(1000);
    if (bt_serial.connected())
    {
      Serial.println("Connected");
    }
  }
  if (bt_serial.available() > 0)
  {
    //Serial.print("bt msg: ");
    bt_serial.readBytesUntil('{', simhub_message_buf, BUF_SIZE);
    int readCount = bt_serial.readBytesUntil('}', simhub_message_buf, BUF_SIZE);
    simhub_message_buf[min(readCount, BUF_SIZE - 1)] = 0x0;
    process_message();
    //Serial.println(simhub_message_buf);
    memset(simhub_message_buf, 0x0, BUF_SIZE);
  }
}
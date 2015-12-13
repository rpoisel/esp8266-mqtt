/* main.c -- MQTT client example
 *
 * Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Redis nor the names of its contributors may be used
 * to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ets_sys.h"
#include "driver/uart.h"
#include "driver/gpio16.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "user_interface.h"
#include "mem.h"

MQTT_Client mqttClient;
static ETSTimer publishTimer;
static BOOL mqttConnected = FALSE;
static uint32_t count = 0;
static ETSTimer counterIncrementTimer;

#define PUBLISH_DATA_LEN 128
#define MQTT_TOPIC_LED "/mqtt/led/green"
#define MQTT_TOPIC_COUNT_0 "/mqtt/count/0"
#define LED_ON "on"
#define LED_OFF "off"
#define LED_GPIO_PIN 7 /* GPIO13 */
#define INTERVAL_PUBLISH 1000 /* milliseconds */
#define INTERVAL_INCREMENT_COUNT 500 /* milliseconds */

static void ICACHE_FLASH_ATTR publish_counter_value(void *args);
static void ICACHE_FLASH_ATTR increment_counter_value(void *args);

void wifiConnectCb(uint8_t status)
{
  if (status == STATION_GOT_IP)
  {
    INFO("STATION_GOT_IP\r\n");
    MQTT_Connect(&mqttClient);
  }
  else
  {
    MQTT_Disconnect(&mqttClient);
  }
}
void mqttConnectedCb(uint32_t *args)
{
  MQTT_Client* client = (MQTT_Client*) args;
  INFO("MQTT: Connected\r\n");

  MQTT_Subscribe(client, MQTT_TOPIC_LED, 0);

  os_timer_disarm(&publishTimer);
  os_timer_setfn(&publishTimer, (os_timer_func_t *)publish_counter_value, client);
  os_timer_arm(&publishTimer, INTERVAL_PUBLISH, 0);

  mqttConnected = TRUE;
}

void mqttDisconnectedCb(uint32_t *args)
{
  MQTT_Client* client = (MQTT_Client*) args;
  INFO("MQTT: Disconnected\r\n");

  mqttConnected = FALSE;
}

void mqttPublishedCb(uint32_t *args)
{
  MQTT_Client* client = (MQTT_Client*) args;
  INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
  char *topicBuf = (char*) os_zalloc(topic_len + 1),
      *dataBuf = (char*) os_zalloc(data_len + 1);

  MQTT_Client* client = (MQTT_Client*) args;

  os_memcpy(topicBuf, topic, topic_len);
  topicBuf[topic_len] = 0;

  os_memcpy(dataBuf, data, data_len);
  dataBuf[data_len] = 0;

  INFO("Receive topic: '%s', data: '%s'\r\n", topicBuf, dataBuf);

  if (os_strcmp(topicBuf, MQTT_TOPIC_LED) == 0)
  {
    if (os_strcmp(dataBuf, LED_ON) == 0)
    {
      gpio_write(LED_GPIO_PIN, 1);
      INFO("LED on\r\n");
    }
    else if (os_strcmp(dataBuf, LED_OFF) == 0)
    {
      gpio_write(LED_GPIO_PIN, 0);
      INFO("LED off\r\n");
    }
    else
    {
      INFO("INVALID state for LED\r\n");
    }
  }

  os_free(topicBuf);
  os_free(dataBuf);
}

static void ICACHE_FLASH_ATTR publish_counter_value(void *args)
{
  MQTT_Client* client = (MQTT_Client*) args;

  os_timer_disarm(&publishTimer);

  char *dataBuf = (char*) os_zalloc(PUBLISH_DATA_LEN);

  if (!mqttConnected)
  {
    return;
  }

  os_sprintf(dataBuf, "%d", count);

  INFO("Publishing: %s\r\n", dataBuf);
  MQTT_Publish(client, "/mqtt/count/0", dataBuf, os_strlen(dataBuf), 0, 0);
  os_free(dataBuf);

  os_timer_setfn(&publishTimer, (os_timer_func_t *)publish_counter_value, client);
  os_timer_arm(&publishTimer, INTERVAL_PUBLISH, 0);
}

static void ICACHE_FLASH_ATTR increment_counter_value(void *args)
{
  count++;
}

void user_init(void)
{
  uart_init(BIT_RATE_115200, BIT_RATE_115200);
  os_delay_us(1000000);

  CFG_Load();

  set_gpio_mode(LED_GPIO_PIN, GPIO_OUTPUT, GPIO_FLOAT);
  gpio_write(LED_GPIO_PIN, 0);

  MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);

  MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);

  MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
  MQTT_OnConnected(&mqttClient, mqttConnectedCb);
  MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
  MQTT_OnPublished(&mqttClient, mqttPublishedCb);
  MQTT_OnData(&mqttClient, mqttDataCb);

  WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);
  INFO("SSID: %s, Password: %s\r\n", sysCfg.sta_ssid, sysCfg.sta_pwd);

  os_timer_disarm(&counterIncrementTimer);
  os_timer_setfn(&counterIncrementTimer, (os_timer_func_t *)increment_counter_value, NULL);
  os_timer_arm(&counterIncrementTimer, INTERVAL_INCREMENT_COUNT, 1);

  INFO("\r\nSystem started ...\r\n");
}

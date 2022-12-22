
#include "Logging.h"
#include <user_interface.h>
#include <ESP8266WiFi.h>

#include "wifi_settings.h"
#include "master_i2c.h"
#include "setup_ap.h"
#include "sender_blynk.h"
#include "sender_mqtt.h"
#include "UserClass.h"
#include "voltage.h"
#include "utils.h"
#include "cert.h"

MasterI2C masterI2C; // Для общения с Attiny85 по i2c

SlaveData data;       // Данные от Attiny85
Settings sett;        // Настройки соединения и предыдущие показания из EEPROM
CalculatedData cdata; //вычисляемые данные
Voltage voltage;      // клас монитора питания
ADC_MODE(ADC_VCC);

/*
Выполняется однократно при включении
*/
void setup()
{
    memset(&cdata, 0, sizeof(cdata));
    memset(&data, 0, sizeof(data));
    LOG_BEGIN(115200); //Включаем логгирование на пине TX, 115200 8N1
    LOG_INFO(F("Booted"));

    LOG_INFO(F("Saved SSID: ") << WiFi.SSID());
    LOG_INFO(F("Saved password: ") << WiFi.psk());
}

void wifi_handle_event_cb(System_Event_t *evt)
{

    switch (evt->event)
    {
    case EVENT_STAMODE_CONNECTED:
        cdata.channel = evt->event_info.connected.channel;
        cdata.router_mac = evt->event_info.connected.bssid[0];
        cdata.router_mac = cdata.router_mac << 8;
        cdata.router_mac |= evt->event_info.connected.bssid[1];
        cdata.router_mac = cdata.router_mac << 8;
        cdata.router_mac |= evt->event_info.connected.bssid[2];
        break;
    }
}


void start_active_point_and_restart()
{
    WiFi.mode(WIFI_AP_STA);
    setup_ap(sett, data, cdata);

    // ухищрения, чтобы не стереть SSID, pwd
    if (WiFi.getPersistent())
        WiFi.persistent(false); // disable saving wifi config into SDK flash area
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);   // отключаем WIFI
    WiFi.persistent(true); // enable saving wifi config into SDK flash area

    LOG_INFO(F("Restart ESP"));
    LOG_END();
    
    WiFi.forceSleepBegin();
    delay(1000);
    ESP.restart();
}

void loop()
{
    bool success = true;
    loadConfig(sett);

    String ssid = WiFi.SSID();

    if (!ssid.length()) {
        start_active_point_and_restart();
        // never happend 
    } 

    LOG_INFO(F("Starting Wi-fi"));

    wifi_set_event_handler_cb(wifi_handle_event_cb);

    if (sett.ip != 0)
    {
        success = WiFi.config(sett.ip, sett.gateway, sett.mask, sett.gateway, IPAddress(8, 8, 8, 8));
        if (success)
        {
            LOG_INFO(F("Static IP OK"));
        }
        else
        {
            LOG_ERROR(F("Static IP FAILED"));
        }
    }

    if (success)
    {

        WiFi.mode(WIFI_STA); //без этого не записывается hostname
        set_hostname();

        // WifiManager уже записал ssid & pass в Wifi, поэтому не надо самому заполнять
        WiFi.begin();

        //Ожидаем подключения к точке доступа
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < ESP_CONNECT_TIMEOUT)
        {
            LOG_INFO(F("Status: ") << WiFi.status());
            delay(300);
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            print_wifi_mode();
            LOG_INFO(F("Connected time: ") << millis() - start << " ms");
            LOG_INFO(F("Connected, IP: ") << WiFi.localIP().toString());

            cdata.rssi = WiFi.RSSI();
            LOG_INFO(F("RSSI: ") << cdata.rssi);
            LOG_INFO(F("channel: ") << cdata.channel);
            LOG_INFO(F("MAC: ") << String(cdata.router_mac, HEX));

            LOG_INFO(F("Going to sleep"));
            LOG_END();

            ESP.deepSleepInstant(0, RF_DEFAULT); // Спим. Instant не ждет 92мс

        } else {
            start_active_point_and_restart();

            //never happend
        }
    }
}


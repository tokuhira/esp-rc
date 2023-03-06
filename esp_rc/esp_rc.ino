#include "util.h"
#include <Wire.h>
#include <Servo.h>
#include <TM1637.h>// https://github.com/maxint-rd/TM16xx
#include <TM16xxDisplay.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>// https://github.com/me-no-dev/AsyncTCP
#elif ESP8266
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>// https://github.com/me-no-dev/ESPAsyncTCP
#else
  #error "ESPAsyncWebServer supports only Espressif ESP32 or ESP8266."
#endif
#include <ESPAsyncWebServer.h>// https://github.com/me-no-dev/ESPAsyncWebServer
#ifdef ARDUINO_OTA
  #include <ArduinoOTA.h>
#endif
#ifdef CAPTIVE_PORTAL
  #include <DNSServer.h>
#endif

const int DRV_ADDR = 0x64;
const int DRV_REG_CONTROL = 0;
const int DRV_REG_FAULT   = 1;
const int DRV_BIT_FAULT  = 0x01;
const int DRV_BIT_OCP    = 0x02;
const int DRV_BIT_UVLO   = 0x04;
const int DRV_BIT_OTS    = 0x08;
const int DRV_BIT_ILIMIT = 0x10;
const int DRV_BIT_CLEAR  = 0x80;

void drvReset() {
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(DRV_REG_FAULT);
  Wire.write(DRV_BIT_CLEAR);
  Wire.endTransmission();
}

void drvControl(uint8_t vset, bool in2, bool in1) {
  uint8_t control;

  control = vset << 2;
  if (in2) {
    control |= 0x02;
  }
  if (in1) {
    control |= 0x01;
  }
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(DRV_REG_CONTROL);
  Wire.write(control);
  Wire.endTransmission();
}

const long THROTTLE_MIN = -37; // -3.05V (-63 for full)
const long THROTTLE_MAX =  37; //  3.05V ( 63 for full)
const long NO_DRIVE_ABS =   5; // ignore between -5 and 5

void drvThrottle(long value) {
  if (NO_DRIVE_ABS < value && value <= THROTTLE_MAX) {
    drvControl(value, false, true);
  } else if (THROTTLE_MIN <= value && value < -NO_DRIVE_ABS) {
    drvControl(-value, true, false);
  } else {
    drvControl(0, 0, 0);
  }
}

Servo steering;
const long STEERING_MIN = 30;
const long STEERING_MAX = 150;

TM1637 module(SGMT_DIO, SGMT_CLK);
TM16xxDisplay display(&module, N_DIGITS);
const long DISPLAY_MIN = -9;
const long DISPLAY_MAX = 9;

const IPAddress ip(192,168,1,1);
const IPAddress subnet(255,255,255,0);
#ifdef CAPTIVE_PORTAL
  DNSServer dnsServer;
#endif
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
const long PWM_MAX=pow(2,PWM_BIT),_Z=0;
const int ZPAD=1+(int)log10(PWM_MAX*2);

long v[2]={0,0};// X:steering, Y:throttle

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
	switch(type){
		case WS_EVT_CONNECT:
			Serial.printf("ws[%u] connect: %s\n", client->id(), client->remoteIP().toString().c_str());
			client->printf("{\"purpose\":\"init\",\"cid\":%u,\"cip\":\"%s\",\"max\":%u,\"zpad\":%u}", client->id(), client->remoteIP().toString().c_str(), PWM_MAX, ZPAD);
			//client->ping();
			break;
		case WS_EVT_DISCONNECT:
			Serial.printf("ws[%u] disconnect\n", client->id());
			v[0]=0;v[1]=0;
			break;
		case WS_EVT_ERROR:
			Serial.printf("ws[%u] error(%u): %s\n", client->id(), *((uint16_t*)arg), (char*)data);
			break;
		case WS_EVT_PONG:
			Serial.printf("ws[%u] pong\n", client->id());
			break;
		case WS_EVT_DATA:
			{
				AwsFrameInfo *info=(AwsFrameInfo*)arg;
				if(info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT){
					data[len]=0;
					String str=(char*)data;
					Serial.printf("ws[%u] text-msg[%llu]: %s\n", client->id(), info->len, str.c_str());
					ws.printfAll("{\"purpose\":\"pong\",\"pong\":\"%s\"}",str.c_str());
					if(str.startsWith("V_CTR")){
						// "V_CTR[L<int>][R<int>]"
						// 0padding needed
						// example: "V_CTR256256"
						v[0]=str.substring(5,5+ZPAD).toInt()-PWM_MAX;
						v[1]=str.substring(5+ZPAD,5+ZPAD+ZPAD).toInt()-PWM_MAX;
						//v[0]=v[0]*5/6;v[1]=v[1]*5/6;
						Serial.printf("%d %d\n",v[0],v[1]);
						ws.printfAll("{\"purpose\":\"check\",\"vl\":%d,\"vr\":%d}",v[0],v[1]);
					}
				}
			}
			break;
	}
}
#ifdef CAPTIVE_PORTAL
class CP:public AsyncWebHandler{
public:
  CP(){}
  virtual ~CP(){}
  bool canHandle(AsyncWebServerRequest *request){return true;}
  void handleRequest(AsyncWebServerRequest *request){request->send_P(200,"text/html",html);}
};
#endif

void setup(){
	Serial.begin(115200);
	delay(1000);

	// softAP https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFiAP.h
	WiFi.mode(WIFI_AP_STA);
	WiFi.softAP(ssid,pass);
	delay(100);//https://github.com/espressif/arduino-esp32/issues/985
	WiFi.softAPConfig(ip,ip,subnet);
  #ifdef CAPTIVE_PORTAL
    dnsServer.start(53, "*", WiFi.softAPIP());
  #endif
  Serial.printf("SSID: %s\nPASS: %s\nAPIP: %s\n",ssid,pass,WiFi.softAPIP().toString().c_str());

	ws.onEvent(onEvent);
	server.addHandler(&ws);
  #ifdef CAPTIVE_PORTAL
    server.addHandler(new CP());
  #endif
	server.on("/",HTTP_GET,[](AsyncWebServerRequest *request){request->send_P(200,"text/html",html);});
	server.begin();
	Serial.println("server started");

  #ifdef ARDUINO_OTA
  #ifdef ESP32
	ArduinoOTA
		.setPassword("sazanka_")
		.onStart([](){Serial.printf("Start updating %s\n",ArduinoOTA.getCommand()==U_FLASH?"sketch":"filesystem");})
		.onEnd([](){Serial.println("\nEnd");})
		.onProgress([](unsigned int progress, unsigned int total){Serial.printf("Progress: %u%%\r",(progress/(total/100)));})
		.onError([](ota_error_t error){Serial.printf("Error[%u]", error);})
		.begin();
  #elif ESP8266
	ArduinoOTA.setPassword("sazanka_");
	ArduinoOTA.onStart([](){Serial.printf("Start updating %s\n",ArduinoOTA.getCommand()==U_FLASH?"sketch":"filesystem");});
	ArduinoOTA.onEnd([](){Serial.println("\nEnd");});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total){Serial.printf("Progress: %u%%\r",(progress/(total/100)));});
	ArduinoOTA.onError([](ota_error_t error){Serial.printf("Error[%u]", error);});
	ArduinoOTA.begin();
  #endif
  #endif

  Wire.begin();
  steering.attach(SRV_GPIO);
  drvReset();
  drvControl(0, 0, 0); // neutral
}

void loop(){
  #ifdef ARDUINO_OTA
	ArduinoOTA.handle();
  #endif
  #ifdef CAPTIVE_PORTAL
    dnsServer.processNextRequest();
  #endif
	ws.cleanupClients();

  steering.write(map(v[0], -PWM_MAX, PWM_MAX, STEERING_MIN, STEERING_MAX));
  drvThrottle(map(v[1], -PWM_MAX, PWM_MAX, THROTTLE_MIN, THROTTLE_MAX));
  display.printf("%2ld%2ld\n", map(v[0], -PWM_MAX, PWM_MAX, DISPLAY_MIN, DISPLAY_MAX), map(v[1], -PWM_MAX, PWM_MAX, DISPLAY_MIN, DISPLAY_MAX));
}
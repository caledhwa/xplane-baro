#define VERSION 1.30

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <LedControl.h>

String ssid = "Home-2001";
String pskey = "bluedog2014";

#pragma region UDP-WIFI
WiFiUDP udp;
int retry = 30;
IPAddress my_IP;
IPAddress multicastIP (239,255,1,1);  // Do not change this
IPAddress xplane_ip;
uint8_t beacon_major_version;
uint8_t beacon_minor_version;
uint32_t application_host_id;
uint32_t versionNumber;
uint32_t receive_port;
uint32_t role;
uint16_t port;
char xp_hostname[32];
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];
unsigned int my_port = 3017;      // Could be anything, this just happens to be my favorite port number
unsigned int xplane_port = 49000; // Don't change this
unsigned int beacon_port = 49707; // or this...
char buffer[1024];
#pragma endregion

#define STATE_IDLE 0
#define STATE_SEARCH 1
#define STATE_READY 2

#define BARO_DREF "sim/cockpit/misc/barometer_setting"
#define BARO_DREF_UID 42
#define RREF "RREF"

int state = STATE_IDLE;
int display_baro = 2992;

LedControl lc = LedControl(D7,D5,D4,1);

#define ENC_CLK D1
#define ENC_DT D2
#define ENC_SW D3

int counter = 0;
int currentStateCLK;
int lastStateCLK;
bool isButtonPressed = false;
unsigned long lastButtonPress = 0;
unsigned long lastTurn = 0;

void IRAM_ATTR buttonPress() {
  if (isButtonPressed && millis() - lastButtonPress > 50) {
    isButtonPressed = false;
    lastButtonPress = millis();
  } else {
    Serial.println("Button pressed");
    isButtonPressed = true;
    display_baro = 2992;
    writeInt(BARO_DREF, display_baro);
  }
}

void updateBaro(int inc) {
  display_baro += inc;
  writeInt(BARO_DREF, display_baro);
}

void IRAM_ATTR updateEncoder(){

	currentStateCLK = digitalRead(ENC_CLK);
	
  if (currentStateCLK != lastStateCLK && 
      currentStateCLK == 1 && 
      millis() - lastTurn > 50){
	
  	if (digitalRead(ENC_DT) != currentStateCLK) {
      updateBaro(-1);
		} else {
      updateBaro(+1);
		}
  
    lastTurn = millis();
	}
	lastStateCLK = currentStateCLK;
}

void setup() {

  retry = 30;
  Serial.begin(115200);
  Serial.println("X-Plane Altimeter Baro Control " + String(VERSION));
  
  connect();

  pinMode(LED_BUILTIN, OUTPUT); 

  lc.shutdown(0, false);
  lc.setIntensity(0, 15);
  lc.clearDisplay(0);

  state = STATE_SEARCH;

  pinMode(ENC_CLK, INPUT); // CLK
  pinMode(ENC_DT, INPUT); // DT
  pinMode(ENC_SW, INPUT_PULLUP); // SW

  lastStateCLK = digitalRead(ENC_CLK);
  attachInterrupt(ENC_CLK, updateEncoder, CHANGE);
	attachInterrupt(ENC_DT, updateEncoder, CHANGE);
  attachInterrupt(ENC_SW, buttonPress, RISING);
}


void loop() {

  switch(state) {
    case STATE_IDLE: 
      setup();
      break;

    case STATE_SEARCH:
      searchForXPlane();
      break;
    
    case STATE_READY:
      ready();
      break;
  }

}

void ready() {
  displayDigits(display_baro);

  int packetSize = udp.parsePacket();
  char *buff = &buffer[3];  // For alignment
  udp.read(buff, packetSize);
  String type =  String(buff).substring(0,4);

  if (type == RREF) {
    for (int offset = 5; offset < packetSize; offset += 8) {
      int code = *((int *) (buff+offset));
      float value = *((float *) (buff+offset+4));
      switch(code) {                                              
        case BARO_DREF_UID:
          int x = String(value * 100,0).toInt();
          Serial.println("Value: " + String(value) + ", X:" + String(x));
          display_baro = x;
          break;
      }
    }
  }
  delay(100);
}

void subscribe(char *dref, uint32_t freq, uint32_t index) {
  struct {
    char dummy[3]; // For alignment
    char hdr[5] = RREF;
    uint32_t dref_freq;
    uint32_t dref_en;
    char dref_string[400];
  } req __attribute__((packed));

  req.dref_freq = freq;
  req.dref_en = index;
  for (int x = 0; x < sizeof(req.dref_string); x++)
    req.dref_string[x] = 0x20;
  strcpy((char *) req.dref_string, (char *) dref);
  
  udp.beginPacket(xplane_ip, xplane_port);
  udp.write(req.hdr, sizeof(req) - sizeof(req.dummy));
  udp.endPacket();
  Serial.print("Subscribed to dref \"");
  Serial.print(dref);
  Serial.println("\"");
}

void writeInt(char *dref, int i) {

  float value = i / 100.0f;

  char buf[5 + 4 + 500];
	strcpy(buf, "DREF");
	memcpy(buf + 5, &value, 4);
	memset(buf + 9, ' ', sizeof(buf) - 9);
	strcpy(buf + 9, dref);

  udp.beginPacket(xplane_ip, xplane_port);
  udp.write(buf, sizeof(buf));
  udp.endPacket();

  Serial.println("Wrote dref \"" + String(dref) + "\", " + String(value,2));
}

void searchForXPlane() {
  int packetSize = udp.parsePacket();
  if (!packetSize) {
    Serial.print(".");
    delay(250);
    if (!retry--) {
      Serial.println("not found");
      delay(1000);
      setup();
      return;
    }
  }

  if (udp.destinationIP() == multicastIP) {
    char *buff = &buffer[1]; // For Alignment
    xplane_ip = udp.remoteIP();
    udp.read(buff, packetSize);
    beacon_major_version = buff[5];
    beacon_minor_version = buff[6];
    application_host_id = *((int *) (buff+7));
    versionNumber = *((int *) (buff+11));
    receive_port = *((int *) (buff+15));
    strcpy(xp_hostname, &buff[21]);
    
    String version = String(versionNumber/10000)+"."+String((versionNumber%10000)/100)+"r"+String(versionNumber%100);
    String heading = " Found Version "+version+" running on "+String(xp_hostname)+" at IP ";
    Serial.print(heading);
    Serial.println(udp.remoteIP());
    
    state = STATE_READY;
    udp.begin(my_port);

    subscribe(BARO_DREF, 2, BARO_DREF_UID);
  }
}

void displayDigits(int i) {
  int thousandsDigit = (int)i/1000;
  int hundredsDigit = ((int)i/100*100-(int)i/1000*1000)/100;
  int tensDigit = ((int)i/10*10-(int)i/100*100)/10;
  int onesDigit = (int)i-(int)i/10*10;

  lc.setDigit(0,3,thousandsDigit, false);
  lc.setDigit(0,2,hundredsDigit, true);
  lc.setDigit(0,1,tensDigit, false);
  lc.setDigit(0,0,onesDigit, false);
}

void connect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pskey);

  Serial.println("Attempting Connection to Network: ");
  Serial.println(ssid);

  // Wait for connection
  while ((retry-- > 0) && (WiFi.status() != WL_CONNECTED)) {
    delay(250);
    Serial.print(".");
  }

  if (retry > 0) {
    my_IP = WiFi.localIP();
    Serial.println("connected.  My IP = ");
    Serial.println(my_IP);
  }
  else {
    Serial.println("Unable to connect");
    return;
  }

  Serial.println("Searching for X-Plane");
  retry = 30;
  udp.beginMulticast(my_IP, multicastIP, beacon_port);
}
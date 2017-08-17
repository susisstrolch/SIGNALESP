/*
*   RF_RECEIVER v3.3 for Arduino
*   Sketch to use an arduino as a receiver/sending device for digital signals
*
*   The Sketch can also encode and send data via a transmitter,
*   while only PT2262 type-signals for Intertechno devices are implemented in the sketch,
*   there is an option to send almost any data over a send raw interface
*   2014-2015  N.Butzek, S.Butzek
*   2016 S.Butzek

*   This software focuses on remote sensors like weather sensors (temperature,
*   humidity Logilink, TCM, Oregon Scientific, ...), remote controlled power switches
*   (Intertechno, TCM, ARCtech, ...) which use encoder chips like PT2262 and
*   EV1527-type and manchester encoder to send information in the 433MHz Band.
*   But the sketch will also work for infrared or other medias. Even other frequencys
*   can be used
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

//#define CMP_MEMDBG 1

#define CMP_cc1101

#define PROGNAME               "SIGNALEsp"
#define PROGVERS               "3.3.1-dev"
#define VERSION_1               0x33
#define VERSION_2               0x1d

/*
 *   ESP8266            cc1101
 *   
 *   VDD           -------- VDD    3.3V
 *   GPIO4  / D2   -------- GDO0
 *   GPIO5  / D1   -------- GDO2
 *   GPIO12 / D6   -------- MISO  --|  SPI Bus
 *   GPIO13 / D7   -------- MOSI  --|
 *   GPIO14 / D5   -------- SCLK  --|
 *   GPIO15 / D8   -------- CSn   --|
 *   GND           -------- GND
 *   
 *   
 *   Led GPIO16 / D0
 * 
 */

#ifdef CMP_cc1101
#define PIN_LED                16
		#define PIN_SEND              4   // gdo0Pin TX out ESP GPIO5 D1 
	  #define PIN_RECEIVE           5   // gdo2Pin RX in  ESP GPIO4 D2
    #define TRIGGER_PIN           0   // Config Reset wenn bei boot auf Gnd
#else
  #define PIN_LED               16    // Message-LED
#define PIN_SEND               0
  #define PIN_RECEIVE           2
  #define TRIGGER_PIN           5   // Config Reset wenn bei boot auf Gnd
#endif

#define TRIGGER_PIN 0     

#define BAUDRATE               115200
#define FIFO_LENGTH			   255    
#define DEBUG				   1

char* update_username = "admin";  //Benutzername zum login für die OTA-Programmierung
char* update_password = "SIGNALEsp"; //Passwort

char* host = "signalesp";
bool hascc1101 = false;

//default custom static IP
char static_ip[16] = "10.0.1.56";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";

#include <FS.h>   
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "webupdate.h"   //Implementierung des Web-Updaters

#include "cc1101.h"
#include <EEPROM.h>

#include "FastDelegate.h"
#include "output.h"
#include "bitstore.h"
#include "signalDecoder.h"
// #include "TimerOne.h"  // Timer for LED Blinking

#include "SimpleFIFO.h"
SimpleFIFO<int,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
SignalDetectorClass musterDec;

// cc1101 cc1101;

#define pulseMin  90
volatile bool blinkLED = false;
String cmdstring = "";
volatile unsigned long lastTime = micros();


extern "C" {
#include "user_interface.h"
}

os_timer_t cronTimer;

// EEProm Address
#define EE_MAGIC_OFFSET      0
#define addr_features        0xff


void ICACHE_RAM_ATTR handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void ICACHE_RAM_ATTR cronjob(void *pArg);
int freeRam();
void changeReciver();
void changeFilter();
void HandleCommand();
bool command_available = false;
unsigned long getUptime();
void getConfig();
void enDisPrint(bool enDis);
void getPing();
void configCMD();
void storeFunctions(const int8_t ms = 1, int8_t mu = 1, int8_t mc = 1);
void getFunctions(bool *ms, bool *mu, bool *mc);
void initEEPROM(void);
void changeReceiver();
uint8_t cmdstringPos2int(uint8_t pos);
void printHex2(const byte hex);
uint8_t rssiCallback() { return 0; };	// Dummy return if no rssi value can be retrieved from receiver

WiFiServer Server(23);  //  port 23 = telnet
WiFiClient serverClient;

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  DBG_PRINTLN("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  
	Serial.begin(BAUDRATE);
	while (!Serial)
	{
    delay(200);
		; // wait for serial port to connect. Needed for native USB
	}

  pinMode(TRIGGER_PIN, INPUT);
  
  //read configuration from FS json
  DBG_PRINTLN("mounting FS...");

  if (SPIFFS.begin()) {
    DBG_PRINTLN("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      DBG_PRINTLN("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        DBG_PRINTLN("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          DBG_PRINTLN("\nparsed json");
          if(json["ip"]) {
            DBG_PRINTLN("setting custom ip from config");
            //static_ip = json["ip"];
            strcpy(static_ip, json["ip"]);
            strcpy(static_gw, json["gateway"]);
            strcpy(static_sn, json["subnet"]);
            DBG_PRINTLN(static_ip);
          } else {
            DBG_PRINTLN("no custom ip in config");
          }
        } else {
          DBG_PRINTLN("failed to load json config");
        }
      }
    }
  } else {
    DBG_PRINTLN("failed to mount FS");
  }
  //end read

  WiFiManager wifiManager;

   if ( digitalRead(TRIGGER_PIN) == LOW ) {
    wifiManager.resetSettings(); 
   }

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  IPAddress _ip,_gw,_sn;

  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);
  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);   
 
  wifiManager.autoConnect("AutoConnectAP");

// if you get here you have connected to the WiFi
  DBG_PRINTLN("connected....");

//save the custom parameters to FS
  if (shouldSaveConfig) {
    DBG_PRINTLN("saving config");   
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      DBG_PRINTLN("failed to open config file for writing");
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  DBG_PRINT(PROGVERS " SIGNALEsp "); 
  DBG_PRINTLN("- compiled at " __DATE__ " " __TIME__);   
  DBG_PRINT("Using sFIFO  Size: ");DBG_PRINTLN(FIFO_LENGTH);

 
	pinMode(PIN_RECEIVE, INPUT);
	pinMode(PIN_LED, OUTPUT);

  initEEPROM();

	#ifdef CMP_cc1101
  	DBG_PRINTLN(F("CCInit "));

	cc1101::CCinit();					 // cc1101 init
	hascc1101 = cc1101::checkCC1101();	 // Check for cc1101

	if (hascc1101)
	{
		DBG_PRINTLN("cc1101 found");
		musterDec.setRSSICallback(&cc1101::getRSSI);                    // Provide the RSSI Callback
	} else {
		musterDec.setRSSICallback(&rssiCallback);	// Provide the RSSI Callback		
  }
	#endif 

	pinMode(PIN_SEND, OUTPUT);
	DBG_PRINTLN("Starting timerjob");
	delay(50);

  os_timer_disarm(&cronTimer);
  os_timer_setfn(&cronTimer, cronjob, NULL);
  os_timer_arm(&cronTimer, 31, true);

//  Timer1.initialize(31*1000); //Interrupt wird jede n Millisekunden ausgeloest
//	Timer1.attachInterrupt(cronjob);

//   musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = true;

  Server.begin();  // telnet server
  Server.setNoDelay(true);

 	cmdstring.reserve(40);
	
  WiFiVerbindung(update_username, update_password);

#ifdef CMP_cc1101
 if (!hascc1101 || cc1101::regCheck()) {
		enableReceive();
		DBG_PRINTLN(F("receiver enabled"));
	}
	else {
		DBG_PRINTLN(F("cc1101 is not correctly set. Please do a factory reset via command e"));
	}
#else
  enableReceive();
  DBG_PRINTLN("enableReceive");
	#endif

 MSG_PRINTLN("Setup End");

// Software Watchdog sperren
 wdt_disable();
 
//Software Watchdog Timeout setzen
 wdt_enable(WDTO_4S);
}

void ICACHE_RAM_ATTR cronjob(void *pArg) {
	 const unsigned long  duration = micros() - lastTime;
	 if (duration > maxPulse) { //Auf Maximalwert pr�fen.
		 int sDuration = maxPulse;
		 if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			 sDuration = -sDuration;
		 }
		 FiFo.enqueue(sDuration);

		 lastTime = micros();
	 }
	digitalWrite(PIN_LED, blinkLED);
	blinkLED = false;
}

void loop() {
	static int aktVal = 0;
	bool state;

// Watchdog Triggern 
  wdt_reset();
  
  httpServer.handleClient();
  delay(1);

	serialEvent();
	ethernetEvent();
  
	if (command_available) {
		command_available = false;
		HandleCommand();
		if (!command_available) { cmdstring = ""; }
		blinkLED = true;
	}
//  yield();

  wdt_reset();
  
	while (FiFo.count()>0) { //Puffer auslesen und an Dekoder uebergeben
		aktVal = FiFo.dequeue();
		state = musterDec.decode(&aktVal);
		if (state) blinkLED = true; //LED blinken, wenn Meldung dekodiert
		yield();
	}
}

//========================= Pulseauswertung ================================================
void ICACHE_RAM_ATTR handleInterrupt() {
	const unsigned long Time = micros();
	//const bool state = digitalRead(PIN_RECEIVE);
	const unsigned long  duration = Time - lastTime;
	lastTime = Time;
	if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
		int sDuration;
		if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
			sDuration = int(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
    }else {
			sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
		}
		if (isHigh(PIN_RECEIVE)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
			sDuration = -sDuration;
		}
	//MSG_PRINTLN(sDuration);
 
		FiFo.enqueue(sDuration);

  }
}

void enableReceive() {
   attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE), handleInterrupt, CHANGE);

   #ifdef CMP_cc1101
    if (hascc1101) cc1101::setReceiveMode();
   #endif
}

void disableReceive() {
  detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE));

  #ifdef CMP_cc1101
   if (hascc1101) cc1101::setIdleMode();
  #endif
}

//================================= RAW Send ======================================
void send_raw(const uint8_t startpos, const uint16_t endpos, const int16_t *buckets, String *source = &cmdstring)
{
	uint8_t index = 0;
	unsigned long stoptime = micros();
	bool isLow;
	uint16_t dur;
	for (uint16_t i = startpos; i <= endpos; i++)
	{
		//MSG_PRINT(cmdstring.substring(i,i+1));
		index = source->charAt(i) - '0';
		//MSG_PRINT(index);
		isLow = buckets[index] >> 15;
		dur = abs(buckets[index]); 		//isLow ? dur = abs(buckets[index]) : dur = abs(buckets[index]);

		while (stoptime > micros()) {
			;
		}
		isLow ? digitalLow(PIN_SEND) : digitalHigh(PIN_SEND);
		stoptime += dur;
	}
	while (stoptime > micros()) {
		;
	}
}

void send_mc(const uint8_t startpos, const uint8_t endpos, const int16_t clock)
{
	int8_t b;
	char c;
	uint8_t bit;

	unsigned long stoptime = micros();
	for (uint8_t i = startpos; i <= endpos; i++) {
		c = cmdstring.charAt(i);
		b = ((byte)c) - (c <= '9' ? 0x30 : 0x37);

		for (bit = 0x8; bit>0; bit >>= 1) {
			for (byte i = 0; i <= 1; i++) {
				if ((i == 0 ? (b & bit) : !(b & bit)))
					digitalLow(PIN_SEND);
				else
					digitalHigh(PIN_SEND);

				stoptime += clock;
				while (stoptime > micros())
					yield();
			}
		}
	}
}

bool split_cmdpart(int16_t *startpos, String *msg_part)
{
	int16_t endpos = 0;
	//startpos=cmdstring.indexOf(";",startpos);   			 // search first  ";"
	endpos = cmdstring.indexOf(";", *startpos);     			 // search next   ";"

	if (endpos == -1 || *startpos == -1) return false;
	*msg_part = cmdstring.substring(*startpos, endpos);
	*startpos = endpos + 1;    // Set startpos to endpos to extract next part
	return true;
}

struct s_sendcmd {
	int16_t sendclock;
	uint8_t type;
	uint8_t datastart;
	uint16_t dataend;
	int16_t buckets[6];
};

void send_cmd()
{
#define combined 0
#define manchester 1
#define raw 2

	String msg_part;
	msg_part.reserve(30);
	uint8_t repeats = 1;  // Default is always one iteration so repeat is 1 if not set
						  //uint8_t type;
	int16_t start_pos = 0;
	//int16_t buckets[6]={};
	uint8_t counter = 0;
	//uint16_t sendclock;
	bool extraDelay = true;

	s_sendcmd command[5];

	uint8_t ccParamAnz = 0;   // Anzahl der per F= uebergebenen cc1101 Register
	uint8_t ccReg[4];
	uint8_t val;

	disableReceive();

	uint8_t cmdNo = 255;

	while (split_cmdpart(&start_pos, &msg_part))
	{
		if (msg_part.charAt(0) == 'S')
		{
			if (msg_part.charAt(1) == 'C')  // send combined informatio flag
			{
				//type=combined;
				//cmdNo=255;
				extraDelay = false;
			}
			else if (msg_part.charAt(1) == 'M') // send manchester
			{
				//type=manchester;
				cmdNo++;
				command[cmdNo].type = manchester;
			}
			else if (msg_part.charAt(1) == 'R') // send raw
			{
				//type=raw;
				cmdNo++;
				command[cmdNo].type = raw;
				extraDelay = false;

			}
		}
		else if (msg_part.charAt(0) == 'P' && msg_part.charAt(2) == '=') // Do some basic detection if data matches what we expect
		{
			counter = msg_part.substring(1, 2).toInt(); // extract the pattern number
			command[cmdNo].buckets[counter] = msg_part.substring(3).toInt();

		} else if(msg_part.charAt(0) == 'R' && msg_part.charAt(1) == '=') {
			repeats = msg_part.substring(2).toInt();

		} else if (msg_part.charAt(0) == 'D') {
			command[cmdNo].datastart = start_pos - msg_part.length() + 1;
			command[cmdNo].dataend = start_pos - 2;
		} else if(msg_part.charAt(0) == 'C' && msg_part.charAt(1) == '=')
		{
			command[cmdNo].sendclock = msg_part.substring(2).toInt();
		} else if(msg_part.charAt(0) == 'F' && msg_part.charAt(1) == '=')
		{
			ccParamAnz = msg_part.length() / 2 - 1;
			
			if (ccParamAnz > 0 && ccParamAnz <= 5 && hascc1101) {
				uint8_t hex;
				MSG_PRINT("write new ccreg  ");
				for (uint8_t i=0;i<ccParamAnz;i++)
				{
					ccReg[i] = cc1101::readReg(0x0d + i, 0x80);    // alte Registerwerte merken
					hex = (uint8_t)msg_part.charAt(2 + i*2);
					val = cc1101::hex2int(hex) * 16;
					hex = (uint8_t)msg_part.charAt(3 + i*2);
					val = cc1101::hex2int(hex) + val;
					cc1101::writeReg(0x0d + i, val);            // neue Registerwerte schreiben
					printHex2(val);
				}
				MSG_PRINTLN("");
			}
		}
	}

	#ifdef CMP_cc1101
	  if (hascc1101) cc1101::setTransmitMode();	
	#endif

	for (uint8_t i = 0; i<repeats; i++)
	{
		for (uint8_t c = 0; c <= cmdNo; c++)
		{
			if (command[c].type == raw) send_raw(command[c].datastart, command[c].dataend, command[c].buckets);
			if (command[c].type == manchester) send_mc(command[c].datastart, command[c].dataend, command[c].sendclock);
			digitalLow(PIN_SEND);
		}
		if (extraDelay) delay(1);
	}

	if (ccParamAnz > 0) {
		MSG_PRINT("ccreg write back ");
		for (uint8_t i=0;i<ccParamAnz;i++)
		{
			val = ccReg[i];
			printHex2(val);
			cc1101::writeReg(0x0d + i, val);    // gemerkte Registerwerte zurueckschreiben
		}
		MSG_PRINTLN("");
	}

	MSG_PRINTLN(cmdstring); // echo
	enableReceive();	// enable the receiver
 
}

//================================= Kommandos ======================================
void IT_CMDs();

void HandleCommand()
{
  uint8_t reg;
  uint8_t val;

#define  cmd_Version 'V'
#define  cmd_freeRam 'R'
#define  cmd_uptime 't'
#define  cmd_changeReceiver 'X'
#define  cmd_space ' '
#define  cmd_help '?'
#define  cmd_changeFilter 'F'
#define  cmd_send 'S'
#define  cmd_ping 'P'
  #define  cmd_config 'C'     // CG get config, set config, C<reg> get cc1101 register
  #define  cmd_write 'W'      // write EEPROM und write cc1101 register
  #define  cmd_read  'r'      // read EEPROM
  #define  cmd_patable 'x' 
  #define  cmd_ccFactoryReset 'e'  // EEPROM / factory reset
  #define  cmd_boot 'b'   // Reboot

  DBG_PRINT("CMD: ");
  DBG_PRINTLN(cmdstring);

	if (cmdstring.charAt(0) == cmd_ping) {
		getPing();
	}  // ?: Kommandos anzeigen
	else if (cmdstring.charAt(0) == cmd_help) {
		MSG_PRINT(cmd_help);	MSG_PRINT(F(" Use one of "));
		MSG_PRINT(cmd_Version); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_freeRam); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_uptime); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_changeReceiver); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_changeFilter); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_send); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_ping); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_config); MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_read);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_write);MSG_PRINT(cmd_space);
	if (hascc1101) {
		MSG_PRINT(cmd_patable);MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_ccFactoryReset);MSG_PRINT(cmd_space);
	}
  MSG_PRINT(cmd_boot);MSG_PRINT(cmd_space);
		MSG_PRINTLN("");
	}
	// V: Version
	else if (cmdstring.charAt(0) == cmd_Version) {
	  MSG_PRINT("V " PROGVERS " SIGNALduino ");
	  if (hascc1101) {
		MSG_PRINT(F("cc1101 "));
      }
	MSG_PRINTLN("- compiled at " __DATE__ " " __TIME__)

	}
	// R: FreeMemory
	else if (cmdstring.charAt(0) == cmd_freeRam) {
		MSG_PRINTLN(freeRam());
	}
	else if (cmdstring.charAt(0) == cmd_send) {
		if (musterDec.getState() != searching)
		{
			command_available = true;
	} else {
			send_cmd(); // Part of Send
		}
	}
	// t: Uptime
	else if (cmdstring.charAt(0) == cmd_uptime) {
		MSG_PRINTLN(getUptime());
	}
	// XQ disable receiver
	else if (cmdstring.charAt(0) == cmd_changeReceiver) {
    changeReceiver();
	}
	else if (cmdstring.charAt(0) == cmd_changeFilter) {
	}
	else if (cmdstring.charAt(0) == cmd_config) {
    if (cmdstring.charAt(1) == 'G') {
      getConfig();
    }
    else if (cmdstring.charAt(1) == 'E' || cmdstring.charAt(1) == 'D') {  //Todo:  E und D sind auch hexadezimal, werden hier aber abgefangen
		configCMD();
	}
    else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && hascc1101) {
      reg = cmdstringPos2int(1);
      cc1101::readCCreg(reg);
    }
    else {
      MSG_PRINTLN(F("Unsupported command"));
    }
  }
  else if (cmdstring.charAt(0) == cmd_write) {            // write EEPROM und CC11001 register
    if (cmdstring.charAt(1) == 'S' && cmdstring.charAt(2) == '3' && hascc1101)  {       // WS<reg>  Command Strobes
        cc1101::commandStrobes();
    } else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && isHexadecimalDigit(cmdstring.charAt(3)) && isHexadecimalDigit(cmdstring.charAt(4))) {
         reg = cmdstringPos2int(1);
         val = cmdstringPos2int(3);
         EEPROM.begin(512); //Max bytes of eeprom to use
         yield();
         EEPROM.write(reg, val);  
         EEPROM.commit();
         EEPROM.end();
         if (hascc1101) {
           cc1101::writeCCreg(reg, val);
         }
    } else {
         MSG_PRINTLN(F("Unsupported command"));
    }
  }
  // R<adr>  read EEPROM
  else if (cmdstring.charAt(0) == cmd_read && isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {             // R<adr>  read EEPROM
     reg = cmdstringPos2int(1);
     MSG_PRINT(F("EEPROM "));
     printHex2(reg);
     EEPROM.begin(512); //Max bytes of eeprom to use
     yield();
     if (cmdstring.charAt(3) == 'n') {
         MSG_PRINT(F(" :"));
         for (uint8_t i = 0; i < 16; i++) {
             MSG_PRINT(" ");
             printHex2(EEPROM.read(reg + i));
	}
	} else {
        MSG_PRINT(F(" = "));
        printHex2(EEPROM.read(reg));
     }
     MSG_PRINTLN("");
     EEPROM.end();
  }
  else if (cmdstring.charAt(0) == cmd_patable && isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && hascc1101) {
     val = cmdstringPos2int(1);
     cc1101::writeCCpatable(val);
     MSG_PRINT(F("Write "));
     printHex2(val);
     MSG_PRINTLN(F(" to PATABLE done"));
  }
  else if (cmdstring.charAt(0) == cmd_ccFactoryReset && hascc1101) { 
     cc1101::ccFactoryReset();
     cc1101::CCinit();
  }
  else if (cmdstring.charAt(0) == cmd_boot) {
   DBG_PRINTLN("## Reboot ##");
   disableReceive();
   wdt_disable();
   delay(500);
   ESP.restart(); 
  }
  else {
		MSG_PRINTLN(F("Unsupported command"));
	}
}

uint8_t cmdstringPos2int(uint8_t pos) {
  uint8_t val;
  uint8_t hex;
  
       hex = (uint8_t)cmdstring.charAt(pos);
       val = cc1101::hex2int(hex) * 16;
       hex = (uint8_t)cmdstring.charAt(pos+1);
       val = cc1101::hex2int(hex) + val;
       return val;
}

void getConfig()
{
	MSG_PRINT(F("MS="));
	//enDisPrint(musterDec.MSenabled);
	MSG_PRINT(musterDec.MSenabled, DEC);
	MSG_PRINT(F(";MU="));
	//enDisPrint(musterDec.MUenabled);
	MSG_PRINT(musterDec.MUenabled, DEC);
	MSG_PRINT(F(";MC="));
	//enDisPrint(musterDec.MCenabled);
	MSG_PRINTLN(musterDec.MCenabled, DEC);
}


void enDisPrint(bool enDis)
{
	if (enDis) {
		MSG_PRINT(F("enable"));
	}
	else {
		MSG_PRINT(F("disable"));
	}
}


void configCMD()
{
	bool *bptr;

	if (cmdstring.charAt(2) == 'S') {  	  //MS
		bptr = &musterDec.MSenabled;;
	}
	else if (cmdstring.charAt(2) == 'U') {  //MU
		bptr = &musterDec.MUenabled;;
	}
	else if (cmdstring.charAt(2) == 'C') {  //MC
		bptr = &musterDec.MCenabled;;
	}

	if (cmdstring.charAt(1) == 'E') {   // Enable
		*bptr = true;
	}
	else if (cmdstring.charAt(1) == 'D') {  // Disable
		*bptr = false;
  } else {
		return;
	}
	storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled);
}

inline void ethernetEvent()
{
	//check if there are any new clients
	if (Server.hasClient()) {
		if (!serverClient || !serverClient.connected()) {
			if (serverClient) serverClient.stop();
			serverClient = Server.available();
			DBG_PRINTLN("New client: ");
			return;
		}
	}
}

void serialEvent()
{
	while (MSG_PRINTER.available())
	{
		char inChar = (char)MSG_PRINTER.read();
		switch (inChar)
		{
		case '\n':
		case '\r':
		case '\0':
		case '#':
			command_available = true;
			break;
		default:
			cmdstring += inChar;
		}
		yield();
	}
}

int freeRam() {
	return system_get_free_heap_size();
}

unsigned long getUptime()
{
	unsigned long now = millis();
	static uint16_t times_rolled = 0;
	static unsigned long last = 0;
	// If this run is less than the last the counter rolled
	unsigned long seconds = now / 1000;
	if (now < last) {
		times_rolled++;
		seconds += ((long(4294967295) / 1000)*times_rolled);
	}
	last = now;
	return seconds;
}

void getPing()
{
	MSG_PRINTLN("OK");
	yield();
}

void changeReceiver() {
	if (cmdstring.charAt(1) == 'Q')
	{
		disableReceive();
	}
	if (cmdstring.charAt(1) == 'E')
	{
		enableReceive();
	}
}

  void printHex2(const byte hex) {   // Todo: printf oder scanf nutzen
    if (hex < 16) {
      MSG_PRINT("0");
    }
    MSG_PRINT(hex, HEX);
  }

//================================= EEProm commands ======================================

void storeFunctions(const int8_t ms, int8_t mu, int8_t mc)
{
	mu = mu << 1;
	mc = mc << 2;
  EEPROM.begin(512); //Max bytes of eeprom to use
  yield();

	int8_t dat = ms | mu | mc;
	EEPROM.write(addr_features, dat);
    EEPROM.commit();
    EEPROM.end(); 
}

void getFunctions(bool *ms, bool *mu, bool *mc)
{
    EEPROM.begin(512); //Max bytes of eeprom to use
    yield();
    
	int8_t dat = EEPROM.read(addr_features);

	*ms = bool(dat &(1 << 0));
	*mu = bool(dat &(1 << 1));
	*mc = bool(dat &(1 << 2));
    EEPROM.end();     
}

void initEEPROM(void) {
  EEPROM.begin(512); //Max bytes of eeprom to use
  yield();
  if (EEPROM.read(EE_MAGIC_OFFSET) == VERSION_1 && EEPROM.read(EE_MAGIC_OFFSET+1) == VERSION_2) {
    DBG_PRINTLN("Reading values fom eeprom");
  } else {
   
    //hier fehlt evtl ein getFunctions()
    DBG_PRINTLN("Init eeprom to defaults after flash");
     EEPROM.write(EE_MAGIC_OFFSET, VERSION_1);
     EEPROM.write(EE_MAGIC_OFFSET+1, VERSION_2);
  EEPROM.commit();
     storeFunctions(1, 1, 1);    // Init EEPROM with all flags enabled

     cc1101::ccFactoryReset(); // der ccFactoryReset muss auch durchgefuehrt werden, wenn der cc1101 nicht erkannt wurde
  }
  EEPROM.end(); 
  getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled);
}




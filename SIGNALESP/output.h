// output.h

#define ETHERNET_PRINT

#define digitalLow(P) digitalWrite(P,LOW)
#define digitalHigh(P) digitalWrite(P,HIGH)
#define isHigh(P) (digitalRead(P) == HIGH)
#define isLow(P) (digitalRead(P) == LOW)
#define digitalState(P)((uint8_t)isHigh(P))

#ifndef _OUTPUT_h
 #define _OUTPUT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#ifdef ETHERNET_PRINT
 #include <WiFiClient.h>
 extern WiFiClient serverClient;
 #define MSG_PRINTER serverClient // Not Implemented at this time
#else
 #define MSG_PRINTER Serial
#endif

#ifdef ETHERNET_DEBUG
 #define DBG_PRINTER Client // Not Implemented at this time
#else
 #define DBG_PRINTER Serial
#endif

#define MSG_PRINT(...) { MSG_PRINTER.print(__VA_ARGS__); }
#define MSG_PRINTLN(...) { MSG_PRINTER.println(__VA_ARGS__); }

#ifdef DEBUG
  #define DBG_PRINT(...) { DBG_PRINTER.print(__VA_ARGS__); }
  #define DBG_PRINTLN(...) { DBG_PRINTER.println(__VA_ARGS__); }
#else
  #define DBG_PRINT(...) 
  #define DBG_PRINTLN(...) 
#endif

#endif

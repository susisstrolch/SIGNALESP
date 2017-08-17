#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#include "output.h"

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void WiFiVerbindung(char* update_username, char* update_password)
{
   MDNS.begin(host);

  httpUpdater.setup(&httpServer,update_username, update_password);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  DBG_PRINTLN("HTTPUpdateServer ready!");
  DBG_PRINTLN(WiFi.localIP());
}

#ifndef SettingsServer_h
#define SettingsServer_h

#include "Arduino.h"
#include <WebServer.h>

class SettingsServer {
  public:
    SettingsServer() {
      WebServer server(80);
    }
    void start();
    
  private:
    WebServer server;
    void createServerHandlers();  
};

#endif

#include <Arduino.h>
#include "pMBB32.h"
#include <NativeEthernet.h>

#define ON 0xFF;// duty cycle - 100/0.392157 = 0xFF = 100%
#define OFF 0;


extern byte mac[];





extern uint32_t temp;
extern uint8_t minCell;
extern uint8_t maxCell;
extern uint8_t minModule;
extern uint8_t maxModule;
extern uint16_t minCellV;
extern uint16_t maxCellV;
extern uint32_t packCurrent;
extern uint32_t packVoltage;
extern uint32_t preChargeV;
extern uint16_t batteryVoltage;
extern uint8_t keypadStatus;// 0x01 = Park, 0x02 = Reverse, 0x03 = Neutral, 0x04 = Drive, 0x05 = Ignition, 0x06 = SpeedMode, 0x07 = AUX, 0x08 = DriveMode
extern uint32_t U3V;

extern uint16_t rpm;
extern uint16_t power;
extern uint16_t throttle;
extern uint16_t groundSpeed;
extern uint16_t GPSaltitude;

extern char ReplyBuffer[];
extern uint8_t displayBuffer[16];



void SendCANFramesToEth(EthernetClient& client) {
  //client.beginFrame(clientIP, staticIP, 32)
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 2");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("Hello World!");
          client.print("<br />");
          client.print("Max Cell Voltage = ");
          client.print((double)maxCellV*0.000076293945);
          client.print("<br />");
          client.print("Min Cell Voltage = ");
          client.print((double)minCellV*0.000076293945);
          client.print("<br />");
          client.print("12V Battery Voltage = ");
          client.print((double)batteryVoltage/100);
          client.println("<br />"); 
          client.println("</html>");

          //client.print(displayBuffer, 16);
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
  }
  // give the web browser time to receive the data
  delay(1);
  client.stop();

}


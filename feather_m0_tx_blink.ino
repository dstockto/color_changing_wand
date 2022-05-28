// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

// TODO: change to use pushbutton to change color
// TODO: use color sensor to pick new color instead of random
// TODO: add on/off switch

#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define RING_PIN 12
#define RING_LEDS 16
#define BUTTON_PIN 11

Adafruit_NeoPixel ring = Adafruit_NeoPixel(RING_LEDS, RING_PIN, NEO_GRBW);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

uint32_t color = ring.Color(75, 250, 100);
byte gammatable[256];

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
// Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13

#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13

#elif defined(ESP8266)    // ESP8266 feather w/wing
#define RFM69_CS      2    // "E"
#define RFM69_IRQ     15   // "B"
#define RFM69_RST     16   // "D"
#define LED           0

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
#define RFM69_INT     9  // "A"
#define RFM69_CS      10  // "B"
#define RFM69_RST     11  // "C"
#define LED           13

#elif defined(ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#define LED           13

#elif defined(ARDUINO_NRF52832_FEATHER)
/* nRF52832 feather w/wing */
#define RFM69_RST     7   // "A"
#define RFM69_CS      11   // "B"
#define RFM69_INT     31   // "C"
#define LED           17

#endif


/* Teensy 3.x w/wing
  #define RFM69_RST     9   // "A"
  #define RFM69_CS      10   // "B"
  #define RFM69_IRQ     4    // "C"
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* WICED Feather w/wing
  #define RFM69_RST     PA4     // "A"
  #define RFM69_CS      PB4     // "B"
  #define RFM69_IRQ     PA15    // "C"
  #define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    gammatable[i] = x;
    //Serial.println(gammatable[i]);
  }


  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  ring.begin();
  ring.setPixelColor(0, ring.Color(255, 255, 0, 0));
  ring.show();
  ring.setBrightness(40);
}

// CRGBW

uint8_t packet_color[5];
boolean oldState = HIGH;

void loop() {
  packet_color[0] = (uint8_t)'C';
  delay(100);  // Wait .1 second between transmits, could also 'sleep' here!

  boolean newState = digitalRead(BUTTON_PIN);

  // TODO wire button and see if this bails from the loop
  if ((newState == LOW) && (oldState == HIGH)) {
    delay(20);
    newState = digitalRead(BUTTON_PIN);
    if (newState == LOW) {
      packet_color[1] = random(0, 255); // R
      packet_color[2] = random(0, 255); // G
      packet_color[3] = random(0, 255); // B
      packet_color[4] = 0; // W

      for (int i = 0; i < RING_LEDS; i++) {
        ring.setPixelColor(i, ring.Color(packet_color[1], packet_color[2], packet_color[3], packet_color[4]));
      }
      ring.show();

      packetnum += 2;
      packetnum = packetnum % 7;

      char radiopacket[20] = "Blink #";
      itoa(packetnum, radiopacket + 7, 10);
      Serial.print("Sending "); Serial.printf("R%d G%d B%d W%d\n", packet_color[1], packet_color[2], packet_color[3], packet_color[4]);

      // Send a message!
      rf69.send((uint8_t *)packet_color, 5);
      rf69.waitPacketSent();

      // Now wait for a reply
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf69.waitAvailableTimeout(500))  {
        // Should be a reply message for us now
        if (rf69.recv(buf, &len)) {
          Serial.print("Got a reply: ");
          Serial.print((char*)buf);
          Blink(LED, 50, packetnum); //blink LED 3 times, 50ms between blinks
        } else {
          Serial.println("Receive failed");
        }
      } else {
        Serial.println("No reply, is another RFM69 listening?");
      }
    }
  }
  oldState = newState;
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}

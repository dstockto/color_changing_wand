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

char buff[50];
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

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.begin(115200);

    // If you are connected to a computer and want to see all the messages, uncomment the line below. If you are not
    // connected and you have the line below enabled, then your program will hang here.
//  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

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
    Serial.println("Looking for color sensor");
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
    }

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                     0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
    };
    rf69.setEncryptionKey(key);

    pinMode(LED, OUTPUT);

    Serial.print("RFM69 radio @");
    Serial.print((int) RF69_FREQ);
    Serial.println(" MHz");

    ring.begin();
    // When the wand is ready to go, there will be a single pixel turned on to yellow. Feel free to change this however
    // you want.
    ring.setPixelColor(0, ring.Color(255, 255, 0, 0));
    ring.setPixelColor(8, ring.Color(255, 255, 0, 0));
    ring.show();
    ring.setBrightness(40);
}

// CRGBW

uint8_t packet_color[5];
boolean oldState = HIGH;

uint16_t scanRed, scanGreen, scanBlue, clear;

void loop() {
    packet_color[0] = (uint8_t) 'C';
    delay(100);  // Wait .1 second between transmits, could also 'sleep' here!

    boolean newState = digitalRead(BUTTON_PIN);

    if ((newState == LOW) && (oldState == HIGH)) {
        // Debounce the button - it will be high and low really quickly when pushed because the switch literally bounces
        // so we will wait 20ms to let it settle down and make sure we are still getting the same value
        delay(20);
        newState = digitalRead(BUTTON_PIN);
        // LOW means the button is pushed because it's wired to ground
        if (newState == LOW) {
            tcs.setInterrupt(false); // scanner LED on
            delay(60);
            tcs.getRawData(&scanRed, &scanGreen, &scanBlue, &clear);
            tcs.setInterrupt(true); // scanner LED off

            uint32_t sum = scanRed + scanGreen + scanBlue;

            Serial.print("R");
            Serial.println(scanRed);

            Serial.print("G");
            Serial.println(scanGreen);

            Serial.print("B");
            Serial.println(scanBlue);
            
            float r, g, b, largest;
//            r = scanRed;
//            r /= sum;
//            g = scanGreen;
//            g /= sum;
//            b = scanBlue;
//            b /= sum;
//            r *= 256;
//            g *= 256;
//            b *= 256;
            r = scanRed / 255;
            g = scanGreen / 255;
            b = scanBlue / 255;
            largest = r;
            if (g > largest) {
              largest = g;
            }
            if (b > largest) {
              largest = b;
            }
            float  mult = 255.0 / largest;
            
     

//            float mult = 1.0; // This "brightens" the color by about 50%

            packet_color[1] = gammatable[(int) (r * mult)]; // R
            packet_color[2] = gammatable[(int) (g * mult)]; // G
            packet_color[3] = gammatable[(int) (b * mult)]; // B
            packet_color[4] = 0; // W - don't turn on the white part

            // Set the pixel ring to the color we got from the scanner
            for (int i = 0; i < RING_LEDS; i++) {
                ring.setPixelColor(i, ring.Color(packet_color[1], packet_color[2], packet_color[3], packet_color[4]));
            }
            ring.show();

            Serial.print("Sending ");
            sprintf(buff, "R%d G%d B%d W%d\n", (int)packet_color[1], (int)packet_color[2], (int)packet_color[3], (int)packet_color[4]);
            Serial.println(buff);

            // Send a message! - Message looks like C<R><G><B> where C is literally the letter C and the <R>, <G> and <B>
            // values are single byte values representing 0-255 for each color value
            rf69.send((uint8_t *) packet_color, 5);
            rf69.waitPacketSent();

            // Now wait for a reply
            uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
            uint8_t len = sizeof(buf);

            if (rf69.waitAvailableTimeout(500)) {
                // Should be a reply message for us now
                if (rf69.recv(buf, &len)) {
                    Serial.print("Got a reply: ");
                    Serial.print((char *) buf);
                    Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
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
    for (byte i = 0; i < loops; i++) {
        digitalWrite(PIN, HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN, LOW);
        delay(DELAY_MS);
    }
}

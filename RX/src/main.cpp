#include <Arduino.h>
#include <CRC.h>
#include <FastLED.h>
#include <SoftwareSerial.h>

#define NUM_LEDS 28

#define DATA_PIN 3

SoftwareSerial xbee(12, 11);

// uint8_t rxbuff[256];
typedef enum
{
    DRIVE_STOP,
    DRIVE_AUTO,
    DRIVE_MANUAL,
} drive_state;
typedef enum
{
    LED_AUTO,
    LED_RED,
    LED_DISCO
} led_state;

#define CH_B2 10
#define CH_B1 9

#define ENABLE 7
#define CH_A2  6
#define CH_A1  5

CRGB leds[NUM_LEDS];

void motorsWrite(int32_t left, int32_t right)
{
    bool fwdL = left > 0;
    bool fwdR = right > 0;
    left = abs(left);
    right = abs(right);
    if (left > 255)
        left = 255;
    if (right > 255)
        right = 255;

    if (fwdL)
    {
        analogWrite(CH_A1, left);
        analogWrite(CH_B1, 0);
    }
    else
    {
        analogWrite(CH_A1, 0);
        analogWrite(CH_B1, left);
    }
    if (fwdR)
    {
        analogWrite(CH_A2, right);
        analogWrite(CH_B2, 0);
    }
    else
    {
        analogWrite(CH_A2, 0);
        analogWrite(CH_B2, right);
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    xbee.begin(38400);
    Serial.begin(115200);

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed

    pinMode(ENABLE, OUTPUT);
    pinMode(CH_A1, OUTPUT);
    pinMode(CH_A2, OUTPUT);
    pinMode(CH_B1, OUTPUT);
    pinMode(CH_B2, OUTPUT);
    digitalWrite(ENABLE, LOW);
    motorsWrite(0, 0);
}
void loop()
{
    static uint8_t ledState = LED_AUTO;
    static uint8_t driveState = DRIVE_STOP;

    static uint32_t last_update = 0;
    static uint32_t last_xbee_rx = 0;
    static uint32_t last_xbee_char = 0;
    static uint32_t last_auto_rx = 0;
    static uint32_t last_auto_char = 0;
    static uint32_t last_ka = 0;

    static int32_t manualLeft = 0;
    static int32_t manualRight = 0;
    static int32_t autoLeft = 0;
    static int32_t autoRight = 0;

    static uint8_t xbeebuf[256];
    static size_t xbeebufPos = 0;
    static uint8_t autobuf[256];
    static size_t autobufPos = 0;
    static char snpb[256];
    static bool remote_kill = true;
    static bool auto_kill = true;
    static uint32_t last_led = 0;
    static uint32_t last_led_blink = 0;
    static bool blink = false;
    static uint8_t hue_off = 0;

    /* digitalWrite(ENABLE, HIGH);
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(i, 0);
        delay(10);
    }
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(255 - i, 0);
        delay(10);
    }
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(-i, 0);
        delay(10);
    }
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(-255 + i, 0);
        delay(10);
    }
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(0, i);
        delay(10);
    }
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(0, 255 - i);
        delay(10);
    }
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(0, -i);
        delay(10);
    }
    for (int i = 0; i < 256; i++)
    {
        motorsWrite(0, -255 + i);
        delay(10);
    }
    motorsWrite(0, 0);
    digitalWrite(ENABLE, LOW);
    delay(1000);
    return; */

    // Turn the LED on, then pause
    if (millis() - last_led_blink >= 500)
    {
        last_led_blink = millis();
        blink = !blink;
    }
    if (millis() - last_led >= 10)
    {
        hue_off += 1;
        last_led = millis();
        static uint8_t brightness = 150;
        if (remote_kill)
        {
            fill_solid(leds, NUM_LEDS, blink ? CRGB::Red : CRGB::Black);
        }
        else if (driveState == DRIVE_AUTO && auto_kill)
        {
            fill_solid(leds, NUM_LEDS, blink ? CRGB::Orange : CRGB::Black);
        }
        else
        {
            if (ledState == LED_DISCO)
            {
                CHSV hsv1(-hue_off, 255, brightness);
                CHSV hsv2(-hue_off + 150, 255, brightness);
                fill_gradient(leds, NUM_LEDS, hsv1, hsv2, FORWARD_HUES);
            }
            else if (ledState == LED_RED)
            {
                CRGB hsv1(10, 0, 0);
                fill_solid(leds, NUM_LEDS, hsv1);
            }
            else
            {
                if (driveState == DRIVE_AUTO)
                {
                    CHSV hsv1(0, 255, brightness);
                    fill_solid(leds, NUM_LEDS, hsv1);
                }
                else if (driveState == DRIVE_MANUAL)
                {
                    CRGB rgb(0, brightness, 0);
                    fill_solid(leds, NUM_LEDS, rgb);
                }

                else
                {
                    // CRGB rgb(0, brightness, 0);
                    fill_solid(leds, NUM_LEDS, CRGB::Black);
                    // CHSV hsv1(-hue_off, 255, brightness);
                    // CHSV hsv2(-hue_off + 120, 255, brightness);
                    // fill_gradient(leds, NUM_LEDS, hsv1, hsv2, FORWARD_HUES);
                }
            }
        }
        FastLED.show();
    }

    digitalWrite(LED_BUILTIN, (millis() - last_xbee_rx) < 300);
    if (millis() - last_update >= 10)
    {
        last_update = millis();
        static int i = 0;
        i++;
        i %= 100;
        if (i == 0)
            xbee.println(millis());
        int32_t left, right;
        if (driveState == DRIVE_AUTO)
        {
            digitalWrite(ENABLE, HIGH);
            left = autoLeft;
            right = autoRight;
        }
        else if (driveState == DRIVE_MANUAL)
        {
            digitalWrite(ENABLE, HIGH);
            left = manualLeft;
            right = manualRight;
        }
        else
        {
            digitalWrite(ENABLE, LOW);
            left = 0;
            right = 0;
        }
        if (remote_kill)
        {
            digitalWrite(ENABLE, LOW);
            left = 0;
            right = 0;
        }
        motorsWrite(left, right);
    }

    while (xbee.available())
    {
        uint8_t c = xbee.read();
        last_xbee_char = millis();
        if (xbeebufPos == 0)
        {
            // Serial.println("Buf start");
            if (c == 0xAA)
            {
                // Serial.println("RX hdr");
                xbeebuf[xbeebufPos++] = c;
            }
        }
        else if (xbeebufPos < 11)
        {
            // Serial.print(c, HEX);
            // Serial.print(" ");
            // Serial.println(bufPos);
            xbeebuf[xbeebufPos++] = c;
        }
        else
        {
            xbeebufPos = 0;
            uint8_t crc = crc8(xbeebuf, 11);
            // snprintf(snpb, 256, "RX %02X, calc %02X", c, crc);
            // Serial.println(snpb);
            if (crc == c)
            {
                last_xbee_rx = millis();
                driveState = xbeebuf[1];
                ledState = xbeebuf[2];
                memcpy(&manualLeft, xbeebuf + 3, 4);
                memcpy(&manualRight, xbeebuf + 7, 4);
                Serial.print(manualLeft);
                Serial.print(" ");
                Serial.print(manualRight);
                Serial.println(" XB_RECV");
                xbee.println("XB_RECV");
                remote_kill = false;
                break;
            }
        }
    }
    while (Serial.available())
    {
        uint8_t c = Serial.read();
        last_auto_char = millis();
        if (autobufPos == 0)
        {
            // Serial.println("Buf start");
            if (c == 0xAA)
            {
                // Serial.println("RX hdr");
                autobuf[autobufPos++] = c;
            }
        }
        else if (autobufPos < 9)
        {
            // Serial.print(c, HEX);
            // Serial.print(" ");
            // Serial.println(bufPos);
            autobuf[autobufPos++] = c;
        }
        else
        {
            autobufPos = 0;
            uint8_t crc = crc8(autobuf, 9);
            // snprintf(snpb, 256, "RX %02X, calc %02X", c, crc);
            // Serial.println(snpb);
            if (crc == c)
            {
                last_auto_rx = millis();
                memcpy(&autoLeft, autobuf + 1, 4);
                memcpy(&autoRight, autobuf + 5, 4);
                Serial.println("SER_RECV");
                xbee.println("SER_RECV");
                // Serial.println("RECV");
                auto_kill = false;
                break;
            }
        }
    }
    if (millis() - last_xbee_char > 50 && xbeebufPos)
    {
        xbeebufPos = 0;
        Serial.println("RX TIMEOUT");
        xbee.println("RX TIMEOUT");
    }
    if (millis() - last_auto_char > 50 && autobufPos)
    {
        autobufPos = 0;
        Serial.println("AUTO RX TIMEOUT");
        xbee.println("AUTO RX TIMEOUT");
    }
    if (millis() - last_xbee_rx > 300 && driveState != DRIVE_STOP)
    {
        Serial.println("SAFETY KILL");
        xbee.println("SAFETY KILL");
        remote_kill = true;
        driveState = DRIVE_STOP;
        manualLeft = 0;
        manualRight = 0;
    }
    if (millis() - last_auto_rx > 300 && (autoLeft || autoRight))
    {
        Serial.println("AUTO KILL");
        xbee.println("AUTO KILL");
        auto_kill = true;
        autoLeft = 0;
        autoRight = 0;
    }

    //     while (Serial.available())
    //     {
    //         // Serial.write(Serial.peek());
    //         xbee.write(Serial.read());
    // }
}
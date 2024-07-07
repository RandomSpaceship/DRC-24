#include <Arduino.h>
#include <CRC.h>
#include <FastLED.h>
#include <SoftwareSerial.h>

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
        analogWrite(CH_A2, 0);
    }
    else
    {
        analogWrite(CH_A1, 0);
        analogWrite(CH_A2, left);
    }
    if (fwdR)
    {
        analogWrite(CH_B1, left);
        analogWrite(CH_B2, 0);
    }
    else
    {
        analogWrite(CH_B1, 0);
        analogWrite(CH_B2, left);
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    xbee.begin(38400);
    Serial.begin(115200);

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
    static uint32_t last_rx = 0;
    static uint32_t last_char = 0;

    static int32_t manualLeft = 0;
    static int32_t manualRight = 0;
    static int32_t autoLeft = 0;
    static int32_t autoRight = 0;

    static uint8_t buf[256];
    static char snpb[256];
    static bool safe_kill = true;
    static size_t bufPos = 0;

    digitalWrite(LED_BUILTIN, (millis() - last_rx) < 300);
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
            left = autoLeft;
            right = autoRight;
        }
        else if (driveState == DRIVE_MANUAL)
        {
            left = manualLeft;
            right = manualRight;
        }
        else
        {
            left = 0;
            right = 0;
        }
        if (safe_kill)
        {
            left = 0;
            right = 0;
        }
        motorsWrite(left, right);
    }

    while (xbee.available())
    {
        uint8_t c = xbee.read();
        last_char = millis();
        if (bufPos == 0)
        {
            // Serial.println("Buf start");
            if (c == 0xAA)
            {
                // Serial.println("RX hdr");
                buf[bufPos++] = c;
            }
        }
        else if (bufPos < 11)
        {
            // Serial.print(c, HEX);
            // Serial.print(" ");
            // Serial.println(bufPos);
            buf[bufPos++] = c;
        }
        else
        {
            bufPos = 0;
            uint8_t crc = crc8(buf, 11);
            // snprintf(snpb, 256, "RX %02X, calc %02X", c, crc);
            // Serial.println(snpb);
            if (crc == c)
            {
                last_rx = millis();
                driveState = buf[1];
                ledState = buf[2];
                memcpy(&manualLeft, buf + 3, 4);
                memcpy(&manualRight, buf + 7, 4);
                Serial.println("RECV");
                xbee.println("RECV");
                safe_kill = false;
                break;
            }
        }
    }
    if (millis() - last_char > 50 && bufPos)
    {
        bufPos = 0;
        Serial.println("RX TIMEOUT");
    }
    if (millis() - last_rx > 300 && driveState != DRIVE_STOP)
    {
        Serial.println("SAFETY KILL");
        xbee.println("SAFETY KILL");
        safe_kill = true;
        driveState = DRIVE_STOP;
        manualLeft = 0;
        manualRight = 0;
    }

    //     while (Serial.available())
    //     {
    //         // Serial.write(Serial.peek());
    //         xbee.write(Serial.read());
    // }
}
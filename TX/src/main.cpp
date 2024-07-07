#include <Arduino.h>
#include <CRC.h>
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

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    xbee.begin(38400);
    Serial.begin(115200);

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
}
void loop()
{
    static uint8_t ledState = LED_AUTO;
    static uint8_t driveState = DRIVE_STOP;

    static uint32_t last_send = 0;
    static uint32_t last_rx = 0;

    static int32_t xPos = 0;
    static int32_t yPos = 0;
    static uint8_t buf[256];

    if (millis() - last_send >= 10)
    {
        last_send = millis();
        buf[0] = 0xAA;
        buf[1] = driveState;
        buf[2] = ledState;
        memcpy(buf + 3, &xPos, 4);
        memcpy(buf + 7, &yPos, 4);
        buf[11] = crc8(buf, 11);
        // xbee.write(buf, 13);
        xbee.write(buf, 12);
    }
    digitalWrite(LED_BUILTIN, (millis() - last_rx) < 500);

    if (!digitalRead(2))
        driveState = DRIVE_MANUAL;
    else if (!digitalRead(3))
        driveState = DRIVE_MANUAL;
    else
        driveState = DRIVE_STOP;
    if (!digitalRead(4))
        ledState = LED_DISCO;
    else if (!digitalRead(5))
        ledState = LED_RED;
    else
        driveState = LED_AUTO;

    while (xbee.available())
    {
        last_rx = millis();
        Serial.write(xbee.read());
    }

    int16_t xRaw = analogRead(A0) - 512;
    int16_t yRaw = analogRead(A1) - 512;
    char snpb[128];
    snprintf(snpb, 128, "%+04d, %+04d", xRaw, yRaw);
    Serial.println(snpb);
    if (abs(xRaw) < 50)
        xRaw = 0;
    if (abs(yRaw) < 50)
        yRaw = 0;
    xPos = xRaw;
    yPos = yRaw;

    // while (Serial.available())
    // {
    //     // Serial.write(Serial.peek());
    //     xbee.write(Serial.read());
    // }
}
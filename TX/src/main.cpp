#include <Arduino.h>
#include <CRC.h>
#include <SoftwareSerial.h>

SoftwareSerial xbee(10, 9);

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
    xbee.begin(38400);
    Serial.begin(38400);
    pinMode(LED_BUILTIN, OUTPUT);

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

    static int32_t left = 0;
    static int32_t right = 0;
    static uint8_t buf[256];

    static char snpb[256];

    int16_t xRaw = analogRead(A0) - 512;
    int16_t yRaw = analogRead(A1) - 512;
    int x = xRaw;
    int y = yRaw;
    if (abs(x) < 45)
        x = 0;
    if (abs(y) < 45)
        y = 0;

    const static int xRange = 100;
    const static int yRange = 150;
    int turn = map(x, -512, 511, -xRange, xRange);
    int fwd = map(y, -512, 511, -yRange, yRange);
    left = fwd + turn;
    right = fwd - turn;

    if (!digitalRead(2))
        driveState = DRIVE_MANUAL;
    else if (!digitalRead(3))
        driveState = DRIVE_AUTO;
    else
        driveState = DRIVE_STOP;

    if (!digitalRead(4))
        ledState = LED_DISCO;
    else if (!digitalRead(5))
        ledState = LED_RED;
    else
        ledState = LED_AUTO;

    snprintf(snpb, 256, "L%d D%d  |  %+04d, %+04d  |  %+04ld, %+04ld", ledState, driveState, x, y, left, right);

    if (millis() - last_send >= 10)
    {
        last_send = millis();
        buf[0] = 0xAA;
        buf[1] = driveState;
        buf[2] = ledState;
        memcpy(buf + 3, &left, 4);
        memcpy(buf + 7, &right, 4);
        buf[11] = crc8(buf, 11);
        // xbee.write(buf, 13);
        // Serial.println(snpb);
        xbee.write(buf, 12);
        Serial.write(buf, 12);
    }
    digitalWrite(LED_BUILTIN, (millis() - last_rx) < 500);

    while (xbee.available())
    {
        last_rx = millis();
        Serial.write(xbee.read());
    }

    // int16_t xRaw = analogRead(A0) - 512;
    // int16_t yRaw = analogRead(A1) - 512;
    // snprintf(snpb, 256, "%+04d, %+04d", xRaw, yRaw);
    // Serial.println(snpb);

    // while (Serial.available())
    // {
    //     // Serial.write(Serial.peek());
    //     xbee.write(Serial.read());
    // }
}
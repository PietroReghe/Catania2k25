#include <Wire.h>
#include <MPU6050.h>

#define DIR_PIN_1 2
#define STEP_PIN_1 3
#define DIR_PIN_2 4
#define STEP_PIN_2 5
#define ENABLE_PIN 6

MPU6050 mpu;
float setpoint = 0;
float input, output;
float Kp = 1.5, Ki = 0.1, Kd = 0.5;
float previousError = 0, integral = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }

    pinMode(DIR_PIN_1, OUTPUT);
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_2, OUTPUT);
    pinMode(STEP_PIN_2, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
}

float getGyroAngle() {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    return gz / 131.0; // Convert raw data to degrees per second
}

float PIDControl(float setpoint, float input) {
    float error = setpoint - input;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

void stepMotors(int speedLeft, int speedRight) {
    digitalWrite(DIR_PIN_1, speedLeft > 0 ? HIGH : LOW);
    digitalWrite(DIR_PIN_2, speedRight > 0 ? HIGH : LOW);
    int stepsLeft = abs(speedLeft);
    int stepsRight = abs(speedRight);

    for (int i = 0; i < max(stepsLeft, stepsRight); i++) {
        if (i < stepsLeft) digitalWrite(STEP_PIN_1, HIGH);
        if (i < stepsRight) digitalWrite(STEP_PIN_2, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN_1, LOW);
        digitalWrite(STEP_PIN_2, LOW);
        delayMicroseconds(500);
    }
}

void loop() {
    input = getGyroAngle();
    output = PIDControl(setpoint, input);

    int baseSpeed = 50;
    int speedRight = baseSpeed + output;
    int speedLeft = baseSpeed - output;

    stepMotors(speedLeft, speedRight);
}

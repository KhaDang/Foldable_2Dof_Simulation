#include <Arduino.h>
#include "platform_kinematics.h"

// Only supports SX1276/SX1278
#include <LoRa.h>
#include "LoraReceiverMod/LoRaBoards.h"

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           433.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   20
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif


#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples"
#endif

double kalmanX = 0.0;
double kalmanY = 0.0;

// === Motor Definitions ===
StepperMotor motor1 = {18, 19, 800, 0.0};  // STEP, DIR, steps/rev, initial angle
StepperMotor motor2 = {21, 22, 800, 0.0};  // Adjust pins as needed

PlatformParams params1 = {50, 20, 30, 25, 35, 15, 60, 70};
PlatformParams params2 = {48, 19, 29, 26, 34, 16, 60, 70};

// === Motor Control ===
void setupMotor(const StepperMotor& m) {
    pinMode(m.stepPin, OUTPUT);
    pinMode(m.dirPin, OUTPUT);

     setupBoards();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.println("LoRa Receiver");

#ifdef  RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(CONFIG_RADIO_FREQ * 1000000)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);

    LoRa.setSignalBandwidth(CONFIG_RADIO_BW * 1000);

    LoRa.setSpreadingFactor(10);

    LoRa.setPreambleLength(16);

    LoRa.setSyncWord(0xAB);

    LoRa.disableCrc();

    LoRa.disableInvertIQ();

    LoRa.setCodingRate4(7);

    // put the radio into receive mode
    LoRa.receive();

}

void moveMotorToAngle(StepperMotor& m, float targetAngle) {
    float deltaAngle = targetAngle - m.currentAngle;

    // Normalize to [-180, 180] for shortest path
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;

    int steps = abs(deltaAngle) * m.stepsPerRev / 360.0;
    bool direction = deltaAngle > 0;

    digitalWrite(m.dirPin, direction ? HIGH : LOW);

    for (int i = 0; i < steps; ++i) {
        digitalWrite(m.stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(m.stepPin, LOW);
        delayMicroseconds(1000);
    }

    m.currentAngle = targetAngle;  // Update motor state
}

// === Setup & Loop ===
void setup() {
    Serial.begin(115200);

    setupMotor(motor1);
    setupMotor(motor2);

    delay(1000); // optional pause

}

void loop() {
    
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        String recv = "";
        // read packet
        while (LoRa.available()) {
            recv += (char)LoRa.read();
        }
        
        Serial.println(recv);
        
        // print RSSI of packet, bring some information to T-Display
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
        if (u8g2) {
            u8g2->clearBuffer();
            char buf[256];
            u8g2->drawStr(0, 12, "Received OK!");
            // u8g2->drawStr(0, 26, recv.c_str());
            snprintf(buf, sizeof(buf), "RSSI:%i", LoRa.packetRssi());
            u8g2->drawStr(0, 40, buf);
            snprintf(buf, sizeof(buf), "SNR:%.1f", LoRa.packetSnr());
            u8g2->drawStr(0, 56, buf);
            u8g2->sendBuffer();
        }
        // parse the received data to kalmanX and kalmanY

        int sepIndex = recv.indexOf(';');
        int endIndex = recv.indexOf('X');

        if (sepIndex != -1 && endIndex != -1 && sepIndex < endIndex) {
        String xStr = recv.substring(0, sepIndex);
        String yStr = recv.substring(sepIndex + 1, endIndex);

        kalmanX = xStr.toFloat();
        kalmanY = yStr.toFloat();

        Serial.print("Parsed kalmanX: ");
        Serial.println(kalmanX);
        Serial.print("Parsed kalmanY: ");
        Serial.println(kalmanY);
        
        // compute the angle to drive motors
 
        float roll = kalmanX * DEG_TO_RAD;
        float pitch = kalmanY * DEG_TO_RAD;

        float alpha1, alpha2;
        computeMotorAngles(roll, pitch, params1, params2, alpha1, alpha2);

        float deg1 = alpha1 * RAD_TO_DEG;
        float deg2 = alpha2 * RAD_TO_DEG;

        Serial.print("Target α1: "); Serial.print(deg1); Serial.println("°");
        Serial.print("Target α2: "); Serial.print(deg2); Serial.println("°");

        moveMotorToAngle(motor1, deg1);
        moveMotorToAngle(motor2, deg2);

        Serial.println("Motors moved.");

    }

}
}
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- PIN CONFIGURATION ---
const int PIN_SERVO_L = 10;
const int PIN_SERVO_R = 40;
const int PIN_SDA      = 24;
const int PIN_SCL      = 23;

// --- FLIGHT CORE OBJECTS ---
Servo servoLeft;
Servo servoRight;

// --- BLE CONFIGURATION ---
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb" 
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

bool deviceConnected = false;
float targetRollInput = 0.0;   
float targetThrottleInput = 0.0; 

// --- IMU & PID FLIGHT DATA ---
const int MPU_ADDR = 0x68; 
float gyroX, gyroY; 
float pidOutputRoll = 0.0;
float pidOutputPitch = 0.0;

// PID Tuning Constants
const float Kp_Roll  = 0.8,  Kd_Roll  = 0.2;
const float Kp_Pitch = 0.5,  Kd_Pitch = 0.1;

float lastErrorRoll = 0.0;
float lastErrorPitch = 0.0;
unsigned long lastTime;

// --- ORNITHOPTER KINEMATICS CONFIGURATION ---
const int WING_CENTER_L      = 90;  
const int WING_CENTER_R      = 90;  
const float MAX_FLAP_FREQ    = 6.0; 
const float MAX_AMPLITUDE    = 40.0; 
float runningPhase = 0.0;           

class FlightServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

class FlightControlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() >= 2) {
            int8_t rawX = (int8_t)value[0];
            int8_t rawY = (int8_t)value[1];
            
            targetRollInput     = constrain(rawX / 100.0f, -1.0f, 1.0f);
            targetThrottleInput = constrain((rawY / 100.0f), 0.0f, 1.0f); 
        }
    }
};

void initMPU() {
    Wire.begin(PIN_SDA, PIN_SCL, 400000);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0);    
    Wire.endTransmission(true);
}

void readIMUData() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 4, true);
    
    int16_t rawX = Wire.read() << 8 | Wire.read();
    int16_t rawY = Wire.read() << 8 | Wire.read();
    
    gyroX = rawX / 131.0f; 
    gyroY = rawY / 131.0f;
}

void setup() {
    Serial.begin(115200);
    lastTime = micros();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servoLeft.setPeriodHertz(50);
    servoRight.setPeriodHertz(50);
    servoLeft.attach(PIN_SERVO_L, 500, 2500);
    servoRight.attach(PIN_SERVO_R, 500, 2500);

    initMPU();

    BLEDevice::init("BionicButterfly-FC");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new FlightServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pChar->setCallbacks(new FlightControlCallbacks());
    pChar->addDescriptor(new BLE2902());
    pService->start();
    
    pServer->getAdvertising()->start();
}

void loop() {
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastTime = currentTime;

    readIMUData();

    float targetRollRate = targetRollInput * 40.0f; 
    float errorRoll = targetRollRate - gyroX;
    float derivRoll = (errorRoll - lastErrorRoll) / dt;
    pidOutputRoll = (Kp_Roll * errorRoll) + (Kd_Roll * derivRoll);
    lastErrorRoll = errorRoll;

    float errorPitch = (0.0f) - gyroY; 
    float derivPitch = (errorPitch - lastErrorPitch) / dt;
    pidOutputPitch = (Kp_Pitch * errorPitch) + (Kd_Pitch * derivPitch);
    lastErrorPitch = errorPitch;

    float activeFrequency = targetThrottleInput * MAX_FLAP_FREQ;
    float activeAmplitude = targetThrottleInput * MAX_AMPLITUDE;

    runningPhase += 2.0f * PI * activeFrequency * dt;
    if (runningPhase > 2.0f * PI) runningPhase -= 2.0f * PI;

    float baseFlapWave = sin(runningPhase);

    float commandL = WING_CENTER_L + (baseFlapWave * activeAmplitude) + pidOutputRoll + pidOutputPitch;
    float commandR = WING_CENTER_R - (baseFlapWave * activeAmplitude) + pidOutputRoll - pidOutputPitch;

    int safeServoPosL = constrain(round(commandL), 15, 165);
    int safeServoPosR = constrain(round(commandR), 15, 165);

    servoLeft.write(safeServoPosL);
    servoRight.write(safeServoPosR);

    delay(10); 
}
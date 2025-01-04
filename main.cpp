//Required libraries
#include "I2Cdev.h"
#include "Servo.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "MPU6050_6Axis_MotionApps20.h"

#include "helper_3dmath.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//mpu6050 I2C address is 0x68
MPU6050 mpu;
Adafruit_BMP280 bmp;

// PID variables
float Kp = 500.0f; // Proportional gain
float Kd = 0.5f; // Derivative gain
float Ki = 0.0f; // Integral gain


VectorFloat integralErr = {0.0f, 0.0f, 0.0f};

Quaternion worldQuaternion = {0, 0, 0, 1};
Quaternion refQuaternion;


float targetPitch = 0.0;  // Target pitch orientation (degrees)
float targetRoll = 0.0;   // Target roll orientation (degrees)
float lastPitchError = 0.0;
float lastRollError = 0.0;
float pitchIntegral = 0.0;
float rollIntegral = 0.0;
float dt;

unsigned long lastUpdateTime = 0;

float altitudeOffset = 0.0;

// Declare servo objects
Servo pitchServo1;
Servo pitchServo2;
Servo yawServo1;
Servo yawServo2;

// Servo pin assignments
#define PITCH_SERVO1_PIN 3
#define PITCH_SERVO2_PIN 4
#define ROLL_SERVO1_PIN 2
#define ROLL_SERVO2_PIN 5

#define LED_PIN 13 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===                    HELPER METHODS                        ===
// ================================================================
void setupServos() {

    pitchServo1.attach(PITCH_SERVO1_PIN);
    pitchServo2.attach(PITCH_SERVO2_PIN);
    yawServo1.attach(ROLL_SERVO1_PIN);
    yawServo2.attach(ROLL_SERVO2_PIN);

    // Initialize servos to neutral pos
    pitchServo1.write(90);
    pitchServo2.write(90);
    yawServo1.write(90);
    yawServo2.write(90);

}

//Adjusts servos with signals from PID controller
void adjustServos(float pitchControlSignal, float rollControlSignal) {
    int pitchAdjustment = (int)pitchControlSignal;
    int rollAdjustment = (int)rollControlSignal;

    // Map adjustments to be between 15 and 165 degrees
    int pitchServo1Angle = constrain(90 + pitchAdjustment, 30, 150);
    int pitchServo2Angle = constrain(90 - pitchAdjustment, 30, 150);
    int rollServo1Angle = constrain(90 + rollAdjustment, 30, 150);
    int rollServo2Angle = constrain(90 - rollAdjustment, 30, 150);

    pitchServo1.write(pitchServo1Angle);
    pitchServo2.write(pitchServo2Angle);
    yawServo1.write(rollServo1Angle);
    yawServo2.write(rollServo2Angle);
}

// Cross product of two VectorFloat objects
VectorFloat cross(const VectorFloat& v1, const VectorFloat& v2) {
        
    VectorFloat cross_P;

    cross_P.x = v1.y * v2.z - v1.z * v2.y;
    cross_P.y = v1.z * v2.x - v1.x * v2.z;
    cross_P.z = v1.x * v2.y - v1.y * v2.x;

    return cross_P;
}

// Dot product of two VectorFloat objects
float dot(const VectorFloat& v1, const VectorFloat& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// Magnitude of a VectorFloat
float magnitude(const VectorFloat& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Quaternion calculatePIDWithQuaternions() {

    VectorFloat vref = {0.0f, 1.0f, 0.0f}; // Reference vector
    Quaternion worldQuat(refQuaternion.w, refQuaternion.x, refQuaternion.y, refQuaternion.z);

    // Rotate vref to body frame using the world quaternion
    Quaternion vrefQuat(0.0f, vref.x, vref.y, vref.z);
    Quaternion rotatedQuat = (worldQuat.getProduct(vrefQuat)).getProduct(worldQuat.getConjugate());
    VectorFloat vdesBody = {rotatedQuat.x, rotatedQuat.y, rotatedQuat.z};

    // Calculate the rotation vector (vrot)
    VectorFloat vrot = cross(vref, vdesBody);

    // Calculate rotation quaternion's w-component
    float vrefLength = magnitude(vref);
    float vdesBodyLength = magnitude(vdesBody);
    float dotProd = dot(vref, vdesBody);
    float w = std::sqrt((vrefLength * vrefLength) * (vdesBodyLength * vdesBodyLength)) + dotProd;

    // Create and normalize rotation quaternion
    Quaternion rotQuat(w, vrot.x, vrot.y, vrot.z);
    rotQuat.normalize();

    return rotQuat;

}

void calibrateAltimeter() {
    Serial.println(F("Calibrating altimeter..."));
    float totalAltitude = 0.0;
    int sampleCount = 100;

    for (int i = 0; i < sampleCount; i++) {
        totalAltitude += bmp.readAltitude(1013.25); // Default sea level pressure
        delay(10); // Small delay between samples
    }

    altitudeOffset = totalAltitude / sampleCount;
    Serial.print(F("Altitude offset set to: "));
    Serial.println(altitudeOffset);
}

void controlServosWithPID(Quaternion rotQuat, VectorFloat gyro) {

    // Step 1: Compute the error vector (Verr) from rotQuat
    VectorFloat Verr = {rotQuat.x, rotQuat.y, rotQuat.z};

    // Step 2: Derivative term (angular velocity from gyro)
    VectorFloat angularVelocity = gyro;

    // Step 3: Integral term (accumulate error over time)
    float currentTime = millis() / 1000.0f; // Time in seconds
    float deltaTime = currentTime - lastUpdateTime;
    integralErr.x += Verr.x * deltaTime;
    integralErr.y += Verr.y * deltaTime;
    integralErr.z += Verr.z * deltaTime;
    lastUpdateTime = currentTime;

    // Step 4: Compute control signals for pitch and yaw
    float pitchControlSignal = Kp * Verr.x + Kd * angularVelocity.x + Ki * integralErr.x;
    float yawControlSignal = Kp * Verr.z + Kd * angularVelocity.z + Ki * integralErr.z;

    // Step 5: Map control signals to servo angles
    // Assuming servos are centered at 90 degrees and have a range of 0-180 degrees
    int pitchServo1Angle = constrain(90 + pitchControlSignal, 0, 180);
    int pitchServo2Angle = constrain(90 - pitchControlSignal, 0, 180); // Opposing servo
    int yawServo1Angle = constrain(90 + yawControlSignal, 0, 180);
    int yawServo2Angle = constrain(90 - yawControlSignal, 0, 180); // Opposing servo

    // Step 6: Write to servos
    pitchServo1.write(pitchServo1Angle);
    pitchServo2.write(pitchServo2Angle);
    yawServo1.write(yawServo1Angle);
    yawServo2.write(yawServo2Angle);
    
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure mpu initialized (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(60);
        mpu.CalibrateGyro(60);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    if (!bmp.begin(0x76)) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        while (1);
    }

    calibrateAltimeter();

    setupServos();

    Serial.print("SETUP COMPLETE!");
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    unsigned long currentTime = millis();
    dt = (currentTime - lastUpdateTime) / 1000.0; // Calculate true time delta in seconds
    lastUpdateTime = currentTime;


    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
            
        refQuaternion = q;

        float currentHeight = bmp.readAltitude(1013.25) - altitudeOffset; // Adjust sea level pressure if needed

        int16_t gyroX, gyroY, gyroZ;
        mpu.getRotation(&gyroX, &gyroY, &gyroZ);

        float gyroX_dps = gyroX / 16.4; //Sensitivity is 16.4 LSB/degrees/sec
        float gyroY_dps = gyroY / 16.4;
        float gyroZ_dps = gyroZ / 16.4;

        VectorFloat gyro = {gyroX_dps, gyroY_dps, gyroZ_dps};
        controlServosWithPID(calculatePIDWithQuaternions(), gyro);


        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

    }

}
    


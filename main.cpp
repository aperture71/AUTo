//Required libraries
#include "I2Cdev.h"
#include "Servo.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//mpu6050 I2C address is 0x68
MPU6050 mpu;

// PID variables
float kp = 2.0;  // Proportional gain
float ki = 0.0;  // Integral gain
float kd = 0.1;  // Derivative gain

float targetPitch = 0.0;  // Target pitch orientation (degrees)
float targetRoll = 0.0;   // Target roll orientation (degrees)
float lastPitchError = 0.0;
float lastRollError = 0.0;
float pitchIntegral = 0.0;
float rollIntegral = 0.0;
float dt;

unsigned long lastUpdateTime = 0;

// Declare servos
Servo pitchServo1;
Servo pitchServo2;
Servo rollServo1;
Servo rollServo2;

// Servo pin assignments
#define PITCH_SERVO1_PIN 2
#define PITCH_SERVO2_PIN 5
#define ROLL_SERVO1_PIN 3
#define ROLL_SERVO2_PIN 4


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


#define LED_PIN 13 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//HELPER METHODS
void setupServos() {
    pitchServo1.attach(PITCH_SERVO1_PIN);
    pitchServo2.attach(PITCH_SERVO2_PIN);
    rollServo1.attach(ROLL_SERVO1_PIN);
    rollServo2.attach(ROLL_SERVO2_PIN);

    // Initialize servos to neutral pos
    pitchServo1.write(90);
    pitchServo2.write(90);
    rollServo1.write(90);
    rollServo2.write(90);

}

void adjustPitchServos(float controlSignal) {
    int adjustment = (int)controlSignal;

    // Constrain between 15 and 165 deg
    int pitchServo1Angle = constrain(90 + adjustment, 15, 165);
    int pitchServo2Angle = constrain(90 - adjustment, 15, 165);

    pitchServo1.write(pitchServo1Angle);
    pitchServo2.write(pitchServo2Angle);
}

void adjustRollServos(float controlSignal) {
    int adjustment = (int)controlSignal;

    // Constrain between 15 and 165 deg
    int rollServo1Angle = constrain(90 + adjustment, 15, 165);
    int rollServo2Angle = constrain(90 - adjustment, 15, 165);

    rollServo1.write(rollServo1Angle);
    rollServo2.write(rollServo2Angle);
}

void calculatePitchPID(float currentQuaternionW, float currentQuaternionX) {
   
    // Convert quat to deg
    float currentPitch = atan2(2.0 * (currentQuaternionW * currentQuaternionX), 1.0 - 2.0 * (currentQuaternionX * currentQuaternionX)) * 180 / M_PI;
    float pitchError = targetPitch - currentPitch;
    pitchIntegral += pitchError * dt;
    float pitchDerivative = (pitchError - lastPitchError) / dt;

    float pitchControlSignal = kp * pitchError + ki * pitchIntegral + kd * pitchDerivative;

    adjustPitchServos(pitchControlSignal);

    Serial.print(">PITCH: ");
    Serial.println(pitchControlSignal);

    lastPitchError = pitchError;
}

void calculateRollPID(float currentQuaternionW, float currentQuaternionY) {

    // Convert quat to deg
    float currentRoll = atan2(2.0 * (currentQuaternionW * currentQuaternionY), 1.0 - 2.0 * (currentQuaternionY * currentQuaternionY)) * 180 / M_PI;
    float rollError = targetRoll - currentRoll;
    rollIntegral += rollError * dt;
    float rollDerivative = (rollError - lastRollError) / dt;

    float rollControlSignal = kp * rollError + ki * rollIntegral + kd * rollDerivative;

    adjustRollServos(rollControlSignal);

    Serial.print(">ROLL: ");
    Serial.println(rollControlSignal);

    lastRollError = rollError;
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

    // supply your own gyro offsets here, scaled for min sensitivity
    //mpu.setXGyroOffset(220);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788);

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
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            /*
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
            */

            //Calculate new dt
            unsigned long currentTime = millis();
            dt = (currentTime - lastUpdateTime) / 1000.0; // Calculate true time delta in seconds
            lastUpdateTime = currentTime;


            // calculates PID and adjusts servos
            calculatePitchPID(q.w, q.x);
            calculateRollPID(q.w, q.y);

            // Send PID values to serial for plotting
            /*
            Serial.print("PID\t");
            Serial.print(kp);
            Serial.print("\t");
            Serial.print(ki);
            Serial.print("\t");
            Serial.println(kd);
            
            Serial.print(pitchIntegral);
            Serial.print("\t");
            Serial.print(rollIntegral);
            Serial.print("\t");
            Serial.print(lastPitchError);
            Serial.print("\t");
            Serial.println(lastRollError);
            */

        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);

        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
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
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

}

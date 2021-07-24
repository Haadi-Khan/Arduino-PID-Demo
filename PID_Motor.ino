#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Encoder
#define ENCA 3 // yellow wire
#define ENCB 4 // white wire

// Motor
#define PWM 2
#define IN1 5
#define IN2 6

volatile int pos = 0;
long prevT = 0;
float prevErr = 0;
float eintegral = 0;

void setup() {
    Serial.begin(9600);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
    pinMode(PWM, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    Serial.println("target currPos");
}

void loop() {
    // set target position
    //int target = 1200;
    int target = -210;

    // PID constants
    float kp = 1;
    float ki = 0;
    float kd = 0;

    // time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    // Read the position in an atomic block to avoid a potential
    // misread if the interrupt coincides with this code running
    int currPos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currPos = pos;
    }

    // position, absement, velocity
    int err =  currPos - target;
    float absement = absement + err * deltaT;
    float velocity = (err - prevErr) / (deltaT);

    // control signal
    float u = kp * err + ki * absement +  kd * velocity;

    // motor power
    float pwr = fabs(u); // fabs() is float abs
    pwr = pwr > 255 ? 255 : pwr;

    // motor direction
    int dir = u < 0 ? -1 : 1;

    // signal the motor
    //setMotor(dir, pwr, PWM, IN1, IN2);

    // store previous error
    prevErr = err;

    Serial.print(target);
    Serial.print(" ");
    Serial.print(currPos);
    Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
    analogWrite(pwm, pwmVal);
    switch (dir) {
        case 1:
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            break;
        case -1:
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            break;
        default:
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            break;
    }
}

void readEncoder() {
    digitalRead(ENCB) ? pos++ : pos--;
}

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Encoder
#define ENCA 2 // yellow wire
#define ENCB 3 // white wire

// Motor
#define PWM 5
#define IN1 4
#define IN2 6

// Defining the target as a global variable is only useful if the target has a set location. If the target is variable, say following a trig function, define it in the loop() function.
int target = 90;
volatile int pos = 0;
long prevT = 0;
float prevErr = 0;
float absement = 0;

void setup() {
    Serial.begin(9600);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
    pinMode(PWM, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    Serial.println("u");
}

void loop() {
    int currPos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currPos = pos;
    }

    // first three parameters are kP, kI, and kD
    float u = pidControl(1, 0, 0, currPos);
    float pwr = fabs(u) > 255 ? 255 : fabs(u); // fabs() is float absolute value
    int dir = u < 0 ? -1 : 1;
    setMotor(dir, pwr, PWM, IN1, IN2);
    Serial.print(u);
    Serial.println();
}

float pidControl(float kP, float kI, float kD, int currPos) {
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    // err is our position, absement is our integral, and velocity is our derivative
    int err =  currPos - target;
    float absement = absement + err * deltaT;
    float velocity = (err - prevErr) / (deltaT);
    prevErr = err;
    return kP * err + kI * absement +  kD * velocity;
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

void display(int target, int currPos) {
    Serial.print(target);
    Serial.print(" ");
    Serial.print(currPos);
    Serial.println();
}

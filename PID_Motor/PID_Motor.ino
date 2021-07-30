#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Encoder
#define ENCA 2 // yellow wire
#define ENCB 3 // white wire

// Motor
#define IN1 13
#define IN2 12
#define PWM 11

// Defining the target as a global variable is only useful if the target has a set location. If the target is variable, say following a trig function, define it in the loop() function.
int target = 190;
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
}

void loop() {
    int currPos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currPos = pos;
    }

    // first three parameters are kP, kI, and kD
    float u = pidControl(4.12, 2.1, 1, currPos);
    float pwr = fabs(u) > 255 ? 255 : fabs(u); // This step causes issue
    int dir = u < 0 ? -1 : 1;
    setMotor(dir, pwr);
    display(target, currPos);
}

float pidControl(float kP, float kI, float kD,int currPos) {
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    // err is our position, absement is our integral, and velocity is our derivative
    int err = target - currPos;
    float absement = absement + err * deltaT;
    float velocity = (err - prevErr) / (deltaT);
    prevErr = err;
    return kP * err + kI * absement +  kD * velocity;
}

void setMotor(int dir, int pwmVal) {
    analogWrite(PWM, pwmVal);
    switch (dir) {
        case 1:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            break;
        case -1:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            break;
        default:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            break;
    }
}

void readEncoder() {
    digitalRead(ENCB) > 0 ? pos++ : pos--;
}

void display(int target, int currPos) {
    Serial.print(0);
    Serial.print(" ");
    Serial.print(target);
    Serial.print(" ");
    Serial.print(currPos);
    Serial.print(" ");
    Serial.print(target + 0.25*target); // these added numbers are to make the graph look better over time
    Serial.println();
}

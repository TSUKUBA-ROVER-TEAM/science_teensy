#include "dc_motor.h"

DCMotor::DCMotor(int pin1, int pin2, int pin3)
    : pin1_(pin1), pin2_(pin2), pin3_(pin3) {}
    
void DCMotor::init() {
    pinMode(pin1_, OUTPUT);
    pinMode(pin2_, OUTPUT);
    pinMode(pin3_, OUTPUT);
}

void DCMotor::setSpeed(int16_t speed) {
    if (speed > 0) {
        digitalWrite(pin1_, HIGH);
        digitalWrite(pin2_, LOW);
        analogWrite(pin3_, speed);
    } else if (speed < 0) {
        digitalWrite(pin1_, LOW);
        digitalWrite(pin2_, HIGH);
        analogWrite(pin3_, -speed);
    } else {
        digitalWrite(pin1_, LOW);
        digitalWrite(pin2_, LOW);
        analogWrite(pin3_, 0);
    }
}
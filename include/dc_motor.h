#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>
#include <cstdint>

class DCMotor {
public:
    DCMotor(int pin1, int pin2, int pin3);
    void init();
    void setSpeed(int16_t speed);

private:
    int pin1_;
    int pin2_;
    int pin3_;
};

#endif // DC_MOTOR_H
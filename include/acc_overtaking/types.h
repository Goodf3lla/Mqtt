#ifndef _INCLUDE__SIMCOM_TYPES_H_
#define _INCLUDE__SIMCOM_TYPES_H_

#include "v2t.h"

typedef v2t<float> vec2;

struct SensorDataIn
{
    float middleFront;
    float steeringValue;
    bool isPositionTracked;
    bool isSpeedTracked;
    vec2 leadPos;
    vec2 ownPos;
    vec2 cornerFrontRight;
    vec2 cornerFrontLeft;
    vec2 cornerRearRight;
    vec2 cornerRearLeft;
    float leadSpeed;
    float ownSpeed;
    int curGear;
    float steerLock;
    float brake;
    float brakeFL; //Front left
    float brakeFR; //Front right
    float brakeRL; //Rear left
    float brakeRR; //Rear right
};

struct CommandDataOut
{
    float steer;
    float accel;
    float brakeFL; //Front left
    float brakeFR; //Front right
    float brakeRL; //Rear left
    float brakeRR; //Rear right
    int gear;
};

#endif

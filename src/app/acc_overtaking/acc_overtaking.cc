#include <cmath>
#include <chrono>
#include <vector>
#include <acc_overtaking/acc_overtaking.h>
#include <base/log.h>
#include <util/xml_node.h>
#include <os/config.h>
#include <string.h>
#include <errno.h>
#include <acc_overtaking/types.h>
#include <algorithm>
#include <string.h>


using namespace std;
using namespace std::chrono;


float oldSensorRearLeft;
float mDistance = 0.0;
float distanceTreshold = 0.0;
float oldSensorFront;
float timeRequestResponded;
float timeRequestStarted;
bool readyToOvertake;
float sensorLeftFront;
float sensorLeftMid;
bool isMeasuringSpeed;
int speedOfCarInFront;
int size;
bool accActive;
bool isAutonomous;
bool autonomousWasDisabled;
vector<int> speedMeasurements; //change to stack?
vector<int> fallbackPoints;


acc_overtaking::acc_overtaking(const char* id) : mosquittopp(id)
{
	/* initialization */
	sem_init(&allValSem, 0, 1);
	sem_init(&allData, 0, 0);
	mosqpp::lib_init();

	/* configure mosquitto library */
	Genode::Xml_node mosquitto = Genode::config()->xml_node().sub_node("mosquitto");
	try {
		mosquitto.attribute("ip-address").value(this->host, sizeof(host));
	} catch(Genode::Xml_node::Nonexistent_attribute) {
		Genode::error("mosquitto ip-address is missing from config");
	}
	this->port = mosquitto.attribute_value<unsigned int>("port", 1883);
	this->keepalive = mosquitto.attribute_value<unsigned int>("keepalive", 60);

	/* connect to mosquitto server */
	int ret;
	do {
		ret = this->connect(host, port, keepalive);
		switch(ret) {
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto connect");
			return;
		case MOSQ_ERR_ERRNO:
			break;
			Genode::log("mosquitto ", (const char *)strerror(errno));
		}
	} while(ret != MOSQ_ERR_SUCCESS);

	/* subscribe to topic */
	do {
		ret = this->subscribe(NULL, topic1);
		switch(ret) {
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto subscribe");
			return;
		case MOSQ_ERR_NOMEM:
			Genode::error("out of memory condition occurred");
			return;
		case MOSQ_ERR_NO_CONN:
			Genode::error("not connected to a broker");
			return;
		}
	} while(ret != MOSQ_ERR_SUCCESS);

		do {
		ret = this->subscribe(NULL, topic2);
		switch(ret) {
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto subscribe");
			return;
		case MOSQ_ERR_NOMEM:
			Genode::error("out of memory condition occurred");
			return;
		case MOSQ_ERR_NO_CONN:
			Genode::error("not connected to a broker");
			return;
		}
	} while(ret != MOSQ_ERR_SUCCESS);

	/* start non-blocking loop */
	ret = this->loop_start();
	if (ret != MOSQ_ERR_SUCCESS) {
		switch(ret) {
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto loop_start");
			return;
		case MOSQ_ERR_NOT_SUPPORTED:
			Genode::error("mosquitto no thrad support");
			return;
		}
	}

	/***************
	 ** main loop **
	 ***************/
	CommandDataOut cdo;  /* command data for the next simulation step */
	char val[512];       /* buffer to convert values to string for mosq */
	
	while(true) {
		/* wait till we get all data */
		sem_wait(&allData);

		/* calculate commanddataout */
		cdo = followDriving(this->sdi);
		// driving(this->sdi);

		/* publish */
		snprintf(val, sizeof(val), "%f", cdo.steer);
		myPublish("steer", val);

		snprintf(val, sizeof(val), "%f", cdo.accel);
		myPublish("accel", val);

		snprintf(val, sizeof(val), "%f", cdo.brakeFL);
		myPublish("brakeFL", val);

		snprintf(val, sizeof(val), "%f", cdo.brakeFR);
		myPublish("brakeFR", val);

		snprintf(val, sizeof(val), "%f", cdo.brakeRL);
		myPublish("brakeRL", val);

		snprintf(val, sizeof(val), "%f", cdo.brakeRR);
		myPublish("brakeRR", val);

		snprintf(val, sizeof(val), "%d", cdo.gear);
		myPublish("gear", val);
	}
}

/* TODO */
acc_overtaking::~acc_overtaking() {
}

CommandDataOut acc_overtaking::driving(SensorDataIn sdi) {
	CommandDataOut cd = {0};

	if(!sdi.isPositionTracked)
	{
		return cd;
	}
	int sensorFront = 50;
	// Sensor Front
    if (sensorFront != 100.0 && oldSensorFront != 100.0 && sensorFront != -1.0 && oldSensorFront != -1.0) {
        if (sensorFront != oldSensorFront) {
            if (isMeasuringSpeed) {
                // time passed in seconds
                //timeRequestResponded = System.currentTimeMillis();
                const float time = (timeRequestResponded - timeRequestStarted) * 0.001;
                // distance covered by argos vehicle in m
                const int travelled = time * sdi.ownSpeed / 3.6;
                // distance covered by the vehicle in front
                const float delta = travelled - (oldSensorFront - sensorFront);
                const float speed = (delta / time) * 3.6;

                int finalSpeed = 0;
                if (speed >= 0 && sdi.ownSpeed >= speed) {
                    finalSpeed = speed;
                }

                if (finalSpeed != 0.0) {
                    // set speed of car in front temporarily in case of final measurement has not yet been made
                    speedOfCarInFront = finalSpeed;
                    size++;
                    speedMeasurements.push_back(finalSpeed);
                }

                if (speedMeasurements.size() >= 15) {
                    isMeasuringSpeed = false;
                    speedOfCarInFront = (int) average(speedMeasurements);
                }
            }
        }
    }
    if (sensorFront >= 12 && 15 >= sensorFront && !readyToOvertake) {
        if (sensorLeftFront < 50 || sensorLeftMid < 50) {
            // car in middle lane, speed measured, active acc
            if (!accActive && speedOfCarInFront != 0) {
                adaptiveCruiseControl();
            }
        } else if (speedOfCarInFront < sdi.ownSpeed) {
            overtakeManoeuvre();
        }

    }


	/*
		// SensorRearRight
		if (sdi.cornerRearRight >= 3 && 29 >= sdi.cornerRearRight) {
		if (readyToOvertake) {
			// finishOvertakeManoeuvre()
		}
	}
	 */



	//calculate distance
	float distance = 18.5;
	checkOverTaking(distance, sdi.ownSpeed);
}

/**
 * custom publish function
 *
 * \param type  value name
 * \param value value as string
 */
void acc_overtaking::myPublish(const char *type, const char *value) {
	char topic[1024];
	strcpy(topic, "ecu/acc/");
	strncat(topic, type, sizeof(topic));
	//FrontCamera/steeringData
	publish(NULL, topic, strlen(value), value);
}

void acc_overtaking::on_message(const struct mosquitto_message *message)
{
	/* split type from topic */
	char *type = strrchr(message->topic, '/') + 1;

	/* get pointer to payload for convenience */
	char *value = (char *)message->payload;
	// unknown payload: distance: 0.0; direction: -1

	Genode::log("the payload is: ", (const char *)message->payload);
	Genode::log("the topic is: ", (const char *)message->topic);

	/* split x,y into two separate values */
	// strstr Returns a pointer to the first occurrence of str2 in str1, or a null pointer if str2 is not part of str1.
	// strtok string delimiters splits string in single words
	float x = 5.5, y = 5.6;
	if (strstr(value, ":")) {
		x = atof(strtok(value, ";"));
		Genode::log("x is: ", x);
		y = atof(strtok(NULL, ","));
		Genode::log("y is: ", x);

	}

	if (!strcmp(value, "distance")){
		Genode::log("steeringData is: ", (const char *)message->payload);
	} else
	/* fill sensorDataIn struct */
	if (!strcmp(type, "isPositionTracked")) {
		Genode::log("steeringData is: ", *value);
		sdi.isPositionTracked = atoi(value);
	} else if (!strcmp(type, "isSpeedTracked")) {
		Genode::log("steeringData is: ", *value);
		sdi.isSpeedTracked = atoi(value);
	} else if (!strcmp(type, "leadPos")) {
		Genode::log("steeringData is: ", *value);
		sdi.leadPos = vec2(x, y);
	} else if (!strcmp(type, "ownPos")) {
		Genode::log("steeringData is: ", *value);
		sdi.ownPos = vec2(x, y);
	} else if (!strcmp(type, "cornerFrontRight")) {
		sdi.cornerFrontRight = vec2(x, y);
		Genode::log("steeringData is: ", *value);
	} else if (!strcmp(type, "cornerFrontLeft")) {
		sdi.cornerFrontLeft = vec2(x, y);
		Genode::log("steeringData is: ", *value);
	} else if (!strcmp(type, "cornerRearLeft")) {
		sdi.cornerRearLeft = vec2(x, y);
		Genode::log("steeringData is: ", *value);
	} else if (!strcmp(type, "cornerRearRight")) {
		sdi.cornerRearRight = vec2(x, y);
		Genode::log("steeringData is: ", *value);
	} else if (!strcmp(type, "leadSpeed")) {
		sdi.leadSpeed = atof(value);
		Genode::log("steeringData is: ", *value);
	} else if (!strcmp(type, "ownSpeed")) {
		sdi.ownSpeed = atof(value);
		Genode::log("steeringData is: ", *value);
	} else if (!strcmp(type, "curGear")) {
		sdi.curGear = atoi(value);
		Genode::log("steeringData is: ", *value);
	} else if (!strcmp(type, "steerLock")) {
		sdi.steerLock = atof(value);
		Genode::log("steeringData is: ", *value);
	} else {
		Genode::log("unknown topic: ", (const char *)message->topic);
		return;
	}

	/* check if we got all values */
	sem_wait(&allValSem);
	allValues = (allValues + 1) % 12;
	if (!allValues) {
		sem_post(&allData);
	}
	sem_post(&allValSem);
}

void acc_overtaking::on_connect(int rc)
{
	Genode::log("connected to mosquitto server");
}

void acc_overtaking::on_disconnect(int rc)
{
	Genode::log("disconnect from mosquitto server");
}

/**
 * calculates the gear
 * based on empiric values? of the speed
 *
 * took it 1:1 from https://github.com/argos-research/genode-Simcom
 */
int acc_overtaking::getSpeedDepGear(float speed, int currentGear)
{
	// 0	 60  100 150 200 250 km/h
	float gearUP[6] = {-1, 17, 27, 41, 55, 70}; //Game uses values in m/s: xyz m/s = (3.6 * xyz) km/h
	float gearDN[6] = {0,	 0,  15, 23, 35, 48};

	int gear = currentGear;

	if (speed > gearUP[gear])
	{
		gear = std::min(5, currentGear + 1);
	}
	if (speed < gearDN[gear])
	{
		gear = std::max(1, currentGear - 1);
	}
	return gear;
}

/**
 * calculates command data for the next simulation step
 * based on the sensor data from the previous step
 *
 * took it 1:1 from https://github.com/argos-research/genode-Simcom
 */
CommandDataOut acc_overtaking::followDriving(SensorDataIn sd)
{
	// Get position of nearest opponent in front
	// via: - Sensor data
	// If distance below certain threshold
	// Drive in that direction (set angle)
	// If position in previous frame is known:
	//	 Calculate speed from old and new world positon
	//	 Try to adjust accel and brake to match speed of opponent
	//	 (Try to shift gear accordingly)
	// Save new world position in old position
	CommandDataOut cd = {0};

	if(!sd.isPositionTracked)
	{
		return cd;
	}
	vec2 curLeadPos = sd.leadPos;
	vec2 ownPos = sd.ownPos;

	// Get point of view axis of car in world coordinates
	// by substracting the positon of front corners and position of rear corners
	vec2 axis = (sd.cornerFrontRight - sd.cornerRearRight) + (sd.cornerFrontLeft - sd.cornerRearLeft);
	axis.normalize();

	//Get angle beween view axis and curleadPos to adjust steer
	vec2 leadVec = curLeadPos - ownPos;
	float dist = leadVec.len(); // absolute distance between cars

	// printf("DISTANCE: %f\n", leadVec.len());
	leadVec.normalize();


	// printf("CROSS: %f\n", axis.fakeCrossProduct(&leadVec));
	// printf("ANGLE: %f\n", RAD2DEG(asin(axis.fakeCrossProduct(&leadVec))));

	const float cross = axis.fakeCrossProduct(&leadVec);
	const float dot = axis * leadVec;
	const float angle = std::atan2(cross, dot) / sd.steerLock / 2.0;

	cd.steer = angle; // Set steering angle

	// Only possible to calculate accel and brake if speed of leading car known
	if(!sd.isSpeedTracked) // If position of leading car known in last frame
	{
		return cd;
	}

	float fspeed = sd.ownSpeed; // speed of following car

	float lspeed = sd.leadSpeed; // speed of leading car

	// adjusted distance to account for different speed, but keep it positive so brake command will not be issued if leading speed is too high
	//float adist = std::max<float>(0.1, g_followDist + (fspeed - lspeed));

	// Accel gets bigger if we are further away from the leading car
	// Accel goes to zero if we are at the target distance from the leading car
	// Target distance is adjusted, dependent on the the speed difference of both cars
	// Accel = maxAccel if dist = threshold
	// Accel = 0 if dist = adist (adjusted target dist)
	//cd.accel = std::max<float>(0, std::min<float>(g_maxAccel, std::sqrt(g_maxAccel * (dist - adist) / (g_distThreshold - adist))));


	// Äquivalent to accel but the other way round
	//float b = std::max<float>(0, std::min<float>(g_maxBrake, std::sqrt(g_maxBrake * (adist - dist) / adist)));
	float dv = (lspeed - fspeed);
	if (dv > 0.0 && dist > 5.0)
	{
		cd.accel = 0.5 * dv;
	}
	else if (dist < 30.0)
	{
		cd.brakeFL = cd.brakeFR = cd.brakeRL = cd.brakeRR = -0.5 * dv;
	}
	//char format[256];
	//sprintf(format, "Speeds: %4.4f %4.4f %4.4f\n", lspeed, fspeed, dist);
	//PDBG("%s", format);

	// Individual brake commands for each wheel

	cd.gear = getSpeedDepGear(sd.ownSpeed, sd.curGear);

	return cd;
}

void acc_overtaking::checkOverTaking(float distance, float mSpeed){
    double mSlowSpeed = 20.0;
    double mFastSpeed = 50.0;
    bool isOvertaking = false;
    int distanceTreshHold;

    if (distance < 10 && mSpeed >= mFastSpeed && mDistance != distance && !isOvertaking) {
        distanceTreshHold++;
    }
    mDistance = distance;
    if (distanceTreshHold >= 12) {
        distanceTreshHold = 0.0;

            /* move left 
            * print the lane
            */
        isOvertaking = true; 
        int timePassed = 0;
        while (isOvertaking) {
            /* getDistance, sensorDistance 
                    Thread.sleep(250)
                    timePassed += 250
            */
            float sensorDistance;
            if ((sensorDistance < 20 && sensorDistance > 5 )|| timePassed > 20 * 1000) {
                isOvertaking = false;
            }
		}
        /* moveRight overtaking is over or not possible */
    }
};

float acc_overtaking::average(vector<int> vec){
	float average = accumulate( vec.begin(), vec.end(), 0.0)/vec.size();
	return average;
};

void acc_overtaking::overtakeManoeuvre() {
    readyToOvertake = true;
    disableAutonomous();
    /* Move Left */
};

void acc_overtaking::finishOvertakeManoeuvre() {
    /* MOVE_RIGHT */
    readyToOvertake = false;
    accActive = false;
    /* delay 4000 */
    enableAutonomous();
};

void acc_overtaking::adaptiveCruiseControl() {

    // if estimated speed of car in front is less or equal to, we assume that it is not moving
    if (speedOfCarInFront >= 0 && 10 >= speedOfCarInFront) {
            speedOfCarInFront = 0;
            /* SET_SPEED 0.0 */
            /*  SET_BRAKE 1.0 */

            // delay 3000
            /* SET_BRAKE 0.0 */
    } else if (speedOfCarInFront >= 10 && sdi.ownSpeed >= speedOfCarInFront) {
        /* SET_SPEED 2*speedOfCarInFront */
    }
    accActive = true;
}

void acc_overtaking::disableAutonomous() {
    if (isAutonomous) {
        /* SET_STEER 0.0 */
        /* TOGGLE_AUTONOMOUS */
        autonomousWasDisabled = true;
    }
};

void acc_overtaking::enableAutonomous() {
    if (!isAutonomous && autonomousWasDisabled) {
        toggleAutonomous();
        autonomousWasDisabled = false;
    }
};

void acc_overtaking::steerForLaneKeeping(double distance, int steerDirection) {
    double steerValue = 0.0;
    const double absDistance = abs(distance);
    if (fallbackPoints.size() >= 2) {
        const double prevDistance = fallbackPoints.back(); 
        //fallbackPoints.remove()
        //fallbackPoints.remove()
        const double delta = absDistance - prevDistance;
        // distance inreased from old val
        // try to do a sharper steer
        if (absDistance >= 13) {
                if (delta >= -2 && 2 >= delta) {return;}
                if (delta >= 2 && 10 >= delta) {steerValue += (0.04 * steerDirection);}
                if (delta >= 10 && 20 >= delta) {steerValue += (0.05 * steerDirection);}
                if (delta >= 20 && 30 >= delta) {steerValue += (0.06 * steerDirection);}
                if (delta >= 30 && 40 >= delta) {steerValue += (0.07 * steerDirection);}
                if (delta >= 40 && 50 >= delta) {steerValue += (0.08 * steerDirection);}
                if (delta >= 100 && 300 >= delta) {steerValue += (0.5 * steerDirection);}

                if (delta >= -10 && -2 >= delta) {steerValue -= (0.04 * steerDirection);}
                if (delta >= -20 && -10 >= delta) {steerValue -= (0.05 * steerDirection);}
                if (delta >= -30 && -20 >= delta) {steerValue -= (0.06 * steerDirection);}
                if (delta >= -40 && -30 >= delta) {steerValue -= (0.07 * steerDirection);}
                if (delta >= -50 && -40 >= delta) {steerValue -= (0.08 * steerDirection);}
                if (delta >= -300 && -100 >= delta) {steerValue -= (0.5 * steerDirection);}

        }
    } else {
        steerValue = steerDirection * absDistance / 4000.0;
        steerValue = roundf(steerValue/*, 5 */);
    }

    if (distance >= -4 && 4 >= distance) {
        steerValue = 0.0;
    }
    if (absDistance >= 300 && isAutonomous) {
        toggleAutonomous();
        return;
    }

    fallbackPoints.push_back(absDistance);
    /* STEER steerValue */
}

void acc_overtaking::toggleAutonomous() {
    /*steerValue = 0.0
    postRequest(STEER, Pair("steer", 0.0))
    postRequest(TOGGLE_AUTONOMOUS, null)
	*/
}
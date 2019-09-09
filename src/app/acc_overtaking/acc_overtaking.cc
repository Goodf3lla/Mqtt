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
double steerValue = 0.0;
double oldSteerValue = 0.0;
double distanceMqtt;
double steeringMqtt;
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
bool isAutonomous = true; // should be set to false initial, but a by the broker persisted toggleAutonomous command in my setup forces to init with true
bool isOvertaking = false;
int desired_speed = -1;
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
/**
 * taken from the android app, adapted but never really tested, 
 * this should be tested and finished with a stable setup without graphic freezes!
 **/
CommandDataOut acc_overtaking::doOvertaking(SensorDataIn sdi) {
	CommandDataOut cd = {0};
	if(!sdi.isPositionTracked)
	{
		return cd;
	}
    if (sdi.middleFront != 100.0 && oldSensorFront != 100.0 && sdi.middleFront != -1.0 && oldSensorFront != -1.0) {
        if (sdi.middleFront != oldSensorFront) {
            if (sdi.isSpeedTracked) {
				/**
				 * this uses the camera distance to calculate the speed of the car in front, unless it s stable it s better to use the game sensor
                // time passed in seconds
                //timeRequestResponded = System.currentTimeMillis();
                const float time = (timeRequestResponded - timeRequestStarted) * 0.001;
                // distance covered by argos vehicle in m
                const int travelled = time * sdi.ownSpeed / 3.6;
                // distance covered by the vehicle in front
                const float delta = travelled - (oldSensorFront - sdi.middleFront);
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
				*/
				// speed up to a faster speed than the leading car
				if (sdi.leadSpeed > sdi.ownSpeed){
					cd.accel = 0.5 * (sdi.leadSpeed - sdi.ownSpeed + 5);
				} else { // no need to speed up further
					cd.accel = 0.0;
				}
				
            }
        }
    }
	// the vectors coming from the simulation need to be normalized, not yet done!
   /* if (sdi.middleFront >= 12 && 15 >= sdi.middleFront && !readyToOvertake) {
        if (sdi.cornerFrontLeft < 50 || sdi.cornerRearLeft < 50) {
            // car in middle lane, speed measured, active acc
            if (!accActive && sdi.leadSpeed != 0) {
				return ;
            }
        } else if (sdi.leadSpeed < sdi.ownSpeed) {
			readyToOvertake = true; // is overtaking now
            // switch to left lane
			cd.steer = -0.5
        }
    }
	if (sdi.middleFront = 0 && readyToOvertake){
		if (sdi.cornerRearRight < 50 && sdi.cornerFrontRight < 50) {
			readyToOvertake = false; // stop overtaking now
			// switch to right lane
			cd.steer = 0.5
		}		    
	}*/
	return cd;
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
	publish(NULL, topic, strlen(value), value);
}

void acc_overtaking::on_message(const struct mosquitto_message *message)
{
	/* split type from topic */
	char *type = strrchr(message->topic, '/') + 1;

	/* get pointer to payload for convenience */
	char *value = (char *)message->payload;

	// Genode::log("the payload is: ", (const char *)message->payload);
	// Genode::log("the topic is: ", (const char *)message->topic);

	/* split x,y into two separate values */
	// strstr Returns a pointer to the first occurrence of str2 in str1, or a null pointer if str2 is not part of str1.
	// strtok string delimiters splits string in single words
	float x = 0.0, y = 0.0;
	if (strstr(value, ",")) {
		x = atof(strtok(value, ","));
		y = atof(strtok(NULL, ","));
	}

	if(!strcmp((const char *)message->topic, "FrontCamera/sensorOff")){
		Genode::log("Sensor is going offline");
		isAutonomous = false;
		return;
	}
	if(!strcmp((const char *)message->topic, "FrontCamera/setSpeed")){
		Genode::log("setting speed");
		desired_speed = (atoi((const char *)message->payload))/3.6;
		return;
	} else if(!strcmp((const char *)message->topic, "FrontCamera/toggleAutonomous")){
		Genode::log("toggling autonomous");
		toggleAutonomous();
		return;
	} else if (!strcmp((const char *)message->topic, "FrontCamera/steeringData")){
		Genode::log("steeringData is: ", (const char *)message->payload);
		steeringMqtt = atof(value);
		sdi.steeringValue = steeringMqtt;
		return;
	} else if  (!strcmp((const char *)message->topic, "FrontCamera/distance")){
		Genode::log("distance is: ", (const char *)message->payload);
		distanceMqtt = atof(value);
		sdi.middleFront = distanceMqtt;
		return;
	} else
	/* fill sensorDataIn struct */
	if (!strcmp(type, "isPositionTracked")) {
		//Genode::log("isPositionTracked is: ", *value);
		sdi.isPositionTracked = atoi(value);
	} else if (!strcmp(type, "isSpeedTracked")) {
		Genode::log("isSpeedTracked is: ", *value);
		sdi.isSpeedTracked = atoi(value);
	} else if (!strcmp(type, "leadPos")) {
		//Genode::log("leadPos is: ", *value);
		sdi.leadPos = vec2(x, y);
	} else if (!strcmp(type, "ownPos")) {
		Genode::log("ownPos is: ", *value);
		sdi.ownPos = vec2(x, y);
	} else if (!strcmp(type, "cornerFrontRight")) {
		sdi.cornerFrontRight = vec2(x, y);
		//Genode::log("cornerFrontRight is: ", *value);
	} else if (!strcmp(type, "cornerFrontLeft")) {
		sdi.cornerFrontLeft = vec2(x, y);
		//Genode::log("cornerFrontLeft is: ", *value);
	} else if (!strcmp(type, "cornerRearLeft")) {
		sdi.cornerRearLeft = vec2(x, y);
		//Genode::log("cornerRearLeft is: ", *value);
	} else if (!strcmp(type, "cornerRearRight")) {
		sdi.cornerRearRight = vec2(x, y);
		//Genode::log("cornerRearRight is: ", *value);
	} else if (!strcmp(type, "leadSpeed")) {
		sdi.leadSpeed = atof(value);
		Genode::log("leadSpeed is: ", *value);
	} else if (!strcmp(type, "ownSpeed")) {
		sdi.ownSpeed = atof(value);
		Genode::log("ownSpeed is: ", *value);
	} else if (!strcmp(type, "curGear")) {
		sdi.curGear = atoi(value);
		//Genode::log("curGear is: ", *value);
	} else if (!strcmp(type, "steerLock")) {
		sdi.steerLock = atof(value);
		//Genode::log("steerLock is: ", *value);
	} else {
		//Genode::log("unknown topic: ", (const char *)message->topic);
		return;
	}


	/* check if we got all values */
	sem_wait(&allValSem);
	Genode::log("allValues is ",allValues);
	allValues = (allValues + 1) % 12; // 13
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
 * 
 **/
bool acc_overtaking::checkOvertaking(SensorDataIn sd){
    double mFastSpeed = 40.0;
    int distanceTreshHold;

    if (sd.middleFront < 10 && sd.ownSpeed >= mFastSpeed && mDistance != sd.middleFront && !isOvertaking) {
        distanceTreshHold++;
    }
    mDistance = sd.middleFront;
    if (distanceTreshHold >= 12) {
        distanceTreshHold = 0.0;
        isOvertaking = true; 
        int timePassed = 0;
		return true;
    } else {
		return false;
	}
};

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
	Genode::log("followDriving autonomous is ", isAutonomous);
	if(isAutonomous && sd.middleFront != 0){
		Genode::log("followDriving distance is ", sd.middleFront);
		cd.steer = steerForLaneKeeping(sd.middleFront, sd.steeringValue);
	/*	if (checkOvertaking(sd)) {
			cd = doOvertaking(sd);
			if (!cd) {
				// overtaking not possible, doOvertaking returns empty!
				cd.steer = angle;
			} else {
				//doing the overtaking
				return cd;
			}
		}*/
	} else {
		cd.steer = angle; // Set steering angle
	}
	Genode::log("followDriving steering angle is ", cd.steer);


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
	/**
	 * if desiredSpeed != -1 the user set a speed value over the phone
	 * therefore we need to accelerate if we are not fast enough yet or brake if we are to fast
	 **/
	if (desired_speed != -1 && dist > 5.0){
		dv = (desired_speed - fspeed);
		if (desired_speed > fspeed) {
			cd.accel = 0.5 * dv;
			Genode::log("followDriving speed set by user ", cd.accel);

		} else if (desired_speed < fspeed){
			cd.brakeFL = cd.brakeFR = cd.brakeRL = cd.brakeRR = -0.5 * dv;
		} else {
			/* speed was reached, reset to -1
			 * remove this else {} if speed should be kept!
			 */
			desired_speed = -1;
		}		
	}else if (dv > 0.0 && dist > 5.0){
		cd.accel = 0.5 * dv;
	} else if (dist < 30.0 && !isAutonomous){
		cd.brakeFL = cd.brakeFR = cd.brakeRL = cd.brakeRR = -0.5 * dv;
	}
	//char format[256];
	//sprintf(format, "Speeds: %4.4f %4.4f %4.4f\n", lspeed, fspeed, dist);
	//PDBG("%s", format);

	// Individual brake commands for each wheel

	cd.gear = getSpeedDepGear(sd.ownSpeed, sd.curGear);
	Genode::log("followDriving cd.gear is ", cd.gear);

	return cd;
}




float acc_overtaking::average(vector<int> vec){
	float average = accumulate( vec.begin(), vec.end(), 0.0)/vec.size();
	return average;
};

double acc_overtaking::steerForLaneKeeping(double distance, int steerDirection) {
	Genode::log("steerForLaneKeeping distance ", distance);
	Genode::log("steerForLaneKeeping steeringDirection ", steerDirection);
    const double absDistance = abs(distance);
	oldSteerValue = steerValue;
    if (fallbackPoints.size() >= 2) {
        const double prevDistance = fallbackPoints.back(); 
        fallbackPoints.pop_back();
        fallbackPoints.pop_back();
        const double delta = absDistance - prevDistance;
        // distance inreased from old val
        // try to do a sharper steer
        if (absDistance >= 13) {
                if (delta >= -2 && 2 >= delta) {return oldSteerValue;}
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
    fallbackPoints.push_back(absDistance);
	Genode::log("steerForLaneKeeping angle ", steerValue);
	return steerValue;
}

void acc_overtaking::toggleAutonomous() {
	if(isAutonomous) {
		isAutonomous = false;
	} else {
		isAutonomous = true;
	}
}

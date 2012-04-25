#include "tuhh.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Storage/Blackboard.h"
#include "Modules/Alias.h"



// private members
#ifndef WEBOTS
boost::signals::connection TUHH::cycleConnection;
#endif

float* TUHH::jointActuator[keys::joints::JOINTS_MAX];
float* TUHH::jointCurrent[keys::joints::JOINTS_MAX];
float* TUHH::jointHardness[keys::joints::JOINTS_MAX];
float* TUHH::jointSensor[keys::joints::JOINTS_MAX];
float* TUHH::jointTemperature[keys::joints::JOINTS_MAX];

float* TUHH::imu[keys::sensor::IMU_MAX];
float* TUHH::switches[keys::sensor::SWITCH_MAX];
float* TUHH::sonar[keys::sensor::SONAR_MAX];
float* TUHH::fsrLeft[keys::sensor::FSR_MAX];
float* TUHH::fsrRight[keys::sensor::FSR_MAX];
float* TUHH::battery[keys::sensor::BATTERY_MAX];



void TUHH::init()
{
    // create aliases
	Alias::init();
    for (int i = 0; i < Alias::ALIAS_MAX; i++)
        DcmConnector::createAlias(Alias::alias(i));

#ifndef WEBOTS
	// store pointer to joint variables
	for (int i = 0; i < keys::joints::JOINTS_MAX; i++)
	{
		jointActuator[i]		= DcmConnector::getDataPtr(keys::joints::actuatorKey[i]);
		jointSensor[i]			= DcmConnector::getDataPtr(keys::joints::sensorKey[i]);
		jointCurrent[i]			= DcmConnector::getDataPtr(keys::joints::currentKey[i]);
		jointHardness[i]		= DcmConnector::getDataPtr(keys::joints::currentKey[i]);
                jointTemperature[i]             = DcmConnector::getDataPtr(keys::joints::temperatureKey[i]);
	}

	// store pointer to imu variables
	for (int i= 0; i< keys::sensor::IMU_MAX; i++)
		imu[i] = DcmConnector::getDataPtr(keys::sensor::imuKey[i]);

	// store pointer to switch variables
	for (int i = 0; i <keys::sensor::SWITCH_MAX; i++)
		switches[i] = DcmConnector::getDataPtr(keys::sensor::switchKey[i]);

	// store pointer to sonar variables
	for (int i= 0; i < keys::sensor::SONAR_MAX; i++)
		sonar[i] = DcmConnector::getDataPtr(keys::sensor::sonarKey[i]);

	// store pointer to fsr variables
	for (int i = 0; i < keys::sensor::FSR_MAX; i++)
	{
		fsrLeft[i]	= DcmConnector::getDataPtr(keys::sensor::fsrLeftKey[i]);
		fsrRight[i]	= DcmConnector::getDataPtr(keys::sensor::fsrRightKey[i]);
	}

	// store pointer to battery variables
	for (int i = 0; i < keys::sensor::BATTERY_MAX; i++)
		battery[i] = DcmConnector::getDataPtr(keys::sensor::batteryKey[i]);	

	// connect cycle-function to DCM-Process
	cycleConnection = DcmConnector::bindPost(boost::bind(&TUHH::cycle));
#endif
}


void TUHH::cycle()
{
#ifndef WEBOTS
	// emergency stiffness release on chestbutton pressed
	if ( *switches[keys::sensor::SWITCH_CHEST_BUTTON] == 1)
		DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_HARDNESS_BODY),"ClearAll",20,0);

	// temp storage variables
	std::vector<float> vecJointActuator(keys::joints::JOINTS_MAX);
	std::vector<float> vecJointSensor(keys::joints::JOINTS_MAX);
	std::vector<float> vecJointHardness(keys::joints::JOINTS_MAX);
	std::vector<float> vecJointCurrent(keys::joints::JOINTS_MAX);
	std::vector<float> vecJointTemperature(keys::joints::JOINTS_MAX);

	Vector3<float> accData;
	Vector2<float> gyrData;
	Vector2<float> angles;

	// fill joint vectors
	for (int i = 0; i < keys::joints::JOINTS_MAX; i++)
	{
		vecJointActuator[i]		= *jointActuator[i];
                vecJointSensor[i]		= *jointSensor[i];
		vecJointHardness[i]		= *jointHardness[i];
		vecJointCurrent[i]		= *jointCurrent[i];
                vecJointTemperature[i]          = *jointTemperature[i];
	}

	// store imu data
	accData.x = *imu[keys::sensor::IMU_ACC_X];
	accData.y = *imu[keys::sensor::IMU_ACC_Y];
	accData.z = *imu[keys::sensor::IMU_ACC_Z];

	gyrData.x = *imu[keys::sensor::IMU_GYR_X];
	gyrData.y = *imu[keys::sensor::IMU_GYR_Y];

	angles.x = *imu[keys::sensor::IMU_ANGLE_X];
	angles.y = *imu[keys::sensor::IMU_ANGLE_Y];

	// update Blackboard
	Blackboard::updateJointAngles(vecJointSensor);
	Blackboard::updateLastCommands(vecJointActuator);
	Blackboard::updateTime(DcmConnector::getTime() );
	Blackboard::updateKinematicMatrices();
	Blackboard::updateCom();
	Blackboard::updateAccelerometerData(accData);
	Blackboard::updateGyroscopeData(gyrData);
	Blackboard::updateAngleData(angles);
#endif
}

void TUHH::stopCycle()
{
#ifndef WEBOTS
	cycleConnection.disconnect();
#endif
}



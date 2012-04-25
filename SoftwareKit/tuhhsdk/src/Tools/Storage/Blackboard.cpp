/*
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */

#include "Blackboard.h"

// initialization of private members
vector<float> Blackboard::jointAngles = vector<float>();
vector<float> Blackboard::lastCommands = vector<float>();
Vector3<float> Blackboard::accelerometerData = Vector3<float>();
Vector3<float> Blackboard::filteredAccelerometerData = Vector3<float>();
Vector2<float> Blackboard::gyroscopeData = Vector2<float>();
Vector2<float> Blackboard::angleData = Vector2<float>();
vector<float> Blackboard::fsr = vector<float>();
Vector3<float> Blackboard::comBody = Vector3<float>();
KinematicMatrix Blackboard::kinMatrices[JOINTS::JOINTS_ADD_MAX] = {KinematicMatrix()};
int Blackboard::time = 0;

float Blackboard::accelerometerFilterStrength = 0.1f;


KalmanFilter Blackboard::kalmanAngleX = KalmanFilter(Matrix2x2<float>(1.0f, -0.01f, 0.0f, 1.0f),
													 Vector2<float>(0.01f ,0.0f),
													 Vector2<float>(1.0f,0.0f),
													 Vector2<float>(),
													 Matrix2x2<float>(),
													 Matrix2x2<float>()*0.0000001f,
													 100);
													 
													 // WEBOTS: Q = eye(2)*0.0000001f
													 // 		R = 100;

KalmanFilter Blackboard::kalmanAngleY = KalmanFilter(Matrix2x2<float>(1.0f, -0.01f, 0.0f, 1.0f),
													 Vector2<float>(0.01f , 0),
													 Vector2<float>(1.0f,0.0f),
													 Vector2<float>(),
													 Matrix2x2<float>(),
													 Matrix2x2<float>()*0.0000001f,
													 100);
													 // WEBOTS: Q = eye(2)*0.0000001f
													 // 		R = 100;


//
// Functions
//

/* updateTime */
void Blackboard::updateTime(const int& t)
{
	time = t;
}

/* updateCom */
void Blackboard::updateCom()
{
	comBody = Com::getComBody();
}

/* getCom */
Vector3< float > Blackboard::getCom()
{
	return comBody;
}

/* updateAccelerometerData */
void Blackboard::updateAccelerometerData(const Vector3<float>& data)
{
	accelerometerData = data;
	filteredAccelerometerData = filteredAccelerometerData * accelerometerFilterStrength +
		data * (1- accelerometerFilterStrength);		
}

/* getAccelerometerData */
Vector3<float> Blackboard::getAccelerometerData()
{
	return accelerometerData;
}

/* getFilteredAccelerometerData */
Vector3<float> Blackboard::getFilteredAccelerometerData()
{
	return filteredAccelerometerData;
}

/* updateGyroscopeData */
void Blackboard::updateGyroscopeData(const Vector2<float>& data)
{
	gyroscopeData = data;
}

/* updateAngleData */
void Blackboard::updateAngleData(const Vector2<float>& data)
{
	angleData = data;
}

/* updateAngleEstimation */
void Blackboard::updateAngleEstimation()
{
#ifndef WEBOTS
	int accFactor = -1;
#else
	int accFactor = 1;
#endif

	Vector2<float> xX = kalmanAngleX.predict(	
		gyroscopeData[0], 												
		accFactor *atan2(	filteredAccelerometerData[1], 														
		sqrt(	filteredAccelerometerData[0]*																
		filteredAccelerometerData[0] +																
		filteredAccelerometerData[2]*																
		filteredAccelerometerData[2])));

    Vector2<float> xY = kalmanAngleY.predict(	
		gyroscopeData[1],
		-atan2(	filteredAccelerometerData[0], 														
		sqrt(	filteredAccelerometerData[1]*															
		filteredAccelerometerData[1] +															
		filteredAccelerometerData[2]*																
		filteredAccelerometerData[2])));														
	
	
	angleData.x = xX.x;															 
	angleData.y = xY.x;
}

/* getGyroscopeData */
Vector2<float> Blackboard::getGyroscopeData()
{
	return gyroscopeData;
}

/* getAngleData */
Vector2<float> Blackboard::getAngleData()
{
	return angleData;
}

/* updateJointAngles */
void Blackboard::updateJointAngles(const std::vector< float >& angles)
{
	jointAngles = angles;
}

/* getJointAngles */
vector<float> Blackboard::getJointAngles(const string& chainName)
{
	if ( chainName.compare("Body") == 0 )
	{
		return jointAngles;
	}
	else if ( chainName.compare("LLeg") == 0 )
	{
		vector<float> temp;
		for ( int i = 8; i <= 13; i++)
			temp.push_back(jointAngles.at(i));
		return temp;
	}
	else if ( chainName.compare("RLeg") == 0 )
	{
		vector<float> temp;
		for ( int i = 14; i <= 19; i++)
			temp.push_back(jointAngles.at(i));
		return temp;
	}
	else if ( chainName.compare("LArm") == 0 )
	{
		vector<float> temp;
		for ( int i = 2; i <= 7; i++)
			temp.push_back(jointAngles.at(i));
		return temp;
	}
	else if ( chainName.compare("RArm") == 0 )
	{
		vector<float> temp;
		for ( int i = 20; i <= 25; i++)
			temp.push_back(jointAngles.at(i));
		return temp;
	}
	else if ( chainName.compare("Head") == 0 )
	{
		vector<float> temp;
		for ( int i = 0; i <= 1; i++)
			temp.push_back(jointAngles.at(i));
		return temp;
	}
	else
	{
		return jointAngles;
	}		
}

/* updateKinematicMatrices */
void Blackboard::updateKinematicMatrices()
{
	vector<float> headAngles = Blackboard::getJointAngles("Head");
	kinMatrices[JOINTS::HEAD_YAW] = ForwardKinematics::getHeadYaw(headAngles);
	kinMatrices[JOINTS::HEAD_PITCH] = ForwardKinematics::getHeadPitch(headAngles);
	
	vector<float> lLegAngles = Blackboard::getJointAngles("LLeg");
	kinMatrices[JOINTS::L_HIP_YAW_PITCH] = ForwardKinematics::getLHipYawPitch(lLegAngles);
	kinMatrices[JOINTS::L_HIP_ROLL] = ForwardKinematics::getLHipRoll(lLegAngles);
	kinMatrices[JOINTS::L_HIP_PITCH] = ForwardKinematics::getLHipPitch(lLegAngles);
	kinMatrices[JOINTS::L_KNEE_PITCH]=  ForwardKinematics::getLKneePitch(lLegAngles);
	kinMatrices[JOINTS::L_ANKLE_PITCH] = ForwardKinematics::getLAnklePitch(lLegAngles);
	kinMatrices[JOINTS::L_ANKLE_ROLL] = ForwardKinematics::getLAnkleRoll(lLegAngles);
	kinMatrices[JOINTS::L_FOOT] = ForwardKinematics::getLFoot(lLegAngles);

	kinMatrices[JOINTS::L_PELVIS] = ForwardKinematics::getLPelvis(lLegAngles);

	vector<float> rLegAngles = Blackboard::getJointAngles("RLeg");
	kinMatrices[JOINTS::R_HIP_YAW_PITCH] = ForwardKinematics::getRHipYawPitch(rLegAngles);
	kinMatrices[JOINTS::R_HIP_ROLL] = ForwardKinematics::getRHipRoll(rLegAngles);
	kinMatrices[JOINTS::R_HIP_PITCH] = ForwardKinematics::getRHipPitch(rLegAngles);
	kinMatrices[JOINTS::R_KNEE_PITCH] = ForwardKinematics::getRKneePitch(rLegAngles);
	kinMatrices[JOINTS::R_ANKLE_PITCH] = ForwardKinematics::getRAnklePitch(rLegAngles);
	kinMatrices[JOINTS::R_ANKLE_ROLL] = ForwardKinematics::getRAnkleRoll(rLegAngles);
	kinMatrices[JOINTS::R_FOOT] = ForwardKinematics::getRFoot(rLegAngles);

	kinMatrices[JOINTS::R_PELVIS] = ForwardKinematics::getRPelvis(rLegAngles);

	vector<float> lArmAngles = Blackboard::getJointAngles("LArm");
	kinMatrices[JOINTS::L_SHOULDER_PITCH] = ForwardKinematics::getLShoulderPitch(lArmAngles);
	kinMatrices[JOINTS::L_SHOULDER_ROLL] = ForwardKinematics::getLShoulderRoll(lArmAngles);
	kinMatrices[JOINTS::L_ELBOW_YAW] = ForwardKinematics::getLElbowYaw(lArmAngles);
	kinMatrices[JOINTS::L_ELBOW_ROLL] = ForwardKinematics::getLElbowRoll(lArmAngles);
	kinMatrices[JOINTS::L_WRIST_YAW] = ForwardKinematics::getLWristYaw(lArmAngles);
	kinMatrices[JOINTS::L_HAND] = ForwardKinematics::getLHand(lArmAngles);

	vector<float> rArmAngles = Blackboard::getJointAngles("RArm");
	kinMatrices[JOINTS::R_SHOULDER_PITCH] = ForwardKinematics::getRShoulderPitch(rArmAngles);
	kinMatrices[JOINTS::R_SHOULDER_ROLL] = ForwardKinematics::getRShoulderRoll(rArmAngles);
	kinMatrices[JOINTS::R_ELBOW_YAW] = ForwardKinematics::getRElbowYaw(rArmAngles);
	kinMatrices[JOINTS::R_ELBOW_ROLL] = ForwardKinematics::getRElbowRoll(rArmAngles);
	kinMatrices[JOINTS::R_WRIST_YAW] = ForwardKinematics::getRWristYaw(rArmAngles);
	kinMatrices[JOINTS::R_HAND] = ForwardKinematics::getRHand(rArmAngles);
}

/* getKinematicMatrix */
KinematicMatrix Blackboard::getKinematicMatrix(const int &index)
{
	return kinMatrices[index];
}

/* getTime */
int Blackboard::getTime()
{
	return time;
}

/* updateFsr */
void Blackboard::updateFsr(const std::vector< float >& fsr)
{
	Blackboard::fsr = fsr;
}

/* getFsr */
vector<float> Blackboard::getFsr()
{
	return fsr;
}

/* updateLastCommands */
void Blackboard::updateLastCommands(const vector<float>& angles)
{
	lastCommands = angles;
}

/* getLastCommands */
vector<float> Blackboard::getLastCommands(const std::string &chain)
{
	vector<float> tmp;

	if (chain.compare("Head") == 0)
		for (int i = 0; i<2; i++)
			tmp.push_back(lastCommands[i]);

	else if (chain.compare("LArm") == 0)
		for (int i = 0; i < 6; i++)
			tmp.push_back(lastCommands[JOINTS::L_SHOULDER_PITCH+i]);

	else if (chain.compare("RArm") == 0)
		for (int i = 0; i < 6; i++)
			tmp.push_back(lastCommands[JOINTS::R_SHOULDER_PITCH+i]);

	else if (chain.compare("LLeg") == 0)
		for (int i = 0; i < 6; i++)
			tmp.push_back(lastCommands[JOINTS::L_HIP_YAW_PITCH +i]);

	else if (chain.compare("RLeg") == 0)
		for (int i = 0; i < 6; i++)
			tmp.push_back(lastCommands[JOINTS::R_HIP_YAW_PITCH +i]);

	else if (chain.compare("Body") == 0)
		tmp = lastCommands;

	return tmp;
}


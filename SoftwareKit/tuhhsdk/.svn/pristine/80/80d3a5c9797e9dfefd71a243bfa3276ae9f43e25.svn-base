//
#include "ForwardKinematics.h"
#include "../Storage/Blackboard.h"

/*  +----------+
 *  |   Head   |
 *  +----------+
 */

//HeadYaw
KinematicMatrix ForwardKinematics::getHeadYaw(vector<float> jointAngles)
{
	KinematicMatrix HeadYaw2Torso = 
			KinematicMatrix::transZ(LINKS::NECK_OFFSET_Z) *
		KinematicMatrix::rotZ(jointAngles.at(0));

	return HeadYaw2Torso;
}

// HeadPitch
KinematicMatrix ForwardKinematics::getHeadPitch(vector<float> jointAngles)
{
	KinematicMatrix HeadPitch2HeadYaw = 
		KinematicMatrix::rotY(jointAngles.at(1));

	return getHeadYaw( jointAngles) * HeadPitch2HeadYaw;
}


/*  +----------+
 *  | Left Arm |
 *  +----------+
 */

// Left Shoulder Pitch
KinematicMatrix ForwardKinematics::getLShoulderPitch(vector<float> jointAngles)
{
	KinematicMatrix LShoulderBase2Torso = 
		KinematicMatrix::transZ(LINKS::SHOULDER_OFFSET_Z) *
		KinematicMatrix::transY(LINKS::SHOULDER_OFFSET_Y);

	KinematicMatrix LShoulderPitch2LShoulderBase = 
		KinematicMatrix::rotY(jointAngles.at(0));

	return LShoulderBase2Torso * LShoulderPitch2LShoulderBase;
}

// Left Shoulder Roll
KinematicMatrix ForwardKinematics::getLShoulderRoll(vector<float> jointAngles)
{
	KinematicMatrix LShoulderRoll2LShoulderPitch = 
		KinematicMatrix::rotZ(jointAngles.at(1));

	return getLShoulderPitch( jointAngles ) * LShoulderRoll2LShoulderPitch;
}

// Left Elbow Yaw
KinematicMatrix ForwardKinematics::getLElbowYaw(vector<float> jointAngles)
{
	KinematicMatrix LElbowYaw2LShoulderRoll = 
		KinematicMatrix::transX(LINKS::UPPER_ARM_LENGTH) *
		KinematicMatrix::transY(LINKS::ELBOW_OFFSET_Y) *
		KinematicMatrix::rotX( jointAngles.at(2) );

	return getLShoulderRoll( jointAngles ) * LElbowYaw2LShoulderRoll;
}

// Left Elbow Roll
KinematicMatrix ForwardKinematics::getLElbowRoll(vector<float> jointAngles)
{
	KinematicMatrix LElbowRoll2LElbowYaw = 
		KinematicMatrix::rotZ( jointAngles.at(3) );

	return getLElbowYaw( jointAngles ) * LElbowRoll2LElbowYaw;
}

// Left Wrist Yaw 
KinematicMatrix ForwardKinematics::getLWristYaw(vector<float> jointAngles)
{
	KinematicMatrix LWristYaw2LElbowRoll = 
		KinematicMatrix::transX(LINKS::LOWER_ARM_LENGTH) *
		KinematicMatrix::rotX( jointAngles.at(4) );
	
	return getLElbowRoll( jointAngles ) * LWristYaw2LElbowRoll;
}

// Left Hand
KinematicMatrix ForwardKinematics::getLHand(vector<float> jointAngles)
{
	KinematicMatrix LHand2LWristYaw = 
		KinematicMatrix::transX(LINKS::HAND_OFFSET_X);// *
		//KinematicMatrix::transZ(Blackboard::HAND_OFFSET_Z) *
		//KinematicMatrix::rotY( jointAngles.at(5) );
	
	return getLWristYaw( jointAngles ) * LHand2LWristYaw;
}

/*  +-----------+
 *  | Right Arm |
 *  +-----------+
 */

// Right Shoulder Pitch
KinematicMatrix ForwardKinematics::getRShoulderPitch(vector<float> jointAngles)
{
	KinematicMatrix RShoulderBase2Torso = 
		KinematicMatrix::transZ(LINKS::SHOULDER_OFFSET_Z) *
		KinematicMatrix::transY(-LINKS::SHOULDER_OFFSET_Y);

	KinematicMatrix RShoulderPitch2RShoulderBase = 
		KinematicMatrix::rotY(jointAngles.at(0));

	return RShoulderBase2Torso * RShoulderPitch2RShoulderBase;
}

// Right Shoulder Roll
KinematicMatrix ForwardKinematics::getRShoulderRoll(vector<float> jointAngles)
{
	KinematicMatrix RShoulderRoll2RShoulderPitch = 
		KinematicMatrix::rotZ(jointAngles.at(1));

	return getRShoulderPitch( jointAngles ) * RShoulderRoll2RShoulderPitch;
}

// Right Elbow Yaw
KinematicMatrix ForwardKinematics::getRElbowYaw(vector<float> jointAngles)
{
	KinematicMatrix RElbowYaw2RShoulderRoll = 
		KinematicMatrix::transX(LINKS::UPPER_ARM_LENGTH) *
		KinematicMatrix::transY(-LINKS::ELBOW_OFFSET_Y) *
		KinematicMatrix::rotX( jointAngles.at(2) );

	return getRShoulderRoll( jointAngles ) * RElbowYaw2RShoulderRoll;
}

// Right Elbow Roll
KinematicMatrix ForwardKinematics::getRElbowRoll(vector<float> jointAngles)
{
	KinematicMatrix RElbowRoll2RElbowYaw = 
		KinematicMatrix::rotZ( jointAngles.at(3) );

	return getRElbowYaw( jointAngles ) * RElbowRoll2RElbowYaw;
}

// Left Wrist Yaw 
KinematicMatrix ForwardKinematics::getRWristYaw(vector<float> jointAngles)
{
	KinematicMatrix RWristYaw2RElbowRoll = 
		KinematicMatrix::transX(LINKS::LOWER_ARM_LENGTH) *
		KinematicMatrix::rotX( jointAngles.at(4) );
	
	return getRElbowRoll( jointAngles ) * RWristYaw2RElbowRoll;
}

// Right Hand
KinematicMatrix ForwardKinematics::getRHand(vector<float> jointAngles)
{
	KinematicMatrix RHand2RWristYaw = 
		KinematicMatrix::transX(LINKS::HAND_OFFSET_X);// *
		//KinematicMatrix::transZ(Blackboard::HAND_OFFSET_Z) *
		//KinematicMatrix::rotY( jointAngles.at(5) );
	
	return getRWristYaw( jointAngles ) * RHand2RWristYaw;
}



/*  +----------+
 *  | Left Leg |
 *  +----------+
 */

// LHipYawPitch
KinematicMatrix ForwardKinematics::getLHipYawPitch(vector<float> jointAngles)
{
	// From Torso to Hip
	KinematicMatrix LHipBase2Torso =	
		KinematicMatrix::transZ(-LINKS::HIP_OFFSET_Z) *
		KinematicMatrix::transY(LINKS::HIP_OFFSET_Y);

	// From Hip to rotated Hip
	KinematicMatrix LHipYawPitch2LHipBase = 
		KinematicMatrix::rotX(-45.0f * TO_RAD) * 
		KinematicMatrix::rotY(jointAngles.at(0));

	// From rotated Hip to Torso
	return LHipBase2Torso * LHipYawPitch2LHipBase;
}

// LHipRoll
KinematicMatrix ForwardKinematics::getLHipRoll(vector<float> jointAngles)
{
	// From LHipRoll to LHipYawPitch
	KinematicMatrix LHipRoll2LHipYawPitch = 
		KinematicMatrix::rotX(45.0f * TO_RAD + jointAngles.at(1));

	// From LHipRoll to Torso
	return getLHipYawPitch( jointAngles ) * LHipRoll2LHipYawPitch;
}


// LPelvis
KinematicMatrix ForwardKinematics::getLPelvis(vector<float> jointAngles)
{
	// From LPelvis to LHipYawPitch
	KinematicMatrix LPelvis2LHipYawPitch = 
		KinematicMatrix::rotX(45.0f * TO_RAD );

	// From LHipRoll to Torso
	return getLHipYawPitch( jointAngles ) * LPelvis2LHipYawPitch;
}

// LHipPitch
KinematicMatrix ForwardKinematics::getLHipPitch(vector<float> jointAngles)
{
	// From LHipPitch to LHipRoll
	KinematicMatrix LHipPitch2LHipRoll = 
		KinematicMatrix::rotY( jointAngles.at(2) );

	// From LHipPitch to Torso
	return getLHipRoll( jointAngles ) * LHipPitch2LHipRoll;
}

// LKneePitch
KinematicMatrix ForwardKinematics::getLKneePitch(vector<float> jointAngles)
{
	// From LKneePitch to LHipPitch
	KinematicMatrix LKneePitch2LHipPitch = 
		KinematicMatrix::transZ(-LINKS::THIGH_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(3) );

	// From LKneePitch to Torso
	return getLHipPitch( jointAngles ) * LKneePitch2LHipPitch;
}

// LAnklePitch
KinematicMatrix ForwardKinematics::getLAnklePitch(vector<float> jointAngles)
{
	// From LAnklePitch to LKneePitch
	KinematicMatrix LAnklePitch2LKneePitch = 
		KinematicMatrix::transZ(-LINKS::TIBIA_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(4) );

	// From LAnklePitch to Torso
	return getLKneePitch( jointAngles ) * LAnklePitch2LKneePitch;
}

// LAnklePitch
KinematicMatrix ForwardKinematics::getLAnkleRoll(vector<float> jointAngles)
{
	// From LAnkleRoll to LAnklePitch
	KinematicMatrix LAnkleRoll2LAnklePitch = 
		KinematicMatrix::rotX( jointAngles.at(5) );

	// From LAnkleRoll to Torso
	return getLAnklePitch( jointAngles ) * LAnkleRoll2LAnklePitch;
}

// LFoot
KinematicMatrix ForwardKinematics::getLFoot(vector<float> jointAngles)
{
	// From LFoot to LAnkleRoll
	KinematicMatrix LFoot2LAnkleRoll = 
		KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT);

	// From LFoot to Torso
	return getLAnkleRoll( jointAngles ) * LFoot2LAnkleRoll;
}


/*  +-----------+
 *  | Right Leg |
 *  +-----------+
 */


// RHipYawPitch
KinematicMatrix ForwardKinematics::getRHipYawPitch(vector<float> jointAngles)
{
	// From Torso to Hip
	KinematicMatrix RHipBase2Torso =	
		KinematicMatrix::transZ(-LINKS::HIP_OFFSET_Z) *
		KinematicMatrix::transY(-LINKS::HIP_OFFSET_Y);

	// From Hip to rotated Hip
	KinematicMatrix RHipYawPitch2LHipBase = 
		KinematicMatrix::rotX(-135 * TO_RAD) * 
		KinematicMatrix::rotY(-jointAngles.at(0));

	// From rotated Hip to Torso
	return RHipBase2Torso * RHipYawPitch2LHipBase;
}


// RPelvis
KinematicMatrix ForwardKinematics::getRPelvis(vector<float> jointAngles)
{
	// From LPelvis to RHipYawPitch
	KinematicMatrix RPelvis2RHipYawPitch = 
		KinematicMatrix::rotX(135.0f * TO_RAD );

	// From LHipRoll to Torso
	return getRHipYawPitch( jointAngles ) * RPelvis2RHipYawPitch;
}

// RHipRoll
KinematicMatrix ForwardKinematics::getRHipRoll(vector<float> jointAngles)
{
	// From LHipRoll to LHipYawPitch
	KinematicMatrix RHipRoll2RHipYawPitch = 
		KinematicMatrix::rotX(135 * TO_RAD + jointAngles.at(1));

	// From LHipRoll to Torso
	return getRHipYawPitch( jointAngles ) * RHipRoll2RHipYawPitch;
}

// RHipPitch
KinematicMatrix ForwardKinematics::getRHipPitch(vector<float> jointAngles)
{
	// From RHipPitch to RHipRoll
	KinematicMatrix RHipPitch2RHipRoll = 
		KinematicMatrix::rotY( jointAngles.at(2) );

	// From LHipPitch to Torso
	return getRHipRoll( jointAngles ) * RHipPitch2RHipRoll;
}

// RKneePitch
KinematicMatrix ForwardKinematics::getRKneePitch(vector<float> jointAngles)
{
	// From RKneePitch to RHipPitch
	KinematicMatrix RKneePitch2RHipPitch = 
		KinematicMatrix::transZ(-LINKS::THIGH_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(3) );

	// From RKneePitch to Torso
	return getRHipPitch( jointAngles ) * RKneePitch2RHipPitch;
}

// RAnklePitch
KinematicMatrix ForwardKinematics::getRAnklePitch(vector<float> jointAngles)
{
	// From RAnklePitch to RKneePitch
	KinematicMatrix RAnklePitch2RKneePitch = 
		KinematicMatrix::transZ(-LINKS::TIBIA_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(4) );

	// From RAnklePitch to Torso
	return getRKneePitch( jointAngles ) * RAnklePitch2RKneePitch;
}

// RAnklePitch
KinematicMatrix ForwardKinematics::getRAnkleRoll(vector<float> jointAngles)
{
	// From RAnkleRoll to RAnklePitch
	KinematicMatrix RAnkleRoll2RAnklePitch = 
		KinematicMatrix::rotX( jointAngles.at(5) );

	// From RAnkleRoll to Torso
	return getRAnklePitch( jointAngles ) * RAnkleRoll2RAnklePitch;
}

// RFoot
KinematicMatrix ForwardKinematics::getRFoot(vector<float> jointAngles)
{
	// From RFoot to RAnkleRoll
	KinematicMatrix RFoot2RAnkleRoll = 
		KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT);

	// From LFoot to Torso
	return getRAnkleRoll( jointAngles ) * RFoot2RAnkleRoll;
}

vector<KinematicMatrix> ForwardKinematics::getHead(vector<float> jointAngles)
{
	KinematicMatrix HeadYaw2Torso = 
		KinematicMatrix::transZ(LINKS::NECK_OFFSET_Z) *
		KinematicMatrix::rotZ(jointAngles.at(0));

	KinematicMatrix HeadPitch2HeadYaw = 
		KinematicMatrix::rotY(jointAngles.at(1));

	KinematicMatrix HeadPitch2Torso = HeadYaw2Torso * HeadPitch2HeadYaw;

	vector<KinematicMatrix> out;
	out.push_back(HeadYaw2Torso);
	out.push_back(HeadPitch2Torso);

	return out;
}

vector<KinematicMatrix> ForwardKinematics::getLArm(vector<float> jointAngles)
{
	KinematicMatrix LShoulderBase2Torso = 
		KinematicMatrix::transZ(LINKS::SHOULDER_OFFSET_Z) *
		KinematicMatrix::transY(LINKS::SHOULDER_OFFSET_Y);

	KinematicMatrix LShoulderPitch2LShoulderBase = 
		KinematicMatrix::rotY(jointAngles.at(0));

	// Left Shoulder Pitch
	KinematicMatrix LShoulderPitch2Torso = LShoulderBase2Torso * LShoulderPitch2LShoulderBase;

	KinematicMatrix LShoulderRoll2LShoulderPitch = 
		KinematicMatrix::rotZ(jointAngles.at(1));

	// Left Shoulder Roll
	KinematicMatrix LShoulderRoll2Torso = LShoulderPitch2Torso * LShoulderRoll2LShoulderPitch;

	KinematicMatrix LElbowYaw2LShoulderRoll = 
		KinematicMatrix::transX(LINKS::UPPER_ARM_LENGTH) *
		KinematicMatrix::transY(LINKS::ELBOW_OFFSET_Y) *
		KinematicMatrix::rotX( jointAngles.at(2) );

	// Left Elbow Yaw
	KinematicMatrix LElbowYaw2Torso = LShoulderRoll2Torso * LElbowYaw2LShoulderRoll;

	
	KinematicMatrix LElbowRoll2LElbowYaw = 
		KinematicMatrix::rotZ( jointAngles.at(3) );
	
	// Left Elbow Roll
	KinematicMatrix LElbowRoll2Torso = LElbowYaw2Torso * LElbowRoll2LElbowYaw;

	KinematicMatrix LWristYaw2LElbowRoll = 
		KinematicMatrix::transX(LINKS::LOWER_ARM_LENGTH) *
		KinematicMatrix::rotX( jointAngles.at(4) );
	
	// Left Wrist Yaw
	KinematicMatrix LWrist2Torso = LElbowRoll2Torso * LWristYaw2LElbowRoll;

	KinematicMatrix LHand2LWristYaw = 
		KinematicMatrix::transX(LINKS::HAND_OFFSET_X) *
		//KinematicMatrix::transZ(Blackboard::HAND_OFFSET_Z) *
		KinematicMatrix::rotY( jointAngles.at(5) );
	
	KinematicMatrix LHand2Torso = LWrist2Torso * LHand2LWristYaw;

	vector<KinematicMatrix> out;
	out.push_back(LShoulderPitch2Torso);
	out.push_back(LShoulderRoll2Torso);
	out.push_back(LElbowYaw2Torso);
	out.push_back(LElbowRoll2Torso);
	out.push_back(LWrist2Torso);
	out.push_back(LHand2Torso);

	return out;



}
vector<KinematicMatrix> ForwardKinematics::getRArm(vector<float> jointAngles)
{
	KinematicMatrix RShoulderBase2Torso = 
		KinematicMatrix::transZ(LINKS::SHOULDER_OFFSET_Z) *
		KinematicMatrix::transY(-LINKS::SHOULDER_OFFSET_Y);

	KinematicMatrix RShoulderPitch2RShoulderBase = 
		KinematicMatrix::rotY(jointAngles.at(0));

	// Right Shoulder Pitch
	KinematicMatrix RShoulderPitch2Torso = RShoulderBase2Torso * RShoulderPitch2RShoulderBase;

	KinematicMatrix RShoulderRoll2RShoulderPitch = 
		KinematicMatrix::rotZ(jointAngles.at(1));

	// Right Shoulder Roll 
	KinematicMatrix RShoulderRoll2Torso = RShoulderPitch2Torso * RShoulderRoll2RShoulderPitch;

	KinematicMatrix RElbowYaw2RShoulderRoll = 
		KinematicMatrix::transX(LINKS::UPPER_ARM_LENGTH) *
		KinematicMatrix::transY(-LINKS::ELBOW_OFFSET_Y) *
		KinematicMatrix::rotX( jointAngles.at(2) );

	// Right Elbow Yaw
	KinematicMatrix RElbowYaw2Torso = RShoulderRoll2Torso * RElbowYaw2RShoulderRoll;

	KinematicMatrix RElbowRoll2RElbowYaw = 
		KinematicMatrix::rotZ( jointAngles.at(3) );

	// Right Elbow Roll
	KinematicMatrix RElbowRoll2Torso = RElbowYaw2Torso * RElbowRoll2RElbowYaw;

	KinematicMatrix RWristYaw2RElbowRoll = 
		KinematicMatrix::transX(LINKS::LOWER_ARM_LENGTH) *
		KinematicMatrix::rotX( jointAngles.at(4) );
	
	// Right Wrist Yaw
	KinematicMatrix RWristYaw2Torso = RElbowRoll2Torso * RWristYaw2RElbowRoll;

	KinematicMatrix RHand2RWristYaw = 
		KinematicMatrix::transX(LINKS::HAND_OFFSET_X) *
		//KinematicMatrix::transZ(Blackboard::HAND_OFFSET_Z) *
		KinematicMatrix::rotY( jointAngles.at(5) );
	
	KinematicMatrix RHand2Torso = RWristYaw2Torso * RHand2RWristYaw;

	vector<KinematicMatrix> out;
	out.push_back(RShoulderPitch2Torso);
	out.push_back(RShoulderRoll2Torso);
	out.push_back(RElbowYaw2Torso);
	out.push_back(RElbowRoll2Torso);
	out.push_back(RWristYaw2Torso);
	out.push_back(RHand2Torso);

	return out;
}
vector<KinematicMatrix> ForwardKinematics::getLLeg(vector<float> jointAngles)
{
	// From Torso to Hip
	KinematicMatrix LHipBase2Torso =	
		KinematicMatrix::transZ(-LINKS::HIP_OFFSET_Z) *
		KinematicMatrix::transY(LINKS::HIP_OFFSET_Y);

	// From Hip to rotated Hip
	KinematicMatrix LHipYawPitch2LHipBase = 
		KinematicMatrix::rotX(-45.0f * TO_RAD) * 
		KinematicMatrix::rotY(jointAngles.at(0));

	// From rotated Hip to Torso
	KinematicMatrix LHipYawPitch2Torso =  LHipBase2Torso * LHipYawPitch2LHipBase;

	// From LHipRoll to LHipYawPitch
	KinematicMatrix LHipRoll2LHipYawPitch = 
		KinematicMatrix::rotX(45.0f * TO_RAD + jointAngles.at(1));

	// From LHipRoll to Torso
	KinematicMatrix LHipRoll2Torso =  LHipYawPitch2Torso * LHipRoll2LHipYawPitch;

	// From LHipPitch to LHipRoll
	KinematicMatrix LHipPitch2LHipRoll = 
		KinematicMatrix::rotY( jointAngles.at(2) );

	// From LHipPitch to Torso
	KinematicMatrix LHipPitch2Torso = LHipRoll2Torso * LHipPitch2LHipRoll;

	// From LKneePitch to LHipPitch
	KinematicMatrix LKneePitch2LHipPitch = 
		KinematicMatrix::transZ(-LINKS::THIGH_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(3) );

	// From LKneePitch to Torso
	KinematicMatrix LKneePitch2Torso = LHipPitch2Torso * LKneePitch2LHipPitch;

	// From LAnklePitch to LKneePitch
	KinematicMatrix LAnklePitch2LKneePitch = 
		KinematicMatrix::transZ(-LINKS::TIBIA_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(4) );

	// From LAnklePitch to Torso
	KinematicMatrix LAnklePitch2Torso = LKneePitch2Torso * LAnklePitch2LKneePitch;

	// From LAnkleRoll to LAnklePitch
	KinematicMatrix LAnkleRoll2LAnklePitch = 
		KinematicMatrix::rotX( jointAngles.at(5) );

	// From LAnkleRoll to Torso
	KinematicMatrix LAnkleRoll2Torso = LAnklePitch2Torso * LAnkleRoll2LAnklePitch;

	// From LFoot to LAnkleRoll
	KinematicMatrix LFoot2LAnkleRoll = 
		KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT);

	// From LFoot to Torso
	KinematicMatrix LFoot2Torso = LAnkleRoll2Torso * LFoot2LAnkleRoll;

	// From LPelvis to LHipYawPitch
	KinematicMatrix LPelvis2LHipYawPitch = 
		KinematicMatrix::rotX(45.0f * TO_RAD );

	// From LHipRoll to Torso
	KinematicMatrix LPelvis2Torso = LHipYawPitch2Torso * LPelvis2LHipYawPitch;

	vector<KinematicMatrix> out;
	out.push_back(LPelvis2Torso);
	out.push_back(LHipRoll2Torso);
	out.push_back(LHipPitch2Torso);
	out.push_back(LKneePitch2Torso);
	out.push_back(LAnklePitch2Torso);
	out.push_back(LAnkleRoll2Torso);
	out.push_back(LFoot2Torso);

	return out;

}
vector<KinematicMatrix> ForwardKinematics::getRLeg(vector<float> jointAngles)
{
	// From Torso to Hip
	KinematicMatrix RHipBase2Torso =	
		KinematicMatrix::transZ(-LINKS::HIP_OFFSET_Z) *
		KinematicMatrix::transY(-LINKS::HIP_OFFSET_Y);

	// From Hip to rotated Hip
	KinematicMatrix RHipYawPitch2LHipBase = 
		KinematicMatrix::rotX(-135 * TO_RAD) * 
		KinematicMatrix::rotY(-jointAngles.at(0));

	// From rotated Hip to Torso
	KinematicMatrix RHipYawPitch2Torso = RHipBase2Torso * RHipYawPitch2LHipBase;	

	// From LHipRoll to LHipYawPitch
	KinematicMatrix RHipRoll2RHipYawPitch = 
		KinematicMatrix::rotX(135 * TO_RAD + jointAngles.at(1));

	// From LHipRoll to Torso
	KinematicMatrix RHipRoll2Torso = RHipYawPitch2Torso * RHipRoll2RHipYawPitch;

	// From RHipPitch to RHipRoll
	KinematicMatrix RHipPitch2RHipRoll = 
		KinematicMatrix::rotY( jointAngles.at(2) );

	// From LHipPitch to Torso
	KinematicMatrix RHipPitch2Torso = RHipRoll2Torso * RHipPitch2RHipRoll;

	// From RKneePitch to RHipPitch
	KinematicMatrix RKneePitch2RHipPitch = 
		KinematicMatrix::transZ(-LINKS::THIGH_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(3) );

	// From RKneePitch to Torso
	KinematicMatrix RKneePitch2Torso = RHipPitch2Torso * RKneePitch2RHipPitch;

	// From RAnklePitch to RKneePitch
	KinematicMatrix RAnklePitch2RKneePitch = 
		KinematicMatrix::transZ(-LINKS::TIBIA_LENGTH) *
		KinematicMatrix::rotY( jointAngles.at(4) );

	// From RAnklePitch to Torso
	KinematicMatrix RAnklePitch2Torso = RKneePitch2Torso * RAnklePitch2RKneePitch;

	// From RAnkleRoll to RAnklePitch
	KinematicMatrix RAnkleRoll2RAnklePitch = 
		KinematicMatrix::rotX( jointAngles.at(5) );

	// From RAnkleRoll to Torso
	KinematicMatrix RAnkleRoll2Torso = RAnklePitch2Torso * RAnkleRoll2RAnklePitch;

	// From RFoot to RAnkleRoll
	KinematicMatrix RFoot2RAnkleRoll = 
		KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT);

	// From LFoot to Torso
	KinematicMatrix RFoot2Torso = RAnkleRoll2Torso * RFoot2RAnkleRoll;

	// From LPelvis to RHipYawPitch
	KinematicMatrix RPelvis2RHipYawPitch = 
		KinematicMatrix::rotX(135.0f * TO_RAD );

	// From LHipRoll to Torso
	KinematicMatrix RPelvis2Torso = RHipYawPitch2Torso * RPelvis2RHipYawPitch;

	vector<KinematicMatrix> out;

	out.push_back(RPelvis2Torso);
	out.push_back(RHipRoll2Torso);
	out.push_back(RHipPitch2Torso);
	out.push_back(RKneePitch2Torso);
	out.push_back(RAnklePitch2Torso);
	out.push_back(RAnkleRoll2Torso);
	out.push_back(RFoot2Torso);

	return out;
}

///Robot constants NAO H25 - V3.3

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include "Tools/Math/Vector3.h"

#ifndef M_PI
#define M_PI    3.14159265358979323846f 
#endif

#define TO_RAD (float) (M_PI/180)
/// @namespace JOINTS Robot Joints
namespace JOINTS{
	/// Enumeration for Joints
	enum JOINTS{ HEAD_YAW,
							 HEAD_PITCH,
							 L_SHOULDER_PITCH,
							 L_SHOULDER_ROLL,
							 L_ELBOW_YAW,
							 L_ELBOW_ROLL,
							 L_WRIST_YAW,
							 L_HAND,
							 L_HIP_YAW_PITCH,
							 L_HIP_ROLL,
							 L_HIP_PITCH,
							 L_KNEE_PITCH,
							 L_ANKLE_PITCH,
							 L_ANKLE_ROLL,
							 R_HIP_YAW_PITCH,
							 R_HIP_ROLL,
							 R_HIP_PITCH,
							 R_KNEE_PITCH,
							 R_ANKLE_PITCH,
							 R_ANKLE_ROLL,
							 R_SHOULDER_PITCH,
							 R_SHOULDER_ROLL,
							 R_ELBOW_YAW,
							 R_ELBOW_ROLL,
							 R_WRIST_YAW,
							 R_HAND,
							 JOINTS_MAX
						 };
}
/// @namespace ELEMENTS Robot Elements
namespace ELEMENTS{
	/// Enumeration for robot components
	enum ELEMENTS{ HEAD,
								 NECK,
								 TORSO,
								 L_SHOULDER,
								 L_BICEP,
								 L_ELBOW,
								 L_FOREARM,
								 L_HAND,
								 L_PELVIS,
								 L_HIP,
								 L_THIGH,
								 L_TIBIA,
								 L_ANKLE,
								 L_FOOT,
								 R_PELVIS,
								 R_HIP,
								 R_THIGH,
								 R_TIBIA,
								 R_ANKLE,
								 R_FOOT,
								 R_SHOULDER,
								 R_BICEP,
								 R_ELBOW,
								 R_FOREARM,
								 R_HAND,
								 ELEMENTS_MAX
							 };
}

/// Enum for FSR
enum ENUM_FSR{
	L_FL,
	L_FR,
	L_RL,
	L_RR,
	R_FL,
	R_FR,
	R_RL,
	R_RR,
	FSR_MAX
};


// lookup table HeadYaw <-> HeadPitch
const int LOOKUP_HEAD_POINTS = 13;
/// Lookup table for HeadYaw <-> HeadPitch movement
const Vector3<float> LOOKUP_HEAD[LOOKUP_HEAD_POINTS] =
{
	Vector3<float>(-119.52f	* TO_RAD, -25.73f	* TO_RAD, 19.91f	* TO_RAD),
	Vector3<float>(-87.49f	* TO_RAD, -18.91f	* TO_RAD, 11.46f	* TO_RAD),
	Vector3<float>(-62.45f	* TO_RAD, -24.64f	* TO_RAD, 17.19f	* TO_RAD),
	Vector3<float>(-51.74f	*	TO_RAD, -27.50f	* TO_RAD, 18.91f	* TO_RAD),
	Vector3<float>(-43.32f	* TO_RAD, -31.40f	* TO_RAD, 21.20f	* TO_RAD),
	Vector3<float>(-27.85f	* TO_RAD, -38.50f	* TO_RAD, 24.18f	* TO_RAD),
	Vector3<float>(  0.00f	* TO_RAD, -38.50f	* TO_RAD, 29.51f	* TO_RAD),
	Vector3<float>( 27.85f	* TO_RAD, -38.50f	* TO_RAD, 24.18f	* TO_RAD),
	Vector3<float>( 43.32f	* TO_RAD, -31.40f	* TO_RAD,	21.20f	* TO_RAD),
	Vector3<float>( 51.74f	* TO_RAD, -27.50f	* TO_RAD, 19.91f	* TO_RAD),
	Vector3<float>( 62.45f	* TO_RAD, -24.64f	* TO_RAD, 17.91f	* TO_RAD),
	Vector3<float>( 87.49f	* TO_RAD, -19.91f	* TO_RAD, 11.46f	* TO_RAD),
	Vector3<float>(119.52f	* TO_RAD,	-25.73f	* TO_RAD, 18.91f	* TO_RAD)
};

// lookup table LAnklePitch <-> LAnkleRoll
const int LOOKUP_ANKLE_POINTS = 18;
/// Lookup table for AnklePitch <-> AnkleRoll movement
const Vector3<float> LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS] =
{
	Vector3<float>(-70.0f	* TO_RAD, -03.0f	* TO_RAD, 06.0f	* TO_RAD),
	Vector3<float>(-65.0f	* TO_RAD, -03.0f	* TO_RAD, 06.0f	* TO_RAD),
	Vector3<float>(-60.0f	* TO_RAD, -04.0f	* TO_RAD, 06.0f	* TO_RAD),
	Vector3<float>(-55.0f	* TO_RAD, -04.0f	* TO_RAD, 06.0f	* TO_RAD),
	Vector3<float>(-50.0f	* TO_RAD, -05.0f	* TO_RAD, 06.0f	* TO_RAD),
	Vector3<float>(-45.0f	* TO_RAD, -09.0f	* TO_RAD, 13.0f	* TO_RAD),
	Vector3<float>(-40.0f	* TO_RAD, -15.0f	* TO_RAD, 15.0f	* TO_RAD),
	Vector3<float>(-35.0f	* TO_RAD, -32.0f	* TO_RAD, 16.0f	* TO_RAD),
	Vector3<float>(-30.0f	* TO_RAD, -42.0f	* TO_RAD, 22.0f	* TO_RAD),
	Vector3<float>(-25.0f	* TO_RAD, -45.0f	* TO_RAD, 25.0f	* TO_RAD),
	Vector3<float>( 10.0f	* TO_RAD, -45.0f	* TO_RAD, 25.0f	* TO_RAD),
	Vector3<float>( 15.0f	* TO_RAD, -40.0f	* TO_RAD, 25.0f	* TO_RAD),
	Vector3<float>( 20.0f	* TO_RAD, -35.0f	* TO_RAD, 25.0f	* TO_RAD),
	Vector3<float>( 25.0f	* TO_RAD, -25.0f	* TO_RAD, 20.0f	* TO_RAD),
	Vector3<float>( 30.0f	* TO_RAD, -25.0f	* TO_RAD, 20.0f	* TO_RAD),
	Vector3<float>( 35.0f	* TO_RAD, -20.0f	* TO_RAD, 20.0f	* TO_RAD),
	Vector3<float>( 40.0f	* TO_RAD, -15.0f	* TO_RAD, 10.0f	* TO_RAD),
	Vector3<float>( 45.0f	* TO_RAD, -10.0f	* TO_RAD, 06.0f	* TO_RAD)
};

/// minimal range for joints
const float MIN_RANGE[JOINTS::JOINTS_MAX] =
{	-119.5f		* TO_RAD,		// HeadYaw
	-38.5f		* TO_RAD,		// HeadPitch
	-119.5f		* TO_RAD,		// LShoulderPitch
	-18.0f		* TO_RAD,		// LShoulderRoll
	-119.5f		* TO_RAD,		// LElbowYaw
	-88.5f		* TO_RAD,		// LElbowRoll
	-104.5f		* TO_RAD,		// LWristYaw
	 0.0f			* TO_RAD,		// LHand
	-65.62f		* TO_RAD,		// LHipYawPitch
	-21.74f		* TO_RAD,		// LHipRoll
	-101.63f	* TO_RAD,		// LHipPitch
	-5.29f		* TO_RAD,		// LKneePitch
	-68.15f		* TO_RAD,		// LAnklePitch
	-44.06f		* TO_RAD,		// LAnkleRoll
	-65.62f		* TO_RAD,		// RHipYawPitch
	-42.3f		* TO_RAD,		// RHipRoll
	-101.54f	* TO_RAD,		// RHipPitch
	-5.9f			* TO_RAD,		// RKneePitch
	-67.97f		* TO_RAD,		// RAnklePitch
	-22.27f		* TO_RAD,		// RAnkleRoll
	-119.5f		* TO_RAD,		// RShoulderPitch
	-76.0f		* TO_RAD,		// RShoulderRoll
	-119.5f		* TO_RAD,		// RElbowYaw
	 2.0f			* TO_RAD,		// RElbowRoll
	-104.5f		* TO_RAD,		// RWristYaw
	 0.0f			* TO_RAD		// RHand
 };

/// Minimum Range for HeadPitch dependent on HeadYaw
extern const float MIN_RANGE_HEAD_PITCH(const float& HeadYaw);
/// Minimum Range for LAnkleRoll dependent on LAnklePitch
extern const float MIN_RANGE_L_ANKLE_ROLL(const float& LAnklePitch);
/// Minimum Range for RAnkleRoll dependent on RAnklePitch
extern const float MIN_RANGE_R_ANKLE_ROLL(const float& RAnklePitch);

/// Maximal range for joints
const float MAX_RANGE[JOINTS::JOINTS_MAX] =
{
	119.5f		* TO_RAD,		// HeadYaw
	29.5f			* TO_RAD,		// HeadPitch
	119.5f		* TO_RAD,		// LShoulderPitch
	76.0f			* TO_RAD,		// LShoulderRoll
	119.5f		* TO_RAD,		// LElbowYaw
	-2.0f			* TO_RAD,		// LElbowRoll
	104.5f		* TO_RAD,		// LWristYaw
	1.0f			* TO_RAD,		// LHand
	42.44f		* TO_RAD,		// LHipYawPitch
	45.29f		* TO_RAD,		// LHipRoll
	27.73f		* TO_RAD,		// LHipPitch
	121.04f		* TO_RAD,		// LKneePitch
	52.86f		* TO_RAD,		// LAnklePitch
	22.79f		* TO_RAD,		// LAnkleRoll
	42.44f		* TO_RAD,		// RHipYawPitch
	23.76f		* TO_RAD,		// RHipRoll
	27.82f		* TO_RAD,		// RHipPitch
	121.47f		* TO_RAD,		// RKneePitch
	53.4f			* TO_RAD,		// RAnklePitch
	45.03f		* TO_RAD,		// RAnkleRoll
	119.5f		* TO_RAD,		// RShoulderPitch
	18.0f			* TO_RAD,		// RShoulderRoll
	119.5f		* TO_RAD,		// RElbowYaw
	88.5f			* TO_RAD,		// RElbowRoll
	104.5f		* TO_RAD,		// RWristYaw
	1.0f			* TO_RAD		// RHand
};

/// Maximum Range for HeadPitch dependent on HeadYaw
extern const float MAX_RANGE_HEAD_PITCH(const float& HeadYaw);
/// Maximum Range for LAnkleRoll dependent on LAnklePitch
extern const float MAX_RANGE_L_ANKLE_ROLL(const float& LAnklePitch);
/// Maximum Range for RAnkleRoll dependent on RAnklePitch
extern const float MAX_RANGE_R_ANKLE_ROLL(const float& RAnklePitch);

/// Masses for Elements
const float MASS[ELEMENTS::ELEMENTS_MAX] =
{
	0.52065f,		// Head
	0.05930f,		// Neck
	1.03948f,		// Torso
	0.06996f,		// LShoulder
	0.12309f,		// LBicep
	0.05971f,		// LElbow
	0.07724f,		// LForeArm
	0.16653f,		// LHand
	0.07117f,		// LPelvis
	0.13530f,		// LHip
	0.39421f,		// LThigh
	0.29159f,		// LTibia
	0.13892f,		// LAnkle
	0.16175f,		// LFoot
	0.07117f,		// RPelvis
	0.13530f,		// RHip
	0.39421f,		// RThigh
	0.29159f,		// RTibia
	0.13892f,		// RAnkle
	0.16175f,		// RFoot
	0.06996f,		// RShoulder
	0.12309f,		// RBicep
	0.05971f,		// RElbow
	0.07724f,		// RForeArm
	0.16653f		// RHand
};

/// Center of Masses of Elements
const Vector3<float> COM[ELEMENTS::ELEMENTS_MAX] =
{
	Vector3<float>(1.2f,		0.84f,		53.53f),		// Head
	Vector3<float>(-0.02f,	0.17f,		-25.56f),		// Neck
	Vector3<float>(-4.15f,	0.07f,		42.58f),		// Torso
	Vector3<float>(-1.78f,	-24.96f,	0.18f),			// LShoulder
	Vector3<float>(18.85f,	5.77f,		0.65f),			// LBicep
	Vector3<float>(-25.6f,	-0.01f,		-0.19f),		// LElbow
	Vector3<float>(25.56f,	2.73f,		0.96f),			// LForeArm
	Vector3<float>(31.8f,		0.83f,		4.77f),			// LHand
	Vector3<float>(-7.66f,	-12.0f,		27.17f),		// LPelvis
	Vector3<float>(-16.49f,	0.29f,		-4.75f),		// LHip
	Vector3<float>(1.32f,		2.35f,		-53.52f),		// LThigh
	Vector3<float>(4.22f,		2.52f,		-48.68f),		// LTibia
	Vector3<float>(1.42f,		0.28f,		6.38f),			// LAnkle
	Vector3<float>(25.4f,		3.32f,		-32.41f),		// LFoot
	Vector3<float>(-7.66f,	12.0f,		27.17f),		// RPelvis
	Vector3<float>(-16.49f,	-0.29f,		-4.75f),		// RHip
	Vector3<float>(1.32f,		-2.35f,		-53.52f),		// RThigh
	Vector3<float>(4.22f,		-2.52f,		-48.68f),		// RTibia
	Vector3<float>(1.42f,		-0.28f,		6.38f),			// RAnkle
	Vector3<float>(25.4f,		-3.32f,		-32.41f),		// RFoot
	Vector3<float>(-1.78f,	24.96f,	0.18f),			// RShoulder
	Vector3<float>(18.85f,	-5.77f,		0.65f),			// RBicep
	Vector3<float>(-25.6f,	0.01f,		-0.19f),		// RElbow
	Vector3<float>(25.56f,	-2.73f,		0.96f),			// RForeArm
	Vector3<float>(31.8f,		-0.83f,		4.77f)			// RHand
};

/// namespace LINKS Links of the robot
namespace LINKS
{
	const float	NECK_OFFSET_Z			= 126.5f;
	const float	SHOULDER_OFFSET_Y	= 98.0f;
	const float	SHOULDER_OFFSET_Z	= 100.0f;
	const float	UPPER_ARM_LENGTH	= 105.0f;
	const float	LOWER_ARM_LENGTH	= 55.95f;
	const float	HAND_OFFSET_X			= 57.75f;
	const float	HAND_OFFSET_Z			= 12.31f;
	const float	HIP_OFFSET_Z			= 85.0f;
	const float	HIP_OFFSET_Y			= 50.0f;
	const float	THIGH_LENGTH			= 100.0f;
	const float	TIBIA_LENGTH			= 102.9f;
	const float	FOOT_HEIGHT				= 45.19f;
	const float	ELBOW_OFFSET_Y		= 15.0f;

	// own defined constants

	/// forearm length (hand + lower arm)
	const float	FOREARM_LENGTH = LOWER_ARM_LENGTH + HAND_OFFSET_X;

	/// maximal arm length (shoulder <-> hand distance)
	const float  ARM_MAX_LENGTH =
			sqrt(	pow(LINKS::UPPER_ARM_LENGTH,2) +
			pow(FOREARM_LENGTH,2) - 2 * UPPER_ARM_LENGTH *
			FOREARM_LENGTH * cos ( (float) M_PI + MAX_RANGE[JOINTS::L_ELBOW_ROLL]));

	/// minimal arm length (shoulder <-> hand distance)
	const float	ARM_MIN_LENGTH =
			sqrt(	pow(LINKS::UPPER_ARM_LENGTH,2) +
			pow(FOREARM_LENGTH,2) - 2 * UPPER_ARM_LENGTH *
			FOREARM_LENGTH * cos ( (float) M_PI + MIN_RANGE[JOINTS::L_ELBOW_ROLL]));

	/// minimal leg length (hip <-> foot distance)
	const float	LEG_MIN_LENGTH =
			sqrt(	pow(TIBIA_LENGTH,2)	+	pow(THIGH_LENGTH,2)	-
						2 * TIBIA_LENGTH		*			THIGH_LENGTH *
						cos ( (float) M_PI - MAX_RANGE[JOINTS::L_KNEE_PITCH]));

	/// maximal leg length (hip <-> foot distance)
	const float	LEG_MAX_LENGTH = TIBIA_LENGTH + THIGH_LENGTH;

	/// max y-position for LElbow
	const float  L_ELBOW_MAX_Y = sin(MAX_RANGE[JOINTS::L_SHOULDER_ROLL]) * UPPER_ARM_LENGTH;

	/// minimal y-position for LElbow
	const float	L_ELBOW_MIN_Y = sin(MIN_RANGE[JOINTS::L_SHOULDER_ROLL]) * UPPER_ARM_LENGTH;

	/// maximal y-position for RElbow
	const float	R_ELBOW_MAX_Y = sin(MAX_RANGE[JOINTS::R_SHOULDER_ROLL]) * UPPER_ARM_LENGTH;

	/// minimal y-position for RElbow
	const float	R_ELBOW_MIN_Y = sin(MIN_RANGE[JOINTS::R_SHOULDER_ROLL]) * UPPER_ARM_LENGTH;
}


/// FSR positions
const Vector3<float> FSR_POSITION[FSR_MAX] =
{
	Vector3<float>(70.25f, 29.9f, 0.0f),
	Vector3<float>(70.25f, -23.2f, 0.0f),
	Vector3<float>(-30.25f, 29.9f, 0.0f),
	Vector3<float>(-29.65f, -19.1f, 0.0f),
	Vector3<float>(70.25f, 23.1f, 0.0f),
	Vector3<float>(70.25f, -29.9f, 0.0f),
	Vector3<float>(-30.25f, 19.1f, 0.0f),
	Vector3<float>(-29.65f, -29.9f, 0.0f)
};

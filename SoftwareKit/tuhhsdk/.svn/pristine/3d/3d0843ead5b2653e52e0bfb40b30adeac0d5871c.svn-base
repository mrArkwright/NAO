/*
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */

#include "Com.h"
#include "Definitions/robotConstants.h"

#include "ForwardKinematics.h"
#include "Tools/Storage/Blackboard.h"

/* --------------*/
/*	  Left Leg	 */
/* --------------*/


// com
Vector3<float> Com::getComLLeg(vector<KinematicMatrix> legKin)
{
	// calculate the com positions relative to the torso

	// LPelvis calculate postion vector
	KinematicMatrix lPelvis		=	legKin[0];
	Vector3<float> r_LPelvis	=	lPelvis * COM[ELEMENTS::L_PELVIS];

	// RHip
	KinematicMatrix lHipRoll	=	legKin[1];
	Vector3<float> r_LHip		=	lHipRoll * COM[ELEMENTS::L_HIP];

	// RThigh
	KinematicMatrix lHipPitch	=	legKin[2];
	Vector3<float> r_LThigh		=	lHipPitch * COM[ELEMENTS::L_THIGH];

	// RTibia
	KinematicMatrix lKneePitch	=	legKin[3];
	Vector3<float> r_LTibia		=	lKneePitch * COM[ELEMENTS::L_TIBIA];

	// RAnkle
	KinematicMatrix lAnklePitch	=	legKin[4];
	Vector3<float> r_LAnkle		=	lAnklePitch * COM[ELEMENTS::L_ANKLE];

	// RFoot
	KinematicMatrix lAnkleRoll	=	legKin[5];
	Vector3<float> r_LFoot		=	lAnkleRoll * COM[ELEMENTS::L_FOOT];

	// sumproduct of mass and position vectors
	Vector3<float> rLegComSumProduct	=
			r_LPelvis	* MASS[ELEMENTS::L_PELVIS]	+
			r_LHip		* MASS[ELEMENTS::L_HIP]			+
			r_LThigh	* MASS[ELEMENTS::L_THIGH]		+
			r_LTibia	* MASS[ELEMENTS::L_TIBIA]		+
			r_LAnkle	* MASS[ELEMENTS::L_ANKLE]		+
			r_LFoot		* MASS[ELEMENTS::L_FOOT];
	
	// position vector of center of mass
	return rLegComSumProduct / getMassLLeg();
}
Vector3<float> Com::getComLLeg()
{
	vector<KinematicMatrix> legKinematics;
	// pelvis is additional so it can't be used as start value for loop
	legKinematics.push_back(Blackboard::getKinematicMatrix(JOINTS::L_PELVIS));

	// fill kinematic vector
	for (int i = JOINTS::L_HIP_ROLL; i <= JOINTS::L_ANKLE_ROLL; i++)
		legKinematics.push_back(Blackboard::getKinematicMatrix(i));

	return getComLLeg(legKinematics);
}

// mass
float Com::getMassLLeg()
{
	return	MASS[ELEMENTS::L_PELVIS]	+
			MASS[ELEMENTS::L_HIP]					+
			MASS[ELEMENTS::L_THIGH]				+
			MASS[ELEMENTS::L_TIBIA]				+
			MASS[ELEMENTS::L_ANKLE]				+
			MASS[ELEMENTS::L_FOOT];
}

/* --------------*/
/*	 Right Leg	 */
/* --------------*/

// com
Vector3<float> Com::getComRLeg(vector<KinematicMatrix> legKin)
{
	// calculate the com positions relative to the torso

	// RPelvis calculate postion vector
	KinematicMatrix rPelvis		=	legKin[0];
	Vector3<float> r_RPelvis	=	rPelvis * COM[ELEMENTS::R_PELVIS];
	// RHip
	KinematicMatrix rHipRoll	=	legKin[1];
	Vector3<float> r_RHip		=	rHipRoll * COM[ELEMENTS::R_HIP];

	// RThigh
	KinematicMatrix rHipPitch	=	legKin[2];
	Vector3<float> r_RThigh		=	rHipPitch * COM[ELEMENTS::R_THIGH];

	// RTibia
	KinematicMatrix rKneePitch	=	legKin[3];
	Vector3<float> r_RTibia		=	rKneePitch * COM[ELEMENTS::R_TIBIA];

	// RAnkle
	KinematicMatrix rAnklePitch	=	legKin[4];
	Vector3<float> r_RAnkle		=	rAnklePitch* COM[ELEMENTS::R_ANKLE];

	// RFoot
	KinematicMatrix rAnkleRoll	=	legKin[5];
	Vector3<float> r_RFoot		=	rAnkleRoll * COM[ELEMENTS::R_FOOT];

	// sumproduct of mass and position vectors
	Vector3<float> rLegComSumProduct	=
			r_RPelvis	* MASS[ELEMENTS::R_PELVIS]	+
			r_RHip		* MASS[ELEMENTS::R_HIP]			+
			r_RThigh	* MASS[ELEMENTS::R_THIGH]		+
			r_RTibia	* MASS[ELEMENTS::R_TIBIA]		+
			r_RAnkle	* MASS[ELEMENTS::R_ANKLE]		+
			r_RFoot		* MASS[ELEMENTS::R_FOOT];
	
	// position vector of center of mass
	return rLegComSumProduct / getMassRLeg();		
}
Vector3<float> Com::getComRLeg()
{
	vector<KinematicMatrix> legKinematics;
	// pelvis is additional so it can't be used as start value for loop
	legKinematics.push_back(Blackboard::getKinematicMatrix(JOINTS::R_PELVIS));

	// fill kinematic vector
	for (int i= JOINTS::R_HIP_ROLL; i <= JOINTS::R_ANKLE_ROLL; i++)
		legKinematics.push_back(Blackboard::getKinematicMatrix(i));

	return getComRLeg(legKinematics);
}

// mass
float Com::getMassRLeg()
{
	return	MASS[ELEMENTS::R_PELVIS]	+
			MASS[ELEMENTS::R_HIP]					+
			MASS[ELEMENTS::R_THIGH]				+
			MASS[ELEMENTS::R_TIBIA]				+
			MASS[ELEMENTS::R_ANKLE]				+
			MASS[ELEMENTS::R_FOOT];
}


/* --------------*/
/*	  Left Arm	 */
/* --------------*/

// com
Vector3<float> Com::getComLArm(vector<KinematicMatrix> armKin)
{
	// shoulder
	KinematicMatrix lShoulderPitch	=	armKin[0];
	Vector3<float> r_LShoulder		=	lShoulderPitch * COM[ELEMENTS::L_SHOULDER];

	// bicep
	KinematicMatrix lShoulderRoll	=	armKin[1];
	Vector3<float> r_LBicep			=	lShoulderRoll * COM[ELEMENTS::L_BICEP];

	// elbow
	KinematicMatrix lElbowYaw		=	armKin[2];
	Vector3<float> r_LElbow			=	lElbowYaw * COM[ELEMENTS::L_ELBOW];

	// forarm 
	KinematicMatrix lElbowRoll		=	armKin[3];
	Vector3<float> r_LForeArm		=	lElbowRoll * COM[ELEMENTS::L_FOREARM];

	// hand
	KinematicMatrix lHand			=	armKin[4];
	Vector3<float> r_LHand			=	lHand * COM[ELEMENTS::L_FOREARM];

	// sumproduct of mass and position vectors
	Vector3<float> lArmComSumProduct	=
			r_LShoulder	* MASS[ELEMENTS::L_SHOULDER]	+
			r_LBicep	* MASS[ELEMENTS::L_BICEP]				+
			r_LElbow	* MASS[ELEMENTS::L_ELBOW]				+
			r_LForeArm	* MASS[ELEMENTS::L_FOREARM]		+
			r_LHand		* MASS[ELEMENTS::L_HAND];

	
	return lArmComSumProduct / getMassLArm();
}
Vector3<float> Com::getComLArm()
{
	vector<KinematicMatrix> armKinematics;

	// fill kinematic vector
	for (int i = JOINTS::L_SHOULDER_PITCH; i <= JOINTS::L_WRIST_YAW; i++)
		armKinematics.push_back(Blackboard::getKinematicMatrix(i));

	return getComLArm(armKinematics);
}

// mass
float Com::getMassLArm()
{
	return	MASS[ELEMENTS::L_SHOULDER]	+
			MASS[ELEMENTS::L_BICEP]					+
			MASS[ELEMENTS::L_ELBOW]					+
			MASS[ELEMENTS::L_FOREARM]				+
			MASS[ELEMENTS::L_HAND];
}


/* --------------*/
/*	 Right Arm	 */
/* --------------*/

// com

Vector3<float> Com::getComRArm(vector<KinematicMatrix> armKin)
{
	// shoulder
	KinematicMatrix rShoulderPitch	=	armKin[0];
	Vector3<float> r_RShoulder		=	rShoulderPitch * COM[ELEMENTS::R_SHOULDER];

	// bicep
	KinematicMatrix rShoulderRoll	=	armKin[1];
	Vector3<float> r_RBicep			=	rShoulderRoll * COM[ELEMENTS::R_BICEP];

	// elbow
	KinematicMatrix rElbowYaw		=	armKin[2];
	Vector3<float> r_RElbow			=	rElbowYaw * COM[ELEMENTS::R_ELBOW];

	// forarm 
	KinematicMatrix rElbowRoll		=	armKin[3];
	Vector3<float> r_RForeArm		=	rElbowRoll * COM[ELEMENTS::R_FOREARM];

	// hand
	KinematicMatrix rHand			=	armKin[4];
	Vector3<float> r_RHand			=	rHand * COM[ELEMENTS::R_HAND];

	// sumproduct of mass and position vectors
	Vector3<float> rArmComSumProduct	=
			r_RShoulder	* MASS[ELEMENTS::R_SHOULDER]	+
			r_RBicep	* MASS[ELEMENTS::R_BICEP]				+
			r_RElbow	* MASS[ELEMENTS::R_ELBOW]				+
			r_RForeArm	* MASS[ELEMENTS::R_FOREARM]		+
			r_RHand		* MASS[ELEMENTS::R_HAND];

	
	return rArmComSumProduct / getMassRArm();
}

Vector3<float> Com::getComRArm()
{
	vector<KinematicMatrix> armKinematics;

	// fill kinematic vector
	for (int i= JOINTS::R_SHOULDER_PITCH; i <= JOINTS::R_WRIST_YAW; i++)
		armKinematics.push_back(Blackboard::getKinematicMatrix(i));

	return getComRArm(armKinematics);
	
}

// mass
float Com::getMassRArm()
{
	return	MASS[ELEMENTS::R_SHOULDER]	+
			MASS[ELEMENTS::R_BICEP]					+
			MASS[ELEMENTS::R_ELBOW]					+
			MASS[ELEMENTS::R_FOREARM]				+
			MASS[ELEMENTS::R_HAND];
}


/* --------------*/
/*	    Head     */
/* --------------*/

// com

Vector3<float> Com::getComHead(vector<KinematicMatrix> headKin)
{
	// HeadYaw
	KinematicMatrix headYaw				=	headKin[0];
	Vector3<float> r_HeadYaw			=	headYaw * COM[ELEMENTS::NECK];

	// HeadPitch
	KinematicMatrix headPitch			=	headKin[1];
	Vector3<float> r_HeadPitch			=	headPitch * COM[ELEMENTS::HEAD];

	Vector3<float> headComSumProduct =
			r_HeadYaw	* MASS[ELEMENTS::NECK] +
			r_HeadPitch	* MASS[ELEMENTS::HEAD];

	return headComSumProduct / getMassHead();
}

Vector3<float> Com::getComHead()
{
	vector<KinematicMatrix> headKinematics;
	headKinematics.push_back(Blackboard::getKinematicMatrix(JOINTS::HEAD_YAW));
	headKinematics.push_back(Blackboard::getKinematicMatrix(JOINTS::HEAD_PITCH));

	return getComHead(headKinematics);
}

// mass
float Com::getMassHead()
{
	return	MASS[ELEMENTS::NECK] +
			MASS[ELEMENTS::HEAD];
}

/* --------------*/
/*	    Body     */
/* --------------*/

// com
Vector3<float> Com::getComBody()
{
	Vector3<float> BodyComSumProduct	=
			getComHead()	*	getMassHead()	+
											getComLArm()	*	getMassLArm()	+
											getComRArm()	*	getMassRArm()	+
											getComLLeg()	*	getMassLLeg()	+
											getComRLeg()	*	getMassRLeg()	+
											COM[ELEMENTS::TORSO] * MASS[ELEMENTS::TORSO];
	return BodyComSumProduct / getMassBody();
}

// mass
float Com::getMassBody()
{
	return	getMassHead()	+
			getMassLArm()	+
			getMassRArm()	+
			getMassLLeg()	+
			getMassRLeg()	+
			MASS[ELEMENTS::TORSO];
}

/* --------------*/
/*   individual  */
/* --------------*/

// com
Vector3<float> Com::getCom(string name)
{
	if (name == "LLeg")
		return getComLLeg();

	else if	(name == "RLeg")
		return getComRLeg();

	else if (name == "LArm")
		return getComLArm();

	else if (name == "RArm")
		return getComRArm();

	else if (name == "Head")
		return getComHead();

	else if (name == "Body")
		return getComBody();

	else
		return Vector3<float>();
}

// mass 
float Com::getMass(string name)
{
	if ( name == "Head")
		return getMassHead();

	else if ( name == "LLeg")
		return getMassLLeg();

	else if ( name == "RLeg")
		return getMassRLeg();

	else if ( name == "LArm")
		return getMassLArm();

	else if ( name == "RArm")
		return getMassRArm();

	else if ( name == "Body")
		return getMassBody();

	else 
		return getMassBody();
}



Vector3<float> Com::getCom(vector<float> jointAngles)
{
	vector<float> headAngles(2);
	vector<float> lArmAngles(6);
	vector<float> rArmAngles(6);
	vector<float> lLegAngles(6);
	vector<float> rLegAngles(6);

	headAngles[0] = jointAngles[0];
	headAngles[1] = jointAngles[1];

	// initialize joint angle vectors
	for (int i = 0; i < 6; i++)
	{
		lArmAngles[i] = jointAngles[i + JOINTS::L_SHOULDER_PITCH];
		rArmAngles[i] = jointAngles[i + JOINTS::R_SHOULDER_PITCH];
		lLegAngles[i] = jointAngles[i + JOINTS::L_HIP_YAW_PITCH];
		rLegAngles[i] = jointAngles[i + JOINTS::R_HIP_YAW_PITCH];
	}

	// get Kinematic matrices to joints
	vector<KinematicMatrix> headKin = ForwardKinematics::getHead(headAngles);
	vector<KinematicMatrix> lArmKin = ForwardKinematics::getLArm(lArmAngles);
	vector<KinematicMatrix> rArmKin = ForwardKinematics::getRArm(rArmAngles);
	vector<KinematicMatrix> lLegKin = ForwardKinematics::getLLeg(lLegAngles);
	vector<KinematicMatrix> rLegKin = ForwardKinematics::getRLeg(rLegAngles);

	Vector3<float> BodyComSumProduct	=
			getComHead(headKin)	*	getMassHead()	+
											getComLArm(lArmKin)	*	getMassLArm()	+
											getComRArm(rArmKin)	*	getMassRArm()	+
											getComLLeg(lLegKin)	*	getMassLLeg()	+
											getComRLeg(rLegKin)	*	getMassRLeg()	+
											COM[ELEMENTS::TORSO] * MASS[ELEMENTS::TORSO];
	return BodyComSumProduct / getMassBody();





}

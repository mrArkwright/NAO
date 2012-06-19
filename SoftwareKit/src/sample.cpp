#include "sample.h"
#include "Tools/Storage/Blackboard.h"
#include "Modules/DcmConnector.h"
#include "Tools/Kinematics/InverseKinematics.h"
#include "Modules/Alias.h"

//positive dz: move foots up, nao shifts down
void Sample::moveUpperBody(const float& dx, const float& dy, const float& dz, const int& time) {
	// store left foot information
	KinematicMatrix leftFoot = Blackboard::getKinematicMatrix(JOINTS::L_FOOT);
	
	// store right foot information
	KinematicMatrix rightFoot = Blackboard::getKinematicMatrix(JOINTS::R_FOOT);
	
	// shift the body around
	leftFoot.posV.x += dx;
	rightFoot.posV.x += dx;
	leftFoot.posV.y += dy;
	rightFoot.posV.y += dy;
	leftFoot.posV.z += dz;
	rightFoot.posV.z += dz;
	
	// calculate new leg angles
	std::vector<float> lLegAngles = InverseKinematics::getLLegAngles(leftFoot);
	std::vector<float> rLegAngles = InverseKinematics::getFixedRLegAngles(rightFoot,lLegAngles[0]);
	
	// send commands
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_L_LEG), "ClearAll", time, lLegAngles);
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_R_LEG), "ClearAll", time, rLegAngles);
}

void Sample::moveFoot(const float& x, const float& y, const float& z, const int& time){
	// store left foot information
	KinematicMatrix leftFoot = Blackboard::getKinematicMatrix(JOINTS::L_FOOT);
	// store right foot information
	KinematicMatrix rightFoot = Blackboard::getKinematicMatrix(JOINTS::R_FOOT);
	
	// move the foot
	rightFoot.posV.x += x;
	rightFoot.posV.y += y;
	rightFoot.posV.z += z;
	
	// calculate new leg angles
	std::vector<float> lLegAngles = InverseKinematics::getLLegAngles(leftFoot);
	std::vector<float> rLegAngles = InverseKinematics::getFixedRLegAngles(rightFoot,lLegAngles[0]);
	
	// send commands
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_L_LEG), "ClearAll", time, lLegAngles);
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_R_LEG), "ClearAll", time, rLegAngles);
}

void Sample::rotateRLeg(const float& hipAngle, const float& kneeAngle, const float& hipYawAngle, const int& time) {
	vector<float> jointAnglesRLeg = Blackboard::getJointAngles("RLeg");
	
	jointAnglesRLeg[2] += hipAngle*3.14/180;
	jointAnglesRLeg[3] += kneeAngle*3.14/180;
	jointAnglesRLeg[0] += hipYawAngle*3.14/180;
	
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_R_LEG), "ClearAll", time, jointAnglesRLeg);
}

void Sample::statBalance(const bool& moveRightFootFlag, const int& time) {
	vector<float> jointAngles = Blackboard::getJointAngles("");
	vector<float> jointAnglesLLeg = Blackboard::getJointAngles("LLeg");
	vector<float> jointAnglesRLeg = Blackboard::getJointAngles("RLeg");
	
	for (int i = 0; i < 5; i++) {
		KinematicMatrix leftFoot = ForwardKinematics::getLFoot(jointAnglesLLeg);
		KinematicMatrix rightFoot = ForwardKinematics::getRFoot(jointAnglesRLeg);
		float foot_distance = abs(leftFoot.posV.y - rightFoot.posV.y);
		
		KinematicMatrix Com2Torso(Com::getCom(jointAngles));
		
		KinematicMatrix Torso2LFoot = leftFoot.invert();
		KinematicMatrix Torso2RFoot = rightFoot.invert();
		
		KinematicMatrix Com2LFoot = Torso2LFoot * Com2Torso;
		KinematicMatrix Com2RFoot = Torso2RFoot * Com2Torso;
		
		Com2LFoot.posV.y = 0;
		Com2LFoot.posV.x = 0;
		Com2RFoot.posV.y = foot_distance;
		Com2RFoot.posV.x = 0;
		
		KinematicMatrix LFoot2Com = Com2LFoot.invert();
		KinematicMatrix RFoot2Com = Com2RFoot.invert();

		leftFoot = Com2Torso * LFoot2Com;
		rightFoot = Com2Torso * RFoot2Com;
		
		// calculate new leg angles
		jointAnglesLLeg = InverseKinematics::getLLegAngles(leftFoot);
		jointAnglesRLeg = InverseKinematics::getFixedRLegAngles(rightFoot,jointAnglesLLeg[0]);
		
		for (int j = 0; j < 6; j++) {
			jointAngles[8 + j] = jointAnglesLLeg[j];
			if (moveRightFootFlag) jointAngles[14 + j] = jointAnglesRLeg[j];
		}
	}
	
	// send commands
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_L_LEG), "ClearAll", time, jointAnglesLLeg);
	if (moveRightFootFlag) DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_R_LEG), "ClearAll", time, jointAnglesRLeg);
}

void Sample::moveLArm(const float& angle,const int& time){
	std::vector<float> LArmAngles = Blackboard::getJointAngles("LArm");

	LArmAngles[1] += angle * 3.14/180;

	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_L_ARM), "ClearAll", time, LArmAngles);
}
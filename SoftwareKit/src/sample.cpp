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

void Sample::moveFoot(const float& heigth, const int& time){
	// store left foot information
	KinematicMatrix leftFoot = Blackboard::getKinematicMatrix(JOINTS::L_FOOT);
	// store right foot information
	KinematicMatrix rightFoot = Blackboard::getKinematicMatrix(JOINTS::R_FOOT);
	
	// move the foot
	rightFoot.posV.z += heigth;
	
	// calculate new leg angles
	std::vector<float> lLegAngles = InverseKinematics::getLLegAngles(leftFoot);
	std::vector<float> rLegAngles = InverseKinematics::getFixedRLegAngles(rightFoot,lLegAngles[0]);
	
	// send commands
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_L_LEG), "ClearAll", time, lLegAngles);
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_R_LEG), "ClearAll", time, rLegAngles);
}

void Sample::statBalance(const int& time){
	// store left foot information
	KinematicMatrix leftFoot = Blackboard::getKinematicMatrix(JOINTS::L_FOOT);
	// store right foot information
	KinematicMatrix rightFoot = Blackboard::getKinematicMatrix(JOINTS::R_FOOT);
	float foot_distance = abs(leftFoot.posV.y - rightFoot.posV.y);

	KinematicMatrix Torso2LFoot = leftFoot.invert();
	KinematicMatrix Torso2RFoot = rightFoot.invert();

	Torso2LFoot.posV.y = 0;
	//Torso2LFoot.rotM *= RotationMatrix::rotX(45*3.14/180);		//funktioniert noch nicht, aus sicht der hüfte drehen??
	Torso2RFoot.posV.y = foot_distance;

	leftFoot = Torso2LFoot.invert();
	rightFoot = Torso2RFoot.invert();

	// calculate new leg angles
	std::vector<float> lLegAngles = InverseKinematics::getLLegAngles(leftFoot);
	std::vector<float> rLegAngles = InverseKinematics::getFixedRLegAngles(rightFoot,lLegAngles[0]);
	
	// send commands
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_L_LEG), "ClearAll", time, lLegAngles);
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_R_LEG), "ClearAll", time, rLegAngles);
}
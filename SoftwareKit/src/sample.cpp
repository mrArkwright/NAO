#include "sample.h"
#include "Tools/Storage/Blackboard.h"
#include "Modules/DcmConnector.h"
#include "Tools/Kinematics/InverseKinematics.h"
#include "Modules/Alias.h"

void Sample::goDown(const float &height, const int &time) {
	// store left foot information
	KinematicMatrix leftFoot = Blackboard::getKinematicMatrix(JOINTS::L_FOOT);
	
	// store right foot information
	KinematicMatrix rightFoot = Blackboard::getKinematicMatrix(JOINTS::R_FOOT);
	
	// change height
	leftFoot.posV.z += height;
	rightFoot.posV.z += height;
	
	// calculate new leg angles
	std::vector<float> lLegAngles = InverseKinematics::getLLegAngles(leftFoot);
	std::vector<float> rLegAngles = InverseKinematics::getFixedRLegAngles(rightFoot,lLegAngles[0]);
	
	// send commands
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_L_LEG), "ClearAll", time, lLegAngles);
	DcmConnector::sendCommands(Alias::aliasName(Alias::JOINT_ACTUATOR_R_LEG), "ClearAll", time, rLegAngles);
}

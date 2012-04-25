#ifndef __Com_h__
#define __Com_h__


#include "../Math/Vector3.h"
#include "KinematicMatrix.h"
#include <string>
#include <vector>

using namespace std;

/// Center of Mass calculation
/**
 * This class calculates the center of mass for a chain of joints
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class Com
{
public:
	/** 
	 * default constructor 
	 */
	Com(){};
	
	
	/** calculates the position of the center of mass of the left leg relative to the torso
     * @param name the name of the chain \n \n
	 * name can be: 
	 * - "Head"
	 * - "LArm"
	 * - "RArm"
	 * - "LLeg"
	 * - "RLeg"
	 * - "Body" 
	 * .
	 * If name does not match any of the valid names, the center of mass of the whole Body is returned
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getCom(string name);

	/** calculate the position of the center of mass of the left leg relative to the torso. \n
	 * The joint angles stored in Blackboard are used for the calculation
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComLLeg();

	/** calculate the position of the center of mass of the left leg relative to the torso
	 * @param legKin A vector containing the kinematic information for the leg joints \n \n
	 * order of legKin
	 * - [0] LHipYawPitch
	 * - [1] LHipRoll
	 * - [2] LHipPitch
	 * - [3] LKneePitch
	 * - [4] LAnklePitch
	 * - [5] LAnkleRoll 
	 * .
	 * legKin can be calculated by ForwardKinematics
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComLLeg(vector<KinematicMatrix> legKin);

	/** calculate the position of the center of mass of the right leg relative to the torso \n
	 * The joint angles stored in Blackboard are used for the calculation
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComRLeg();

	/** calculate the position of the center of mass of the right leg relative to the torso
	 * @param legKin A vector containing the kinematic information for the leg joints \n \n
	 * order of legKin
	 * - [0] RHipYawPitch
	 * - [1] RHipRoll
	 * - [2] RHipPitch
	 * - [3] RKneePitch
	 * - [4] RAnklePitch
	 * - [5] RAnkleRoll 
	 * .
	 * legKin can be calculated by ForwardKinematics
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComRLeg(vector<KinematicMatrix> legKin);

	/** calculate the position of the center of mass of the left arm relative to the torso \n
	 * The joint angles stored in Blackboard are used for the calculation
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComLArm();

	/** calculate the position of the center of mass of the left arm relative to the torso
	 * @param armKin A vector containing the kinematic information of the arm \n \n
	 * order of armKin:
	 * - [0] LShoulderPitch
	 * - [1] LShoulderRoll
	 * - [2] LElbowYaw
	 * - [3] LElbowRoll
	 * - [4] LWristYaw 
	 * .
	 * armKin can be calculated by ForwardKinematics
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComLArm(vector<KinematicMatrix> armKin);

	/** calculate the position of the center of mass of the right arm relative to the torso \n
	 * The joint angles stored in Blackboard are used for the calculation
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComRArm();

	/** calculate the position of the center of mass of the right arm relative to the torso
	 * @param armKin A vector containing the kinematic information of the arm \n \n
	 * order of armKin:
	 * - [0] RShoulderPitch
	 * - [1] RShoulderRoll
	 * - [2] RElbowYaw
	 * - [3] RElbowRoll
	 * - [4] RWristYaw 
	 * .
	 * armKin can be calculated by ForwardKinematics
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComRArm(vector<KinematicMatrix> armKin);

	/** calculate the position of the center of mass of the head relative to the torso \n
     * The joint angles stored in Blackboard are used for the calculation
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComHead();

	/** calculate the position of the center of mass of the head relative to the torso
	 * @param headKin A vector containing the KinematicMatrices for the head \n \n
	 * order of headKin
	 * - [0] HeadYaw
	 * - [1] HeadPitch 
	 * .
	 * headKin can be calculated by ForwardKinematics
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComHead(vector<KinematicMatrix> headKin);

	/** calculate the position of the center of mass of the body relative to the torso \n
	 * The joint angles stored in Blackboard are used for the calculation
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getComBody();

	/** calculate the total mass of the left leg
	 * @return the total mass
	 */
	static float getMassLLeg();

	/** calculate the total mass of the right leg
	 * @return the total mass
	 */
	static float getMassRLeg();

	/** calculate the total mass of the left arm
	 * @return the total mass
	 */
	static float getMassLArm();

	/** calculate the total mass of the right arm
	 * @return the total mass
	 */
	static float getMassRArm();

	/** calculate the total mass of the head
	 * @return the total mass
	 */
	static float getMassHead();

	/** calculate the total mass of the body
	 * @return the total mass
	 */
	static float getMassBody();
	
	/** calculate the total mass
	 * @param name the name of the chain \n \n
	 * name can be: 
	 * - "Head"
	 * - "LArm"
	 * - "RArm"
	 * - "LLeg"
	 * - "RLeg"
	 * - "Body"
	 * .
	 * If name does not match any of the valid names, the center of mass of the whole Body is returned
	 * @return the total mass
	 */
	static float getMass(string name);

	/** calculate the position of the center of mass of the body relative to the torso
	 * @param jointAngles the jointAngles for the body \n \n
	 * order of jointAngles
	 * - [0] HeadYaw
	 * - [1] HeadPitch
	 * - [2] LShoulderPitch
	 * - [3] LShoulderRoll
	 * - [4] LElbowYaw
	 * - [5] LElbowRoll
	 * - [6] LWristYaw
	 * - [7] LHand
	 * - [8] LHipYawPitch
	 * - [9] LHipRoll
	 * - [10] LHipPitch
	 * - [11] LKneePitch
	 * - [12] LAnklePitch
	 * - [13] LAnkleRoll
	 * - [14] RHipYawPitch (=LHipYawPitch)
	 * - [15] RHipRoll
	 * - [16] RHipPitch
	 * - [17] RKneePitch
	 * - [18] RAnklePitch
	 * - [19] RAnkleRoll
	 * - [20] RShoulderPitch
	 * - [21] RShoulderRoll
	 * - [22] RElbowYaw
	 * - [23] RElbowRoll
	 * - [24] RWristYaw
	 * - [25] RHand
	 * @return a vector containing the x,y,z position of the com
	 */
	static Vector3<float> getCom(vector<float> jointAngles);

	


};
#endif
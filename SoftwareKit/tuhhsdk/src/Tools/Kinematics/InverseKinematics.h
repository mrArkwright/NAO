#ifndef __InverseKinematics_h__
#define __InverseKinematics_h__


#define _USE_MATH_DEFINES

#include "KinematicMatrix.h"
#include <vector>
#include <limits>



using namespace std;

/// Implementation of Inverse Kinematics
/**
 * This class implements the inverse kinematics for the Nao robot.
 * It calculates the joints angles for a specified position of an endeffector.
 * All positions are relative to the torso space
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class InverseKinematics
{
public:
	/** default constructor */
	InverseKinematics(){};

	/**
	 * Calculation of the angles for the left leg for a specified position and rotation of the left foot
	 * @param desired a KinematicMatrix containing the rotation and position of the foot relative to the torso
	 * @return angles the angles for the leg
	 */
	static vector<float> getLLegAngles(const KinematicMatrix& desired);

	/**
	 * Calculation of the angles for the left leg for a specified position and rotation of the left foot
	 * @param desired a KinematicMatrix containing the rotation and position of the foot relative to the torso
	 * @return angles the angles for the leg
	 */
	static vector<float> getRLegAngles(const KinematicMatrix& desired);

	/**
	 * Calculation of the left leg angles with a given HipYawPitch joint value
	 * @param desired The desired foot position and orientation
	 * @param a_HipYawPitch The desired HipYawPitch angle
	 * @return A vector containing the angles for the leg
	 */	
	static vector<float> getFixedLLegAngles(const KinematicMatrix& desired, const float& a_HipYawPitch);

	/**
	 * Calculation of the right leg angles with a given HipYawPitch joint value
	 * @param desired The desired foot position and orientation
	 * @param a_HipYawPitch The desired HipYawPitch angle
	 * @return A vector containing the angles for the leg
	 */	
	static vector<float> getFixedRLegAngles(const KinematicMatrix& desired, const float& a_HipYawPitch);

	/**
	 * Calculation of the angles for the left arm
	 * @param desired A KinematicMatrix containing the desired Position and Orientation of the Left Hand
	 * @return The angles for the arm
	 */
	static vector<float> getLArmAngles(const KinematicMatrix& desired);

	/**
	 * Calculation of the angles for the left arm
	 * @param desired A KinematicMatrix containing the desired Position and Orientation of the Left Hand
	 * @return The angles for the arm
	 */
	static vector<float> getRArmAngles(const KinematicMatrix& desired);

private:

	/**
	* calculation of Pitch limitation curve
	* @param y the y-value for which the limit shall be calculated
	* @param k a constant depending on max ShoulderPitch angle
	* @return the x-limit of the shoulder Pitch
	*/
	static inline float getPitchlimit(const float& y, const float& k);



};
#endif

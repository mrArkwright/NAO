#pragma once

#include "Definitions/robotConstants.h"
#include <vector>
#include "Tools/Kinematics/ForwardKinematics.h"
#include "Tools/Kinematics/Com.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/KalmanFilter.h"


namespace JOINTS{
/// Enumeration for joints
	enum additionalKinematics{
		L_FOOT = JOINTS_MAX,
		R_FOOT,
		L_PELVIS,
		R_PELVIS,
		JOINTS_ADD_MAX
	};
};

/// Storage module
/**
 * This class realizes a storage of important data
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class Blackboard
{
private:
	// poses
	static vector<float> homePosition;
	static vector<float> takeAwayPosition;
	
	// sensor data
	static vector<float> jointAngles;
	static vector<float> lastCommands;
	static Vector3<float> accelerometerData;
	static Vector3<float> filteredAccelerometerData;
	static Vector2<float> gyroscopeData;
	static Vector2<float> angleData;	
	static vector<float> fsr;
	static int time;

	static Vector3<float> comBody;
	static KinematicMatrix kinMatrices[JOINTS::JOINTS_ADD_MAX];

	static KalmanFilter kalmanAngleX;
	static KalmanFilter kalmanAngleY;

	static float accelerometerFilterStrength;
	
public:
	/*-----------------*/
	/*    Functions    */
	/*-----------------*/

	/**
	 * @name Time
	 * Functions dealing with time
	 */

	//@{
	/**
	 * update the Blackboard time 
	 * @param t time
	 */
	static void updateTime(const int& t);

	/**
	* get the time 
	* @return The time
	*/
	static int getTime();	
	//@}

	
	
	/**
	 * @name IMU functions
	 * Functions for setting or reading IMU data
	 */
	//@{
	/**
	 * updates the accelerometer Data 
	 * @param data The accelerometer data
	 */
	static void updateAccelerometerData(const Vector3<float> &data);
	
	/**
	 * updates the gyroscope data 
	 * @param data The gyroscope data
	 */
	static void updateGyroscopeData(const Vector2<float> &data);
	
	/**
	 * updates angle data
	 * @param data The angle data
	 */
	static void updateAngleData(const Vector2<float> &data);

	/**
	 * Updates the estimated Angle data. Useful for Webots.
	 * Calculates estimated torso angles from filtered 
	 * accelerometer data and gyrosope data
	 */
	static void updateAngleEstimation();

	/**
	 * get the accelerometer Data 
	 * @return The accelerometerData
	 */
	static Vector3<float> getAccelerometerData();

	/**
	 * get the filtered accelerometer Data 
	 * @return The filtered accelerometerData
	 */
	static Vector3<float> getFilteredAccelerometerData();
		
	/**
	 * get the gyroscope Data 
	 * @return The gyroscope Data
	 */
	static Vector2<float> getGyroscopeData();
	
	/**
	 * get the angle Data 
	 * @return The angle data
	 */
	static Vector2<float> getAngleData();	
	//@}
	
	/**
	 * @name Joint Angles
	 * Functions dealing with joint angles
	 */
	//@{

	/**
	 * update all joint Angles 
	 * @param angles The joint angles
	 */
	static void updateJointAngles(const vector<float>& angles);
	
	/**
	 * Update all Commands which were sent to the joints
	 * @param angles The last commands
	 */
	static void updateLastCommands(const vector<float>& angles);

	/**
	 * get the last commands which were sent to the joints
	 * @param chain A string with the name of the chain \n \n
	 * chain can be:
	 * - "Head"
	 * - "LArm"
	 * - "RArm"
	 * - "LLeg"
	 * - "RLeg"
	 * - "Body"
	 * .
	 * If chain does not match one of the valid strings, the commands of the whole
	 * body are returned.
	 * @return A vector containing the last commands
	 */
	static vector<float> getLastCommands(const string& chain);	
	
	

	/** 
	 * get joint Angles of a chain 
	 * @param chainName The name of the servo chain 
	 * chainName can be:
	 * - "Head"
	 * - "LArm"
	 * - "RArm"
	 * - "LLeg"
	 * - "RLeg"
	 * - "Body"
	 * .
	 * If chainName does not match one of the valid strings, the commands of the whole
	 * body are returned.
	 * @return The angles of the chain servos
	 */
	static vector<float> getJointAngles(const string& chainName);
	//@}
	
	/**
	 * @name Sensors
	 * Functions dealing with sensor values
	 */

	//@{
	/**
	 * Update of all FSR-Sensor values.
	 * @param fsr The sensor values of the FSRs
	 */
	static void updateFsr(const vector<float>& fsr );	

	/**
	 * Get FSR values
	 * @return A vector containing FSR values
	 */
	static vector<float> getFsr();
	//@}
	

	/**
	 * @name Center of Mass
	 * Functions dealing with center of mass
	 */

	//@{
	/**
	 * Updates the center of mass of the body. Herefor the
	 * stored angle data are used
	 */
	static void updateCom();

	/**
	 * get the bodys center of mass
	 * @return The bodys center of mass
	 */
	static Vector3<float> getCom();
	//@}

	
	
	/** 
	 * @name Kinematics Information
	 * Functions for reading the Kinematic Information stored in Blackboard.
	 */
	//@{

	/**
	 * Update of all KinematicMatrices. Therefore the stored
	 * angle data are used.
	 */
	static void updateKinematicMatrices();

	/**
	 * Get the KinematicMatrix for the given index
	 * @param index The index for the KinematicMatrix
	 * corresponding to enumeration of namespace JOINTS
	 * @return The KinematicMatrix
	 */
	static KinematicMatrix getKinematicMatrix(const int& index);
	//@}
};

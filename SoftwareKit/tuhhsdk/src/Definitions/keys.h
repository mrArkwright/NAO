#pragma once

/// Enumeration of DCM keys
namespace keys{

	/*-----------------------------------------------------*/
	/*----------------------LED KEYS-----------------------*/
	/*-----------------------------------------------------*/
	/// LED keys
	namespace led
	{
		/// Enum of all chest LED keys
		enum chest{ CHEST_BLUE, CHEST_GREEN, CHEST_RED, CHEST_MAX};
		/// Chest LED keys
		extern const char* chestKey[CHEST_MAX];

		/// Enum of all ear LED keys
		enum ear{EAR_DEG_0, EAR_DEG_36, EAR_DEG_72, EAR_DEG_108, EAR_DEG_144,
						 EAR_DEG_180, EAR_DEG_216, EAR_DEG_252, EAR_DEG_288, EAR_DEG_324,
						 EAR_MAX};
		/// Left ear keys
		extern const char* earLeftKey[EAR_MAX];
		/// Right ear keys
		extern const char* earRightKey[EAR_MAX];

		// Enum of all eye LED keys
		enum eye{EYE_BLUE_DEG_0, EYE_BLUE_DEG_45, EYE_BLUE_DEG_90,
						 EYE_BLUE_DEG_135, EYE_BLUE_DEG_180, EYE_BLUE_DEG_225,
						 EYE_BLUE_DEG_270, EYE_BLUE_DEG_315, EYE_GREEN_DEG_0,
						 EYE_GREEN_DEG_45, EYE_GREEN_DEG_90, EYE_GREEN_DEG_135,
						 EYE_GREEN_DEG_180, EYE_GREEN_DEG_225, EYE_GREEN_DEG_270,
						 EYE_GREEN_DEG_315, EYE_RED_DEG_0, EYE_RED_DEG_45,
						 EYE_RED_DEG_90, EYE_RED_DEG_135, EYE_RED_DEG_180,
						 EYE_RED_DEG_225, EYE_RED_DEG_270, EYE_RED_DEG_315, EYE_MAX};

		/// Left eye keys
		extern const char* eyeLeftKey[EYE_MAX];
		/// Right eye keys
		extern const char* eyeRightKey[EYE_MAX];

		/// Enum of all head LED keys
		enum head{HEAD_FRONT_LEFT_0, HEAD_FRONT_LEFT_1, HEAD_FRONT_RIGHT_0,
							HEAD_FRONT_RIGHT_1, HEAD_MIDDLE_LEFT_0, HEAD_MIDDLE_LEFT_1,
							HEAD_REAR_LEFT_0, HEAD_REAR_LEFT_1, HEAD_REAR_LEFT_2,
							HEAD_REAR_RIGHT_0, HEAD_REAR_RIGHT_1, HEAD_REAR_RIGHT_2,
							HEAD_MAX};
		/// Head keys
		extern const char* headKey[HEAD_MAX];

		/// Enum of all foot LED keys
		enum foot{FOOT_BLUE, FOOT_GREEN, FOOT_RED, FOOT_MAX};
		/// Left foot keys
		extern const char* footLeftKey[FOOT_MAX];
		/// Right foor keys
		extern const char* footRightKey[FOOT_MAX];
	}

	/*-----------------------------------------------------*/
	/*----------------------JOINT KEYS---------------------*/
	/*-----------------------------------------------------*/
	/// Joint keys
	namespace joints{
		/// Enum of all Joints
		enum enumJoints {HEAD_YAW, HEAD_PITCH, L_SHOULDER_PITCH, L_SHOULDER_ROLL,
										 L_ELBOW_YAW, L_ELBOW_ROLL, L_WRIST_YAW, L_HAND,
										 L_HIP_YAW_PITCH, L_HIP_ROLL, L_HIP_PITCH, L_KNEE_PITCH,
										 L_ANKLE_PITCH, L_ANKLE_ROLL, R_HIP_YAW_PITCH, R_HIP_ROLL,
										 R_HIP_PITCH, R_KNEE_PITCH, R_ANKLE_PITCH, R_ANKLE_ROLL,
										 R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_ELBOW_YAW, R_ELBOW_ROLL,
										 R_WRIST_YAW, R_HAND, JOINTS_MAX};

		/// Hardness keys
		extern const char* hardnessKey[JOINTS_MAX];
		/// Actuator keys
		extern const char* actuatorKey[JOINTS_MAX];
		/// Sensor keys
		extern const char* sensorKey[JOINTS_MAX];
		/// Current keys
		extern const char* currentKey[JOINTS_MAX];
		/// Temperature keys
		extern const char* temperatureKey[JOINTS_MAX];
	}


	/*-----------------------------------------------------*/
	/*---------------------SENSOR KEYS---------------------*/
	/*-----------------------------------------------------*/

	/// Sensor keys
	namespace sensor
	{
		/// Enum of all sonar keys
		enum sonar { SONAR_ACTUATOR, SONAR_SENSOR, SONAR_LEFT_SENSOR_0, SONAR_LEFT_SENSOR_1,
								 SONAR_LEFT_SENSOR_2, SONAR_LEFT_SENSOR_3, SONAR_LEFT_SENSOR_4,
								 SONAR_LEFT_SENSOR_5, SONAR_LEFT_SENSOR_6, SONAR_LEFT_SENSOR_7,
								 SONAR_LEFT_SENSOR_8, SONAR_LEFT_SENSOR_9, SONAR_RIGHT_SENSOR_0,
								 SONAR_RIGHT_SENSOR_1, SONAR_RIGHT_SENSOR_2, SONAR_RIGHT_SENSOR_3,
								 SONAR_RIGHT_SENSOR_4, SONAR_RIGHT_SENSOR_5, SONAR_RIGHT_SENSOR_6,
								 SONAR_RIGHT_SENSOR_7, SONAR_RIGHT_SENSOR_8, SONAR_RIGHT_SENSOR_9, SONAR_MAX};
		/// Sonar keys
		extern const char* sonarKey[SONAR_MAX];

		/// Enum of all IMU keys
		enum imu {IMU_GYR_REF, IMU_GYR_X, IMU_GYR_Y, IMU_ACC_X, IMU_ACC_Y, IMU_ACC_Z,
							IMU_ANGLE_X, IMU_ANGLE_Y, IMU_MAX};
		/// IMU keys
		extern const char* imuKey[IMU_MAX];

		/// Enum of all FSR keys
		enum fsr { FSR_FRONT_LEFT, FSR_FRONT_RIGHT, FSR_REAR_LEFT, FSR_REAR_RIGHT,
							 FSR_TOTAL_WEIGHT, FSR_COP_X, FSR_COP_Y, FSR_MAX};
		/// Left foot keys
		extern const char* fsrLeftKey[FSR_MAX];
		/// Right foot keys
		extern const char* fsrRightKey[FSR_MAX];

		/// Enum of all switch keys
		enum switches{ SWITCH_HEAD_FRONT, SWITCH_HEAD_MIDDLE, SWITCH_HEAD_REAR,
									 SWITCH_L_HAND_BACK, SWITCH_L_HAND_LEFT, SWITCH_L_HAND_RIGHT,
									 SWITCH_R_HAND_BACK, SWITCH_R_HAND_LEFT, SWITCH_R_HAND_RIGHT,
									 SWITCH_CHEST_BUTTON, SWITCH_L_FOOT_LEFT, SWITCH_L_FOOT_RIGHT,
									 SWITCH_R_FOOT_LEFT, SWITCH_R_FOOT_RIGHT, SWITCH_MAX};
		/// Switch keys
		extern const char* switchKey[SWITCH_MAX];

		/// Enum of all battery keys
		enum battery{ BATTERY_TEMPERATURE, BATTERY_CURRENT, BATTERY_MAX};
		/// Battery keys
		extern const char* batteryKey[BATTERY_MAX];
	}
}// end of namespace keys

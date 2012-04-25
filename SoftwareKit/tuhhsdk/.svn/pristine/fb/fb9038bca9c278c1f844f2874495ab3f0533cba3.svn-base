/**
 * @mainpage Documentation for the tuhhSDK
 * @section sec1 Decription
 * The tuhhSDK consists of multiple classes and a lot of functions. It is developed by students of the TU Hamburg-Harburg. 
 * The SDK is provided as Open Source, so you can look into the functions, make changes or extend the SDK. 
 *
 * If something is not documented well, please contact <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */

#pragma once

#include "Modules/DcmConnector.h"
#include "Definitions/keys.h"

#ifndef WEBOTS

#include <boost/signal.hpp>
#include <boost/bind.hpp>

#endif
/// TUHH main module
/**
 * This class simplifies the use of the SDK 
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class TUHH
{
public:
	/// Initializaion of the Module
	static void init();

	/** Cycle function. Bound to DCM and called every 10ms.
	 * Updates the Blackboard.
	 */
	static void cycle();

	/// Stops the cycle.
	static void stopCycle();

private:
	// joint information
	static float* jointActuator[keys::joints::JOINTS_MAX];
	static float* jointSensor[keys::joints::JOINTS_MAX];
	static float* jointHardness[keys::joints::JOINTS_MAX];
	static float* jointCurrent[keys::joints::JOINTS_MAX];
	static float* jointTemperature[keys::joints::JOINTS_MAX];

	// sensor information
	static float* switches[keys::sensor::SWITCH_MAX];
	static float* imu[keys::sensor::IMU_MAX];
	static float* fsrLeft[keys::sensor::FSR_MAX];
	static float* fsrRight[keys::sensor::FSR_MAX];
	static float* sonar[keys::sensor::SONAR_MAX];
	static float* battery[keys::sensor::BATTERY_MAX];

#ifndef WEBOTS
	static boost::signals::connection cycleConnection;
#endif
};


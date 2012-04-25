#ifndef WEBOTSSDK_DCMENGINE_H
#define WEBOTSSDK_DCMENGINE_H

#include "Tools/Var/Command.h"
#include "Definitions/robotConstants.h"
#include "Definitions/keys.h"
#include <string>


/// DCM Simulator
/** 
 * Simulation of the DCM module
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class DcmEngine
{
public:

    /**
     * Default Constructor.
     */
    DcmEngine();

    /**
     * Destructor.
     */
    virtual ~DcmEngine() {};

    /**
    * Call this function to send a timed-command list to an actuator
    * @param commands AL::ALValue with all data
    */
    static void set(const Command& commands);

    /**
    * Call this function to send timed-command list to an alias (list of actuators)
    * @param commands AL::ALValue with all data
    */
    static void setAlias(const Command& commands);

    /**
    * Return the DCM time
    * @param offset optional time in ms (signed) to add/remove
    * @return time An integer (could be signed) with the DCM time
    */
    static int getTime (const int &offset);

    /**
    * Sets the DCM time
    * @param timeMs the time
    */
    static void setTime (const int &timeMs);

    /**
    * Create or change an alias (list of actuators)
    * @param alias Alias name and description
    * @return 0
    */
    static int createAlias (const vector<string>& alias);

    /**
    * Return the STM base name (not implemented yet)
    * @return prefix the STM base name for all device/sensors (1st string in the array) and all devices (2nd string in the array)
    */
    static void getPrefix ();

    /**
    * Special DCM commands (not implemented yet)
    * @param result one string and could be Reset, Version, Chain, Diagnostic, Config
    */
    static void special (const string& result);

    /**
    * Calibration of a joint (not implemented yet)
    */
    static void calibration ();

    /**
    * Save updated value from DCM in XML pref file (not implemented yet)
    */
    static int preferences ();

    /**
    * This function has to be called in every cycle of the simulator to get the actual joint commands
	* @return A vector containing the interpolated joint commands
    */
    static vector<float> updateCommands();

    /**
    * This function has to be called once after naoqi has started and before updateCommands is calles the firtst time
	* It is automatically called by DcmConnecter:: init()
	* @param angles The starting angles of the robot
    */
    static void initialize(const vector<float>& angles);





private:
    static vector<vector<string> >vector_aliases;
    static vector<Command> jointCommands;
    static int lastCycleTime[JOINTS::JOINTS_MAX];
    static int lastCommandTime[JOINTS::JOINTS_MAX];
    static vector<float> sendCommands;
    static double lastCommandDouble[JOINTS::JOINTS_MAX];
    static int time;
};
#endif

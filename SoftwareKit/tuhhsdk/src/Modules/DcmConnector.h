#pragma once

#include <vector>
#include <string>

#ifndef WEBOTS
	#include <alcore/alptr.h>
	#include <alproxies/dcmproxy.h>
	#include <alproxies/almemoryproxy.h>
	#include <alcommon/alproxy.h>
	#include <alcommon/albroker.h>
	#include <boost/signals/connection.hpp>
	#include <boost/signal.hpp>
	#include "Definitions/keys.h"
#endif


/// Connector to DCM module
/**
 * This class realizes the connection to the DCM module, if it is compiled for
 * the real robot
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class DcmConnector
{
public:
	/** 
	 * default constructor
	 */
	DcmConnector(){};
	#ifndef WEBOTS
		/**
		 * Initializes Proxies and creates useful aliases
		 * @param dcm The proxy to the dcm module
		 * @param parent The parent Broker
		 */
	static void init(const boost::shared_ptr<AL::DCMProxy>& dcm, const boost::shared_ptr<AL::ALBroker>& parent);
	#else
		/** 
		 * Initialization and Creation of useful aliases
		 */
		static void init();
	#endif

	/**
	 * Sending one command to a single actuator
	 * @param name The name of a single joint or the name of an alias
	 * @param update The kind of update \n \n
	 * can be: 
	 * - "ClearAll"
	 * - "ClearAfter
	 * .
	 * @param time The time, when the command shall reach the target value
	 * @param commands The command
	 */
	static void sendCommands(const std::string& name, const std::string &update, const int& time, const float commands);

        /**
         * Sending one command to a single actuator
         * @param name The name of a single joint or the name of an alias
         * @param update The kind of update \n \n
         * can be:
         * - "ClearAll"
         * - "ClearAfter
         * .
         * @param time The time, when the command shall reach the target value
         * @param commands The commands for the joints
         */
        static void sendCommands(const std::string& name, const std::string &update, const int& time, const std::vector<float>& commands);

	/**
	 * Sending a list of commands to a single actuator
	 * @param name The name of a single joint or the name of an alias
	 * @param update The kind of update \n \n
	 * can be: 
	 * - "ClearAll"
	 * - "ClearAfter
	 * .
	 * @param time The time vector, when the commands shall reach the target values
	 * @param commands The commands
	 */
	static void sendCommands(const std::string& name, const std::string &update, std::vector<int>& time, const std::vector<float> commands);

	/**
	 * Sending a list of commands to a list of actuator (command times are equal)
	 * @param name The name of a single joint or the name of an alias
	 * @param update The kind of update \n \n
	 * can be: 
	 * - "ClearAll"
	 * - "ClearAfter
	 * .
	 * @param time The time vector, when the commands shall reach the target values
	 * @param commands The commands
	 */
	static void sendCommands(const std::string& name, const std::string &update, std::vector<int>& time, const std::vector<std::vector<float> >commands);

	/**
	 * Sending a list of commands to a list of actuator (command times are different)
	 * @param name The name of a single joint or the name of an alias
	 * @param update The kind of update \n \n
	 * can be: 
	 * - "ClearAll"
	 * - "ClearAfter
	 * .
	 * @param time The time vector, when the commands shall reach the target values
	 * @param commands The commands
	 */
	static void sendCommands(const std::string& name, const std::string &update, std::vector<std::vector<int> > time, const std::vector<std::vector<float> > commands);
	
	/**
	 * Creation of an alias
	 * @param alias A vector containing as first element the name of the alias. The following elements are a list of
	 * devices which shall be part of the alias
	 */
	static void createAlias(const std::vector<std::string>& alias);

	/**
	 * Logging messages 
	 * @param message The message you want to log
	 */
	static void log(const std::string& message);

	/**
	 * Logging messages and numeric values 
	 * @param message The message you want to log
	 * @param value The value you want to log
	 */
	static void log(const std::string& message, const float& value);

	/**
	 * Sends a vector of joint angles for a predefined pose
	 * @param poseIndex The index of the pose you want to use
	 * @param time The time, when the robot shall reach the pose
	 */
	static void pose( const int& poseIndex, const int &time);

#ifndef WEBOTS
	/**
	 * Get the pointer to data in ALMemory
	 * @param key The key for the variable from which the pointer
	 * shall be returned.
	 * @return The pointer
	 */
	static float* getDataPtr(const char* key);

	/**
	 * Get the DCM time.
	 * @return The time in ms
	 */
	static int getTime();

	/**
	 * Bind a method to the signal which is sent just before DCM will run
	 * @param subscriber The method which shall be bound
	 * @return The connection
	 */
	static boost::signals::connection bindPre(
			const boost::signal<void ()>::slot_function_type& subscriber);

	/**
	 * Bind a method to the signal which is sent right after DCM ran
	 * @param subscriber The method which shall be bound
	 * @return The connection
	 */
	static boost::signals::connection bindPost(
			const boost::signal<void ()>::slot_function_type& subscriber);
#endif



private:
#ifndef WEBOTS
	// connection to AL Modules
	static boost::shared_ptr<AL::DCMProxy> dcmProxy;
	static boost::shared_ptr<AL::ALBroker> parentBroker;
	static boost::shared_ptr<AL::ALMemoryProxy> memProxy;
#endif
};


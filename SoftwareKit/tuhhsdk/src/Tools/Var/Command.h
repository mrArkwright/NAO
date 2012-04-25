#ifndef WEBOTSSDK_COMMAND_H
#define WEBOTSSDK_COMMAND_H

#include <string>
#include <vector>

using namespace std;

///structure for saving commands
/**
 * A Command can represent a single command to a single joint, 
 * as well as a list of commands to a single joint, 
 * as well as a list of commands to a list of joints.
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class Command
{
public:

	/** 
	 * default constructor
	 */
	Command(){};

	/**
	 * clear contents stored in Command
	 */
	void clear()
	{
		jointName = "";
		update = "";
		aliasMode ="";
		time = vector<int>();
		commands = vector<float>();
		timeAlias = vector<vector<int> >();
		commandsAlias = vector<vector<float> >();
	}

	/**
	 * name of the joint or name of an alias \n
	 * a alias can also represent a chain of joints
	 */
	string jointName;

	/** 
	 * kine of update \n \n
	 * can be:
	 * - "ClearAfter"
	 * - "ClearAll"
	 */
	string update;

	/** 
	 * mode of alias \n \n
	 * can be:
	 * - "time-mixed"
	 * - "time-separate"
	 */
	string aliasMode;

	/**
	 * time when command shall reach the final value \n
	 * time is a vector for a list of commands
	 */
	vector<int> time;

	/**
	 * joint angles which shall be reached \n
	 * commands is a vector for a list of commands
	 */
	vector<float> commands;

	/**
	 * For sending commands to more than one joint, each joint has a time vector.
	 * The time vectors for each joint are also stored in a vector
	 */
	vector<vector<int> > timeAlias;

	/**
	 * For sending commands to more than one joint, each joint has a command vector.
	 * The command vectors for each joint are also stored in a vector
	 */
	vector<vector<float> > commandsAlias;

};

#endif

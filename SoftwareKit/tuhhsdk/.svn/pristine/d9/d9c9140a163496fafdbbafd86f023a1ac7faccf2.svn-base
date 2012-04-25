#pragma once

#include <string>

using namespace std;

/// Module to play motion files
/**
 * This class implements a Motion Player for Motion files
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 * A Motion File is a simple text file with Motion Information. You can add comments
 * in a Motion file by "//". A Motion file has four commands: \n \n
 * - NAME:
 * - UPDATE:
 * - TIMES:
 * - COMMANDS:
 * .
 * \n
 * Each command ends with an ";". There can only be one command per line.
 * An example of an Motion file can be: \n \n
 * \htmlonly
 * <pre>
 * <font color="blue">NAME:</font> <font color="orchid">       "Head"</font>; 
 * <font color="blue">UPDATE:</font>      <font color="orchid">"ClearAll"</font>; 
 * <font color="blue">TIMES:</font>       1000    2000    3000    4000; 
 * <font color="blue">COMMANDS:</font>    0.5     0.8     0.0     -0.5; <font color="green">// HeadYaw </font> 
 *              0.2     0.5     -0.1    -0.5; <font color="green">// HeadPitch</font> 
 * </pre>
 * \endhtmlonly
 */
class MotionPlayer
{
	/** 
	 * default constructor
	 */
	MotionPlayer(){};

public:
	/** 
	 * Function to play a motion File
	 * @param filePath The path to the motion file
	 */
	static int playMotionFile(string filePath);

private:
	static int isCommand(string word);
	static bool isComment(string word);
	enum {NO_COMMAND, NAME, UPDATE, TIME, COMMAND};

};


#include "../Tools/Storage/Blackboard.h"
#include "DcmConnector.h"
#include "MotionPlayer.h"
#include <fstream>
#include <limits>
#include <stdlib.h>

int MotionPlayer::playMotionFile(std::string filePath)
{
	// open file
	ifstream file;
	file.open(filePath.c_str()); 
	
	// if found
	if(file)
	{
		// actual read word
		string word; 
		
		// actual command number
		int cNumber;

		// actual command
		int activeData;

		bool endOfLine;
		
		int commandJoint = 0;
		int commandNumber = 0;
		string aliasName;
		string update;
		vector<int> times;
		vector<vector<float> > commands;

		// scan file
		while(!file.eof())
		{
			file >> word;
	
			// if word is no comment
			if (!isComment(word))
			{
				// read command number
				cNumber = isCommand(word);

				// if word is a command
				if ( cNumber > 0)
					activeData = cNumber;
				else{
					// if word ends with ";" then cut off and set endOfLine
					if (word[word.size()-1] == ';')
					{
						word = word.substr(0,word.size()-1);
						endOfLine = true;
					}
					else 
						endOfLine = false;

					// check active command
					switch(activeData)
					{
						case NAME:
							// store Name (cut off " ")
							if (word.size()>0)
								aliasName = word.substr(1,word.size()-2);
							break;
						case UPDATE:
							// store update (cut off " ")
							if(word.size()>0)
								update = word.substr(1,word.size()-2);
							break;
						case TIME:
							// store times in vector
							if (word.size()>0)
								times.push_back(atoi(word.c_str()));
							break;
						case COMMAND:
							// store commands
							if (word.size()>0)
							{
								if (commandJoint == 0)
									commands.push_back(vector<float>());
								commands[commandNumber].push_back((float) atof(word.c_str()));
								commandNumber++;								
							}
							// reset commandnumber and count commandJoint
							if (endOfLine)
							{
								commandNumber = 0;
								commandJoint++;
							}
							break;
					}
				}
			}
			// if word is comment then ignore rest of line
			else
			{
				file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			}
		}
		// close file
		file.close();

		// send commands to actuators
		DcmConnector::sendCommands(aliasName, update, times, commands);
		return times.back();
	}
	else
		return -1;
}

int MotionPlayer::isCommand(string word)
{	
	if (word == "NAME:")
		return NAME;
	else if (word == "UPDATE:")
		return UPDATE;
	else if (word == "TIMES:")
		return TIME;
	else if (word == "COMMANDS:")
		return COMMAND;
	else
		return NO_COMMAND;
}

bool MotionPlayer::isComment(string word)
{
	if ( word.size() >=2)
	{
		if ( word[0] =='/' && word[1] =='/' )
			return true;
		else
			return false;
	}
	else
		return false;
}



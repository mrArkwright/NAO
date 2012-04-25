/**
 * @author Stefan Kaufmann
 */

#include "DcmEngine.h"
#include "../Tools/Storage/Blackboard.h"

vector<vector<string> > DcmEngine::vector_aliases = vector<vector<string> >();
vector<Command>			DcmEngine::jointCommands  = vector<Command>(JOINTS::JOINTS_MAX);
vector<float>			DcmEngine::sendCommands   = vector<float>(JOINTS::JOINTS_MAX);

int	DcmEngine::lastCycleTime[JOINTS::JOINTS_MAX];
int DcmEngine::lastCommandTime[JOINTS::JOINTS_MAX];
int DcmEngine::time = 0;

double DcmEngine::lastCommandDouble[JOINTS::JOINTS_MAX];


DcmEngine::DcmEngine()
{
	vector<Command> tmp(26, Command());
	jointCommands = tmp;
	sendCommands = vector<float>(26,0.0f);
	for (int i = 0; i < JOINTS::JOINTS_MAX; i++)
	{
		lastCycleTime[i] = 0;
		lastCommandTime[i] = 0;
		lastCommandDouble[i] = 0.0;
	}


	initialize(sendCommands);
}



void DcmEngine::set(const Command &commands)
{
	// set commands to the selected joint
	for (int i = 0; i < JOINTS::JOINTS_MAX; i++)
	{
		
		// get the joint
		if ( commands.jointName == keys::joints::actuatorKey[i])
		{
			
			// update Merge
			if ( commands.update == "Merge")
			{
				// not implemented yet
				// throw(AL::ALError("DCMBridge", "set", "Merge is not implemented in DCMBridge"));
			}
			
			// update ClearAll
			else if (commands.update == "ClearAll" || 
				(commands.update == "ClearAfter" && (int) jointCommands.at(i).time.size() == 0) )
			{
				jointCommands.at(i).time = commands.time;
				jointCommands.at(i).commands = commands.commands;
			}
			
			// update ClearAfter
			else if (commands.update == "ClearAfter")
			{
				// check the previous commands of the selected Servo
				for (int j = 0; j < (int) jointCommands.at(i).time.size(); j++)
				{
					// if the command new command acts later than a previous command
					if ( commands.time.at(0) < (int) jointCommands.at(i).time.at(j))
					{
						// delete all commands acting later than new command
						while( (int) jointCommands.at(i).time.size() > j )
						{
							jointCommands.at(i).time.pop_back();
							jointCommands.at(i).commands.pop_back();
						}
						break;
					}
				}
				// add new commands to command vector
				jointCommands.at(i).time.insert(jointCommands.at(i).time.end(), 
					commands.time.begin(), commands.time.end());
				jointCommands.at(i).commands.insert(jointCommands.at(i).commands.end(),
					commands.commands.begin(), commands.commands.end());
						
					
				
			}
			else if (commands.update == "ClearBefore")
			{
				//throw(AL::ALError("DCMBridge", "set", "ClearBefore is not implemented in DCMBridge"));
			}
			else
			{
				//throw(AL::ALError("DCMBridge", "set", "invalid kind of update"));
			}
			break;
		}
		
	}
}

void DcmEngine::setAlias(const Command &commands)
{
	
	std::vector<std::string> alias;
	Command tmpCommands;
	bool aliasFound = false;
		
	// getAlias from list
	for (int i = 0; i <(int) vector_aliases.size(); i++)
	{
		if ( commands.jointName == vector_aliases.at(i).at(0) )
		{
			alias = vector_aliases.at(i);
			aliasFound = true;
			break;
		}
	}

	// if alias was in list
	if ( aliasFound )
	{
		// check mode (time-mixed or time-seperate)
		if ( commands.aliasMode == "time-mixed" )
		{
			// convertion to normal "set" commands
			for ( int j = 1; j < (int) alias.size(); j++)
			{
				tmpCommands.clear();
				tmpCommands.jointName = alias.at(j);
				tmpCommands.update = commands.update;
				tmpCommands.time = commands.timeAlias.at(j-1);
				tmpCommands.commands = commands.commandsAlias.at(j-1);

				set(tmpCommands);

			}
		}	

		else if ( commands.aliasMode == "time-separate" )
		{
			// convertion to normal "set" commands
			for ( int j = 1; j < (int) alias.size(); j++)
			{
				tmpCommands.clear();
				tmpCommands.jointName = alias.at(j);
				tmpCommands.update = commands.update;
				tmpCommands.time = commands.time;
				tmpCommands.commands = commands.commandsAlias.at(j-1);

				set(tmpCommands);
			}
		}
		//throw(AL::ALError("DCMBridge", "setAlias", "invalid time mode"));
	}
}

int DcmEngine::getTime(const int &offset)
{
	return time + offset;
}

void DcmEngine::setTime(const int &timeMs)
{
	time = timeMs;
}

int DcmEngine::createAlias(const std::vector<std::string> &alias)
{
	bool inserted = false;
	// check if alias already exists
	for (int i = 0; i < (int) vector_aliases.size(); i++)
	{
		// replace alias 
		if (vector_aliases.at(i).at(0) == alias.at(0))
		{
			vector_aliases.at(i) = alias;
			inserted = true;
		}
	}

	// else, add new alias 
	if(!inserted)
		vector_aliases.push_back(alias);

	return 0;			
}

void DcmEngine::getPrefix()
{
}

void DcmEngine::special(const string &result)
{

}

void DcmEngine::calibration()
{

}

int DcmEngine::preferences()
{
	return 0;
}

vector<float> DcmEngine::updateCommands()
{
	int actualTime = getTime(0);
	// update commands for all servos
	for ( int i = 0; i < JOINTS::JOINTS_MAX; i++)
	{
		// if there are no commands, set times to current time
		if (jointCommands.at(i).time.size() == 0)
		{
			lastCycleTime[i] = actualTime;
		}
		// if there are stored commands
		else
		{
			bool lastCommand = false;
			// temporary variables
			int tmpCommandTime = (int) jointCommands.at(i).time.at(0);
			int varLastTime;
			double varLastCommand;
			
			// if next command time > current time 
			if (tmpCommandTime >= actualTime )
			{
				varLastTime = lastCycleTime[i];
				varLastCommand = lastCommandDouble[i];
			}

			// if commandTime < current Time 
			if (tmpCommandTime < actualTime )
			{
				varLastTime = tmpCommandTime;				
				if ((int) jointCommands.at(i).time.size() > 1)
					tmpCommandTime = (int) jointCommands.at(i).time.at(1);
				else 
					lastCommand =true;
				varLastCommand = (float) jointCommands.at(i).commands.at(0);
				
			}
			
			int difftime1 = actualTime - varLastTime;
			int difftime2 = tmpCommandTime - varLastTime;

			
			if (lastCommand)
			{
				lastCommandDouble[i] = (float) jointCommands.at(i).commands.at(0);				
			}
			else
			{
				lastCommandDouble[i] = (( (float) jointCommands.at(i).commands.at(0) - varLastCommand)*difftime1)/difftime2 + varLastCommand;				
			}
			sendCommands.at(i) = (float) lastCommandDouble[i];
			lastCycleTime[i] = actualTime;

                        if ( actualTime >= (int) jointCommands.at(i).time.at(0))
			{
				if ( (int) jointCommands.at(i).time.size() > 1)
				{
					jointCommands.at(i).time.erase(jointCommands.at(i).time.begin());
					jointCommands.at(i).commands.erase(jointCommands.at(i).commands.begin());
				}
			    else
				{
					jointCommands.at(i).clear();
				}
							
			}
		}
	}
	Blackboard::updateLastCommands(sendCommands);
	return sendCommands;
}

void DcmEngine::initialize(const vector<float>& angles)
{
	for (int i = 0; i < JOINTS::JOINTS_MAX; i++)
	{
		lastCommandDouble[i] = angles.at(i);
		sendCommands.at(i) = (float) lastCommandDouble[i];
	}	
}


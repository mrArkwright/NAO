#ifndef __DcmConverter_h__
#define __DcmConverter_h__

#include <vector>
#include <alvalue/alvalue.h>
#include "Storage/Blackboard.h"

using namespace std;

/// Converter for Commands
/**
 * Convertation from command vectors to ALValue commands
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class DcmConverter
{
public:
	DcmConverter(){};

	/**
	 * convert commands to array of ALValue
	 * for usage with "set" command of dcm module with single joint commands
	 * @param name The joint name
	 * @param update The kind of update ("Merge", "ClearAll", "ClearAfter", "ClearBefore")
	 * @param time Time when the commands shall be applied
	 * @param commands Command for the joint
	 * @return The command array
	 */
	static AL::ALValue convert(const string& name, const string &update, const int& time, const float commands)
	{
		AL::ALValue cmdArray;
		cmdArray.arraySetSize(3);
		cmdArray[0] = name;
		cmdArray[1] = update;
		
		cmdArray[2].arraySetSize(1);
		cmdArray[2][0].arraySetSize(2);
		cmdArray[2][0][0] = commands;
		cmdArray[2][0][1] = time + Blackboard::getTime();

		return cmdArray;
	}


	
	/**
	 * convert commands to array of ALValue
	 * for usage with "set" command of dcm module with multiple joint commands
	 * @param name The joint name
	 * @param update The kind of update ("Merge", "ClearAll", "ClearAfter", "ClearBefore")
	 * @param time Time vector when the commands shall be applied
	 * @param commands Command Vector for the joint
	 * @return The command array
	 */
	static AL::ALValue convert(const string& name, const string &update, const vector<int>& time, const vector<float> commands)
	{
		AL::ALValue cmdArray;
		cmdArray.arraySetSize(3);
		cmdArray[0] = name;
		cmdArray[1] = update;

		cmdArray[2].arraySetSize((int) time.size());

		for (int i = 0; i < (int) time.size(); i++)
		{
			cmdArray[2][i].arraySetSize(2);
			cmdArray[2][i][0] = commands.at(i);
			cmdArray[2][i][1] = time.at(i) + Blackboard::getTime();
		}

		return cmdArray;
	}
	
	/**
	 * convert commands to array of ALValue
	 * for usage with "setAlias" command of dcm module with multiple joint commands and "time-separate"-mode
	 * @param name The joint name
	 * @param update The kind of update ("Merge", "ClearAll", "ClearAfter", "ClearBefore")
	 * @param time Time vector when the commands shall be applied
	 * @param commands Vector of command Vectors for the joints
	 * @return The command array
	 */
	static AL::ALValue convert(const string& name, const string &update, const vector<int>& time, const vector<vector<float> >commands)
	{
		AL::ALValue cmdArray;

		cmdArray.arraySetSize(6);
		cmdArray[0] = name;
		cmdArray[1] = update;
		cmdArray[2] = "time-separate";
		cmdArray[3] = 0;
		
		cmdArray[4].arraySetSize((int) time.size());
		for ( int i = 0; i < (int) time.size(); i++)
		{
			cmdArray[4][i] = time.at(i) + Blackboard::getTime();
		}

		cmdArray[5].arraySetSize( (int) commands.at(0).size());
		for (int i = 0; i < (int) commands.at(0).size(); i++)
		{
			cmdArray[5][i].arraySetSize( (int) commands.size() );
			for (int k = 0; k < (int) commands.size(); k++)
			{
				cmdArray[5][i][k] = commands.at(k).at(i);
			}
		}
	    
		return cmdArray;

	}
	
	/**
	 * convert commands to array of ALValue
	 * for usage with "setAlias" command of dcm module with multiple joint commands and "time-mixed"-mode
	 * @param name The joint name
	 * @param update The kind of update ("Merge", "ClearAll", "ClearAfter", "ClearBefore")
	 * @param time vector of time vectors when the commands shall be applied
	 * @param commands Vector of command Vectors for the joints
	 * @return The command array
	 */	
	static AL::ALValue convert(const string& name, const string &update, const vector<vector<int> > time, const vector<vector<float> > commands)
	{
		AL::ALValue cmdArray;

		cmdArray.arraySetSize(4);
		cmdArray[0] = name;
		cmdArray[1] = update;
		cmdArray[2] = "time-mixed";
		
		cmdArray[3].arraySetSize( (int) time.at(0).size() );

		for (int i = 0; i < (int) time.at(0).size() ; i++)
		{
			cmdArray[3][i].arraySetSize( (int) time.size() );
			for (int k = 0; k < (int) time.size(); k++)
			{
				cmdArray[3][i][k].arraySetSize(2);
				cmdArray[3][i][k][0] = commands.at(k).at(i);
				cmdArray[3][i][k][1] = time.at(k).at(i) + Blackboard::getTime();
			}

		}

		return cmdArray;


	}

	/**
	 * converts an alias in vector form to an ALValue alias
	 * @param alias The alias in string vector form
	 * @return The alias in ALValue form
	 */	
	static AL::ALValue convertAlias( const vector<string> &alias)
	{
		AL::ALValue aliasArray;
		aliasArray.arraySetSize(2);
		aliasArray[0] = alias.at(0);
		aliasArray[1].arraySetSize((int) alias.size() -1);

		for (int i = 0; i < (int) alias.size()-1; i++)
			aliasArray[1][i] = alias.at(i+1);

		return aliasArray;

	}




};
#endif

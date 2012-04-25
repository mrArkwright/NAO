/*
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */

#include "DcmConnector.h"

#ifndef WEBOTS	
	#include <alvalue/alvalue.h>
	#include "Tools/DcmConverter.h"
	#include <qi/log.hpp>
#else
	#include "DcmEngine.h"
	#include "Tools/Var/Command.h"
#endif

#include "Tools/Storage/Blackboard.h"
#include "Modules/Poses.h"
#include <sstream>

#ifndef WEBOTS
// initialization of static variables
boost::shared_ptr<AL::DCMProxy> DcmConnector::dcmProxy =
		boost::shared_ptr<AL::DCMProxy>();
boost::shared_ptr<AL::ALMemoryProxy> DcmConnector::memProxy =
		boost::shared_ptr<AL::ALMemoryProxy>();
boost::shared_ptr<AL::ALBroker> DcmConnector::parentBroker =
		boost::shared_ptr<AL::ALBroker>(new AL::ALBroker);


void DcmConnector::init(const boost::shared_ptr<AL::DCMProxy> &dcm, const boost::shared_ptr<AL::ALBroker>& parent)
{
	// initialize dcmProxy
	DcmConnector::dcmProxy = dcm;
	
	// initialize parent Broker
	parentBroker = parent;
	
	// create memory Proxy
	memProxy = boost::shared_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(parentBroker));
}


#else
void DcmConnector::init()
{
	DcmEngine::initialize(vector<float>(26));
}
#endif

void DcmConnector::sendCommands(const std::string &name, const std::string &update, const int &time, const float commands)
{
	#ifndef WEBOTS
		AL::ALValue cmd = DcmConverter::convert(name, update, time, commands);
		dcmProxy->set(cmd);
	#else
		Command cmd;
		cmd.jointName = name;
		cmd.update = update;
		vector<int> timeVec;
		timeVec.push_back(time + Blackboard::getTime());
		cmd.time = timeVec;
		vector<float> commandsVec;
		commandsVec.push_back(commands);
		cmd.commands = commandsVec;
		DcmEngine::set(cmd);
	#endif
}

void DcmConnector::sendCommands(const std::string &name, const std::string &update, std::vector<int> &time, const std::vector<float> commands)
{
	#ifndef WEBOTS
		AL::ALValue cmd = DcmConverter::convert(name, update, time, commands);
		dcmProxy->set(cmd);
	#else
		for (int i = 0; i < (int) time.size(); i++)
			time[i] += Blackboard::getTime();
		Command cmd;
		cmd.jointName = name;
		cmd.update = update;
		cmd.time = time;
		cmd.commands = commands;
		DcmEngine::set(cmd);
	#endif
}

void DcmConnector::sendCommands(const std::string &name, const std::string &update, const int &time, const std::vector<float> &commands)
{
    std::vector<int> timeVec;
    timeVec.push_back(time);
    std::vector<std::vector<float> > cmdVec;
    cmdVec.push_back(commands);
    sendCommands(name, update, timeVec, cmdVec);
}

void DcmConnector::sendCommands(const std::string &name, const std::string &update, std::vector<int> &time, const std::vector<std::vector<float> > commands)
{
	#ifndef WEBOTS
		AL::ALValue cmd = DcmConverter::convert(name, update, time, commands);
		dcmProxy->setAlias(cmd);
	#else
		for (int i = 0; i < (int) time.size(); i++)
			time[i] += Blackboard::getTime();
		Command cmd;
		cmd.jointName = name;
		cmd.update = update;
		cmd.aliasMode = "time-separate";
		cmd.time = time;		
		vector<vector<float> > tmpCmds((int) commands[0].size() );
		for (int i = 0; i < (int) commands[0].size(); i++)
		{
			for(int j = 0; j < (int) commands.size(); j++)
			{
				tmpCmds[i].push_back(commands[j][i]);
			}
		}
		cmd.commandsAlias = tmpCmds;
		DcmEngine::setAlias(cmd);
	#endif
}

void DcmConnector::sendCommands(const std::string &name, const std::string &update, const std::vector<std::vector<int> > time, const std::vector<std::vector<float> > commands)
{
	#ifndef WEBOTS
		AL::ALValue cmd = DcmConverter::convert(name, update, time, commands);
		dcmProxy->setAlias(cmd);
	#else
		Command cmd;
		cmd.jointName = name;
		cmd.update = update;
		cmd.aliasMode = "time-mixed";
		vector<vector<int> > tmpTime;
		vector<vector<float> > tmpCmds((int) commands[0].size() );
		for (int i = 0; i < (int) commands[0].size(); i++)
		{
			for(int j = 0; j < (int) commands.size(); j++)
			{
				tmpCmds[i].push_back(commands[j][i]);
				tmpTime[i].push_back(time[j][i] + Blackboard::getTime());
			}
		}
		cmd.timeAlias = tmpTime;
		cmd.commandsAlias = tmpCmds;
		DcmEngine::setAlias(cmd);
	#endif
}

void DcmConnector::createAlias(const std::vector<std::string>& alias)
{
	#ifndef WEBOTS
		AL::ALValue al = DcmConverter::convertAlias(alias);
		dcmProxy->createAlias(al);
	#else
		DcmEngine::createAlias(alias);
	#endif
}

void DcmConnector::log(const std::string& message)
{
	#ifndef WEBOTS
		qiLogInfo(message.c_str());
	#endif
}

void DcmConnector::log(const std::string& message, const float& value)
{
	#ifndef WEBOTS
		std::ostringstream s;
		s << message + " ";
		s << value;
			qiLogInfo(s.str().c_str());
	#endif
}

void DcmConnector::pose(const int& poseIndex, const int& time)
{
	vector<int> time2;
	time2.push_back(time);
	vector<float> cmd = Poses::getPose(poseIndex);
	vector<vector<float> > cmdVec;
	cmdVec.push_back(cmd);
	sendCommands("Body","ClearAll", time2, cmdVec);
}

#ifndef WEBOTS

float* DcmConnector::getDataPtr(const char *key)
{
	return (float*) memProxy->getDataPtr(key);
}

int DcmConnector::getTime()
{
	return dcmProxy->getTime(0);
}

boost::signals::connection DcmConnector::bindPre(
		const boost::signal<void ()>::slot_function_type& subscriber)
{
	return parentBroker->getProxy("DCM")->getModule()->atPreProcess(subscriber);
}

boost::signals::connection DcmConnector::bindPost(
		const boost::signal<void ()>::slot_function_type& subscriber)
{
	return parentBroker->getProxy("DCM")->getModule()->atPostProcess(subscriber);
}

#endif




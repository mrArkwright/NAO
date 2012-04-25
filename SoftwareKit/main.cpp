/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <boost/shared_ptr.hpp>
#include <signal.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

/* --------------------------- */
/* ADD INCLUDE TO YOUR PROJECT */
/* --------------------------- */

class Project : public AL::ALModule
{
public:
	Project(boost::shared_ptr<AL::ALBroker> broker, const std::string &name):
	  AL::ALModule(broker, name)
	  {
		// example bind:
		/*	
		functionName("foo", getName(), "do some stuff");
		addParam("param1", "A float");
		addParam("param2", "A int");
		BIND_METHOD(Project::foo);
		*/
	  }

	  // example function
	  /*void foo(const float& param1, const int& param2){}*/
};


#ifdef _WIN32
# define ALCALL __declspec(dllexport)
#else
# define ALCALL
#endif

extern "C"
{
  ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
  {
    // init broker with the main broker instance
    // from the parent executable
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);

    // create module instances
	AL::ALModule::createModule<Project>(pBroker,"Sample" );
    return 0;
  }

  ALCALL int _closeModule()
  {
    return 0;
  }

} // extern "C"

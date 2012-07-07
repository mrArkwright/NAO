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
#include <sample.h>

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
		functionName("controllerFunction", getName(), "Kick the ball, yeah!");
		// is it correct to give the param as string?!
		addParam("kickAngle", "5");
		BIND_METHOD(Project::controllerFunction);)
	  }

	  // example function
	  /*void foo(const float& param1, const int& param2){}*/
	  void controllerFunction(int kickAngle) {
		
		if (kickAngle >= -7 && kickAngle <= 14) {
			
			// expecting the DCMProxy::getTime returns time in ms
			int start_time, current_time;
			start_time = current_time = DCMProxy::getTime();	  
		  	counter = 0;
		  	diff = current_time - start_time;
		  	
		  	while (diff < 3100) {
		  		
		  		if(diff >= 1000 && counter == 0) {
		  			
		  			// statBalance
		  			counter++;
		  			Sample::statBalance(true, 500);
		  			
		  		} else if (diff >= 1500 && counter == 1){
		  			
		  			//raise Foot
		  			counter++;
		  			float deltaX;
		  			
		  			if (kickAngle <= 0) {
						deltaX = -kickAngle/(-7.63195750157995)*25;		  					
		  			} else {
						deltaX = kickAngle/(14.2093727765404)*25;			  			
		  			}
		  			
		  			Sample::moveFoot(0, deltaX, 30, 500);
		  			
		  		} else if (diff >= 2000 && counter == 2) {
		  			
		  			//move foot back
					counter++;  		
					Sample::moveFoot(-40, 0, 0, 500);
					
		  		} else if (diff >= 3000 && counter == 3) {
		  			
		  			// kick
		  			counter++;
		  			Sample::moveFoot(200, 0, 0, 100);
		  			
		  		} 
		  	
		  	}
		  	
	  	}
	  	
	  }
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

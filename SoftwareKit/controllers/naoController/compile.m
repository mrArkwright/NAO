

disp('compiling C_call.cpp ...');
% C Call

    % Debug
    mex -g -DWEBOTS -I../../tuhhsdk/src/ ...
        C_call.cpp...
        ../../tuhhsdk/src/Modules/DcmEngine.cpp ...
        ../../tuhhsdk/src/Tools/Kinematics/InverseKinematics.cpp ...
        ../../tuhhsdk/src/Tools/Storage/Blackboard.cpp ...
        ../../tuhhsdk/src/Tools/Kinematics/Com.cpp ...
        ../../tuhhsdk/src/Tools/Math/KalmanFilter.cpp ...
        ../../tuhhsdk/src/Tools/Kinematics/ForwardKinematics.cpp...
        ../../tuhhsdk/src/Modules/DcmConnector.cpp ...
        ../../tuhhsdk/src/Definitions/keys.cpp ...
        ../../tuhhsdk/src/Definitions/robotConstants.cpp ...
        ../../tuhhsdk/src/Modules/Poses.cpp ...
        ../../tuhhsdk/src/Modules/Alias.cpp ...
        ../../tuhhsdk/src/tuhh.cpp ...
		../../src/sample.cpp                   
        

disp('Finished');


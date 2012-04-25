
TO_RAD = pi/180;

HOME = [    0;  % HeadYaw
            0;  % HeadPitch
           90;  % LShoulderPitch
        20.05;  % LShoulderRoll
          -90;  % LElbowYaw
         -0.5;  % LElbowRoll
            0;  % LWristYaw
            0;  % LHand
            0;  % LHipYawPitch
            0;  % LHipRoll
        -20.8;  % LHipPitch
        41.02;  % LKneePitch
       -20.22;  % LAnklePitch
            0;  % LAnkleRoll
            0;  % RHipYawPitch
            0;  % RHipRoll
        -20.8;  % RHipPitch
        41.02;  % RKneePitch
       -20.22;  % RAnklePitch
            0;  % RAnkleRoll
           90;  % RShoulderPitch
       -20.05;  % RShoulderRoll
           90;  % RElbowYaw
          0.5;  % RElbowRoll
            0;  % RWristYaw
            0   % RHand
] * TO_RAD;

RARM_FRONT = [  50; % ShoulderPitch
                 0; % ShoulderRoll
                20; % ElbowYaw
                40; % ElbowRoll
                 0  % WristYaw
] * TO_RAD;

LARM_FRONT = [  50; % ShoulderPitch
                 0; % ShoulderRoll
               -20; % ElbowYaw
               -40; % ElbowRoll
                 0  % WristYaw
] * TO_RAD;

RARM_BACK = [  119; % ShoulderPitch
               -26; % ShoulderRoll
                82; % ElbowYaw
                30; % ElbowRoll
                 0  % WristYaw
] * TO_RAD;

LARM_BACK = [  119; % ShoulderPitch
                26; % ShoulderRoll
               -82; % ElbowYaw
               -30; % ElbowRoll
                 0  % WristYaw
] * TO_RAD;
            
    


HOME_STRUCT.aliasName = 'JointActuatorBody';
HOME_STRUCT.update = 'ClearAll';
HOME_STRUCT.aliasMode = 'time-separate';
HOME_STRUCT.timeAlias = 1000;
HOME_STRUCT.commandsAlias = HOME';

RFOOT_DUMMY.aliasName = 'RLeg';
RFOOT_DUMMY.update = 'ClearAfter';
RFOOT_DUMMY.aliasMode = 'time-separate';

LFOOT_DUMMY= RFOOT_DUMMY;
LFOOT_DUMMY.aliasName = 'LLeg';

RARM_FRONT_STRUCT.aliasName = 'RArm';
RARM_FRONT_STRUCT.update = 'ClearAfter';
RARM_FRONT_STRUCT.aliasMode = 'time-separate';
RARM_FRONT_STRUCT.timeAlias = 0;
RARM_FRONT_STRUCT.commandsAlias = RARM_FRONT';

LARM_FRONT_STRUCT = RARM_FRONT_STRUCT;
LARM_FRONT_STRUCT.aliasName = 'LArm';
LARM_FRONT_STRUCT.commandsAlias = LARM_FRONT';

RARM_BACK_STRUCT = RARM_FRONT_STRUCT;
RARM_BACK_STRUCT.commandsAlias = RARM_BACK';

LARM_BACK_STRUCT = LARM_FRONT_STRUCT;
LARM_BACK_STRUCT.commandsAlias = LARM_BACK';


HIP_OFFSET_Y = 50;

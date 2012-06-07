
global servos sonar fsrs bumpers gps accelerometer gyro camera rphalanx lphalanx;

%%   Setting up Devices
CAMERA_TIME_STEP = 40;

% camera
camera = wb_robot_get_device('camera');
cameraSelect = wb_robot_get_device('CameraSelect');
wb_camera_enable(camera, CAMERA_TIME_STEP);
alreadyFailedToSendImage = 0;

% intertial unit
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);
gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

% GPS (not a real Nao device)
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);

% sonar
sonar(USTopRight)    = wb_robot_get_device('USSensor1');
sonar(USTopLeft)     = wb_robot_get_device('USSensor3');
sonar(USBottomRight) = wb_robot_get_device('USSensor2');
sonar(USBottomLeft)  = wb_robot_get_device('USSensor4');
for i = 1:US_MAX-1
    wb_distance_sensor_enable(sonar(i), TIME_STEP);
end

% force sensitive resistors (pressure sensors)
fsrs(LFsrFL) = wb_robot_get_device('LFsrFL');
fsrs(LFsrFR) = wb_robot_get_device('LFsrFR');
fsrs(LFsrBR) = wb_robot_get_device('LFsrRR');
fsrs(LFsrBL) = wb_robot_get_device('LFsrRL');
fsrs(RFsrFL) = wb_robot_get_device('RFsrFL');
fsrs(RFsrFR) = wb_robot_get_device('RFsrFR');
fsrs(RFsrBR) = wb_robot_get_device('RFsrRR');
fsrs(RFsrBL) = wb_robot_get_device('RFsrRL');
for i = 1:FSR_MAX-1
    wb_touch_sensor_enable(fsrs(i), TIME_STEP);
end

% foot bumpers
bumpers(RFootBumperRight) = wb_robot_get_device('BumperRFootRight');
bumpers(RFootBumperLeft)  = wb_robot_get_device('BumperRFootLeft');
bumpers(LFootBumperRight) = wb_robot_get_device('BumperLFootRight');
bumpers(LFootBumperLeft)  = wb_robot_get_device('BumperLFootLeft');
for i = 1:BUMPER_MAX-1
    wb_touch_sensor_enable(bumpers(i), TIME_STEP);
end

% joints
servos(HeadYaw)        = wb_robot_get_device('HeadYaw');
servos(HeadPitch)      = wb_robot_get_device('HeadPitch');
servos(LShoulderPitch) = wb_robot_get_device('LShoulderPitch');
servos(LShoulderRoll)  = wb_robot_get_device('LShoulderRoll');
servos(LElbowYaw)      = wb_robot_get_device('LElbowYaw');
servos(LElbowRoll)     = wb_robot_get_device('LElbowRoll');
servos(LWristYaw)      = wb_robot_get_device('LWristYaw');
servos(LHand)          = 0;
servos(LHipYawPitch)   = wb_robot_get_device('LHipYawPitch');
servos(LHipRoll)       = wb_robot_get_device('LHipRoll');
servos(LHipPitch)      = wb_robot_get_device('LHipPitch');
servos(LKneePitch)     = wb_robot_get_device('LKneePitch');
servos(LAnklePitch)    = wb_robot_get_device('LAnklePitch');
servos(LAnkleRoll)     = wb_robot_get_device('LAnkleRoll');
servos(RHipYawPitch)   = wb_robot_get_device('RHipYawPitch');
servos(RHipRoll)       = wb_robot_get_device('RHipRoll');
servos(RHipPitch)      = wb_robot_get_device('RHipPitch');
servos(RKneePitch)     = wb_robot_get_device('RKneePitch');
servos(RAnklePitch)    = wb_robot_get_device('RAnklePitch');
servos(RAnkleRoll)     = wb_robot_get_device('RAnkleRoll');
servos(RShoulderPitch) = wb_robot_get_device('RShoulderPitch');
servos(RShoulderRoll)  = wb_robot_get_device('RShoulderRoll');
servos(RElbowYaw)      = wb_robot_get_device('RElbowYaw');
servos(RElbowRoll)     = wb_robot_get_device('RElbowRoll');
servos(RWristYaw)      = wb_robot_get_device('RWristYaw');
servos(RHand)          = 0;
for i = 1:SERVO_MAX-1
    if (servos(i))  % avoid warning with the hands
                     % enable reading servo feedback position from Webots
        wb_servo_enable_position(servos(i), TIME_STEP);
    end
end


% Hands
% get phalanx motors
% the real Nao has only 2 motors for RHand/LHand
% but in Webots we must implement RHand/LHand with 2x8 motors
for i = 1:PHALANX_MAX-1
    name = ['LPhalanx', num2str(i)];
    lphalanx(i) = wb_robot_get_device(name);
    name = ['RPhalanx', num2str(i)];
    rphalanx(i) = wb_robot_get_device(name);
end

% initialize c++ modules
C_call('TU_init');
C_call('DCM_init');
C_call('DCM_setTime',0);
C_call('DCM_setAlias', HOME_STRUCT);
C_call('BB_setSupport',0);

    %set Time for C++ Modules
    C_call('DCM_setTime', time);
    C_call('BB_updateTime', time);
    
     % update Blackboard Kinematics
    
    % read joint angles from webots (look in getJointAngles.m for further
    % information )
    jointAngles   = getJointAngles();
    
    % Store angles in Blackboard
    C_call('BB_updateJointAngles', jointAngles);
    
    % update kinematics in Blackboard
    C_call('BB_updateKinematicMatrices');
    
    % update center of mass in Blackboard
    C_call('BB_updateCom');
    
    
    % read sensor-data
    for n = 1:8
        fsrdata(n) = wb_touch_sensor_get_value(fsrs(n));
    end   
    accData = wb_accelerometer_get_values(accelerometer);
    gyrData = wb_gyro_get_values(gyro);   
    
    % Store sensor data
    C_call('BB_updateFsr', fsrdata);
    C_call('BB_updateAccelerometerData', accData);   
    C_call('BB_updateGyroscopeData', gyrData); 
    C_call('BB_updateAngleEstimation');
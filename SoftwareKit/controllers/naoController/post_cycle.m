% get interpolated command angles from DCM
    jointCommands = C_call('DCM_updateCommands');
    
    % set commands in webots
    setJointAngles(jointCommands);  
    
    % refresh plots, if there are any
    drawnow;
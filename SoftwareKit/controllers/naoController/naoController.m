% MATLAB controller for Webots
% File:          naoController.m
% Date:          
% Description:   Nao Controller

% Author:        Stefan Kaufmann
% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

TIME_STEP = 10;

% load enumerations
enums;
% load constants
constants;

% initialize
wb_robot_init();
init;

%% Main Loop

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination


while wb_robot_step(TIME_STEP) ~= -1
    
    % get simulation time from webots
    time = round(1000 * wb_robot_get_time()); 
    
    pre_cycle;
    
          
    post_cycle;    
end



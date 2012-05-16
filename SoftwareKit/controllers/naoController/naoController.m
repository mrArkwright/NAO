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

wb_robot_keyboard_enable(TIME_STEP);
disp 'select the simulation window'
disp 'shift the upper body around by pressing the arrowkeys and q,e'
disp 'switch into automatic mode by pressing a'

manual_mode_flag=false;
if manual_mode_flag
  disp 'manual mode...'
else
  disp 'automatic mode...'
end
time_entering_automode=0;


manual_velocity=20;

while wb_robot_step(TIME_STEP) ~= -1
  
  % get simulation time from webots
  time = round(1000 * wb_robot_get_time()); 
  
  pre_cycle;
  
  if (~manual_mode_flag) && (time-time_entering_automode > 100) && (time-time_entering_automode < 2000)
    disp 'calling statBalance...'
    C_call('S_statBalance',true,100)
  end
  
  if (~manual_mode_flag) && (time-time_entering_automode > 2000)
    disp 'calling statBalance...'
    C_call('S_statBalance',false,100)
  end
  
  if (~manual_mode_flag) && (time-time_entering_automode == 2000)
    disp 'calling moveFoot...'
    C_call('S_moveFoot',50,500)
  end
  
  if (~manual_mode_flag) && (time-time_entering_automode == 3000)
    disp 'calling moveLArm...'
    C_call('S_moveLArm',60 ,500)
  end
  
  
  key = wb_robot_keyboard_get_key();
  if key ~= 0
    if (~manual_mode_flag) && (key ~= 'A')
      manual_mode_flag=true;
      disp 'manual mode...'
    end
    switch key
      case WB_ROBOT_KEYBOARD_UP
        C_call('S_moveUpperBody',0,0,-manual_velocity,TIME_STEP)
        %disp up
      case WB_ROBOT_KEYBOARD_DOWN
        C_call('S_moveUpperBody',0,0,manual_velocity,TIME_STEP)
        %disp down
      case WB_ROBOT_KEYBOARD_RIGHT
        C_call('S_moveUpperBody',0,manual_velocity,0,TIME_STEP)
        %disp right
      case WB_ROBOT_KEYBOARD_LEFT
        C_call('S_moveUpperBody',0,-manual_velocity,0,TIME_STEP)
        %disp left
      case 'Q'
        C_call('S_moveUpperBody',-manual_velocity,0,0,TIME_STEP)
        %disp forward
      case 'E'
        C_call('S_moveUpperBody',manual_velocity,0,0,TIME_STEP)
        %disp backward
      case 'A'
        if manual_mode_flag
          manual_mode_flag=false;
          disp 'automatic mode...'
          time_entering_automode = time;
        end
    end
  end
  
  
	%if (time >= 4000 && time < 4001)
	%	C_call('S_goDown', 40, 500);
	%end
  
  post_cycle;    
end



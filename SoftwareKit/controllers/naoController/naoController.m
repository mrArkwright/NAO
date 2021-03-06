% Author:        Stefan Kaufmann
% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

TIME_STEP = 5;

kickAngle = 5; %degree -7..14

% load enumerations
enums;
% load constants
constants;

% initialize
wb_robot_init();
init;

wb_robot_keyboard_enable(TIME_STEP);
disp 'select the simulation window'
disp 'shift the upper body around by pressing the arrowkeys and q,e'
disp 'switch into automatic mode by pressing a'

manual_velocity=20;
manual_mode_flag=false;
if manual_mode_flag
  disp 'manual mode...'
else
  disp 'automatic mode...'
end
time_entering_automode=0;


% Test: initialisieren der Kamera
%   cam = wb_robot_get_device('camera');
%   wb_camera_enable(cam, TIME_STEP);

while wb_robot_step(TIME_STEP) ~= -1
  
  % get simulation time from webots
  time = round(1000 * wb_robot_get_time()); 
  
  pre_cycle;
  
  %Test: Ausgabe des Kamerabildes
  %   im = wb_camera_get_image(cam);
  %   image(im);
  
  rel_time = time-time_entering_automode;
  
  %auf beiden füßen stehend den schwerpunkt über den fuß schieben
  if (~manual_mode_flag) && (rel_time == 1000)
    disp 'statBalance...'
    C_call('S_statBalance',true,500)
  end

  %Fuß heben
  if (~manual_mode_flag) && (rel_time == 1500)
    disp 'raise Foot...'
    
    if (kickAngle <= 0)
      deltaX = -kickAngle/(-7.63195750157995)*25;
    else
      deltaX = kickAngle/(14.2093727765404)*25;
    end
    
    C_call('S_moveFoot',0,deltaX,30,500)
  end
  
  if (~manual_mode_flag) && (rel_time == 2000)
    disp 'move foot back...'
    C_call('S_moveFoot',-40,0,0,500)
  end
  
  if (~manual_mode_flag) && (rel_time == 3000)
    disp 'kick!'
    C_call('S_moveFoot',200,0,0,100)
  end
  
  
  
  %keyboard input:
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

  
  post_cycle;    
end





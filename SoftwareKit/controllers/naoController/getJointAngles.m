function angles = getJointAngles()
global SERVO_MAX RHand LHand servos rphalanx lphalanx
angles = zeros(26,1);   
    for i = 1:SERVO_MAX-1    
        switch i
            case RHand
                angles(i) = wb_servo_get_position(rphalanx(1));
            case LHand
                angles(i) = wb_servo_get_position(lphalanx(1));
            otherwise
                angles(i) = wb_servo_get_position(servos(i));
        end
    end
end


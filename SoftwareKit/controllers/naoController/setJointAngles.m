function setJointAngles( angles )
global SERVO_MAX RHand LHand PHALANX_MAX servos rphalanx lphalanx
    for i = 1:SERVO_MAX-1    
        switch i
            case RHand
                for j = 1:PHALANX_MAX-1
                    wb_servo_set_position(rphalanx(j), angles(RHand));
                end
            case LHand
                for j = 1:PHALANX_MAX-1
                    wb_servo_set_position(lphalanx(j), angles(LHand));
                end
            otherwise
                wb_servo_set_position(servos(i), angles(i));
        end
    end



end


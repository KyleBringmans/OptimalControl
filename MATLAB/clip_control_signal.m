function control_signal = clip_control_signal(control_signal, thrust)
% this function clamps the control signal to the proper allowed
% common thrust
% 
the_size = size(control_signal,1);
MAX_CONTROL = 0.05;
MAX_YAW_CONTROL = 0.08;
MAX_NAV_CONTROL = 0.05;

% clamp control signal for attitude
if the_size == 3
    if (control_signal(3) > MAX_YAW_CONTROL)
        control_signal(3) = MAX_YAW_CONTROL;
    elseif (control_signal(3) < -MAX_YAW_CONTROL)
        control_signal(3) = -MAX_YAW_CONTROL;
    end
    
    %1) calculate total thrust:
    absolute_sum = abs(control_signal(1)) + abs(control_signal(2)) +...
        abs(control_signal(3));
    max_absolute_sum = min(thrust, 1 - thrust);
    
    if (absolute_sum > max_absolute_sum)
        factor = max_absolute_sum / absolute_sum;
        control_signal = control_signal .* factor;
    end
    
% clamp control signal for altitude
elseif the_size == 1
     if (control_signal > MAX_CONTROL)
            control_signal = MAX_CONTROL;
     elseif (control_signal < -MAX_CONTROL)
            control_signal = -MAX_CONTROL;
     end
    
% clamp control signal for navigation
elseif the_size == 2
     if (control_signal(1) > MAX_NAV_CONTROL)
            control_signal(1) = MAX_NAV_CONTROL;
     elseif (control_signal(1) < -MAX_NAV_CONTROL)
            control_signal(1) = -MAX_NAV_CONTROL;
     end
     
     if (control_signal(2) > MAX_NAV_CONTROL)
            control_signal(2) = MAX_NAV_CONTROL;
     elseif (control_signal(2) < -MAX_NAV_CONTROL)
            control_signal(2) = -MAX_NAV_CONTROL;
     end
    
end



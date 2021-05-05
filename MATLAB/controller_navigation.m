function [next_state_estimate, next_control_signal, z_int] = controller_navigation...
    (meas, reference, state_estimate, control, plant, z_int,controller)
%Z_INT MEEGEVEN?(mssn beter buiten deze functie?) MEAS ALS Y?
% this function calculates the control signals and the state estimate for
% the attitude system.
% inputs: 
%       - meas: measurement of some state elements.
%       - reference: the reference to track.
%       - thrust: thrust input.
%       - state: the previous state.
%
% returns:
%       - control_signal: the new control signal, calculated with the
%                           inputs
%       - state_estimate: the new state estimate, calculated with the 
%                           inputs
     
    % Calculate next state estimate
    next_state_estimate = plant.navigation.Ad * state_estimate +...
    controller.navigation.L * (plant.navigation.Cd * state_estimate - meas)...
        + plant.navigation.Bd * control;
    
    % Calculate the equilibrium x and u
    xue = controller.navigation.G*reference;
    x_e = xue(1:6);
    u_e = xue(6 + 1:2 + 6);
    
    % integral action
    %z_int = z_int + reference - meas;
    z_int = z_int + reference - next_state_estimate(3:4);

    
    % Finally calculate the control action
    next_control_signal = u_e + controller.navigation.Kx*(next_state_estimate - x_e)...
        + controller.navigation.Kz*z_int;
    next_control_signal = clip_control_signal(next_control_signal);
    
    
    
    
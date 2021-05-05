function [next_state_estimate, next_control_signal,z_int] = controller_altitude...
    (meas, reference, thrust, state_estimate, control,plant,z_int,controller)
%Z_INT MEEGEVEN?(mssn beter buiten deze functie?) MEAS ALS Y?
% this function calculates the control signals and the state estimate for
% the attitude system.
% inputs: 
%       - meas: measurement of some state elements.
%       - reference: the reference to track.
%       - thrust: thrust input.
%       - state: the previous state_estimate.
%       - control: the previous control_signal.
%
% returns:
%       - control_signal: the new control signal, calculated with the
%                           inputs
%       - state_estimate: the new state estimate, calculated with the 
%                           inputs

% 1) bereken de next_state_estimate
    next_state_estimate = plant.altitude.Ad * state_estimate +...
        controller.altitude.L * (plant.altitude.Cd * state_estimate - meas)...
        + plant.altitude.Bd * control;
     
% 2) bereken het control_signal
    % a) [x_e u_e] = G * ref
    %    Calculate the equilibrium x and u
    xue = controller.altitude.G*reference;
    x_e = xue(1:3); % nx ingevuld!!!
    u_e = xue(4); % nx & nu ingevuld!!
    
    % integral action
    z_int = z_int + reference - meas;
    
    % 2) control = u_e + K * (next_state_estimate - x_e)
    %temp =  xEst - x_e;
    % u = u_e + Kx * temp + Kz*z_int;
    temp = state_estimate - x_e;
    % Finally calculate the control action
    next_control_signal = u_e + controller.altitude.Kx*temp + controller.altitude.Kz*z_int;
    next_control_signal = clip_control_signal(next_control_signal);
    
    
    
    
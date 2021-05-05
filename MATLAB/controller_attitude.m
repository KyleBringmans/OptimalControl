function [next_state_estimate, next_control_signal] = controller_attitude...
    (meas, reference, thrust, state_estimate, control,plant,controller)
% this function calculates the control signals and the state estimate for
% the attitude system.
% inputs: 
%       - meas: measurement of some state elements.
%       - reference: the reference to track.
%       - thrust: thrust input.
%       - state: the previous state_estimate.
%       - control: the previous control_signal.
%       - a: system matrices.
%
% returns:
%       - control_signal: the new control signal, calculated with the
%                           inputs
%       - state_estimate: the new state estimate, calculated with the 
%                           inputs


% 1) bereken de next_state_estimate
    next_state_estimate = plant.attitude.Ad * state_estimate +...
        controller.attitude.L * (plant.attitude.Cd * state_estimate - meas) + plant.attitude.Bd * control;
    
    % opm: quaternionen aftrekking gebruiken?
    
% 2) bereken het control_signal
    % a) [x_e u_e] = G * ref
    %    Calculate the equilibrium x and u
    xue = controller.attitude.G*reference;
    x_e = xue(1:9); % nx ingevuld!!!
    u_e = xue(9 + 1 : 3 + 9); % nx & nu ingevuld!!
    
    
    % 2) control = u_e + K * (next_state_estimate - x_e)
    % compute the control action
    % First three elements of x are quaternions, so no normal substraction
    % Construct q0_e, in order to use quatmultiply
    %diff = next_state_estimate - x_e
    diff = zeros(9,1);
    diff(1:3) = quatSubstract(next_state_estimate(1:3),x_e(1:3));
    diff(4:9) = next_state_estimate(4:9) - x_e(4:9);
    

    % Finally calculate the control action
    next_control_signal = u_e + controller.attitude.K * diff;
    next_control_signal = clip_control_signal(next_control_signal,thrust);
    
    
    
    
    
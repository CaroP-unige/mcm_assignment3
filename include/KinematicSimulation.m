function [q_new] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q_new new joint configuration

q_new = zeros(length(q), 1);
q_new = max(q_min, min(q_max, q + q_dot *ts));
end
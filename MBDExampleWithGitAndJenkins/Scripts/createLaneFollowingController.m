% This script presents the steps for designing an MPC controller for lane
% following applications.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2018-2020 The MathWorks, Inc.

%% Specify internal MPC model 
% time constant for longitudinal dynamics 1/s/(tau*s+1)
tau = 0.5; 

% Specify state-space longitudinal model 
%   outputs: relative distance and longitudianl velocity
%   inputs: acceleration and lead car velocity
A1 = [-1/tau,0,0;1,0,0;0,-1,0]; 
B1 = [1/tau,0;0,0;0,1]; 
C1 = [0,0,1;0,1,0];
sys_longitudinal = ss(A1,B1,C1,0); 
                                                                                      
% lateral dynamics
[Ag,Bg,Cg] = getVehModelFromParam(m,Iz,lf,lr,Cf,Cr,v0_ego);
n = size(Bg,1); % number of states for lateral dynamics  

% Specify state-space lateral model 
%   outputs: lateral deviation and relative yaw angle
%   inputs: steering angle and previewed curvature
[A2,B2] = getFullModel(Ag,Bg,Cg,v0_ego);
C2 = [zeros(2,n),eye(2)];
sys_lateral = ss(A2,B2,C2,0);                  

% MPC internal model
sys = [sys_longitudinal,zeros(2,2);zeros(2,2),sys_lateral]; % combined model
% add lag dynamics
lag = 0.1;
sys = sys*tf(1,[lag,1]);
sys = minreal(sys); % minimal realization

%% MPC design
status = mpcverbosity('off');

sys = setmpcsignals(sys,'MV',[1,3],'MD',[2,4]); % set signal types
sysd = c2d(sys,Ts); % discretize model
mpc1 = mpc(sysd,Ts); % start mpc design

%% Specify the nominal values based on the simulation initial conditions.
%  plant inputs at operating point
mpc1.Model.Nominal.U(1) = 0;                                    % acceleration
mpc1.Model.Nominal.U(2) = v0_ego;                               % lead car velocity
%  plant outputs at operating point
mpc1.Model.Nominal.Y(1) = time_gap*v0_ego + default_spacing;    % relative distance
mpc1.Model.Nominal.Y(2) = v0_ego;                               % longitudinal velocity

%% Specify the prediction horizon.
mpc1.PredictionHorizon = PredictionHorizon;

%% Specify scale factors based on the operating ranges of the variables
%--- Normalize dynamic ranges to improve optimization
mpc1.MV(1).ScaleFactor = max_ac - min_ac;           % acceleration
mpc1.MV(2).ScaleFactor = max_steer - min_steer;     % steering

mpc1.OV(1).ScaleFactor = 50;       % relative distance
mpc1.OV(2).ScaleFactor = 30;       % ego longitudinal velocity
mpc1.OV(3).ScaleFactor = 1;        % lateral deviation
mpc1.OV(4).ScaleFactor = 0.2;      % yaw angle deviation

mpc1.DV(1).ScaleFactor = 30;       % lead car velocity
mpc1.DV(2).ScaleFactor = 0.1;      % road yaw angle rate = v0_ego*curvature

%% Specify constraints
mpc1.MV(1).Min = min_ac;    % min acceleration 
mpc1.MV(1).Max = max_ac;    % max acceleration

mpc1.MV(2).Min = min_steer; % min steering
mpc1.MV(2).Max = max_steer; % max steering

mpc1.OV(1).Min = time_gap*v0_ego + default_spacing; % safe distance
% Constrain relative distance to ensure safe distance between ego and mio

%% Specify weights for optimization
alpha = 1; % non-zero
mpc1.Weights.MVRate = [0.1 0.1]*alpha;  % robustness for acceleration and steering
% Minimize jerk (derivative of acceleration) and derivative of steering (weight > 0)

mpc1.Weights.OV = [0 0.1 1 0]/alpha;    % outputs = [relative distance, ego velocity,
                                        %            lateral deviation, yaw angle deviation]
% Ignore space error and yaw angle error during optimization  (weight = 0)
% Minimize error against measured velocity and lateral deviation (weight > 0)

mpcverbosity(status);

%% Local function: get continuous vehicle lateral model from parameters
function [Ag,Bg,Cg] = getVehModelFromParam(m,Iz,lf,lr,Cf,Cr,Vx)
% Get continuous vehicle model from vehicle parameters with steering angle
% as input and the first output is the lateal velocity and the second
% output is the yaw angle rate.

% Specify vehicle state-space model with state varaibles [vy,phidot]. 
%   vy:       lateral velocity
%   phidot:   yaw angle rate
%   dynamics: dx = [a1,a2;a3,a4]x + [b1;b2]u.
a1 = -(2*Cf+2*Cr)/m/Vx;
a2 = -(2*Cf*lf-2*Cr*lr)/m/Vx - Vx;
a3 = -(2*Cf*lf-2*Cr*lr)/Iz/Vx;
a4 = -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx;
b1 = 2*Cf/m;
b2 = 2*Cf*lf/Iz;
% Specify the vehicle model matrices with states [vy,phidot].
Ag = [a1,a2;a3,a4];
Bg = [b1;b2];
Cg = eye(2);
end

%% Local function: get lateral model
function [Af,Bf] = getFullModel(Ag,Bg,Cg,Vx)
% dynamics for Gp from [delta,phidotdes] to [vy,phidot,phidotdes]
Ap = Ag;
n = size(Bg,1);                                                
Bp = [Bg,zeros(n,1)];
Cp = [Cg;zeros(1,n)];
Dp = [zeros(2);0,1];

% continuous dynamics for Ge from [vy,phidot,phidotdes] to [e1,e2]
Ae = [0,Vx;0,0];
Be = [1,0,0;0,1,-1];

% series connection of Gp and Ge
Af = [Ap,zeros(n,2);Be*Cp,Ae];
Bf = [Bp;Be*Dp];
end
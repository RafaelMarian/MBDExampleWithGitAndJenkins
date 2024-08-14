function [A,B,C,D] = getDiscreteModelForAdaptiveMPC(Ts,Vx,lag,tau,m, Cr, Cf, lr, lf, Iz)
%#codegen
%
% Get discrete state-space model for adaptive MPC. 

% Inputs:
%   Ts:                sample time
%   Vx:                longitudinal velocity
%   lag:               lumped lag
% Outputs:
%   A,B,C,D:           discrete time model for MPC

%   Author: Meng Xia
%   Copyright 2017-2020 The MathWorks, Inc.


% Specify state-space longitudinal model 
%   outputs: relative distance and longitudianl velocity
%   inputs: acceleration and lead car velocity
A1 = [-1/tau,0,0;1,0,0;0,-1,0]; 
B1 = [1/tau,0;0,0;0,1]; 
C1 = [0,0,1;0,1,0];
D1 = zeros(2);       

% lateral dynamics
[Ag,Bg,Cg] = getVehModelFromParam(m,Iz,lf,lr,Cf,Cr,Vx);
n = size(Bg,1); % number of states for lateral dynamics                                         
% Specify state-space lateral model 
%   outputs: lateral deviation and relative yaw angle
%   inputs: steering angle and previewed curvature
[A2,B2] = getFullModel(Ag,Bg,Cg,Vx);
% Define constant outputs
C2 = [zeros(2,n),eye(2)];
D2 = zeros(2);

% Get MPC internal model. 
A5 = [A1,zeros(3,4);zeros(4,3),A2];
B5 = [B1,zeros(3,2);zeros(4,2),B2];
C5 = [C1,zeros(2,4);zeros(2,3),C2];
D5 = [D1,zeros(2);zeros(2),D2];

% Add lag dynamics
[Az,Bz,Cz,Dz] = addLagModel(A5,B5,C5,D5,lag);

% Discretize model
[A, B] = getDiscrete(Az, Bz, Ts);
C = Cz;
D = Dz;
end






%% Local function: augment the lag dynamics
function [Az,Bz,Cz,Dz] = addLagModel(Af,Bf,Cf,Df,lag)
% lag dynamics Gl
Al = -1/lag*eye(4);
Bl = 1/lag*eye(4);
Cl = eye(4);
Dl = zeros(4);

% series connection of Gf and and Gl
n = size(Bf,1);                                                
Az = [Af,zeros(n,4);Bl*Cf,Al];
Bz = [Bf;Bl*Df];
Cz = [Dl*Cf,Cl];
Dz = Dl*Df;
end

%% Local function: Deiscretize model with sample time Ts
function [A, B] = getDiscrete(a, b, Ts)
% Convert to discrete time
A = expm(a*Ts);
nx = size(b,1);
n = 4;  % Number of points for Simpson's Rule, an even integer >= 2.
% Use Simpson's rule to compute integral(0,Ts){expm(a*s)*ds*b}
h = Ts/n;
Ai = eye(nx) + A;        % First and last terms
Coef = 2;
for i = 1:n-1
    if Coef == 2
        Coef = 4;
    else
        Coef = 2;
    end
    Ai = Ai + Coef*expm(a*i*h);     % Intermediate terms
end
B = (h/3)*Ai*b;
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

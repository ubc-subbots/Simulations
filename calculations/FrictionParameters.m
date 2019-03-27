clear;
clc;
close all;

%% Flow Friction parameters
Cd_x = 1.0;     % Finite cylinder, horizontal
Cd_y = 1.0;     % Finite cylinder, horizontal
Cd_z = 1.0;     % Finite cylinder, horizontal

%% Fluid parameters at 22 deg.C
rho = 998.2;    % kg/m3

%% Vessel Projected Areas [m2]
Ax = 0.32897*0.60971;
Ay = 0.17178704;
Az = 0.75627*0.31998 - 0.02532104;

%% Load inertia, added inertia
m = 24.5;
ma_x = 12.8340;
ma_y = 17.3295;
ma_z = 21.9371;
I_x = 1.7047;
I_z = 1.6031;


%% Friction Damping Parameter Calculations
k_Qx = 0.5*rho*Ax*Cd_x;
k_Qy = 0.5*rho*Ay*Cd_y;
k_Qz = 0.5*rho*Az*Cd_z;

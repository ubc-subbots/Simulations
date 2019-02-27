% System Parameters
g = 9.81; %m/s^2

% Ideally, State vector x = [x, y, z, v_x, v_y, v_z ..
%                          ..roll, pitch, yaw, w_x, w_y, w_z]

% Sensors are IMUs and Hydrophones
% Output measurements:
% a_x, a_y, a_z
% w_x, w_y, w_z
% z

% Indirect measurements:
% v_x, v_y (integrate accel)
% v_z (differentiate z)
% roll, pitch, yaw (integrate omega)

s = tf('s');
% C = [0, 0, 0, 1/s, 0, 0, 0, 0, 0, 0,  0; ...
%      0, 0, 0, 0, 1/s, 0, 0, 0, 0, 0, 0, 0; ...
%      0, 0, 0, 0, 0, 1/s, 0, 0, 0, 0, 0, 0; ...
%      0, 0, 0, 0, 0, 0, 1/s, 0, 0, 0, 0, 0; ...
%      0, 0, 0, 0, 0, 0, 0, 1/s, 0, 0, 0, 0; ...
%      0, 0, 0, 0, 0, 0, 0, 0, 1/s, 0, 0, 0; ...
%      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0; ...
%      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0; ...
%      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];

C = eye(12);
 
 
 %% Importing Parameters

filename = 'parameters2.xlsx';
rho = 1000;
g = -9.81; %m/s^2
sheet = 1;
xlRange = 'D1:D42';
parameters = xlsread(filename,sheet,xlRange);


m = parameters(1); % sub mass measured using hanging scale in air
W = m*g;
COM = [parameters(6),parameters(7),parameters(8)];
Ix = parameters(2); %inertia about the COM + parallel axis theorem to take all inertias about geometric center
Iy = parameters(3);
Iz = parameters(4);

% Buoyancy
%B = parameters(5); %%%
COB = [parameters(9),parameters(10),parameters(11)];

% Viscous
bx = parameters(12); %linear damping is 'b'
by = parameters(13);
bz = parameters(14);
cx = parameters(15); %rotational damping is 'c'
cy =parameters(16);
cz = parameters(17);
COPx = [parameters(18),parameters(19),parameters(20)]; % y,z of front face COP
COPy = [parameters(21),parameters(22),parameters(23)]; % x,z of right face COP
COPz = [parameters(24),parameters(25),parameters(26)]; % x,y of top face COP

% Thrusters
d100 = [parameters(30),parameters(31),parameters(32)]; %x,y,z of starboard T200
d200 = [parameters(27),parameters(28),parameters(29)]; %x,y,z of starboard T100
theta100 = pi/4; % rad
%K1_T = parameters(33); %linearized constant converting PWM in to Torque and Thrust out for the T100
%K1_F = parameters(34);
%K2_T = parameters(35); %again for the T200
%K2_F = parameters(36);

% using torque values directly insread of converting PWM to torque, will do
% the conversion in software
K1_T = 1; %linearized constant converting PWM in to Torque and Thrust out for the T100
K1_F = 1;
K2_T = 1; %again for the T200
K2_F = 1;

Ap_x = parameters(37);
Ap_y = parameters(38);
Ap_z = parameters(39);

Dim_x = parameters(40)*(10^-3);
Dim_y = parameters(41)*(10^-3);
Dim_z = parameters(42)*(10^-3);

% Predetermined Constants
ratios = [1,2,3,4,5,6,7,10];
C_A = [0.68, 0.36, 0.24, 0.19, 0.15, 0.13, 0.11, 0.09];

% Calculate Ma_y (Thin Rectangle) V_R = (PI/4)a^2 b where b>a
dim_ratio_y = (Dim_x/Dim_z);
C_Ay = interp1(ratios, C_A, dim_ratio_y);
V_Ry = pi/4*(Dim_z^2)*Dim_x;
Ratio_y = Ap_y/(Dim_z*Dim_y);
Ma_y = rho*C_Ay*V_Ry*Ratio_y;

% Calculate Ma_z (Thin Rectangle) 
dim_ratio_z = (Dim_x/Dim_y);
C_Az = interp1(ratios, C_A, dim_ratio_z)
V_Rz = pi/4*(Dim_y^2)*Dim_x;
Ratio_z = Ap_z/(Dim_y*Dim_x);
Ma_z = rho*C_Ay*Ap_y*Dim_y;

% Calculate Ma_x (Rectangular Prism) V_R = a^2 b where b>a
dim_ratio_x = (Dim_x/(0.5*(Dim_y+Dim_z)));
C_Ax = interp1(ratios, C_A, dim_ratio_x);
V_Rx = Dim_x*((0.5*(Dim_y+Dim_z))^2);
Ratio_x = Ap_x/(Dim_y*Dim_z);
Ma_x = rho*C_Ax*V_Rx*Ratio_x;

%% A and B Matrix Creation

A = zeros(12,12);
A(1,:) = (unit_vec(12,4))';
A(2,:) = (unit_vec(12,5))';
A(3,:) = (unit_vec(12,6))';
A(4,:) = (-0.5*unit_vec(12,4))'; % TODO - Add correct coefficients for linear drag
A(5,:) = (-0.5*unit_vec(12,5))';
A(6,:) = (-0.5*unit_vec(12,6))';
A(7,:) = (unit_vec(12,10))';
A(8,:) = (unit_vec(12,11))';
A(9,:) = (unit_vec(12,12))';
A(10,:) = (-0.5*unit_vec(12,10))';
A(11,:) = (-0.5*unit_vec(12,11))';
A(12,:) = (-0.5*unit_vec(12,12))';

L_lever = 0.2; % TODO - Correct lever arm length
B = zeros(12,6); % TODO - 
B(4,:) = [-cosd(45) -cosd(45) -cosd(45) -cosd(45) 0 0];
B(5,:) = [-cosd(45) +cosd(45) -cosd(45) +cosd(45) 0 0];
B(6,:) = [ 0 0 0 0 1 1];
B(12,:) = [-L_lever L_lever L_lever -L_lever 0 0];

%% Analysis PT1 Stability, Controllability, Observability

rank(obsv(A,C))
rank(ctrb(A,B))
eig(A)

[A_bar, B_bar, C_bar, T_bar, k] = ctrbf(A,B,C)
% Conclusion - stable, observable BUT not controllable
%              This is expected. Y, Roll, Pitch are uncontrollable based on
%              motor config.

%% Kalman Decomposition

sys = ss(A,B,C,0);
sys_min = minreal(sys);



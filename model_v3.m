%% 2019 Subbots Underactuated Vehicle Model
clc;
clear;
close all;

%% Parameters:
%defining indices
x=1; 
y=2;
z=3;

% Constants
rho = 1000;
g = -9.81;

% Inertial
m = 25; % sub mass measured using hanging scale in air
W = m*g;
COM = [0,0,0];
Ix = 1 + m*COM(x)^2; %inertia about the COM + parallel axis theorem to take all inertias about geometric center
Iy = 1 + m*COM(y)^2;
Iz = 1 + m*m^2;

% Buoyancy
B = W*-1.1; %%%
COB = [0,0,0];

% Viscous
bx = .1; %linear damping is 'b'
by = .1;
bz = .1;
cx = .1; %rotational damping is 'c'
cy = .1;
cz = .1;
COPx = [0,0.01,0.01]; % y,z of front face COP
COPy = [0.01,0,0.01]; % x,z of right face COP
COPz = [0.01,0.01,0]; % x,y of top face COP



d100 = [0,.3,-.2]; %x,y,z of starboard T200
d200 = [0,.3, .2]; %x,y,z of starboard T100
theta100 = 45; %deg
K1_T = 1; %linearized constant converting PWM in to Torque and Thrust out for the T100
K1_F = 2;
K2_T = 3; %again for the T200
K2_F = 4;


%% Importing Parameters

filename = 'parameters2.xlsx';
rho = 1000;
g = -9.81; %m/s^2
sheet = 1;
xlRange = 'D1:D45';
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

Ma_x = parameters(43);
Ma_y = parameters(44);
Ma_z = parameters(45);

M_x = Ma_x + m;
M_y = Ma_y + m;
M_z = Ma_z + m;

% Load friction coefficients (Linearized around 0.3m/s, 10deg/s)
B_x = -60.0646;
B_y = -51.4433;
B_z = -64.8841;
B_tx = -37.7409;
B_tz = -29.9229;


%% Force Balance Equations written in plain text

%%%% NOTE: In an effort to keep things more linear, a simplification is made
%%%% such that the thruster forces do not gain additional directional
%%%% components when the sub pitches or rolls. In reality, a roll would
%%%% cause the T100s to result in unexpected motions in some off angle to horizontal
%%%% and vertical. Similarly, a pitch would yield vertical components to the
%%%% T200 thrust and additional components to the T100s. 

%%%% Coriolis effect will be ignored, assuming that rotational and
%%%% translational movements will be completed separately

%%%% Added mass during acceleration is also ignored for the sake of
%%%% linearity. Nonlinear viscous effects are linearized about an approximate operating
%%%% point
%%% Sum of Forces
% Fx = K2_F*(PWMs+PWMp)-bx*vx; %=m*ax
% Fy = sqrt(2)/2*K1_F*(PWMr-PWMl)-by*vy;%=m*ay
% Fz = sqrt(2)/2*K1_F*(PWMr+PWMl)-bz*vz;%=m*az

% % Sum of Torques
% Tx = sqrt(2)/2*K1_F*(PWMr*(-d100(y)+d100(z))+PWMl*(d100(y)-d100(z))) ... %x moments from the T100s
%         + K2_T*(PWMs-PWMp) ... % reaction torque on the T200 thrusters (Assumes starboard T200 has prop that will give POSITIVE torque when moving FWD)
%         + (B-W)*(theta_x_0+theta_x)*d_f_0 ... %restoring roll moment from bouyancy
%         - vy*by*COPy(z) ... %moment from y drag
%         + vz*bz*COPz(y) ... %moment from z drag
%         - wx*cx ... %moment from roll drag
%         ; %=Ix*alpha_x
% 
% Ty = K2_F*(PWMs+PWMp)*d200(z) ... %T200 pitching torque
%         ... %T100 pitching torque assumed zero - only true if both thrusters mounted at geometric center
%         + K1_T*sqrt(2)/2*((PWMr+PWMl)) ... % reaction torque on the T100 thrusters. Assumes that right T100 creates POSITIVE Y torque
%         + (B-W)*(theta_y_0+theta_y)*d_r_0 ... %restoring roll moment from bouyancy
%         + vx*bx*COPx(z) ... %moment from x drag
%         - vz*bz*COPz(x) ... %moment from z drag
%         - wy*cy ... %moment from pitch drag
%         - K2_F*d200(z)(PWMs-PWMp)
%         ; %=Iy*alpha_y
% 
% Tz = K2_F*(PWMs-PWMp)*d200(y) ... %T200 yaw torque
%         + K1_T*sqrt(2)/2*((PWMr-PWMl)) ... % reaction torque on the T100 thrusters. Assumes that right T100 creates POSITIVE Y torque
%         - vx*bx*COPx(y) ... %moment from x drag
%         + vy*by*COPy(x) ... %moment from y drag
%         - wz*cz ... %moment from yaw drag
%         - K2_F*d200(y)*(PWMs-PWMp)...% torque from thruster location
%         ; %=Iz*alpha_z

%% useful values for torque calcs
theta_x_0 = atan2d(COB(z)-COM(z),COB(y)-COM(y));% starting angle between COB and COM in Front Plane
d_f_0 = sqrt((COB(y)-COM(y))^2+(COB(z)-COM(z))^2); % distance between COB and COM in Front Plane
d_f_B = sqrt(COB(y)^2+COB(z)^2); % distance between COB and origin in Front Plane
d_f_M = sqrt(COM(y)^2+COM(z)^2); % distance between COM and origin in Front Plane
theta_y_0 = atan2d(COB(z)-COM(z),COB(x)-COM(x));% starting angle between COB and COM in Right Plane
d_r_0 = sqrt((COB(x)-COM(x))^2+(COB(z)-COM(z))^2); % distance between COB and COM in Right Plane
d_r_B = sqrt(COB(x)^2+COB(z)^2); % distance between COB and origin in Right Plane
d_r_M = sqrt(COM(x)^2+COM(z)^2); % distance between origin and COM in Right Plane


% x = [x,y,z,theta_x, theta_y, theta_z, vx, vy, vz, wx, wy, wz] %, ax, ay, az, alphax, alphay, alphaz]
% u = [PWMsf, PWMpf, PWMsb, PMWpb, PWMsu, PWMpu, C, W, B]
% C is the y-intercept of the linear best fit line of thrust vs PWM input
% curve from Bluestar T100 technical data
% y = [vx, vy, vz, wx, wy, wz]
% dx/dt = Ax + Bu
%        z   tx                   tz  vx          vy          vz          wx       wz
% A = [[   0   0                     0   0           0           1           0       0];     ... % vz
%      [   0   0                     0   0           0           0           1       0];     ... % wx
%      [   0   0                     0   0           0           0           0       1];     ... % wz
%      [   0   0                     0   -bx         0           0           0       0]/m;   ... % ax
%      [   0   0                     0   0          -by          0           0       0]/m;   ... % ay
%      [   0   0                     0   0           0          -bz          0       0]/m;   ... % az
%      [   0  -B*d_f_B+W*d_f_M       0   0          -by*COPy(z)  bz*COPz(y) -cx      0]/Ix;  ... % alphax
%      [   0   0                     0  -bx*COPx(y)  by*COPy(z)  0           0     -cz]/Iz];    % alphaz


%% Thruster (B) Matrix

% Thruster Locations
dpf = [1, 1, 0]*0.25;    % Top left
dsf = [1, -1, 0]*0.25;   % Top right
dpb = [-1, 1, 0]*0.25;   % Bottom left
dsb = [-1, -1, 0]*0.25;  % Bottom right
dl = [0, 2, 0]*0.25;     % left
dr = [0, -2, 0]*0.25;    % right

X = cos(theta100);
Y = sin(theta100); %keeping the thruster forces clean
Z = 1;

% Unit vectors of distances for torque equations for the motors 
Tx_sf = X*dsf(y);
Tx_pf = X*dpf(y);
Tx_sb = X*dsb(y);
Tx_pb = X*dpb(y);

Ty_sf = Y*dsf(x);
Ty_pf = -Y*dsf(x); 
Ty_sb = -Y*dsf(x);
Ty_pb = Y*dpb(x);

T_sf = Tx_sf + Ty_sf;
T_pf = Tx_pf + Ty_pf;
T_sb = Tx_sb + Ty_sb;
T_pb = Tx_pb + Ty_pb;

T_l = Z*dl(y);
T_r = Z*dr(y);

kappa = sqrt(2)/2*K1_F*(d100(y)-d100(z));
beta = theta_x_0*d_f_0;
nu = sqrt(2)/2*K1_T;
zu = K2_F*d200(z);
yu = K2_F*d200(y);
rho = theta_y_0*d_r_0;

% Input matrix consists of force supplied to the motors
% Assuming that the bot can roll, NOT pitch

%     Fsf   Fpf    Fsb    Fpb    Fsu    Fpu  
B = [[0      0      0      0      0      0             ];             ... % vz
     [0      0      0      0      0      0             ];             ... % wx
     [0      0      0      0      0      0             ];             ... % wz
     [X      X      X      X      0      0             ]/m;           ... % ax
     [Y      -Y     -Y     Y      0      0             ]/m;           ... % ay
     [0      0      0      0      Z      Z             ]/m;           ... % az
     [0      0      0      0     T_l    T_r            ]/Ix;          ... % alphax
     [T_sf  T_pf   T_sb   T_pb    0      0             ]/Iz];             % alphaz
 

 %% Output (C) matrix 
 
% y = Cx + Du
%     z  tx  tz  vx  vy  vz  wx   wz
C = [[0   0   0   1   0   0   0   0];
     [0   0   0   0   1   0   0   0];
     [1   0   0   0   0   0   0   0];
     [0   0   0   0   0   0   1   0];
     [0   0   0   0   0   0   0   0];
     [0   0   0   0   0   0   0   1]];
 
 D = zeros(6,6);
 

 %% Reduced State Model
%  A = [[   0   0                     0   -bx         0           0           0       0]/m;   ... % ax
%      [   0   0                     0   0          -by          0           0       0]/m;   ... % ay
%      [   0   0                     0   0           0          -bz          0       0]/m;   ... % az
%      [   0  -B*d_f_B+W*d_f_M       0   0          -by*COPy(z)  bz*COPz(y) -cx      0]/Ix;  ... % alphax
%      [   0   0                     0  -bx*COPx(y)  by*COPy(z)  0           0     -cz]/Iz];    % alphaz

I_x = Ix;
I_y = Iy;
I_z = Iz;

A = zeros(5,5);
A(1,1) = B_x/(m+Ma_x);
A(2,2) = B_y/(m+Ma_y);
A(3,3) = B_z/(m+Ma_z);
A(4,4) = B_tx/(I_x);
A(5,5) = B_tz/(I_z);

L_diags = 0.48; 
L_sides = 0.21;

B = zeros(5,6); 
B(1,:) = [cosd(45) cosd(45) cosd(45) cosd(45) 0 0];
B(2,:) = [cosd(45) -cosd(45) -cosd(45) cosd(45) 0 0];
B(3,:) = [ 0 0 0 0 1 1];
B(4,:) = [ 0 0 0 0 L_sides -L_sides];
B(5,:) = [L_diags -L_diags L_diags -L_diags 0 0];
B_inv = pinv(B);

C = eye(5);

% Discretization
% sys = ss(A,B,C,0);
% Ts = 0.02;
% sysd = c2d(sys, Ts);
% 
% Ad = sysd.A;
% Bd = sysd.B;
% Cd = sysd.C;
% 
% % Set A, B, C to Discrete version (For Simulink)
% A = Ad
% B = Bd
% C = Cd

%% Servo Control 
% Aaug = [A zeros(size(A,1),6);
%     -C zeros(size(C,1))]
% Baug = [B; zeros(size(B,2))]
% polevec = [-1 -2 -3 -4 -8 -10 -20 -30 -40 -50 -60 -55 -65 -75]; % [INPUT 1-BY-14 VECTOR HERE];
% Kaug = place(Aaug,Baug,polevec);
% K = Kaug(1:8); Ka = Kaug(9:14);
% return; % Comment out if using LQR
return; % Comment out if using LQR


%% Servo Control with Reduced States

regen_gain = true;

if(regen_gain)
    Aaug = [A zeros(size(A,1),5);
        -C zeros(size(C,1))]
    Baug = zeros(10,6);
    Baug(1:5,:) = B;
    Baug(6:10,:) = 0;
    
    % Continuous Case
    pole_offset = -13;
    polevec = [ pole_offset  , pole_offset-1, pole_offset-2, pole_offset-3, pole_offset-4, ...
                pole_offset-6, pole_offset-7, pole_offset-8, pole_offset-9, pole_offset-10 ]; % [INPUT 1-BY-14 VECTOR HERE];


    % Discrete Case
%     pole_offset = -0.05;
%     polevec = [ pole_offset  , pole_offset-0.001, pole_offset-0.002, pole_offset-0.003, pole_offset-0.004, ...
%                 pole_offset-0.006, pole_offset-0.007, pole_offset-0.008, pole_offset-0.009, pole_offset-0.010 ]; % [INPUT 1-BY-14 VECTOR HERE];

    Kaug = place(Aaug,Baug,polevec);
    K = Kaug(:,1:5); Ka = Kaug(:,6:10); % dimensions of K and Ka?
    sim('Servo_Control');
end

% Plot States
close all;
figure(); hold on;
plot(simout.Time, simout.Data(:,1));
plot(simout.Time, simout.Data(:,2));
plot(simout.Time, simout.Data(:,3));
plot(simout.Time, simout.Data(:,4));
plot(simout.Time, simout.Data(:,5));
xlabel('Time');
legend('vx','vy','vz','vtx','vtz');
stepdata = stepinfo(simout.Data(:,1), simout.Time)

% Plot Thruster Inputs % u = [sf, pf, sb, pb, l, r]
figure(); hold on;
plot(inputout.Time, inputout.Data(:,1));
plot(inputout.Time, inputout.Data(:,2));
plot(inputout.Time, inputout.Data(:,3));
plot(inputout.Time, inputout.Data(:,4));
plot(inputout.Time, inputout.Data(:,5));
plot(inputout.Time, inputout.Data(:,6));
title('Motor Thrusts');
ylabel('Force (N)');
xlabel('Time (s)');
legend('sf','pf','sb', 'pb', 'L', 'R');

return

%% LQI
% %Q = diag([9 3 3 3 3 3 3 3 3 3 3 3 2 3 9 3 3 3]*1);         % increases penalty as value increases, reaches target position/velocity quicker
% %Q = diag([3 3 3 3 3 3 3 3 3 3 3 3 94 3 3 3 3 84]*1);  
% %Q = diag([3 3 3 3 3 3 3 3 3 3 3 3 46 72 8 3 3 30]*1);  
% Q = diag([3 3 3 3 3 3 3 3 3 3 3 3 44]*1);  
% R = diag([1 1 1 1 1 1]);                                 % As value cost of input reduces. e.g if 1st coefficient increases, the T100 magnitude reduces
% %Q = 3*eye(18);
% %R = eye(4);
% N = eye(13,6)*1;
% %N = zeros(18,4); %default lqi
% % eig([Q N;N' R])
% sys = ss(A,B,C,D);
% Ts = 0.04;
% sys = c2d(sys,Ts);
% 
% % Minimum realization
% [Gmin,P] = minreal(sys);
% rank(ctrb(Gmin.a, Gmin.b))
% rank(obsv(Gmin.a, Gmin.c))
% sys_min = ss(Gmin.a,Gmin.b,Gmin.c,Gmin.d);
% sys_min = c2d(sys_min,Ts);
% 
% % Lyapunov Equation
% % lyapunov = dlyap(Gmin.a,Gmin.b,Gmin.c)
% 
% %A = (A -B*k);
% eig([Q N;N' R])
% [K,S,E] = lqi(sys_min,Q,R,N); %lqr(A,B,Q,R);


%% LQR
Q = diag([3 3 3 3 3 3 3 3]*1);  
% Q = zeros(14,14);
R = diag([1 1 1 1 1 1]);                                 % As value cost of input reduces. e.g if 1st coefficient increases, the T100 magnitude reduces

%Q = 3*eye(18);
%R = eye(4);
N = eye(13,6)*1;
%N = zeros(18,4); %default lqi
% eig([Q N;N' R])
sys = ss(A,B,C,D);
Ts = 0.04;
sys = c2d(sys,Ts);
[K,S,E] = lqr(A, B, Q,R);


%  stepinput = [0 0 0 0 0 1];
%  sim('model_sim')
 
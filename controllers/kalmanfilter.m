clear all
format compact
close all

% System Parameters
g = 9.81; %m/s^2
s = tf('s');

filename = 'parameters2.xlsx';
rho = 1000;
g = -9.81; %m/s^2
sheet = 1;
xlRange = 'D1:D45';
parameters = xlsread(filename,sheet,xlRange);


m = parameters(1); % sub mass measured using hanging scale in air
W = m*g;
COM = [parameters(6),parameters(7),parameters(8)];
I_x = parameters(2); %inertia about the COM + parallel axis theorem to take all inertias about geometric center
I_y = parameters(3);
I_z = parameters(4);

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

Ma_x = parameters(43);
Ma_y = parameters(44);
Ma_z = parameters(45);

%% A,B,C,D Matrix Creation
% Load friction coefficients (Linearized around 0.3m/s, 10deg/s)
B_x = -60.0646;
B_y = -51.4433;
B_z = -64.8841;
B_tx = -37.7409;
B_tz = -29.9229;

A = zeros(5,5);
A(1,1) = B_x/(m+Ma_x);
A(2,2) = B_y/(m+Ma_y);
A(3,3) = B_z/(m+Ma_z);
A(4,4) = B_tx/(I_x);
A(5,5) = B_tz/(I_z);

L_diags = 0.48; 
L_sides = 0.21;

% u = [sf, pf, sb, pb, l, r]
B = zeros(5,6); 
B(1,:) = [cosd(45)/(m+Ma_x) cosd(45)/(m+Ma_x) cosd(45)/(m+Ma_x) cosd(45)/(m+Ma_x) 0 0];
B(2,:) = [cosd(45)/(m+Ma_y) -cosd(45)/(m+Ma_y) -cosd(45)/(m+Ma_y) cosd(45)/(m+Ma_y) 0 0];
B(3,:) = [ 0 0 0 0 1/(m+Ma_z) 1/(m+Ma_z)];
B(4,:) = [ 0 0 0 0 L_sides/I_x -L_sides/I_x];
B(5,:) = [L_diags/I_z -L_diags/I_z L_diags/I_z -L_diags/I_z 0 0];
B_inv = pinv(B);

C = [ A(1:3,1:3), zeros(3,2); zeros(2,3), eye(2,2)];
D = [ B(1:3,:) ; zeros(2,6) ];

rank(obsv(A,C))
rank(ctrb(A,B))
eig(A)
sys = ss(A,B,C,D);
%%
%
%   Discretization
%
T = 0.2; % sampling time
sysd = c2d(sys,T);
Ad = sysd.a; 
Bd = sysd.b(:,1); Bwd = sysd.b(:,2);
Cd = sysd.c; Dd = sysd.d;

%
%   Covariances
%
Rv = diag([0.0025, 0.0025, 0.0025, 0.0025, 0.0025]) % measurement noise
Rw = diag([0.001, 0.001, 0.001, 0.001, 0.001])
%%

%
%   Design of time-varying Kalman filter
%
kfinal = 100; % number of iteration
% kfinal = length(w0);
M(:,:,1) = eye(5); % Error covariance initial value
for k = 1:kfinal
    %
    %   (TASK) WRITE CODE FOR COMPUTING TIME-VARYING 
    %   KALMAN GAIN HERE!!!
    %
    P(:,:,k) = M(:,:,k) - M(:,:,k)*Cd'/(Cd*M(:,:,k)*Cd'+Rv)*Cd*M(:,:,k);
    %%% WRITE HERE!!!
    M(:,:,k+1) = Ad*P(:,:,k)*Ad' + Rw;
    %%% WRITE HERE!!!
    
    %
    %   Kalman gain
    %
    K(:,:,k) = Ad*P(:,:,k)*Cd'/Rv;
    %%% WRITE HERE!!!
end


figure(); hold on;
plot(squeeze(P(1,1,:)));
plot(squeeze(P(1,2,:)));
plot(squeeze(P(2,1,:)));
plot(squeeze(P(2,2,:)));
legend('P(1,1)', 'P(1,2)', 'P(2,1)', 'P(2,2)');
title('Time-Varying P');
xlabel('Time (k)');

%
%   Design of steady-state Kalman filter
%

% Attempt #1
M_ss = dare(Ad', Cd', Rw, Rv);
Pss = M_ss - M_ss*C'/(Cd*M_ss*Cd'+Rv)*Cd*M_ss
P(:,:,kfinal)
K_ss = Pss*Cd'/Rv;

% Attempt #2
% [Kss, Mss, Pss] = dlqe(Ad, Bw, Cd, Rw, Rv);

%
% %   (TASK) Plot of Kalman gains HERE!!!
figure();
subplot(2,1,1); hold on;
line([0,kfinal],[K_ss(1),K_ss(1)], 'LineStyle', '--');
plot(squeeze(K(1,1,:)));
legend('Steady State', 'Time-Varying');
title('K(1)');

subplot(2,1,2); hold on;
line([0,kfinal],[K_ss(2),K_ss(2)], 'LineStyle', '--');
plot(squeeze(K(2,1,:)));
xlabel('Index k');
title('K(2)');
%legend('Steady State', 'Time-Varying');

% %   Input-output data for the state estimator design
% %
kfinal = 40;
u1 = [ones(1,10) zeros(1,10) ...
    ones(1,10) zeros(1,10)]; % square-wave input
u = [u1' u1' u1' u1' zeros(1,40)' zeros(1,40)'];
w = sqrt(Rw(1,1)).*randn(size(u)); % disturbance input
[y0,Ttmp,X] = lsim(sysd,[u;w]); % output without measurement noise
v = sqrt(Rv(1,1)).*randn(size(y0)); % measurement noise
y = y0+v; % measurement with noise

%   State estimate with time-varying Kalman filter
%
% xhat(0|-1) (time update)
xhat_t = [];
xhat_t(:,:,1) = [0.1 0.1 0 0 0]'; % initial estimate

for k = 1:kfinal
    % xhat(k|k) (measurement update)
    xhat_m(:,:,k) = xhat_t(:,:,k) + P(:,:,k)*Cd'/Rv*(y(k)'-Cd*xhat_t(:,:,k));%%% (TASK) WRITE HERE!!!
    
    % xhat(k+1|k) (time update)
    xhat_t(:,:,k+1) = Ad*xhat_m(:,:,k) + Bd*u(k)'; %%% (TASK) WRITE HERE!!!
end

%   State estimate with steady-state Kalman filter
%
% xhat(0|-1) (time update)
xhat_t_ss = [];
xhat_t_ss(:,:,1) = [0.1 0.1 0 0 0]'; % initial estimate
for k = 1:kfinal
    % xhat(k|k) (measurement update)
    xhat_m_ss(:,:,k) = xhat_t_ss(:,:,k) + Pss*Cd'/Rv*(y(k)'-Cd*xhat_t_ss(:,:,k));%%% (TASK) WRITE HERE!!!
    %%% (TASK) WRITE HERE!!!
    % xhat(k+1|k) (time update)
    xhat_t_ss(:,:,k+1) = Ad*xhat_m_ss(:,:,k) + Bd*u(k)';%%% (TASK) WRITE HERE!!!
end


% %
% %   (TASK) PLOT BOTH ESTIMATES xhat_m & xhat_m_ss
% %   AND COMPARE THEM WITH THE ACTUAL STATE X 
% %   (WHICH IS COMPUTED ABOVE BY lsim.m)!!!
% %

figure();
subplot(2,1,1); hold on;
plot(X(1:kfinal,1));
plot(squeeze(xhat_m(1,1,:)));
plot(squeeze(xhat_m_ss(1,1,:)));
legend('Lsim', 'Time-Varying', 'Steady-State');
title('X(1)');

subplot(2,1,2); hold on;
plot(X(1:kfinal,2));
plot(squeeze(xhat_m(2,1,:)));
plot(squeeze(xhat_m_ss(2,1,:)));
xlabel('Index k');
title('X(2)');
%legend('Lsim', 'Time-Varying', 'Steady-State');
clear all
format compact
close all

% Kalman Filter

load('workspace_var');

% Continuous System
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
sys = ss(A,B,C,0);
Ts = 0.02;
sysd = c2d(sys, Ts);
Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;
Dd = sysd.d;

%
%   Covariances
%
Rw = 0.1*eye(5,5);
Rv = (0.5^2)*eye(5,5);

%
%   Design of time-varying Kalman filter
%
kfinal = 1000; % number of iteration
% kfinal = length(w0);
M = [];
M(:,:,1) = eye(5); % Error covariance initial value
P = [];
K = [];
for k = 1:kfinal
    P(:,:,k) = M(:,:,k) - M(:,:,k)*Cd'/(Cd*M(:,:,k)*Cd'+Rv)*Cd*M(:,:,k);
    M(:,:,k+1) = Ad*P(:,:,k)*Ad' + Rw;
    K(:,:,k) = Ad*P(:,:,k)*Cd'/Rv;
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
M_ss = dare(Ad', Cd', zeros(5,5), Rv);
Pss = M_ss - M_ss*Cd'/(Cd*M_ss*Cd'+Rv)*Cd*M_ss
P(:,:,kfinal)
K_ss = Pss*Cd'/Rv;

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

%   Input-output data for the state estimator design
%
kfinal = 40;
u1 = [10*ones(1,10) zeros(1,10) ...
    10*ones(1,10) zeros(1,10)]; % square-wave input
u = [u1;u1;u1;u1;zeros(1,40);zeros(1,40)];
u = u';
x0 = [1, 1, 0, 0, 0]'
t = (0:Ts:39*Ts);
[y0,Ttmp,X] = lsim(sysd,u,t,x0); % output without measurement noise
v = sqrt(Rv)*(randn(size(y0))'); % measurement noise
y = y0+(v'); % measurement with noise

%   State estimate with time-varying Kalman filter
%
% xhat(0|-1) (time update)
xhat_t = [];
xhat_t(:,:,1) = [1 1 0 0 0]'; % initial estimate

xhat_m = [];
for k = 1:kfinal
    % xhat(k|k) (measurement update)
    xhat_m(:,:,k) = xhat_t(:,:,k) + P(:,:,k)*Cd'/Rv*((y(k,:))'-Cd*xhat_t(:,:,k));%%% (TASK) WRITE HERE!!!
    
    % xhat(k+1|k) (time update)
    xhat_t(:,:,k+1) = Ad*xhat_m(:,:,k) + Bd*(u(k,:)'); %%% (TASK) WRITE HERE!!!
end

%   State estimate with steady-state Kalman filter
%
% xhat(0|-1) (time update)
xhat_t_ss = [];
xhat_t_ss(:,:,1) = [1 1 0 0 0]'; % initial estimate

xhat_m_ss = [];
for k = 1:kfinal
    % xhat(k|k) (measurement update)
    xhat_m_ss(:,:,k) = xhat_t_ss(:,:,k) + Pss*Cd'/Rv*((y(k)')-Cd*xhat_t_ss(:,:,k));%%% (TASK) WRITE HERE!!!
    
    % xhat(k+1|k) (time update)
    xhat_t_ss(:,:,k+1) = Ad*xhat_m_ss(:,:,k) + Bd*(u(k,:)');%%% (TASK) WRITE HERE!!!
end

figure();
subplot(5,1,1); hold on;
plot(X(1:kfinal,1));
plot(squeeze(xhat_m(1,1,:)));
plot(squeeze(xhat_m_ss(1,1,:)));
legend('Lsim', 'Time-Varying', 'Steady-State');
title('X(1)');

subplot(5,1,2); hold on;
plot(X(1:kfinal,2));
plot(squeeze(xhat_m(2,1,:)));
plot(squeeze(xhat_m_ss(2,1,:)));
xlabel('Index k');
title('X(2)');

subplot(5,1,3); hold on;
plot(X(1:kfinal,3));
plot(squeeze(xhat_m(3,1,:)));
plot(squeeze(xhat_m_ss(3,1,:)));
legend('Lsim', 'Time-Varying', 'Steady-State');
title('X(3)');

subplot(5,1,4); hold on;
plot(X(1:kfinal,4));
plot(squeeze(xhat_m(4,1,:)));
plot(squeeze(xhat_m_ss(4,1,:)));
xlabel('Index k');
title('X(4)');

subplot(5,1,5); hold on;
plot(X(1:kfinal,5));
plot(squeeze(xhat_m(5,1,:)));
plot(squeeze(xhat_m_ss(5,1,:)));
xlabel('Index k');
title('X(5)');
legend('Lsim', 'Time-Varying', 'Steady-State');
legend('Lsim', 'Time-Varying', 'Steady-State');
function [state, du, F_dot, G, thrust_allocation] = symbolic_state_space()
% This function creates a fully symbolic non-linear state space model. The
% model is used to simulate the dynamics of the sub

% Parameters
syms x y z roll pitch yaw u v w p q r du0 du1 du2 du3 du4 du5 % state and control input

syms mass Ix Iy Iz Ixy Ixz Iyz mzg  % M_RB matrix 

syms Xu_dot Yv_dot Zw_dot Kp_dot Mq_dot Nr_dot Xq_dot Yp_dot %M_A matrix

syms Xu Xuu Yv Yvv Zw Zww Kp Kpp Nr Nrr       % Damping matrices

syms gx gy gz bx by bz gravity radius water_density  % G Matrix

pos = [x y z roll pitch yaw];
vel = [u v w p q r];

state = sym(zeros(12,1));
state(1:6) = transpose(pos);
state(7:12) = transpose(vel);

du = [du0 du1 du2 du3 du4 du5];

gravity_center = [gx gy gz];
bouoyancy_center = [bx by bz];

Mrb = sym([mass, 0.0,  0.0,  0.0,  mzg, 0.0;
           0.0,  mass, 0.0,  -mzg, 0.0, 0.0;
           0.0,  0.0,  mass, 0.0,  0.0, 0.0;
           0.0,  -mzg, 0.0,  Ix,   0.0, Ixz;
           mzg,  0.0,  0.0,  0.0,  Iy,  0.0;
           0.0,  0.0,  0.0,  Ixz,  0.0, Iz])

% Added mass matrix
Ma = sym([Xu_dot,   0.0,    0.0,    0.0,    Xq_dot, 0.0;
            0.0,    Yv_dot, 0.0,    Yp_dot, 0.0,    0.0;
            0.0,    0.0,    Zw_dot, 0.0,    0.0,    0.0;
            0.0,    Yp_dot, 0.0,    Kp_dot, 0.0,    0.0;
            Xq_dot, 0.0,    0.0,    0.0,    Mq_dot, 0.0;
            0.0,    0.0,    0.0,    0.0,    0.0,    Nr_dot]);

linear_damping = sym([  -Xu, 0.0, 0.0, 0.0, 0.0, 0.0;
                         0.0, -Yv, 0.0, 0.0, 0.0, 0.0;
                         0.0, 0.0, -Zw, 0.0, 0.0, 0.0;
                         0.0, 0.0, 0.0, -Kp, 0.0, 0.0;
                         0.0, 0.0, 0.0, 0.0, -Mq, 0.0;
                         0.0, 0.0, 0.0, 0.0, 0.0, -Nr]);
        
quadratic_damping = sym([-Xuu, 0.0,  0.0,  0.0,  0.0,  0.0;
                          0.0, -Yvv, 0.0,  0.0,  0.0,  0.0;
                          0.0, 0.0,  -Zww, 0.0,  0.0,  0.0;
                          0.0, 0.0,  0.0,  -Kpp, 0.0,  0.0;
                          0.0, 0.0,  0.0,  0.0,  -Mqq, 0.0;
                          0.0, 0.0,  0.0,  0.0,  0.0,  -Nrr]);

thruster_position = [ 0.0, 0.0, 0.0;    % thruster 0
                      0.0, 0.0, 0.0;    % thruster 1   
                      0.0, 0.0, 0.0;    % thruster 2  
                      0.0, 0.0, 0.0;    % thruster 3  
                      0.0, 0.0, 0.0;    % thruster 4  
                      0.0, 0.0, 0.0 ];  % thruster 5  

thruster_direction = [0, 0, 0;      % thruster 0
                      0, 0, 0;      % thruster 1
                      0, 0, 0;      % thruster 2
                      0, 0, 0;      % thruster 3
                      0, 0, 0;      % thruster 4    
                      0, 0, 0 ];    % thruster 5

                  
% Dynamics
M = sym(Mrb + Ma);
C = sym(coriolisMatrix(M, state));
D = sym(linear_damping + quadratic_damping);
G = sym(gravityMatrix(state, mass, gravity, radius, water_density, gravity_center, bouyancy center));

% non-linear dynamics function f
f1 = sym(zeros(12,12));
f1(1:6, 7:12) = J(state);
f1(7:12,7:12) = -inv(M)*(C-D);

f2 = sym(zeros(12,1))
f2(7:12,1) = -inv(M)*G;

f = f1*state + f2;


%% Control

% thrust allocation matrix
thrust_allocation = zeros(6,6);
thrust_allocation(1:6, 1:3) = thrust_direction;

for i = 1:6
    thrust_allocation(i,4:6) = cross(thruster_position(i, 1:3), thrust_direction(i, 1:3));
end

thrust_allocation = transpose(thrust_allocation);

% control input u
u_control = sym(zeros(1,6));

for i = 1:6:
    u_control(i) = du(i)*abs(du(i));
end

% generalized force tau
tau = thrust_allocataion*transpose(u_control);  

% control function g
g = sym(zeros(12,1));
g(7:12, 1) = M\tau;


%% State space

% non-linear state space model F_dot
F_dot = sym(zeros(12,1));

for i = 1:length(f)
    F_dot(i,1) = f(i,1) + g(i,1);
end

end

%% Sub-functions

function [ret] = s(vec)
% Creates the 3x3 anti-symmetry matrix from a 3 element input vector (Handbook of Marine Craft, pg 20)
    ret = [0.0 -vec(3), vec(2);
           vec(3), 0.0, -vec(1);
           -vec(2), vec(1), 0.0];

end

function [C] = coriolisMatrix
% Creates the coriolis matrix (Handbook of Marine Craft, pg 53)
    v1 = state(7:9)
    v2 = state(10:12)
    
    M11 = M(1:3, 1:3)
    M12 = M(1:3, 4:6)
    M21 = M(4:6, 1:3)
    M22 = M(4:6, 4:6)
    
    s1 = s(M11*v1 + M12*v2)
    s2 = s(M21*v1 + M   22*v2)
    
    C(1:3, 4:6) = -s1
    C(4:6, 1:3) = -s1
    C(4:6, 4:6) = -s2

end


function [G] = gravityMatrix(state, mass, gravity, radius, water_density, gravity_center, buoyancy_center)
% Creates the gravity matrix defined on page 60 of handbook of marine craft

    % theta: pitch angle; phi: roll angle; psi: yaw angle
    [phi, theta, psi] = deal(state(4), state(5), state(6));

    W = mass*gravity;   % Weight of the sub

    pi = 3.14159265359

    B = displaced_water_volume*water_density*gravity; % Bouyancy force acting on the sub

    % position of CG (center of gravity) with respect to CO (IMU mount
    % location)
    xg = gravity_center(1);
    yg = gravity_center(2);
    zg = gravity_center(3);

    % position of CB (center of buoyancy) with respect to CO
    xb = buoyancy_center(1);
    yb = buoyancy_center(2);
    zb = buoyancy_center(3);

    % Gravity restoring force matrix
    G = [ (W-B) * sin(theta);
          -(W-B)*cos(theta)*sin(phi);
          -(W-B)*cos(theta)*cos(phi);
          -(yg*W - yb*B)*cos(theta)*cos(phi) + (zg*W - zb*B)*cos(theta)*sin(phi);
          (zg*W - zb*B)*sin(theta) + (xg*W - xb*B) * cos(theta)*cos(phi);
          -(xg*W - xb*B)*cos(theta)*sin(phi) - (yg*W - yb*B)*sin(theta)];  

end
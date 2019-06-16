function [df_dstate, df_dstate_sym, df_dcontrol, G, thrust_allocation] = lqr_config()
% This function generates the LQR cost matrix and populates the symbolic
% state space model with the robot's actual parameters

syms x y z roll pitch yaw u v w p q r du0 du1 du2 du3 du4 du5 radius

% set up symbolic linear state-space model
[state_symbolic, du_symbolic, F_dot, G_sym, thrust_allocation] = symbolic_state_space();

% Gravity matrix parameters
displaced_water_volume = 1;
gz = 1;
water_density = 1000;   % [kg/m^3]
gx = 0;
gy = 0;
gz = 0;
bx = 0;
by = 0;
bz = 0;
gravity = 9.81;         % [m/s^2]

% Mass matrix parameters
mass = 1;
Ix = 1;
Iy = 1;
Iz = 1; 
Ixy = 0; 
Ixz = 1; % not equal to zero
Iyz = 0;
mzg = mass*abs(gz);

% Added mass matrix parameters
added_mass = water_density * displaced_water_volume;
mass_ratio = added_mass/mass;
Xu_dot = mass_ratio*mass;
Yv_dot = mass_ratio*mass;
Zw_dot = mass_ratio*mass;
Kp_dot = mass_ratio*Ix;
Mq_dot = mass_ratio*Iy;
Nr_dot = mass_ratio*Iz;
Xq_dot = mass_ratio*mzg;
Yp_dot = mass_ratio*mzg;

% Linear damping
Xu = 1;
Yv = 1;
Zw = 1;
Kp = 1;
Mq = 1;
Nr = 1;

% Quadratic Damping
Xuu = 1;
Yvv = 1;
Zww = 1;
Kpp = 1;
Mqq = 1;
Nrr = 1;

% Substitute constant parameters
state_dot = subs(F_dot);
G = subs(G_sym);

% the system is linearized via the Jacobian
df_dstate_sym(x, y, z, roll, pitch, yaw, u, v, w, p, q, r, radius) = jacobian(state_dot, state_symbolic);
df_dstate = matlabFunction(df_dstate_sym);
df_dcontrol_sym(du0, du1, du2, du3, du4, du5) = jacobian(state_dot, transpose(du_symbolic));
df_dcontrol = matlabFunction(df_dcontrol_sym);
df_dcontrol = df_dcontrol(1,1,1,1,1,1);

% Gravity matrix G
G(roll, pitch, radius) = G;
G = matlabFunction(G)

end

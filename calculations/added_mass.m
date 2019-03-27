%% Fluid Properties (At T=22 degC)
rho = 0.9978e3; % density
mu = 0.9544e-3; % dynamic viscosity
v = 0.3;
K_0 = 0.3;
K_90 = 0.8;

%% Main Cylinder
D = 0.22;
L = 0.48;
V = pi/4*(D^2)*L;
ma_x1 = K_0*V*rho;
ma_y1 = K_90*V*rho;
ma_z1 = K_90*V*rho;

%% Front Cylinder
D = 0.09;
L = 0.31;
V = pi/4*(D^2)*L;
ma_x2 = K_90*V*rho;
ma_y2 = K_0*V*rho;
ma_z2 = K_90*V*rho;

%% Back Cylinder
D = 0.17;
L = 0.32;
V = pi/4*(D^2)*L;
ma_x3 = K_90*V*rho;
ma_y3 = K_0*V*rho;
ma_z3 = K_90*V*rho;

%% Component Build-Up

ma_x = ma_x1 + ma_x2 + ma_x3;
ma_y = ma_y1 + ma_y2 + ma_y3;
ma_z = ma_z1 + ma_z2 + ma_z3;

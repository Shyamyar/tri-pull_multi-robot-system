function y=Multirobot_static2(del_p1,del_p2,del_p3,X0)

% This defines the static model for the multirobot system based on the statics.
warning('off')
rotation_matrices;

% Naming variables
rho_DT1 = X0(1);
rho_DT2 = X0(2);
rho_DT3 = X0(3);
phi_DT1 = X0(4);
phi_DT2 = X0(5);
phi_DT3 = X0(6);
rho_DT = X0(7);
rho_DT0 = X0(8);
phi_DT = X0(9);
r_D = X0(10:11)';
r_T = X0(12:13)';
p1 = X0(14);
p2 = X0(15);
p3 = X0(16);

% Other initial values
% Relative positions
rel_vec = relative_vec(r_D,r_T);
r_1D_u = rel_vec(10:11)';
r_2D_u = rel_vec(12:13)';
r_3D_u = rel_vec(13:14)';

% Changing String Lengths
p1 = p1 + del_p1; % Adding additive change to the p1
p2 = p2 + del_p2; % Adding additive change to the p1
p3 = p3 + del_p3; % Adding additive change to the p1

% First States
r_D = (-1/3)*((p1-l0)*r_1D_u + (p2-l0)*r_2D_u + (p3-l0)*r_3D_u);

% New positions
rel_vec_new = relative_vec(r_D,r_T);

% Return output
y = [rel_vec_new(1:9),r_D',r_T',p1,p2,p3];
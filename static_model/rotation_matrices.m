theta_O1 = deg2rad(-90);
theta_O2 = deg2rad(30);
theta_O3 = deg2rad(150);

R = @(theta) [cos(theta) -sin(theta);
    sin(theta) cos(theta)];

R_O1 = R(theta_O1)';
R_O2 = R(theta_O2)';
R_O3 = R(theta_O3)';

% Robot positions
r_1 = [0;0.5]; % Robot 1 position
r_2 = [0.5*cos(deg2rad(30));-0.5*sin(deg2rad(30))]; % Robot 2 position
r_3 = [-0.5*cos(deg2rad(30));-0.5*sin(deg2rad(30))]; % Robot 3 position

% Initial String Length
l0 = 1;

T_O1 = [R_O1 -1.*R_O1*r_1
        0 0 1];
T_O2 = [R_O2 -1.*R_O2*r_2;
        0 0 1];
T_O3 = [R_O3 -1.*R_O3*r_3;
        0 0 1];

T_1O = [R_O1' r_1
        0 0 1];
T_2O = [R_O2' r_2;
        0 0 1];
T_3O = [R_O3' r_3;
        0 0 1];

function cost = CostFun_dec(vec,fis1)  % This function computes the cost value for each individual.

% vec refers to the individual vector from GA's population defined by GA.
% This function is evaluated for each individual during every generation.

warning('off')
rotation_matrices;

t0=0;   % Start time of simulation
dt=0.1;
tmax=20;   % End time

% Position of Robots
global r_1 r_2 r_3 l0 r_D r_1D r_2D r_3D r_1D_u r_2D_u r_3D_u
r_1 = [0;0.5]; % Robot 1 position
r_2 = [0.5*cos(deg2rad(30));-0.5*sin(deg2rad(30))]; % Robot 2 position
r_3 = [-0.5*cos(deg2rad(30));-0.5*sin(deg2rad(30))]; % Robot 3 position

% Initial String Length
l0 = 1;

% Initial Conditions
r_D = [0;0]; % First Disk position

% Desired
% global r_T
% r_T = [0.0;0.4];  % Target position
% rho_des = 0.02; % Desired vicinity of 2cm

% global r_T MAXGEN gen
% g = MAXGEN/3;
% r_T_i = [ones(2,g).*[0.3;-0.22],ones(2,g).*[0.15;0.15],ones(2,g).*[0.0;0.4]];  % Target positions
% r_T = r_T_i(:,1);
% rho_des = 0.02; % Desired vicinity of 2cm

global r_T
theta = pi/3;
radius = 0.23;
r_T_i = [radius.*cos(theta);radius.*sin(theta)];  % Target positions
r_T = r_T_i(:,1);
rho_des = 0.02; % Desired vicinity of 2cm

% global r_T
% theta = linspace(0,2*pi,tmax/dt);
% radius = linspace(0.03,0.24,tmax/dt);
% r_T_i = [radius.*cos(theta);radius.*sin(theta)];  % Target positions
% r_T = r_T_i(:,1);
% rho_des = 0.02; % Desired vicinity of 2cm

% Relative positions
r_1D = r_D - r_1; % Relative position of disk w.r.t robot 1
r_2D = r_D - r_2; % Relative position of disk w.r.t robot 2
r_3D = r_D - r_3; % Relative position of disk w.r.t robot 3
r_1T = r_T - r_1; % Relative position of target w.r.t robot 1
r_2T = r_T - r_2; % Relative position of target w.r.t robot 2
r_3T = r_T - r_3; % Relative position of target w.r.t robot 3

r_1D_u = r_1D / norm(r_1D); % Unit vector along r_1D
r_2D_u = r_2D / norm(r_2D); % Unit vector along r_2D
r_3D_u = r_3D / norm(r_3D); % Unit vector along r_3D

r_1T_u = r_1T / norm(r_1T); % Unit vector along r_1T
r_2T_u = r_2T / norm(r_2T); % Unit vector along r_2T
r_3T_u = r_3T / norm(r_3T); % Unit vector along r_3T

r_D_u = r_D / norm(r_D); % Unit vector along r_D
r_T_u = r_T / norm(r_T); % Unit vector along r_T

global p1 p2 p3;
[p1,p2,p3] = deal(1); %1m of string is already reeled around the spool at the t = 0 if at origin
p1 = p1+norm(r_1)-norm(r_1D); %1m of string is already reeled around the spool at the t = 0 if at origin
p2 = p2+norm(r_2)-norm(r_2D); %1m of string is already reeled around the spool at the t = 0 if at origin
p3 = p3+norm(r_3)-norm(r_3D); %1m of string is already reeled around the spool at the t = 0 if at origin

% Radial Distances
rho_iDT = norm(r_D - r_T); % Distance between ball and target 
rho_DT0 = norm(r_T) - norm(r_D); % Radial distance between ball and target as viewed from Origin
rho_DT1 = norm(r_1D) - norm(r_1T); % Radial distance between ball and target as viewed from Robot1
rho_DT2 = norm(r_2D) - norm(r_2T); % Radial distance between ball and target as viewed from Robot2
rho_DT3 = norm(r_3D) - norm(r_3T); % Radial distance between ball and target as viewed from Robot3

% Angles
% r_1D_1k = T_O1*[r_1D;1]; r_1D_1 = r_1D_1k(1:2);
% r_2D_2k = T_O2*[r_2D;1]; r_2D_2 = r_2D_2k(1:2);
% r_3D_3k = T_O3*[r_3D;1]; r_3D_3 = r_3D_3k(1:2);
% 
% r_1T_1k = T_O1*[r_1T;1]; r_1T_1 = r_1T_1k(1:2);
% r_2T_2k = T_O2*[r_2T;1]; r_2T_2 = r_2T_2k(1:2);
% r_3T_3k = T_O3*[r_3T;1]; r_3T_3 = r_3T_3k(1:2);

% if atan2(r_1D_1(2),r_1D_1(1))>atan2(r_1T_1(2),r_1T_1(1))
%     phi_DT1 = -acos(dot(r_1D_u,r_1T_u)); % Angle between r_1D and r_1T
% else
%     phi_DT1 = acos(dot(r_1D_u,r_1T_u)); % Angle between r_1D and r_1T
% end
% if atan2(r_2D_2(2),r_2D_2(1))>atan2(r_2T_2(2),r_2T_2(1))
%     phi_DT2 = -acos(dot(r_2D_u,r_2T_u)); % Angle between r_2D and r_2T
% else
%     phi_DT2 = acos(dot(r_2D_u,r_2T_u)); % Angle between r_2D and r_2T
% end
% if atan2(r_3D_3(2),r_3D_3(1))>atan2(r_3T_3(2),r_3T_3(1))
%     phi_DT3 = -acos(dot(r_3D_u,r_3T_u)); % Angle between r_3D and r_3T
% else
%     phi_DT3 = acos(dot(r_3D_u,r_3T_u)); % Angle between r_3D and r_3T
% end

c_1 = cross([r_1D_u;0],[r_1T_u;0]);
c_2 = cross([r_2D_u;0],[r_2T_u;0]);
c_3 = cross([r_3D_u;0],[r_3T_u;0]);
c_4 = cross([r_D_u;0],[r_T_u;0]);

if c_1(end)<0
    phi_DT1 = -acos(dot(r_1D_u,r_1T_u)); % Angle between r_1D and r_1T
else
    phi_DT1 = acos(dot(r_1D_u,r_1T_u)); % Angle between r_1D and r_1T
end
if c_2(end)<0
    phi_DT2 = -acos(dot(r_2D_u,r_2T_u)); % Angle between r_2D and r_2T
else
    phi_DT2 = acos(dot(r_2D_u,r_2T_u)); % Angle between r_2D and r_2T
end
if c_3(end)<0
    phi_DT3 = -acos(dot(r_3D_u,r_3T_u)); % Angle between r_3D and r_3T
else
    phi_DT3 = acos(dot(r_3D_u,r_3T_u)); % Angle between r_3D and r_3T
end
if c_4(end)<0
    phi_DT = -acos(dot(r_D_u,r_T_u)); % Angle between r_3D and r_3T
else
    phi_DT = acos(dot(r_D_u,r_T_u)); % Angle between r_3D and r_3T
end

X0 = [rho_DT1,rho_DT2,rho_DT3,phi_DT1,phi_DT2,phi_DT3,rho_iDT,r_D',p1,p2,p3,rho_DT0,phi_DT];

vec(:,7:15) = round(vec(:,7:15));
%%% Assign GA individual to the corresponding fis paramters
fis1.input(1).mf(1).params(3) = vec(1);
fis1.input(1).mf(2).params(1) = vec(1);
fis1.input(1).mf(2).params(3) = vec(2);
fis1.input(1).mf(3).params(2) = vec(2);

fis1.input(2).mf(1).params(3) = vec(3);
fis1.input(2).mf(2).params(1) = vec(3);
fis1.input(2).mf(2).params(3) = vec(4);
fis1.input(2).mf(3).params(2) = vec(4);

fis1.output(1).mf(1).params(3) = vec(5);
fis1.output(1).mf(2).params(1) = vec(5);
fis1.output(1).mf(2).params(3) = vec(6);
fis1.output(1).mf(3).params(2) = vec(6);

fis1.rule(1).consequent = vec(7);
fis1.rule(2).consequent = vec(8);
fis1.rule(3).consequent = vec(9);
fis1.rule(4).consequent = vec(10);
fis1.rule(5).consequent = vec(11);
fis1.rule(6).consequent = vec(12);
fis1.rule(7).consequent = vec(13);
fis1.rule(8).consequent = vec(14);
fis1.rule(9).consequent = vec(15);

options = odeset('RelTol',1e-6);

Yhist = X0;
Thist = t0;
tf = dt;
% i=1;
pen=0;
while tf<tmax
    del_p1 = evalfis([Yhist(end,1),Yhist(end,4)],fis1);
    del_p2 = evalfis([Yhist(end,2),Yhist(end,5)],fis1);
    del_p3 = evalfis([Yhist(end,3),Yhist(end,6)],fis1);
    Yout = Multirobot_static(del_p1,del_p2,del_p3);
    Yhist = [Yhist;Yout];
    Thist = [Thist;tf];
%     i = i+1
%     r_T = r_T_i(:,gen+1);
    if (p1+norm(r_1D) > 2) || (p2+norm(r_2D) > 2) || (p3+norm(r_3D) > 2) || (p1+norm(r_1D) < 1) || (p2+norm(r_2D) < 1) || (p3+norm(r_3D) < 1)
        pen = 1000;
    end
    tf = tf+dt;
end
if Yout(end,7) > rho_des
    pen = 1000;
end
SI = stepinfo(Yhist(:,7),Thist);
cost = sum(abs(Yhist(:,7))./rho_iDT)+pen;%-rho_des.*ones(height(Yhist),1)))+pen;%+10*(tmax - SI.SettlingTime);  % Cost is a combination of settling time and steady state error, since we need to minimize both. The objective is get the pendulum to the inverted position quickly.
check=1;

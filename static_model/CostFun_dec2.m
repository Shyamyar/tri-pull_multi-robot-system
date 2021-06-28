function cost = CostFun_dec2(vec,fis1)  % This function computes the cost value for each individual.

% vec refers to the individual vector from GA's population defined by GA.
% This function is evaluated for each individual during every generation.

warning('off')
rotation_matrices;

t0=0;   % Start time of simulation
dt=0.1;
tmax=20;   % End time

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

theta = pi/3;
radius = 0.23;
r_T_i = [radius.*cos(theta);radius.*sin(theta)];  % Target positions
r_T = r_T_i(:,1);
rho_des = 0.02; % Desired vicinity of 2cm
rho_DT_i = norm(r_D-r_T); % Initial Seperation

%Initial spool strings
[p1,p2,p3] = deal(1); %1m of string is already reeled around the spool at the t = 0 if at origin
% p1 = p1+norm(r_1)-norm(r_1D); %1m of string is already reeled around the spool at the t = 0 if at origin
% p2 = p2+norm(r_2)-norm(r_2D); %1m of string is already reeled around the spool at the t = 0 if at origin
% p3 = p3+norm(r_3)-norm(r_3D); %1m of string is already reeled around the spool at the t = 0 if at origin

% Relative positions
rel_vec = relative_vec(r_D,r_T);

% Initial input/output vector
X0 = [rel_vec(1:9),r_D',r_T',p1,p2,p3];

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
    Yout = Multirobot_static2(del_p1,del_p2,del_p3,X0);
    Yhist = [Yhist;Yout];
    Thist = [Thist;tf];
%     i = i+1
%     r_T = r_T_i(:,gen+1);
    r_1D = Yout(10:11)' - r_1; % Relative position of disk w.r.t robot 1
    r_2D = Yout(10:11)' - r_2; % Relative position of disk w.r.t robot 2
    r_3D = Yout(10:11)' - r_3; % Relative position of disk w.r.t robot 3
    p1 = Yout(14);
    p2 = Yout(15);
    p3 = Yout(16);

    if (p1+norm(r_1D) > 2) || (p2+norm(r_2D) > 2) || (p3+norm(r_3D) > 2) || (p1+norm(r_1D) < 1) || (p2+norm(r_2D) < 1) || (p3+norm(r_3D) < 1)
        pen = 1000;
    end
    Yout=X0;
    tf = tf+dt;
end
if Yout(end,7) > rho_des
    pen = 1000;
end
SI = stepinfo(Yhist(:,7),Thist);
cost = sum(abs(Yhist(:,7))./rho_DT_i)+pen;%-rho_des.*ones(height(Yhist),1)))+pen;%+10*(tmax - SI.SettlingTime);  % Cost is a combination of settling time and steady state error, since we need to minimize both. The objective is get the pendulum to the inverted position quickly.
check=1;

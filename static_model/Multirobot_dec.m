clc;
clear;
rotation_matrices;
warning('off')
%% Initialization
%Time parameters:
t0=0;
dt=0.1;
tmax=20;

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
% r_T = [-0.34;-0.23];  % Target position
% rho_des = 0.02; % Desired vicinity of 2cm

global r_T
theta = 2*pi/3;
radius = 0.23;
r_T_i = [radius.*cos(theta);radius.*sin(theta)];
r_T = r_T_i(:,1);
rho_des = 0.02; % Desired vicinity of 2cm

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
rho_DT = norm(r_D - r_T); % Distance between ball and target 
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
% if c_4(end)<0
%     phi_DT = -acos(dot(r_D_u,r_T_u)); % Angle between r_3D and r_3T
% else
%     phi_DT = acos(dot(r_D_u,r_T_u)); % Angle between r_3D and r_3T
% end
phi_DT = atan2(r_T(2),r_T(1));

X0 = [rho_DT1,rho_DT2,rho_DT3,phi_DT1,phi_DT2,phi_DT3,rho_DT,r_D',p1,p2,p3,rho_DT0,phi_DT];

%% Run simulation:
% options = odeset('RelTol',1e-6);

% [Tout,Yout]=ode45(@InvPendulumSSModel,t0:dt:tf,X0);
tf = dt;
Thist = t0;
Yhist = X0;
chosen_fis = readfis('TrainedMultirobot21.fis');
vel = [];
breaks_at = 0;
loose_at = 0;
while tf<tmax
    del_p1 = evalfis([Yhist(end,1),Yhist(end,4)],chosen_fis);
    del_p2 = evalfis([Yhist(end,2),Yhist(end,5)],chosen_fis);
    del_p3 = evalfis([Yhist(end,3),Yhist(end,6)],chosen_fis);
    Yout = Multirobot_static(del_p1,del_p2,del_p3);
    Yhist = [Yhist;Yout];
    Thist = [Thist;tf];
    vel_dt = norm((1/dt).*([Yhist(end,8);Yhist(end,9)]-[Yhist(end-1,8);Yhist(end-1,9)]));
    vel = [vel;vel_dt];
%     if Yout(7) < rho_des
%         disp('Reached by target')
%         break;
%     end
    if (p1+norm(r_1D) > 2) || (p2+norm(r_2D) > 2) || (p3+norm(r_3D) > 2)
        disp('String Breaks')
        breaks_at = [breaks_at;1];
        break;
    end
    if (p1+norm(r_1D) < 1) || (p2+norm(r_2D) < 1) || (p3+norm(r_3D) < 1)
        disp('String Loose')
        loose_at = [loose_at;2];
        break;
    end
    tf = tf+dt;
end
fprintf('p1+r1D: %.2f \np2+r2D: %.2f \np3+r3D: %.2f\n', [p1+norm(r_1D) p2+norm(r_2D) p3+norm(r_3D)])
fprintf('p1: %.2f \np2: %.2f \np3: %.2f\n', [p1 p2 p3])
fprintf('rho_DT: %.2f\n', Yout(7))
fprintf('Time of termination: %2f sec\n', tf);
Tout = Thist;
Yout = Yhist;

%% Animation of the multirobot system
figure(1)
plot([r_1(1),r_2(1),r_3(1),r_1(1)],[r_1(2),r_2(2),r_3(2),r_1(2)],'-ks','MarkerSize',10)
hold on;
grid on;
color_map = [1,0,0;0,0,0];
scatter(r_T(1),r_T(2),'MarkerFaceColor','r','MarkerEdgeColor','r');
h = animatedline;
for k = 1:length(Thist)
    addpoints(h,Yout(k,8),Yout(k,9));
    head = scatter(Yout(k,8),Yout(k,9),'filled','MarkerFaceColor','b','MarkerEdgeColor','b');
    a = plot([r_1(1),Yout(k,8)],[r_1(2),Yout(k,9)],'--m');
    b = plot([r_2(1),Yout(k,8)],[r_2(2),Yout(k,9)],'--m');
    c = plot([r_3(1),Yout(k,8)],[r_3(2),Yout(k,9)],'--m');
    drawnow
    delete(head);
    delete(a);
    delete(b);
    delete(c);
end
a = plot([r_1(1),Yout(k,8)],[r_1(2),Yout(k,9)],'--m');
b = plot([r_2(1),Yout(k,8)],[r_2(2),Yout(k,9)],'--m');
c = plot([r_3(1),Yout(k,8)],[r_3(2),Yout(k,9)],'--m');
scatter(r_D(1),r_D(2),'MarkerFaceColor','b','MarkerEdgeColor','b');
legend('Outline','Path','Ball','Target')

%% Plot results
figure(2)
yyaxis left
plot(Yout(:,7))
hold on
yyaxis right
plot(vel)
grid on;
legend('\rho\_DT','Velocity')

figure (3)
plot(Yout(:,10))
hold on;
grid on;
plot(Yout(:,11))
plot(Yout(:,12))
legend('p1','p2','p3')

figure(4)
plot(rad2deg(Yhist(:,4)))
hold on;
plot(rad2deg(Yhist(:,5)))
plot(rad2deg(Yhist(:,6)))
grid on;
legend('phi\_DT1','phi\_DT2','phi\_DT3')

figure(5)
plot(vel)
grid on;
legend('Velocity(m/s)')
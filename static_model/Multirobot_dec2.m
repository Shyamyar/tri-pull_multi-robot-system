clc;
clear;
rotation_matrices;
warning('off')

%% Initialization
%Time parameters:
t0=0;
dt=0.1;
tmax=40;

% Initial Conditions
r_D = [0;0]; % First Disk position

% Target
% global r_T
% r_T = [-0.34;-0.23];  % Target position
% rho_des = 0.02; % Desired vicinity of 2cm

theta = 1*pi/3;
radius = -0.18;
r_T_i = [radius.*cos(theta);radius.*sin(theta)];
r_T = r_T_i(:,1);
rho_des = 0.02; % Desired vicinity of 2cm

%Initial spool strings
[p1,p2,p3] = deal(1); %1m of string is already reeled around the spool at the t = 0 if at origin
% p1 = p1+norm(r_1)-norm(r_1D); %1m of string is already reeled around the spool at the t = 0 if at origin
% p2 = p2+norm(r_2)-norm(r_2D); %1m of string is already reeled around the spool at the t = 0 if at origin
% p3 = p3+norm(r_3)-norm(r_3D); %1m of string is already reeled around the spool at the t = 0 if at origin

% Relative positions
rel_vec = relative_vec(r_D,r_T);

% Initial input/output vector
X0 = [rel_vec(1:9),r_D',r_T',p1,p2,p3];

%% Run simulation:
% options = odeset('RelTol',1e-6);

% [Tout,Yout]=ode45(@InvPendulumSSModel,t0:dt:tf,X0);
tf = dt;
Thist = t0;
Yhist = X0;
chosen_fis = readfis('TrainedMultirobot20.fis');
vel = [];
breaks_at = 0;
loose_at = 0;
while tf<tmax
    del_p1 = evalfis([Yhist(end,1),Yhist(end,4)],chosen_fis);
    del_p2 = evalfis([Yhist(end,2),Yhist(end,5)],chosen_fis);
    del_p3 = evalfis([Yhist(end,3),Yhist(end,6)],chosen_fis);
    Yout = Multirobot_static2(del_p1,del_p2,del_p3,X0);
    Yhist = [Yhist;Yout];
    Thist = [Thist;tf];
    vel_dt = norm((1/dt).*([Yhist(end,8);Yhist(end,9)]-[Yhist(end-1,8);Yhist(end-1,9)]));
    vel = [vel;vel_dt];
%     if Yout(7) < rho_des
%         disp('Reached by target')
%         break;
%     end
    r_1D = Yout(10:11)' - r_1; % Relative position of disk w.r.t robot 1
    r_2D = Yout(10:11)' - r_2; % Relative position of disk w.r.t robot 2
    r_3D = Yout(10:11)' - r_3; % Relative position of disk w.r.t robot 3
    p1 = Yout(14);
    p2 = Yout(15);
    p3 = Yout(16);

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
    X0=Yout;
    tf = tf+dt;
end
fprintf('p1+r1D: %.2f \np2+r2D: %.2f \np3+r3D: %.2f\n', [p1+norm(r_1D) p2+norm(r_2D) p3+norm(r_3D)])
fprintf('p1: %.2f \np2: %.2f \np3: %.2f\n', [Yout(14) Yout(15) Yout(16)])
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
    addpoints(h,Yout(k,10),Yout(k,11));
    head = scatter(Yout(k,10),Yout(k,11),'filled','MarkerFaceColor','b','MarkerEdgeColor','b');
    a = plot([r_1(1),Yout(k,10)],[r_1(2),Yout(k,11)],'--m');
    b = plot([r_2(1),Yout(k,10)],[r_2(2),Yout(k,11)],'--m');
    c = plot([r_3(1),Yout(k,10)],[r_3(2),Yout(k,11)],'--m');
    drawnow
    delete(head);
    delete(a);
    delete(b);
    delete(c);
end
a = plot([r_1(1),Yout(k,10)],[r_1(2),Yout(k,11)],'--m');
b = plot([r_2(1),Yout(k,10)],[r_2(2),Yout(k,11)],'--m');
c = plot([r_3(1),Yout(k,10)],[r_3(2),Yout(k,11)],'--m');
scatter(Yout(end,10),Yout(end,11),'MarkerFaceColor','b','MarkerEdgeColor','b');
legend('Outline','Path','Ball','Target')

%% Plot results
figure(2)
plot(normalize(Yout(:,7)))
hold on
plot(normalize(vel))
grid on;
legend('Normalized \rho\_DT','Normalized Velocity')

figure (3)
plot(Yout(:,13))
hold on;
grid on;
plot(Yout(:,14))
plot(Yout(:,15))
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
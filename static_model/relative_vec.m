function rel_vec=relative_vec(r_D,r_T)
rotation_matrices;

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

rel_vec = [rho_DT1,rho_DT2,rho_DT3,phi_DT1,phi_DT2,phi_DT3,rho_DT,rho_DT0,phi_DT,r_1D_u',r_2D_u',r_3D_u'];
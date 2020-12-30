function RATA_mocap = mocap_processing_fxn(mocap_ascii_filename, marker_config)
mocap_filename = mocap_ascii_filename;

mdata = readmatrix(mocap_filename, 'FileType', 'text');

%% Assign x,y position data of each marker to variables
% thigh
th1 = mdata(:,1:2);
th2 = mdata(:,3:4);
th3 = mdata(:, 5:6);

%shank
sh1 = mdata(:,7:8);
sh2 = mdata(:, 9:10);
sh3 = mdata(:, 11:12);

%% Pixel to meter conversion for thigh and shank

if marker_config == 1 %marker config for IMU config BBBB, BBCB
    %meter distances
    %thigh
    d12_th = 0.03374; %in meters between marker 1 and 2
    d13_th = 0.03905; %"" between marker 1 and 3
    d23_th = 0.03047; % "" marker 2 and 3
    
    %shank
    d12_sh = 0.026;
    d13_sh = 0.06466;
    d23_sh = 0.06055;
elseif marker_config == 2 %marker config for IMU config BBAB
     %thigh
    d12_th = 0.03374; %in meters between marker 1 and 2
    d13_th = 0.03905; %"" between marker 1 and 3
    d23_th = 0.03047; % "" marker 2 and 3
    
    d12_sh = 0.026;
    d13_sh = 0.1466;
    d23_sh = 0.14429;
elseif marker_config == 3 %marker config for soft-tissue attachements
     %thigh
     d12_th = 0.22674; % distance in meters between marker #1 and #2
     d13_th = 0.24996;
     d23_th = 0.03047;
     
     %shank
     d12_sh = 0.026;
     d13_sh = 0.1956;
     d23_sh = 0.19533;
    
else
    disp('ERROR: check marker configuration')
end

%pixel distances
%take first frame position data (assumed static)
p12_th = sqrt((th2(1,1) - th1(1,1)).^2 + (th2(1,2) - th1(1,2)).^2); % distance formula sqrt((x1-x2)^2 + (y1-y2)^2)
p13_th = sqrt((th3(1,1) - th1(1,1)).^2 + (th3(1,2) - th1(1,2)).^2);
p23_th = sqrt((th3(1,1) - th2(1,1)).^2 + (th3(1,2) - th2(1,2)).^2);

p12_sh = sqrt((sh2(1,1) - sh1(1,1)).^2 + (sh2(1,2) - sh1(1,2)).^2);
p13_sh = sqrt((sh3(1,1) - sh1(1,1)).^2 + (sh3(1,2) - sh1(1,2)).^2);
p23_sh = sqrt((sh3(1,1) - sh2(1,1)).^2 + (sh3(1,2) - sh2(1,2)).^2);


pix_dist_ratio_thigh = mean([d12_th/p12_th, d13_th/p13_th, d23_th/p23_th]);
pix_dist_ratio_shank = mean([d12_sh/p12_sh, d13_sh/p13_sh, d23_sh/p23_sh]);
%% Thigh Angle and Shank Angle (Angles of bodysegments from the vertical)

if marker_config == 1 || marker_config == 2
    % Thigh angle
    marker_offset_angle_thigh = atand(14.48/30.47); %angle in thigh frame between marker 1 and 2 for  config #1 and #2;
elseif marker_config == 3
    marker_offset_angle_thigh = atand(226.59/8.18); % angle in thigh frame between marker 1 and 2 for config #3
end


%angle offset of the markers if the thigh x,y axes were aligned with inertial ref. frame x,y axes
theta_t = atand((th2(:,1) - th1(:,1))./ (th2(:,2) - th1(:,2))) + marker_offset_angle_thigh;

%Shank angle
marker_offset_angle_shank = 0; %using proximal shank markers 1&2 for marker_config = 1, 2, or 3
theta_s = atand((sh2(:,2) - sh1(:,2)) ./ (sh2(:,1) - sh1(:,1))) - marker_offset_angle_shank;

%% Breakpoint, optimal cutoff frequency for angle data
c = 1;
for f = 1:0.5:35
    fs = 400;
    fc = f;
    [b,a] = butter(4, fc/(fs/2));
    theta_t_filt(:,c) = filtfilt(b,a,theta_t);
    theta_s_filt(:,c) = filtfilt(b,a,theta_s);
    c = c+1;
end

rmsd_t = sqrt(sum((theta_t_filt - theta_t).^2)./length(theta_t));
rmsd_s = sqrt(sum((theta_s_filt - theta_s).^2)./length(theta_t));
freq_all = 1:0.5:35;

% figure
% plot((1:0.5:35),rmsd_t);
% hold on
% plot((1:0.5:35), rmsd_s);

freq = 30:0.5:35;
p_t = polyfit(freq,rmsd_t(59:69),1);
p_s = polyfit(freq, rmsd_s(59:69),1);

[vt,ind_t] = min(abs(rmsd_t - p_t(2)));
[vs,ind_s] = min(abs(rmsd_s - p_s(2)));

fc_t = freq_all(ind_t);
fc_s = freq_all(ind_s);

%% Filter Thigh angle data

% fs = 400; %filter, sampling freq
%     fc = fc_t; %butterworth filter cutoff freq for THIGH
%     [b,a] = butter(4, fc/(fs/2));    
% theta_t_f = filtfilt(b,a,theta_t);
% 
% %% Filter Shank angle data
% fs = 400; %filter, sampling freq
%     fc = fc_s; %butterworth filter cutoff freq for SHANK
%     [b,a] = butter(4, fc/(fs/2));
% theta_s_f = filtfilt(b,a,theta_s);

%% Filter positional data + convert to meters
th1_unf = th1;
th2_unf = th2;
th3_unf = th3;
sh1_unf = sh1;
sh2_unf = sh2;
sh3_unf = sh3;



%% Breakpoint, optimal cutoff frequency for position data
c = 1;
for f = 1:0.5:50
    fs = 400;
    fc = f;
    [b,a] = butter(4, fc/(fs/2));
    th1_filt_x(:,c) = filtfilt(b,a,th1_unf(:,1));
    th1_filt_y(:,c) = filtfilt(b,a,th1_unf(:,2));
    sh1_filt_x(:,c) = filtfilt(b,a,sh1_unf(:,1));
    sh1_filt_y(:,c) = filtfilt(b,a,sh1_unf(:,2));
    c = c+1;
end

rmsd_t_x = sqrt(sum((th1_filt_x - th1_unf(:,1)).^2)./length(th1_unf(:,1)));
rmsd_t_y = sqrt(sum((th1_filt_y - th1_unf(:,2)).^2)./length(th1_unf(:,2)));
rmsd_s_x = sqrt(sum((sh1_filt_x - sh1_unf(:,1)).^2)./length(sh1_unf(:,1)));
rmsd_s_y = sqrt(sum((sh1_filt_y - sh1_unf(:,2)).^2)./length(sh1_unf(:,2)));

freq_all = 1:0.5:50;

% figure
% plot(freq_all,rmsd_t_x);
% hold on
% plot(freq_all, rmsd_t_y);
% 
% plot(freq_all,rmsd_s_x);
% plot(freq_all, rmsd_s_y);

freq_p = 40:0.5:50;
p_t_x = polyfit(freq_p,rmsd_t_x(end-20:end),1);
p_t_y = polyfit(freq_p,rmsd_t_y(end-20:end),1);

p_s_x = polyfit(freq_p,rmsd_s_x(end-20:end),1);
p_s_y = polyfit(freq_p,rmsd_s_y(end-20:end),1);

[vt_x,ind_t_x] = min(abs(rmsd_t_x - p_t_x(2)));
[vt_y,ind_t_y] = min(abs(rmsd_t_y - p_t_y(2)));

[vs_x,ind_s_x] = min(abs(rmsd_s_x - p_s_x(2)));
[vs_y,ind_s_y] = min(abs(rmsd_s_y - p_s_y(2)));

fc_t_x = freq_all(ind_t_x);
fc_t_y = freq_all(ind_t_y);

fc_s_x = freq_all(ind_s_x);
fc_s_y = freq_all(ind_s_y);

% f_all = 25;
% fc_t_x = f_all;
% fc_t_y = f_all;
% 
% fc_s_x = f_all;
% fc_s_y = f_all;
%% Filter position data
%Thigh
fs = 400; %filter, sampling freq

fc = fc_t_x; %butterworth filter cutoff freq (thigh x)
[b,a] = butter(4, fc/(fs/2));
th1_f(:,1) = filtfilt(b,a,th1(:,1));
th2_f(:,1) = filtfilt(b,a,th2(:,1));
th3_f(:,1) = filtfilt(b,a,th3(:,1));

fc = fc_t_y; %butterworth filter cutoff freq (thigh y)
[b,a] = butter(4, fc/(fs/2));
th1_f(:,2) = filtfilt(b,a,th1(:,2));
th2_f(:,2) = filtfilt(b,a,th2(:,2));
th3_f(:,2) = filtfilt(b,a,th3(:,2));

fc = fc_s_x; %butterworth filter cutoff freq (shank x)
[b,a] = butter(4, fc/(fs/2));
sh1_f(:,1) = filtfilt(b,a,sh1(:,1));
sh2_f(:,1) = filtfilt(b,a,sh2(:,1));
sh3_f(:,1) = filtfilt(b,a,sh3(:,1));

fc = fc_s_y; %butterworth filter cutoff freq (shank y)
[b,a] = butter(4, fc/(fs/2));
sh1_f(:,2) = filtfilt(b,a,sh1(:,2));
sh2_f(:,2) = filtfilt(b,a,sh2(:,2));
sh3_f(:,2) = filtfilt(b,a,sh3(:,2));

%% Calc angles using filtered positional data.
%angle offset of the markers if the thigh x,y axes were aligned with inertial ref. frame x,y axes
theta_t_f = atand((th2_f(:,1) - th1_f(:,1))./ (th2_f(:,2) - th1_f(:,2))) + marker_offset_angle_thigh;

%Shank angle
marker_offset_angle_shank = 0; %using proximal shank markers 1&2 for marker_config = 1, 2, or 3
theta_s_f = atand((sh2_f(:,2) - sh1_f(:,2)) ./ (sh2_f(:,1) - sh1_f(:,1))) - marker_offset_angle_shank;

%% Position filtered ==> angle ==> gyro

% Thigh angle

%angle offset of the markers if the thigh x,y axes were aligned with inertial ref. frame x,y axes
theta_t_pos_filt = atand((th2_f(:,2) - th1_f(:,2))./ (th2_f(:,1) - th1_f(:,1))) - marker_offset_angle_thigh;

%Shank angle

theta_s_pos_filt = atand((sh2_f(:,2) - sh1_f(:,2)) ./ (sh2_f(:,1) - sh1_f(:,1))) - marker_offset_angle_shank;


%% Kinematic information
%Angular Velocity/Angular acceleration
dt = 0.0025;
omega_t = gradient(theta_t_f,dt);
omega_t = deg2rad(omega_t); %convert to rad/s

omega_s = gradient(theta_s_f, dt);
omega_s = deg2rad(omega_s); %convert to rad/s

alpha_t = gradient(omega_t,dt);
alpha_s = gradient(omega_s,dt);

%position -> velocity -> acceleration
%convert pixels to meters
th1_f = th1_f .* pix_dist_ratio_thigh;
th2_f = th2_f .* pix_dist_ratio_thigh;
th3_f = th3_f .* pix_dist_ratio_thigh;

th1_unf = th1_unf .* pix_dist_ratio_thigh;

sh1_f = sh1_f .* pix_dist_ratio_shank;
sh2_f = sh2_f .* pix_dist_ratio_shank;
sh3_f = sh3_f .* pix_dist_ratio_shank;

sh1_unf = sh1_unf .* pix_dist_ratio_shank;
%velocity of thigh and shank points in inertial X-Y frame of video
[aa,vel_t1] = gradient(th1_f,dt);
[aa,vel_t2] = gradient(th2_f,dt);
[aa,vel_t3] = gradient(th3_f,dt);

[aa,vel_s1] = gradient(sh1_f,dt);
[aa,vel_s2] = gradient(sh2_f,dt);
[aa,vel_s3] = gradient(sh3_f,dt);

%acceleration of thigh and shank points in inertial frame of video

[aa,acc_t1] = gradient(vel_t1,dt);
[aa,acc_t2] = gradient(vel_t2,dt);
[aa,acc_t3] = gradient(vel_t3,dt);

[aa,acc_s1] = gradient(vel_s1,dt);
[aa,acc_s2] = gradient(vel_s2,dt);
[aa,acc_s3] = gradient(vel_s3,dt);

%% Convert kinematic info to body segment ref. frames

%Thigh
acc_t1_thigh(:,1) = acc_t1(:,1) .* cosd(theta_t_f) + acc_t1(:,2) .* sind(theta_t_f); % x-acceleration of marker #1, in thigh-frame
acc_t1_thigh(:,2) = -acc_t1(:,1) .* sind(theta_t_f) + acc_t1(:,2) .* cosd(theta_t_f); % y-acceleration
acc_t1_thigh(:,3) = 0;

acc_t2_thigh(:,1) = acc_t2(:,1) .* cosd(theta_t_f) + acc_t2(:,2) .* sind(theta_t_f); % x-acceleration of marker #1, in thigh-frame
acc_t2_thigh(:,2) = -acc_t2(:,1) .* sind(theta_t_f) + acc_t2(:,2) .* cosd(theta_t_f); % y-acceleration
acc_t2_thigh(:,3) = 0;

acc_t3_thigh(:,1) = acc_t3(:,1) .* cosd(theta_t_f) + acc_t3(:,2) .* sind(theta_t_f); % x-acceleration of marker #1, in thigh-frame
acc_t3_thigh(:,2) = -acc_t3(:,1) .* sind(theta_t_f) + acc_t3(:,2) .* cosd(theta_t_f); % y-acceleration
acc_t3_thigh(:,3) = 0;

%Shank
acc_s1_shank(:,1) = acc_s1(:,1) .* cosd(theta_s_f) - acc_s1(:,2) .* sind(theta_s_f);
acc_s1_shank(:,2) = acc_s1(:,1) .* sind(theta_s_f) + acc_s1(:,2) .* cosd(theta_s_f);
acc_s1_shank(:,3) = 0;

acc_s2_shank(:,1) = acc_s2(:,1) .* cosd(theta_s_f) - acc_s2(:,2) .* sind(theta_s_f);
acc_s2_shank(:,2) = acc_s2(:,1) .* sind(theta_s_f) + acc_s2(:,2) .* cosd(theta_s_f);
acc_s2_shank(:,3) = 0;

acc_s3_shank(:,1) = acc_s3(:,1) .* cosd(theta_s_f) - acc_s3(:,2) .* sind(theta_s_f);
acc_s3_shank(:,2) = acc_s3(:,1) .* sind(theta_s_f) + acc_s3(:,2) .* cosd(theta_s_f);
acc_s3_shank(:,3) = 0;

%% Calculate acceleration at Thigh KJC and Shank KJC. Average 3 tracked points for each segment.
% Position vectors from marker to KJC (meters) from CAD model
if marker_config == 1 % for BBBB, BBCB IMU configuration
    r_t1 = [-0.01414, -0.08488, 0];
    r_t2 = [0.01634, -0.0704, 0];
    r_t3 = [-0.00353, -0.0473, 0];
    
    r_s1 = [0.013, 0, 0];
    r_s2 = [-0.013, 0, 0];
    r_s3 = [-0.00353, 0.06047, 0];
elseif marker_config == 2  % for BBAB IMU configuration
    r_t1 = [-0.01414, -0.08488, 0];
    r_t2 = [0.01634, -0.0704, 0];
    r_t3 = [-0.00353, -0.0473, 0];
    
    r_s1 = [0.013, 0, 0];
    r_s2 = [-0.013, 0, 0];
    r_s3 = [-0.01294, 0.14429, 0];
    
elseif marker_config == 3  % for soft -tissue RATA trials
    r_t1 = [0.00816, -0.29699, 0];
    r_t2 = [0.01634, -0.0704, 0];
    r_t3 = [-0.00353, 0.06047, 0];
    
    r_s1 = [0.013, 0, 0];
    r_s2 = [-0.013, 0, 0];
    r_s3 = [-0.00931, 0.19529, 0];
else
    disp('ERROR: check marker configuration')
end
%make omega and alpha 3d vectors with only non-zero values along hte
%z-component
for j = 1:3
    if j ~= 3
        omega_t_3d(:,j) = zeros(length(omega_t),1);
        omega_s_3d(:,j) = zeros(length(omega_t),1);
        alpha_t_3d(:,j) = zeros(length(omega_t),1);
        alpha_s_3d(:,j) = zeros(length(omega_t),1);
    else
        omega_t_3d(:,j) = omega_t;
        omega_s_3d(:,j) = omega_s;
        alpha_t_3d(:,j) = alpha_t;
        alpha_s_3d(:,j) = alpha_s;
    end
end

% Thigh KJC acceleration in Thigh frame -- format: %acc_kjc_bodysegment-marker#_frame
for i = 1:length(omega_t)
    acc_kjc_t1_thigh(i,:) = acc_t1_thigh(i,:) + cross(omega_t_3d(i,:), cross(omega_t_3d(i,:), r_t1)) + cross(alpha_t_3d(i,:), r_t1);
    acc_kjc_t2_thigh(i,:) = acc_t2_thigh(i,:) + cross(omega_t_3d(i,:), cross(omega_t_3d(i,:), r_t2)) + cross(alpha_t_3d(i,:), r_t2);
    acc_kjc_t3_thigh(i,:) = acc_t3_thigh(i,:) + cross(omega_t_3d(i,:), cross(omega_t_3d(i,:), r_t3)) + cross(alpha_t_3d(i,:), r_t3);
end
%Thigh KJC acceleration in Shank  Frame (Anterior direction only)

acc_kjc_t1_shank = acc_kjc_t1_thigh(:,1) .* cosd(theta_t_f - theta_s_f) - acc_kjc_t1_thigh(:,2) .* sind(theta_t_f - theta_s_f);
acc_kjc_t2_shank = acc_kjc_t2_thigh(:,1) .* cosd(theta_t_f - theta_s_f) - acc_kjc_t2_thigh(:,2) .* sind(theta_t_f - theta_s_f);
acc_kjc_t3_shank = acc_kjc_t3_thigh(:,1) .* cosd(theta_t_f - theta_s_f) - acc_kjc_t3_thigh(:,2) .* sind(theta_t_f - theta_s_f);

acc_kjc_tavg_shank = (acc_kjc_t1_shank + acc_kjc_t2_shank +acc_kjc_t3_shank)./3;


% Shank KJC acceleration
for i = 1:length(omega_t)
    acc_kjc_s1_shank(i,:) = acc_s1_shank(i,:) + cross(omega_s_3d(i,:), cross(omega_s_3d(i,:), r_s1)) + cross(alpha_s_3d(i,:), r_s1);
    acc_kjc_s2_shank(i,:) = acc_s2_shank(i,:) + cross(omega_s_3d(i,:), cross(omega_s_3d(i,:), r_s2)) + cross(alpha_s_3d(i,:), r_s2);
    acc_kjc_s3_shank(i,:) = acc_s3_shank(i,:) + cross(omega_s_3d(i,:), cross(omega_s_3d(i,:), r_s3)) + cross(alpha_s_3d(i,:), r_s3);
end

acc_kjc_savg_shank = (acc_kjc_s1_shank(:,1) + acc_kjc_s2_shank(:,1) + acc_kjc_s3_shank(:,1))./3;
%
%% RATA MOCAP

RATA_derived = acc_kjc_savg_shank - acc_kjc_tavg_shank;

%% ATT MOCAP
%Thigh vector offset
r_t_offset_x = (sind(90-theta_t_f) .* r_t1(1)) + sind(theta_t_f) .* r_t1(2); % position vectors from marker #1 on thigh to kjc in xy frame
r_t_offset_y = (cosd(90-theta_t_f).* r_t1(1)) + cosd(theta_t_f) .* r_t1(2);
zero_vec = zeros(length(th1_f),1);

r_KJC_t_xy = [th1_f(:,1) + r_t_offset_x, th1_f(:,2) + r_t_offset_y,zero_vec ]; %KJC expressed in XY inertial fframe

%Shank vector offset expressed in XY frame
r_s_offset_x = (cosd(theta_s_f) .* r_s1(1)); %pos vectors from marker #1 on shank to kjc in x-y frame
r_s_offset_y = (sind(theta_s_f) .* r_s1(1));


r_KJC_s_xy = [sh1_f(:,1) + r_s_offset_x, sh1_f(:,2) + r_s_offset_y, zero_vec];

%initial difference between KJCs in XY frame
r_KJC_diff_xy = r_KJC_s_xy(1,:) - r_KJC_t_xy(1,:);

%
r_KJC_s_xy_adj  = r_KJC_s_xy - r_KJC_diff_xy;

ATT_xy = r_KJC_s_xy_adj - r_KJC_t_xy;

%ATT_s = ATT_xy(:,1) .* cosd(theta_t_f(:,1) - theta_s_f(:,1)) - ATT_xy(:,2) .* sind(theta_t_f(:,1) - theta_s_f(:,1));

ATT_s = ATT_xy(:,1) .* cosd(theta_s_f(:,1)) + ATT_xy(:,2) .* sind(1*theta_s_f(:,1));


vel_kjc_shank_x = gradient(r_KJC_s_xy_adj(:,1),0.0025);
vel_kjc_shank_y = gradient(r_KJC_s_xy_adj(:,2), 0.0025);

vel_kjc_thigh_x = gradient(r_KJC_t_xy(:,1),0.0025);
vel_kjc_thigh_y = gradient(r_KJC_t_xy(:,2),0.0025);

acc_kjc_shank_x = gradient(vel_kjc_shank_x,0.0025);
acc_kjc_shank_y = gradient(vel_kjc_shank_y, 0.0025);

acc_kjc_thigh_x = gradient(vel_kjc_thigh_x,0.0025);
acc_kjc_thigh_y = gradient(vel_kjc_thigh_y,0.0025);

acc_kjc_thigh_xy(:,1) = acc_kjc_thigh_x;
acc_kjc_thigh_xy(:,2) = acc_kjc_thigh_y;
acc_kjc_shank_xy(:,1) = acc_kjc_shank_x;
acc_kjc_shank_xy(:,2) = acc_kjc_shank_y;

acc_kjc_shank(:,1) = acc_kjc_shank_x .*cosd(theta_s_f(:,1)) + acc_kjc_shank_y .* sind(1*theta_s_f(:,1)) + 1*9.81 .* sind(theta_s_f(:,1));
acc_kjc_thigh(:,1) = acc_kjc_thigh_x .*cosd(theta_s_f(:,1)) + acc_kjc_thigh_y .* sind(1*theta_s_f(:,1)) + 1*9.81 .* sind(theta_s_f(:,1));

acc_kjc_shank(:,2) = acc_kjc_shank_y .*cosd(theta_s_f(:,1)) + acc_kjc_shank_x .* sind(1*theta_s_f(:,1)) + 9.81 .* cosd(theta_s_f(:,1));


acc_kjc_thigh(:,2) = acc_kjc_thigh_y .*cosd(theta_s_f(:,1)) + acc_kjc_thigh_x .* sind(1*theta_s_f(:,1)) + 9.81 .* cosd(theta_s_f(:,1));


p_kjc_shank = r_KJC_s_xy_adj(:,1) .* cosd(theta_s_f(:,1)) + r_KJC_s_xy_adj(:,2) .* sind(theta_s_f(:,1));
p_kjc_thigh = r_KJC_t_xy(:,1) .* cosd(theta_s_f(:,1)) + r_KJC_t_xy(:,2) .* sind(theta_s_f(:,1));




ATV_s = gradient(ATT_s,0.0025);

ATV_xy(:,1) = gradient(ATT_xy(:,1), 0.0025);
ATV_xy(:,2) = gradient(ATT_xy(:,2), 0.0025);

%RATA_s = gradient(ATV_s, 0.0025); % double derivative of ATT measured directly from Mocap data.
RATA_s = acc_kjc_shank(:,1) - acc_kjc_thigh(:,1);
RATA_xy(:,1) = gradient(ATV_xy(:,1), 0.0025);
RATA_xy(:,2) = gradient(ATV_xy(:,2), 0.0025);

RATA_s2 = RATA_xy(:,1) .* cosd(theta_s_f(:,1)) + RATA_xy(:,2) .* sind(1*theta_s_f(:,1));
%%

RATA_mocap = struct('RATA_mocap',RATA_s,'RATA_mocap2', RATA_s2,'RATA_xy', RATA_xy,'ATT_mocap',ATT_s,'ATV_mocap',ATV_s,'RATA_derived',RATA_derived,'acc_kjc_shank',acc_kjc_shank, 'acc_kjc_thigh', acc_kjc_thigh,'ATT_s',ATT_s,'p_kjc_shank',p_kjc_shank, 'p_kjc_thigh', p_kjc_thigh,'fc_t_angle',...
     fc_t, 'fc_s_angle',fc_s, 'fc_t_x',fc_t_x, 'fc_t_y', fc_t_y, 'fc_s_x', fc_s_x, 'fc_s_y', fc_s_y,'t_mark_mocap_sync',...
    acc_kjc_t1_thigh(:,2),'vel_t1',vel_t1, 'vel_s1',vel_s1,'acc_t1', acc_t1_thigh,'acc_s1', acc_s1_shank,'omega_s_mocap', omega_s, 'omega_t_mocap', omega_t,...
    'alpha_s_mocap', alpha_s, 'alpha_t_mocap', alpha_t,'pix_dist_ratio_thigh',pix_dist_ratio_thigh, 'pix_dist_ratio_shank', pix_dist_ratio_shank, 'thigh_raw', th1_unf, 'shank_raw', sh1_unf, 'thigh_filt' , th1_f, 'shank_filt', sh1_f...
    ,'theta_s_raw',theta_s, 'theta_s_filt', theta_s_f, 'theta_t_raw', theta_t, 'theta_t_filt', theta_t_f, 'acc_kjc_shank_derived',acc_kjc_s1_shank(:,1), 'acc_kjc_thigh_derived', acc_kjc_t1_shank, 'acc_kjc_thigh_xy', acc_kjc_thigh_xy, 'acc_kjc_shank_xy', acc_kjc_shank_xy);

end
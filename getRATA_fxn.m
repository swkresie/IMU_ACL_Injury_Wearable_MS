%% README
% This function is used during the analysis of RATA data for the mechanical
% test rig with soft-tissue implemented on the bodysegments as well as for
% RATA data from human subjet testing. This function takes the ouput of
% importData_fxn.m (data from the recording session), the output of
% getDCM_fxn.m (IMU frame to anatomical ref. frame DCMs for each IMU), the
% output of getposvecs_fxn.m (position vectors from each IMU's accelerometer
% location to KJC) as its INPUTS. Another optional input is button_presses
% for indication of different sections of RATA collection (if applicable)
% default will assume only a single section of RATA collection (one button
% press to start, one button press to end)

%OUTPUT
% trialRATA = struct('RATA',RATA_IMU,'shank_acc_kjc',shank_acc_kjc, 'thigh_acc_kjc', thigh_acc_kjc,'max_RATA_index',max_RATA_index);
% RATA_IMU is a nxm matrix where n is the number of trials (drop trials or
% RATA sections ,types of movements) that were collected. m is the length
% of each RATA trial. shank and thigh KJC acc are also output in the
% structure as a 3xm matrix. max_RATA is a 1xn matrix where the index for where max RATA
% occurs for each trial. max_RATA can be used with mocap data and
% mocap_processing_fxn.m output to align IMU RATA and MOCAP RATA.

function trialRATA = getRATA_fxn(data, cutoff_freq, DCMs, posvecs, button_presses, mocap_filename, marker_config)

if ~exist('button_presses','var')
    % second argument does not exist so default to single calibration
    button_presses = [11:12]%button press #s for when RATA data was being calculated
    num_trials = 1;
else
    num_trials = length(button_presses)/2;
end

if ~exist('mocap_filename','var')
    mocap_filename = 0;
    disp('no mocap filename provided, mocap not used')
end

if ~exist('marker_config','var')
    marker_config = 0;
    disp('no marker configuration provided, mocap not used')
end

%% IMU#1 Filter acceleration and gyro data
%acceleration
acc_1x = data.sensors1.accelerometerX.*9.81;
acc_1y = data.sensors1.accelerometerY.*9.81;
acc_1z = data.sensors1.accelerometerZ.*9.81;
acc1 = data.sensors1.accelerometerVector;


acc1 = acc1'; % Make it a 3 X N array, will be pre-multiplied by a 3x3 Rotation Matrix
acc1 = acc1 .* 9.81; % convert g's to m/s^2
%angular velocity
gyr_1x = data.sensors1.gyroscopeX;
gyr_1y = data.sensors1.gyroscopeY;
gyr_1z = data.sensors1.gyroscopeZ;
gyr1 = data.sensors1.gyroscopeVector; 


gyr1 = gyr1'; % Make it a 3 X N array, will be pre-multipled by a 3x3 rotation marix.

%% IMU#2 Filter acceeration and gyro data

%acceleration
acc_2x = data.sensors2.accelerometerX.*9.81;
acc_2y = data.sensors2.accelerometerY.*9.81;
acc_2z = data.sensors2.accelerometerZ.*9.81;
acc2 = data.sensors2.accelerometerVector;

acc2 = acc2';
acc2 = acc2 .* 9.81; % convert g's to m/s^2

%angular velocity

gyr_2x = data.sensors2.gyroscopeX;
gyr_2y = data.sensors2.gyroscopeY;
gyr_2z = data.sensors2.gyroscopeZ;
gyr2 = data.sensors2.gyroscopeVector;

gyr2 = gyr2'; %3XN Convention


%% IMU TIME SYNC -- Lag between IMU #1 and #2
button_1 = data.button1(1); % TIME SYNC button presses, used for IMU sync only, not for DCM calibration.
button_2 = data.button1(2);

IMU1_accx = acc_1x(1:button_2);
IMU2_accx = acc_2x(1:button_2);

[IMU1_a,IMU2_a,t21] = alignsignals(IMU1_accx, IMU2_accx);

%% Define angular acceleration for IMU#1 and IMU#2
dt = 0.0025;
alpha1 = zeros(3,size(acc1,2));
alpha1(:,1) = (gyr1(:,2)-gyr1(:,1))/dt;
alpha1(:,2:end-1) = (gyr1(:,3:end) - gyr1(:,1:end-2))/(2*dt);
alpha1(:,end) = (gyr1(:,end) - gyr1(end-1))/dt;

alpha1_unf = alpha1;

% fs = 400; %filter, sampling freq
%     fc = 80; %butterworth filter cutoff freq
%     [b,a] = butter(4, fc/(fs/2));
%     alpha1(1,:) = filtfilt(b,a,alpha1(1,:));
%     alpha1(2,:) = filtfilt(b,a,alpha1(2,:));
%     alpha1(3,:) = filtfilt(b,a,alpha1(3,:));
% 
alpha2 = zeros(3,size(acc2,2));
alpha2(:,1) = (gyr2(:,2)-gyr2(:,1))/dt;
alpha2(:,2:end-1) = (gyr2(:,3:end) - gyr2(:,1:end-2))/(2*dt);
alpha2(:,end) = (gyr2(:,end) - gyr2(end-1))/dt;
% 
alpha2_unf = alpha2;
% 
%     alpha2(1,:) = filtfilt(b,a,alpha2(1,:));
%     alpha2(2,:) = filtfilt(b,a,alpha2(2,:));
%     alpha2(3,:) = filtfilt(b,a,alpha2(3,:));

    %% Create quaternion variable
    quats_1 = data.quat1;
    quats_2 = data.quat2;
    
    quats_1_conj = quatconj(quats_1);
    quats_2_conj = quatconj(quats_2);

%% Button Press Indices (IMU 1)
%button press of thigh IMU (IMU 1) to indicate 
% TIME SYNC
%1. Heel drop (x3) start (human testing) / IMU Collissions (mechanical test rig) (x3)
%2. Heel drop end
% KINEMATIC DATA COLLECTION
%DCM
%Default: button press 3,4,5,6
%PosVECS
%Default: button press 7,8,9,10
%7. start shank isolation movement
%8. end shank isolation movement
%9. start thigh isolation movement
%10. end thigh isolation movement
%RATA
%11. Start RATA 
%12. End RATA
% presses 11-12 are repeated for consecutive trials

for i = 1:num_trials
    
    ind_RATA_start(i,1) = data.button1(button_presses(2*(i - 1) + 1)); %default num_trials = 1, start is button_presses(1) which is the 3rd button press.
    ind_RATA_end(i,1) = data.button1(button_presses(2*(i - 1) + 2)); %default num_trials = 1, start is button_presses(2) which is the 4th button press.
   
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% num_trials
% ind_RATA_start
% ind_RATA_end
for n = 1:num_trials
    trial_number = strcat('tr', num2str(n));
    %% CUT Data to Mocap Portion, Shift IMU 2 Data
    
    clear acc1_mocap
    clear acc2_mocap
    clear gyr1_mocap
    clear gyr2_mocap
    clear quats_1_mocap
    clear quats_1_conj_mocap
    clear quats_2_mocap
    clear quats_2_conj_mocap
    
    acc1_mocap = acc1(:,(ind_RATA_start(n):ind_RATA_end(n)));
    acc2_mocap = acc2(:,(ind_RATA_start(n)+t21:ind_RATA_end(n)+t21));
    
    gyr1_mocap = gyr1(:,(ind_RATA_start(n):ind_RATA_end(n)));
    gyr2_mocap = gyr2(:, (ind_RATA_start(n) +t21: ind_RATA_end(n)+t21));
    
    quats_1_mocap = quats_1(ind_RATA_start(n):ind_RATA_end(n),:);
    quats_1_conj_mocap = quats_1_conj(ind_RATA_start(n):ind_RATA_end(n),:);
    quats_2_mocap = quats_2(ind_RATA_start(n)+t21:ind_RATA_end(n)+t21,:);
    quats_2_conj_mocap = quats_2_conj(ind_RATA_start(n) + t21:ind_RATA_end(n) + t21, :);
    
   
    
    %% LOAD DCMs for IMU #1 and IMU#2
    R_body_IMU_1 = DCMs.DCM1T.tr1;
    R_body_IMU_2 = DCMs.DCM2S.tr1;
    
    %% Convert Quaternions to DCMs
    DCM_1S = zeros(3,3, length(acc1_mocap));
    DCM_2T = zeros(3,3, length(acc1_mocap));
    
    for i = 1:length(acc1_mocap)
        DCM_1S(:,:,i) = R_body_IMU_2 * quat2dcm(quatmultiply(quats_1_mocap(i,:),quats_2_conj_mocap(i,:)));  %IMU1 -> IMU2 -> Shank Frame
        DCM_2T(:,:,i) = R_body_IMU_1 * quat2dcm(quatmultiply(quats_2_mocap(i,:),quats_1_conj_mocap(i,:))); % IMU2 -> IMU1 -> Thigh frame
    end
    
    %% Convert Kinematic data to body segment frames, deg--> rad
    
    acc1_mocap_ana = R_body_IMU_1 * acc1_mocap;
    gyr1_mocap_ana = deg2rad(R_body_IMU_1 * gyr1_mocap); %deg/s to rad/s
    
    acc2_mocap_ana = R_body_IMU_2 * acc2_mocap;
    gyr2_mocap_ana = deg2rad(R_body_IMU_2 * gyr2_mocap); %deg/s to rad/s
    
    %% Filtering cutoff frequency
    
    if cutoff_freq == 0 %(Yu, 1999 Fitering method)
    % cutoff_freq variable not input by user
    % use imu data to estimate optimal cutoff frequency using Bing Yu
    % (1999) method
    %Identify peak vertical acceleration (approximate impact)
    
    [pk_v_t, pk_ind_t] = max(acc1_mocap_ana(1,:)); % max thigh IMU vertical acceleration (thigh frame), index
    [pk_v_s, pk_ind_s] = max(acc2_mocap_ana(1,:)); % max shank IMU vert acceleration (shank frame) & index,

    
    % 50 samples on either side of the peak are used to estimate optimal
    % cutoff freq. This was determined by the length of the non-static
    % acceleration signal during the mechanical test rig drop tests.
   fs = 400;
    
   fc_1 = 0.071*fs - 0.00003*fs^2; % mean cutoff freq. for a given sample freq.
   [b,a] = butter(4, fc_1/(fs/2));
       acc1x_fc1 = filtfilt(b,a, acc1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50)); %filter x-component of thigh IMU acceleration;
       acc1y_fc1 = filtfilt(b,a, acc1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50)); % filter y-component
       acc1z_fc1 = filtfilt(b,a, acc1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50)); %filter z
       
       gyr1x_fc1 = filtfilt(b,a, gyr1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50)); %filter x
       gyr1y_fc1 = filtfilt(b,a, gyr1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50)); %filter y
       gyr1z_fc1 = filtfilt(b,a, gyr1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50)); %filter z
       
       acc2x_fc1 = filtfilt(b,a, acc2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50)); %filter x-component of thigh IMU acceleration;
       acc2y_fc1 = filtfilt(b,a, acc2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50)); % filter y-component
       acc2z_fc1 = filtfilt(b,a, acc2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50)); %filter z
       
       gyr2x_fc1 = filtfilt(b,a, gyr2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50)); %filter x
       gyr2y_fc1 = filtfilt(b,a, gyr2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50)); %filter y
       gyr2z_fc1 = filtfilt(b,a, gyr2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50)); %filter z
       
       epsilon_acc1x = sqrt((sum((acc1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50) - acc1x_fc1).^2))/sum((acc1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50) - mean(acc1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50))).^2)*100)*1;
       epsilon_acc1y = sqrt((sum((acc1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50) - acc1y_fc1).^2))/sum((acc1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50) - mean(acc1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50))).^2)*100)*1;
       epsilon_acc1z = sqrt((sum((acc1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50) - acc1z_fc1).^2))/sum((acc1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50) - mean(acc1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50))).^2)*100)*1;
       
       epsilon_gyr1x = sqrt((sum((gyr1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50) - gyr1x_fc1).^2))/sum((gyr1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50) - mean(gyr1_mocap_ana(1,pk_ind_t - 50:pk_ind_t + 50))).^2)*100)*1;
       epsilon_gyr1y = sqrt((sum((gyr1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50) - gyr1y_fc1).^2))/sum((gyr1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50) - mean(gyr1_mocap_ana(2,pk_ind_t - 50:pk_ind_t + 50))).^2)*100)*1;
       epsilon_gyr1z = sqrt((sum((gyr1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50) - gyr1z_fc1).^2))/sum((gyr1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50) - mean(gyr1_mocap_ana(3,pk_ind_t - 50:pk_ind_t + 50))).^2)*100)*1;
       
       epsilon_acc2x = sqrt((sum((acc2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50) - acc2x_fc1).^2))/sum((acc2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50) - mean(acc2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50))).^2)*100)*1;
       epsilon_acc2y = sqrt((sum((acc2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50) - acc2y_fc1).^2))/sum((acc2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50) - mean(acc2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50))).^2)*100)*1;
       epsilon_acc2z = sqrt((sum((acc2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50) - acc2z_fc1).^2))/sum((acc2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50) - mean(acc2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50))).^2)*100)*1;
       
       epsilon_gyr2x = sqrt((sum((gyr2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50) - gyr2x_fc1).^2))/sum((gyr2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50) - mean(gyr2_mocap_ana(1,pk_ind_s - 50:pk_ind_s + 50))).^2)*100)*1;
       epsilon_gyr2y = sqrt((sum((gyr2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50) - gyr2y_fc1).^2))/sum((gyr2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50) - mean(gyr2_mocap_ana(2,pk_ind_s - 50:pk_ind_s + 50))).^2)*100)*1;
       epsilon_gyr2z = sqrt((sum((gyr2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50) - gyr2z_fc1).^2))/sum((gyr2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50) - mean(gyr2_mocap_ana(3,pk_ind_s - 50:pk_ind_s + 50))).^2)*100)*1;
        
       fc_2_acc1x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc1x);
       fc_2_acc1y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc1y);
       fc_2_acc1z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc1z);
       
       fc_2_gyr1x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr1x);
       fc_2_gyr1y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr1y);
       fc_2_gyr1z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr1z);
               
       fc_2_acc2x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc2x);
       fc_2_acc2y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc2y);
       fc_2_acc2z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc2z);
      
       fc_2_gyr2x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr2x);
       fc_2_gyr2y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr2y);
       fc_2_gyr2z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr2z);
       
       fc_acc1_x = fc_2_acc1x ;
       fc_acc1_y = fc_2_acc1y ;
       fc_acc1_z = fc_2_acc1z ;
       
       fc_gyr1_x = fc_2_gyr1x ;
       fc_gyr1_y = fc_2_gyr1y ;
       fc_gyr1_z = fc_2_gyr1z ;
       
       
       fc_acc2_x = fc_2_acc2x;
       fc_acc2_y = fc_2_acc2y ;
       fc_acc2_z = fc_2_acc2z ;
       
       fc_gyr2_x = fc_2_gyr2x ;
       fc_gyr2_y = fc_2_gyr2y ;
       fc_gyr2_z = fc_2_gyr2z ;
       
       order = 4; %Order of butterworth filter
       %IMU1 output filter cutoffs
       filters.(trial_number).acc1x = fc_acc1_x;
       filters.(trial_number).acc1y = fc_acc1_y;
       filters.(trial_number).acc1z = fc_acc1_z;
       filters.(trial_number).gyr1x = fc_gyr1_x;
       filters.(trial_number).gyr1y = fc_gyr1_y;
       filters.(trial_number).gyr1z = fc_gyr1_z;
       %IMU2 output filter cutoffs
       filters.(trial_number).acc2x = fc_acc2_x;
       filters.(trial_number).acc2y = fc_acc2_y;
       filters.(trial_number).acc2z = fc_acc2_z;
       filters.(trial_number).gyr2x = fc_gyr2_x;
       filters.(trial_number).gyr2y = fc_gyr2_y;
       filters.(trial_number).gyr2z = fc_gyr2_z;
    
    elseif cutoff_freq == -1
        % cutoff_freq variable not input by user
    % use imu data to estimate optimal cutoff frequency using Bing Yu
    % (1999) method
    % Use all data between RATA button press to identify optimal cutoff freq(for walking, jogging trials) 
   fs = 400;
    
   fc_1 = 0.071*fs - 0.00003*fs^2; % mean cutoff freq. for a given sample freq.
   [b,a] = butter(4, fc_1/(fs/2));
       acc1x_fc1 = filtfilt(b,a, acc1_mocap_ana(1,:)); %filter x-component of thigh IMU acceleration;
       acc1y_fc1 = filtfilt(b,a, acc1_mocap_ana(2,:)); % filter y-component
       acc1z_fc1 = filtfilt(b,a, acc1_mocap_ana(3,:)); %filter z
       
       gyr1x_fc1 = filtfilt(b,a, gyr1_mocap_ana(1,:)); %filter x
       gyr1y_fc1 = filtfilt(b,a, gyr1_mocap_ana(2,:)); %filter y
       gyr1z_fc1 = filtfilt(b,a, gyr1_mocap_ana(3,:)); %filter z
       
       acc2x_fc1 = filtfilt(b,a, acc2_mocap_ana(1,:)); %filter x-component of thigh IMU acceleration;
       acc2y_fc1 = filtfilt(b,a, acc2_mocap_ana(2,:)); % filter y-component
       acc2z_fc1 = filtfilt(b,a, acc2_mocap_ana(3,:)); %filter z
       
       gyr2x_fc1 = filtfilt(b,a, gyr2_mocap_ana(1,:)); %filter x
       gyr2y_fc1 = filtfilt(b,a, gyr2_mocap_ana(2,:)); %filter y
       gyr2z_fc1 = filtfilt(b,a, gyr2_mocap_ana(3,:)); %filter z
       
       epsilon_acc1x = sqrt((sum((acc1_mocap_ana(1,:) - acc1x_fc1).^2))/sum((acc1_mocap_ana(1,:) - mean(acc1_mocap_ana(1,:))).^2)*100)*1;
       epsilon_acc1y = sqrt((sum((acc1_mocap_ana(2,:) - acc1y_fc1).^2))/sum((acc1_mocap_ana(2,:) - mean(acc1_mocap_ana(2,:))).^2)*100)*1;
       epsilon_acc1z = sqrt((sum((acc1_mocap_ana(3,:) - acc1z_fc1).^2))/sum((acc1_mocap_ana(3,:) - mean(acc1_mocap_ana(3,:))).^2)*100)*1;
       
       epsilon_gyr1x = sqrt((sum((gyr1_mocap_ana(1,:) - gyr1x_fc1).^2))/sum((gyr1_mocap_ana(1,:) - mean(gyr1_mocap_ana(1,:))).^2)*100)*1;
       epsilon_gyr1y = sqrt((sum((gyr1_mocap_ana(2,:) - gyr1y_fc1).^2))/sum((gyr1_mocap_ana(2,:) - mean(gyr1_mocap_ana(2,:))).^2)*100)*1;
       epsilon_gyr1z = sqrt((sum((gyr1_mocap_ana(3,:) - gyr1z_fc1).^2))/sum((gyr1_mocap_ana(3,:) - mean(gyr1_mocap_ana(3,:))).^2)*100)*1;
       
       epsilon_acc2x = sqrt((sum((acc2_mocap_ana(1,:) - acc2x_fc1).^2))/sum((acc2_mocap_ana(1,:) - mean(acc2_mocap_ana(1,:))).^2)*100)*1;
       epsilon_acc2y = sqrt((sum((acc2_mocap_ana(2,:) - acc2y_fc1).^2))/sum((acc2_mocap_ana(2,:) - mean(acc2_mocap_ana(2,:))).^2)*100)*1;
       epsilon_acc2z = sqrt((sum((acc2_mocap_ana(3,:) - acc2z_fc1).^2))/sum((acc2_mocap_ana(3,:) - mean(acc2_mocap_ana(3,:))).^2)*100)*1;
       
       epsilon_gyr2x = sqrt((sum((gyr2_mocap_ana(1,:) - gyr2x_fc1).^2))/sum((gyr2_mocap_ana(1,:) - mean(gyr2_mocap_ana(1,:))).^2)*100)*1;
       epsilon_gyr2y = sqrt((sum((gyr2_mocap_ana(2,:) - gyr2y_fc1).^2))/sum((gyr2_mocap_ana(2,:) - mean(gyr2_mocap_ana(2,:))).^2)*100)*1;
       epsilon_gyr2z = sqrt((sum((gyr2_mocap_ana(3,:) - gyr2z_fc1).^2))/sum((gyr2_mocap_ana(3,:) - mean(gyr2_mocap_ana(3,:))).^2)*100)*1;
        
       fc_2_acc1x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc1x);
       fc_2_acc1y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc1y);
       fc_2_acc1z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc1z);
       
       fc_2_gyr1x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr1x);
       fc_2_gyr1y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr1y);
       fc_2_gyr1z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr1z);
               
       fc_2_acc2x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc2x);
       fc_2_acc2y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc2y);
       fc_2_acc2z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_acc2z);
      
       fc_2_gyr2x = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr2x);
       fc_2_gyr2y = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr2y);
       fc_2_gyr2z = 0.06*fs - 0.000022*fs^2 + 5.95*(1/epsilon_gyr2z);
       
       fc_acc1_x = fc_2_acc1x;
       fc_acc1_y = fc_2_acc1y;
       fc_acc1_z = fc_2_acc1z;
       
       fc_gyr1_x = fc_2_gyr1x;
       fc_gyr1_y = fc_2_gyr1y;
       fc_gyr1_z = fc_2_gyr1z;
       
       
       fc_acc2_x = fc_2_acc2x;
       fc_acc2_y = fc_2_acc2y ;
       fc_acc2_z = fc_2_acc2z ;
       
       fc_gyr2_x = fc_2_gyr2x;
       fc_gyr2_y = fc_2_gyr2y;
       fc_gyr2_z = fc_2_gyr2z;
       
       order = 4; %Order of butterworth filter
       %IMU1 output filter cutoffs
       filters.(trial_number).acc1x = fc_acc1_x;
       filters.(trial_number).acc1y = fc_acc1_y;
       filters.(trial_number).acc1z = fc_acc1_z;
       filters.(trial_number).gyr1x = fc_gyr1_x;
       filters.(trial_number).gyr1y = fc_gyr1_y;
       filters.(trial_number).gyr1z = fc_gyr1_z;
       %IMU2 output filter cutoffs
       filters.(trial_number).acc2x = fc_acc2_x;
       filters.(trial_number).acc2y = fc_acc2_y;
       filters.(trial_number).acc2z = fc_acc2_z;
       filters.(trial_number).gyr2x = fc_gyr2_x;
       filters.(trial_number).gyr2y = fc_gyr2_y;
       filters.(trial_number).gyr2z = fc_gyr2_z;
    else
 

    fc_acc1_x = cutoff_freq;
    fc_acc1_y = cutoff_freq;
    fc_acc1_z = cutoff_freq;
    
    fc_gyr1_x = cutoff_freq;
    fc_gyr1_y = cutoff_freq;
    fc_gyr1_z = cutoff_freq;
    
    fc_acc2_x = cutoff_freq;
    fc_acc2_y = cutoff_freq;
    fc_acc2_z = cutoff_freq;
        
    fc_gyr2_x = cutoff_freq;
    fc_gyr2_y = cutoff_freq;  
    fc_gyr2_z = cutoff_freq;
    
    filters = cutoff_freq;
%     A = [];
    order = 4; %order of butterworth filter
    end
    %% FILTER ACCELERATION, GYRO,  DATA
    fs = 400;
   
    %Thigh
    fc = fc_acc1_x;
    [b,a] = butter(order, fc/(fs/2));
    clear acc1_mocap_ana_f
    acc1_mocap_ana_f(1,:) = filtfilt(b,a, acc1_mocap_ana(1,:)); %filter x-component of thigh IMU acceleration;
    
    
    fc = fc_acc1_y;
    [b,a] = butter(order, fc/(fs/2));
    acc1_mocap_ana_f(2,:) = filtfilt(b,a, acc1_mocap_ana(2,:));
    
    fc = fc_acc1_z;
    [b,a] = butter(order, fc/(fs/2));
    acc1_mocap_ana_f(3,:) = filtfilt(b,a,acc1_mocap_ana(3,:));
    
    fc = fc_gyr1_z;
    [b,a] = butter(order, fc/(fs/2));
    clear gyr1_mocap_ana_f %clear previous trial data
    gyr1_mocap_ana_f(3,:) = filtfilt(b,a, gyr1_mocap_ana(3,:)); %filter x-component of thigh IMU acceleration;
    
    fc = fc_gyr1_y;
    [b,a] = butter(order, fc/(fs/2));
    gyr1_mocap_ana_f(2,:) = filtfilt(b,a, gyr1_mocap_ana(2,:));
    
    fc = fc_gyr1_x;
    [b,a] = butter(order, fc/(fs/2));
    gyr1_mocap_ana_f(1,:) = filtfilt(b,a, gyr1_mocap_ana(1,:));
    
    %Shank
    fc = fc_acc2_x;
    [b,a] = butter(order, fc/(fs/2));
     clear acc2_mocap_ana_f  %clear previous trial data
    acc2_mocap_ana_f(1,:) = filtfilt(b,a, acc2_mocap_ana(1,:)); %filter x-component of thigh IMU acceleration;
    
    fc = fc_acc2_y;
    [b,a] = butter(order, fc/(fs/2));
    acc2_mocap_ana_f(2,:) = filtfilt(b,a, acc2_mocap_ana(2,:));
    
    fc = fc_acc2_z;
    [b,a] = butter(order, fc/(fs/2));
    acc2_mocap_ana_f(3,:) = filtfilt(b,a, acc2_mocap_ana(3,:));
    
    fc = fc_gyr2_z;
    [b,a] = butter(order, fc/(fs/2));
    clear gyr2_mocap_ana_f  %clear previous trial data
    gyr2_mocap_ana_f(3,:) = filtfilt(b,a, gyr2_mocap_ana(3,:)); %filter x-component of thigh IMU acceleration;
    % gyr2_mocap_ana_f(2,:) = zeros(1,length(gyr1_mocap_ana));
    % gyr2_mocap_ana_f(1,:) = zeros(1,length(gyr1_mocap_ana));
    fc = fc_gyr2_y;
    [b,a] = butter(order, fc/(fs/2));
    gyr2_mocap_ana_f(2,:) = filtfilt(b,a, gyr2_mocap_ana(2,:));
    
    fc = fc_gyr2_x;
    [b,a] = butter(order, fc/(fs/2));
    gyr2_mocap_ana_f(1,:) = filtfilt(b,a, gyr2_mocap_ana(1,:));
    
    
    %% Calculate Angular Acceleration from filtered gyro
    dt = 0.0025; %400Hz sample rate
    alpha1_ana = gradient(gyr1_mocap_ana_f,dt);
    alpha2_ana = gradient(gyr2_mocap_ana_f,dt);
    
    %% LOAD POSITION VECTORS
    posvec1 = posvecs.posvec1.tr1;
    posvec2 = posvecs.posvec2.tr1;
    
     %posvec1 = [0, -0.175];
     % posvec2 = [0, 0.19];
    
   posvec1(3) = 0; %z-component of position vector is 0. Only x,y components are involved in RATA calculation
   posvec2(3) = 0;
     
%     posvec1(3) = -0.1143; %z-component of position vector -- SCOTT, based   %based  on measurements from scaled photos during testing
%   posvec2(3) = 0.0635;
%   
%    posvec1(3) = -0.09; %z-component of position vector --DAVE
%    posvec2(3) = 0.04;  
    
    %% Calculate IMU RATA
clear acc1_f
clear gyr1_f
clear alpha1_f
for i = 1:length(acc1_mocap_ana_f)
    acc1_f(:,i) = inv(R_body_IMU_1) * (acc1_mocap_ana_f(:,i));
    gyr1_f(:,i) = inv(R_body_IMU_1) * (gyr1_mocap_ana_f(:,i));
    alpha1_f(:,i) =inv(R_body_IMU_1) *(alpha1_ana(:,i));
    
    acc2_f(:,i) = inv(R_body_IMU_2) * (acc2_mocap_ana_f(:,i));
    gyr2_f(:,i) = inv(R_body_IMU_2) * (gyr2_mocap_ana_f(:,i));
    alpha2_f(:,i) = inv(R_body_IMU_2) * (alpha2_ana(:,i));
end

posvec1_IMU1frame = inv(R_body_IMU_1) * posvec1'; % Change posvec into IMU1 frame for thigh knee-joint center calc since values are in IMU 1 frame then converted to IMU2 then shank frame
posvec2_IMU2frame = inv(R_body_IMU_2) * posvec2';

clear shank_acc_kjc
clear thigh_acc_kjc
clear thigh_acc_kjc_tframe
for i = 1:length(acc2_mocap_ana_f)
    shank_acc_kjc(:,i) = 1.*acc2_mocap_ana_f(:,i) + 1.*cross(gyr2_mocap_ana_f(:,i), cross(gyr2_mocap_ana_f(:,i), posvec2')) + 1.*cross(alpha2_ana(:,i),posvec2');
    
    thigh_acc_kjc(:,i) = DCM_1S(:,:,i) * (1.*acc1_f(:,i) + 1.*cross(gyr1_f(:,i),cross(gyr1_f(:,i),posvec1_IMU1frame)) + 1.*cross(alpha1_f(:,i),posvec1_IMU1frame));

    thigh_acc_kjc_tframe(:,i) = R_body_IMU_1 * (acc1_f(:,i) + cross(gyr1_f(:,i),cross(gyr1_f(:,i),posvec1_IMU1frame)) + cross(alpha1_f(:,i),posvec1_IMU1frame));

    shank_acc_kjc_tframe(:,i) = DCM_2T(:,:,i) * (acc2_f(:,i) + cross(gyr2_f(:,i),cross(gyr2_f(:,i),posvec2_IMU2frame)) + cross(alpha2_f(:,i),posvec2_IMU2frame));
end



RATA_IMU.(trial_number) = shank_acc_kjc(1,:) - thigh_acc_kjc(1,:);
thigh_acc_kjc_all.(trial_number) = thigh_acc_kjc;
shank_acc_kjc_all.(trial_number) = shank_acc_kjc;
thigh_acc_kjc_tframe_all.(trial_number) = thigh_acc_kjc_tframe;
shank_acc_kjc_tframe_all.(trial_number) = shank_acc_kjc_tframe;
shank_gyr_all.(trial_number) = gyr2_mocap_ana_f;
thigh_gyr_all.(trial_number) = gyr1_mocap_ana_f;
shank_alpha_all.(trial_number) = alpha2_ana;
thigh_alpha_all.(trial_number) = alpha1_f;
[value, max_RATA] = max(RATA_IMU.(trial_number));

max_RATA_index.(trial_number) = max_RATA;
IMU1_raw.(trial_number) = vertcat(acc1_mocap_ana, gyr1_mocap_ana);
IMU1_filt.(trial_number) = vertcat(acc1_mocap_ana_f, gyr1_mocap_ana_f, alpha1_ana);
IMU2_raw.(trial_number) = vertcat(acc2_mocap_ana, gyr2_mocap_ana);
IMU2_filt.(trial_number) = vertcat(acc2_mocap_ana_f, gyr2_mocap_ana_f, alpha2_ana);
%disp(['trial ' ,num2str(n), ' complete']);
end

trialRATA = struct('RATA',RATA_IMU,'shank_gyr',shank_gyr_all, 'shank_alpha', shank_alpha_all,'shank_acc_kjc',shank_acc_kjc_all,...
    'thigh_gyr',thigh_gyr_all,'thigh_alpha', thigh_alpha_all,'thigh_acc_kjc',...
    thigh_acc_kjc_all,'thigh_acc_kjc_tframe', thigh_acc_kjc_tframe_all,'shank_acc_kjc_tframe',shank_acc_kjc_tframe_all,...
    'max_RATA_index',max_RATA_index, 'cutoff_freqs',filters,'IMU1_raw',IMU1_raw,'IMU2_raw', IMU2_raw, 'IMU1_filt', IMU1_filt, 'IMU2_filt', IMU2_filt);
end



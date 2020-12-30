%% README
% This function is used during the analysis of RATA data for the mechanical
% test rig with soft-tissue implemented on the bodysegments as well as for
% RATA data from human subject testing. This function takes the output of importData_fxn.m
%... the other input is a 1xn vector containing a series of the button
%presses recorded by the IMU #1 to indicate the different sections of the
%data collection session. Button press 1 and 2 are for syncing the IMUs and
%button presses 3 and on are for signifying the start and end of DCM
%calibration data collection.

%One input is A structure called dcm_calib_data which contains accelerometer and gyroscope data
% from each IMU. The fields/paths in the structure are:
% dcm_calib_data.{IMU1data,IMU2data}.{accelerometerX,accelerometerY,accelerometerZ,accelerometerVector,} where the fields inside the curly
% brackets denote the different possible fieldnames at each level of the
% structure.

%This function outputs a structure called trialDCMs which has the fields:
% {DCM1T, DCM2S} for the DCM to transform IMU#1 (thigh IMU) data to the
% thigh anatomical reference frame and the DCM to transform IMU#2 (shank
% IMU) to the shank anatomical reference frame, respectively.


function trialDCMs = getDCM_fxn(dcm_calib_data,button_presses)
if ~exist('button_presses','var')
    % second argument does not exist so default to single calibration
    button_presses = [3:6]; % default button press #s for when DCM calib data was being calculated (1 trial)
    num_trials = 1;
else
    num_trials = length(button_presses)/4;
end

% button_presses could look like 3:10 if two consecutive DCM calib data
% were taken... button press 3,4,5,6 for first calib dataset, button press
% 7,8,9,10 for second calib dataset

%% IMU#1 Filter acceleration and gyro data
%acceleration
acc_1x = dcm_calib_data.sensors1.accelerometerX.*9.81;
acc_1y = dcm_calib_data.sensors1.accelerometerY.*9.81;
acc_1z = dcm_calib_data.sensors1.accelerometerZ.*9.81;
acc1 = dcm_calib_data.sensors1.accelerometerVector;


%UNFILTERED ACC #1
acc1_unf = horzcat(acc_1x,acc_1y,acc_1z);
acc1_unf = acc1_unf'; %3XN array convention

%FILTERED ACC #1%%%%%%
fs = 400; %filter, sampling freq
    fc = 5; %butterworth filter cutoff freq
    [b,a] = butter(4, fc/(fs/2));
    acc1(:,1) = filtfilt(b,a,acc1(:,1));
    acc1(:,2) = filtfilt(b,a,acc1(:,2));
    acc1(:,3) = filtfilt(b,a,acc1(:,3));

acc1 = acc1'; % Make it a 3 X N array, will be pre-multiplied by a 3x3 Rotation Matrix
acc1 = acc1 .* 9.81; % convert g's to m/s^2
%angular velocity
gyr_1x = dcm_calib_data.sensors1.gyroscopeX;
gyr_1y = dcm_calib_data.sensors1.gyroscopeY;
gyr_1z = dcm_calib_data.sensors1.gyroscopeZ;
gyr1 = dcm_calib_data.sensors1.gyroscopeVector; 

%UNFILTERED GYRO #1%%%
gyr1_unf = horzcat(gyr_1x, gyr_1y, gyr_1z);
gyr1_unf = gyr1_unf';

%FILTERED GYRO #1%%%%
fs = 400; %filter, sampling freq
    fc = 5; %butterworth filter cutoff freq
    [b,a] = butter(4, fc/(fs/2));
    gyr1(:,1) = filtfilt(b,a,gyr1(:,1));
    gyr1(:,2) = filtfilt(b,a,gyr1(:,2));
    gyr1(:,3) = filtfilt(b,a,gyr1(:,3));

gyr1 = gyr1'; % Make it a 3 X N array, will be pre-multipled by a 3x3 rotation marix.

%% IMU#2 Filter acceeration and gyro data

%acceleration
acc_2x = dcm_calib_data.sensors2.accelerometerX.*9.81;
acc_2y = dcm_calib_data.sensors2.accelerometerY.*9.81;
acc_2z = dcm_calib_data.sensors2.accelerometerZ.*9.81;
acc2 = dcm_calib_data.sensors2.accelerometerVector;

%UNFILTERED ACC #2%%%%%% 
acc2_unf = horzcat(acc_2x,acc_2y,acc_2z);
acc2_unf = acc2_unf'; %3XN array convention

%FILTERED ACC #2%%%%%
fs = 400; %filter, sampling freq
    fc = 5; %butterworth filter cutoff freq
    [b,a] = butter(4, fc/(fs/2));
    acc2(:,1) = filtfilt(b,a,acc2(:,1));
    acc2(:,2) = filtfilt(b,a,acc2(:,2));
    acc2(:,3) = filtfilt(b,a,acc2(:,3));

acc2 = acc2';
acc2 = acc2 .* 9.81; % convert g's to m/s^2



%angular velocity
fs = 400; %filter, sampling freq
    fc = 5; %butterworth filter cutoff freq
    [b,a] = butter(4, fc/(fs/2));

gyr_2x = dcm_calib_data.sensors2.gyroscopeX;
gyr_2y = dcm_calib_data.sensors2.gyroscopeY;
gyr_2z = dcm_calib_data.sensors2.gyroscopeZ;
gyr2 = dcm_calib_data.sensors2.gyroscopeVector;

%UNFILTERED GYRO #1%%%
gyr2_unf = horzcat(gyr_2x, gyr_2y, gyr_2z);
gyr2_unf = gyr2_unf'; %3XN Convention

%FILTERED GYR #2 %%%%%%%
    gyr2(:,1) = filtfilt(b,a,gyr2(:,1));
    gyr2(:,2) = filtfilt(b,a,gyr2(:,2));
    gyr2(:,3) = filtfilt(b,a,gyr2(:,3));
gyr2 = gyr2'; %3XN Convention


%% IMU TIME SYNC -- Lag between IMU #1 and #2
button_1 = dcm_calib_data.button1(1); % TIME SYNC button presses, used for IMU sync only, not for DCM calibration.
button_2 = dcm_calib_data.button1(2);

IMU1_accx = acc_1x(1:button_2);
IMU2_accx = acc_2x(1:button_2);

[IMU1_a,IMU2_a,t21] = alignsignals(IMU1_accx, IMU2_accx);

%% Button Press Indices (IMU 1)
%button press of thigh IMU (IMU 1) to indicate 
% TIME SYNC
%1. Slide IMUs fixe don same rigid body for cross-correlation sync analysis:start
%2. Slide IMUs on fixed same rigid body for cross-correlation sync
%analysis: end

% KINEMATIC DATA COLLECTION
%3. start standing still
%4. end standing still
%5. start straight-leg swing
%6. end straight-leg swing

% presses 3-6 are repeated for consecutive trials


for i = 1:num_trials
    
    ind_standing_start(i,1) = dcm_calib_data.button1(button_presses(4*(i - 1) + 1)); %default num_trials = 1, start is button_presses(1) which is the 3rd button press.
    ind_standing_end(i,1) = dcm_calib_data.button1(button_presses(4*(i - 1) + 2)); %default num_trials = 1, start is button_presses(2) which is the 4th button press.
    ind_swing_start(i,1) = dcm_calib_data.button1(button_presses(4*(i - 1) + 3));
    ind_swing_end(i,1) = dcm_calib_data.button1(button_presses(4*(i - 1) + 4));
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% DCM CALCULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DCM1 = struct; %create final structure which will contain DCMs for all 6 different configurations. Each field will contain another structure of 10 fields, each containing 1 DCM for each of the 10 IMU1 trials
DCM2 = struct; % Ex. DCM2.AB.tr2 --> DCM for IMU2, AB configuration, trial #2/10.
for n = 1:num_trials
    ind_1_stand = round(ind_standing_start(n) + ((ind_standing_end(n) - ind_standing_start(n))/4)); % remove the first 25% of the standing phase
    ind_2_stand = round(ind_standing_end(n) - ((ind_standing_end(n) - ind_standing_start(n))/4)); % remove the last 25 % of the standing phase. Only view middle 50 %. 
    
    ind_1_flex = round(ind_swing_start(n) + ((ind_swing_end(n) - ind_swing_start(n))/4)); % remove the first 25% of the swing phase
    ind_2_flex = round(ind_swing_end(n) - ((ind_swing_end(n) - ind_swing_start(n))/4)); % remove the last 25 % of the swing phase. Only view middle 50 %. 
    %% Static Standing, mean acceleration -> Y
    
    % Use gravity vector to define segment Y axis
    ma_1_stand = horzcat(mean(acc_1x(ind_1_stand:ind_2_stand)),mean(acc_1y(ind_1_stand:ind_2_stand)),mean(acc_1z(ind_1_stand:ind_2_stand))); %mean acceleration vector during static standing
    ma_2_stand = horzcat(mean(acc_2x(ind_1_stand+t21:ind_2_stand+t21)),mean(acc_2y(ind_1_stand + t21:ind_2_stand+t21)), mean(acc_2z(ind_1_stand + t21: ind_2_stand + t21)));
    
    Y1 = ma_1_stand./norm(ma_1_stand);
    Y2 = ma_2_stand./norm(ma_2_stand);
    %% Straight-legged hip flexion, PCA -> Z
    % Use rotation to define segment Z axis
    PCAcoeff1 = pca(horzcat(gyr_1x(ind_1_flex:ind_2_flex),gyr_1y(ind_1_flex:ind_2_flex), gyr_1z(ind_1_flex:ind_2_flex)));
    PCAcoeff2 = pca(horzcat(gyr_2x(ind_1_flex + t21: ind_2_flex + t21), gyr_2y(ind_1_flex + t21: ind_2_flex + t21), gyr_2z(ind_1_flex + t21 : ind_2_flex + t21)));
    
    Z_pca1 = (PCAcoeff1(:,1))';
    Z_pca1 = Z_pca1./norm(Z_pca1);
    
    Z_pca2 = (PCAcoeff2(:,1))';
    Z_pca2 = Z_pca2./norm(Z_pca2);
    
    % Calculate X according to X = Y x Z
    X_pca1 = cross(Y1,Z_pca1);
    X_pca1 = X_pca1./norm(X_pca1);
    
    X_pca2 = cross(Y2,Z_pca2);
    X_pca2 = X_pca2./norm(X_pca2);
    
    % Re-calculate Z to ensure orthogonality (Z = X x Y)
    Z1 = cross(X_pca1,Y1);
    Z1 = Z1./norm(Z1);
    
    Z2 = cross(X_pca2,Y2);
    Z2 = Z2/norm(Z2);
    %
    %% Define direction cosine matrix for IMU #1 and IMU #2
    R_body_IMU_1 = vertcat(X_pca1,Y1, Z1);
    R_body_IMU_2 = vertcat(X_pca2,Y2, Z2);

    

    
    %% Check DCMs align IMUs in with correct anatomical directions. For right shank and thigh, +X = anterior, +Y = proximal, +Z = Lateral. For left shank and thigh, +X = anterior, +Y = proximal, +Z = medial
    
    gyr1_Z = R_body_IMU_1 * gyr1(:,ind_1_flex:ind_2_flex); gyr1_Z = gyr1_Z(3,:); %Thigh angular velocity in thigh's Z direction
    gyr2_Z = R_body_IMU_2 * gyr2(:,ind_1_flex+t21 : ind_2_flex+t21); gyr2_Z = gyr2_Z(3,:); %Shank angular velocity in shank's Z direction.
    
    acc1_Y = R_body_IMU_1 * acc1(:,ind_1_flex:ind_2_flex); 
    acc1_Y = acc1_Y(2,:); %Thigh acceleration in thigh's Y direction
    acc2_Y = R_body_IMU_2 * acc2(:,ind_1_flex+t21 : ind_2_flex+t21); 
    acc2_Y = acc2_Y(2,:); %Shank acceleration in shank's Y direction.
    
    zci = @(v) find(v(:).*circshift(v(:), [-1 0]) <= 0);   % function to find indices where angular velocity vector crosses 0.
    
    zero_gyr1 = zci(gyr1_Z)-1; %indices in gyr1_Z vector that are zero-crossings
    zero_gyr2 = zci(gyr2_Z)-1; %indicies in gyr2_Z vector that are zero-crossings
    
    for i = 1:2:length(zero_gyr1) %IMU1 Analysis
        if gyr1_Z(zero_gyr1(i)) < gyr1_Z(zero_gyr1(i)+1)  %hip is in flexion for first zero-crossing. extension for 2nd, flexion for 3rd etc.
            acc1_y_flex((i+1)/2) = acc1_Y(zero_gyr1(i));
            acc1_y_ext((i+1)/2) = acc1_Y(zero_gyr1(i+1));
            
        elseif gyr1_Z(zero_gyr1(i)) > gyr1_Z(zero_gyr1(i)+1) %hip is in extension for first zero-crossing. flexion for 2nd, extension for 3rd etc.
            acc1_y_ext((i+1)/2) = acc1_Y(zero_gyr1(i));
            acc1_y_flex((i+1)/2) = acc1_Y(zero_gyr1(i+1));
        end
    end
    
    for i = 1:2:length(zero_gyr2) %IMU2 Analysis
        if gyr2_Z(zero_gyr2(i)) < gyr2_Z(zero_gyr2(i)+1)  %hip is in flexion for first zero-crossing. extension for 2nd, flexion for 3rd etc.
            acc2_y_flex((i+1)/2) = acc2_Y(zero_gyr2(i));
            acc2_y_ext((i+1)/2) = acc2_Y(zero_gyr2(i+1));
            
        elseif gyr2_Z(zero_gyr2(i)) > gyr2_Z(zero_gyr2(i)+1) %hip is in extension for first zero-crossing. flexion for 2nd, extension for 3rd etc.
            acc2_y_ext((i+1)/2) = acc2_Y(zero_gyr2(i));
            acc2_y_flex((i+1)/2) = acc2_Y(zero_gyr2(i+1));
        end
    end
    
    no_change1 = 0; %votes for no sign change
    change1 = 0; % votes for sign change
    
    no_change2 = 0;
    change2 = 0;
    %if the acceleration in the y direction of the thigh during flexion is
    %greater than acceleration in the y direction during extension then no sign change should occur
    %However, if acc_y_flex < acc_y_ext, sign change should occur
    
    %IMU1
    if length(acc1_y_flex) >= length(acc1_y_ext)  %if acc1_y_flex vector is equal to or greater in length than acc1_ext, use the shorter vector -> extension
        for i = 1:length(acc1_y_ext) %use the shorter vector (acc1_y_flex vs. acc1_y_ext)
            if acc1_y_flex(i) > acc1_y_ext
                no_change1 = no_change1 + 1;
            elseif acc1_y_flex(i) < acc1_y_ext
                change1 = change1 + 1;
            end
        end
    else
        for i = 1:length(acc1_y_flex)  %the shorter vector is _flex
            if acc1_y_flex(i) > acc1_y_ext
                no_change1 = no_change1 + 1;
            elseif acc1_y_flex(i) < acc1_y_ext
                change1 = change1 + 1;
            end
        end
    end
    
    if change1 > no_change1
        R_body_IMU_1 = vertcat(-X_pca1,Y1, -Z1);  %signs flip
    else
        R_body_IMU_1 = R_body_IMU_1;
    end
    
    %IMU2
    if length(acc2_y_flex) >= length(acc2_y_ext)  %if acc2_y_flex vector is equal to or greater in length than acc2_ext, use the shorter vector -> extension
        for i = 1:length(acc2_y_ext) %use the shorter vector (acc1_y_flex vs. acc1_y_ext)
            if acc2_y_flex(i) > acc2_y_ext
                no_change2 = no_change2 + 1;
            elseif acc2_y_flex(i) < acc2_y_ext
                change2 = change2 + 1;
            end
        end
    else
        for i = 1:length(acc2_y_flex)
            if acc2_y_flex(i) > acc2_y_ext
                no_change2 = no_change2 + 1;
            elseif acc2_y_flex(i) < acc2_y_ext
                change2 = change2 + 1;
            end
        end
    end
    
    if change2 > no_change2
        R_body_IMU_2 = vertcat(-X_pca2,Y2, -Z2);
    else
        R_body_IMU_2 = R_body_IMU_2;
    end
 
    R_body_IMU_1_all{n,1} = R_body_IMU_1; %10 X N cell array of 3x3 DCMs for IMU1 for the 10 trials
    R_body_IMU_2_all{n,1} = R_body_IMU_2;
    
    trial_number = strcat('tr',num2str(n));
    thigh_index_stand.(trial_number) = [ind_1_stand, ind_1_stand - 1 + length(ind_1_stand:ind_2_stand)];
    thigh_index_flex.(trial_number) = [ind_1_flex, ind_1_flex - 1 + length(ind_1_flex:ind_2_flex)];
    shank_index_stand.(trial_number) = [ind_1_stand + t21  , ind_1_stand - 1 + t21 + length(ind_1_stand:ind_2_stand)];
    shank_index_flex.(trial_number) = [ind_1_flex + t21 , ind_1_flex - 1 + t21 + length(ind_1_flex:ind_2_flex)];
end
%% STORE DCMs in structure arrays, accessed by IMU#,configuration, and trial (i.e. DCM1.AA.tr4 = IMU #1, configuration AA, tr4)
% STORE DCM Euler angle in structure arrays accessed by IMU# and
% configuration. Output is a 10x3 matrix, each row is the rotations in the x,
% y and z.


for i = 1:num_trials
    trial_names{i} = strcat('tr',num2str(i));
end
DCM1 = cell2struct(R_body_IMU_1_all, trial_names);
DCM2 = cell2struct(R_body_IMU_2_all, trial_names);

trialDCMs = struct('DCM1T',DCM1,'DCM2S',DCM2, 'thigh_index_stand', thigh_index_stand, 'thigh_index_flex', thigh_index_flex, 'shank_index_stand', shank_index_stand, 'shank_index_flex', shank_index_flex);
end
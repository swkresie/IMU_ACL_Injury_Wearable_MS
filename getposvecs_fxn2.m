%% README
% This function is used during the analysis of RATA data for the mechanical
% test rig with soft-tissue impemented on the body segments as well as for
% RATA data from human subject testing. This function takes the output of
% importData_fxn.m as one of its inputs.

% Another input is a 1xn vector
%containing a series of the button presses recorded by IMU #2 which
% indicate the sections of data used for the position vector estimation.
%
%Another input is the DCM structure, the output of the getDCM_fxn.m function.
%

function trialPosvecs = getposvecs_fxn2(data,DCMs,button_presses)


if ~exist('button_presses','var')
    % second argument does not exist so default to single calibration
    button_presses = [7:10]; %button press #s for when DCM calib data was being calculated
    num_trials = 1;
else
    num_trials = length(button_presses)/4;
end

%% IMU#1 Filter acceleration and gyro data
%acceleration
acc_1x = data.sensors1.accelerometerX.*9.81;
acc_1y = data.sensors1.accelerometerY.*9.81;
acc_1z = data.sensors1.accelerometerZ.*9.81;
acc1 = data.sensors1.accelerometerVector;


%UNFILTERED ACC #1
acc1_unf = horzcat(acc_1x,acc_1y,acc_1z);
acc1_unf = acc1_unf'; %3XN array convention


acc1 = acc1'; % Make it a 3 X N array, will be pre-multiplied by a 3x3 Rotation Matrix
acc1 = acc1 .* 9.81; % convert g's to m/s^2
%angular velocity
gyr_1x = data.sensors1.gyroscopeX;
gyr_1y = data.sensors1.gyroscopeY;
gyr_1z = data.sensors1.gyroscopeZ;
gyr1 = data.sensors1.gyroscopeVector;

%UNFILTERED GYRO #1%%%
gyr1_unf = horzcat(gyr_1x, gyr_1y, gyr_1z);
gyr1_unf = gyr1_unf';


gyr1 = gyr1'; % Make it a 3 X N array, will be pre-multipled by a 3x3 rotation marix.

%% IMU#2 Filter acceeration and gyro data

%acceleration
acc_2x = data.sensors2.accelerometerX.*9.81;
acc_2y = data.sensors2.accelerometerY.*9.81;
acc_2z = data.sensors2.accelerometerZ.*9.81;
acc2 = data.sensors2.accelerometerVector;

%UNFILTERED ACC #2%%%%%%
acc2_unf = horzcat(acc_2x,acc_2y,acc_2z);
acc2_unf = acc2_unf'; %3XN array convention


acc2 = acc2';
acc2 = acc2 .* 9.81; % convert g's to m/s^2



%angular velocity

gyr_2x = data.sensors2.gyroscopeX;
gyr_2y = data.sensors2.gyroscopeY;
gyr_2z = data.sensors2.gyroscopeZ;
gyr2 = data.sensors2.gyroscopeVector;

%UNFILTERED GYRO #1%%%
gyr2_unf = horzcat(gyr_2x, gyr_2y, gyr_2z);
gyr2_unf = gyr2_unf'; %3XN Convention

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

% presses 7-10 are repeated for consecutive trials

for i = 1:num_trials
    
    ind_shankiso_start(i,1) = data.button1(button_presses(4*(i - 1) + 1)); %default num_trials = 1, start is button_presses(1) which is the 3rd button press.
    ind_shankiso_end(i,1) = data.button1(button_presses(4*(i - 1) + 2)); %default num_trials = 1, start is button_presses(2) which is the 4th button press.
    ind_thighiso_start(i,1) = data.button1(button_presses(4*(i - 1) + 3));
    ind_thighiso_end(i,1) = data.button1(button_presses(4*(i - 1) + 4));
    
end

%% POS. VECTOR ESTIMATION

for n = 1:num_trials %default, num_trials = 1
    trial_number = strcat('tr', num2str(n));
    
    %% LOAD DCMs for IMU #1 and IMU#2
    R_body_IMU_1 = DCMs.DCM1T.(trial_number);
    R_body_IMU_2 = DCMs.DCM2S.(trial_number);
    
    %% Indices for shank isolated movement data (for shank vector estimation)
    % & indicies for thigh isolate movement data (for thigh vec estimation)
    ind_1_squat = round(ind_shankiso_start(n) + ((ind_shankiso_end(n) - ind_shankiso_start(n))/4)); % only take middle 50% of data to avoid noise at the end points.
    ind_2_squat = round(ind_shankiso_end(n) - ((ind_shankiso_end(n) - ind_shankiso_start(n))/4)); %SHANK ISOLATION, SHANK VECTOR ESTIMATION
    ind_3_squat = round(ind_thighiso_start(n) + ((ind_thighiso_end(n) - ind_thighiso_start(n))/4));
    ind_4_squat = round(ind_thighiso_end(n) - ((ind_thighiso_end(n) - ind_thighiso_start(n))/4));
    
    %% Define variables in ANATOMICAL REF. FRAME (acceleration, angular velocity, and angular acceleration vectors)
    
    
    %clear prev. trials data
    clear acc1_ana_s
    clear acc2_ana_s
    clear gyr1_ana_s
    clear gyr2_ana_s
    
    clear acc1_ana_t
    clear acc2_ana_t
    clear gyr1_ana_t
    clear gyr2_ana_t
    
    clear quats_1_trial
    clear quats_1_conj_trial
    clear quats_2_trial
    clear quats_2_conj_trial
    
    clear quats_1_trial_t
    clear quats_1_conj_trial_t
    clear quats_2_trial_t
    clear quats_2_conj_trial_t
    
    %% FOR SHANK POS.VECTOR
    acc1_ana_s = R_body_IMU_1 * acc1(:,(ind_1_squat:ind_2_squat));
    gyr1_ana_s = deg2rad(R_body_IMU_1 * gyr1(:,ind_1_squat:ind_2_squat)); %convert to radians
    
    acc2_ana_s = R_body_IMU_2 * acc2(:,(ind_1_squat+t21:ind_2_squat+t21));
    gyr2_ana_s = deg2rad(R_body_IMU_2 * gyr2(:,(ind_1_squat+t21:ind_2_squat+t21))); %convert to radians
    
    %% FOR THIGH POS. VECTOR
    acc1_ana_t = R_body_IMU_1 * acc1(:,(ind_3_squat:ind_4_squat));
    gyr1_ana_t = deg2rad(R_body_IMU_1 * gyr1(:,ind_3_squat:ind_4_squat)); %convert to radians
    
    acc2_ana_t = R_body_IMU_2 * acc2(:,(ind_3_squat+t21:ind_4_squat+t21));
    gyr2_ana_t = deg2rad(R_body_IMU_2 * gyr2(:,(ind_3_squat+t21:ind_4_squat+t21))); %convert to radians
    
    
    %% QUATERNIONS
    quats_1_trial = quats_1(ind_1_squat:ind_2_squat,:);
    quats_1_conj_trial = quats_1_conj(ind_1_squat:ind_2_squat,:);
    quats_2_trial = quats_2(ind_1_squat + t21: ind_2_squat+t21,:);
    quats_2_conj_trial = quats_2_conj(ind_1_squat + t21: ind_2_squat+ t21,:);
    
    quats_1_trial_t = quats_1(ind_3_squat:ind_4_squat,:);
    quats_1_conj_trial_t = quats_1_conj(ind_3_squat:ind_4_squat,:);
    quats_2_trial_t = quats_2(ind_3_squat + t21: ind_4_squat+t21,:);
    quats_2_conj_trial_t = quats_2_conj(ind_3_squat + t21: ind_4_squat+ t21,:);
    
    
    
    %% Extract current trial's quaternions ==> convert to DCM
    
    DCM_1S_trial_s = zeros(3,3,ind_2_squat - ind_1_squat + 1);
    DCM_1S_trial_t = zeros(3,3,ind_4_squat - ind_3_squat + 1);
    
    DCM_2T_trial = zeros(3,3,ind_4_squat - ind_3_squat + 1);
    
    for i = 1:(ind_2_squat-ind_1_squat+1) %Indices in IMU1 timeframe%%%%% CHANGE THIS SO THAT DCM_1S_all is extracted for each trial, DCM_2T_all (i +t21) is extracted for each trial
        DCM_1S_trial_s(:,:,i) = R_body_IMU_2 * quat2dcm(quatmultiply(quats_1_trial(i,:),quats_2_conj_trial(i,:)));  %IMU1 -> IMU2 -> Shank Frame    end
    end
    for i = 1:(ind_4_squat - ind_3_squat + 1) %Indices in IMU1 timeframe
        DCM_1S_trial_t(:,:,i) = R_body_IMU_2 * quat2dcm(quatmultiply(quats_1_trial_t(i,:),quats_2_conj_trial_t(i,:)));  %IMU1 -> IMU2 -> Shank Frame    end
        
        DCM_2T_trial(:,:,i) = R_body_IMU_1 * quat2dcm(quatmultiply(quats_2_trial_t(i,:),quats_1_conj_trial_t(i,:))); % IMU2 -> IMU1 -> Thigh frame    end
    end
    %% Filtering cutoff freq
    cutoff_freq = 5;
    fc_acc1_x = cutoff_freq;
    fc_acc1_y = cutoff_freq;
    fc_gyr1_z = cutoff_freq;
    
    fc_acc2_x = cutoff_freq;
    fc_acc2_y = cutoff_freq;
    fc_gyr2_z = cutoff_freq;
    
    %% FILTER ACCELERATION, GYRO,  DATA for SHANK ESTIMATION
    fs = 400;
    
    %Thigh
    fc = fc_acc1_x;
    [b,a] = butter(2, fc/(fs/2));
    clear acc1_ana_f_s %clear previous trial data
    clear acc1_ana_f_t
    acc1_ana_f_s(1,:) = filtfilt(b,a, acc1_ana_s(1,:)); %filter x-component of thigh IMU acceleration;
    acc1_ana_f_t(1,:) = filtfilt(b,a, acc1_ana_t(1,:)); %filter x-component of thigh IMU acceleration;
    
    
    fc = fc_acc1_y;
    [b,a] = butter(2, fc/(fs/2));
    acc1_ana_f_s(2,:) = filtfilt(b,a, acc1_ana_s(2,:)); % for shank pos. vector estimation
    acc1_ana_f_s(3,:) = filtfilt(b,a,acc1_ana_s(3,:));
    acc1_ana_f_t(2,:) = filtfilt(b,a, acc1_ana_t(2,:)); % for thigh pos. vector estimation
    acc1_ana_f_t(3,:) = filtfilt(b,a,acc1_ana_t(3,:));
    
    fc = fc_gyr1_z;
    [b,a] = butter(2, fc/(fs/2));
    clear gyr1_ana_f_s %clear previous trial data
    clear gyr1_ana_f_t %clear previous trial data
    gyr1_ana_f_s(3,:) = filtfilt(b,a, gyr1_ana_s(3,:)); %filter x-component of thigh IMU acceleration;
    gyr1_ana_f_t(3,:) = filtfilt(b,a, gyr1_ana_t(3,:)); %filter x-component of thigh IMU acceleration;
    
    
    gyr1_ana_f_s(2,:) = filtfilt(b,a, gyr1_ana_s(2,:)); %for shank pos. vector estimation
    gyr1_ana_f_s(1,:) = filtfilt(b,a, gyr1_ana_s(1,:));
    gyr1_ana_f_t(2,:) = filtfilt(b,a, gyr1_ana_t(2,:)); %for thigh pos. vector estimation
    gyr1_ana_f_t(1,:) = filtfilt(b,a, gyr1_ana_t(1,:));
    
    %Shank
    fc = fc_acc2_x;
    [b,a] = butter(2, fc/(fs/2));
    clear acc2_ana_f_s  %clear previous trial data
    clear acc2_ana_f_t
    acc2_ana_f_s(1,:) = filtfilt(b,a, acc2_ana_s(1,:)); %filter x-component of thigh IMU acceleration;
    acc2_ana_f_t(1,:) = filtfilt(b,a, acc2_ana_t(1,:)); %filter x-component of thigh IMU acceleration;
    
    fc = fc_acc2_y;
    [b,a] = butter(2, fc/(fs/2));
    acc2_ana_f_s(2,:) = filtfilt(b,a, acc2_ana_s(2,:)); %for shank pos. vector estimation
    acc2_ana_f_s(3,:) = filtfilt(b,a, acc2_ana_s(3,:));
    acc2_ana_f_t(2,:) = filtfilt(b,a, acc2_ana_t(2,:)); %for thigh pos. vector estimation
    acc2_ana_f_t(3,:) = filtfilt(b,a, acc2_ana_t(3,:));
    
    fc = fc_gyr2_z;
    [b,a] = butter(2, fc/(fs/2));
    clear gyr2_ana_f_s  %clear previous trial data
    clear gyr2_ana_f_t  %clear previous trial data
    
    gyr2_ana_f_s(3,:) = filtfilt(b,a, gyr2_ana_s(3,:)); %filter x-component of thigh IMU acceleration;
    gyr2_ana_f_t(3,:) = filtfilt(b,a, gyr2_ana_t(3,:)); %filter x-component of thigh IMU acceleration;
    
    
    gyr2_ana_f_s(2,:) = filtfilt(b,a, gyr2_ana_s(2,:));
    gyr2_ana_f_s(1,:) = filtfilt(b,a, gyr2_ana_s(1,:));
    gyr2_ana_f_t(2,:) = filtfilt(b,a, gyr2_ana_t(2,:));
    gyr2_ana_f_t(1,:) = filtfilt(b,a, gyr2_ana_t(1,:));
    
    
    %% Calculate Angular Acceleration from filtered gyro
    dt = 0.0025; %400Hz sample rate
    clear alpha1_ana_s
    clear alpha2_ana_s
    clear alpha1_ana_t
    clear alpha2_ana_t
    
    alpha1_ana_s = gradient(gyr1_ana_f_s,dt);
    alpha2_ana_s = gradient(gyr2_ana_f_s,dt);
    alpha1_ana_t = gradient(gyr1_ana_f_t,dt);
    alpha2_ana_t = gradient(gyr2_ana_f_t,dt);
    
    
    %% Shank Vector Estimation (Y-component)
    for k = 1:2 
%algorithm run 2x , first time for Y component minimizing acceleration in x direction of shank, ...
%2nd time  for x component minimizing acceleration in y direction
        
        %% Calculate IMU RATA
        clear acc1_f_s
        clear gyr1_f_s
        clear alpha1_f_s
        clear acc2_f_s
        clear gyr2_f_s
        clear alpha2_f_s
        
        clear acc1_f_t
        clear gyr1_f_t
        clear alpha1_f_t
        clear acc2_f_t
        clear gyr2_f_t
        clear alpha2_f_t
        
        for i = 1:length(acc1_ana_f_s)
            acc1_f_s(:,i) = inv(R_body_IMU_1) * (acc1_ana_f_s(:,i));
            gyr1_f_s(:,i) = inv(R_body_IMU_1) * (gyr1_ana_f_s(:,i));
            alpha1_f_s(:,i) =inv(R_body_IMU_1) *(alpha1_ana_s(:,i));
        end
        
        for i = 1:length(acc1_ana_f_t)
            acc1_f_t(:,i) = inv(R_body_IMU_1) * (acc1_ana_f_t(:,i));
            gyr1_f_t(:,i) = inv(R_body_IMU_1) * (gyr1_ana_f_t(:,i));
            alpha1_f_t(:,i) =inv(R_body_IMU_1) *(alpha1_ana_t(:,i));
            
            acc2_f_t(:,i) = inv(R_body_IMU_2) * (acc2_ana_f_t(:,i));
            gyr2_f_t(:,i) = inv(R_body_IMU_2) * (gyr2_ana_f_t(:,i));
            alpha2_f_t(:,i) =inv(R_body_IMU_2) *(alpha2_ana_t(:,i));
        end
        
        
        range_t = ind_1_squat:ind_2_squat;
        DCM1S_11 = reshape(DCM_1S_trial_s(1,1,:),[1,length(range_t)]);
        DCM1S_12 = reshape(DCM_1S_trial_s(1,2,:),[1,length(range_t)]);
        DCM1S_13 = reshape(DCM_1S_trial_s(1,3,:),[1,length(range_t)]);
        %
        fx = @(p) (sum(abs( (acc2_ana_s(1,:) - gyr2_ana_s(3,:).^2 .* p(4) - alpha2_ana_s(3, :) .* p(5)) - ...
            (DCM1S_11 .* (acc1_f_s(1,:) + gyr1_f_s(1,:) .* gyr1_f_s(2,:).* p(2) - gyr1_f_s(2,:).^2 .* p(1) - gyr1_f_s(3,:).^2 .* p(1) + gyr1_f_s(1,:) .* gyr1_f_s(3,:) .* p(3) + alpha1_f_s(2, :) .* p(3) - alpha1_f_s(3,:) .* p(2)) +...
            DCM1S_12 .* (acc1_f_s(2,:) + gyr1_f_s(1,:) .* gyr1_f_s(2,:).* p(1) - gyr1_f_s(1,:).^2 .* p(2) + gyr1_f_s(2,:).* gyr1_f_s(3,:).*p(3) - gyr1_f_s(3,:).^2 .* p(2) + alpha1_f_s(3,:) .* p(1) - alpha1_f_s(1,:) .* p(3)) + ...
            DCM1S_13 .* (acc1_f_s(3,:) + gyr1_f_s(1,:) .* gyr1_f_s(3,:).* p(1) - gyr1_f_s(1,:).^2 .* p(3) +  gyr1_f_s(2,:).*gyr1_f_s(3,:).* p(2) - gyr1_f_s(2,:).^2 .* p(3) + alpha1_f_s(1,:) .* p(2) - alpha1_f_s(2,:) .* p(1))))));
        
        DCM1S_21 = reshape(DCM_1S_trial_s(2,1,:),[1,length(range_t)]);
        DCM1S_22 = reshape(DCM_1S_trial_s(2,2,:),[1,length(range_t)]);
        DCM1S_23 = reshape(DCM_1S_trial_s(2,3,:),[1,length(range_t)]);
        
        fy = @(p) (sum(abs( (acc2_ana_s(2,:) - gyr2_ana_s(3,:).^2 .* p(5) + alpha2_ana_s(3, :) .* p(4)) - ...
            (DCM1S_21 .* (acc1_f_s(1,:) + gyr1_f_s(1,:) .* gyr1_f_s(2,:).* p(2) - gyr1_f_s(2,:).^2 .* p(1) - gyr1_f_s(3,:).^2 .* p(1) + gyr1_f_s(1,:) .* gyr1_f_s(3,:) .* p(3) + alpha1_f_s(2, :) .* p(3) - alpha1_f_s(3,:) .* p(2)) +...
            DCM1S_22 .* (acc1_f_s(2,:) + gyr1_f_s(1,:) .* gyr1_f_s(2,:).* p(1) - gyr1_f_s(1,:).^2 .* p(2) + gyr1_f_s(2,:).* gyr1_f_s(3,:).*p(3) - gyr1_f_s(3,:).^2 .* p(2) + alpha1_f_s(3,:) .* p(1) - alpha1_f_s(1,:) .* p(3)) + ...
            DCM1S_23 .* (acc1_f_s(3,:) + gyr1_f_s(1,:) .* gyr1_f_s(3,:).* p(1) - gyr1_f_s(1,:).^2 .* p(3) +  gyr1_f_s(2,:).*gyr1_f_s(3,:).* p(2) - gyr1_f_s(2,:).^2 .* p(3) + alpha1_f_s(1,:) .* p(2) - alpha1_f_s(2,:) .* p(1))))));
        
        
        if k == 1
            f = fx;
        elseif k == 2
            f = fy;
        end
        
        lb_thigh_frame = [0,-0.35,0];
        lb_IMU1_frame = R_body_IMU_1 \ lb_thigh_frame'; %pre-multiply by inverse DCM to go from Thigh frame back to IMU1 Frame
        lb_IMU1_frame = lb_IMU1_frame';
        ub_thigh_frame = [0, -0.05, 0];
        ub_IMU1_frame = R_body_IMU_1 \ ub_thigh_frame';
        ub_IMU1_frame = ub_IMU1_frame';
        
        for ii = 1:3
            if lb_IMU1_frame(1,ii) > ub_IMU1_frame(1,ii)  %due to the different reperesentation in different frames, ensure the lower and upper bounds in the IMU1 frame are actually lower and upper bounds, otherwise flip them to make them as such
                val_change_ub = ub_IMU1_frame(1,ii); % the inequality may not be true if
                val_change_lb = lb_IMU1_frame(1,ii);
                lb_IMU1_frame(1,ii) = val_change_ub;
                ub_IMU1_frame(1,ii) = val_change_lb;
            end
        end
        
        j = 1;
        
        lb = horzcat(lb_IMU1_frame,[-0.1,0]); %Initial bounds
        ub =  horzcat(ub_IMU1_frame,[0.1,0.35]);
        
   
        x0 = horzcat((lb_IMU1_frame + ub_IMU1_frame)./2,[0.01,0.15]);
   
        
        
        %x0 = horzcat(lb_IMU1_frame + ((ub_IMU1_frame - lb_IMU1_frame)./2),[0,0.15,0]);
        options = optimset('MaxIter',1000); % maximum 1000 iterations to minimize objective function
        vecs_cell{j} = fminsearchbnd(f,x0,lb,ub,options);
        x_shank(j) = vecs_cell{1,j}(4);
        y_shank(j) = vecs_cell{1,j}(5);
        % z_shank(j) = vecs_cell{1,j}(6);
        x_thigh_s(j) = R_body_IMU_1(1,:) * [vecs_cell{1,j}(1); vecs_cell{1,j}(2); vecs_cell{1,j}(3)];
        y_thigh_s(j) = R_body_IMU_1(2,:) * [vecs_cell{1,j}(1); vecs_cell{1,j}(2); vecs_cell{1,j}(3)];
        z_thigh_s(j) = R_body_IMU_1(3,:) * [vecs_cell{1,j}(1); vecs_cell{1,j}(2); vecs_cell{1,j}(3)];
        
        
        acc1_shank_frame(:,j) = DCM_1S_trial_s(:,:,j) * acc1(:,j+ind_1_squat-1);
        
        if k == 1
            best_shank_pos_vec_y = y_shank;
        elseif k == 2
            best_shank_pos_vec_x = x_shank;
        end
    end
    v = [vecs_cell{1,1}(1),vecs_cell{1,1}(2),vecs_cell{1,1}(3),vecs_cell{1,1}(4),vecs_cell{1,1}(5)];
    
    RATA.(trial_number) = (acc2_ana_s(1,:) - gyr2_ana_s(3,:).^2 .* v(4) - alpha2_ana_s(3, :) .* v(5)) - ...
        (DCM1S_11 .* (acc1_f_s(1,:) + gyr1_f_s(1,:) .* gyr1_f_s(2,:).* v(2) - gyr1_f_s(2,:).^2 .* v(1) - gyr1_f_s(3,:).^2 .* v(1) + gyr1_f_s(1,:) .* gyr1_f_s(3,:) .* v(3) + alpha1_f_s(2, :) .* v(3) - alpha1_f_s(3,:) .* v(2)) +...
        DCM1S_12 .* (acc1_f_s(2,:) + gyr1_f_s(1,:) .* gyr1_f_s(2,:).* v(1) - gyr1_f_s(1,:).^2 .* v(2) + gyr1_f_s(2,:).* gyr1_f_s(3,:).*v(3) - gyr1_f_s(3,:).^2 .* v(2) + alpha1_f_s(3,:) .* v(1) - alpha1_f_s(1,:) .* v(3)) + ...
        DCM1S_13 .* (acc1_f_s(3,:) + gyr1_f_s(1,:) .* gyr1_f_s(3,:).* v(1) - gyr1_f_s(1,:).^2 .* v(3) +  gyr1_f_s(2,:).*gyr1_f_s(3,:).* v(2) - gyr1_f_s(2,:).^2 .* v(3) + alpha1_f_s(1,:) .* v(2) - alpha1_f_s(2,:) .* v(1)));
    y_shank_all.(trial_number) =  y_shank;
    y_thigh_all_shank_opt.(trial_number) = y_thigh_s;
    shank_index.(trial_number) = [(ind_1_squat + t21 -1) , (ind_1_squat + t21 -1 + length(ind_1_squat:ind_2_squat))];
    
    %%
   
    
    %  disp(best_shank_pos_vec_y);
    
    %%

    %disp(best_shank_pos_vec_x);
    x_shank_all.(trial_number) =  x_shank;
    clear vecs_cell x_shank y_shank z_shank
    
    %%  FINALIZED SHANK X AND Y COMPONENTS
    best_shank_pos_vec_y_remout = best_shank_pos_vec_y(~isoutlier(best_shank_pos_vec_y)); %remove outliers
    
    if isrow(best_shank_pos_vec_x)
        shank_pos_vec_all{n,1} = struct('x', best_shank_pos_vec_x', 'y', best_shank_pos_vec_y'); %if the best pos vecs are saved in a row vector, change them to column vectors
        shank_pos_vec_all_remout{n,1} = struct( 'x', best_shank_pos_vec_x', 'y', best_shank_pos_vec_y_remout');
    else
        shank_pos_vec_all{n,1} = struct('x', best_shank_pos_vec_x, 'y', best_shank_pos_vec_y);
        shank_pos_vec_all_remout{n,1} = struct( 'x', best_shank_pos_vec_x, 'y', best_shank_pos_vec_y_remout); %outliers removed
    end
    shank_pos_vec_xy{n,1} = [mean(best_shank_pos_vec_x), mean(best_shank_pos_vec_y)]; %store the x and y components of the shank position vector in a cell, 1 cell per trial.
    
    shank_pos_vec_xy_remout{n,1} = [mean(best_shank_pos_vec_x), mean(best_shank_pos_vec_y_remout)]; % outliers for y -component removed
    
    
    %%  Thigh Vector Estimation (Y-component)
    for k = 1:2
        
        range_t = ind_3_squat:ind_4_squat;
        DCM2T_11 = reshape(DCM_2T_trial(1,1,:),[1,length(range_t)]);
        DCM2T_12 = reshape(DCM_2T_trial(1,2,:),[1,length(range_t)]);
        DCM2T_13 = reshape(DCM_2T_trial(1,3,:),[1,length(range_t)]);
        %
        fx = @(p) (sum(abs( (acc1_ana_t(1,:) - gyr1_ana_t(3,:).^2 .* p(4) - alpha1_ana_t(3, :) .* p(5)) - ...
            (DCM2T_11 .* (acc2_f_t(1,:) + gyr2_f_t(1,:) .* gyr2_f_t(2,:).* p(2) - gyr2_f_t(2,:).^2 .* p(1) - gyr2_f_t(3,:).^2 .* p(1) + gyr2_f_t(1,:) .* gyr2_f_t(3,:) .* p(3) + alpha2_f_t(2, :) .* p(3) - alpha2_f_t(3,:) .* p(2)) +...
            DCM2T_12 .* (acc2_f_t(2,:) + gyr2_f_t(1,:) .* gyr2_f_t(2,:).* p(1) - gyr2_f_t(1,:).^2 .* p(2) + gyr2_f_t(2,:).* gyr2_f_t(3,:).*p(3) - gyr2_f_t(3,:).^2 .* p(2) + alpha2_f_t(3,:) .* p(1) - alpha2_f_t(1,:) .* p(3)) + ...
            DCM2T_13 .* (acc2_f_t(3,:) + gyr2_f_t(1,:) .* gyr2_f_t(3,:).* p(1) - gyr2_f_t(1,:).^2 .* p(3) +  gyr2_f_t(2,:).*gyr2_f_t(3,:).* p(2) - gyr2_f_t(2,:).^2 .* p(3) + alpha2_f_t(1,:) .* p(2) - alpha2_f_t(2,:) .* p(1))))));
        
        DCM2T_21 = reshape(DCM_2T_trial(2,1,:),[1,length(range_t)]);
        DCM2T_22 = reshape(DCM_2T_trial(2,2,:),[1,length(range_t)]);
        DCM2T_23 = reshape(DCM_2T_trial(2,3,:),[1,length(range_t)]);
        
        fy = @(p) (sum(abs( (acc1_ana_t(2,:) - gyr1_ana_t(3,:).^2 .* p(5) + alpha1_ana_t(3, :) .* p(4)) - ...
            (DCM2T_21 .* (acc2_f_t(1,:) + gyr2_f_t(1,:) .* gyr2_f_t(2,:).* p(2) - gyr2_f_t(2,:).^2 .* p(1) - gyr2_f_t(3,:).^2 .* p(1) + gyr2_f_t(1,:) .* gyr2_f_t(3,:) .* p(3) + alpha2_f_t(2, :) .* p(3) - alpha2_f_t(3,:) .* p(2)) +...
            DCM2T_22 .* (acc2_f_t(2,:) + gyr2_f_t(1,:) .* gyr2_f_t(2,:).* p(1) - gyr2_f_t(1,:).^2 .* p(2) + gyr2_f_t(2,:).* gyr2_f_t(3,:).*p(3) - gyr2_f_t(3,:).^2 .* p(2) + alpha2_f_t(3,:) .* p(1) - alpha2_f_t(1,:) .* p(3)) + ...
            DCM2T_23 .* (acc2_f_t(3,:) + gyr2_f_t(1,:) .* gyr2_f_t(3,:).* p(1) - gyr2_f_t(1,:).^2 .* p(3) +  gyr2_f_t(2,:).*gyr2_f_t(3,:).* p(2) - gyr2_f_t(2,:).^2 .* p(3) + alpha2_f_t(1,:) .* p(2) - alpha2_f_t(2,:) .* p(1))))));
        
        if k == 1
            f = fx;
        elseif k == 2
            f = fy;
        end
        
        lb_thigh_frame = [-0.07,-0.35];
        
        
        ub_thigh_frame = [0.07, 0];
        
        
        lb_shank_frame = [best_shank_pos_vec_x-0.01,best_shank_pos_vec_y-0.01,0];
        lb_IMU2_frame = R_body_IMU_2 \ lb_shank_frame';
        lb_IMU2_frame = lb_IMU2_frame';
        
        ub_shank_frame = [best_shank_pos_vec_x+0.01,best_shank_pos_vec_y+0.01,0];
        ub_IMU2_frame = R_body_IMU_2 \ ub_shank_frame';
        ub_IMU2_frame = ub_IMU2_frame';
        
        for ii = 1:3
            if lb_IMU2_frame(1,ii) > ub_IMU2_frame(1,ii)
                val_change_ub = ub_IMU2_frame(1,ii);
                val_change_lb = lb_IMU2_frame(1,ii);
                lb_IMU2_frame(1,ii) = val_change_ub;
                ub_IMU2_frame(1,ii) = val_change_lb;
            end
        end
        
        j = 1;
        
        lb = horzcat(lb_IMU2_frame,lb_thigh_frame ); %Initial bounds
        ub =  horzcat(ub_IMU2_frame,ub_thigh_frame);
        
        % use first estimate in second iteration of optimization function for
        % estimating thigh vector y-component.
        
        x0 = horzcat((lb_IMU2_frame + ub_IMU2_frame)./2,[0,-0.175]);
        
        
        options = optimset('MaxIter',1000);
        vecs_cell{j} = fminsearchbnd(f,x0,lb,ub,options);
        x_shank_t(j) = R_body_IMU_2(1,:) * [vecs_cell{1,j}(1);vecs_cell{1,j}(2);vecs_cell{1,j}(3);];
        y_shank_t(j) = R_body_IMU_2(2,:) * [vecs_cell{1,j}(1);vecs_cell{1,j}(2);vecs_cell{1,j}(3);];
        z_shank_t(j) = R_body_IMU_2(3,:) * [vecs_cell{1,j}(1);vecs_cell{1,j}(2);vecs_cell{1,j}(3);];
        x_thigh(j) = vecs_cell{1,j}(4);
        y_thigh(j) = vecs_cell{1,j}(5);
        
        if k == 1
            best_thigh_pos_vec_y = y_thigh;
        elseif k == 2
            best_thigh_pos_vec_x = x_thigh;
        end
        
    end
    y_thigh_all.(trial_number) =  y_thigh;
    y_shank_all_thigh_opt.(trial_number) = y_thigh;
    thigh_index.(trial_number) = [ind_3_squat, ind_3_squat - 1 + length(ind_3_squat:ind_4_squat)];
    
    %% Save best estimates, clear old variables
    
    
    
    %  disp(best_thigh_pos_vec_x);
    % disp(best_thigh_pos_vec_y);
    clear vecs_cell x_thigh y_thigh z_thigh
    
    
    
    %% Save thigh vectors in a structure, All vecs and the mean of vecs for the trial.
    best_thigh_pos_vec_y_remout = best_thigh_pos_vec_y(~isoutlier(best_thigh_pos_vec_y)); %remove outliers
    
    if isrow(best_thigh_pos_vec_x)
        thigh_pos_vec_all{n,1} = struct('x',best_thigh_pos_vec_x','y',best_thigh_pos_vec_y'); %if the best pos vecs are saved in a row vector, change them to column vectors
        thigh_pos_vec_all_remout{n,1} = struct( 'x', best_thigh_pos_vec_x', 'y', best_thigh_pos_vec_y_remout');
    else
        thigh_pos_vec_all{n,1} = struct('x', best_thigh_pos_vec_x, 'y', best_thigh_pos_vec_y);
        thigh_pos_vec_all_remout{n,1} = struct( 'x', best_thigh_pos_vec_x, 'y', best_thigh_pos_vec_y_remout); %outliers removed
    end
    
    thigh_pos_vec_xy{n,1} = [mean(best_thigh_pos_vec_x), mean(best_thigh_pos_vec_y)]; %store finalized x,y components of position vector in a cell , 1 cell per trial
    thigh_pos_vec_xy_remout{n,1} = [mean(best_thigh_pos_vec_x), mean(best_thigh_pos_vec_y_remout)]; % outliers for y -component removed
    
    
    
    clear shank_vector_index_x
    clear shank_vector_index_y
    clear thigh_vector_index_x
    clear thigh_vector_index_y
    
    disp(['trial #' num2str(n), 'complete'])
end

%% STORE FINAL RESULTS IN A STRUCTURE
% Store position vectors for thigh and shank in structure
for i = 1:num_trials
    trial_names{i} = strcat('tr',num2str(i));
end

POSVEC1 = cell2struct(thigh_pos_vec_xy, trial_names);
POSVEC2 = cell2struct(shank_pos_vec_xy, trial_names);

trialPosvecs = struct('posvec1',POSVEC1,'posvec2',POSVEC2);

end
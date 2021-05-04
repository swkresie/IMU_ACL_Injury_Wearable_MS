%% README
%Script to re-analyze mechanical test rig RATA capable of using a different filtering
%technique for both mocap and IMU derived RATA (Yu, 1999).

clear all
close all
data_directory = 'C:\Users\scott\Documents\HPL\IMU\DCM_Protocol_Data\'; %location of IMU data

%% (IMU #1 INFO)
folder_IMU1 = 'IMU1_MECH_02_BBBB'; %foldername for IMU #1 data where DCM for IMU#1 is accessed
folder_IMU2 = 'IMU2_MECH_02_BBBB'; % foldername for IMU #2 data where DCM for IMU#2 is accessed

% folder_IMU1 = 'IMU1_MECH_01_ABAB'; %foldername for IMU #1 data
% folder_IMU2 = 'IMU2_MECH_01_ABAB'; % foldername for IMU #2 data

% folder_IMU1 = 'IMU1_MECH_04_CBCB'; %foldername for IMU #1 data
% folder_IMU2 = 'IMU2_MECH_04_CBCB'; % foldername for IMU #2 data

%% IMPORT DATA
data = importData_fxn(data_directory,folder_IMU1,folder_IMU2); %import data from DCM calibration protocol repeatablity experiment.
%% DCMs (IMU #1) (USER INPUT, button presses  if more than 1 trial collected (optional) )
dcm_button_presses = [3:6,9:12,15:18,21:24,27:30,33:36,39:42, 45:48, 51:54, 57:60]; %% user input here!!%%
DCMs_1 = getDCM_fxn(data, dcm_button_presses);

%% (IMU #2 INFO)
folder_IMU1 = 'IMU1_MECH_02_BBBB'; %foldername for IMU #1 data
folder_IMU2 = 'IMU2_MECH_02_BBBB'; % foldername for IMU #2 data

% folder_IMU1 = 'IMU1_MECH_01_ABAB'; %foldername for IMU #1 data
% folder_IMU2 = 'IMU2_MECH_01_ABAB'; % foldername for IMU #2 data

% folder_IMU1 = 'IMU1_MECH_04_CBCB'; %foldername for IMU #1 data
% folder_IMU2 = 'IMU2_MECH_04_CBCB'; % foldername for IMU #2 data

%% IMPORT DATA
data = importData_fxn(data_directory,folder_IMU1,folder_IMU2);
%% DCMs (= (USER INPUT, button presses  if more than 1 trial collected (optional) )
dcm_button_presses = [3:6,9:12,15:18,21:24,27:30,33:36,39:42, 45:48, 51:54, 57:60]; %% user input here!!%%
DCMs_2 = getDCM_fxn(data, dcm_button_presses);

DCMs = DCMs_1; % start with oriignal DCM structure with correct IMU #1 configuration
DCMs = rmfield(DCMs,'DCM2S'); % remove field for shank DCMs
DCMs.DCM2S = DCMs_2.DCM2S; % use field for shankd DCMs with correct configuration

%% IMU 1 INFO
%% PosVecs (USER INPUT, button presses  if more than 1 trial collected  (optional) )

data_directory = 'C:\Users\scott\Documents\HPL\IMU\POS_VEC_DATA\'; %location of IMU data
clear folder_IMU1
clear folder_IMU2
folder_IMU1 = 'IMU1_MECH_16_BBBB'; %foldername for IMU #1 data
folder_IMU2 = 'IMU2_MECH_16_BBBB'; % foldername for IMU #2 data

% folder_IMU1 = 'IMU1_MECH_14_ABAB'; %foldername for IMU #1 data (BBAB config)
% folder_IMU2 = 'IMU2_MECH_14_ABAB'; % foldername for IMU #2 data (BBAB config)

% folder_IMU1 = 'IMU1_MECH_11_CBCB'; %foldername for IMU #1 data (BBCB config)
% folder_IMU2 = 'IMU2_MECH_11_CBCB'; % foldername for IMU #2 data (BBCB config)


%% IMPORT DATA
data = importData_fxn(data_directory,folder_IMU1,folder_IMU2);
posvec_button_presses = [3:6,9:12,15:18,21:24,27:30,33:36,39:42, 45:48, 51:54, 57:60]; %%user input here!!%%%
posvecs_1 = getposvecs_fxn2(data,DCMs_1, posvec_button_presses);

posvec1 = transpose(struct2mat(posvecs_1.posvec1));

%% IMU 2 INFO
%% POSVEC 2
folder_IMU1 = 'IMU1_MECH_16_BBBB'; %foldername for IMU #1 data (BBBB)
folder_IMU2 = 'IMU2_MECH_16_BBBB'; % foldername for IMU #2 data (BBBB)
% 
% folder_IMU1 = 'IMU1_MECH_14_ABAB'; %foldername for IMU #1 data (BBAB config)
% folder_IMU2 = 'IMU2_MECH_14_ABAB'; % foldername for IMU #2 data (BBAB config)

% folder_IMU1 = 'IMU1_MECH_11_CBCB'; %foldername for IMU #1 data (BBCB config)
% folder_IMU2 = 'IMU2_MECH_11_CBCB'; % foldername for IMU #2 data (BBCB config)

%% IMPORT DATA
data = importData_fxn(data_directory,folder_IMU1,folder_IMU2);
posvec_button_presses = [3:6,9:12,15:18,21:24,27:30,33:36,39:42, 45:48, 51:54, 57:60]; %%user input here!!%%%
posvecs_2 = getposvecs_fxn2(data,DCMs_2, posvec_button_presses);

posvec2 = transpose(struct2mat(posvecs_2.posvec2));

posvecs = posvecs_1; %use first posvecs structure with correct IMU #1 configuration
posvecs = rmfield(posvecs,'posvec2'); % remove posvec2 field and replace with correct IMU # 2 configuration
posvecs.posvec2 = posvecs_2.posvec2;


%% MOCAP RATA 
data_directory = 'C:\Users\scott\Documents\HPL\IMU\MAXTRAQ\'; %location of IMU data
%%% BBBB folder numbers
mocap_num = {'22_1';'23_2'; '24_3';'25_4';'26_5';'27_1';'28_2';'29_3';'30_4';'31_5';'50_1';'51_2';'52_3';'53_4';'54_5';'65_1';'66_2';'67_3';'68_4';'69_5';'70_6';'71_7';'72_8';'73_9';'74_10';}; % BBBB config trials

%%% BBAB folder numbers
%mocap_num = {'37_1';'38_2'; '39_3'; '40_4';'41_5'; '42_1';'43_2';'44_3';'45_4';'46_5'};

%%% BBCB folder numbers
%mocap_num = {'32_1';'33_2'; '34_3'; '35_4'; '36_5'};

rata_button_presses = [3:4]; %% USER INPUT HERE!! %%%
num_RATA_trials = length(mocap_num);
for n = 1:num_RATA_trials %= number of RATA trials collected
    trial_name = ['tr',mocap_num{n}(1:2)];
MOCAP_RATA.(trial_name) = mocap_processing_fxn(['MOCAP_RATA_',mocap_num{n},'_cam1.mqa'], 1);%% USER INPUT HERE!! %%% %mocap_procssing_fxn(ascii filename, marker config), marker config == 1 -> BBBB, BBCB configs, == 2 --> BBAB config
end

%% IMU RATA /Optimal filter frequency
data_directory = 'C:\Users\scott\Documents\HPL\IMU\RATA_IMU_DATA\'; %location of IMU data
clear folder_IMU1
clear folder_IMU2
%% USER INPUT HERE!! %%%
imu_num = {'22';'23';'24';'25';'26';'27';'28';'29';'30';'31';'50';'51';'52';'53';'54';'65';'66';'67';'68';'69';'70';'71';'72';'73';'74';}; %BBBB config
%imu_num = {'37';'38';'39';'40';'41';'42';'43';'44';'45';'46'}; %BBAB config
%imu_num = {'32';'33';'34';'35';'36'}; % BBCB config 
for n = 1:length(imu_num)
    trial_name = ['tr',imu_num{n}];
    folder_IMU1 = ['IMU1_MECH_',imu_num{n},'_BBBB']; %foldername for IMU #1 data %% USER INPUT HERE!! %%%
    folder_IMU2 = ['IMU2_MECH_',imu_num{n},'_BBBB']; % foldername for IMU #2 data%% USER INPUT HERE!! %%%
        rata_button_presses = [3:4]; %% USER INPUT HERE!! %%%
        num_RATA_trials = length(imu_num);
        
        %% OPTIMAL FREQ CODE
        data = importData_fxn(data_directory,folder_IMU1,folder_IMU2);
        freqs = [18];  %%USER INPUT, range of possible optimal freqeuncies to search within.
        trial_ind = 1;
             
        
       trial_fieldname = ['tr', imu_num{n}]; % trial fieldname ('tr22', etc.) of each trial analyzed.

        
        f_ind = 1; %initialize frequency index
        f_vec_length = length(freqs);
        for g = 1:f_vec_length  % cutoff frequency possibilties
            f = freqs(g);
            trial_num = 'tr1';
            %f_fieldname = ['f',num2str(f)];
            
            IMU_RATA_f = getRATA_fxn(data,f,DCMs,posvecs,rata_button_presses);   % calculate RATA values
            
            RATA_pk_IMU(f_ind,1) = max(IMU_RATA_f.RATA.(trial_num)); %access pk IMU RATA
            RATA_pk_mocap(f_ind,1) = max(MOCAP_RATA.(trial_fieldname).RATA_mocap); %access pk mocap RATA
            thigh_pk(f_ind,1) = max(IMU_RATA_f.thigh_acc_kjc_tframe.(trial_num)(2,:)); %max longitudinal thigh acc.
            shank_pk(f_ind,1) = max(IMU_RATA_f.shank_acc_kjc.(trial_num)(2,:)); %max longitudinal shank acc.
            f_ind = f_ind + 1; % f_ind grows by one each iteration
        end
        
        [val,ind] = min(abs(RATA_pk_IMU - RATA_pk_mocap)); %find index where difference in peaks is an absolute minimum.
        optimal_freq(trial_ind,1) = freqs(ind); %for this loop iteration (trial) the optimal frequency is stored
        peaks.IMU.(trial_fieldname) = RATA_pk_IMU;
        peaks.mocap.(trial_fieldname) = RATA_pk_mocap;
        peaks.diff.(trial_fieldname) = RATA_pk_IMU - RATA_pk_mocap;
        peaks.thigh.(trial_fieldname) = thigh_pk(ind);  % peak thigh acc when optimal filter frequency was used
        peaks.shank.(trial_fieldname) = shank_pk(ind); %peak shank acc when optimal filter frequency was used
        trial_ind = trial_ind +1; % trial_ind grows by one each iteration to allow optimal_freq to grow each trial iteration.
        disp(['Completed' ,' ',trial_fieldname]);
        peaks.optimal_freq.(trial_fieldname) = freqs(ind); % optimal filter frequency
        peaks.mindiff.(trial_fieldname) = val;
        peaks.twenty_pct_range.(trial_fieldname) = freqs(abs(peaks.diff.(trial_fieldname)) ./ peaks.mocap.(trial_fieldname) <0.2);  %cutoff freqs that result in less than 20% difference betwen imu and mocap derived RATA
        peaks.fifteen_pct_range.(trial_fieldname) = freqs(abs(peaks.diff.(trial_fieldname)) ./ peaks.mocap.(trial_fieldname) <0.15); % '' less than 15% diff between imu and mocap dervied RATA.
        peaks.ten_pct_range.(trial_fieldname) = freqs( abs(peaks.diff.(trial_fieldname)) ./ peaks.mocap.(trial_fieldname) < 0.1);  % '' less than 10% ''

        
        
        
        
        %% IMU RATA, Single Frequency, or Yu Filtered or Optimal Filtered
        data_directory = 'C:\Users\scott\Documents\HPL\IMU\RATA_IMU_DATA\'; %location of IMU data
        clear folder_IMU1
        clear folder_IMU2
        
        folder_IMU1 = ['IMU1_MECH_',imu_num{n},'_BBBB']; %% USER INPUT HERE!! %%%%foldername for IMU #1 data
        folder_IMU2 = ['IMU2_MECH_',imu_num{n},'_BBBB']; %% USER INPUT HERE!! %%%% foldername for IMU #2 data
        rata_button_presses = [3:4]; %% USER INPUT HERE!! %%%
        num_RATA_trials = length(imu_num);
            
            data = importData_fxn(data_directory,folder_IMU1,folder_IMU2);
            trial_fieldname = ['tr', imu_num{n}];
            optf = peaks.optimal_freq.(trial_fieldname);   %use optimal frequency to calculate RATA         
            IMU_RATA.(trial_fieldname) = getRATA_fxn(data,20,DCMs,posvecs,rata_button_presses);  %getRATA_fxn(sessionData,cutoff filter freq, DCMs, posvecs, button presses in session for RATA trials);
           % peaks.optimal_freq.(trial_fieldname)
            % IMU_RATA.cutoff_freqs
            % optimal_freq
            % peaks.fifteen_pct_range.tr1
            % max_IMU_RATA = max(IMU_RATA.RATA.tr1)
            % max_MOCAP_RATA = max(MOCAP_RATA.tr1.RATA_mocap)
            % (max_IMU_RATA - max_MOCAP_RATA) / max_MOCAP_RATA
            
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Align RATA signals, output peak values for IMU and MOCAP derived RATA and corrcoef(R) and shift amount

%% TIME SYNC MOCAP TO IMU1

for n = 1:num_RATA_trials % = number of RATA trials collected
    %%
    RATA_trial_num = 'tr1';
    trial_fieldname = ['tr', imu_num{n}]; % trial fieldname ('tr22', etc.) of each trial analyzed.

mocap_thigh_y_acc_kjc = MOCAP_RATA.(trial_fieldname).t_mark_mocap_sync;
IMU_thigh_y_acc_kjc = IMU_RATA.(trial_fieldname).thigh_acc_kjc.(RATA_trial_num)(2,:)';

%align signals
[IMU_thigh_align,mocap_thigh_align,t_sync_mocap_IMU1] = alignsignals(IMU_thigh_y_acc_kjc, mocap_thigh_y_acc_kjc);
sync_duration = -t_sync_mocap_IMU1+1: -t_sync_mocap_IMU1+length(mocap_thigh_y_acc_kjc);

t_series = 0:0.0025:((length(IMU_RATA.(trial_fieldname).RATA.(RATA_trial_num)(sync_duration))-1)*0.0025);

% figure
% plot(t_series, IMU_RATA.RATA.(RATA_trial_num)(sync_duration))
% hold on
% plot(t_series, MOCAP_RATA.(RATA_trial_num).RATA_mocap)
% 
% figure
% plot(mocap_thigh_align)
% hold on
% plot(IMU_thigh_align)

%% Correlation Coeff
RATA_IMU = IMU_RATA.(trial_fieldname).RATA.(RATA_trial_num);
mocap_data = MOCAP_RATA.(trial_fieldname);
% R_neg = corrcoef(RATA_IMU(sync_duration - 1), mocap_data.RATA_mocap); % shift right by 1 time step
% R_pos = corrcoef(RATA_IMU(sync_duration + 1), mocap_data.RATA_mocap);  % shift left by 1 time step
% R_i = corrcoef(RATA_IMU(sync_duration), mocap_data.RATA_mocap); % intitial correlation coeff.
R_neg = zeros(2,2);
R_pos = zeros(2,2);
R_i = zeros(2,2);

R_neg(1,2) = xcorr(RATA_IMU(sync_duration - 1), mocap_data.RATA_mocap,0,'coeff'); % shift right by 1 time step
R_pos(1,2) = xcorr(RATA_IMU(sync_duration + 1), mocap_data.RATA_mocap,0,'coeff');  % shift left by 1 time step
R_i(1,2) = xcorr(RATA_IMU(sync_duration), mocap_data.RATA_mocap,0,'coeff'); % intitial correlation coeff.

if R_neg(1,2) > R_pos(1,2) && R_i(1,2) < R_neg(1,2)  % if shifting right increases correlation coeff
    prev_R = R_i(1,2); % store R before shift
    shift = 2;
    R = R_neg(1,2);  % R_neg is used in while loop, keep shift in right until R decreases
    shift_sign = -1;
    while (1 - R) < 1-prev_R
        prev_R = R;
        R = corrcoef(RATA_IMU(sync_duration - shift), mocap_data.RATA_mocap);
        R = R(1,2);
        shift = shift + 1; %shift IMU RATA signal right 1 time step
    end
elseif R_neg(1,2) < R_pos(1,2) && R_i(1,2) < R_pos(1,2)  % if shifting left increases correlation coeff
    prev_R =  R_i(1,2);
    shift = 2;
    R = R_pos(1,2);
    shift_sign = 1;
    while (1 - R) < 1-prev_R % keep shifting left until R decreases.
        prev_R =  R;
        R = corrcoef(RATA_IMU(sync_duration + shift), mocap_data.RATA_mocap);
        R = R(1,2);
        shift = shift + 1;
    end
else
    shift = 2;
    R = R_i;
    shift_sign = 1;
    prev_R = R_i(1,2); % store R before shift
end
shift = shift -2;   % account for last while loop iteration

aligned_signals.(trial_fieldname) = struct('RATA_IMU',IMU_RATA.(trial_fieldname).RATA.(RATA_trial_num)(sync_duration+shift*shift_sign),'MOCAP_RATA', ...
    mocap_data.RATA_mocap, 'R',prev_R,'shift',shift*shift_sign);

pk_IMU_RATA.(trial_fieldname) = max(IMU_RATA.(trial_fieldname).RATA.(RATA_trial_num));
pk_MOCAP_RATA.(trial_fieldname) = max(MOCAP_RATA.(trial_fieldname).RATA_mocap);
R_struct.(trial_fieldname) = aligned_signals.(trial_fieldname).R; %reorganize strucure for correlation coeffs
R_i_struct.(trial_fieldname) = R_i(1,2); %initial corr coeff
shift_struct.(trial_fieldname) = aligned_signals.(trial_fieldname).shift;
sync_struct.(trial_fieldname) = sync_duration;
%Yu_cutoff_freqs.(trial_fieldname) = IMU_RATA.(trial_fieldname).cutoff_freqs.tr1;


%% shift amount to align IMU with mocap signals to determine net relative shift between tib and fem origin imu derived signals
[v,i2] = max(MOCAP_RATA.(trial_fieldname).acc_kjc_shank(:,2));
[v,i1] = max(IMU_RATA.(trial_fieldname).shank_acc_kjc.(RATA_trial_num)(2,sync_struct.(trial_fieldname)(1:i2+10)));


shift_shank = i2-i1;
[v,i3] = max(IMU_RATA.(trial_fieldname).thigh_acc_kjc.(RATA_trial_num)(2,sync_struct.(trial_fieldname)(1:i2+20)));
[v,i4] = max(MOCAP_RATA.(trial_fieldname).acc_kjc_thigh(1:i2+20,2));
shift_thigh = i4-i3;


% [v,i2] = max(IMU_RATA.(trial_fieldname).shank_acc_kjc.(RATA_trial_num)(2,sync_struct.(trial_fieldname)(1,:)));
% [v,i1] = max(MOCAP_RATA.(trial_fieldname).acc_kjc_shank(1:i2+10,2));
% 
% shift_shank = i1-i2;
% [v,i3] = max(IMU_RATA.(trial_fieldname).thigh_acc_kjc.(RATA_trial_num)(2,sync_struct.(trial_fieldname)(1:i2+20)));
% [v,i4] = max(MOCAP_RATA.(trial_fieldname).acc_kjc_thigh(1:i2+20,2));
% shift_thigh = i4-i3;

shift_amt.shank.(trial_fieldname) = shift_shank;
shift_amt.thigh.(trial_fieldname) = shift_thigh;

newRATA = IMU_RATA.(trial_fieldname).shank_acc_kjc.(RATA_trial_num)(1,sync_struct.(trial_fieldname) - 1*shift_amt.shank.(trial_fieldname)) - IMU_RATA.(trial_fieldname).thigh_acc_kjc.(RATA_trial_num)(1,sync_struct.(trial_fieldname) - 1*shift_amt.thigh.(trial_fieldname));
shiftRATA.(trial_fieldname) = newRATA;
R_shiftRATA.(trial_fieldname) = xcorr(shiftRATA.(trial_fieldname),MOCAP_RATA.(trial_fieldname).RATA_mocap,0,'coeff');
R_shiftRATA.(trial_fieldname) = aligned_signals.(trial_fieldname).R;
pk_shiftRATA.(trial_fieldname) = max(shiftRATA.(trial_fieldname));

%RMSE calc takes into account sync between MOCAP and RATA signals found
%earlier by cross-correlating the two signals to maximize
%cross-correlation. 
RMSE.(trial_fieldname) =sqrt(mean((MOCAP_RATA.(trial_fieldname).RATA_mocap - transpose(IMU_RATA.(trial_fieldname).RATA.tr1(1,sync_struct.(trial_fieldname) + aligned_signals.(trial_fieldname).shift))).^2));
RMSE_shift.(trial_fieldname) = sqrt(mean((MOCAP_RATA.(trial_fieldname).RATA_mocap - transpose(IMU_RATA.(trial_fieldname).RATA.tr1(1,sync_struct.(trial_fieldname) + aligned_signals.(trial_fieldname).shift))).^2));
end


%% Plot difference in curves
% figure
% hold on
% t_axis = -160:4:200;
for n = [1:length(imu_num)]
    trial_fieldname = ['tr', imu_num{n}];% trial fieldname ('tr22', etc.) of each trial analyzed.
%     [v, ind] = max(MOCAP_RATA.(trial_fieldname).RATA_mocap);
%     %101 data points, 50 before, 50 after peak IMU-derived RATA 
%     IMU_diff.(trial_fieldname) = (IMU_RATA.(trial_fieldname).RATA.tr1((IMU_RATA.(trial_fieldname).max_RATA_index.tr1) - 40:(IMU_RATA.(trial_fieldname).max_RATA_index.tr1 + 50)) - ...
%         transpose(MOCAP_RATA.(trial_fieldname).RATA_mocap(ind-40:ind+50))); %normalized by mocap derived peak RATA
%     RATA_diff_imu.(trial_fieldname) = IMU_RATA.(trial_fieldname).RATA.tr1((IMU_RATA.(trial_fieldname).max_RATA_index.tr1) - 40:(IMU_RATA.(trial_fieldname).max_RATA_index.tr1 + 50))/ v;
%     RATA_diff_mocap.(trial_fieldname) =  transpose(MOCAP_RATA.(trial_fieldname).RATA_mocap(ind-40:ind+50))/v;
%    
% 
%     
%     plot(t_axis,RATA_diff_imu.(trial_fieldname),'b')
%     plot(t_axis,RATA_diff_mocap.(trial_fieldname),'r')
% ylim([-1.5,1.5])


%%%PLOT anterior acceelration signals estimated origins (tibial/femoral),
%%%approx. as KJC, for mocap and IMU derived data.
%%%Also, shift acceleration signals of IMU so that max tibial anterior
%%%acceleration and minimum femoral anterior* acceleration sigals align in
%%%time with the mocap versions of these same signals (*shank anterior
%%%direction). Re-calculate RATA using these aligned signals.
RATA_trial_num = trial_fieldname;
% figure
%    subplot(2,1,1)
% plot(MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,1))
% hold on
% plot(IMU_RATA.(RATA_trial_num).shank_acc_kjc.tr1(1,sync_struct.(RATA_trial_num)))
% plot(MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,1))
% plot(IMU_RATA.(RATA_trial_num).thigh_acc_kjc.tr1(1,sync_struct.(RATA_trial_num)))
% legend('mocap shank','imu shank', 'mocap thigh','imu thigh')
%% Tib/Fem Origin Acceleration Signals IMU vs. Mocap - Shift vs. Unshift
figure
%     subplot(2,1,1)
%   
%     t_axis = 0:0.0025:(length(MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,1))-1)*0.0025;
%     
% plot(t_axis,MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,1),'r--','LineWidth',0.5)
%   title('Unshifted A.T. Acceleration Data')
% hold on
% plot(t_axis,IMU_RATA.(RATA_trial_num).shank_acc_kjc.tr1(1,sync_struct.(RATA_trial_num)),'r','LineWidth',0.5)
% plot(t_axis,MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,1),'b--','LineWidth',0.5)
% plot(t_axis,IMU_RATA.(RATA_trial_num).thigh_acc_kjc.tr1(1,sync_struct.(RATA_trial_num)),'b','LineWidth',0.5)
% legend('MOCAP tib','IMU tib', 'MOCAP fem','IMU fem')
% set(gca, 'XTickLabel', [])


% % % [x1,x2, shift_shank] = alignsignals(IMU_RATA.shank_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num)), MOCAP_RATA.(RATA_trial_num).acc_kjc_shank);
% % % [x3,x4, shift_thigh] = alignsignals(IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num)), MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh);
% % % [v,i2] = max(IMU_RATA.(trial_fieldname).shank_acc_kjc.tr1(1,sync_struct.(trial_fieldname)(1,:)));
% % % [v,i1] = max(MOCAP_RATA.(trial_fieldname).acc_kjc_shank(1:i2+10,1));
% % % 
% % % shift_shank = i1-i2;
% % % [v,i3] = min(IMU_RATA.(RATA_trial_num).thigh_acc_kjc.tr1(1,sync_struct.(RATA_trial_num)(1:i2+10)));
% % % [v,i4] = min(MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(1:i2+10,1));
% % % shift_thigh = i4-i3;
% % % 
% % % 
% % % shift_amt.shank.(RATA_trial_num) = shift_shank;
% % % shift_amt.thigh.(RATA_trial_num) = shift_thigh;


% subplot(2,1,2)
% 
% plot(t_axis,MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,1),'r--','LineWidth',0.5)
% title('Shifted A.T. Acceleration Data')
% hold on
% plot(t_axis, IMU_RATA.(RATA_trial_num).shank_acc_kjc.tr1(1,sync_struct.(RATA_trial_num) - 1*shift_amt.shank.(RATA_trial_num)),'r','LineWidth',0.5)
% plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,1),'b--','LineWidth',0.5)
% plot(t_axis, IMU_RATA.(RATA_trial_num).thigh_acc_kjc.tr1(1,sync_struct.(RATA_trial_num) - 1*shift_amt.thigh.(RATA_trial_num)),'b','LineWidth',0.5)
% ylabel('Acceleration, (m/s^2)')
% xlabel('Time (s)')
% set(gca, 'XTickLabel', [])
% subplot(2,1,2)
% plot(MOCAP_RATA.(RATA_trial_num).RATA_mocap)
% hold on
% plot(IMU_RATA.(RATA_trial_num).RATA.tr1(1,sync_struct.(RATA_trial_num)))
% newRATA = IMU_RATA.(RATA_trial_num).shank_acc_kjc.tr1(1,sync_struct.(RATA_trial_num) - shift_amt.shank.(RATA_trial_num)) - IMU_RATA.(RATA_trial_num).thigh_acc_kjc.tr1(1,sync_struct.(RATA_trial_num) - 1*shift_amt.thigh.(RATA_trial_num));
% plot(newRATA)
% legend('mocap','IMU','newRATA')    
% pk_newRATA.(RATA_trial_num) = max(newRATA);



%    figure
%     subplot(2,1,1)
%    
%     
% plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,2),'r--','LineWidth',0.5)
% title('Unshifted Longitudinal Tibial Acceleration Data')
% hold on
% plot(t_axis, IMU_RATA.(RATA_trial_num).shank_acc_kjc.tr1(2,sync_struct.(RATA_trial_num)),'r','LineWidth',0.5)
% plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,2),'b--','LineWidth',0.5)
% plot(t_axis, IMU_RATA.(RATA_trial_num).thigh_acc_kjc.tr1(2,sync_struct.(RATA_trial_num)),'b','LineWidth',0.5)
% legend('MOCAP tib','IMU tib', 'MOCAP fem','IMU fem')
% set(gca, 'XTickLabel', [])
%     
% subplot(2,1,2)
% 
% plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,2),'r--','LineWidth',0.5)
% title('Shifted Longitudinal Tibial Acceleration Data')
% hold on
% plot(t_axis, IMU_RATA.(RATA_trial_num).shank_acc_kjc.tr1(2,sync_struct.(RATA_trial_num) - 1*shift_amt.shank.(RATA_trial_num)),'r','LineWidth',0.5)
% plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,2),'b--','LineWidth',0.5)
% plot(t_axis, IMU_RATA.(RATA_trial_num).thigh_acc_kjc.tr1(2,sync_struct.(RATA_trial_num) - 1*shift_amt.thigh.(RATA_trial_num)),'b','LineWidth',0.5)
% ylabel('Acceleration, (m/s^2)')
% xlabel('Time (s)')
% set(gca, 'XTickLabel', [])
%% Contribution, how much does the ang. acc. portion of RATA contribute to RATA
contribution.(RATA_trial_num) = transpose(IMU_RATA.(RATA_trial_num).RATA.tr1(sync_struct.(RATA_trial_num)));
%% MOCAP FFT
shx_fft.(RATA_trial_num) = fft(MOCAP_RATA.(RATA_trial_num).shank_raw(:,1));
shy_fft.(RATA_trial_num) = fft(MOCAP_RATA.(RATA_trial_num).shank_raw(:,2));

thx_fft.(RATA_trial_num) = fft(MOCAP_RATA.(RATA_trial_num).thigh_raw(:,1));
thy_fft.(RATA_trial_num) = fft(MOCAP_RATA.(RATA_trial_num).thigh_raw(:,2));

nsam.(RATA_trial_num) = length(MOCAP_RATA.(RATA_trial_num).shank_raw(:,1));

power_shx.(RATA_trial_num) = abs(shx_fft.(RATA_trial_num)).^2/nsam.(RATA_trial_num);
power_shy.(RATA_trial_num) = abs(shy_fft.(RATA_trial_num)).^2/nsam.(RATA_trial_num);
power_thx.(RATA_trial_num) = abs(thx_fft.(RATA_trial_num)).^2/nsam.(RATA_trial_num);
power_thy.(RATA_trial_num) = abs(thy_fft.(RATA_trial_num)).^2/nsam.(RATA_trial_num);

f_fft.(RATA_trial_num) = (0:nsam.(RATA_trial_num)-1)*(400/nsam.(RATA_trial_num));
end
%% 
s(:,1) = transpose(struct2mat(shift_amt.shank)); %soft-tissue shift
s(:,2) = transpose(struct2mat(shift_amt.thigh)); %soft-tissue shift
a(:,1) = transpose(struct2mat(pk_IMU_RATA)); %peak RATA (IMU-derived)
a(:,2) = transpose(struct2mat(pk_MOCAP_RATA));%peak RATA (mocap-derived)
a(:,3) = transpose(struct2mat(pk_shiftRATA)); %peak RATA(IMU-derived, with shift applied, (soft-tissue only))
a(:,4) = a(:,1)-a(:,2); %pk IMU RATA - pk MOCAP RATA
a(:,5) = a(:,3)-a(:,2); %pk SHIFT IMU RATA (thigh/shank acc signal relative shift applied) - pk MOCAP RATA
a(:,6) = transpose(struct2mat(RMSE)); %RMSE between RATA signals from two measurement methods
a(:,7) = transpose(struct2mat(RMSE_shift)); %RMSE calculated with relative shift applied, (soft-tissue only)
a(:,8) = transpose(struct2mat(R_struct)); %xcorr coeff
a(:,9) = transpose(struct2mat(R_shiftRATA)); %xcorr coeff with shift appplied (soft-tissue only)

   %% Mean and standard deviation RATA plots (NORMALIZED)
  
% RATA_imu_stdev(1,:) = std(struct2mat(RATA_diff_imu),0,2);
% RATA_imu_stdev(2,:) = std(struct2mat(RATA_diff_imu),0,2);
% 
% RATA_mocap_stdev(1,:) = std(struct2mat(RATA_diff_mocap),0,2);
% RATA_mocap_stdev(2,:) = std(struct2mat(RATA_diff_mocap),0,2);
% 
% RATA_imu_mean = mean(struct2mat(RATA_diff_imu),2);
% RATA_mocap_mean = mean(struct2mat(RATA_diff_mocap),2);
% 
% shadedErrorBar(t_axis,RATA_imu_mean,RATA_imu_stdev,'lineProps','-b');
% hold on
% shadedErrorBar(t_axis,RATA_mocap_mean,RATA_mocap_stdev,'lineProps','-r');
% ylim([-1.6,1.6])
% xlim([-160,200])



%% Save Tables to Excel
peak_IMU_RATA_T = struct2table(pk_IMU_RATA);
peak_MOCAP_RATA_T = struct2table(pk_MOCAP_RATA);
optimal_freq_T = struct2table(peaks.optimal_freq); %create table for optimal filtering frequencies (1xnum_RATA_trials)
%optimal_freq_T.Properties.VariableNames = peak_IMU_RATA_T.Properties.VariableNames; %re-name columns
peak_thigh_acc_T = struct2table(peaks.thigh); %create table for peak thigh longitudinal acceleration when filtered using optimal freq.
peak_shank_acc_T = struct2table(peaks.shank); %create table for peak shank longitudinal acceleratio nwhen filtered using optimal freq.
R_T = struct2table(R_struct);
shift_T = struct2table(shift_struct);

%cut_f_T = array2table(cut_f .* ones(1,num_RATA_trials));
%cut_f_T.Properties.VariableNames = peak_IMU_RATA_T.Properties.VariableNames;
variablenames = cell2table({'pk_IMU_RATA';'pk_MOCAP_RATA'; 'optimal_freq';'peak_thigh_accY'; 'peak_shank_accY';'CorrCoef'; 'shift'});

compile_table = vertcat(peak_IMU_RATA_T, peak_MOCAP_RATA_T, optimal_freq_T, peak_thigh_acc_T, peak_shank_acc_T, R_T, shift_T);
compile_table = horzcat(variablenames,compile_table);

% %%
% trial_num_i = ['tr',imu_num{1}];
% Yu_cutoff_freq_T = struct2table(Yu_cutoff_freqs.(trial_num_i));
% for i = 2:num_RATA_trials
%      trial_fieldname = ['tr', imu_num{i}]; % trial fieldname ('tr22', etc.) of each trial analyzed.
%      Yu_cutoff_freq_T = vertcat(Yu_cutoff_freq_T, struct2table(Yu_cutoff_freqs.(trial_fieldname)));
% end
% %%
% cut_f = 0;
% save_file_path = ['C:\Users\scott\Documents\HPL\IMU\RATA_RESULTS\Mechanical_Test_Rig_Yu_Filtering\'];
% mkdir([save_file_path, folder_IMU1(end-3:end-2), folder_IMU2(end-1:end)])
% save_file_path_newfolder = [save_file_path, folder_IMU1(end-3:end-2),folder_IMU2(end-1:end),'\'];
% excelname = ['session',folder_IMU1(end-3:end-2),folder_IMU2(end-1:end),'_',num2str(cut_f),'Yu_filtering','.xls'];
% writetable(compile_table, [save_file_path_newfolder,excelname])
% %%
% excelname = [folder_IMU1(end-3:end-2),folder_IMU2(end-1:end),'Yu_filtering_cutoffs','.xls'];
% writetable(Yu_cutoff_freq_T,[save_file_path_newfolder, excelname]);



    
    
%Script for analyzing RATA data for soft-tissue test experiments

clear all
close all

%% Data identifying information (USER INPUT, data directory and foldernames)
data_directory = 'C:\Users\scott\Documents\HPL\IMU\RATA_IMU_DATA\'; %location of IMU data
folder_IMU1 = 'IMU1_MECH_79'; %foldername for IMU #1 data
folder_IMU2 = 'IMU2_MECH_79'; % foldername for IMU #2 data

%% IMPORT DATA
data = importData_fxn(data_directory,folder_IMU1,folder_IMU2);
%% DCMs (USER INPUT, button presses  if more than 1 trial collected (optional) )
dcm_button_presses = [3:6]; %% user input here!!%%
DCMs = getDCM_fxn(data,dcm_button_presses);

%% PosVecs (USER INPUT, button presses  if more than 1 trial collected  (optional) )
posvec_button_presses = [7:10]; %%user input here!!%%%
posvecs = getposvecs_fxn2(data,DCMs,posvec_button_presses);

%% USER INPUT

rata_button_presses = [11:48, 51:52]; %% USER INPUT HERE!! %%%
num_RATA_trials = length(rata_button_presses)/2;

%% MOCAP RATA 
for n = 1:num_RATA_trials %= number of RATA trials collected
    trial_name = ['tr',num2str(n)];
MOCAP_RATA.(trial_name) = mocap_processing_fxn(['MOCAP_RATA_',folder_IMU1(end-1:end),'_',num2str(n),'_cam1.mqa'], 3); %mocap_procssing_fxn(ascii filename, marker config)
end

%% IMU RATA  ( FINAL USER INPUT, button presses if more than 1 trial collected (optional) )


%% ADD OPTIMAL FREQUENCY CODE HERE

freqs = [20];  %%USER INPUT, range of possible optimal freqeuncies to search within.
trial_ind = 1;

for trial = 1:num_RATA_trials
    
    f_ind = 1;
    for f = freqs  % cutoff frequency possibilties
     
        trial_fieldname = ['tr', num2str(trial)];
        f_fieldname = ['f',num2str(f)];
        
        IMU_RATA_f = getRATA_fxn(data,f,DCMs,posvecs,rata_button_presses);   % calculate RATA values

        RATA_pk_IMU(f_ind,1) = max(IMU_RATA_f.RATA.(trial_fieldname)); %access pk IMU RATA
        RATA_pk_mocap(f_ind,1) = max(MOCAP_RATA.(trial_fieldname).RATA_mocap); %access pk mocap RATA
        thigh_pk(f_ind,1) = max(IMU_RATA_f.thigh_acc_kjc_tframe.(trial_fieldname)(2,:)); %max longitudinal thigh acc.
        shank_pk(f_ind,1) = max(IMU_RATA_f.shank_acc_kjc.(trial_fieldname)(2,:)); %max longitudinal shank acc.
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
end


%%
cut_f = 20;
IMU_RATA = getRATA_fxn(data,cut_f,DCMs,posvecs,rata_button_presses);  %getRATA_fxn(sessionData,cutoff filter freq, DCMs, posvecs, button presses in session for RATA trials);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Align RATA signals, output peak values for IMU and MOCAP derived RATA and corrcoef(R) and shift amount

%% TIME SYNC MOCAP TO IMU1

for n = 1:num_RATA_trials % = number of RATA trials collected
    %%
    RATA_trial_num = ['tr',num2str(n)];
    
mocap_shank_y_acc_kjc = MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,2);

IMU_shank_y_acc_kjc = IMU_RATA.shank_acc_kjc.(RATA_trial_num)(2,:)';

%align signals
[IMU_thigh_align,mocap_thigh_align,t_sync_mocap_IMU1] = alignsignals(IMU_shank_y_acc_kjc, mocap_shank_y_acc_kjc);
sync_duration = -t_sync_mocap_IMU1+1: -t_sync_mocap_IMU1+length(mocap_shank_y_acc_kjc);

t_series = 0:0.0025:((length(IMU_RATA.RATA.(RATA_trial_num)(sync_duration))-1)*0.0025);

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
RATA_IMU = IMU_RATA.RATA.(RATA_trial_num);
mocap_data = MOCAP_RATA.(RATA_trial_num);
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
    shift_sign  = 1;
    prev_R = R_i(1,2);
end
shift = shift -2;   % account for last while loop iteration

aligned_signals.(RATA_trial_num) = struct('RATA_IMU',IMU_RATA.RATA.(RATA_trial_num)(sync_duration+shift*shift_sign),'MOCAP_RATA', ...
    mocap_data.RATA_mocap, 'R',prev_R,'shift',shift*shift_sign);

pk_IMU_RATA.(RATA_trial_num) = max(IMU_RATA.RATA.(RATA_trial_num));
pk_MOCAP_RATA.(RATA_trial_num) = max(MOCAP_RATA.(RATA_trial_num).RATA_mocap);
R_struct.(RATA_trial_num) = aligned_signals.(RATA_trial_num).R; %reorganize strucure for correlation coeffs
R_i_struct.(RATA_trial_num) = R_i(1,2);

shift_struct.(RATA_trial_num) = aligned_signals.(RATA_trial_num).shift;
sync_struct.(RATA_trial_num) = sync_duration;
%Yu_cutoff_freqs.(RATA_trial_num) = IMU_RATA.cutoff_freqs.(RATA_trial_num); 

%% Tib/Fem Origin Acceleration Signals IMU vs. Mocap - Shift vs. Unshift
figure
    subplot(2,1,1)

     t_axis = 0:0.0025:(length(MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,1))-1)*0.0025;
plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,1),'r--','LineWidth',0.5)
title('Unshifted A.T. Acceleration Data')
hold on
plot(t_axis, IMU_RATA.shank_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num)),'r','LineWidth',0.5)
plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,1),'b--','LineWidth',0.5)
plot(t_axis, IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num)),'b','LineWidth',0.5)
legend('MOCAP tib','IMU tib', 'MOCAP fem','IMU fem')
set(gca, 'XTickLabel', [])
%xlim([120,220])

% [x1,x2, shift_shank] = alignsignals(IMU_RATA.shank_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num)), MOCAP_RATA.(RATA_trial_num).acc_kjc_shank);
% [x3,x4, shift_thigh] = alignsignals(IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num)), MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh);
%% shift amount to align IMU with mocap signals to determine net relative shift between tib and fem origin imu derived signlas
%% identify lag between tib and fem origin signals caused by soft tissue
[v,i2] = max(MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,2));
[v,i1] = max(IMU_RATA.shank_acc_kjc.(RATA_trial_num)(2,sync_struct.(RATA_trial_num)(1:i2+5)));

shift_shank = i2-i1;
[v,i3] = max(IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(2,sync_struct.(RATA_trial_num)(1:i2+5)));
[v,i4] = max(MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(1:i2,2));
shift_thigh = i4-i3;


shift_amt.shank.(RATA_trial_num) = shift_shank;
shift_amt.thigh.(RATA_trial_num) = shift_thigh;

subplot(2,1,2)

plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,1),'r--','LineWidth',0.5)
title('Shifted A.T. Acceleration Data')
hold on
plot(t_axis, IMU_RATA.shank_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num) -  1*shift_amt.shank.(RATA_trial_num)),'r','LineWidth',0.5)
plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,1),'b--','LineWidth',0.5)
plot(t_axis, IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num) - 1*shift_amt.thigh.(RATA_trial_num)),'b','LineWidth',0.5)
ylabel('Acceleration, (m/s^2)')
xlabel('Time (s)')
set(gca, 'XTickLabel', [])
%xlim([120,220])
% subplot(2,1,2)
% plot(MOCAP_RATA.(RATA_trial_num).RATA_mocap)
% hold on
% plot(IMU_RATA.RATA.(RATA_trial_num)(sync_struct.(RATA_trial_num)))
newRATA = IMU_RATA.shank_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num) - 1*shift_amt.shank.(RATA_trial_num) ) - IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(1,sync_struct.(RATA_trial_num) - 1*shift_amt.thigh.(RATA_trial_num) );
%plot(newRATA)
%legend('mocap','IMU','newRATA')    
pk_newRATA.(RATA_trial_num) = max(newRATA);
shiftRATA.(RATA_trial_num) = newRATA;

%% Code to sync MOCAP and IMU based RATA signals. + RMSE calcs of unshifted and shifted data (shift = shank IMU data relative to thigh IMU data)
for shift2 = -3:1:-1
    R_s(shift2+4) = xcorr(shiftRATA.(RATA_trial_num)(1-shift2:end), MOCAP_RATA.(RATA_trial_num).RATA_mocap(1:end+shift2), 0 , 'coeff');
end
for shift1 = 0:3
    R_s(shift1+4) = xcorr(shiftRATA.(RATA_trial_num)(1:end-shift1), MOCAP_RATA.(RATA_trial_num).RATA_mocap(shift1+1:end),0,'coeff');
end
[v, index ] = max(R_s);   
%R_shiftRATA.(RATA_trial_num) = max(R_s);
 R_shiftRATA.(RATA_trial_num) = xcorr(shiftRATA.(RATA_trial_num),MOCAP_RATA.(RATA_trial_num).RATA_mocap,0,'coeff');
    vec = -3:1:3;
RATA_sync_shift.(RATA_trial_num) = vec(index);
RMSE_noshift.(RATA_trial_num) = sqrt(mean((MOCAP_RATA.(RATA_trial_num).RATA_mocap - transpose(IMU_RATA.RATA.(RATA_trial_num)(1,sync_struct.(RATA_trial_num) + 0*aligned_signals.(RATA_trial_num).shift))).^2));
RMSE_shift.(RATA_trial_num) =  sqrt(mean((MOCAP_RATA.(RATA_trial_num).RATA_mocap - transpose(shiftRATA.(RATA_trial_num))).^2));
% if index >=4
% RMSE_shift.(RATA_trial_num) = sqrt(mean((MOCAP_RATA.(RATA_trial_num).RATA_mocap(index -3:end) - transpose(shiftRATA.(RATA_trial_num)(1:end-(index - 4)))) .^2));
% else
%     RMSE_shift.(RATA_trial_num) = sqrt(mean((MOCAP_RATA.(RATA_trial_num).RATA_mocap(1:end-(4-index)) - transpose(shiftRATA.(RATA_trial_num)(5-index :end))) .^2));
% end
%% Long. Acceleration Unshift/Shift

   figure
    subplot(2,1,1)
   
    
plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,2),'r--','LineWidth',0.5)
title('Unshifted Longitudinal Tibial Acceleration Data')
hold on
plot(t_axis, IMU_RATA.shank_acc_kjc.(RATA_trial_num)(2,sync_struct.(RATA_trial_num)),'r','LineWidth',0.5)
plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,2),'b--','LineWidth',0.5)
plot(t_axis, IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(2,sync_struct.(RATA_trial_num)),'b','LineWidth',0.5)
legend('MOCAP tib','IMU tib', 'MOCAP fem','IMU fem')
set(gca, 'XTickLabel', [])
    
subplot(2,1,2)

plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_shank(:,2),'r--','LineWidth',0.5)
title('Shifted Longitudinal Tibial Acceleration Data')
hold on
plot(t_axis, IMU_RATA.shank_acc_kjc.(RATA_trial_num)(2,sync_struct.(RATA_trial_num) - shift_amt.shank.(RATA_trial_num)),'r','LineWidth',0.5)
plot(t_axis, MOCAP_RATA.(RATA_trial_num).acc_kjc_thigh(:,2),'b--','LineWidth',0.5)
plot(t_axis, IMU_RATA.thigh_acc_kjc.(RATA_trial_num)(2,sync_struct.(RATA_trial_num) - 1*shift_amt.thigh.(RATA_trial_num)),'b','LineWidth',0.5)
ylabel('Acceleration, (m/s^2)')
xlabel('Time (s)')
set(gca, 'XTickLabel', [])

end
s(:,1) = transpose(struct2mat(shift_amt.shank));
s(:,2) = transpose(struct2mat(shift_amt.thigh));
a(:,1) = transpose(struct2mat(pk_IMU_RATA));
a(:,2) = transpose(struct2mat(pk_MOCAP_RATA));
a(:,3) = transpose(struct2mat(pk_newRATA));
a(:,4) = a(:,1)-a(:,2); %pk IMU RATA - pk MOCAP RATA
a(:,5) = a(:,3)-a(:,2); %pk SHIFT IMU RATA (thigh/shank acc signal relative shift applied) - pk MOCAP RATA
a(:,6) = transpose(struct2mat(RMSE_noshift));
a(:,7) = transpose(struct2mat(RMSE_shift));
a(:,8) = transpose(struct2mat(R_i_struct));
a(:,9) = transpose(struct2mat(R_shiftRATA));
%% Save Tables to Excel
peak_IMU_RATA_T = struct2table(pk_IMU_RATA);
peak_MOCAP_RATA_T = struct2table(pk_MOCAP_RATA);
optimal_freq_T = array2table(optimal_freq'); %create table for optimal filtering frequencies (1xnum_RATA_trials)
optimal_freq_T.Properties.VariableNames = peak_IMU_RATA_T.Properties.VariableNames; %re-name columns
peak_thigh_acc_T = struct2table(peaks.thigh); %create table for peak thigh longitudinal acceleration when filtered using optimal freq.
peak_shank_acc_T = struct2table(peaks.shank); %create table for peak shank longitudinal acceleratio nwhen filtered using optimal freq.
R_T = struct2table(R_struct);
shift_T = struct2table(shift_struct);
cut_f_T = array2table(cut_f .* ones(1,num_RATA_trials));
cut_f_T.Properties.VariableNames = peak_IMU_RATA_T.Properties.VariableNames;
variablenames = cell2table({'pk_IMU_RATA';'pk_MOCAP_RATA'; 'optimal_freq';'peak_thigh_accY'; 'peak_shank_accY';'CorrCoef'; 'shift';'filter_freq'});

compile_table = vertcat(peak_IMU_RATA_T, peak_MOCAP_RATA_T, optimal_freq_T, peak_thigh_acc_T, peak_shank_acc_T, R_T, shift_T, cut_f_T);
compile_table = horzcat(variablenames,compile_table);

%%
trial_num_i = ['tr1'];
Yu_cutoff_freq_T = struct2table(Yu_cutoff_freqs.(trial_num_i));
for i = 2:num_RATA_trials
     trial_fieldname = ['tr', num2str(i)]; % trial fieldname ('tr22', etc.) of each trial analyzed.
     Yu_cutoff_freq_T = vertcat(Yu_cutoff_freq_T, struct2table(Yu_cutoff_freqs.(trial_fieldname)));
end


save_file_path = ['C:\Users\scott\Documents\HPL\IMU\RATA_RESULTS\Mechanical_Test_Rig_Soft_Tissue_YuFilter\'];
mkdir([save_file_path, folder_IMU1(end-1:end)])
save_file_path_newfolder = [save_file_path, folder_IMU1(end-1:end),'\'];
excelname = ['session',folder_IMU1(end-1:end),'_',num2str(cut_f),'Hz','.xls'];
writetable(compile_table, [save_file_path_newfolder,excelname])

excelname = [folder_IMU1(end-1:end),'_Yu_filtering_cutoffs','.xls'];
writetable(Yu_cutoff_freq_T,[save_file_path_newfolder, excelname]);

%%
[v,ind1_a] = max(aligned_signals.tr1.RATA_IMU);
[v,ind1_b] = max(aligned_signals.tr1.MOCAP_RATA);

[v,ind2_a] = max(aligned_signals.tr3.RATA_IMU);
[v,ind2_b] = max(aligned_signals.tr3.MOCAP_RATA);


[v,ind3_a] = max(aligned_signals.tr10.RATA_IMU);
[v,ind3_b] = max(aligned_signals.tr10.MOCAP_RATA);
figure
subplot(3,1,1)
x = -4*30:4:4*99;
plot(x,aligned_signals.tr1.RATA_IMU(ind1_a-30:ind1_a+99));
hold on
plot(x,aligned_signals.tr1.MOCAP_RATA(ind1_b-30:ind1_b+99));
xlim([-120,400])
ylim([-30,40]);

subplot(3,1,2)
plot(x,aligned_signals.tr3.RATA_IMU(ind2_a-30:ind2_a+99));
hold on
plot(x,aligned_signals.tr3.MOCAP_RATA(ind2_b-30:ind2_b+99));
xlim([-120,400])
ylim([-30,40]);
subplot(3,1,2)

subplot(3,1,3)
plot(x,aligned_signals.tr10.RATA_IMU(ind3_a-30:ind3_a+99));
hold on
plot(x,aligned_signals.tr10.MOCAP_RATA(ind3_b-30:ind3_b+99));
xlim([-120,400])
ylim([-30,40]);

%%
% sync = sync_struct.tr14;
% signal = data.sensors1.accelerometerX(data.button1(37)+sync:data.button1(37)+sync+500);
% t_domain =  signal;
% t_domain = t_domain;
% % Sample frequency 
%  Fs = 400;
% % Fast fourier transform (MATLAB actually does a discrete fourier transform)  
% % Taking Fs+1 "pads" the signal to Fs+1 data points
% % For me, at 1000 Hz, this means that my output will be 0 to 500 Hz and then 499 to 1 Hz in 1 Hz bins. I like doing it this way because I like the round bin size.
% % Another common practice is to zero pad to 2048 or another 2^x to ensure periodicity which makes the function faster. I never find speed to be a problem though. 
% f_domain = fft(t_domain, Fs+1, 1);
% % View your results
% % figure
% % stem(abs(f_domain));
% % Mean power in the time domain  
% P_t = mean(t_domain.^2);
% % Sum of powers in the frequency domain
% % Whenever working with f_domain need to remember it is complex x + i
% % Therefore take absolute value to just get real part of it
% P_f = sum(abs(f_domain(1:Fs/2+1)),1);
% % To express the f_domain in meaningful units normalize the time and frequency powers
% % Your units should now be (time domain units)^2/Hz
% f_norm = abs(f_domain(1:Fs/2+1)).*(P_t./P_f);
% % View your results
% figure
% stem(f_norm);
% title(signal_name)
%
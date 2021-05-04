clear all
close all

%% Data identifying information (USER INPUT, data directory and foldernames)
data_directory = 'C:\Users\scott\Documents\HPL\IMU\RATA_IMU_DATA\'; %location of IMU data
folder_IMU1 = 'IMU1_HUMN_11'; %foldername for IMU #1 data
folder_IMU2 = 'IMU2_HUMN_11'; % foldername for IMU #2 data

%% IMPORT DATA
data = importData_fxn(data_directory,folder_IMU1,folder_IMU2);
%% DCMs (USER INPUT, button presses  if more than 1 trial collected (optional) )
dcm_button_presses = [3:6]; %% user input here!!%%
DCMs = getDCM_fxn(data,dcm_button_presses);
num_trials = length(dcm_button_presses)/4;
for i = 1:num_trials
    trial_name = ['tr',num2str(i)];
    DCM1T_mat(:,:,i) = DCMs.DCM1T.(trial_name); % for element-wise accuracy/precision
    DCM2S_mat(:,:,i) = DCMs.DCM2S.(trial_name);
    
    %Metrics for quantifying DCM estimation accuracy (Nazarahari 2019,
    %Mecheri 2016)
    theta_1T(i) = acosd((trace(DCMs.DCM1T.(trial_name)) -1)/2); % axis-angle theta
    theta_2S(i) = acosd((trace(DCMs.DCM2S.(trial_name)) -1)/2); % axis-angle theta
    
    quat_1T(i) = acosd(abs(dot(dcm2quat(eye(3)), dcm2quat(DCMs.DCM1T.(trial_name)))));
    quat_2S(i) = acosd(abs(dot(dcm2quat(eye(3)), dcm2quat(DCMs.DCM2S.(trial_name)))));
end
theta_1T_stdev = std(theta_1T);
theta_2S_stdev = std(theta_2S);
quat_1T_stdev = std(quat_1T);
quat_2S_stdev = std(quat_2S);

DCM1T_stdev = std(DCM1T_mat,0,3);
DCM2S_stdev = std(DCM2S_mat,0,3);
%% PosVecs (USER INPUT, button presses  if more than 1 trial collected  (optional) )
posvec_button_presses = [7:10]; %%user input here!!%%%
posvecsA = getposvecs_fxn2(data,DCMs, posvec_button_presses);

posvec1 = transpose(struct2mat(posvecsA.posvec1));
posvec2 = transpose(struct2mat(posvecsA.posvec2));
%% RATA
close all


rata_button_presses = [25:26];
cut_f = 0;
IMU_RATA = getRATA_fxn(data,cut_f,DCMs,posvecsA,rata_button_presses);


%%%% PLOT Characteristic Curve for Non-Walking/Jogging Trials
%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %Peak Shank Y acc location
% for n = 1:length(rata_button_presses)/2
%     trial_num = ['tr',num2str(n)];
% [v_asy, ind_pk_asy] = max(IMU_RATA.shank_acc_kjc.(trial_num)(2,:));
% [v_aty, ind_pk_aty] = max(IMU_RATA.thigh_acc_kjc_tframe.(trial_num)(2,:));
% [v_asx_min, ind_pk_asx_min] = min(IMU_RATA.shank_acc_kjc.(trial_num)(1,:));
% [v_atx_min, ind_pk_atx_min] = min(IMU_RATA.thigh_acc_kjc_tframe.(trial_num)(1,:));
% [v_asx_max, ind_pk_asx_max] = min(IMU_RATA.shank_acc_kjc.(trial_num)(1,:));
% [v_atx_max, ind_pk_atx_max] = min(IMU_RATA.thigh_acc_kjc_tframe.(trial_num)(1,:));
% [v_rata,ind_pk_rata] = max(IMU_RATA.RATA.(trial_num)(ind_pk_asy-200:ind_pk_asy+200));
% 
% %save indices
% pk_ind.sy.(trial_num) = ind_pk_asy; %peak vertical acc, shank
% pk_ind.ty.(trial_num) = ind_pk_aty; % peak vertical acc, thigh
% pk_ind.sx.(trial_num) = ind_pk_asx_min; %minimum anterior acc (max posterior acc), near impact, shank
% pk_ind.tx.(trial_num) = ind_pk_atx_min;  % minimum anterior acc(max posterior acc, near impact, thigh
% pk_ind.rata.(trial_num) = ind_pk_rata; %peak rata near impact
%   
% 
% %Use peak RATA near impact (defined by peak shank y-acc +/- 0.5seconds) to align curves
% IMU_RATA_impact.(trial_num) = IMU_RATA.RATA.(trial_num)(ind_pk_asy - 200 + pk_ind.rata.(trial_num)-100:ind_pk_asy - 200 + pk_ind.rata.(trial_num)+100);
% 
% %use peak thigh acc to align curves
% %IMU_RATA_impact.(trial_num) = IMU_RATA.RATA.(trial_num)(pk_ind.ty.(trial_num)-100: pk_ind.ty.(trial_num)+100);
% 
% %save values of peak accelerations which occur near initial ground contact
% pk_acc.asy.(trial_num) = v_asy;
% pk_acc.aty.(trial_num) = v_aty;
% pk_acc.asx_min.(trial_num) = v_asx_min;
% pk_acc.atx_min.(trial_num) = v_atx_min;
% pk_acc.asx_max.(trial_num) = v_asx_max;
% pk_acc.atx_max.(trial_num) = v_atx_max;
% pk_rata.(trial_num) = v_rata;
% end
% 
% %create matrices for key values to copy/paste to excel file
% pk_accy_mat = transpose(struct2mat(pk_acc.asy)); % peak Y acc of shank, in shank frame near ground impact
% pk_accx_max_mat = transpose(struct2mat(pk_acc.asx_max)); % peak X acc of shank, in shank frame near ground impact
% pk_accx_min_mat = transpose(struct2mat(pk_acc.asx_min)); % min X acc of shank in shank frame near ground impact
% pk_rata_mat = transpose(struct2mat(pk_rata)); % peak RATA
% 
% 
% RATA_imu_stdev(1,:) = std(struct2mat(IMU_RATA_impact),0,2);
% RATA_imu_stdev(2,:) = RATA_imu_stdev(1,:);
% RATA_imu_mean = mean(struct2mat(IMU_RATA_impact),2);
% t_axis = -0.25:0.0025:0.25;
% xlim([-0.25,0.25])
% xticks([-0.25, -0.125, 0, 0.125,0.25]);
% xticklabels({-0.25,-0.125, 0 ,0.125, 0.25});
% yticks([-100,-50, 0, 50, 100])
% yticklabels({-100,-50, 0, 50, 100});
% 
% shaded_fig = shadedErrorBar(t_axis, RATA_imu_mean,RATA_imu_stdev,'lineProps','-b');
% fig_title = title('Sprint to Backpedal RATA Characteristic Curve');
% title_pos =  get(fig_title, 'position');
% title_pos(2) = 105.5;
% set(fig_title, 'position', title_pos);
% 
% out_pos = get(gca, 'OuterPosition');
% out_pos = [0,0,1.05,0.95];
% set(gca,'OuterPosition', out_pos);
% 
% xlabel('Time(seconds)')
% ylabel('RATA (m/s^2)')
% ylim([-100,100])
% set(gca,'FontSize',12)
% 
% 
% %%% PLOT Full Trial for Walking/Jogging Trials
% figure
% 
% subplot(3,1,1)
% plot(IMU_RATA.shank_acc_kjc.(trial_num)(1,1:end)) %shank anterior acc shifted
% hold on
% plot(IMU_RATA.thigh_acc_kjc.(trial_num)(1,1:end)) %thigh anterior acc
% hline(0);
% title('Walking - Acceleration in Shank Anterior Direction')
% legend('Point on Shank', 'Point on Thigh')
% 
% subplot(3,1,2)
% plot(IMU_RATA.shank_acc_kjc.(trial_num)(1,1:end)-IMU_RATA.thigh_acc_kjc.(trial_num)(1,1:end))
% hold on
% 
% yyaxis right
%  plot(IMU_RATA.shank_acc_kjc.(trial_num)(2,1:end)) % shank y acc shifted
%  plot(IMU_RATA.thigh_acc_kjc_tframe.(trial_num)(2,1:end),'--k') % thigh y acc
%  hline(0);
%  title('RATA and vertical acceleration of Shank & Thigh')
% legend('RATA','Shank Y acc', 'Thigh Y acc')
% 
% subplot(3,1,3)
% 
% plot(IMU_RATA.shank_gyr.(trial_num)(3,:))
% title('Shank Z Gyr')

%%% PLOT Characteristic Curve for Walking Trials %%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Get data for characteristic curve

for m = 1:length(rata_button_presses)/2
    trial_num = ['tr',num2str(m)];
    % create figure to select step indices
    figure
    %plot shank X, Thigh X
    subplot(3,1,1)
    plot(IMU_RATA.shank_acc_kjc.(trial_num)(1,1:end)) %shank anterior acc shifted
    hold on
    plot(IMU_RATA.thigh_acc_kjc.(trial_num)(1,1:end)) %thigh anterior acc
    hline(0);
    title('Acceleration in Shank Anterior Direction')
    legend('Point on Shank', 'Point on Thigh')
    
    %plot shank Y, Thigh Y, RATA
    subplot(3,1,2)
    %plot(IMU_RATA.shank_acc_kjc.(trial_num)(1,1:end)-IMU_RATA.thigh_acc_kjc.(trial_num)(1,1:end))
    hold on
    

    plot(IMU_RATA.shank_acc_kjc.(trial_num)(2,1:end)) % shank y acc shifted
    plot(IMU_RATA.thigh_acc_kjc_tframe.(trial_num)(2,1:end)) % thigh y acc
    hline(0);
    title('Y acceleration of Shank & Thigh')
    legend('Shank Y acc', 'Thigh Y acc')
    
    %plot shank gyro
    subplot(3,1,3)
    plot(IMU_RATA.shank_gyr.(trial_num)(3,:))
    hold on
    plot(IMU_RATA.thigh_gyr.(trial_num)(3,:))
      hline(0)
    legend('shank','thigh')
    title('Shank/Thigh Z Gyr')
    
    clear walk_ind
    %%% select step indices
    [walk_ind, y] = ginput(8);
    walk_ind_a = walk_ind(1:1:end); % user select snippets of data correpsonding to steps.
    walk_ind_b = walk_ind(1:1:end)+300;
    
    % use selected step indices to identify peak RATA and acceleration peak
    % values using the peak shank y acc for alignment
    for n = 1:length(walk_ind)
        step = ['step',num2str(m),'_',num2str(n)]; % field name for each step analyzed
        
        acc_shank_impact.(step) = IMU_RATA.shank_acc_kjc.(trial_num)(:,walk_ind_a(n):walk_ind_b(n)); %cut data down to selected section near impact
        acc_thigh_impact.(step) = IMU_RATA.thigh_acc_kjc.(trial_num)(:,walk_ind_a(n):walk_ind_b(n)); % cut data down to selected section near impact
        
        IMU_RATA_impact_walk.(step) = IMU_RATA.RATA.(trial_num)(walk_ind_a(n):walk_ind_b(n));
        [v_asy, ind_asy] = max(acc_shank_impact.(step)(2,:)); %value and index of peak y acceleration for shank near initial contact time
        [v_asx_min, ind_asx_min] = min(acc_shank_impact.(step)(1,:)); %minimum anterior acceleration value (neg = posterior) near impact.
        [v_asx_max, ind_asx_max] = max(acc_shank_impact.(step)(1,:)); %max anterior acceleration value  near impact.
        [v_atx_min, ind_atx_min] = min(acc_thigh_impact.(step)(1,:)); % min ant. acc. for thigh near impact.
        [v_atx_max, ind_atx_max] = max(acc_thigh_impact.(step)(1,:)); % max ant. acc for thigh near impact.
        [v_rata, ind_rata] = max(IMU_RATA_impact_walk.(step));
  
        
        IMU_RATA_walk_impact.(step) = IMU_RATA.RATA.(trial_num)(walk_ind_a(n) + ind_rata - 100:  walk_ind_a(n) + ind_rata + 100); %RATA values near initial contact for each step
 
        %%%%%%%%%%%%%
        %%%%%%%%%%%%%%%
        %%%RATA calculated using shifted acceleration curves based on
        %%%cross-correlation between the shank KJC point X-acceleration
        %%%(shank-anterior direction) and the thigh KJC pint x-acceleration
        %%%(shank-anterior direction). 
        
        [x1,x2,shift_pk] = alignsignals(IMU_RATA.shank_acc_kjc.(trial_num)(1,walk_ind_a(n):walk_ind_b(n)), IMU_RATA.thigh_acc_kjc.(trial_num)(1,walk_ind_a(n):walk_ind_b(n))); % difference between peak minimums (valleys) of anterior acceleration.
        shift_pk_struct.(step) = shift_pk;
        
        shank_kjc_shift.(step) = IMU_RATA.shank_acc_kjc.(trial_num)(1,walk_ind_a(n):walk_ind_b(n));
        thigh_kjc_shift.(step) = IMU_RATA.thigh_acc_kjc.(trial_num)(1,walk_ind_a(n)+shift_pk : walk_ind_b(n) + shift_pk);
        thigh_kjc_unshift.(step) = IMU_RATA.thigh_acc_kjc.(trial_num)(1,walk_ind_a(n) : walk_ind_b(n) );
        
        pk2pk.(step) = IMU_RATA.shank_acc_kjc.(trial_num)(1,walk_ind_a(n):walk_ind_b(n)) - IMU_RATA.thigh_acc_kjc.(trial_num)(1,walk_ind_a(n)+shift_pk : walk_ind_b(n) + shift_pk);
        [v_rata_pk2pk, ind_rata_pk2pk] = max(pk2pk.(step)); % peak RATA within selected section.
        pk2pk_impact.(step) =  IMU_RATA.shank_acc_kjc.(trial_num)(1,walk_ind_a(n) + ind_rata_pk2pk - 100:  walk_ind_a(n) + ind_rata_pk2pk + 100) - IMU_RATA.thigh_acc_kjc.(trial_num)(1,walk_ind_a(n)+shift_pk + ind_rata_pk2pk - 100 : walk_ind_a(n) + shift_pk + ind_rata_pk2pk + 100);
        
        %%% Identify Peak to Peak, Valley to Valley, Peak to
        %%% Valley differences
        zci = @(v) find(v(:).*circshift(v(:), [-1 0]) <= 0);   % function to find indices where gradient of anterior acceleration vectros cross zero
        
        shank_grad_zeros.(step) = zci(gradient(shank_kjc_shift.(step)(1:100))); %indices where gradient of shank anterior acceleration crosses zero (peak/valleys)
        thigh_grad_zeros.(step) = zci(gradient(thigh_kjc_unshift.(step)(1:100))); %indices where gradient of thigh anterior acceleration (shank frame) crosses zero (peaks/valleys)
        
       
        
%%%assume shank has first peak/valley, delete elements from thigh  zero-crossings vector until this is true.
        while shank_grad_zeros.(step)(1) > thigh_grad_zeros.(step)(1) %
            thigh_grad_zeros.(step)(1) =[];
           % shank_grad_zeros.(step)(end) = [];
        end
        
% %  %%%delete  multiple zero crossings from shank that occur before first thigh crossing
%        [ind_a,~] = find(shank_grad_zeros.(step)(:) < thigh_grad_zeros.(step)(1)); 
%        if length(ind_a) >= 2 && mod(length(ind_a),2) == 1
%            shank_grad_zeros.(step)(ind_a(2:end)) = []; %keep the first element
%        elseif  length(ind_a) >=2 && mod(length(ind_a),2) == 0
%           shank_grad_zeros.(step)(ind_a(1:end-2,end)) = []; % keep second to last element
%        elseif length(ind_a) == 2
%            shank_grad_zeros.(step)(2) = [];
%            
%        end

% check if 2 inflection points occur consecutively between 5 or less indices (0.0125s)--> delete both
% check if 3 inflection points occur consecutively with 5 or less indices
% between each --> keep first inflection point, delete second two.
% check if 4 inflection points occur consecutively with 5 or less indices
% between each --> keep second to last inflection point, delete others
% check if 5 inflection points occur consecutively with 5 or less indices
% betweene each --> keep last one , delete others. 
% interval = 1;
% while interval < length(shank_grad_zeros.(step))-1
%        consec_pts = 1;
%        streak = 1;
%     while streak == 1
% 
%         if shank_grad_zeros.(step)(interval+1) - shank_grad_zeros.(step)(interval) <= 5 && interval < length(shank_grad_zeros.(step)) -1
%             consec_pts = consec_pts + 1;
%             interval = interval +1;
%         else
%             streak = 0;
%         end
%     end
%     
%     if consec_pts == 2
%         shank_grad_zeros.(step)(interval) = [];
%         shank_grad_zeros.(step)(interval-1) = [];
%       interval = interval -2;
%     elseif consec_pts == 3
%         shank_grad_zeros.(step)(interval-1:interval) = [];
%         interval = interval -2;
%     elseif consec_pts == 4
%         shank_grad_zeros.(step)([interval-3:interval-2,interval]) = [];
%         interval = interval  -3;
%     elseif consec_pts == 5
%         shank_grad_zeros.(step)(interval-4:interval-1) = [];
%         interval = interval -4;
%     else
%     end
%     
%     interval = interval+1;
% end
%         
%  %%% REPEAT Consecutive peak deletion for Thigh   
% interval = 1;
% while interval < length(thigh_grad_zeros.(step))-1
%        consec_pts = 1;
%        streak = 1;
%     while streak == 1
%   
%         if thigh_grad_zeros.(step)(interval+1) - thigh_grad_zeros.(step)(interval) <= 5 && interval < length(thigh_grad_zeros.(step)) -1
%             consec_pts = consec_pts + 1;
%             interval = interval +1;
%         else
%             streak = 0;
%         end
%     end
%     
%     if consec_pts == 2
%         thigh_grad_zeros.(step)(interval) = [];
%         thigh_grad_zeros.(step)(interval-1) = [];
%       interval = interval -2;
%     elseif consec_pts == 3
%         thigh_grad_zeros.(step)(interval-1:interval) = [];
%         interval = interval -2;
%     elseif consec_pts == 4
%         thigh_grad_zeros.(step)([interval-3:interval-2,interval]) = [];
%         interval = interval  -3;
%     elseif consec_pts == 5
%         thigh_grad_zeros.(step)(interval-4:interval-1) = [];
%         interval = interval -4;
%     else
%     end
%     
%     interval = interval+1;
% end

%     check if 3 or more inflection points in shank signal occur between
%    thigh inflection point index A and B + 6. 6 was chosen because the
%    third infleciton point often happens just after (~1-4 indices after) the
%    second thigh inflection point
%    interval = 1;
%    while  interval < length(shank_grad_zeros.(step))-1
%       
% 
%            
%            [ind, ~] = find(shank_grad_zeros.(step)(:) >= thigh_grad_zeros.(step)(interval) & shank_grad_zeros.(step)(:) < thigh_grad_zeros.(step)(interval+1)+ 6);
%            if length(ind) >= 3 && mod(length(ind),2) == 1
%                shank_grad_zeros.(step)(ind(2:end)) = []; %delete inflection points after the first one of 3 in the shank grad zeros vector
%                thigh_grad_zeros.(step)(end-(length(ind) - 2):end) = []; % delete inflection points at the end of the thigh grad zeros vector so vectors have same length.
%                interval = length(shank_grad_zeros.(step)) - 1;
%            elseif length(ind) >= 4 && mod(length(ind),2) == 0
%                shank_grad_zeros.(step)(ind(2:end-1)) = []; %keep first and last inflection point of series, delete others in between
%                thigh_grad_zeros.(step)(end-(length(ind) - 3):end) = []; % delete inflection points at the end of the thigh grad zeros vector so vectors have same length.
%            else
%                interval = interval + 1;
%            end
%    end
       
  %%%% MAKE PEAK/VALLEY INDEX VECTORS SAME LENGTH
        spillover = abs(length(shank_grad_zeros.(step)) - length(thigh_grad_zeros.(step))); % difference between zero-crossing vectors
        if length(shank_grad_zeros.(step)) > length(thigh_grad_zeros.(step)) %shank vector is longer
           
            shank_grad_zeros.(step)(length(shank_grad_zeros.(step)) + 1 - spillover : length(shank_grad_zeros.(step))) = []; % delete elements in longer vector
            
        elseif length(thigh_grad_zeros.(step)) > length(shank_grad_zeros.(step)) %thigh vector is longer
            
            thigh_grad_zeros.(step)(length(thigh_grad_zeros.(step)) + 1 - spillover : length(thigh_grad_zeros.(step))) = [];  % delete elements in longer vector

        else
            %number of elements in both vectors are equal, do nothing
        end
        
        

        
        
        %%%Line up vallyes and peaks
    inflection_s.(step) = shank_kjc_shift.(step)(shank_grad_zeros.(step)) - shank_kjc_shift.(step)(shank_grad_zeros.(step) + 2);
    
    for i = 1:length(inflection_s.(step))
        if inflection_s.(step)(i) < 0 %inflection point is a valley --> zero crossing is < two indices after zero crossing
            pk_val_s.(step)(i,1) = 0;
        else
            pk_val_s.(step)(i,1) = 1; %inflection point is a peak
        end
    end
    
    inflection_t.(step) = thigh_kjc_unshift.(step)(thigh_grad_zeros.(step)) - thigh_kjc_unshift.(step)(thigh_grad_zeros.(step) + 2);
    
    for i = 1:length(inflection_t.(step))
        if inflection_t.(step)(i) < 0 %inflection point is a valley --> zero crossing is < two indices after zero crossing
            pk_val_t.(step)(i,1) = 0;
        else
            pk_val_t.(step)(i,1) = 1; %inflection point is a peak
        end
    end
    
    shank_zeros_shift = 0;
    while not(isequal(pk_val_s.(step)(1),pk_val_t.(step)(1))) % while the peaks and valleys do not line up
        pk_val_s.(step)(1) = []; % delete the first zero crossing peak/valley in the shank vector
        pk_val_t.(step)(end) = []; %delete the last zero crossing peak/valley in the thigh vector to keep vectors same length
        shank_zeros_shift = shank_zeros_shift + 1; % keep track the amount of shifts until the peaks and valleys line up.
       
    end
    
   shank_grad_zeros.(step) = shank_grad_zeros.(step)(1+shank_zeros_shift:end);
   thigh_grad_zeros.(step) = thigh_grad_zeros.(step)(1:end-shank_zeros_shift);
   
    
   index = 1;
%    while shank_grad_zeros.(step)(index) <= 32
%        if index == 1 || index == 2
%            inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%            inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index);
%        elseif  index >= length(shank_grad_zeros.(step)) - 2
%             left_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index-2));
%             cent_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index));
%             if cent_diff <= left_diff
%                 inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                 inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index);
%             else
%                 inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                 infleciton_compare.(step)(index,2) = thigh_grad_zeros.(step)(index - 2);
%             end
%       
%        elseif index > 2 && index < length(shank_grad_zeros.(step)) - 2 && mod(index,2) == 1 %if inflection point is odd
%            cent_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index));
%            right_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index+2));
%            left_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index -2));
%            
%            if cent_diff <= right_diff && cent_diff <= left_diff
%                inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index);
%            elseif right_diff < cent_diff && right_diff < left_diff
%                inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index+ 2);
%            elseif left_diff < cent_diff && left_diff < right_diff
%                inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index - 2);
%            end
%        
%        elseif index > 2 && index < length(shank_grad_zeros.(step)) - 2 && mod(index,2) == 0 % if inflection point is even
%            
%            cent_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index));
%            right_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index+2));
%            left_diff = abs(shank_grad_zeros.(step)(index) - thigh_grad_zeros.(step)(index -2));
%             if cent_diff <= right_diff && cent_diff <= left_diff
%                inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index);
%            elseif right_diff < cent_diff && right_diff < left_diff
%                inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index+ 2);
%            elseif left_diff < cent_diff && left_diff < right_diff
%                inflection_compare.(step)(index,1) = shank_grad_zeros.(step)(index);
%                inflection_compare.(step)(index,2) = thigh_grad_zeros.(step)(index - 2);
%             end
%        end     
%        index = index + 1;
%    end
%        
           
   
        pk2pk_diff.(step) = max(shank_kjc_shift.(step)(shank_grad_zeros.(step)) - thigh_kjc_unshift.(step)(thigh_grad_zeros.(step))); %max RATA calculated from peak to peak or valley to valley differences
        

        %%%% FIGURE TO SHOW WHICH PEAKS/VALLEYS WERE IDENTIFIED AND
        %%%% ANALYZED
       figure
        t = 0:0.0025:(0.0025*99);
        plot(t,shank_kjc_shift.(step)(1:100),'LineWidth',1);
     hold on
        plot(t,thigh_kjc_unshift.(step)(1:100),'LineWidth',1);
        
        xlim([0,0.25])
        legend('tib-origin','fem-origin')
        xlabel('Time (seconds)')
        ylabel('Acceleration (m/s^2)');
        
        figure
        subplot(3,1,1)
        title({'Peak to Peak / Valley to Valley Difference Analysis';'Anterior Acceleration Derivative'})
        hold on
        t = 0:0.0025:(0.0025*99);
        plot(t,shank_kjc_shift.(step)(1:100));
     
        plot(t,thigh_kjc_unshift.(step)(1:100));
           
       % plot((shank_grad_zeros.(step))*0.0025,shank_kjc_shift.(step)(shank_grad_zeros.(step)),'r*')
        clear numstr
        for i = 1:length(shank_grad_zeros.(step))
            numstr{i} = num2str(i);
        end
        %text((shank_grad_zeros.(step))*0.0025,shank_kjc_shift.(step)(shank_grad_zeros.(step))+10,numstr,'Color','red')
        
        %plot((thigh_grad_zeros.(step))*0.0025,thigh_kjc_unshift.(step)(thigh_grad_zeros.(step)),'k*')
                %text((thigh_grad_zeros.(step))*0.0025,thigh_kjc_unshift.(step)(thigh_grad_zeros.(step))+10,numstr,'Color','black')
        xlim([0,0.25])
        legend('shank','thigh')
        xlabel('Time (seconds)')
        ylabel('Acceleration (m/s^2)');
        
        subplot(3,1,2)
        plot(t,IMU_RATA.shank_acc_kjc.(trial_num)(2,walk_ind_a(n):walk_ind_a(n)+99));
        hold on
        hline(0);
        title('Shank Y ACC')
        
        subplot(3,1,3)
        plot(t, IMU_RATA.shank_gyr.(trial_num)(3,walk_ind_a(n):walk_ind_a(n)+99));
        hold on
        hline(0);
        title('Shank Z GYR')
        
%         subplot(4,1,4)
%         plot(t, IMU_RATA.thigh_gyr.(trial_num)(3,walk_ind_a(n):walk_ind_a(n)+99));
%         hole on
%         hline(0);
%         title('Thigh Z GYR')
        
        
        
 %%%% SINGLE INTEGRATAION RATA => RATV
 t2 = 0:0.0025:199*0.0025;
 shank_integrate.(step) = cumtrapz(t2, shank_kjc_shift.(step)(1:200));
 thigh_integrate.(step) = cumtrapz(t2, thigh_kjc_shift.(step)(1:200));
 thigh_integrate_unshift.(step) = cumtrapz(t2,thigh_kjc_unshift.(step)(1:200));
 
 RATV_shift.(step) = shank_integrate.(step) - thigh_integrate.(step);
 RATV_unshift.(step) = shank_integrate.(step) - thigh_integrate_unshift.(step);
 
 
%         [peaks_window, y] = ginput(4);
%         [valley_window,y] = ginput(4);
%         
%         for c = 1:2:length(peaks_window)
%             pk2pk_diff.(step)((c+1)/2,1) = max(shank_kjc_shift.(step)(peaks_window(c):peaks_window(c+1))) - max(thigh_kjc_unshift.(step)(peaks_window(c):peaks_window(c+1)));
%         end
%         
%         for c = 1:2:length(valley_window)
%             val2val_diff.(step)((c+1)/2,1) = min(thigh_kjc_unshift.(step)(valley_window(c):valley_window(c+1))) - min(shank_kjc_shift.(step)(valley_window(c):valley_window(c+1)));
%         end
            
            %%% RATA PEAK WIDTH- Time it takes to cross zero, peak and then
        %%% cross zero again
        zci = @(v) find(v(:).*circshift(v(:), [-1 0]) <= 0);   % function to find indices where angular velocity vector crosses 0.
    
        %%% UNSHIFTED Signals calculate RATA
        zero_rata = zci(IMU_RATA_walk_impact.(step)(101-15:101+15)); %indices in RATA that are zero-crossings within 37.5ms of peak RATA
        rata_zeros.(step) = zero_rata';
        if length(zero_rata) > 2
            disp(step);
            disp('more than 2 zero crossings detected')
        else
        end
        
        %%%% SHIFTED signals calculate RATA
        zero_rata_pk2pk = zci(pk2pk_impact.(step)(101-25:101+25));
        rata_zeros_pk2pk.(step) = zero_rata_pk2pk';
        if length(rata_zeros_pk2pk) > 2
            disp(step);
            disp('more than 2 zero crossings detected, pk2pk')
        else
        end
        
        %store peak values of RATA, X,Y shank accelerations
        pk_acc_walk.sy.(step) = v_asy; %UNSHIFTED/SHIFTED
        pk_acc_walk.sx_min.(step) = v_asx_min; %UNSHIFTED/SHIFTED
        pk_acc_walk.sx_max.(step) = v_asx_max; %UNSHIFTED/SHIFTED
        pk_rata_walk.(step) = v_rata; %UNSHIFTED
        pk_rata_pk2pk.(step) = v_rata_pk2pk; % SHIFTED
        rata_width.(step) = max(diff(zero_rata)); %UNSHIFTED
        rata_width_pk2pk.(step) = max(diff(zero_rata_pk2pk)); % SHIFTED
        
    end
    
    
end

%% Contribution, how much does the ang. acc. portion of RATA contribute to RATA
for m = 1:length(rata_button_presses)/2
    trial_num = ['tr',num2str(m)];
contribution.tib.(trial_num) = transpose(IMU_RATA.shank_acc_kjc.(trial_num)(1,IMU_RATA.max_RATA_index.(trial_num)-10:IMU_RATA.max_RATA_index.(trial_num)+10));
contribution.fem.(trial_num) = transpose(IMU_RATA.thigh_acc_kjc.(trial_num)(1,IMU_RATA.max_RATA_index.(trial_num)-10:IMU_RATA.max_RATA_index.(trial_num) + 10));
end
%%
%peak 2 peak difference method results
        pk2pk_diff_mat = transpose(struct2mat(pk2pk_diff));

%XCORR method
%peak values of key variable for copy/paste into Excel file. %
pk_rata_walk_mat = transpose(struct2mat(pk_rata_walk)); % peak RATA, UNSHIFTED
pk_rata_pk2pk_walk_mat = transpose(struct2mat(pk_rata_pk2pk)); %peak RATA after shifting curves to alignment, SHIFTED
pk_accy_walk_mat = transpose(struct2mat(pk_acc_walk.sy));% peak Y acc of shank, in shank frame near ground impact of each step
pk_accx_min_walk_mat = transpose(struct2mat(pk_acc_walk.sx_min));  % minimum X acc of shank, in shank frame near ground impact of each step
pk_accx_max_walk_mat = transpose(struct2mat(pk_acc_walk.sx_max)); % max X accof shank in shank frame near ground impact of each step

%RATA widths for copy/paste into Excel file.
rata_width_unshift_mat = transpose(struct2mat(rata_width))*0.0025; %seconds
rata_width_shift_mat = transpose(struct2mat(rata_width_pk2pk))*0.0025; %seconds
shift_time_mat = transpose(struct2mat(shift_pk_struct))*0.0025; % seconds

%%%%%%% PLOT Characteristic Curve - UNSHIFTED
figure
RATA_imu_walk_stdev(1,:) = std(struct2mat(IMU_RATA_walk_impact),0,2);
RATA_imu_walk_stdev(2,:) = RATA_imu_walk_stdev(1,:);
RATA_imu_walk_mean = mean(struct2mat(IMU_RATA_walk_impact),2);
t_axis = -0.125:0.0025:0.125;
xlim([-0.125,0.125])
xticks([-0.125, 0, 0.125]);
xticklabels({-0.125, 0 ,0.125});
yticks([-50, -25, 0, 25, 50])
yticklabels({-50,-25, 0, 25, 50});

shaded_fig = shadedErrorBar(t_axis, RATA_imu_walk_mean(50:150),RATA_imu_walk_stdev(50:150),'lineProps','-b');
fig_title = title('Vertical Jump RATA Characteristic Curve - Dave');
title_pos =  get(fig_title, 'position');
title_pos(2) = 105.5;
set(fig_title, 'position', title_pos);

out_pos = get(gca, 'OuterPosition');
out_pos = [0,0,1.05,0.95];
set(gca,'OuterPosition', out_pos);

xlabel('Time(seconds)')
ylabel('RATA (m/s^2)')
ylim([-50,50])
set(gca,'FontSize',12)


%%%%%%% PLOT Characteristic Curve - SHIFTED
figure
RATA_pk2pk_walk_stdev(1,:) = std(struct2mat(pk2pk_impact),0,2);
RATA_pk2pk_walk_stdev(2,:) = RATA_pk2pk_walk_stdev(1,:);
RATA_pk2pk_walk_mean = mean(struct2mat(pk2pk_impact),2);
t_axis = -0.125:0.0025:0.125;
xlim([-0.125,0.125])
xticks([-0.125, 0, 0.125]);
xticklabels({-0.125, 0 ,0.125});
yticks([-50, -25, 0, 25, 50])
yticklabels({-50,-25, 0, 25, 50});

shaded_fig = shadedErrorBar(t_axis, RATA_pk2pk_walk_mean (50:150),RATA_pk2pk_walk_stdev(50:150),'lineProps','-g');
fig_title = title('Vertical Jump RATA Characteristic Curve, Shifted - Dave, 18Hz Cutoff');
title_pos =  get(fig_title, 'position');
title_pos(2) = 105.5;
set(fig_title, 'position', title_pos);

out_pos = get(gca, 'OuterPosition');
out_pos = [0,0,1.05,0.95];
set(gca,'OuterPosition', out_pos);

xlabel('Time(seconds)')
ylabel('RATA (m/s^2)')
ylim([-50,50])
set(gca,'FontSize',12)


%% Filter analysis

figure
subplot(3,1,1)
plot(IMU_RATA.shank_acc_kjc.tr1(1,:))
hold on
plot(IMU_RATA.thigh_acc_kjc.tr1(1,:))
title('IMU1 filt vs. raw Acc x,y, gyr y, z')

subplot(3,1,2)
plot(IMU_RATA.IMU1_raw.tr1(1,:))
hold on
plot(IMU_RATA.IMU1_filt.tr1(1,:))
plot(IMU_RATA.IMU1_raw.tr1(2,:))
plot(IMU_RATA.IMU1_filt.tr1(2,:))
legend('accx_raw', 'acc_x_filt', 'acc_y_raw', 'acc_y_filt')

subplot(3,1,3)
plot(IMU_RATA.IMU1_raw.tr1(5,:))
hold on
plot(IMU_RATA.IMU1_filt.tr1(5,:))
plot(IMU_RATA.IMU1_raw.tr1(6,:))
plot(IMU_RATA.IMU1_filt.tr1(6,:))
legend('gyr_y_raw','gyr_y_filt','gyr_z_raw','gyr_z_filt')

figure
subplot(3,1,1)
plot(IMU_RATA.shank_acc_kjc.tr1(1,:))
hold on
plot(IMU_RATA.thigh_acc_kjc.tr1(1,:))
title('IMU2 filt vs. raw Acc x,y, gyr y, z')

subplot(3,1,2)
plot(IMU_RATA.IMU2_raw.tr1(1,:))
hold on
plot(IMU_RATA.IMU2_filt.tr1(1,:))
plot(IMU_RATA.IMU2_raw.tr1(2,:))
plot(IMU_RATA.IMU2_filt.tr1(2,:))
legend('accx_raw', 'acc_x_filt', 'acc_y_raw', 'acc_y_filt')

subplot(3,1,3)
plot(IMU_RATA.IMU2_raw.tr1(5,:))
hold on
plot(IMU_RATA.IMU2_filt.tr1(5,:))
plot(IMU_RATA.IMU2_raw.tr1(6,:))
plot(IMU_RATA.IMU2_filt.tr1(6,:))
legend('gyr_y_raw','gyr_y_filt','gyr_z_raw','gyr_z_filt')




%%%%%%%%%%%%%%%%%%%%%
% figure
% 
% subplot(2,1,1)
% plot(IMU_RATA.shank_acc_kjc.(trial_num)(1,pk_ind.sy.(trial_num)-100:pk_ind.sy.(trial_num)+100))
% hold on
% plot(IMU_RATA.thigh_acc_kjc.(trial_num)(1,pk_ind.sy.(trial_num)-100:pk_ind.sy.(trial_num)+100))
% title('Acceleration in Shank Anterior Direction')
% legend('Point on Shank', 'Point on Thigh')
% 
% subplot(2,1,2)
% plot(IMU_RATA.RATA.(trial_num)(pk_ind.sy.(trial_num)-100:pk_ind.sy.(trial_num)+100));
% hold on
% yyaxis right
%  plot(IMU_RATA.shank_acc_kjc.(trial_num)(2,pk_ind.sy.(trial_num)-100:pk_ind.sy.(trial_num)+100))
%  plot(IMU_RATA.thigh_acc_kjc_tframe.(trial_num)(2,pk_ind.sy.(trial_num)-100:pk_ind.sy.(trial_num)+100),'--k')
%  title('RATA and vertical acceleration of Shank & Thigh')
% legend('RATA','Shank Y acc', 'Thigh Y acc')
% 
% figure
% hold on
% for n = 1:length(rata_button_presses)/2
%      trial_num = ['tr',num2str(n)];
% plot(IMU_RATA.RATA.(trial_num)(pk_ind.sy.(trial_num)-100:pk_ind.sy.(trial_num)+100))
% end
% % subplot(2,2,4)
% % ATV = cumtrapz(0.0025,IMU_RATA.RATA.tr1);
% % ATV_shifted = detrend(ATV,2);
% % plot(ATV_shifted)
% % title('Anterior Tibial Velocity - Detrend')
% % 
% % subplot(2,2,2)
% % plot(ATV)
% % title('Anterior Tibial Velocity- Integration Drift')
% % 
% figure
% hold on
% title('RATA moving Average, window size = 100ms')
% a = movmean(IMU_RATA.shank_acc_kjc.tr1(1,:),25);
% b =movmean(IMU_RATA.thigh_acc_kjc.tr1(1,:),25);
% plot(a-b)



%% PosVecs (USER INPUT, button presses  if more than 1 trial collected  (optional) )
% posvec_button_presses = [11:14]; %%user input here!!%%%
% posvecsB = getposvecs_fxn(data,DCMs, posvec_button_presses);




function importedData = importData_fxn(data_directory,folder_name1,folder_name2)

%% IMPORT IMU#1 (THIGH IMU) Data

sessionData1 = importSession(strcat(data_directory,folder_name1)); %load recording session
[resampledSessionData1, time] = resampleSession(sessionData1, 0.0025); %re-sampled at 400Hz to have consistent time intervals
IMUname1 = sessionData1.deviceNames{1}; %IMU #1 Name (Identifier)
sensors1 = resampledSessionData1.(IMUname1).sensors; %Sensor data (accelerometer,gyro)
button1 = round(sessionData1.(IMUname1).button.time ./ 0.0025); %  indices of button presses (sample #)
quat1 = resampledSessionData1.(IMUname1).quaternion.vector; % orientation quaternion output

%% IMPORT IMU#2 (SHANK IMU) Data
sessionData2 = importSession(strcat(data_directory,folder_name2));
[resampledSessionData2, time] = resampleSession(sessionData2, 0.0025); %re-sampled at 400Hz to have consistent time intervals
IMUname2 = sessionData2.deviceNames{1}; %IMU #2 Name (Identifier)
sensors2 = resampledSessionData2.(IMUname2).sensors; % Sensor data (accelerometer,gyro)


quat2 = resampledSessionData2.(IMUname2).quaternion.vector; % orientation quaternion output

if isfield(sessionData2.(IMUname2),'button') %if IMU#2 button was pressed
    button2 = round(sessionData2.(IMUname2).button.time ./ 0.0025);  %  indices of button presses (sample #)
    
    importedData = struct('sensors1',sensors1,'sensors2',sensors2,'quat1',quat1, 'quat2', quat2,...
    'button1', button1,'button2',button2);
else
    
importedData = struct('sensors1',sensors1,'sensors2',sensors2,'quat1',quat1, 'quat2', quat2,...
    'button1', button1);%'button2',button2);
end
%sensors structure contains accelerometer and gyroscope 3xn vectors as well
%as 1xn vectors for each individual X,Y, or Z component. These are accessed
%by using the fields: accelerometerX(Y,Z); gyroscopeX(Y,Z);
%accelerometerVector (3xn); gyroscopeVector;
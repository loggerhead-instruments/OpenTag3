% ot_load
% loads OpenTag3 CSV files into matrices
% need to delete columns after gyroscope for this to work

[FileName,PathName,FilterIndex] = uigetfile({'*.CSV','CSV files (*.CSV)'},'Select a CSV file');
if isequal(FileName,0)|isequal(PathName,0)
   return
end
    
FileName
cd(PathName);
M = csvread(FileName, 1, 0);

% Calibration factors
accelFullRange = 16.0;
gyroFullRange = 1000.0;
magFullRange = 4800.0e-6; % Tesla
magFullRange = magFullRange * 10000.0; % 1 tesla = 10000 Gauss

accel_cal = accelFullRange / 32768.0;  
gyro_cal = gyroFullRange / 32768.0; 
mag_cal = magFullRange / 32768.0;

% calibrate and re-orient magnetometer to align with accel and gyro
INER=[];
INER.accel(:,1) = M(:,1) * accel_cal * -1;      %X
INER.accel(:,2) = M(:,2) * accel_cal; %Y
INER.accel(:,3) = M(:,3) * accel_cal * -1; %Z

INER.mag(:,1) = M(:,5) * mag_cal;     % X: swap magnetometer X & Y
INER.mag(:,2) = M(:,4) * mag_cal * -1;          % Y
INER.mag(:,3) = M(:,6) * mag_cal;     % Z

INER.gyro(:,1) = M(:,7) * gyro_cal;   % X
INER.gyro(:,2) = M(:,8) * gyro_cal * -1;   % Y
INER.gyro(:,3) = M(:,9) * gyro_cal;        % Z

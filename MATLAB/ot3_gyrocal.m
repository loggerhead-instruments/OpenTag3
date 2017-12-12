% ot_gyrocal
% Calculates offsets in gyroscope readings
% offsets stored in gyro_offset vector [x y z]


% 1. Make a 10 s recording with OpenTag where it is flat and not moving.
% The first and last 20% will not be used for the calibration
% 2. Run ot_gyrocal

% Typical offsets may look like: [0.3156   -0.1651   -0.3899]

ot_load;  % load data file

%% Limit range of data anlaysis (if changes when device picked up)
startgyro=floor(0.2*length(INER.gyro));
stopgyro=floor(0.8*length(INER.gyro));
%startmag=1;
%stopmag=length(INER.mag);

% Calulate offset as maximim value + minimum value/2
gyrooffset=(mean(INER.gyro(startgyro:stopgyro,:)));


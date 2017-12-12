% ot_AHRS
% based on the ExampleScript.m from Madgwick

% See: http://www.x-io.co.uk/open-source/

% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with OpenTag data. 
% The OpenTag data sensor axes and signs are adjusted to match the example
% data set provided with Madgwick code and assuming a 'NED' orientation of
% the tag.
%
% 'NED' means
% X is pointing north
% Y positive is East
% Z positive is Down
% Positive values follow right-hand rule.  Thumb along axis, with positive
% direction of rotation in direction of fingers curl

% The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches �90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected
% 10/12/2017    dmann           OpenTag3 support

% 
%     This file is part of eulerAHRS.
% 
%     ot_AHRS is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, version 3 of the License.
% 
%     ot_AHRS is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with eulerAHRS.  If not, see <http://www.gnu.org/licenses/>.
    
%     addpath('quaternion_library');      % include quaternion library

%% Import and plot sensor data
ot3_load;
srate = 50;
plots_on=1;

%% Settings for OpenTag oriented in potted fashion with SD card up. 
accelsign=[1 1 1]; %NED  % sign (y, x, z with swapping)
magsign=[1 1 1]; %NED
gyrosign=[1 1 1]; %NED

%%  Check for magnetometer offset
% See ot_magcal.m for how to calculate this
if(exist('magoffset'))
    magoffset
else
    magoffset=[0 0 0];
    disp('No magnetometer offset correction');
end

% See ot_magcal.m for how to calculate this
if(exist('gyrooffset'))
    gyrooffset
else
    gyrooffset=[0 0 0];
    disp('No gyroscope offset correction');
end

% Vectors to hold rearranged data
Accelerometer=[];
Magnetometer=[];
Gyroscope=[];

k=2;  % read in y first, so can swap with x
Accelerometer=[Accelerometer INER.accel(:,k)*accelsign(k)];
Gyroscope=[Gyroscope, (INER.gyro(:,k)-gyrooffset(k))*gyrosign(k)];    
Magnetometer=[Magnetometer (INER.mag(:,k)-magoffset(k))*magsign(k)]; %correct magoffset

k=1;  
Accelerometer=[Accelerometer INER.accel(:,k)*accelsign(k)];
Gyroscope=[Gyroscope, (INER.gyro(:,k)-gyrooffset(k))*gyrosign(k)];    
Magnetometer=[Magnetometer (INER.mag(:,k)-magoffset(k))*magsign(k)]; %correct magoffset

k=3;  % read in z
Accelerometer=[Accelerometer INER.accel(:,k)*accelsign(k)];
Gyroscope=[Gyroscope, (INER.gyro(:,k)-gyrooffset(k))*gyrosign(k)];    
Magnetometer=[Magnetometer (INER.mag(:,k)-magoffset(k))*magsign(k)]; %correct magoffset

[r,c]=size(Accelerometer);  
time=[0:r-1]/srate;  % create time scale in seconds

if(plots_on==1)
    figure('Name', 'Sensor Data');
    axis(1) = subplot(3,1,1);
    hold on;
    plot(time, Gyroscope(:,1), 'r');
    plot(time, Gyroscope(:,2), 'g');
    plot(time, Gyroscope(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Angular rate (deg/s)');
    title('Gyroscope');
    hold off;
    axis(2) = subplot(3,1,2);
    hold on;
    plot(time, Accelerometer(:,1), 'r');
    plot(time, Accelerometer(:,2), 'g');
    plot(time, Accelerometer(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    title('Accelerometer');
    hold off;
    axis(3) = subplot(3,1,3);
    hold on;
    plot(time, Magnetometer(:,1), 'r');
    plot(time, Magnetometer(:,2), 'g');
    plot(time, Magnetometer(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
    linkaxes(axis, 'x');
end

AHRS = MadgwickAHRS('SamplePeriod', 1/srate, 'Beta', 1);
%AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches �90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

if(plots_on==1)
    figure('Name', 'Euler Angles');
    hold on;
    plot(time, euler(:,1), 'r');
    plot(time, euler(:,2), 'g');
    plot(time, euler(:,3), 'b');
    title('Euler angles');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('roll', 'pitch', 'heading');
    hold off;
end
%figure
%iPsi = atan2(-Magnetometer(:,2),Magnetometer(:,1));
%plot(radtodeg(iPsi));
%title('Heading assuming flat--using x and y mag');
%% End of script
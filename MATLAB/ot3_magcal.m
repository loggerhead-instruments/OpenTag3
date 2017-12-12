% ot_magcal
% Calculates offsets in magnetometer readings
% Does not calculate elliptical offset
% offsets stored in mag_offset vector [x y z]


% 1. Make a recording with OpenTag where you rotate it about all axis. 
% 2. Run ot_magcal

% Typical offsets may look like: [0.3156   -0.1651   -0.3899]

ot3_load;  % load data file

%% Limit range of data anlaysis (if changes when device picked up)
%startmag=2200;
%stopmag=length(INER.mag)-3000;
startmag=10;
stopmag=length(INER.mag)-10;

% Calulate offset as maximim value + minimum value/2
magoffset=(max(INER.mag(startmag:stopmag,:))+min(INER.mag(startmag:stopmag,:)))/2;

%% Plot data
figure(10)
subplot(2,2,1)
plot(INER.mag(startmag:stopmag,1:3:end),INER.mag(startmag:stopmag,2:3:end),'.');
xlabel('X uncorrected');
ylabel('Y uncorrected');

subplot(2,2,2)
plot(INER.mag(startmag:stopmag,1:3:end),INER.mag(startmag:stopmag,3:3:end),'.');
xlabel('X uncorrected');
ylabel('Z uncorrected');

subplot(2,2,3)
plot(INER.mag(startmag:stopmag,1:3:end)-magoffset(1),INER.mag(startmag:stopmag,2:3:end)-magoffset(2),'.');
xlabel('X corrected');
ylabel('Y corrected');

subplot(2,2,4)
plot(INER.mag(startmag:stopmag,1:3:end)-magoffset(1),INER.mag(startmag:stopmag,3:3:end)-magoffset(3),'.');
xlabel('X corrected');
ylabel('Z corrected');


%%
% 3D plot
%
%figure(11)
%plot3(INER.mag(startmag:stopmag,1:3:end)-magoffset(1),INER.mag(startmag:stopmag,2:3:end)-magoffset(2), INER.mag(startmag:stopmag,3:3:end)-magoffset(3),'.');
%title('3D plot with offset corrected');
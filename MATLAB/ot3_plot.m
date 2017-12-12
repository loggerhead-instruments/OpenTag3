% ot_plot
figure('Name', 'Sensor Data');
n=1;

if(bitand(INER.SensorType,32)) % accelerometer
    npts=length(INER.accel);
    t=[0:npts-1]/(1000000/INER.SPus);  %time scale in seconds 
    labels(n)={'g'};
    n=n+1;
    axis(2) = subplot(3,1,1);
    hold on;
    plot(t, INER.accel(:,1), 'r');
    plot(t, INER.accel(:,2), 'g');
    plot(t, INER.accel(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    title('Accelerometer');
    hold off;
 end
 
 if(bitand(INER.SensorType,16)) %magnetometer
    npts=length(INER.mag);
    t=[0:npts-1]/(1000000/INER.SPus);  %time scale in seconds 
    labels(n)={'gauss'};
    n=n+1;
    axis(3) = subplot(3,1,2);
    hold on;
    plot(t, INER.mag(:,1), 'r');
    plot(t, INER.mag(:,2), 'g');
    plot(t, INER.mag(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
 end
 
 
 if(bitand(INER.SensorType,8)) %gyro
    npts=length(INER.gyro);
    t=[0:npts-1]/(1000000/INER.SPus);  %time scale in seconds 
    labels(n)={'deg/s'};
    n=n+1;
    axis(1) = subplot(3,1,3);
    hold on;
    plot(t, INER.gyro(:,1), 'r');
    plot(t, INER.gyro(:,2), 'g');
    plot(t, INER.gyro(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Angular rate (deg/s)');
    title('Gyroscope');
    hold off;
 end

linkaxes(axis, 'x');


if(isempty(PTMP)==0)
    npts=length(depth);
    ptime=[0:npts-1]/(1000000/PTMP.SPus);  %time scale in seconds 
    figure(2)
    subplot(2,1,1)
    plot(ptime,depth);
    title('Dive Profile');
    ylabel('Depth (m)');

    subplot(2,1,2)
    plot(ptime,temperature);
    title('Temperature'); 
    ylabel('C');
    xlabel('Time (s)');
end
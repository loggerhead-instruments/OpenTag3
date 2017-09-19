% ot3_read

fid = fopen('e:/test.16');
data = fread(fid, 'int16');
figure()
subplot(2,1,1)
plot(data(1:6:end), 'r')
hold on
plot(data(2:6:end), 'g')
plot(data(3:6:end), 'b')
subplot(2,1,2)
plot(data(4:6:end), 'r')
hold on
plot(data(5:6:end), 'g')
plot(data(6:6:end), 'b')
hold off
fclose(fid);
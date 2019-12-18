function outfile = pi2mat(filename)
%
%  This function takes a csv input file name from the SenseLogger object
%  Updates should be consistent across this file and the python SenseLogger
%  File
%
%  Encodings
%     Message 0 - Raw gyro measurements
%         0, elapsed time,gyro x, gyro y, gyro z - measured in rad/sec
%     Message 1 - Raw accel measurements
%         1,time,accel x, accel y, accel z - measured in g
%     Message 2 - Raw mag measurement
%         2,time, mag x, mag y, mag z - measured in microTesla 
%     Message 3 - Compensated gyro measurements (IMU)
%         3, elapsed time,gyro x, gyro y, gyro z - measured in rad/sec
%     Message 4 - Compensated accel measurements (IMU)
%         4,time,accel x, accel y, accel z - measured in g
%     Message 5 - Compensated mag measurement (IMU)
%         5,time, mag x, mag y, mag z - measured in microTesla
%     Message 6 - Temp/Misc Sensor Data
%         6,time, temp (deg), humidity (%), pressure (mm HG)
%
%    
% 
%  
messageNames = { 'rawGyro' 'rawAccel' 'rawMag' 'gyro' 'accel' 'mag' 'temp' };

outfile = [ filename(1:end-4) '.mat' ];
% filename = ('C:\\Users\\E704652\\Documents\\Boat\\VMS\\heading_swing_30.csv')

data = csvread(filename);

for msg = 1:length(messageNames)
%     messageNames{msg}
    msgTag = msg-1;
    msgIndex = data(:,1) == msgTag;
    thisMsgTime = data(msgIndex,2);
    msgData = data(msgIndex,3:end);
    eval(sprintf('%s.time=thisMsgTime;',messageNames{msg}));
    eval(sprintf('%s.data=msgData;',messageNames{msg}))
    if msg == 1
        save(outfile,messageNames{msg})
    else
        save(outfile,messageNames{msg},'-append')
    end
    
end
    

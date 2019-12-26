function outfile = pi2matv3(filename)
%
%
%  Encodings
%     Message 0 - raw gyro measurements
%         0,time,gyro x, gyro y, gyro z - measured in rad/sec
%     Message 1 - raw accel measurements
%         1,time,accel x, accel y, accel z - measured in g
%     Message 2 - mag measurement
%         2,time, mag x, mag y, mag z - measured in microTesla 
%     Message 3 - Temp, pressure, humidity data
%         3,time, temp, pressure, humidity
%     Message 4 - Tilt data
%         4,time, tiltHybrid, tiltAccel, tiltGyro - tiltHybrid(computed heading) tiltAccel(tilt based on accel data) tiltGyro(tilt based on gyro data)
%     Message 5 - control data
%         5,time, tiltHybrid, setTilt, error, ctrlOutput - tiltHybrid(computed heading) setTilt(commanded input) error(estimated error) ctrlOutput(control output from PID)
%     Message 6 - motor  data
%         6,time, error, ctrlOutput, drive1, drive2, motorPwm - rror(estimated error) ctrlOutput(control output from PID) drive1 drive2(motor direction command) motorPwm(0-100 motor command)
      
  
messageNames = { 'gyro' 'accel' 'mag' 'temp' 'att' 'control' 'motor'};

outfile = [ filename(1:end-4) '.mat' ];
# filename = ('C:\\Octave\\Octave-5.1.0.0\\home\\e702248\\*.csv')

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
    

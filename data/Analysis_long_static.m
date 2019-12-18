% Script to analyze static SH Noise Errors
basename = 'C:\\Users\\E704652\\Documents\\Boat\\DasBoot\\data\\'
static = load([basename 'S2_long_static'])

mGyro = mean(rad2deg(static.gyro.data))
% mGyro =
% 
%    -0.0005    0.0005    0.0055
sGyro = std(rad2deg(static.gyro.data))
% sGyro =
% 
%     0.0829    0.0930    0.1152
mAccel = mean(static.accel.data)
% mAccel =
% 
%    -0.0160   -0.0227    0.9990
sAccel = std(static.accel.data)
% sAccel =
% 
%     0.0023    0.0029    0.0141
%     Note Z Accel has "chatter" on it
%     One dist starting at 0.937 but on the right of center
%     One dist starting at 1.0615 but on the left of center
%     Delta is 0.1245  (~ 1/8 g)
mMag = mean(static.mag.data)
% mMag =
% 
%    29.2008   27.4016   36.0569
%    This was mainly pointed north.  
%    Well off of [11.886 -.0039 -33.875 ] truth
sMag = std(static.mag.data)
% sMag =
% 
%     0.4087    0.4496    0.5400
%     There is a step in the Y Mag data
%     The data is not really noise, there are some temporal components
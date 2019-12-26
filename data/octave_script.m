# Created by Octave 5.1.0, Fri Dec 13 08:26:20 2019 GMT <unknown@MN51LT9PPB2H2>
run1 = load('C:\\Octave\\Octave-5.1.0.0\\home\\e702248\\Heading Cal Files\\Heading_cal_log_run1_12_12_19.mat');
run2 = load('C:\\Octave\\Octave-5.1.0.0\\home\\e702248\\Heading Cal Files\\Heading_cal_log_run2_12_12_19.mat');
run3 = load('C:\\Octave\\Octave-5.1.0.0\\home\\e702248\\Heading Cal Files\\Heading_cal_log_run3_12_12_19.mat');

HorzMag = 11.8868 % Horizong Mag Field Magnitude in MSP (micro Tesla)

# Figure 1.  Plot Mag x vs. Mag y for each of the 3 runs
figure
plot(run1.mag.data(:,1),run1.mag.data(:,2),'.')
hold on
grid on
plot(run2.mag.data(:,1),run2.mag.data(:,2),'r.')
plot(run3.mag.data(:,1),run3.mag.data(:,2),'g.')

# Calculate Hard Iron shift (run1cal) and scale to size (run1cal2)
span1 = max(run1.mag.data)-min(run1.mag.data);
b1 = min(run1.mag.data)+(span1)/2;
run1Cal = run1;
run1Cal.mag.data = run1.mag.data - b1;
spanCal = max(run1Cal.mag.data)-min(run1Cal.mag.data);
run1Cal.mag.hMag = sqrt(run1Cal.mag.data(:,1).^2 + run1Cal.mag.data(:,2).^2);
Ts = [ HorzMag/(spanCal(:,1)/2) 0;
       0                        HorzMag/(spanCal(:,2)/2) ];

run1Cal2 = run1Cal;
run1Cal2.mag.data(:,1) = Ts(1,1)*run1Cal.mag.data(:,1);
run1Cal2.mag.data(:,2) = Ts(2,2)*run1Cal.mag.data(:,2);

# Figure 2. Plot run 1 un compensated (black), Shift due to hard Iron (Red) and scaled (green)
figure
plot(run1.mag.data(:,1),run1.mag.data(:,2),'k.')
grid on
hold on
plot(run1Cal.mag.data(:,1),run1Cal.mag.data(:,2),'r.')
plot(run1Cal2.mag.data(:,1),run1Cal2.mag.data(:,2),'g.')

# Figure 3 Compute mag heading from Magx and Magy, Plot all three.  Black is uncompensated
# blue is hard Iron compensated, and green is hardiron and scale compensated
figure
plot(run1.mag.time,rad2deg(atan2(run1.mag.data(:,2),run1.mag.data(:,1))),'k')
hold on
grid on
plot(run1Cal.mag.time,rad2deg(atan2(run1Cal.mag.data(:,2),run1Cal.mag.data(:,1))),'b')
plot(run1Cal2.mag.time,rad2deg(atan2(run1Cal2.mag.data(:,2),run1Cal2.mag.data(:,1))),'g')

# use Hardiron and scale compesation from run 1 to calibrate run 2
run2Cal = run2;
run2Cal.mag.data = run2.mag.data - b1;
run2Cal2 = run2Cal;
run2Cal2.mag.data(:,1) = Ts(1,1)*run2Cal.mag.data(:,1);
run2Cal2.mag.data(:,2) = Ts(2,2)*run2Cal.mag.data(:,2);

#Figure 4 Plot run 2 uncompensated (black), run 2 Shift due to hard Iron from run 1 (Red) 
# and run 2 scaled fron run 1 (green)
figure
plot(run2.mag.data(:,1),run2.mag.data(:,2),'k.')
grid on
hold on
plot(run2Cal.mag.data(:,1),run2Cal.mag.data(:,2),'r.')
plot(run2Cal2.mag.data(:,1),run2Cal2.mag.data(:,2),'g.')

# Figure 5, Mag heading run1 calibrated to self (black) and run2 calibrated with run 1 data(red)
figure
plot(run1Cal2.mag.time,rad2deg(atan2(run1Cal2.mag.data(:,2),run1Cal2.mag.data(:,1))),'k')
hold on
grid on
plot(run2Cal2.mag.time,rad2deg(atan2(run2Cal2.mag.data(:,2),run2Cal2.mag.data(:,1))),'r')

#use Hardiron and scale compesation from run 1 to calibrate run 3
run3Cal = run3;
run3Cal.mag.data = run3.mag.data - b1;
run3Cal2 = run3Cal;
run3Cal2.mag.data(:,1) = Ts(1,1)*run3Cal.mag.data(:,1);
run3Cal2.mag.data(:,2) = Ts(2,2)*run3Cal.mag.data(:,2);

# Figure 6 Plot run 3 uncompensated (black), run 2 Shift due to hard Iron from run 1 (Red) 
# and run 2 scaled fron run 1 (green)
figure
plot(run3.mag.data(:,1),run3.mag.data(:,2),'k.')
grid on
hold on
plot(run3Cal.mag.data(:,1),run3Cal.mag.data(:,2),'r.')
plot(run3Cal2.mag.data(:,1),run3Cal2.mag.data(:,2),'g.')

# Figure 7, Mag heading run1 calibrated to self (black) and run2 calibrated with run 1 data(red)
# and run3 calibrated with run 1 data(green) and run 3 calibrated to its own data (Magenta)
figure
plot(run1Cal2.mag.time,rad2deg(atan2(run1Cal2.mag.data(:,2),run1Cal2.mag.data(:,1))),'k')
hold on
grid on
plot(run2Cal2.mag.time,rad2deg(atan2(run2Cal2.mag.data(:,2),run2Cal2.mag.data(:,1))),'r')
plot(run3Cal2.mag.time,rad2deg(atan2(run3Cal2.mag.data(:,2),run3Cal2.mag.data(:,1))),'g')
span3 = max(run3.mag.data)-min(run3.mag.data);
b3 = min(run3.mag.data)+(span1)/2;
run3Cal3 = run3;
run3Cal3.mag.data = run3.mag.data - b3;
run3Cal4 = run3Cal3;
run3Cal4.mag.data(:,1) = Ts(1,1)*run3Cal3.mag.data(:,1);
run3Cal4.mag.data(:,2) = Ts(2,2)*run3Cal3.mag.data(:,2);
plot(run3Cal4.mag.time,rad2deg(atan2(run3Cal4.mag.data(:,2),run3Cal4.mag.data(:,1))),'m')
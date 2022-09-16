clc
close all
clear

%% define lowpass bytterworth filter and dihedral model

[b, a] = butter(6, 0.20);
window_movAvg = 10;
theta_potd = [-70.0390548676547, 105.372402307992];
theta_pwtd = [-0.0017409816453928, 53.5611863806371];
RevP_coeff_9V = [25.866635482391400,-1.664910993036515e+02,4.030483719450837e+02,-4.325309182694595e+02,1.730907713055474e+02];

%% pwm = 200-700 adaptive

% plot intensity_300_1

currentCase = intensityData;
currentCase.Filename = '0817_inten_700_001.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
currentCase.stableInterval = [1328, 2400];
currentCase.Pot = currentCase.readPot();
currentCase.Pot = currentCase.Pot(3:end);
currentCase.airVolt = currentCase.readAirVolt();
currentCase.airVolt = currentCase.airVolt(3:end);
currentCase.Serv = currentCase.readServ();
currentCase.Serv = currentCase.Serv(3:end);
currentCase.VeloX = currentCase.readVeloX();
currentCase.VeloX = currentCase.VeloX(3:end);
currentCase.posX = currentCase.readPosX();
currentCase.posX = currentCase.posX(3:end);
currentCase.posZ = currentCase.readPosZ();
currentCase.posZ = currentCase.posZ(3:end);
currentCase.posX = filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX));
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

currentCase.Dihedral = theta_potd(2) + currentCase.Pot*theta_potd(1);
currentCase.Dihedral = movAver(currentCase.Dihedral, window_movAvg);
currentCase.Comm = theta_pwtd(2) + currentCase.Serv*theta_pwtd(1);

currentCase.airVelo = polyval(RevP_coeff_9V, currentCase.airVolt);
currentCase.airVoltFilt = movAver(currentCase.airVolt, window_movAvg);
currentCase.airVeloFilt = polyval(RevP_coeff_9V, currentCase.airVoltFilt);
currentCase.stabAirVolt = mean(currentCase.airVoltFilt(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.stabPosX = mean(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)))+0.8;
currentCase.stabPosZ = mean(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)))-1.1;
currentCase.stabOtPitch = mean(currentCase.otPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(1);
sgtitle('pwm = 700 adaptive');

subplot(2,2,1);
hold on;
grid on;
yyaxis left;
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt, 'r-');
ylabel('time history of airflow voltage values [V]');
yyaxis right;
plot(currentCase.airVelo, 'Color', [0.9290 0.6940 0.1250]);
plot(currentCase.airVeloFilt, 'b-');
plot(currentCase.airVeloFilt-currentCase.VeloX, 'c-');
plot((currentCase.airVeloFilt-currentCase.VeloX)./cosd(currentCase.filtOtPitch), 'm-');
legend('Volt airflow true', 'Volt airflow filt', 'Vair on board', 'Vair filtered', 'Vair VeloX-free', 'Vair VeloX-free filtered')
ylim([-2, 5]);
xlabel('time [ms]');
ylabel('time history of airspeed values [m/s]');

subplot(2,2,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,2,3);
hold on;
grid on;
yyaxis left;
plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r-');
ylabel('Position deviation [m]');
% yyaxis right;
% plot(currentCase.otPitch, 'g');
% % plot(currentCase.imuPitch);
% plot(currentCase.filtOtPitch, 'm-');
% line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
% line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
% line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
% patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
legend('posX','posZ', 'pitch OT', 'OT filtered', 'strike', 'x peak', 'z peak', 'stable interval');
% ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('pitch angle from OT [deg]');

subplot(2,2,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

% subplot(2,3,4);
% hold on;
% grid on;
% plot(currentCase.posZ-1.1);
% line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
% line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
% patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
% ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
% xlabel('time [ms]');
% ylabel('heights derivation from 1.1m [m]');



% subplot(2,3,6);
% hold on;
% grid on;
% plot(currentCase.otPitch);
% plot(currentCase.imuPitch);
% plot(currentCase.filtOtPitch);
% legend('Optitrack', 'Imu', 'OT filtered');
% line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
% patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
% ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
% xlabel('time [ms]');
% ylabel('time history of pitch angle [deg]');

intensity_700_1_ad = currentCase;

%% pwm = 200-700 before

% plot intensity_700_1

currentCase = intensityData;
currentCase.Filename = '0707_intensity_700_1.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
currentCase.stableInterval = [1328, 2400];
currentCase.Pot = currentCase.readPot();
currentCase.Pot = currentCase.Pot(3:end);
currentCase.airVolt = currentCase.readAirVolt();
currentCase.airVolt = currentCase.airVolt(3:end);
currentCase.Serv = currentCase.readServ();
currentCase.Serv = currentCase.Serv(3:end);
currentCase.VeloX = currentCase.readVeloX();
currentCase.VeloX = currentCase.VeloX(3:end);
currentCase.posX = currentCase.readPosX();
currentCase.posX = currentCase.posX(3:end);
currentCase.posZ = currentCase.readPosZ();
currentCase.posZ = currentCase.posZ(3:end);
currentCase.posX = filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX));
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

currentCase.Dihedral = theta_potd(2) + currentCase.Pot*theta_potd(1);
currentCase.Dihedral = movAver(currentCase.Dihedral, window_movAvg);
currentCase.Comm = theta_pwtd(2) + currentCase.Serv*theta_pwtd(1);

currentCase.airVelo = polyval(RevP_coeff_9V, currentCase.airVolt);
currentCase.airVoltFilt = movAver(currentCase.airVolt, window_movAvg);
currentCase.airVeloFilt = polyval(RevP_coeff_9V, currentCase.airVoltFilt);
currentCase.stabAirVolt = mean(currentCase.airVoltFilt(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.stabPosX = mean(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)))+0.8;
currentCase.stabPosZ = mean(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)))-1.1;
currentCase.stabOtPitch = mean(currentCase.otPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(2);
sgtitle('pwm = 700 before');

subplot(2,2,1);
hold on;
grid on;
yyaxis left;
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt, 'r-');
ylabel('time history of airflow voltage values [V]');
yyaxis right;
plot(currentCase.airVelo, 'Color', [0.9290 0.6940 0.1250]);
plot(currentCase.airVeloFilt, 'b-');
plot(currentCase.airVeloFilt-currentCase.VeloX, 'c-');
plot((currentCase.airVeloFilt-currentCase.VeloX)./cosd(currentCase.filtOtPitch), 'm-');
legend('Volt airflow true', 'Volt airflow filt', 'Vair on board', 'Vair filtered', 'Vair VeloX-free', 'Vair VeloX-free filtered')
ylim([-2, 5]);
xlabel('time [ms]');
ylabel('time history of airspeed values [m/s]');

subplot(2,2,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,2,3);
hold on;
grid on;
yyaxis left;
plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r-');
ylabel('Position deviation [m]');
% yyaxis right;
% plot(currentCase.otPitch, 'g');
% % plot(currentCase.imuPitch);
% plot(currentCase.filtOtPitch, 'm-');
% line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
% line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
% line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
% patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
legend('posX','posZ', 'pitch OT', 'OT filtered', 'strike', 'x peak', 'z peak', 'stable interval');
% ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('pitch angle from OT [deg]');

subplot(2,2,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

intensity_700_1 = currentCase;

%% compare

figure(3);
hold on;
grid on;
plot(intensity_700_1.posX+0.8, 'b');
plot(intensity_700_1_ad.posX+0.8, 'r');
plot(intensity_700_1.posZ-1.1, 'c');
plot(intensity_700_1_ad.posZ-1.1, 'm');
legend('x before', 'x ad', 'z before', 'z ad');
xlabel('time [ms]');
ylabel('Position deviation [m]');
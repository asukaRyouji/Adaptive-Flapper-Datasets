clc
close all
% clear

%% define lowpass bytterworth filter and dihedral model

[b, a] = butter(6, 0.20);
window_movAvg = 10;
theta_potd = [-70.0390548676547, 105.372402307992];
theta_pwtd = [-0.0017409816453928, 53.5611863806371];
RevP_coeff_9V = [25.866635482391400,-1.664910993036515e+02,4.030483719450837e+02,-4.325309182694595e+02,1.730907713055474e+02];

%% pwm = 200-700 4s none adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_na_4_5_1.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
currentCase.stableInterval = [541, 1668];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(1021:2127)-mean(currentCase.filtOtPitch(1021:2127))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(1);
sgtitle('pwm = 600 none adaptive T = 4s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');


subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_na_4_5_1 = currentCase;

%% pwm = 200-600 3s none adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_na_3_5_3.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
% currentCase.stableInterval = [524, 1688];
currentCase.stableInterval = [524, 1898];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(656:1832)-mean(currentCase.filtOtPitch(656:1832))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(2);
sgtitle('pwm = 600 none adaptive T = 3s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_na_3_5_1 = currentCase;

%% pwm = 200-600 2s none adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_na_2_5_1.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
% currentCase.stableInterval = [703, 2540];
currentCase.stableInterval = [703, 2215];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(798:2578)-mean(currentCase.filtOtPitch(798:2578))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(3);
sgtitle('pwm = 600 none adaptive T = 2s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');


subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_na_2_5_1 = currentCase;

%% pwm = 200-600 1.334s none adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_na_13_5_2.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
% currentCase.stableInterval = [904, 3140];
% currentCase.stableInterval = [1182, 2896];
currentCase.stableInterval = [1159, 3140];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(1115:3050)-mean(currentCase.filtOtPitch(1115:3050))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(4);
sgtitle('pwm = 600 none adaptive T = 1.333s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');


subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_na_13_5_1 = currentCase;

%% pwm = 200-700 4s adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_ad2_4_5_1.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
currentCase.stableInterval = [1282, 3196];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(1068:2945)-mean(currentCase.filtOtPitch(1068:2945))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(5);
sgtitle('pwm = 600 adaptive T = 4s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');


subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_ad2_4_5_1 = currentCase;

%% pwm = 200-700 3s adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_ad2_3_5_1.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
currentCase.stableInterval = [1535, 3778];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(1535:3221)-mean(currentCase.filtOtPitch(1535:3221))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(6);
sgtitle('pwm = 600 adaptive T = 3s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');


subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_ad2_3_5_1 = currentCase;


%% pwm = 200-600 2s adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_ad2_2_5_1.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
currentCase.stableInterval = [1531, 2944];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(1947:3741)-mean(currentCase.filtOtPitch(1947:3741))).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(1550:3850)-mean(currentCase.filtOtPitch(1550:3850))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(7);
sgtitle('pwm = 600 adaptive T = 2s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');


subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_ad2_2_5_1 = currentCase;

%% pwm = 200-600 1.334s adaptive

currentCase = intensityData;
currentCase.Filename = '0825_intensity_600_ad2_13_5_2.csv';
currentCase.startPwm = 200;
currentCase.peakPwm = 700;
currentCase.Strike = 820;
currentCase.windEnd = 3700;
currentCase.txPeak = 993;
currentCase.xPeak = 0.475;
currentCase.tzPeak = 1063;
currentCase.zPeak = 0.200;
currentCase.stableInterval = [1355, 3342];
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
currentCase.stabOtPitch = mean(currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.rmseX = sqrt(mean((currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosX-0.8)).^2));
currentCase.rmseZ = sqrt(mean((currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabPosZ+1.1)).^2));
% currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(currentCase.stableInterval(1):currentCase.stableInterval(2))-(currentCase.stabOtPitch)).^2));
currentCase.rmseOtPitch = sqrt(mean((currentCase.filtOtPitch(1156:3239)-mean(currentCase.filtOtPitch(1156:3239))).^2));

currentCase.filtCurrent = movAver(currentCase.filtCurrent, window_movAvg);
currentCase.stabCurrent = mean(movAver(currentCase.filtCurrent(currentCase.stableInterval(1):currentCase.stableInterval(2)), window_movAvg));

figure(8);
sgtitle('pwm = 600 adaptive T = 1.333s');

subplot(2,3,1);
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

subplot(2,3,2);
hold on;
grid on;
plot(currentCase.Dihedral);
plot(currentCase.Comm);
legend('actual output', 'desired angle');
xlabel('time [ms]');
ylabel('time history of dihedral angle values [deg]');

subplot(2,3,3);
hold on;
grid on;

plot(currentCase.posX+0.8, 'b');
plot(currentCase.posZ-1.1, 'r');

%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
legend('posX','posZ', 'stable interval');
xlabel('time [ms]');
ylabel('position deviation [m]');


subplot(2,3,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.otPitch, 'g');
plot(currentCase.filtOtPitch, 'k');
legend('ot', 'filt');
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

intensity_600_ad2_13_5_1 = currentCase;

%% plot dihedral angles

dihedral = zeros(1, 2*length(intensity_600_na_4_5_1.Dihedral));
command = zeros(1, 2*length(intensity_600_na_4_5_1.Comm));

for i = 1:2063
    dihedral(2*i-1) = intensity_600_na_4_5_1.Dihedral(i);
    dihedral(2*i) = intensity_600_na_4_5_1.Dihedral(i);
    command(2*i-1) = intensity_600_na_4_5_1.Comm(i);
    command(2*i) = intensity_600_na_4_5_1.Comm(i);
end

intensity_600_na_4_5_1.Dihedral = dihedral;
intensity_600_na_4_5_1.Comm = command;

intensity_600_na_4_5_1.Comm = movAver(intensity_600_na_4_5_1.Comm, 20);
intensity_600_na_4_5_1.Dihedral = movAver(intensity_600_na_4_5_1.Dihedral, 20);
intensity_600_ad2_4_5_1.Dihedral = movAver(intensity_600_ad2_4_5_1.Dihedral, 20);

dihedral = zeros(1, 2*length(intensity_600_na_3_5_1.Dihedral));
command = zeros(1, 2*length(intensity_600_na_3_5_1.Comm));

for i = 1:2063
    dihedral(2*i-1) = intensity_600_na_3_5_1.Dihedral(i);
    dihedral(2*i) = intensity_600_na_3_5_1.Dihedral(i);
    command(2*i-1) = intensity_600_na_3_5_1.Comm(i);
    command(2*i) = intensity_600_na_3_5_1.Comm(i);
end

intensity_600_na_3_5_1.Dihedral = dihedral;
intensity_600_na_3_5_1.Comm = command;

intensity_600_ad2_3_5_1.Dihedral = movAver(intensity_600_ad2_3_5_1.Dihedral, 20);
intensity_600_na_3_5_1.Comm = movAver(intensity_600_na_3_5_1.Comm, 20);
intensity_600_na_3_5_1.Dihedral = movAver(intensity_600_na_3_5_1.Dihedral, 20);
% 
% dihedral = zeros(1, 2*length(intensity_700_ad2_2_5_3.Dihedral));
% 
% for i = 1:1537
%     dihedral(2*i-1) = intensity_700_ad2_2_5_3.Dihedral(i);
%     dihedral(2*i) = intensity_700_ad2_2_5_3.Dihedral(i);
% end
% 
% intensity_700_ad2_2_5_3.Dihedral = dihedral;
% 
intensity_600_na_2_5_1.Comm = movAver(intensity_600_na_2_5_1.Comm, 20);
intensity_600_na_2_5_1.Dihedral = movAver(intensity_600_na_2_5_1.Dihedral, 20);
intensity_600_ad2_2_5_1.Dihedral = movAver(intensity_600_ad2_2_5_1.Dihedral, 20);

intensity_600_na_13_5_1.Comm = movAver(intensity_600_na_13_5_1.Comm, 20);
intensity_600_na_13_5_1.Dihedral = movAver(intensity_600_na_13_5_1.Dihedral, 20);
intensity_600_ad2_13_5_1.Dihedral = movAver(intensity_600_ad2_13_5_1.Dihedral, 20);


com_600_4 = comData;
com_600_4.avgDihe = mean(intensity_600_ad2_4_5_1.Dihedral(1153:1346));
com_600_4.stdDihe = std(intensity_600_ad2_4_5_1.Dihedral(1153:1346));
com_600_4.avgComm = mean(intensity_600_na_4_5_1.Comm(1153:1346));
com_600_4.stdComm = std(intensity_600_na_4_5_1.Comm(1153:1346)); 
com_600_4.avgNon = mean(intensity_600_na_4_5_1.Dihedral(1153:1346));
com_600_4.stdNon = std(intensity_600_na_4_5_1.Dihedral(1153:1346));

com_600_3 = comData;
com_600_3.avgDihe = mean(intensity_600_ad2_3_5_1.Dihedral(3492:3647));
com_600_3.stdDihe = std(intensity_600_ad2_3_5_1.Dihedral(3492:3647));
com_600_3.avgComm = mean(intensity_600_na_3_5_1.Comm(3492:3647));
com_600_3.stdComm = std(intensity_600_na_3_5_1.Comm(3492:3647)); 
com_600_3.avgNon = mean(intensity_600_na_3_5_1.Dihedral(3492:3647));
com_600_3.stdNon = std(intensity_600_na_3_5_1.Dihedral(3492:3647));

com_600_2 = comData;
com_600_2.avgDihe = mean(intensity_600_ad2_2_5_1.Dihedral(1764:1931));
com_600_2.stdDihe = std(intensity_600_ad2_2_5_1.Dihedral(1764:1931));
com_600_2.avgComm = mean(intensity_600_na_2_5_1.Comm(1764:1931));
com_600_2.stdComm = std(intensity_600_na_2_5_1.Comm(1764:1931)); 
com_600_2.avgNon = mean(intensity_600_na_2_5_1.Dihedral(1764:1931));
com_600_2.stdNon = std(intensity_600_na_2_5_1.Dihedral(1764:1931));

com_600_1 = comData;
com_600_1.avgDihe = mean(intensity_600_ad2_13_5_1.Dihedral(1344:1485));
com_600_1.stdDihe = std(intensity_600_ad2_13_5_1.Dihedral(1344:1485));
com_600_1.avgComm = mean(intensity_600_na_13_5_1.Comm(1344:1485));
com_600_1.stdComm = std(intensity_600_na_13_5_1.Comm(1344:1485)); 
com_600_1.avgNon = mean(intensity_600_na_13_5_1.Dihedral(1344:1485));
com_600_1.stdNon = std(intensity_600_na_13_5_1.Dihedral(1344:1485));

figure(9);

subplot(1,2,1);
hold on;
grid on;

x = [0.25, 0.50, 0.75, 1.00];
y_comm = [com_700_4.avgComm, com_700_3.avgComm, com_700_2.avgComm, com_700_1.avgComm];
y_dihe = [com_700_4.avgDihe, com_700_3.avgDihe, com_700_2.avgDihe, com_700_1.avgDihe];
y_non = [com_700_4.avgNon, com_700_3.avgNon, com_700_2.avgNon, com_700_1.avgNon];
err_comm = [com_700_4.stdComm, com_700_3.stdComm, com_700_2.stdComm, com_700_1.stdComm-1.5];
err_dihe = [com_700_4.stdDihe, com_700_3.stdDihe, com_700_2.stdDihe, com_700_1.stdDihe];
err_non = [com_700_4.stdNon, com_700_3.stdNon, com_700_2.stdNon, com_700_1.stdNon];

errorbar(x, y_comm, err_comm, '--' , 'Marker', '.', 'Color', 'b' , 'MarkerSize',20, 'MarkerEdgeColor','blue','MarkerFaceColor','blue','CapSize',10);
errorbar(x, y_dihe, err_dihe, '--' , 'Marker', '.', 'Color', 'r' ,  'MarkerSize',20, 'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',10);
errorbar(x, y_non, err_non, '--' , 'Marker', '.',  'Color', 'k' , 'MarkerSize',20, 'MarkerEdgeColor','k','MarkerFaceColor','k','CapSize',10);

xticks([0.25, 0.50, 0.75, 1.00]);
xticklabels({'0.25', '0.33', '0.50', '0.75'});
xlim([0.15, 1.10]);
ylim([-8, 4]);
legend('Commands original', 'Dihedral adaptive', 'Dihedral original');
xlabel('Gust alternating frequency [Hz]');
ylabel('Dihedral angle [deg]');
title('A', 'Position', [-0.1, 4], 'FontSize', 15);

subplot(1,2,2);
hold on;
grid on;

x = [0.25, 0.50, 0.75, 1.00];
y_comm = [com_600_4.avgComm, com_600_3.avgComm, com_600_2.avgComm, com_600_1.avgComm];
y_dihe = [com_600_4.avgDihe, com_600_3.avgDihe, com_600_2.avgDihe, com_600_1.avgDihe];
y_non = [com_600_4.avgNon, com_600_3.avgNon, com_600_2.avgNon, com_600_1.avgNon];
err_comm = [com_600_4.stdComm, com_600_3.stdComm, com_600_2.stdComm, com_600_1.stdComm];
err_dihe = [com_600_4.stdDihe, com_600_3.stdDihe, com_600_2.stdDihe, com_600_1.stdDihe];
err_non = [com_600_4.stdNon, com_600_3.stdNon, com_600_2.stdNon, com_600_1.stdNon];

errorbar(x, y_comm, err_comm, '--' , 'Marker', '.', 'Color', 'b' , 'MarkerSize',20, 'MarkerEdgeColor','blue','MarkerFaceColor','blue','CapSize',10);
errorbar(x, y_dihe, err_dihe, '--' , 'Marker', '.', 'Color', 'r' ,  'MarkerSize',20, 'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',10);
errorbar(x, y_non, err_non, '--' , 'Marker', '.',  'Color', 'k' , 'MarkerSize',20, 'MarkerEdgeColor','k','MarkerFaceColor','k','CapSize',10);

xticks([0.25, 0.50, 0.75, 1.00]);
xticklabels({'0.25 Hz', '0.33 Hz', '0.50 Hz', '0.75 Hz'});
xlim([0.15, 1.10]);
ylim([-8, 4]);
legend('Commands original', 'Dihedral adaptive', 'Dihedral original');
xlabel('Gust alternating frequency [Hz]');
ylabel('Dihedral angle [deg]');
title('B', 'Position', [-0.1, 4], 'FontSize', 15);

figure(10);
hold on;
grid on;
plot(intensity_600_na_4_5_1.Comm);
plot(intensity_600_na_4_5_1.Dihedral);
plot(intensity_600_ad2_4_5_1.Dihedral, 'k');

legend('comm', 'no', 'ad');
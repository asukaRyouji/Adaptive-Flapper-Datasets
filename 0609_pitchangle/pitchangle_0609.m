clc
close all
clear

%% define lowpass bytterworth filter and dihedral model

[b, a] = butter(6, 0.20);
window_movAvg = 10;
theta_potd = [-70.0390548676547, 105.372402307992];
theta_pwtd = [-0.0017409816453928, 53.5611863806371];
RevP_coeff_9V = [25.866635482391400,-1.664910993036515e+02,4.030483719450837e+02,-4.325309182694595e+02,1.730907713055474e+02];

%% pwm = 0

% plot hover_2

currentCase = inwindData;
currentCase.Filename = '0707_intensity_test.csv';
currentCase.Pwm = 0;
% currentCase.Strike = 949;
% currentCase.txPeak = 1054;
% currentCase.xPeak = 0.805;
% currentCase.tzPeak = 1079;
% currentCase.zPeak = 0.258;
currentCase.stableInterval = [1, 355];
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

% plot hover_2

currentCase.Dihedral = theta_potd(2) + currentCase.Pot*theta_potd(1);
currentCase.Dihedral = movAver(currentCase.Dihedral, window_movAvg);
currentCase.Comm = theta_pwtd(2) + currentCase.Serv*theta_pwtd(1);

currentCase.airVelo = polyval(RevP_coeff_9V, currentCase.airVolt);
currentCase.airVoltFilt = movAver(currentCase.airVolt, window_movAvg);
currentCase.airVeloFilt = polyval(RevP_coeff_9V, currentCase.airVoltFilt);
currentCase.stabAirVolt = mean(currentCase.airVoltFilt(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(1);
sgtitle('pwm = 0');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

hover_000 = currentCase;

%% pwm = 200

currentCase.Filename = '0701_pitchangle_200.csv';
currentCase.Pwm = 200;
currentCase.Strike = 1468;
currentCase.txPeak = 1728;
currentCase.xPeak = 0.128;
currentCase.tzPeak = 1559;
currentCase.zPeak = 0.054;
currentCase.stableInterval = [2170, 3300];
currentCase.Pot = currentCase.readPot();
currentCase.Pot = currentCase.Pot(3:end);
currentCase.airVolt = currentCase.readAirVolt();
currentCase.airVolt = currentCase.airVolt(3:end);
currentCase.Serv = currentCase.readServ();
currentCase.Serv = currentCase.Serv(3:end);
currentCase.VeloX = currentCase.readVeloX();
currentCase.VeloX = currentCase.VeloX(3:end);
currentCase.VeloX = movAver(currentCase.VeloX, window_movAvg);
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

% plot hover_200_2, stable 1435 to 1937

currentCase.Dihedral = theta_potd(2) + currentCase.Pot*theta_potd(1);
currentCase.Dihedral = movAver(currentCase.Dihedral, window_movAvg);
currentCase.Comm = theta_pwtd(2) + currentCase.Serv*theta_pwtd(1);
currentCase.Comm = movAver(currentCase.Comm, window_movAvg);

currentCase.airVelo = polyval(RevP_coeff_9V, currentCase.airVolt);
currentCase.airVoltFilt = movAver(currentCase.airVolt, window_movAvg);
currentCase.airVeloFilt = polyval(RevP_coeff_9V, currentCase.airVoltFilt);
currentCase.stabAirVolt = mean(currentCase.airVoltFilt(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.stabPosX = mean(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)))+0.8;
currentCase.stabPosZ = mean(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)))-1.1;
currentCase.stabOtPitch = mean(currentCase.otPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));

currentCase.pitchMax = std(currentCase.otPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)), 1);

currentCase.stdX = std(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.stdZ = std(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(2);
sgtitle('pwm = 200');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-10, 10], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

subplot(2,3,6);
hold on;
grid on;
plot(currentCase.otPitch);
plot(currentCase.imuPitch);
legend('Optitrack', 'Imu');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

hover_200 = currentCase;

%% pwm = 300
% plot hover_300_2

currentCase.Filename = '0701_pitchangle_300_aft.csv';
currentCase.Pwm = 300;
currentCase.Strike = 1150;
currentCase.txPeak = 1745;
currentCase.xPeak = 0.251;
currentCase.tzPeak = 1836;
currentCase.zPeak = 0.100;
currentCase.stableInterval = [2350, 3954];
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

% plot hover_300_2

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

currentCase.stdX = std(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.stdZ = std(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(3);
sgtitle('pwm = 300');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
xlim([0, max(length(currentCase.Dihedral), length(currentCase.Comm))]);
ylim([-15, 20]);

subplot(2,3,3);
hold on;
grid on;
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-10, 10], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7 max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

subplot(2,3,6);
hold on;
grid on;
plot(currentCase.otPitch);
plot(currentCase.imuPitch);
legend('Optitrack', 'Imu');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

hover_300 = currentCase;

%% pwm = 400
% plot hover_400_1

currentCase.Filename = '0701_pitchangle_400_aft.csv';
currentCase.Pwm = 400;
currentCase.Strike = 1374;
currentCase.txPeak = 1679;
currentCase.xPeak = 0.539;
currentCase.tzPeak = 1846;
currentCase.zPeak = 0.141;
currentCase.stableInterval = [2354, 3685];
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

% plot hover_400_1

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

currentCase.stdX = std(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.stdZ = std(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(4);
sgtitle('pwm = 400');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-10, 10], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7 max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

subplot(2,3,6);
hold on;
grid on;
plot(currentCase.otPitch);
plot(currentCase.imuPitch);
legend('Optitrack', 'Imu');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

hover_400 = currentCase;

%% pwm = 500
% plot hover_500_1

currentCase.Filename = '0701_pitchangle_500_aft.csv';
currentCase.Pwm = 500;
currentCase.Strike = 460;
currentCase.txPeak = 529;
currentCase.xPeak = 0.723;
currentCase.tzPeak = 566;
currentCase.zPeak = 0.212;
currentCase.stableInterval = [684, 1093];
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

% plot hover_500_1

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

currentCase.stdX = std(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.stdZ = std(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(5);
sgtitle('pwm = 500');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-10, 10], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7 max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

subplot(2,3,6);
hold on;
grid on;
plot(currentCase.otPitch);
plot(currentCase.imuPitch);
legend('Optitrack', 'Imu');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

hover_500 = currentCase;

%% pwm = 600
% plot hover_600_1

currentCase.Filename = '0701_pitchangle_600.csv';
currentCase.Pwm = 600;
currentCase.Strike = 1502;
currentCase.txPeak = 1743;
currentCase.xPeak = 0.776;
currentCase.tzPeak = 1923;
currentCase.zPeak = 0.267;
currentCase.stableInterval = [2119, 3677];
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

% plot hover_600_1

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

currentCase.stdX = std(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.stdZ = std(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(6);
sgtitle('pwm = 600');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
xlim([0, max(length(currentCase.Dihedral), length(currentCase.Comm))]);
ylim([-15, 20]);

subplot(2,3,3);
hold on;
grid on;
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-10, 10], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7 max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

subplot(2,3,6);
hold on;
grid on;
plot(currentCase.otPitch);
plot(currentCase.imuPitch);
legend('Optitrack', 'Imu');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

hover_600 = currentCase;

%% pwm = 700
% plot hover_700_1

%currentCase.Filename = '0609_pitchangle_hover_pwm_700_1.csv'; % for peak value
currentCase.Filename = '0701_pitchangle_700.csv';
currentCase.Pwm = 700;
currentCase.Strike = 1200;
currentCase.txPeak = 1486;
currentCase.xPeak = 0.825;
currentCase.tzPeak = 1572;
currentCase.zPeak = 0.306;
currentCase.stableInterval = [1644, 3489];
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

% plot hover_700_1

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

currentCase.stdX = std(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.stdZ = std(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(7);
sgtitle('pwm = 700');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-10, 10], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7 max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

subplot(2,3,6);
hold on;
grid on;
plot(currentCase.otPitch);
plot(currentCase.imuPitch);
legend('Optitrack', 'Imu');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

hover_700 = currentCase;

%% pwm = 800
currentCase.Filename = '0701_pitchangle_800.csv';
currentCase.Pwm = 800;
currentCase.Strike = 870;
currentCase.txPeak = 1084;
currentCase.xPeak = 1.014;
currentCase.tzPeak = 1175;
currentCase.zPeak = 0.310;
currentCase.stableInterval = [1585, 2519];
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

currentCase.stabPosX = mean(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)))+0.8;
currentCase.stabPosZ = mean(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)))-1.1;
currentCase.stabOtPitch = mean(currentCase.otPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.pitchMax = std(currentCase.otPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)), 1);

currentCase.stdX = std(currentCase.posX(currentCase.stableInterval(1):currentCase.stableInterval(2)));
currentCase.stdZ = std(currentCase.posZ(currentCase.stableInterval(1):currentCase.stableInterval(2)));

% plot hover_800_2

currentCase.Dihedral = theta_potd(2) + currentCase.Pot*theta_potd(1);
currentCase.Dihedral = movAver(currentCase.Dihedral, window_movAvg);
currentCase.Comm = theta_pwtd(2) + currentCase.Serv*theta_pwtd(1);

currentCase.airVelo = polyval(RevP_coeff_9V, currentCase.airVolt);
currentCase.airVoltFilt = movAver(currentCase.airVolt, window_movAvg);
currentCase.airVeloFilt = polyval(RevP_coeff_9V, currentCase.airVoltFilt);
currentCase.stabAirVolt = mean(currentCase.airVoltFilt(currentCase.stableInterval(1):currentCase.stableInterval(2)));

figure(8);
sgtitle('pwm = 800');

subplot(2,3,1);
hold on;
grid on;
plot(currentCase.airVelo, 'g');
plot(currentCase.airVeloFilt);
plot(currentCase.airVeloFilt-currentCase.VeloX, 'b');
legend('on board', 'filtered', 'VeloX-free')
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
plot(currentCase.airVolt, 'g');
plot(currentCase.airVoltFilt);
legend('true', 'filt');
xlabel('time [ms]');
ylabel('time history of airflow voltage values [V]');

subplot(2,3,4);
hold on;
grid on;
plot(currentCase.posZ-1.1);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--');
line([currentCase.tzPeak, currentCase.tzPeak], [-10, 10], 'Color','red','LineStyle','--');
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posZ)-1.2, max(currentCase.posZ)-1.0]);
xlabel('time [ms]');
ylabel('heights derivation from 1.1m [m]');

subplot(2,3,5);
hold on;
grid on;
plot(currentCase.posX+0.8);
line([currentCase.Strike, currentCase.Strike], [-10, 10], 'Color','red','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-10, 10], 'Color','red','LineStyle','--')
%patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-10,10, 10, -10],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.posX)+0.7 max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('x drviation from -0.8m [m]');

subplot(2,3,6);
hold on;
grid on;
plot(currentCase.otPitch);
plot(currentCase.imuPitch);
legend('Optitrack', 'Imu');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','red','LineStyle','--')
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
ylim([min(currentCase.otPitch)-5 max(currentCase.otPitch)+5]);
xlabel('time [ms]');
ylabel('time history of pitch angle [deg]');

hover_800 = currentCase;

%% mean value of airflow voltage in stable interval

pwm = [hover_200.Pwm hover_300.Pwm, hover_400.Pwm, hover_500.Pwm, hover_600.Pwm, hover_700.Pwm, hover_800.Pwm];
stabPosX = [hover_200.stabPosX, hover_300.stabPosX, hover_400.stabPosX, hover_500.stabPosX, hover_600.stabPosX, hover_700.stabPosX, hover_800.stabPosX];
stabPosZ = [hover_200.stabPosZ, hover_300.stabPosZ, hover_400.stabPosZ, hover_500.stabPosZ, hover_600.stabPosZ, hover_700.stabPosZ, hover_800.stabPosZ];
stabOtPitch = [hover_200.stabOtPitch, hover_300.stabOtPitch, hover_400.stabOtPitch, hover_500.stabOtPitch, hover_600.stabOtPitch, hover_700.stabOtPitch, hover_800.stabOtPitch];
stabAirVolt = [hover_200.stabAirVolt, hover_300.stabAirVolt, hover_400.stabAirVolt, hover_500.stabAirVolt, hover_600.stabAirVolt, hover_700.stabAirVolt, hover_800.stabAirVolt];
windspeed = [0.5000, 0.9310, 1.3340, 1.6550, 2.0660, 2.4160, 2.7010];
windspeed_k = [0, 0.5000, 0.9310, 1.3340, 1.6550, 2.0660, 2.4160, 2.7010];

stabPitch_re = [-hover_200.stabOtPitch, 10.3616, 15.2402, 20.8207, 22.8102, 28.2417, 31.2463];

xPeak = [hover_200.xPeak, hover_300.xPeak, hover_400.xPeak, hover_500.xPeak, hover_600.xPeak, hover_700.xPeak, hover_800.xPeak];
zPeak = [hover_200.zPeak, hover_300.zPeak, hover_400.zPeak, hover_500.zPeak, hover_600.zPeak, hover_700.zPeak, hover_800.zPeak];

drag_est = tand(stabPitch_re)*112.04/1000*9.81;
poly_drag = polyfit(windspeed.*cosd(stabPitch_re), drag_est, 1);
drag_lin = polyval(poly_drag, windspeed.*cosd(stabPitch_re));

Kwind = [1.0673, 0.9264, 0.7958, 0.7189, 0.6068, 0.8577, 0.7712, 0.6731];
Ccorr = [-2.0120, -2.3848, -2.1173, -1.5965, -2.2204, -1.9784, -1.4400];

drag_err_neg = 1.0991*[0.0041, 0.0299, 0.0354, 0.0288, 0.0467, 0.0717, 0.0880];
drag_err_pos = 1.0991*[0.0044, 0.0303, 0.0361, 0.0293, 0.0483, 0.0788, 0.0956];

pos_err_x = [hover_200.stdX hover_300.stdX, hover_400.stdX, hover_500.stdX, hover_600.stdX, hover_700.stdX, hover_800.stdX];
pos_err_z = [hover_200.stdZ hover_300.stdZ, hover_400.stdZ, hover_500.stdZ, hover_600.stdZ, hover_700.stdZ, hover_800.stdZ];

figure(9);
sgtitle('Overall');

subplot(2,3,1);
hold on;
grid on;
plot(windspeed, stabPosX);
plot(windspeed, stabPosZ);
legend('x position average', 'z position average');
xlabel('Continuous wind speed [m/s]');
ylabel('average value when stably hovering [m]');

subplot(2,3,2);
hold on;
grid on;
plot(pwm, xPeak);
plot(pwm, zPeak);
legend('x position maximum', 'z position maximum');
xlabel('PWM');
ylabel('maximum value [m]');

subplot(2,3,3);
hold on;
grid on;
plot(pwm, -stabOtPitch);
xlabel('PWM');
ylabel('pitch angle average values when stably hovering [deg]');

subplot(2,3,4);
hold on;
grid on;
plot(pwm, stabAirVolt);
xlabel('PWM');
ylabel('airflow voltage average values when stably hovering [V]');

subplot(2,3,5);
hold on;
grid on;
plot(windspeed, drag_est);
plot(windspeed, drag_lin);
legend('Estimation from pitch angles', 'Linear model');
xlabel('Airspeed [m/s]');
ylabel('Wind drag force [N]');

figure(10);

subplot(1,2,1);
hold on;
grid on;
a = errorbar(windspeed, stabPosX, pos_err_x, '-', 'Marker', '.', 'MarkerSize', 15);
b = errorbar(windspeed, stabPosZ, pos_err_z, '-', 'Marker', '.', 'MarkerSize', 15);
a.Color = 'b';
b.Color = 'r';

legend('Position error X_{G}', 'Position error Z_{G}');
xlabel('Continuous wind speed [m/s]');
ylabel('Position error in stable in-wind hovering [m]');
title('A', 'Position', [-0.1, 0.24], 'FontSize', 15);

subplot(1,2,2);
hold on;
grid on;
plot(windspeed_k, Kwind, '-b.', 'MarkerSize',15);

legend('K_{wind}');
xlabel('Continuous wind speed [m/s]');
ylabel('K_{wind} [-]');
ylim([0.4, 1.10]);
title('B', 'Position', [-0.5, 1.08], 'FontSize', 15);

len_300 = 1:length(hover_300.Dihedral);
len_600 = 1:length(hover_600.Dihedral);

figure(12);
subplot(1,2,1);
hold on;
grid on;
plot(len_300/100, hover_300.Dihedral, 'b');
plot(len_300/100, hover_300.Comm, 'r');
rectangle('Position',[12.10,-7, 8.21, 10], 'EdgeColor','g', 'LineWidth',2)
% patch([1210, 1210, 2031, 2031],[-20, 20, 20, -20],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);

ylim([-20, 20]);
legend('\gamma_{output}', '\gamma_{command}');
xlabel('time [s]');
ylabel('Dihedral angle [deg]');
title('A', 'Position', [-5.0, 18], 'FontSize', 15);

subplot(1,2,2);
hold on;
grid on;
plot(len_600/100, hover_600.Dihedral, 'b');
plot(len_600/100, hover_600.Comm, 'r');
rectangle('Position',[10.00,-7, 6.32, 9], 'EdgeColor','g', 'LineWidth',2)
% patch([1000, 1000, 1632, 1632],[-20, 20, 20, -20],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);

ylim([-20, 20]);
legend('\gamma_{output}', '\gamma_{command}');
xlabel('time [s]');
ylabel('Dihedral angle [deg]');
title('B', 'Position', [-5.0, 18], 'FontSize', 15);

figure(13);
hold on;
grid on;
errorbar(windspeed, drag_est, drag_err_neg, drag_err_pos, '-s', 'MarkerSize', 10);
plot(windspeed, drag_lin);
legend('Estimation from pitch angles', 'Linear model');
xlabel('Continuous wind speed [m/s]');
ylabel('Wind drag force [N]');
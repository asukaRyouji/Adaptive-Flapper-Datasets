clc
close all
clear

%% define lowpass bytterworth filter and dihedral model

[b, a] = butter(6, 0.20);
window_movAvg = 10;
theta_potd = [-70.0390548676547, 105.372402307992];
theta_pwtd = [-0.0017409816453928, 53.5611863806371];
RevP_coeff_9V = [25.866635482391400,-1.664910993036515e+02,4.030483719450837e+02,-4.325309182694595e+02,1.730907713055474e+02];

%% Kp_x = 1.5 kFFx = 19.5

currentCase = intensityData;
currentCase.Filename = '0721_pitch_600_195.csv';
currentCase.startPwm = 0;
currentCase.peakPwm = 400;
currentCase.Strike = 923;
currentCase.windEnd = 2646;
currentCase.txPeak = 1075;
currentCase.xPeak = 0.453;
currentCase.tzPeak = 1234;
currentCase.zPeak = 0.214;
currentCase.stableInterval = [1783, 2792];
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
currentCase.posX = movAver(filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX)), window_movAvg);
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.varPosX = currentCase.calcVar();

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
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

figure(1);
sgtitle('Kp_{x} = 1.5 k_{FF_{x}} = 19.5');

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
yyaxis right;
plot(currentCase.otPitch, 'g');
% plot(currentCase.imuPitch);
plot(currentCase.filtOtPitch, 'm-');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
legend('posX','posZ', 'pitch OT', 'OT filtered', 'strike', 'x peak', 'z peak', 'stable interval');
% ylim([min(currentCase.posX)+0.7, max(currentCase.posX)+0.9]);
xlabel('time [ms]');
ylabel('pitch angle from OT [1737deg]');

subplot(2,2,4);
hold on;
grid on;
plot(currentCase.current);
plot(currentCase.filtCurrent);
legend('current', 'filtered');
xlabel('time [ms]');
ylabel('time history of current [A]');

pitch_600_195 = currentCase;

%% Kp_x = 1.5 kFFx = 20.0

currentCase = intensityData;
currentCase.Filename = '0721_pitch_600_20.csv';
currentCase.startPwm = 0;
currentCase.peakPwm = 500;
currentCase.Strike = 870;
currentCase.windEnd = 2646;
currentCase.txPeak = 1022;
currentCase.xPeak = 0.535;
currentCase.tzPeak = 1088;
currentCase.zPeak = 0.235;
currentCase.stableInterval = [1645, 3000];
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
currentCase.posX = movAver(filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX)), window_movAvg);
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.varPosX = currentCase.calcVar();

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
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

figure(2);
sgtitle('Kp_{x} = 1.5 k_{FF_{x}} = 20.0');

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
yyaxis right;
plot(currentCase.otPitch, 'g');
% plot(currentCase.imuPitch);
plot(currentCase.filtOtPitch, 'm-');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
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

pitch_600_20 = currentCase;

%% Kp_x = 1.5 kFFx = 21.0

currentCase = intensityData;
currentCase.Filename = '0721_pitch_600_21_1.csv';
currentCase.startPwm = 0;
currentCase.peakPwm = 600;
currentCase.Strike = 1200;
currentCase.windEnd = 2646;
currentCase.txPeak = 1358;
currentCase.xPeak = 0.516;
currentCase.tzPeak = 1469;
currentCase.zPeak = 0.254;
currentCase.stableInterval = [1737, 3500];
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
currentCase.posX = movAver(filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX)), window_movAvg);
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.varPosX = currentCase.calcVar();

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
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

figure(3);
sgtitle('Kp_{x} = 1.5 k_{FF_{x}} = 21.0');

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
yyaxis right;
plot(currentCase.otPitch, 'g');
% plot(currentCase.imuPitch);
plot(currentCase.filtOtPitch, 'm-');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
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

pitch_600_21 = currentCase;

%% Kp_x = 1.5 kFFx = 22.0

currentCase = intensityData;
currentCase.Filename = '0721_pitch_600_22.csv';
currentCase.startPwm = 0;
currentCase.peakPwm = 600;
currentCase.Strike = 600;
currentCase.windEnd = 2646;
currentCase.txPeak = 679;
currentCase.xPeak = 0.558;
currentCase.tzPeak = 730;
currentCase.zPeak = 0.251;
currentCase.stableInterval = [867, 1900];
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
currentCase.posX = movAver(filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX)), window_movAvg);
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.varPosX = currentCase.calcVar();

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
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

figure(5);
sgtitle('Kp_{x} = 1.5 k_{FF_{x}} = 22.0');

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
yyaxis right;
plot(currentCase.otPitch, 'g');
% plot(currentCase.imuPitch);
plot(currentCase.filtOtPitch, 'm-');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
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

pitch_600_22 = currentCase;

%% Kp_x = 1.5 kFFx = 23.0

currentCase = intensityData;
currentCase.Filename = '0721_pitch_600_23.csv';
currentCase.startPwm = 0;
currentCase.peakPwm = 600;
currentCase.Strike = 950;
currentCase.windEnd = 2646;
currentCase.txPeak = 1094;
currentCase.xPeak = 0.352;
currentCase.tzPeak = 1269;
currentCase.zPeak = 0.204;
currentCase.stableInterval = [1559, 3180];
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
currentCase.posX = movAver(filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX)), window_movAvg);
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.varPosX = currentCase.calcVar();

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
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

figure(6);
sgtitle('Kp_{x} = 1.5 k_{FF_{x}} = 23.0');

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
yyaxis right;
plot(currentCase.otPitch, 'g');
% plot(currentCase.imuPitch);
plot(currentCase.filtOtPitch, 'm-');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
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

pitch_600_23 = currentCase;

%% Kp_x = 1.5 kFFx = 21.5

currentCase = intensityData;
currentCase.Filename = '0721_pitch_600_215.csv';
currentCase.startPwm = 0;
currentCase.peakPwm = 600;
currentCase.Strike = 750;
currentCase.windEnd = 2646;
currentCase.txPeak = 877;
currentCase.xPeak = 0.426;
currentCase.tzPeak = 948;
currentCase.zPeak = 0.187;
currentCase.stableInterval = [1071, 2300];
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
currentCase.posX = movAver(filloutliers(currentCase.posX, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posX)), window_movAvg);
currentCase.posZ = filloutliers(currentCase.posZ, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.posZ));
currentCase.otPitch = currentCase.readOtPitch();
currentCase.otPitch = currentCase.otPitch(3:end);
currentCase.otPitch = filloutliers(currentCase.otPitch, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.otPitch));
currentCase.imuPitch = currentCase.readImuPitch();
currentCase.imuPitch = currentCase.imuPitch(3:end);
currentCase.varPosX = currentCase.calcVar();

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
currentCase.filtOtPitch = movAver(currentCase.otPitch, window_movAvg);
currentCase.current = currentCase.readCurrent();
currentCase.current = currentCase.current(3:end);
currentCase.filtCurrent = filloutliers(currentCase.current, 'linear', 'movmedian', 3, 'SamplePoints', 1:length(currentCase.current));

currentCase.pitchMax = std(currentCase.otPitch(currentCase.stableInterval(1):currentCase.stableInterval(2)), 1);

figure(4);
sgtitle('Kp_{x} = 1.5 k_{FF_{x}} = 21.5');

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
yyaxis right;
plot(currentCase.otPitch, 'g');
% plot(currentCase.imuPitch);
plot(currentCase.filtOtPitch, 'm-');
line([currentCase.Strike, currentCase.Strike], [-50, 50], 'Color','black','LineStyle','--')
line([currentCase.txPeak, currentCase.txPeak], [-50, 50], 'Color', 'blue','LineStyle','--')
line([currentCase.tzPeak, currentCase.tzPeak], [-50, 50], 'Color', 'red','LineStyle','--');
patch([currentCase.stableInterval(1), currentCase.stableInterval(1), currentCase.stableInterval(2), currentCase.stableInterval(2)],[-50,50, 50, -50],[0 1 0],'FaceAlpha',0.15, 'EdgeAlpha', 0.2);
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

pitch_600_215 = currentCase;
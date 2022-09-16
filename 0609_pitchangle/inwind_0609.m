clc
close all
clear

%% define lowpass bytterworth filter and dihedral model

[b, a] = butter(6, 0.28);
theta_potd = [-70.0390548676547, 105.372402307992];
theta_pwtd = [-0.0017409816453928, 53.5611863806371];

%% read data from CSV files
% read position data derived from OT

% end 2133
pos_ot_hover_2 = readmatrix('0609_pitchangle_hover_2.csv', 'Range', 'K280:M2000');
pitch_ot_hover_2 = readmatrix('0609_pitchangle_hover_2.csv', 'Range', 'O2:O2000');
pitch_imu_hover_2 = readmatrix('0609_pitchangle_hover_2.csv', 'Range', 'C2:C2000');
pot_hover_2 = readmatrix('0609_pitchangle_hover_2.csv', 'Range', 'AG2:AG2000');
serv_hover_2 = readmatrix('0609_pitchangle_hover_2.csv', 'Range', 'R2:R2000');

% end 1912
pos_ot_hover_3 = readmatrix('0609_pitchangle_hover_3.csv', 'Range', 'K2:M1800');
pitch_ot_hover_3 = readmatrix('0609_pitchangle_hover_3.csv', 'Range', 'O2:O1800');
pitch_imu_hover_3 = readmatrix('0609_pitchangle_hover_3.csv', 'Range', 'C2:C1800');
pot_hover_3 = readmatrix('0609_pitchangle_hover_3.csv', 'Range', 'AG2:AG1800');
serv_hover_3 = readmatrix('0609_pitchangle_hover_3.csv', 'Range', 'R2:R1800');

% end 2200
pos_ot_hover_200_1 = readmatrix('0609_pitchangle_hover_pwm_200_1.csv', 'Range', 'K2:M2200');
pitch_ot_hover_200_1 = readmatrix('0609_pitchangle_hover_pwm_200_1.csv', 'Range', 'O2:O2200');
pitch_imu_hover_200_1 = readmatrix('0609_pitchangle_hover_pwm_200_1.csv', 'Range', 'C2:C2200');
pot_hover_200_1 = readmatrix('0609_pitchangle_hover_pwm_200_1.csv', 'Range', 'AG2:AG2200');
serv_hover_200_1 = readmatrix('0609_pitchangle_hover_pwm_200_1.csv', 'Range', 'R2:R2200');
volt_hover_200_1 = readmatrix('0609_pitchangle_hover_pwm_200_1.csv', 'Range', 'AE2:AE2200');

% end 2239
pos_ot_hover_200_2 = readmatrix('0609_pitchangle_hover_pwm_200_2.csv', 'Range', 'K2:M2239');
pitch_ot_hover_200_2 = readmatrix('0609_pitchangle_hover_pwm_200_2.csv', 'Range', 'O2:O2239');
pitch_imu_hover_200_2 = readmatrix('0609_pitchangle_hover_pwm_200_2.csv', 'Range', 'C2:C2239');
pot_hover_200_2 = readmatrix('0609_pitchangle_hover_pwm_200_2.csv', 'Range', 'AG2:AG2239');

pos_ot_hover_300_1 = readmatrix('0609_pitchangle_hover_pwm_300_2.csv', 'Range', 'K2:M2227');
pitch_ot_hover_300_1 = readmatrix('0609_pitchangle_hover_pwm_300_2.csv', 'Range', 'O2:O2227');
pitch_imu_hover_300_1 = readmatrix('0609_pitchangle_hover_pwm_300_2.csv', 'Range', 'C2:C2227');
pot_hover_300_1 = readmatrix('0609_pitchangle_hover_pwm_300_2.csv', 'Range', 'AG2:AG2227');
serv_hover_300_1 = readmatrix('0609_pitchangle_hover_pwm_300_2.csv', 'Range', 'R2:R2227');
volt_hover_300_1 = readmatrix('0609_pitchangle_hover_pwm_300_2.csv', 'Range', 'AE2:AE2227');

pos_ot_hover_400_1 = readmatrix('0609_pitchangle_hover_pwm_400_1.csv', 'Range', 'K2:M2173');
pitch_ot_hover_400_1 = readmatrix('0609_pitchangle_hover_pwm_400_1.csv', 'Range', 'O2:O2173');
pitch_imu_hover_400_1 = readmatrix('0609_pitchangle_hover_pwm_400_1.csv', 'Range', 'C2:C2173');
pot_hover_400_1 = readmatrix('0609_pitchangle_hover_pwm_400_1.csv', 'Range', 'AG2:AG2173');
serv_hover_400_1 = readmatrix('0609_pitchangle_hover_pwm_400_1.csv', 'Range', 'R2:R2173');
volt_hover_400_1 = readmatrix('0609_pitchangle_hover_pwm_400_1.csv', 'Range', 'AE2:AE2173');

pos_ot_hover_500_1 = readmatrix('0609_pitchangle_hover_pwm_500_1.csv', 'Range', 'K2:M2337');
pitch_ot_hover_500_1 = readmatrix('0609_pitchangle_hover_pwm_500_1.csv', 'Range', 'O2:O2337');
pitch_imu_hover_500_1 = readmatrix('0609_pitchangle_hover_pwm_500_1.csv', 'Range', 'C2:C2337');
pot_hover_500_1 = readmatrix('0609_pitchangle_hover_pwm_500_1.csv', 'Range', 'AG2:AG2337');
serv_hover_500_1 = readmatrix('0609_pitchangle_hover_pwm_500_1.csv', 'Range', 'R2:R2337');
volt_hover_500_1 = readmatrix('0609_pitchangle_hover_pwm_500_1.csv', 'Range', 'AE2:AE2337');

pos_ot_hover_600_1 = readmatrix('0609_pitchangle_hover_pwm_600_1.csv', 'Range', 'K2:M2419');
pitch_ot_hover_600_1 = readmatrix('0609_pitchangle_hover_pwm_600_1.csv', 'Range', 'O2:O2419');
pitch_imu_hover_600_1 = readmatrix('0609_pitchangle_hover_pwm_600_1.csv', 'Range', 'C2:C2419');
pot_hover_600_1 = readmatrix('0609_pitchangle_hover_pwm_600_1.csv', 'Range', 'AG2:AG2419');
serv_hover_600_1 = readmatrix('0609_pitchangle_hover_pwm_600_1.csv', 'Range', 'R2:R2419');
volt_hover_600_1 = readmatrix('0609_pitchangle_hover_pwm_600_1.csv', 'Range', 'AE2:AE2419');

pos_ot_hover_700_1 = readmatrix('0609_pitchangle_hover_pwm_700_1.csv', 'Range', 'K2:M2088');
pitch_ot_hover_700_1 = readmatrix('0609_pitchangle_hover_pwm_700_1.csv', 'Range', 'O2:O2088');
pitch_imu_hover_700_1 = readmatrix('0609_pitchangle_hover_pwm_700_1.csv', 'Range', 'C2:C2088');
pot_hover_700_1 = readmatrix('0609_pitchangle_hover_pwm_700_1.csv', 'Range', 'AG2:AG2088');
serv_hover_700_1 = readmatrix('0609_pitchangle_hover_pwm_700_1.csv', 'Range', 'R2:R2088');
volt_hover_700_1 = readmatrix('0609_pitchangle_hover_pwm_700_1.csv', 'Range', 'AE2:AE2088');

pos_ot_hover_800_1 = readmatrix('0609_pitchangle_hover_pwm_800_1.csv', 'Range', 'K2:M1982');
pitch_ot_hover_800_1 = readmatrix('0609_pitchangle_hover_pwm_800_1.csv', 'Range', 'O2:O1982');
pitch_imu_hover_800_1 = readmatrix('0609_pitchangle_hover_pwm_800_1.csv', 'Range', 'C2:C1982');
pot_hover_800_1 = readmatrix('0609_pitchangle_hover_pwm_800_1.csv', 'Range', 'AG2:AG1982');
serv_hover_800_1 = readmatrix('0609_pitchangle_hover_pwm_800_1.csv', 'Range', 'R2:R1982');
volt_hover_800_1 = readmatrix('0609_pitchangle_hover_pwm_800_1.csv', 'Range', 'AE2:AE1982');

pos_ot_hover_800_2 = readmatrix('0609_pitchangle_hover_pwm_800_2.csv', 'Range', 'K2:M2202');
pitch_ot_hover_800_2 = readmatrix('0609_pitchangle_hover_pwm_800_2.csv', 'Range', 'O2:O2202');
pitch_imu_hover_800_2 = readmatrix('0609_pitchangle_hover_pwm_800_2.csv', 'Range', 'C2:C2202');
pot_hover_800_2 = readmatrix('0609_pitchangle_hover_pwm_800_2.csv', 'Range', 'AG2:AG2202');
serv_hover_800_2 = readmatrix('0609_pitchangle_hover_pwm_800_2.csv', 'Range', 'R2:R2202');
volt_hover_800_2 = readmatrix('0609_pitchangle_hover_pwm_800_2.csv', 'Range', 'AE2:AE2202');

%% pwm = 0

% plot hover_2

x_ot_hover_2 = pos_ot_hover_2(:,1);
y_ot_hover_2 = pos_ot_hover_2(:,2);
z_ot_hover_2 = pos_ot_hover_2(:,3);
% x_ot_hover_2 = filter(b, a, pos_ot_hover_2(:,1));
% y_ot_hover_2 = filter(b, a, pos_ot_hover_2(:,2));
% z_ot_hover_2 = filter(b, a, pos_ot_hover_2(:,3));
dihedral_hover_2 = theta_potd(2) + pot_hover_2*theta_potd(1);
comm_hover_2 = theta_pwtd(2) + serv_hover_2*theta_pwtd(1);

figure(1);
sgtitle('pwm = 0');
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_2(10:end), y_ot_hover_2(10:end), z_ot_hover_2(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_2-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_2+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(dihedral_hover_2(3:end));
plot(comm_hover_2(3:end));
legend('actual output', 'desired angle')
xlabel('time 0.01s');
ylabel('time history of dihedral angle values [deg]');

% plot hover_3

x_ot_hover_3 = pos_ot_hover_3(:,1);
y_ot_hover_3 = pos_ot_hover_3(:,2);
z_ot_hover_3 = pos_ot_hover_3(:,3);
% x_ot_hover_3 = filter(b, a, pos_ot_hover_3(:,1));
% y_ot_hover_3 = filter(b, a, pos_ot_hover_3(:,2));
% z_ot_hover_3 = filter(b, a, pos_ot_hover_3(:,3));

figure(2);
sgtitle('pwm = 0')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_3(10:end), y_ot_hover_3(10:end), z_ot_hover_3(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_3-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_3+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(x_ot_hover_3+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

%% pwm = 200
% plot hover_200_2

x_ot_hover_200_2 = pos_ot_hover_200_2(:,1);
y_ot_hover_200_2 = pos_ot_hover_200_2(:,2);
z_ot_hover_200_2 = pos_ot_hover_200_2(:,3);
% x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));

% remove outliers
len_200 = 1:length(x_ot_hover_200_2);
x_ot_hover_200_2 = filloutliers(x_ot_hover_200_2, 'linear', 'movmedian', 3, 'SamplePoints', len_200);
z_ot_hover_200_2 = filloutliers(z_ot_hover_200_2, 'linear', 'movmedian', 3, 'SamplePoints', len_200);
pitch_ot_hover_200_2 = filloutliers(pitch_ot_hover_200_2, 'linear', 'movmedian', 5, 'SamplePoints', len_200);

figure(3);
sgtitle('pwm = 200')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_200_2(10:end), y_ot_hover_200_2(10:end), z_ot_hover_200_2(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_200_2-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_200_2+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

%% pwm = 300
% plot hover_300_1

x_ot_hover_300_1 = pos_ot_hover_300_1(:,1);
y_ot_hover_300_1 = pos_ot_hover_300_1(:,2);
z_ot_hover_300_1 = pos_ot_hover_300_1(:,3);
% x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));

% remove outliers
len_300 = 1:length(x_ot_hover_300_1);
x_ot_hover_300_1 = filloutliers(x_ot_hover_300_1, 'linear', 'movmedian', 3, 'SamplePoints', len_300);
z_ot_hover_300_1 = filloutliers(z_ot_hover_300_1, 'linear', 'movmedian', 3, 'SamplePoints', len_300);
pitch_ot_hover_300_1 = filloutliers(pitch_ot_hover_300_1, 'linear', 'movmedian', 5, 'SamplePoints', len_300);

figure(4);
sgtitle('pwm = 300')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_300_1(10:end), y_ot_hover_300_1(10:end), z_ot_hover_300_1(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_300_1-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_300_1+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(pitch_ot_hover_300_1);
plot(pitch_imu_hover_300_1);
legend('optitrack', 'IMU');
xlabel('time 0.01s');
ylabel('pitch angle from optitrack [degree]');

%% pwm = 400
% plot hover_400_1

x_ot_hover_400_1 = pos_ot_hover_400_1(:,1);
y_ot_hover_400_1 = pos_ot_hover_400_1(:,2);
z_ot_hover_400_1 = pos_ot_hover_400_1(:,3);
% x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));

% remove outliers
len_400 = 1:length(x_ot_hover_400_1);
x_ot_hover_400_1 = filloutliers(x_ot_hover_400_1, 'linear', 'movmedian', 3, 'SamplePoints', len_400);
z_ot_hover_400_1 = filloutliers(z_ot_hover_400_1, 'linear', 'movmedian', 3, 'SamplePoints', len_400);
pitch_ot_hover_400_1 = filloutliers(pitch_ot_hover_400_1, 'linear', 'movmedian', 5, 'SamplePoints', len_400);

figure(5);
sgtitle('pwm = 400')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_400_1(10:end), y_ot_hover_400_1(10:end), z_ot_hover_400_1(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_400_1-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_400_1+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(pitch_ot_hover_400_1);
plot(pitch_imu_hover_400_1);
legend('optitrack', 'IMU');
xlabel('time 0.01s');
ylabel('pitch angle from optitrack [degree]');

%% pwm = 500
% plot hover_500_1

x_ot_hover_500_1 = pos_ot_hover_500_1(:,1);
y_ot_hover_500_1 = pos_ot_hover_500_1(:,2);
z_ot_hover_500_1 = pos_ot_hover_500_1(:,3);
% x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));

% remove outliers
len_500 = 1:length(x_ot_hover_500_1);
x_ot_hover_500_1 = filloutliers(x_ot_hover_500_1, 'linear', 'movmedian', 3, 'SamplePoints', len_500);
z_ot_hover_500_1 = filloutliers(z_ot_hover_500_1, 'linear', 'movmedian', 3, 'SamplePoints', len_500);
pitch_ot_hover_500_1 = filloutliers(pitch_ot_hover_500_1, 'linear', 'movmedian', 5, 'SamplePoints', len_500);

figure(6);
sgtitle('pwm = 500')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_500_1(10:end), y_ot_hover_500_1(10:end), z_ot_hover_500_1(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_500_1-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_500_1+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(pitch_ot_hover_500_1);
plot(pitch_imu_hover_500_1);
legend('optitrack', 'IMU');
xlabel('time 0.01s');
ylabel('pitch angle from optitrack [degree]');

%% pwm = 600
% plot hover_600_1

x_ot_hover_600_1 = pos_ot_hover_600_1(:,1);
y_ot_hover_600_1 = pos_ot_hover_600_1(:,2);
z_ot_hover_600_1 = pos_ot_hover_600_1(:,3);
% x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));

% remove outliers
len_600 = 1:length(x_ot_hover_600_1);
x_ot_hover_600_1 = filloutliers(x_ot_hover_600_1, 'linear', 'movmedian', 3, 'SamplePoints', len_600);
z_ot_hover_600_1 = filloutliers(z_ot_hover_600_1, 'linear', 'movmedian', 3, 'SamplePoints', len_600);
pitch_ot_hover_600_1 = filloutliers(pitch_ot_hover_600_1, 'linear', 'movmedian', 5, 'SamplePoints', len_600);

figure(7);
sgtitle('pwm = 600')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_600_1(10:end), y_ot_hover_600_1(10:end), z_ot_hover_600_1(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_600_1-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_600_1+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(pitch_ot_hover_600_1);
plot(pitch_imu_hover_600_1);
legend('optitrack', 'IMU');
xlabel('time 0.01s');
ylabel('pitch angle from optitrack [degree]');

%% pwm = 700
% plot hover_700_1

x_ot_hover_700_1 = pos_ot_hover_700_1(:,1);
y_ot_hover_700_1 = pos_ot_hover_700_1(:,2);
z_ot_hover_700_1 = pos_ot_hover_700_1(:,3);
% x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));

% remove outliers
len_700 = 1:length(x_ot_hover_700_1);
x_ot_hover_700_1 = filloutliers(x_ot_hover_700_1, 'linear', 'movmedian', 3, 'SamplePoints', len_700);
z_ot_hover_700_1 = filloutliers(z_ot_hover_700_1, 'linear', 'movmedian', 3, 'SamplePoints', len_700);
pitch_ot_hover_700_1 = filloutliers(pitch_ot_hover_700_1, 'linear', 'movmedian', 5, 'SamplePoints', len_700);

figure(8);
sgtitle('pwm = 700')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_700_1(10:end), y_ot_hover_700_1(10:end), z_ot_hover_700_1(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_700_1-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_700_1+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(pitch_ot_hover_700_1);
plot(pitch_imu_hover_700_1);
legend('optitrack', 'IMU');
xlabel('time 0.01s');
ylabel('pitch angle from optitrack [degree]');

%% pwm = 800
% % plot hover_800_1
% 
% x_ot_hover_800_1 = pos_ot_hover_800_1(:,1);
% y_ot_hover_800_1 = pos_ot_hover_800_1(:,2);
% z_ot_hover_800_1 = pos_ot_hover_800_1(:,3);
% % x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% % y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% % z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));
% 
% % remove outliers
% len_800 = 1:length(x_ot_hover_800_1);
% x_ot_hover_800_1 = filloutliers(x_ot_hover_800_1, 'linear', 'movmedian', 5, 'SamplePoints', len_800);
% z_ot_hover_800_1 = filloutliers(z_ot_hover_800_1, 'linear', 'movmedian', 5, 'SamplePoints', len_800);
% pitch_ot_hover_800_1 = filloutliers(pitch_ot_hover_800_1, 'linear', 'movmedian', 5, 'SamplePoints', len_800);
% 
% figure(9);
% sgtitle('pwm = 800')
% subplot(2,2,1);
% hold on;
% grid on;
% plot3(x_ot_hover_800_1(10:end), y_ot_hover_800_1(10:end), z_ot_hover_800_1(10:end), '-');
% plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
% xlabel('x axis');
% ylabel('y axis');
% zlabel('z axis');
% 
% subplot(2,2,2);
% hold on;
% grid on;
% plot(z_ot_hover_800_1-1.1);
% xlabel('time 0.01s');
% ylabel('heights derivation from 1.1m [m]');
% 
% subplot(2,2,3);
% hold on;
% grid on;
% plot(x_ot_hover_800_1+0.8);
% xlabel('time 0.01s');
% ylabel('x drviation from -0.8m [m]');
% 
% subplot(2,2,4);
% hold on;
% grid on;
% plot(pitch_ot_hover_800_1);
% xlabel('time 0.01s');
% ylabel('pitch angle from optitrack [degree]');

% plot hover_800_2

x_ot_hover_800_2 = pos_ot_hover_800_2(:,1);
y_ot_hover_800_2 = pos_ot_hover_800_2(:,2);
z_ot_hover_800_2 = pos_ot_hover_800_2(:,3);
% x_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,1));
% y_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,2));
% z_ot_hover_200_1 = filter(b, a, pos_ot_hover_200_1(:,3));

% remove outliers
len_800 = 1:length(x_ot_hover_800_2);
x_ot_hover_800_2 = filloutliers(x_ot_hover_800_2, 'linear', 'movmedian', 3, 'SamplePoints', len_800);
z_ot_hover_800_2 = filloutliers(z_ot_hover_800_2, 'linear', 'movmedian', 3, 'SamplePoints', len_800);
pitch_ot_hover_800_2 = filloutliers(pitch_ot_hover_800_2, 'linear', 'movmedian', 5, 'SamplePoints', len_800);

figure(9);
sgtitle('pwm = 800')
subplot(2,2,1);
hold on;
grid on;
plot3(x_ot_hover_800_2(10:end), y_ot_hover_800_2(10:end), z_ot_hover_800_2(10:end), '-');
plot3([-0.8, -0.8], [-1.0, -1.0], [0, 1.1]);
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

subplot(2,2,2);
hold on;
grid on;
plot(z_ot_hover_800_2-1.1);
xlabel('time 0.01s');
ylabel('heights derivation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(x_ot_hover_800_2+0.8);
xlabel('time 0.01s');
ylabel('x drviation from -0.8m [m]');

subplot(2,2,4);
hold on;
grid on;
plot(pitch_ot_hover_800_2);
plot(pitch_imu_hover_800_2);
legend('optitrack', 'IMU');
xlabel('time 0.01s');
ylabel('pitch angle from optitrack [degree]');


%% maximum deviation from setpoints after wind strikes

pwm =     [200,    300,    400,    500,    600,    700,    800];
x_max =   [0.2421, 0.3086, 0.4808, 0.7048, 0.8055, 0.8249, 0.9394];
t_x_max = [2.7,    1.77,   1.52,   1.11,   1.06,   0.87,   0.68];
z_max =   [0.0347, 0.1048, 0.1551, 0.2048, 0.2578, 0.2275, 0.2825];
t_z_max = [3.75,   2.1,    2.03,   1.44,   1.29,   1.25,   0.9];

figure(10);
sgtitle('maximum points');
subplot(2,2,1);
hold on;
grid on;
plot(pwm, x_max);
xlabel('pwm');
ylabel('maximum x deviation from -0.8m [m]');

subplot(2,2,2);
hold on;
grid on;
plot(pwm, z_max);
xlabel('pwm');
ylabel('maximum height deviation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(pwm, t_x_max);
xlabel('pwm');
ylabel('time taken to reach x maximum [s]');

subplot(2,2,4);
hold on;
grid on;
plot(pwm, t_z_max);
xlabel('pwm');
ylabel('time taken to reach height maximum [s]');

%% stable hovering position in gust from first dropback minimum to end of wind

pwm          = [200,          300,          400,          500,          600,          700,            800];
% inter_stable = [[1685; 2034], [1292; 1803], [1213; 1860], [1290; 1950], [1305; 1950], [1016; 1800], [1108; 1873]];
inter_stable = [[1685; 2034], [1292; 1803], [1213; 1860], [1290; 1950], [1305; 1950], [1016; 1800], [1110; 1618]];
x_stable     = zeros(1, length(pwm));
z_stable     = zeros(1, length(pwm));
pitch_stable = zeros(1, length(pwm));

x_stable(1) = mean(x_ot_hover_200_2(inter_stable(1, 1):inter_stable(2, 1)))+0.8;
x_stable(2) = mean(x_ot_hover_300_1(inter_stable(1, 2):inter_stable(2, 2)))+0.8;
x_stable(3) = mean(x_ot_hover_400_1(inter_stable(1, 3):inter_stable(2, 3)))+0.8;
x_stable(4) = mean(x_ot_hover_500_1(inter_stable(1, 4):inter_stable(2, 4)))+0.8;
x_stable(5) = mean(x_ot_hover_600_1(inter_stable(1, 5):inter_stable(2, 5)))+0.8;
x_stable(6) = mean(x_ot_hover_700_1(inter_stable(1, 6):inter_stable(2, 6)))+0.8;
x_stable(7) = mean(x_ot_hover_800_2(inter_stable(1, 7):inter_stable(2, 7)))+0.8;

z_stable(1) = mean(z_ot_hover_200_2(inter_stable(1, 1):inter_stable(2, 1)))-1.1;
z_stable(2) = mean(z_ot_hover_300_1(inter_stable(1, 2):inter_stable(2, 2)))-1.1;
z_stable(3) = mean(z_ot_hover_400_1(inter_stable(1, 3):inter_stable(2, 3)))-1.1;
z_stable(4) = mean(z_ot_hover_500_1(inter_stable(1, 4):inter_stable(2, 4)))-1.1;
z_stable(5) = mean(z_ot_hover_600_1(inter_stable(1, 5):inter_stable(2, 5)))-1.1;
z_stable(6) = mean(z_ot_hover_700_1(inter_stable(1, 6):inter_stable(2, 6)))-1.1;
z_stable(7) = mean(z_ot_hover_800_2(inter_stable(1, 7):inter_stable(2, 7)))-1.1;

pitch_stable(1) = mean(pitch_ot_hover_200_1(inter_stable(1, 1):inter_stable(2, 1)));
pitch_stable(2) = mean(pitch_ot_hover_300_1(inter_stable(1, 2):inter_stable(2, 2)));
pitch_stable(3) = mean(pitch_ot_hover_400_1(inter_stable(1, 3):inter_stable(2, 3)));
pitch_stable(4) = mean(pitch_ot_hover_500_1(inter_stable(1, 4):inter_stable(2, 4)));
pitch_stable(5) = mean(pitch_ot_hover_600_1(inter_stable(1, 5):inter_stable(2, 5)));
pitch_stable(6) = mean(pitch_ot_hover_700_1(inter_stable(1, 6):inter_stable(2, 6)));
pitch_stable(7) = mean(pitch_ot_hover_800_2(inter_stable(1, 7):inter_stable(2, 7)));

figure(11);
sgtitle('stable in-gust hovering positions');

subplot(2,2,1);
hold on;
grid on;
plot(pwm, x_stable);
xlabel('pwm');
ylabel('stable x deviation from -0.8m [m]');

subplot(2,2,2);
hold on;
grid on;
plot(pwm, z_stable);
xlabel('pwm');
ylabel('stable height deviation from 1.1m [m]');

subplot(2,2,3);
hold on;
grid on;
plot(pwm, pitch_stable);
xlabel('pwm');
ylabel('pitch angle from optitrack [degree]');

subplot(2,2,4);
hold on;
grid on;
plot(pwm, -tand(pitch_stable));
xlabel('pwm');
ylabel('estimation of wind drag force [times of weight]');

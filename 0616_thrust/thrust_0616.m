
close all


%% define constants

% smaple frquency for PSD
Fs = 100;
[b, a] = butter(4, 0.6);

% store weights and interval for average calculating
weight =     [114.13,      116.18,      117.93,      120.06,      122.01,      124.22,      127.10,      129.10,      132.20,      134.25,       136.00,       139.06];
inter_aver = [[600; 1600], [600; 1600], [600; 1600], [600; 1600], [600; 2600], [420; 1300], [800; 1800], [700; 1700], [1200; 1800],[800; 1800],  [800; 1800],  [600; 1600]];
inter_freq = [[1100; 1600],[1100; 1600],[1200; 1700],[1100; 1600],[2500; 3000],[800; 1300], [1300; 1800],[1200; 1700],[1300; 1800],[1200; 1700], [1400; 1900], [1200; 1700]];
freq =       [[13.6, 15.7],[15.8, 16.0],[15.8, 17.5]];

%% read data from CSV files
% read position data derived from OT

% 2g 1849
pwm_left_2 = readmatrix('0616_thrust_2.csv', 'Range', 'Q3:Q1849');
pwm_right_2 = readmatrix('0616_thrust_2.csv', 'Range', 'T3:T1849');
acc_z_imu_2 = readmatrix('0616_thrust_2.csv', 'Range', 'AA3:AA1849');
acc_x_imu_2 = readmatrix('0616_thrust_2.csv', 'Range', 'Y3:Y1849');
pitch_imu_2 = readmatrix('0616_thrust_2.csv', 'Range', 'C3:C1849');

% 4g 2221                                                         
pwm_left_4 = readmatrix('0620_thrust_4_1.csv', 'Range', 'Q3:Q1823');
pwm_right_4 = readmatrix('0620_thrust_4_1.csv', 'Range', 'T3:T1823');
acc_z_imu_4 = readmatrix('0620_thrust_4_1.csv', 'Range', 'AA3:AA1823');

% 6g
pwm_left_6 = readmatrix('0616_thrust_6.csv', 'Range', 'Q3:Q1885');
pwm_right_6 = readmatrix('0616_thrust_6.csv', 'Range', 'T3:T1885');
acc_z_imu_6 = readmatrix('0616_thrust_6.csv', 'Range', 'AA3:AA1885');

% 8g
pwm_left_8 = readmatrix('0616_thrust_8.csv', 'Range', 'Q3:Q1839');
pwm_right_8 = readmatrix('0616_thrust_8.csv', 'Range', 'T3:T1839');
acc_z_imu_8 = readmatrix('0616_thrust_8.csv', 'Range', 'AA3:AA1839');

% 10g
pwm_left_10 = readmatrix('0616_thrust_10.csv', 'Range', 'Q3:Q3329');
pwm_right_10 = readmatrix('0616_thrust_10.csv', 'Range', 'T3:T3329');
acc_z_imu_10 = readmatrix('0616_thrust_10.csv', 'Range', 'AA3:AA3329');

% 12g
pwm_left_12 = readmatrix('0616_thrust_12.csv', 'Range', 'Q3:Q1400');
pwm_right_12 = readmatrix('0616_thrust_12.csv', 'Range', 'T3:T1400');
acc_z_imu_12 = readmatrix('0616_thrust_12.csv', 'Range', 'AA3:AA1400');

% 15g
pwm_left_15 = readmatrix('0620_thrust_15.csv', 'Range', 'Q3:Q1890');
pwm_right_15 = readmatrix('0620_thrust_15.csv', 'Range', 'T3:T1890');
acc_z_imu_15 = readmatrix('0620_thrust_15.csv', 'Range', 'AA3:AA1890');

% 17g
pwm_left_17 = readmatrix('0620_thrust_17.csv', 'Range', 'Q3:Q1900');
pwm_right_17 = readmatrix('0620_thrust_17.csv', 'Range', 'T3:T1900');
acc_z_imu_17 = readmatrix('0620_thrust_17.csv', 'Range', 'AA3:AA1900');

% 20g
pwm_left_20 = readmatrix('0620_thrust_20.csv', 'Range', 'Q3:Q2039');
pwm_right_20 = readmatrix('0620_thrust_20.csv', 'Range', 'T3:T2039');
acc_z_imu_20 = readmatrix('0620_thrust_20.csv', 'Range', 'AA3:AA2039');

% 22g
pwm_left_22 = readmatrix('0620_thrust_22.csv', 'Range', 'Q3:Q2012');
pwm_right_22 = readmatrix('0620_thrust_22.csv', 'Range', 'T3:T2012');
acc_z_imu_22 = readmatrix('0620_thrust_22.csv', 'Range', 'AA3:AA2012');

% 24g
pwm_left_24 = readmatrix('0620_thrust_24.csv', 'Range', 'Q3:Q2159');
pwm_right_24 = readmatrix('0620_thrust_24.csv', 'Range', 'T3:T2159');
acc_z_imu_24 = readmatrix('0620_thrust_24.csv', 'Range', 'AA3:AA2159');

% 27g
pwm_left_27 = readmatrix('0620_thrust_27.csv', 'Range', 'Q3:Q1907');
pwm_right_27 = readmatrix('0620_thrust_27.csv', 'Range', 'T3:T1907');
acc_z_imu_27 = readmatrix('0620_thrust_27.csv', 'Range', 'AA3:AA1907');
pitch_imu_27 = readmatrix('0620_thrust_27.csv', 'Range', 'C3:C1907');

%% 2g - 114.13g

% PSD analysis
acc_z_2 = acc_z_imu_2(inter_freq(1, 1):inter_freq(2, 1)-1);
N = length(acc_z_2);
zdft_2 = fft(acc_z_2);
zdft_2 = zdft_2(1:N/2+1);
freq_2 = 0:Fs/N:Fs/2;
z_psdx_2 = (1/(Fs*N)) * abs(zdft_2).^2;
z_psdx_2(2:end-1) = 2*z_psdx_2(2:end-1);

pitch_2 = pitch_imu_2(inter_freq(1, 1):inter_freq(2, 1)-1);
N = length(pitch_2);
xdft_2 = fft(pitch_2);
xdft_2 = xdft_2(1:N/2+1);
x_psdx_2 = (1/(Fs*N)) * abs(xdft_2).^2;
x_psdx_2(2:end-1) = 2*x_psdx_2(2:end-1);

figure(1);
sgtitle('2g weights - 114.13g')

subplot(2,3,1);
hold on;
grid on;
plot(pwm_left_2);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,3,2);
hold on;
grid on;
plot(pwm_right_2);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,3,3);
hold on;
grid on;
plot(freq_2,z_psdx_2);
xlabel('Z axis Frequency (Hz) 2g weights');
ylabel('Power/Frequency (dB/Hz)');

subplot(2,3,4);
hold on;
grid on;
plot(freq_2,x_psdx_2);
xlabel('pitch angle Frequency (Hz) 2g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 4g - 116.18g

% PSD analysis
acc_z_4 = acc_z_imu_4(inter_freq(1, 2):inter_freq(2, 2)-1);
N = length(acc_z_4);
zdft_4 = fft(acc_z_4);
zdft_4 = zdft_4(1:N/2+1);
freq_4 = 0:Fs/N:Fs/2;
psdx_4 = (1/(Fs*N)) * abs(zdft_4).^2;
psdx_4(2:end-1) = 2*psdx_4(2:end-1);

figure(2);
sgtitle('4g weights - 115.93g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_4);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_4);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,3);
hold on;
grid on;
plot(freq_4,10*log10(psdx_4));
xlabel('Frequency (Hz) 4g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 6g - 117.93g

% PSD analysis
acc_z_6 = acc_z_imu_6(inter_freq(1, 3):inter_freq(2, 3)-1);
N = length(acc_z_6);
zdft_6 = fft(acc_z_6);
zdft_6 = zdft_6(1:N/2+1);
freq_6 = 0:Fs/N:Fs/2;
psdx_6 = (1/(Fs*N)) * abs(zdft_6).^2;
psdx_6(2:end-1) = 2*psdx_6(2:end-1);

figure(3);
sgtitle('6g weights - 117.93g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_6);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_6);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,3);
hold on;
grid on;
plot(freq_6,10*log10(psdx_6));
xlabel('Frequency (Hz) 6g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 8g - 120.06g

figure(4);
sgtitle('8g weights - 120.06g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_8);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_8);
xlabel('time 0.01s');
ylabel('PWM signal');

% PSD analysis
acc_z_8 = acc_z_imu_8(inter_freq(1, 4):inter_freq(2, 4)-1);
N = length(acc_z_8);
zdft_8 = fft(acc_z_8);
zdft_8 = zdft_8(1:N/2+1);
freq_8 = 0:Fs/N:Fs/2;
psdx_8 = (1/(Fs*N)) * abs(zdft_8).^2;
psdx_8(2:end-1) = 2*psdx_8(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_8,10*log10(psdx_8));
xlabel('Frequency (Hz) 8g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 10g - 122.01g

figure(5);
sgtitle('10g weights - 122.01g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_10);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_10);
xlabel('time 0.01s');
ylabel('PWM signal');

% PSD analysis
acc_z_10 = acc_z_imu_10(inter_freq(1, 5):inter_freq(2, 5)-1);
N = length(acc_z_10);
zdft_10 = fft(acc_z_10);
zdft_10 = zdft_10(1:N/2+1);
freq_10 = 0:Fs/N:Fs/2;
psdx_10 = (1/(Fs*N)) * abs(zdft_10).^2;
psdx_10(2:end-1) = 2*psdx_10(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_10,10*log10(psdx_10));
xlabel('Frequency (Hz) 10g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 12g - 124.22g

figure(6);
sgtitle('12g weights - 124.22g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_12);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_12);
xlabel('time 0.01s');
ylabel('PWM signal');

% PSD analysis
acc_z_12 = acc_z_imu_12(inter_freq(1, 6):inter_freq(2, 6)-1);
N = length(acc_z_12);
zdft_12 = fft(acc_z_12);
zdft_12 = zdft_12(1:N/2+1);
freq_12 = 0:Fs/N:Fs/2;
psdx_12 = (1/(Fs*N)) * abs(zdft_12).^2;
psdx_12(2:end-1) = 2*psdx_12(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_10,10*log10(psdx_10));
xlabel('Frequency (Hz) 12g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 15g - 127.22g

figure(7);
sgtitle('15g weights - 127.22g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_15);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_15);
xlabel('time 0.01s');
ylabel('PWM signal');

% PSD analysis
acc_z_15 = acc_z_imu_15(inter_freq(1, 7):inter_freq(2, 7)-1);
N = length(acc_z_15);
zdft_15 = fft(acc_z_15);
zdft_15 = zdft_15(1:N/2+1);
freq_15 = 0:Fs/N:Fs/2;
psdx_15 = (1/(Fs*N)) * abs(zdft_15).^2;
psdx_15(2:end-1) = 2*psdx_15(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_15,10*log10(psdx_15));
xlabel('Frequency (Hz) 15g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 17g - 129.26g

figure(8);
sgtitle('17g weights - 129.26g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_17);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_17);
xlabel('time 0.01s');
ylabel('PWM signal');

% PSD analysis
acc_z_17 = acc_z_imu_17(inter_freq(1, 8):inter_freq(2, 8)-1);
N = length(acc_z_17);
zdft_17 = fft(acc_z_17);
zdft_17 = zdft_17(1:N/2+1);
freq_17 = 0:Fs/N:Fs/2;
psdx_17 = (1/(Fs*N)) * abs(zdft_17).^2;
psdx_17(2:end-1) = 2*psdx_17(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_17,10*log10(psdx_17));
xlabel('Frequency (Hz) 17g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 20g - 131.96g

figure(9);
sgtitle('20g weights - 131.96g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_20);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_20);
xlabel('time 0.01s');
ylabel('PWM signal');


% PSD analysis
acc_z_20 = acc_z_imu_20(inter_freq(1, 9):inter_freq(2, 9)-1);
N = length(acc_z_20);
zdft_20 = fft(acc_z_20);
zdft_20 = zdft_20(1:N/2+1);
freq_20 = 0:Fs/N:Fs/2;
z_psdx_20 = (1/(Fs*N)) * abs(zdft_20).^2;
z_psdx_20(2:end-1) = 2*z_psdx_20(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_20,10*log10(z_psdx_20));
xlabel('Z axis Frequency (Hz) 20g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 22g - 134.00g

figure(10);
title('22g weights - 134.00g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_22);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_22);
xlabel('time 0.01s');
ylabel('PWM signal');


% PSD analysis
acc_z_22 = acc_z_imu_22(inter_freq(1, 10):inter_freq(2, 10)-1);
N = length(acc_z_22);
zdft_22 = fft(acc_z_22);
zdft_22 = zdft_22(1:N/2+1);
freq_22 = 0:Fs/N:Fs/2;
z_psdx_22 = (1/(Fs*N)) * abs(zdft_22).^2;
z_psdx_22(2:end-1) = 2*z_psdx_22(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_22,10*log10(z_psdx_22));
xlabel('Z axis Frequency (Hz) 22g weights');
ylabel('Power/Frequency (dB/Hz)');


%% 24g - 136.04g

figure(11);
title('24g weights - 136.04g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_24);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_24);
xlabel('time 0.01s');
ylabel('PWM signal');


% PSD analysis
acc_z_24 = acc_z_imu_24(inter_freq(1, 11):inter_freq(2, 11)-1);
N = length(acc_z_24);
zdft_24 = fft(acc_z_24);
zdft_24 = zdft_24(1:N/2+1);
freq_24 = 0:Fs/N:Fs/2;
z_psdx_24 = (1/(Fs*N)) * abs(zdft_24).^2;
z_psdx_24(2:end-1) = 2*z_psdx_24(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_24,10*log10(z_psdx_24));
xlabel('Z axis Frequency (Hz) 24g weights');
ylabel('Power/Frequency (dB/Hz)');

%% 27g - 138.94g

figure(12);
title('27g weights - 138.94g')

subplot(2,2,1);
hold on;
grid on;
plot(pwm_left_27);
xlabel('time 0.01s');
ylabel('PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(pwm_right_27);
xlabel('time 0.01s');
ylabel('PWM signal');


% PSD analysis
acc_z_27 = acc_z_imu_27(inter_freq(1, 12):inter_freq(2, 12)-1);
N = length(acc_z_27);
zdft_27 = fft(acc_z_27);
zdft_27 = zdft_27(1:N/2+1);
freq_27 = 0:Fs/N:Fs/2;
z_psdx_27 = (1/(Fs*N)) * abs(zdft_27).^2;
z_psdx_27(2:end-1) = 2*z_psdx_27(2:end-1);

pitch_27 = pitch_imu_27(inter_freq(1, 12):inter_freq(2, 12)-1);
N = length(acc_z_27);
xdft_27 = fft(pitch_27);
xdft_27 = xdft_27(1:N/2+1);
freq_27 = 0:Fs/N:Fs/2;
x_psdx_27 = (1/(Fs*N)) * abs(xdft_27).^2;
x_psdx_27(2:end-1) = 2*x_psdx_27(2:end-1);

subplot(2,2,3);
hold on;
grid on;
plot(freq_27,z_psdx_27);
xlabel('Z axis Frequency (Hz) 27g weights');
ylabel('Power/Frequency (dB/Hz)');

subplot(2,2,4);
hold on;
grid on;
plot(freq_27,x_psdx_27);
xlabel('pitch angle Frequency (Hz) 27g weights');
ylabel('Power/Frequency (dB/Hz)');

%% calculate average PWM

left_stable  = zeros(1, length(weight));
right_stable = zeros(1, length(weight));
aver_stable  = zeros(1, length(weight));

left_stable(1) = mean(pwm_left_2(inter_aver(1, 1):inter_aver(2, 1)));
left_stable(2) = mean(pwm_left_4(inter_aver(1, 2):inter_aver(2, 2)));
left_stable(3) = mean(pwm_left_6(inter_aver(1, 3):inter_aver(2, 3)));
left_stable(4) = mean(pwm_left_8(inter_aver(1, 4):inter_aver(2, 4)));
left_stable(5) = mean(pwm_left_10(inter_aver(1, 5):inter_aver(2, 5)));
left_stable(6) = mean(pwm_left_12(inter_aver(1, 6):inter_aver(2, 6)));
left_stable(7) = mean(pwm_left_15(inter_aver(1, 7):inter_aver(2, 7)));
left_stable(8) = mean(pwm_left_17(inter_aver(1, 8):inter_aver(2, 8)));
left_stable(9) = mean(pwm_left_20(inter_aver(1, 9):inter_aver(2, 9)));
left_stable(10) = mean(pwm_left_22(inter_aver(1, 10):inter_aver(2, 10)));
left_stable(11) = mean(pwm_left_24(inter_aver(1, 11):inter_aver(2, 11)));
left_stable(12) = mean(pwm_left_27(inter_aver(1, 12):inter_aver(2, 12)));

right_stable(1) = mean(pwm_right_2(inter_aver(1, 1):inter_aver(2, 1)));
right_stable(2) = mean(pwm_right_4(inter_aver(1, 2):inter_aver(2, 2)));
right_stable(3) = mean(pwm_right_6(inter_aver(1, 3):inter_aver(2, 3)));
right_stable(4) = mean(pwm_right_8(inter_aver(1, 4):inter_aver(2, 4)));
right_stable(5) = mean(pwm_right_10(inter_aver(1, 5):inter_aver(2, 5)));
right_stable(6) = mean(pwm_right_12(inter_aver(1, 6):inter_aver(2, 6)));
right_stable(7) = mean(pwm_right_15(inter_aver(1, 7):inter_aver(2, 7)));
right_stable(8) = mean(pwm_right_17(inter_aver(1, 8):inter_aver(2, 8)));
right_stable(9) = mean(pwm_right_20(inter_aver(1, 9):inter_aver(2, 9)));
right_stable(10) = mean(pwm_right_22(inter_aver(1, 10):inter_aver(2, 10)));
right_stable(11) = mean(pwm_right_24(inter_aver(1, 11):inter_aver(2, 11)));
right_stable(12) = mean(pwm_right_27(inter_aver(1, 12):inter_aver(2, 12)));

for i = 1:length(aver_stable)
    aver_stable(i) = 0.5*(left_stable(i) + right_stable(i));
end

%% thrust model

pt_id = [1, 2, 3, 4, 6, 7, 9, 11, 12];
wg_id = weight(pt_id);
A_wg = zeros(length(wg_id), 2);
for i = 1:length(wg_id)
    A_wg(i, 1) = 1;
    A_wg(i, 2) = wg_id(i);
end

theta_wtp = pinv(A_wg'*A_wg)*A_wg'*aver_stable(pt_id)';
pwm_iden = theta_wtp(1) + theta_wtp(2)*weight;
wg_iden = 1/theta_wtp(2)*(aver_stable - theta_wtp(1));

figure(13);
sgtitle('PWM signal average')

subplot(2,2,1);
hold on;
grid on;
plot(weight, left_stable);
xlabel('time 0.01s');
ylabel('left motor PWM signal');

subplot(2,2,2);
hold on;
grid on;
plot(weight, right_stable);
xlabel('time 0.01s');
ylabel('right motor PWM signal');

subplot(2,2,3);
hold on;
grid on;
plot(weight, aver_stable);
plot(weight, pwm_iden);
legend('true', 'id OLS');
xlabel('weights [g]');
ylabel('average motor PWM signal');

figure(14);
subplot(1,2,2);
hold on;
grid on;
plot(aver_stable([1:4, 5:7, 9, 11:end]), weight([1:4, 5:7, 9, 11:end])/1000*9.8, 'b', 'Marker', '.', 'MarkerSize', 15);
plot(aver_stable, wg_iden/1000*9.8, 'r');
legend('Measured thrust force', 'Identified linear model');
xlabel('Motor PWM signal');
ylabel('Thrust [N]');
title('B', 'Position', [37000, 1.45], 'FontSize', 15);

subplot(1,2,1);
hold on;
grid on;
t = errorbar(windspeed, drag_est, drag_err_neg, drag_err_pos, '-', 'Marker', '.', 'MarkerSize', 15);
t.Color = 'b';
plot(windspeed, drag_lin, 'r');
legend('Drag estimated from pitch angles', 'Identified model');
xlabel('Continuous wind speed [m/s]');
ylabel('Wind drag force [N]');
title('A', 'Position', [-0.1, 0.8], 'FontSize', 15);
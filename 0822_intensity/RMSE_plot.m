clc
close all
clear

%% initialize array

RMSE_x = [0.4659 0.3435; 0.3264 0.3184; 0.2208, 0.1947; 0.1914 0.1235];
RMSE_pitch = [7.8265 8.8880; 5.7825 5.5373; 2.1279, 1.6016; 1.1177 3.1116];
aver_x = [0.1101 0.0377; 0.0275 0.0292; 0.0356 0.1284; 0.1115 0.1220];
current = [1.2766 1.2569; 1.2702 1.2591; 1.2559 1.3271; 1.2091 1.2797];

figure(1);

subplot(1,2,1);
hold on;
grid on;

c = bar(RMSE_x);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz', '0.7500Hz'});
legend('Cascaded PID', 'Adaptive-FF');
set(gca, 'FontSize', 10);
ylabel('RMSE_{x} [m]')

subplot(1,2,2);
hold on;
grid on;

c = bar(RMSE_pitch);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz', '0.7500Hz'});
legend('Cascaded PID', 'Adaptive-FF');
set(gca, 'FontSize', 10);
ylabel('RMSE_{\theta} [deg]')


figure(2);

subplot(1,2,1);
hold on;
grid on;

c = bar(aver_x);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz', '0.7500Hz'});
legend('Cascaded PID', 'Adaptive-FF');
set(gca, 'FontSize', 10);
ylabel('Absolute value of X_{error} average [m]')

subplot(1,2,2);
hold on;
grid on;

c = bar(current);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz', '0.7500Hz'});
legend('Cascaded PID', 'Adaptive-FF');
set(gca, 'FontSize', 10);
ylabel('Average current [A]')

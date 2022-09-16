clc
close all
clear

%% initialize array

RMSE_x = [0.4044 0.3560; 0.3445 0.2761; 0.2091, 0.1705; 0.1697 0.1008];
RMSE_pitch = [9.9005 9.2566; 7.9331 7.1083; 4.9303, 5.9009; 5.0958 5.0009];
aver_x = [0.1040 0.0377; 0.1260 0.0292; 0.0654 0.1284; 0.0478 0.1220];
current = [1.2771 1.3137; 1.3148 1.2864; 1.2566 1.2631; 1.2498 1.2833];
RMSE_z = [0.0678 0.0671; 0.0676 0.0483; 0.0329 0.03332; 0.0368 0.0321];

figure(1);

subplot(2,2,1);
hold on;
grid on;

c = bar(RMSE_x);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.25Hz', '0.33Hz', '0.50Hz', '0.75Hz'});
legend('Original PID', 'Adaptive PID');
set(gca, 'FontSize', 10);
ylabel('RMSE_{x} [m]')

subplot(2,2,2);
hold on;
grid on;

c = bar(RMSE_z);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.25Hz', '0.33Hz', '0.50Hz', '0.75Hz'});
legend('Original PID', 'Adaptive PID');
set(gca, 'FontSize', 10);
ylabel('RMSE_{z} [m]')

subplot(2,2,3);
hold on;
grid on;

c = bar(RMSE_pitch);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.25Hz', '0.33Hz', '0.50Hz', '0.75Hz'});
legend('Original PID', 'Adaptive PID');
set(gca, 'FontSize', 10);
ylabel('RMSE_{\theta} [deg]')

subplot(2,2,4);
hold on;
grid on;

c = bar(current);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:4, 'XTickLabel', {'0.25Hz', '0.33Hz', '0.50Hz', '0.75Hz'});
legend('Original PID', 'Adaptive PID');
set(gca, 'FontSize', 10);
ylabel('Average current [A]')

clc
close all
clear

%% initialize array

RMSE_x = [0.4565 0.3435; 0.3499 0.3184; 0.2659, 0.1947; 0.2451 0.1235];
RMSE_pitch = [9.5244 8.8880; 7.3712 5.5373; 5.6884, 1.6880; 5.1634 3.1116];
aver_x = [0.1040 0.0377; 0.1260 0.0292; 0.0654 0.1284; 0.0478 0.1220];
current = [1.2835 1.2569; 1.2900 1.2591; 1.2566 1.3271; 1.2498 1.2797];
RMSE_z = [0.0798 0.0833; 0.0516 0.0620; 0.0475 0.0313; 0.0447 0.0416];

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

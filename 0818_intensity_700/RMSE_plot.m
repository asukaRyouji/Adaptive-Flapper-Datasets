clc
close all
clear

%% initialize array

RMSE_x = [0.4431 0.3702; 0.3148 0.2970; 0.2067, 0.2119];
RMSE_pitch = [3.2209 2.8514; 2.8834 2.2398; 2.6523, 2.7027];
aver_x = [0.0836 0.0664; 0.1177 0.0647; 0.1099 0.0581];
current = [1.2690 1.2651; 1.2487 1.2890; 1.2643 1.2501];

figure(1);

subplot(1,2,1);
hold on;
grid on;

c = bar(RMSE_x);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:3, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz'});
legend('Cascaded PID', 'Adaptive-FF');
set(gca, 'FontSize', 10);
ylabel('RMSE_{x} [m]')

subplot(1,2,2);
hold on;
grid on;

c = bar(RMSE_pitch);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:3, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz'});
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
set(gca, 'XTick', 1:3, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz'});
legend('Cascaded PID', 'Adaptive-FF');
set(gca, 'FontSize', 10);
ylabel('Absolute value of X_{error} average [m]')

subplot(1,2,2);
hold on;
grid on;

c = bar(current);
c(1).FaceColor = 'k';
c(2).FaceColor = [0.3010 0.7450 0.9330];
set(gca, 'XTick', 1:3, 'XTickLabel', {'0.2500Hz', '0.3333Hz', '0.5000Hz'});
legend('Cascaded PID', 'Adaptive-FF');
set(gca, 'FontSize', 10);
ylabel('Average current [A]')

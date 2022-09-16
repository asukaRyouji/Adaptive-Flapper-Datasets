close all;

window_movAvg = 50;
%% pwm = 200

currentCase = hover_200;
inte_pitch = [1413, 1767];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);

dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(1);
sgtitle('PWM = 200');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_200 = currentCase;

%% pwm = 300

currentCase = hover_300;
inte_pitch = [1556, 1902];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);
dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(2);
sgtitle('PWM = 300');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_300 = currentCase;

%% pwm = 400

currentCase = hover_400;
inte_pitch = [1312, 1553];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);
dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(3);
sgtitle('PWM = 400');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_400 = currentCase;

%% pwm = 500

currentCase = hover_500;
% inte_pitch = [364, 500];
inte_pitch = [364, 508];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);
dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(4);
sgtitle('PWM = 500');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_500 = currentCase;

%% pwm = 600

currentCase = hover_600;
% inte_pitch = [1340, 1540];
inte_pitch = [1511, 1650];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);
dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(5);
sgtitle('PWM = 600');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_600 = currentCase;

%% pwm = 700

currentCase = hover_700;
inte_pitch = [1062, 1300];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);
dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(6);
sgtitle('PWM = 700');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_700 = currentCase;

%% pwm = 800

currentCase = hover_800;
inte_pitch = [785, 953];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);
dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(7);
sgtitle('PWM = 800');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_800 = currentCase;

%% pwm = 000

currentCase = hover_000;
% inte_pitch = [2249, 2589];
% inte_pitch = [665, 811];
inte_pitch = [580, 811];

dihedral_comd = currentCase.Comm(inte_pitch(1):inte_pitch(2));
dihedral_comd = movAver(dihedral_comd, window_movAvg);
dihedral_act = currentCase.Dihedral(inte_pitch(1):inte_pitch(2));
dihedral_act = movAver(dihedral_act, window_movAvg);

airspeed = currentCase.airVeloFilt(inte_pitch(1):inte_pitch(2));

% ratio = mean(movAver(dihedral_act./dihedral_comd, 50));

currentCase.poly_k = polyfit(dihedral_comd, dihedral_act, 1);

dihedral_k = polyval(currentCase.poly_k, dihedral_comd);
error = dihedral_act - (dihedral_k - currentCase.poly_k(2));
currentCase.poly_e = polyfit(airspeed, error, 5);
error_u = polyval(currentCase.poly_e, airspeed);
currentCase.error_avg = mean(error);

figure(8);
sgtitle('PWM = 0');

subplot(2,2,1);
hold on;
grid on;
plot(dihedral_act);
plot(dihedral_comd);
plot(dihedral_k+error_u - currentCase.poly_k(2));
legend('true', 'comd', 'Kair');
xlabel('time [ms]');
ylabel('dihedral angle [deg]');

subplot(2,2,2);
hold on;
grid on;
plot(error);
plot(error_u);
legend('true', 'Ccorr(u)');
xlabel('time [ms]');
ylabel('error dihedral angle [deg]');

hover_000 = currentCase;
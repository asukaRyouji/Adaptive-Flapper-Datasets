clc
close all;
clear;


percent_in = [46,      48,      50,      52,       54,       56,       58,       60,       62,       64,       66,       68];
pwm_in     = [30146,   31456,   32767,   34078,    35388,    36699,    38010,    39321,    40631,    41942,    43253,    44563];
pot_in     = [1.4815,  1.5282,  1.5591,  1.5956,   1.6274,   1.6595,   1.6919,   1.7208,   1.7461,   1.7821,   1.8033,   1.8397];
roll_in    = [-1.6504, -0.5632, -1.7529, 0.0880,   -1.7398,  -4.2341,  -1.5798,  -1.7930,  -1.9119,  -0.6913,  -0.4731,  -1.0478];
pitch_in   = [-0.2744, -0.7172, 0.9324,  -1.2402,  1.2270,   4.8911,   1.2399,   1.4078,   1.5648,   -0.4243,  -0.5808,  -0.0133];
yaw_in     = [-3.9665, -6.1535, -8.0288, -10.3491, -13.0695, -14.3412, -16.3134, -18.5534, -19.9125, -23.4350, -24.3392, -26.5126];

roll_in_neu = zeros(1, length(roll_in));
pitch_in_neu = zeros(1, length(roll_in));
yaw_in_neu = zeros(1, length(roll_in));
angle_in_neu = zeros(1, length(roll_in));

for i = 1:length(roll_in_neu)
    roll_in_neu(i) = roll_in(i) - roll_in(1);
    pitch_in_neu(i) = pitch_in(i) - pitch_in(1);
    yaw_in_neu(i) = yaw_in(i) - yaw_in(1);
    angle_in_neu(i) = sqrt(roll_in_neu(i)^2 + pitch_in_neu(i)^2+ yaw_in_neu(i)^2);
end

percent_de = [46,      44,      42,      40,       38,       36,       34,       32,       30,       28,       26,       24];
pwm_de     = [30146,   28835,   27524    26214,    24903,    23592,    22281,    20971,    19660,    18349,    17039,    15728];
pot_de     = [1.4815,  1.4518,  1.4193,  1.3911,   1.3586,   1.3247,   1.2916,   1.2593,   1.2299,   1.1919,   1.1653,   1.1303];
roll_de    = [-1.6504, -2.2880, -1.5644, 1.7318,   -2.8687,  -2.8579,  -2.1354,  -1.8100,  2.8963,   -1.2494,  -1.1954,  -1.0611];
pitch_de   = [-0.2744, 0.9427,  3.5143,  0.0526,   -2.3174,  1.4097,   -7.9638,  -5.5362,  -7.4072,  -1.1588,  -1.5026,  -1.5679];
yaw_de     = [-3.9665, -1.2852, -0.9679, 3.3451,   5.6583,   8.5875,   10.7593,  12.3463,  15.1685,  17.5971,  20.3698,  21.7486];

roll_de_neu = zeros(1, length(roll_de));
pitch_de_neu = zeros(1, length(roll_de));
yaw_de_neu = zeros(1, length(roll_de));
angle_de_neu = zeros(1, length(roll_de));

for i = 1:length(roll_de_neu)
    roll_de_neu(i) = roll_de(i) - roll_de(1);
    pitch_de_neu(i) = pitch_de(i) - pitch_de(1);
    yaw_de_neu(i) = yaw_de(i) - yaw_de(1);
    angle_de_neu(i) = sqrt(roll_de_neu(i)^2 + pitch_de_neu(i)^2+ yaw_de_neu(i)^2);
end

percent = [percent_de(end:-1:2), percent_in];
pwm = [pwm_de(end:-1:2), pwm_in];
pot = [pot_de(end:-1:2), pot_in];
angle_neu = [angle_de_neu(end:-1:2), -angle_in_neu];

%% dihedral angle model

% from PWM to percent
poly_com = polyfit(pwm, percent, 1);
per_act = polyval(poly_com, pwm);

% actual angle value model from POT to actual dihedral angle
poly_act = polyfit(pot, angle_neu, 1);
angle_act = polyval(poly_act, pot);
diff_angle_act = angle_neu - angle_act;

% actual angle value model from pwm to desired dehidral angle
poly_des = polyfit(pwm, angle_neu, 1);
angle_des = polyval(poly_des, pwm);
diff_angle_des = angle_neu - angle_des;

%% plot

figure(1);

subplot(2,2,1);
hold on;
grid on;
plot(percent, pot);
xlabel("percent");
ylabel("POT value");

subplot(2,2,2);
hold on;
grid on;
plot(percent, pwm);
plot(per_act, pwm);
legend('true','id from polyfit');
xlabel("percent");
ylabel("PWM value");

subplot(2,2,3);
hold on;
grid on;
plot(pot, angle_neu);
plot(pot, angle_act);
legend('true', 'id from polyfit');
xlabel("POT reading");
ylabel("actual dihedral angle value");

subplot(2,2,4);
hold on;
grid on;
plot(percent, angle_neu);
plot(percent, angle_act);
legend('true', 'id from polyfit');
xlabel("percent");
ylabel("desired dihedral angle value");

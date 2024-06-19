% This is the initialization script for the motor and axle parameters. Both
% motors are identical PMSM 1FT7042-5AF70-1DA0 HD.

clear all;
clc;

%% Inputs to the switches in the Simulink model

% theta_true = 1 when running P-PI controller (otherwise 0)
theta_true = 1;

% theta_step_true = 1 when running step input for theta_r (0 for sine
% input)
theta_step_true = 0;    % doesn't matter when theta_true = 0

% omega_step_true = 1 when running step input for omega_r (0 for sine
% input)
omega_step_true = 0;

%% Motor electrical parameters
% Motor mechanical parameters
J_m = 2.81e-4 + 5.5e-4; % kgm^2 -- Moment of inertia
N = 1;                  % -- Gear ratio

% Saturation
u_max = 13;             % Nm -- Maximum torque

% Common simulation parameters
T_s = 0.125/1000; 		% Sampling time for control loops. For data acquisition, it is 0.125 ms
T_log = T_s; % har bare valgt en værdi der er større end T_s
% we we use Ts instead of T_log it might give way too many samples

%% Values of friction and shaft parameters
% Taken from Table 4.3: Summary of calculated friction and shaft parameters
% (page 40, Dimitrios Papageorgiou phd thesis)

% Shaft constants
K_S = 32.94;    % N m rad^(-1)
D_S = 0.0548;   % N m s rad^(-1)

% Coulomb friction
% (assuming T_C is the average of T_C_m and T_C_l)
T_C = (0.0223 + 0.0232) / 2;    % N m

% Static friction
% (assuming T_S is the average of T_S_m and T_S_l)
T_S = (0.0441 + 0.0453) / 2;    % N m

% Friction constants
b_fr = 0.0016;  % N m s rad^(-1)

% Load inertia      (not sure...)
J_l = 0.000831; % kgm^2 -- Moment of inertia
inv_J_l = J_l;

% Initial conditions vector (should be zero)
x_0 = [0,0];
x_l_0 = [0,0];

xf = 1;

%% Open model for PI-tuning
%open("driveTrain_PID_controller_PI_tuning.slx");

%% Open model for P-PI tuning
%open("driveTrain_PID_controller.slx");

%% P-PI-controller parameters
% tau_i = 0.0373;
% k_vel = 0.492;
% k_i = 1/tau_i;
% k_pos = 5.5;

% tau_i = 0.06;
% k_pos = 9;
% k_i = 1/tau_i;
% k_vel = tau_i *k_i;

% test
tau_i = 1/1.0084;
k_pos = 5.0008;
k_vel = 9.3599;
%% Simulink simulation - STSMC and P-STSMC hand-tuning
driveTrain_sim = sim("driveTrain_PID_controller", 10);

%% Extracting data
omega_r_timeseries = driveTrain_sim.omega_r_out;
theta_r_timeseries = driveTrain_sim.theta_r_out;
omega_m_timeseries = driveTrain_sim.omega_m_out;
theta_l_timeseries = driveTrain_sim.theta_l_out;


% Extract data and time
time = omega_r_timeseries.Time;
omega_r = omega_r_timeseries.Data;
theta_r = theta_r_timeseries.Data;
omega_m = omega_m_timeseries.Data;
theta_l = theta_l_timeseries.Data;

%% Loss and RSME calculations (same as used for DiffTune)
% Quadratic loss function: (Yi-Yi_hat)^2
% MSE = 1/N sum_i^N((Yi-Yi_hat)^2)

e_theta = theta_r - theta_l;
loss_theta = e_theta .^ 2;
acc_loss_theta = sum(loss_theta);   % accumulated loss
rmse_theta = sqrt(1/length(time) * acc_loss_theta);

%% Plots 1
subplot(2,2,[1 2]);

% STSMC
% plot(driveTrain_sim.omega_m_out, 'LineWidth', 1.5);
% hold on;
% plot(driveTrain_sim.omega_r_out, '--', 'LineWidth', 1.5);
% hold off;
% grid on;
% legend('\omega_m', '\omega_r', 'Location', 'southeast');
% ylim([-1 1]);
% xlabel('time (s)');
% ylabel('velocity (rad/s)');
% text(0.5,-0.3,['k1 = ' sprintf('%.4f', k1)]);
% text(0.5,-0.5,['k2 = ' sprintf('%.4f', k2)]);
% text(0.5,-0.7,['rmse = ' sprintf('%.4f', rmse_theta)]);
% title('Hand-tuned STSMC sine response');

% P-STSMC
h1 = figure(1);
plot(driveTrain_sim.theta_l_out, 'LineWidth', 1.5);
hold on;
plot(driveTrain_sim.theta_r_out, '--', 'LineWidth', 1.5);
hold off;
grid on;
legend('\theta_l', '\theta_r', 'Location', 'southeast');
ylim([-1 1]);
xlabel('time (s)');
ylabel('position (rad)');
text(0.5,-0.20,['tau_i = ' sprintf('%.4f', tau_i)]);
text(0.5,-0.4,['k_vel = ' sprintf('%.4f', k_vel)]);
text(0.5,-0.60,['k_pos = ' sprintf('%.4f', k_pos)]);
text(0.5,-0.8,['rmse = ' sprintf('%.4f', rmse_theta)]);
title('DiffTune tuned P-PI sine response');

subplot(2,2,3);
plot(time, abs(e_theta)*10^3, 'LineWidth', 1.5);
grid on;
legend('|e_\theta|', 'Location', 'northeast');
ylim([-0.5 6]);
xlabel('time (s)');
ylabel('position error (mrad)');
title('Position error');

subplot(2,2,4);
plot(driveTrain_sim.u_out, 'LineWidth', 1.5);
grid on;
legend('u', 'Location', 'northeast');
ylim([-0.2 1]);
xlabel('time (s)');
ylabel('torque (N m)');
title('Torque command');

saveas(h1, 'Matlab plots\DiffTune tuned of P-PI.png');
%% Plots
h1 = figure(1);

if theta_true == 0
    if omega_step_true == 1
        plot(driveTrain_sim.omega_m_out, 'LineWidth', 1.5);
        hold on;
        plot(driveTrain_sim.omega_r_out, '--', 'LineWidth', 1.5);
        hold on;
        yline(1.06, ':k');
        %hold on;
        %yline(0.94, ':k');
        hold on;
        yline(1.02, '--k');
        hold on;
        yline(0.98, '--k');
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('ang. velocity (rad/s)');
        legend('\omega_r', '\omega_m', 'Location', 'southeast');
        title('Simulink simulation of step response');
        saveas(h1, 'Matlab plots\step response of PI hand-tuning.png');
    else
        plot(driveTrain_sim.omega_m_out, 'LineWidth', 1.5);
        hold on;
        plot(driveTrain_sim.omega_r_out, '--', 'LineWidth', 1.5);
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('ang. velocity (rad/s)');
        legend('\omega_r', '\omega_m', 'Location', 'southeast');
        title('Simulink simulation of sine response');
        saveas(h1, 'Matlab plots\sine response of PI hand-tuning.png');
    end
    % text(0.5,-0.10,['k1 = ' num2str(k1)]);
    % text(0.5,-1.25,['k2 = ' num2str(k2)]);
else
    if theta_step_true == 1
        plot(driveTrain_sim.theta_r_out, 'LineWidth', 1.5);
        hold on;
        plot(driveTrain_sim.theta_l_out, 'LineWidth', 1.5);
        hold on;
        yline(1.06, ':k');
        hold on;
        yline(1.02, '--k');
        hold on;
        yline(0.98, '--k');
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('position (rad)');
        legend('\theta_r', '\theta_l', 'Location', 'southeast');
        title('Hand-tuned P-PI step response');
        text(0.5,-0.20,['tau_i = ' sprintf('%.4f', tau_i)]);
        text(0.5,-0.3,['k_vel = ' sprintf('%.4f', k_vel)]);
        text(0.5,-0.40,['k_pos = ' sprintf('%.4f', k_pos)]);
        % text(0.5,-1.10,['acc. loss = ' sprintf('%.4f', acc_loss_theta)]);
        text(0.5,-0.5,['rmse = ' sprintf('%.4f', rmse_theta)]);
        saveas(h1, 'Matlab plots\step response of P-PI hand-tuning.png');
    else
        % plot(driveTrain_sim.theta_r_out, '--', 'LineWidth', 1.5);
        % hold on;
        plot(driveTrain_sim.theta_l_out, 'LineWidth', 1.5);
        hold on;
        plot(driveTrain_sim.theta_r_out, '--', 'LineWidth', 1.5);
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('position (rad)');
        lgd = legend('\theta_l', '\theta_r', 'Location', 'southeast');
        set(lgd, 'FontSize', 11);
        text(0.5,-0.20,['tau_i = ' sprintf('%.4f', tau_i)]);
        text(0.5,-0.3,['k_vel = ' sprintf('%.4f', k_vel)]);
        text(0.5,-0.40,['k_pos = ' sprintf('%.4f', k_pos)]);
        % text(0.5,-1.10,['acc. loss = ' sprintf('%.4f', acc_loss_theta)]);
        text(0.5,-0.5,['rmse = ' sprintf('%.4f', rmse_theta)]);
        % title('Hand-tuned P-STSMC sine response');
        % saveas(h1, 'Matlab plots\sine response of P-STSMC hand-tuning.png');
        title('DiffTune tuned P-PI sine response')
        saveas(h1, 'Matlab plots\Hand-tuned P-PI sine response.png');
    end
end
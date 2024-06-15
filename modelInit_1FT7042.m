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
omega_step_true = 1;

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
k_i = 0.5154813414 * 0.5; 
k_vel =1.453488372 * 2.45 * 0.99;
tau_i = k_vel/k_i;
k_pos = 1.35;

%% Simulink simulation - STSMC and P-STSMC hand-tuning
driveTrain_sim = sim("driveTrain_PID_controller", 10);

h1 = figure(1);

if theta_true == 0
    if omega_step_true == 1
        plot(driveTrain_sim.omega_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.omega_m_out, 'LineWidth', 1);
        hold on;
        yline(1.06, ':k');
        hold on;
        yline(0.94, ':k');
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
        saveas(h1, 'Plots\step response of PI hand-tuning.png');
    else
        plot(driveTrain_sim.omega_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.omega_m_out, 'LineWidth', 1);
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('ang. velocity (rad/s)');
        legend('\omega_r', '\omega_m', 'Location', 'southeast');
        title('Simulink simulation of sine response');
        saveas(h1, 'Plots\sine response of PI hand-tuning.png');
    end
else
    if theta_step_true == 1
        plot(driveTrain_sim.theta_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.theta_l_out, 'LineWidth', 1);
        hold on;
        yline(1.02, '--k');
        hold on;
        yline(0.98, '--k');
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('position (rad)');
        legend('\theta_r', '\theta_l', 'Location', 'southeast');
        title('Simulink simulation of step response');
        saveas(h1, 'Plots\step response of P-PI hand-tuning.png');
    else
        plot(driveTrain_sim.theta_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.theta_l_out, 'LineWidth', 1);
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('position (rad)');
        legend('\theta_r', '\theta_l', 'Location', 'southeast');
        title('Simulink simulation of sine response');
        saveas(h1, 'Plots\sine response of P-PI hand-tuning.png');
        h2 = figure(2);   
        plot(driveTrain_sim.theta_r_out - driveTrain_sim.theta_l_out, 'LineWidth', 1);
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('error (rad)');
        legend('\theta_r', '\theta_l', 'Location', 'southeast');
        title('Error between \theta_r and \theta_l');
        saveas(h2, 'Plots\error sine response.png');
    end
end






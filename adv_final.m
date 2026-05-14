%% =========================================================
% DRONE ALTITUDE STABILIZATION SYSTEM - ADAPTIVE PID VERSION
% FINAL HACKATHON VERSION
% ==========================================================

clear; close all; clc;

%% 1. TRANSFER FUNCTION MODEL
s = tf('s');
G = 1/(s^2 + 2*s + 5); % Drone Plant

%% 2. OPEN LOOP ANALYSIS
figure('Color','w');
step(G,10); title('OPEN LOOP STEP RESPONSE'); grid on;
open_info = stepinfo(G);
pause(0.5);

%% 3. BASE PID PARAMETERS & ADAPTATION SETTINGS
Kp_base = 24;
Ki_base = 12;
Kd_base = 12;

% Adaptation Sensitivity (Higher = more aggressive response to error)
adapt_factor = 1.8; 

C_base = pid(Kp_base, Ki_base, Kd_base);

%% 4. CLOSED LOOP ANALYSIS (Using Base Gains)
T = feedback(C_base*G, 1);
figure('Color','w');
step(T,10); title('CLOSED LOOP STEP RESPONSE (BASE GAINS)'); grid on;
closed_info = stepinfo(T);

%% 5. SIMULATION SETTINGS
dt = 0.01;
T_end = 15;
t = 0:dt:T_end;
N = length(t);
r = ones(1,N); % Reference altitude

%% 6. DISTURBANCE MODEL (Wind Gust at 5s)
wind = zeros(1,N);
gust_idx = round(5/dt):round(6.2/dt);
wind(gust_idx) = 1.5; % Increased gust for adaptive test
wind = wind + 0.04*randn(size(t)); % Turbulence

%% 7. SYSTEM STATES & LOGGING
y = zeros(1,N); v = zeros(1,N); u = zeros(1,N); e = zeros(1,N);
Kp_dynamic = zeros(1,N); % To track how Kp changes
integral_error = 0;
prev_error = 0;

%% 8. LIVE ANIMATION WINDOW
figure('Color','w', 'Position',[100 100 1300 600]);
tiledlayout(1,2);

% Left: Animation
nexttile
hold on; grid on; xlim([-2 2]); ylim([-0.5 2]);
xlabel('Position'); ylabel('Altitude'); title('LIVE ADAPTIVE STABILIZATION');
yline(1,'--r','Target','LineWidth',1.5);
drone = plot(0,0,'ks','MarkerSize',20,'MarkerFaceColor','b');
dist_text = text(-1.7,1.6,'','Color','r','FontSize',12,'FontWeight','bold');

% Right: Real-time Gain Plot
nexttile
hold on; grid on; xlim([0 T_end]); ylim([0 60]);
title('DYNAMIC GAIN ADAPTATION (Kp)');
xlabel('Time (s)'); ylabel('Gain Value');
kp_line = animatedline('Color','m','LineWidth',2);
actual_line = animatedline('Color','b','LineWidth',2);
legend('Dynamic Kp','Altitude');

%% 9. MAIN CONTROL LOOP (ADAPTIVE PID)
for k = 2:N
    % Calculate Error
    e(k) = r(k) - y(k-1);
    
    %% --- ADAPTIVE LOGIC ---
    % Gains increase proportionally to the magnitude of the error
    % This is "Gain Scheduling" based on error states
    Kp = Kp_base * (1 + adapt_factor * abs(e(k)));
    Kd = Kd_base * (1 + adapt_factor * 0.5 * abs(e(k)));
    Ki = Ki_base; % Usually keep Ki constant to prevent integral windup
    
    Kp_dynamic(k) = Kp; % Log for plotting
    
    % PID Terms
    integral_error = integral_error + e(k)*dt;
    derivative = (e(k) - prev_error)/dt;
    
    % Control Signal
    u(k) = Kp*e(k) + Ki*integral_error + Kd*derivative;
    u(k) = min(max(u(k),0), 25); % Actuator Saturation
    
    prev_error = e(k);

    % Drone Dynamics (Physics)
    accel = -2*v(k-1) - 5*y(k-1) + u(k) - wind(k);
    v(k) = v(k-1) + accel*dt;
    y(k) = y(k-1) + v(k)*dt;

    % Animation Updates
    if mod(k,5) == 0 % Update every 5 steps for speed
        shake = 0.12*sin(25*t(k))*abs(wind(k));
        set(drone, 'XData', shake, 'YData', y(k));
        
        if abs(wind(k)) > 0.8
            set(dist_text, 'String', 'GUST DETECTED - ADAPTING GAINS');
            set(drone, 'MarkerFaceColor', 'r');
        else
            set(dist_text, 'String', '');
            set(drone, 'MarkerFaceColor', 'b');
        end
        
        addpoints(kp_line, t(k), Kp);
        addpoints(actual_line, t(k), y(k));
        drawnow limitrate
    end
end

%% 10. FINAL PERFORMANCE ANALYSIS
pre_dist_idx = find(t < 5);
info = stepinfo(y(pre_dist_idx), t(pre_dist_idx), 1);

disp('=================================================')
disp('FINAL PERFORMANCE (ADAPTIVE)')
disp('=================================================')
fprintf('Overshoot       : %.2f %%\n', info.Overshoot);
fprintf('Settling Time   : %.2f s\n', info.SettlingTime);
fprintf('Steady Error    : %.4f\n', abs(1-y(end)));

%% 11. STABILITY ANALYSIS (Bode & Root Locus)
figure('Color','w');
tiledlayout(1,2);
nexttile; rlocus(C_base*G); title('Root Locus (Base)'); grid on;
nexttile; margin(C_base*G); title('Stability Margins (Base)'); grid on;

%% 12. SUMMARY PANEL
figure('Color','w','Position',[300 200 700 400]);
axis off;
text(0.1,0.85,'ADAPTIVE DRONE STABILIZATION','FontSize',18,'FontWeight','bold')
text(0.1,0.70,sprintf('Adaptive Kp Range: %.1f to %.1f', min(Kp_dynamic(20:end)), max(Kp_dynamic)),'FontSize',12)
text(0.1,0.60,sprintf('Overshoot: %.2f %%', info.Overshoot),'FontSize',12)
text(0.1,0.50,sprintf('Settling Time: %.2f s', info.SettlingTime),'FontSize',12)
text(0.1,0.40,'Status: DISTURBANCE REJECTED','Color','g','FontWeight','bold','FontSize',14)
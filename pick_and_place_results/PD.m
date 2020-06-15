%%

% Initial conditions
q = [0, 0, 0, 0, 0, 0, 0];
dq = [0, 0, 0, 0, 0, 0, 0];
ddq = [0, 0, 0, 0, 0, 0, 0];

% Gain matrix
Kp = diag([10 10 10 10 10 10 10]);
Kv = diag([5 5 5 5 5 5 5]);

% Time step
deltat = 0.05;  % seclength(reshape)
result = q;
index = 2;
sim_lenght = 10;
 
% References of position, velocity and acceleration of the joints 
q_des = [pi/2, 0, pi/3, pi/4, 0, 0, 0 ];
dq_des = [0, 0, 0, 0, 0, 0, 0];


%%

for i = 0:deltat:sim_lenght
   
    % Error and derivate of the error   
    err = q_des - q;
    derr = dq_des - dq;
    
    % PD Controller
    tau = ( Kv*(derr') + Kp*(err') )';
       
    % Robot joint accelerations
    ddq_old = ddq;
    ddq = (robot.accel(q, dq, tau))';   % ddq = inv(M)*(tau - C*dq - F - G)
        
    % Tustin integration
    dq_old = dq;
    dq = dq + (ddq_old + ddq) * deltat / 2;
    q = q + (dq + dq_old) * deltat /2;
    
    % Store result for the final plot
    result(index,:) = q;
    index = index + 1;
    
end
%%

% 3D Plot
disp("PLOT")



% Plots for joint variables
t = 0:deltat:(sim_lenght+deltat);
ref = zeros(1,index-1);
refjoint4 = pi/4*ones(1, index-1);
figure;

subplot(2,3,1)
plot(t, result(:,1), '--', 'LineWidth', 3.5)
hold on
grid on;
plot(t, ref, 'LineWidth', 2)
title('joint 1 position');
legend('joint variable','reference');
axis([0 t(index-1) -1 1]);
xlabel('time [s]'); ylabel('angular position [rad]');

subplot(2,3,2)
plot(t, result(:,2), '-', 'LineWidth', 3.5)
hold on
grid on;
plot(t, ref, 'LineWidth', 2)
title('joint 2 position');
legend('joint variable','reference');
axis([0 t(index-1) -1 1]);
xlabel('time [s]'); ylabel('angular position [rad]');

subplot(2,3,3)
plot(t, result(:,3), '-', 'LineWidth', 3.5)
hold on
grid on;
plot(t, ref, 'LineWidth', 2)
title('joint 3 position');
legend('joint variable','reference');
axis([0 t(index-1) -1 1]);
xlabel('time [s]'); ylabel('angular position [rad]');

subplot(2,3,4)
plot(t, result(:,4), '-', 'LineWidth', 3.5)
hold on
grid on;
plot(t, refjoint4, 'LineWidth', 2)
title('joint 4 position');
legend('joint variable','reference');
axis([0 t(index-1) -1 1]);
xlabel('time [s]'); ylabel('angular position [rad]');

subplot(2,3,5)
plot(t, result(:,5), '-', 'LineWidth', 3.5)
hold on
grid on;
plot(t, ref, 'LineWidth', 2)
title('joint 5 position');
legend('joint variable','reference');
axis([0 t(index-1) -1 1]);
xlabel('time [s]'); ylabel('angular position [rad]');

subplot(2,3,6)
plot(t, result(:,6), '-', 'LineWidth', 3.5)
hold on
grid on;
plot(t, ref, 'LineWidth', 2)
title('joint 6 position');
legend('joint variable','reference');
axis([0 t(index-1) -1 1]);
xlabel('time [s]'); ylabel('angular position [rad]');

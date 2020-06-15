%% Clean ENV

clear all
close all
clc

%% Choose Trajectory

fprintf('Choose trajectory: \n');

fprintf('1: Circumference \n');

fprintf('2: Helix \n');

choiche = input(' ... ');

%% Setup
 
load('robot.mat')
load('robotmodel.mat')


grey = [0.5, 0.5, 0.5];
orange = [0.8, 0.6, 0];

t_in = 0; % [s]
t_fin = 10; % [s]
delta_t = 0.001; % [s]
timeSpan= 10;

t = t_in:delta_t:t_fin;

num_of_joints = 7;

Q = zeros(num_of_joints,length(t));
dQ = zeros(num_of_joints,length(t));
ddQ = zeros(num_of_joints,length(t));
TAU = zeros(num_of_joints,length(t));

%% Compute Trajectory

switch choiche
                    
    case 1 % Circonferenza
        
        q0 = [0 pi/3 0 pi/6 0 0 0];
        q_dot0 = [0 0 0 0 0 0 0];
        
        pos0 = PANDA.fkine(q0).t;

        radius = 0.1; % raggio dell'elica [m]
        center = pos0 - [radius;0;0];
        
%           Standard Trajectory
%         x = center(1) + radius * cos(t/t(end)*2*pi);
%         y = center(2) * ones(size(x));
%         z = center(3) + radius * sin(t/t(end)*2*pi);
%         theta = 0.1*sin(t/5*2*pi);
        
%         Fast Trajectory
        x = center(1) + radius * cos(t/t(end)*2*pi);
        y = center(2) * ones(size(x));
        z = center(3) + radius * sin(t/t(end)*2*pi);
        theta = 0.1*sin(t/3*2*pi);
        
        phi = zeros(size(x));
        psi = zeros(size(x));

        xi = [x; y; z; theta; phi; psi]; % twist
        
        
        q_des= generate_trajectory(xi,q0,PANDA)
        dq_des=gradient(q_des)*1000
        ddq_des=gradient(dq_des)*1000
        
        figure
        PANDA.plotopt = {'workspace',[-0.75,0.75,-0.75,0.75,0,1]};
        PANDA.plot(q0,'floorlevel',0,'linkcolor',orange,'jointcolor',grey)
        hold
        plot3(x,y,z,'k','Linewidth',1.5)
        
    case 2 % Traiettoria elicoidale
        
        q0 = [0 pi/3 0 pi/6 0 0 0];
        q_dot0 = [0 0 0 0 0 0 0];
        pos0 = PANDA.fkine(q0).t;
       
        shift = 0.1; % passo dell'elica [m] 
        radius = 0.1; % raggio dell'elica [m]
        num = 2; % numero di giri [#]
        center = pos0 - [radius;0;0];

        x = center(1) + radius * cos(t/t(end)*num*2*pi);
        y = center(2) + t/t(end)*num*shift;
        z = center(3) + radius * sin(t/t(end)*num*2*pi);
        theta = zeros(size(x));
        phi = zeros(size(x));
        psi = zeros(size(x));

        xi = [ x ;y; z; theta ;phi; psi]; % twist
        
        
        q_des= generate_trajectory(xi,q0,PANDA)
        dq_des=gradient(q_des)*1000
        ddq_des=gradient(dq_des)*1000

%         figure
%         PANDA.plot(q0)
%         hold
%         plot3(x',y',z','k','Linewidth',1.5, 'Red')


end

%% Visualize desired trajectory

figure

plot3(x,y,z,'r','Linewidth',1.5)

grid on
PANDA.plotopt = {'workspace',[-0.75,0.75,-0.75,0.75,0,1]};
hold on
for i=1:100:length(q_des)
    
    PANDA.plot(transpose(q_des(:,i)),'floorlevel',0,'fps',1000,'trail','-k','linkcolor',orange,'jointcolor',grey)

end

%% Plot Joint Trajectories


figure
for j=1:num_of_joints
    
    subplot(4,2,j);
    plot(t,q_des(j,1:length(t)))
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
end

%% Plot xyz trajectory
a=[x;y;z]
figure
for j=1:3
    
    subplot(1,3,j);
    plot(t,a(j,1:length(t)))
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
end

%% Trajectory Tracking: Computed Torque Method


% Gain circumference parameters matrix
Kp = 20*diag([3 3 3 3 5 3 30]);
Kv = 10*diag([1 1 1 1 70 2 1]);

% Good Helix parameters matrix
% Kp = 200*diag([3 3 3 3 5 3 5]);
% Kv = 25*diag([1 1 1 1 70 2 70]);

results_computed_torque = q0;
index = 1;
q=q0
dq=q_dot0
ddq=[0 0 0 0 0 0 0]
for i=1:length(t)

   % Error and derivate of the error   
    err = transpose(q_des(:,i)) - q;
    derr = transpose(dq_des(:,i)) - dq;
    
    %Get dynamic matrices
    F = get_FrictionTorque(dq);
    G = get_GravityVector(q);
    C= get_CoriolisVector(q,dq);
    M = get_MassMatrix(q);

    % Computed Torque Controller
    
    tau = ( M*(ddq_des(:,1) + Kv*(derr') + Kp*(err')) + C + G +F)';
      
    % Robot joint accelerations
    ddq_old = ddq;
    ddq = (pinv(M)*(tau - C'- G'-F')')';
        
    % Tustin integration
    dq_old = dq;
    dq = dq + (ddq_old + ddq) * delta_t / 2;
    q = q + (dq + dq_old) * delta_t /2;
    
    % Store result for the final plot
    results_computed_torque(index,:) = q;
    index = index + 1;

end


%% Plot computed torque results for trajectory tracking

figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10001),results_computed_torque(1:10001,j))
%     legend ()
    hold on
%     plot(t(1:10001),results_computed_torque(1:10001,j))
    plot (t,q_des(j,1:length(t)))
    legend ('Computed Torque','Desired angle')
    grid;
end
%%
q_des_error=(results_computed_torque-q_des(:,1:10001)')'
figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t,q_des_error(j,1:length(t)))
%     legend ()
%     hold on
%     plot (t,q_des_error_no_friction(j,1:length(t)))
    legend ('Computed torque Error with no friction model')
    grid;
end

%% Trajectory tracking: Backstepping control


% Good Circumference parameters
Kp = 1* diag([1 1 1 1 3 1 1]);

% Good Helix parameters
% Kp = diag([1 1 1 1 3 1 1]);


results_backstepping = q0;
index = 1;
q=q0
dq=q_dot0
ddq=[0 0 0 0 0 0 0]
for i=1:length(t)

   % Error and derivate of the error   
    err = transpose(q_des(:,i)) - q;
    derr = transpose(dq_des(:,i)) - dq;
    
    dqr = transpose(dq_des(:,i)) + err*(Kp);
    ddqr = transpose(ddq_des(:,i)) + derr*(Kp);
    s = derr + err*(Kp');
     
    %Get dynamic matrices
    F = get_FrictionTorque(dq);
    G = get_GravityVector(q);
    C = get_CoriolisMatrix(q,dq);
    M = get_MassMatrix(q);


    % Backstepping Controller
    tau = (M*(ddqr') + C*(dqr') + G + Kp*(s') + err')';      
    
    % Robot joint accelerations
    ddq_old = ddq;
    ddq = (pinv(M)*(tau - transpose(C*(dq'))- G')')';
        
    % Tustin integration
    dq_old = dq;
    dq = dq + (ddq_old + ddq) * delta_t / 2;
    q = q + (dq + dq_old) * delta_t /2;
    
    % Store result for the final plot
    results_backstepping(index,  :) = q;
    index = index + 1;

end

%% Plot computed torque results for backstepping control

figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t,results_backstepping(:,j))
    hold on
    plot (t,q_des(j,1:length(t)))
%     hold on
%     plot(t,results_computed_torque(:,j))
    grid;
    legend ('Backstepping Results','Desired angle')
end




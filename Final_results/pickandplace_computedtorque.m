clear all
close all
clc

t_in = 0; % [s]
t_fin = 10; % [s]
delta_t = 0.001; % [s]

num_of_joints = 7;

t = t_in:delta_t:t_fin;

Q = zeros(num_of_joints,length(t));
dQ = zeros(num_of_joints,length(t));
ddQ = zeros(num_of_joints,length(t));
TAU = zeros(num_of_joints,length(t));

% Initial conditions
q = [0, 0, 0, 0, 0, 0, 0];
dq = [0, 0, 0, 0, 0, 0, 0];
ddq = [0, 0, 0, 0, 0, 0, 0];
qi=[0,0,0,0,0,0,0];

% Gain matrix
Kp = 200*diag([3 3 3 3 5 3 30]);
Kv = 20*diag([1 1 1 1 70 2 1]);

% References of position, velocity and acceleration of the joints 
q_des = [pi/3, 0, pi/3, pi/3, pi/6, 0 , 0];
dq_des = [0, 0, 0, 0, 0, 0, 0];
ddq_des = [0, 0, 0, 0, 0, 0, 0];
qi_des=[0, 0, 0, 0, 0, 0, 0];
result = q;
index = 1;
err_old=q_des-q
err=q_des-q
ierr=[0 0 0 0 0 0 0]
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computed Torque               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:length(t)

   % Error and derivate of the error   
    err_old=err;
    err = q_des - q;
    derr = dq_des - dq;
    ierr=ierr+(err+err_old)*delta_t/2;

    %Get dynamic matrices
    F = get_FrictionTorque(dq);
    G = get_GravityVector(q);
    C = get_CoriolisVector(q,dq);
    M = get_MassMatrix(q);

    % Computed Torque Controller
    tau = ( rand*5/100*M*(ddq_des' + Kv*(derr') + Kp*(err')) + C + G )';

    
    % Robot joint accelerations
    ddq_old = ddq;
    ddq = (pinv(M)*(tau - C'- G'-F')')';
        
    % Tustin integration
    dq_old = dq;
    dq = dq + (ddq_old + ddq) * delta_t / 2;
    q_old=q;
    q = q + (dq + dq_old) * delta_t /2;
%     qi=qi+(q_old+q)*delta_t / 2;
    
    % Store result for the final plot
    result(index,:) = q;
    index = index + 1;

end

%%

ref_0 = zeros(1,index-1);
ref_1 = ones(1, index-1);
refjoint=[pi/3*ref_1;ref_0;pi/3*ref_1;pi/3*ref_1;pi/6*ref_1; ref_0;ref_0];


%%
figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:5000),result(1:5000,j))
    hold on
    plot (t(1:5000),refjoint(j,1:5000))
    hold on
    plot (t(1:5000),results_with_correct_mass(1:5000,j))
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    legend( 'Noise added','Reference', 'Correct Mass')
    grid;
end





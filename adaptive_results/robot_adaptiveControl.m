% ROBOT CONTROL

clear
clc
close all

addpath('rvctools')
startup_rvc

load('robot') % real robot (KUKA)

load('robotmodel') % model of the robot (KUKAmodel)

n = KUKA.n; 


timeSpan = 10; 
timeStep = 0.001; 
t = (0:timeStep:timeSpan)'; 


%% Traiettoria da seguire nello spazio
fprintf('Selezionare traiettoria da eseguire: \n');
fprintf('     #1: pick and place \n');
fprintf('     #2: circonferenza \n');
fprintf('     #3: elica \n');
sel1 = input(' ... ');

switch sel1
    case 1 % Pick and place
        q0 = [0 pi/2 -pi/2 0 0];
        q_dot0 = [0 0 0 0 0];

        qf = [pi/2 pi/4 -pi/3 pi/2 pi/2];

        qd = repmap(qf,length(t),1); 
        qd_dot = zeros(length(t),n);
        qd_ddot = zeros(length(t),n);
        
    case 2 % Circonferenza
        q0 = [0 pi/2 -pi/2 0 0];
        q_dot0 = [0 0 0 0 0];
        
        T0 = KUKA.fkine(q0);
        pos0 = T0(1:3,4);

        radius = 0.25; 
        center = pos0 - [radius;0;0];

        x = center(1) + radius * cos(t/t(end)*2*pi);
        y = center(2) * ones(size(x));
        z = center(3) + radius * sin(t/t(end)*2*pi);
        theta = zeros(size(x));
        phi = zeros(size(x));
        psi = zeros(size(x));

        xi = [t x y z theta phi psi]; % twist
        
        figure
        KUKA.plot(q0)
        hold
        plot3(x,y,z,'k','Linewidth',1.5)

        sim('trajectory_generator')

    case 3 % Traiettoria elicoidale
        q0 = [pi/6 pi/2 -pi/2 0 0];
        q_dot0 = [0 0 0 0 0];
        
        T0 = KUKA.fkine(q0);
        pos0 = T0(1:3,4);

        shift = 0.15; % passo dell'elica 
        radius = 0.25; % raggio dell'elica
        num = 2; % numero di giri
        center = pos0 - [radius;0;0];

        x = center(1) + radius * cos(t/t(end)*num*2*pi);
        y = center(2) + t/t(end)*num*shift;
        z = center(3) + radius * sin(t/t(end)*num*2*pi);
        theta = zeros(size(x));
        phi = zeros(size(x));
        psi = zeros(size(x));

        xi = [t x y z theta phi psi]; % twist

        figure
        KUKA.plot(q0)
        hold
        plot3(x,y,z,'k','Linewidth',1.5)

        sim('trajectory_generator')

end


%% Selezione del controllo da eseguire
fprintf('Selezionare il tipo di controllo: \n');
fprintf('     #2: adaptive computed torque \n');
fprintf('     #3: LiSlotine \n');
fprintf('     #4 backstepping \n');

sel2 = input(' ... ');


%% Perturbazione iniziale dei parametri
int = 0; % intensit� percentuale della perturbazione sui parametri
for j = 1:n 
    KUKAmodel.links(j).m = KUKAmodel.links(j).m .* (1+int/100*0.5); 
end


%% Simulazione
q = zeros(length(t),n); 
q_dot = zeros(length(t),n); 
tau = zeros(length(t),n); 
piArray = zeros(length(t),n*10); % vettore dei parametri dinamici 
q0 = [0 pi/2 -pi/2 0 0] + pi/6*[0.3 0.4 0.2 0.8 0] - pi/12; % partiamo in una posizione diversa da quella di inizio traiettoria
q(1,:) = q0; 
q_dot(1,:) = q_dot0; 

qr_dot = zeros(length(t),n); 
qr_ddot = zeros(length(t),n); 

pi0 = zeros(1,n*10); 
for j = 1:n
    pi0((j-1)*10+1:j*10) = [KUKAmodel.links(j).m KUKAmodel.links(j).m*KUKAmodel.links(j).r ...
        KUKAmodel.links(j).I(1,1) 0 0 KUKAmodel.links(j).I(2,2) 0 KUKAmodel.links(j).I(3,3)];
end
piArray(1,:) = pi0; 

Kp = 1*diag([200 200 200 20 10]);
Kv = 0.1*diag([200 200 200 10 1]); ]
Kd = 0.1*diag([200 200 200 20 1]);

% P e R fanno parte della candidata di Lyapunov, quindi devono essere definite positive
R = diag(repmat([1e1 repmat(1e3,1,3) 1e2 1e7 1e7 1e2 1e7 1e2],1,n)); 
P = 0.01*eye(10);
lambda = diag([200, 200, 200, 200, 200])*0.03;



tic
for i = 2:length(t)
    %% Interruzione della simulazione se q diverge
    if any(isnan(q(i-1,:)))
        fprintf('Simulazione interrotta! \n')
        return
    end
    
    
    %% Calcolo dell'errore: e, e_dot
    e = qd(i-1,:) - q(i-1,:); 
    e_dot = qd_dot(i-1,:) - q_dot(i-1,:); 
    s = (e_dot + e*lambda);
    
    qr_dot(i-1,:) = qd_dot(i-1,:) + e*lambda;
    if (i > 2)
        qr_ddot(i-1,:) = (qr_dot(i-1) - qr_dot(i-2)) / timeStep;
    end
    
    
    %% Calcolo della coppia (a partire dal modello)
   
    
    if sel2 == 2||sel2 == 3|| sel2 == 4 
        for j = 1:n 
            KUKAmodel.links(j).m = piArray(i-1,(j-1)*10+1); % elemento 1 di pi
        end
    end
    
    Mtilde = KUKAmodel.inertia(q(i-1,:)); 
    Ctilde = KUKAmodel.coriolis(q(i-1,:),q_dot(i-1,:)); 
    Gtilde = KUKAmodel.gravload(q(i-1,:)); 
    
switch sel2
    case 1
        tau(i,:) = qd_ddot(i-1,:)*Mtilde' + q_dot(i-1,:)*Ctilde' + Gtilde + e_dot*Kv' + e*Kp'; 
    case 2
        tau(i,:) = qd_ddot(i-1,:)*Mtilde' + q_dot(i-1,:)*Ctilde' + Gtilde + e_dot*Kv' + e*Kp';
    case 3
        tau(i,:) = qr_ddot(i-1,:)*Mtilde' + qr_dot(i-1,:)*Ctilde' + Gtilde + s*Kd'; 
    case 4
        tau(i,:) = qr_ddot(i-1,:)*Mtilde' + qr_dot(i-1,:)*Ctilde' + Gtilde + s*Kd' + e*Kp'; 
end
    
    
    %% Dinamica del manipolatore (reale)
    % entrano tau, q e q_dot, devo calcolare M, C e G e ricavare q_ddot
    % integro q_ddot due volte e ricavo q e q_dot
    M = KUKA.inertia(q(i-1,:)); 
    C = KUKA.coriolis(q(i-1,:),q_dot(i-1,:)); 
    G = KUKA.gravload(q(i-1,:)); 
    
    q_ddot = (tau(i,:) - q_dot(i-1,:)*C' - G) * (M')^(-1); 
    
    q_dot(i,:) = q_dot(i-1,:) + timeStep*q_ddot; 
    q(i,:) = q(i-1,:) + timeStep*q_dot(i,:); 
    
    %% Dinamica dei parametri
        q1 = q(i,1); q2 = q(i,2); q3 = q(i,3); q4 = q(i,4); q5 = q(i,5);

        q1_dot = q_dot(i,1); q2_dot = q_dot(i,2); q3_dot = q_dot(i,3); 
        q4_dot = q_dot(i,4); q5_dot = q_dot(i,5);

        qd1_dot = qd_dot(i,1); qd2_dot = qd_dot(i,2); qd3_dot = qd_dot(i,3);
        qd4_dot = qd_dot(i,4); qd5_dot = qd_dot(i,5);

        qd1_ddot = qd_ddot(i,1); qd2_ddot = qd_ddot(i,2); qd3_ddot = qd_ddot(i,3); 
        qd4_ddot = qd_ddot(i,4); qd5_ddot = qd_ddot(i,5);

        g = 9.81;

        regressor2;
    if sel2==2
        piArray_dot = ( R^(-1) * Y' * (Mtilde')^(-1) * [zeros(n) eye(n)] * P * [e e_dot]' )'; 
        
        piArray(i,:) = piArray(i-1,:) + timeStep*piArray_dot; 
    end
    
    if sel2==3
        piArray_dot = (R^(-1) * Y' * s')';  
        
        piArray(i,:) = piArray(i-1,:) + timeStep*piArray_dot; 
    end
    
    %% Progresso Simulazione
    if mod(i,100) == 0
        
        fprintf('Percent complete: %0.1f%%.',100*i/(length(t)-1));
        hms = fix(mod(toc,[0, 3600, 60])./[3600, 60, 1]);
        fprintf(' Elapsed time: %0.0fh %0.0fm %0.0fs. \n', ...
            hms(1),hms(2),hms(3));
    end
    
end

return


%% Plot della traiettoria
figure
set(gcf,'Position',[300 400 1200 800])
subplot(5,1,1)
plot(t,180/pi*qd(:,1),'b','LineWidth',1) 
hold on
plot(t,180/pi*q(:,1),'r','LineWidth',1) 
grid on
xlabel('time [s]')
ylabel('q_1 [deg]')
ylim([-180 180])
legend('reference','real')
switch sel2 
    case 1
        title('Computed torque control')
    case 2
        title('Adaptive computed torque control')
    case 3
        title('LiSlotine')
    case 4
        title('Adaptive Backstepping')
end

subplot(5,1,2)
plot(t,180/pi*qd(:,2),'b','LineWidth',1)
hold on
plot(t,180/pi*q(:,2),'r','LineWidth',1) 
grid on
ylim([-180 180])
xlabel('time [s]')
ylabel('q_2 [deg]')
legend('reference','real')

subplot(5,1,3)
plot(t,180/pi*qd(:,3),'b','LineWidth',1) 
hold on
plot(t,180/pi*q(:,3),'r','LineWidth',1) 
grid on
ylim([-180 180])
xlabel('time [s]')
ylabel('q_3 [deg]')
legend('reference','real')

subplot(5,1,4)
plot(t,180/pi*qd(:,4),'b','LineWidth',1) 
hold on
plot(t,180/pi*q(:,4),'r','LineWidth',1) 
grid on
ylim([-180 180])
xlabel('time [s]')
ylabel('q_4 [deg]')
legend('reference','real')

subplot(5,1,5)
plot(t,180/pi*qd(:,5),'b','LineWidth',1)
hold on
plot(t,180/pi*q(:,5),'r','LineWidth',1) 
grid on
ylim([-180 180])
xlabel('time [s]')
ylabel('q_5 [deg]')
legend('reference','real')

set(gcf,'Units','pixels')
set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
switch sel2 
    case 1
        saveas(gcf,'Figures/compTorque_q','png')
        savefig('Figures/compTorque_q.fig')
    case 2
        saveas(gcf,'Figures/AdaptCompTorque_q','png')
        savefig('Figures/AdaptCompTorque_q.fig')
    case 3
        saveas(gcf,'Figures/LiSlotine_q','png')
        savefig('Figures/LiSlotine_q.fig')
    case 4
        saveas(gcf,'Figures/Backstepping_q','png')
        savefig('Figures/Backstepping_q.fig')
end


error = zeros(size(qd,1),1);
for i = 1:length(error)
    Ttmp1 = KUKA.fkine(q(i,:));
    Ttmp2 = KUKA.fkine(qd(i,:));
    error(i) = norm(Ttmp1(1:3,4)-Ttmp2(1:3,4));
end

figure
set(gcf,'Position',[300 400 1200 400])
set(gcf,'Units','pixels')
set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
plot(t,error,'b','LineWidth',1)
grid on
xlabel('time [s]')
ylabel('error norm')
switch sel2 
    case 1
        title('Computed torque control')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/compTorque_error','png')
        savefig('Figures/compTorque_error.fig')
    case 2
        title('Adaptive computed torque control')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/AdaptCompTorque_error','png')
        savefig('Figures/AdaptCompTorque_error.fig')
    case 3
        title('Li Slotine')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/LiSlotine_error','png')
        savefig('Figures/LiSlotine_error.fig')
    case 4
        title('Backstepping adaptive')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/AdaptBackstepping_error','png')
        savefig('Figures/AdaptBackstepping_error.fig')
end


% Parameter estimation
figure
set(gcf,'Position',[300 400 1200 400])
plot(piArray(:,1),'b')
hold on
plot(piArray(:,11),'r')
plot(piArray(:,21),'g')
plot(piArray(:,31),'k')
plot(piArray(:,41),'m')
grid on
xlabel('time [s]')
ylabel('mass [kg]')
legend('link 1','link 2','link 3','link 4','link 5')
switch sel2 
    case 1
        title('Computed torque control')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/compTorque_mass','png')
        savefig('Figures/compTorque_mass.fig')
    case 2
        title('Adaptive computed torque control')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/AdaptCompTorque_mass','png')
        savefig('Figures/AdaptCompTorque_mass.fig')
    case 3
        title('Li Slotine')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/LiSlotine_mass','png')
        savefig('Figures/LiSlotine_mass.fig')
    case 4
        title('Backstepping adaptive')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/AdaptBackstepping_mass','png')
        savefig('Figures/AdaptBackstepping_mass.fig')
end


figure
set(gcf,'Position',[300 400 1200 400])
plot(piArray(:,2:4),'b')
hold on
plot(piArray(:,12:14),'r')
plot(piArray(:,22:24),'g')
plot(piArray(:,32:34),'k')
plot(piArray(:,42:44),'m')
grid on
xlabel('time [s]')
ylabel('position [m]')
title('CG position')
legend('link 1 x','link 1 y','link 1 z','link 2 x','link 2 y','link 2 z', ...
    'link 3 x','link 3 y','link 3 z','link 4 x','link 4 y','link 4 z','link 5 x','link 5 y','link 5 z')
switch sel2 
    case 1
        title('Computed torque control')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/compTorque_CGpos','png')
        savefig('Figures/compTorque_CGpos.fig')
    case 2
        title('Adaptive computed torque control')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/AdaptCompTorque_CGpos','png')
        savefig('Figures/AdaptCompTorque_CGpos.fig')
    case 3
        title('Li Slotine')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/LiSlotine_CGpos','png')
        savefig('Figures/LiSlotine_CGpos.fig')
    case 4
        title('Backstepping adaptive')
        set(gcf,'Units','pixels')
        set(gcf,'PaperUnits','inches','PaperPosition',get(gcf,'Position')/100)
        saveas(gcf,'Figures/AdaptBackstepping_CGpos','png')
        savefig('Figures/AdaptBackstepping_CGpos.fig')
end

return


%% Animation
ang = q;
grey = [0.8, 0.8, 0.8];
orange = [1, 0.4, 0];

figure
switch sel1
    case 1
        Tf = KUKA.fkine(qf);
        posf = Tf(1:3,4);
        plot3(posf(1),posf(2),posf(3),'r*')
    case {2,3}
        plot3(x,y,z,'r','Linewidth',1.5)
end
grid on
KUKA.plotopt = {'workspace',[-0.75,0.75,-0.75,0.75,0,1]};
KUKA.plot(ang(1:10:end,:),'floorlevel',0,'fps',1000,'trail','-k','linkcolor',orange,'jointcolor',grey)

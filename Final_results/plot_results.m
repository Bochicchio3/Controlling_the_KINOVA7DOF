%%


ref_0 = zeros(1,index-1);
ref_1 = ones(1, index-1);
refjoint=[pi/3*ref_1;ref_0;pi/3*ref_1;pi/3*ref_1;pi/6*ref_1; ref_0;ref_0];


pid=getfield(load ('pid.mat','result'),'result');
pd=getfield(load ('pd.mat','result'),'result');
ct=getfield(load('computed_torque.mat','result'),'result');
cpd=getfield(load('compensated_pd.mat','result'),'result');

figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:18000),pid(1:18000,j))
%     legend()
    hold on
    plot(t(1:18000),pd(1:18000,j))
%     legend('pd')
    plot(t(1:18000),ct(1:18000,j))
%     legend('computed torque')
    plot(t(1:18000),cpd(1:18000,j))
%     legend('compensated pd')
    plot (t(1:18000),refjoint(j,1:18000))
    legend('pid', 'pd', 'computed torque',' compensated pd','reference')
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
end
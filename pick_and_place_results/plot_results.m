%%


ref_0 = zeros(1,10001);
ref_1 = ones(1, 10001);
refjoint=[pi/3*ref_1;ref_0;pi/3*ref_1;pi/3*ref_1;pi/6*ref_1; ref_0;ref_0];


pid=getfield(load ('pid_tuned.mat','result'),'result');
pd=getfield(load ('pd.mat','result'),'result');
ct=getfield(load('computed_torque.mat','result'),'result');
cpd=getfield(load('compensated_pd.mat','result'),'result');


%%

pd=pd(1:10001,:)

ct=ct(1:10001,:)

cpd=cpd(1:10001,:)



%%



num_of_joints=7



figure




for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10001),pid(1:10001,j))
%     legend()
    hold on
    plot(t(1:10001),pd(1:10001,j))
%     legend('pd')
    plot(t(1:10001),ct(1:10001,j))
%     legend('computed torque')
    plot(t(1:10001),cpd(1:10001,j))
%     legend('compensated pd')
    plot (t(1:10001),refjoint(j,1:10001))
    legend('pid', 'pd', 'computed torque',' compensated pd','reference')
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
end

%%


% pd_error=pd-refjoint'
% 
% ct_error=ct-refjoint'
% 
% cpd_error=cpd-refjoint'
% 
% pid_error=pid-refjoint'
% 
% 
% %%
% figure
% for j=1:num_of_joints
%     subplot(4,2,j);
%     plot(t(1:10001),pid_error(1:10001,j))
% %     legend()
%     hold on
%     plot(t(1:10001),pd_error(1:10001,j))
% %     legend('pd')
%     plot(t(1:10001),ct_error(1:10001,j))
% %     legend('computed torque')
%     plot(t(1:10001),cpd_error(1:10001,j))
% %     legend('compensated pd')
% %     plot (t(1:10001),refjoint(j,1:10001))
%     legend('pid', 'pd', 'computed torque',' compensated pd')
%     xlabel('time [s]');
%     ylabeltext = sprintf('_%i [rad]',j);
%     ylabel(['Joint position' ylabeltext]);
%     grid;
% end

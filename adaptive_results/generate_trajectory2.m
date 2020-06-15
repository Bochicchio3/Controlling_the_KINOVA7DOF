function [q] = generate_trajectory2(xi,q0, KUKA)
%GENERATE_TRAJECTORY Summary of this function goes here

%   Detailed explanation goes here

    t_in = 0; % [s]
    t_fin = 10; % [s]
    delta_t = 0.001; % [s]
    timeSpan= 10;
    
    xi_dot= gradient(xi)*1000;


    q_dot=zeros(5,length(xi)+1);
    q=zeros(5,length(xi)+1);

    q(:,1)=q0;

    q_old = q0';

    J=KUKA.jacob0(q_old);
    q_dot(:,1)=pinv(J)*xi_dot(1:6,1);
    q(:,1)=q_old+q_dot(:,1)*delta_t;

    for i=1:length(xi_dot)

        J=KUKA.jacob0(q(:,i));
        q_dot(:,i)=pinv(J)*xi_dot(1:6,i);
        q(:,i+1)=q(:,i)+q_dot(:,i)*delta_t;

    end

end


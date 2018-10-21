clc
clear
close all

% Get pendulum variables
P = pendulum_variables();

% Create plots and get handles
[p,theta_plot,torque,dtheta_plot] = create_plots(P);

if P.do_gif
    f = getframe(h);
    [im,map] = rgb2ind(f.cdata,10);
    im(1,1,1,20) = 0;
end

tic;  %Start a timer
timeNow = 0;
q = P.q0; % state vector that gets incremented

for i=1:P.tSpan(2)/P.stepsize
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                                %
    %           THIS IS THE DYNAMICS STUFF           %
    %                                                %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ddq = pendulum_dynamics(0,q,P,P.controlfunc);
    q(1) = q(1) + P.stepsize*ddq(1);
    q(2) = q(2) + P.stepsize*ddq(2);
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                  %
    %           UPDATE PLOTS           %
    %                                  %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    th = q(1);
    dth = q(2);

    % If we selected no control, plot 0 torque
    if P.control
        tau = P.controlfunc([th;dth], P);  % calculate torque using controller
    else
        tau = 0;
    end
    
    % Update plots
    addpoints(torque,i,tau)       % add torque point to curve
    addpoints(theta_plot,i,th)    % add theta point to curve
    addpoints(dtheta_plot,th,dth)       % add point to state space curve
    % Update pendulum
    set(p,'Ydata',[0 P.l*cos(th-pi)],'Xdata',[0 P.l*sin(th-pi)])
    
    drawnow update
    pause(0.007);
    timeNow = toc;
    
    if P.do_gif
        f = getframe(h);  
        im(:,:,1,i) = rgb2ind(f.cdata,map);
%     im(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
    end
end

if P.do_gif
    imwrite(im,map,fiP.lame,'DelayTime',0.001,'LoopCount',inf) %g443800
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                  %
%           SET UP PLOTS           %
%                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [p,theta_plot,torque,dtheta_plot] = create_plots(P)
    %%% Set up plot
    h = figure('pos',[400 300 800 400]);
    set(gcf, 'Toolbar', 'none', 'Menu', 'none');
    set(gcf, 'Name', 'Pendulum Simulator', 'NumberTitle', 'Off') 

    
    duration = P.tSpan(2)/P.stepsize;

    % Plot containing pendulum
    s1 = subplot(3,2,[1 4]);
    p = plot([ 0 P.l*cos(0)],[0 P.l*sin(0)],'r-','LineWidth',6);
    axis equal 
    set(s1,'ylim',1.1*[-P.l P.l])
    set(s1,'xlim',1.1*[-P.l P.l])
    axis square; axis off;

    % Subplot containing angle and torque plots
    s2 = subplot(3,2,5);
    theta_plot = animatedline('Color','k','LineWidth',3);
    torque = animatedline('Color','r','LineWidth',3);
    xlabel('time (s)')
    ylabel('angle (rad)');
    set(s2,'xlim',[0 duration])
    set(s2,'ylim',1.2*[-1*pi 1*pi])

    % State space plot (dtheta vs theta)
    s3 = subplot(3,2,6);
    dtheta_plot = animatedline('Color','k','LineWidth',3);
    xlabel('theta (rad)')
    ylabel('dtheta (rad/s)');
    xlim auto
    set(s3,'xlim',1.2*[-2*pi 2*pi])
    set(s3,'ylim',1.2*[-2*pi 2*pi])
end

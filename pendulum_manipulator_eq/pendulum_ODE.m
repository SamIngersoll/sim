clc
clear
close all

P = pendulum_variables;

% Set some ODE options
options = odeset(...
    'AbsTol',1e-8,...
    'RelTol',1e-8,...
    'Vectorized','on');


%%% ODE CALL
%---------------------------------------------------
% Make function handle for dynamics function (this gets passed to ODE)
dyanmicsfunction = @(t,z)pendulum_dynamics(t,z,P,P.controlfunc);

% Run the simulation
sol = ode45(dyanmicsfunction,P.tSpan,P.q0,options);

%%% PROCESS SOLUTION
%---------------------------------------------------
% Break apart solution for plotting
nPlot = 1000;
time = linspace(P.tSpan(1),P.tSpan(2),nPlot);
z = deval(sol,time); %Evaluate solution from ode45 at points in time
th = z(1,:);
w = z(2,:);
[energy, kinetic, potential] = singlePendulumEnergy(th,w,P.m,P.g,P.l);
singlePendulumAnimate(sol,P);







function singlePendulumAnimate(sol,P)
    %This function is used to run an animation of the pendulum:
    set(gcf, 'Toolbar', 'none', 'Menu', 'none');
    set(gcf, 'Name', 'Pendulum Simulator', 'NumberTitle', 'Off') 

    tic;  %Start a timer
    timeNow = 0;
    len = P.l;

    duration = sol.x(end) - sol.x(1);

    % Plot containing pendulum
    s1 = subplot(3,2,[1 4]);
    p = plot([ 0 len*cos(0)],[0 len*sin(0)],'r-','LineWidth',6);
    axis equal 
    set(s1,'ylim',1.1*[-len len])
    set(s1,'xlim',1.1*[-len len])
    axis square; axis off;

    % Subplot containing angle and torque plots
    s2 = subplot(3,2,5);
    theta_plot = animatedline('Color','k','LineWidth',3);
    torque = animatedline('Color','r','LineWidth',3);
    xlabel('time (s)')
    ylabel('angle (rad)');
    set(s2,'xlim',[min(sol.x) max(sol.x)])
    set(s2,'ylim',1.2*[min(min(sol.y(1,:)), -P.tmax) max(max(sol.y(1,:)), P.tmax)])

    % State space plot (dtheta vs theta)
    s3 = subplot(3,2,6);
    dtheta_plot = animatedline('Color','k','LineWidth',3);
    xlabel('theta (rad)')
    ylabel('dtheta (rad/s)');
    xlim auto
    set(s3,'xlim',1.2*[min(sol.y(1,:)) max(sol.y(1,:))])
    set(s3,'ylim',1.2*[min(sol.y(2,:)) max(sol.y(2,:))])



    % ADD VALUES TO PLOT
    %---------------------------------------------------
    while timeNow < duration
        zNow = deval(sol,timeNow);
        th = zNow(1);
        dth = zNow(2);

        % Update plots
        % If we selected no control, plot 0 torque
        if P.control
            tau = P.controlfunc([th;dth], P);  % calculate torque using controller
        else
            tau = 0;
        end
        addpoints(torque,timeNow,tau)       % add torque point to curve
        addpoints(theta_plot,timeNow,th)    % add theta point to curve
        addpoints(dtheta_plot,th,dth)       % add point to state space curve

        % Update pendulum
        set(p,'Ydata',[0 len*cos(th-pi)],'Xdata',[0 len*sin(th-pi)])

        drawnow update
        pause(0.001);
        timeNow = toc;
    end
end



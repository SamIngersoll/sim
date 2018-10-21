function tau = swing_up(q,P)
    % max torque
    tmax = P.tmax; % we clamp torque to be between [-tmax, tmax]
    
    % unpack state
    theta = q(1,:);
    dtheta = q(2,:);
    
    thresh = 1.5;
    th = mod(theta,2*pi);
    if ( th > pi-thresh && th < pi+thresh )
        q_ = q;
        q_(1) = q_(1) + pi;
        tau = LQR(q_,P);
    else
        tau = energy_shaping(q,P);
    end
%     tau = energy_shaping(q,P); 
    
    % clamp torque between [-tmax, tmax]
    noise = P.noise*rand(1,1)-P.noise/2;
    tau = min(max(tau,-tmax),tmax)+noise;
    
end

function tau = energy_shaping(q,P)
    % unpack globals
    m = P.m;
    g = P.g;
    l = P.l;
    
    % max torque
    tmax = P.tmax; % we clamp torque to be between [-tmax, tmax]
    
    % unpack state
    theta = q(1,:);
    dtheta = q(2,:);

    % calculate energy of system
    T = (1/2)*m*(l^2)*(dtheta^2); % kinetic energy
    U = m*g*l*(1-cos(theta));     % potential energy
    E = T + U;      % total energy
    
    % calculate desired energy
    Ed = 2*m*g*l;   % desired energy (energy at top unstable fixed point)
    
    % calculate energy error (commanded)
    E_com = E - Ed;

    % calculate torque 
    % this is basically like reverse damping, we add torque in the
    % direction of the velocity (and proportional to it)
    tau = dtheta*E_com;  
end

function tau = LQR(q,P)
    % Linear Quadratic Regulator
    
    % unpack globals
    m = P.m;
    g = P.g;
    l = P.l;
    
    % Q and R are gain matrices, hardcoded for now
    % can also solve for the best Q and R if you're feeling good
    Q = eye(2);
    R = 1;
    
    % Dynamics evaluated at the fixed point (unstable one: pi)
    % eqn = a*m*l^2 + g*m*sin(th)*l
    H = m*l^2;
    C = 0;
    G = g*m*sin(pi)*l;
   
    A = [ 0   1  ;
         g/l^3  0  ];
    B = [ 0 ; 1/H ];
    
    [K,S,e] = lqr(A,B,Q,R);
    
    tau = R*B'*S*q;
%     tau = K*q;
end







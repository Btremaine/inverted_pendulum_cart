% pendulum_cart_integral.m
% inverted pendulum on cart with integral control
% design LQR at linearized set point
% then run in Simulink

clear; close all

% MKS units
M = 6.0;
m = 1.5;
L= 1.0; 
kv= 0.02;
b=  0.02;

% select setup
cart = false 
augmented =  true

% set cart true for Gantry cart simulation and false for inverted pendulum
% select cart versus inverted pendulum
switch cart
    case true
        g= -9.81;
    case false
        g=  9.81;
end

% Linearized model @ upright theta_~0
% Control input: force to Cart mass
% states [x, x_dot, th, th_dot]^T
%  
A = [ 0    1     0       0;
      0   -b/M  -m*g/M   kv/(M*L);
      0    0     0       1; 
      0   b/(M*L) g*(M+m)/(M*L) -kv*(1/m+1/M)/L^2];
B = [0;
     1/M;
     0;
    -1/(M*L)];
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
D = 0;
% continuous time model of plant
SYS = ss(A,B,C,D);

% discrete time model of plant
Ts= 0.1;
sys= c2d(SYS,Ts);

% check controllability
CO= ctrb(A,B(:,1));
% check observability with theta and x only
Cm = [1 0 0 0 ;
      0 0 1 0];
OB= obsv(A,Cm);

sprintf('rank controllability is %d',rank(CO))
sprintf('rank observability is %d',rank(OB))

% form discrete system without integral yet
Ad= sys.A;
Bd= sys.B; 
Cd= sys.C;
Dd= sys.D;

switch augmented
    case false
        % calculate lqr 
        lam = 0.90;
        Q= lam*diag([2 0.5 10 10]);  % cart
        R= (1-lam);
        [K, S, CLP] = lqr(sys,Q,R);

       figure(1); dstep(Ad-Bd*K,Bd,Cd,Dd);

    case true
        % now augment with integral state
        [n, m] = size(Ad);
        Cx = [1 zeros(1,m-1)];
        Ai = [Ad   zeros(n,1);
             -Cx   1];

        Bi = [ Bd 
                0 ];
        Ci=  eye(n+1);
        Di=  [Dd;
                0 ];

        sysi = ss(Ai,Bi,Ci,Di,Ts);

        % let lam vary 0.05 to 0.95 when tuning for best response
        lam = 0.90;
        Q= lam*diag([2 0.5 10 10 0.1]);  % cart
        R= (1-lam);
        [K, S, CLP] = lqr(sysi,Q,R);

        figure(1); dstep(Ai-Bi*K,Bi,Ci,Di);

end





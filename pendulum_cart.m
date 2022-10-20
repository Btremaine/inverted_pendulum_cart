% pendulum_cart.m
% inverted pendulum on cart
% design LQR at linearized set point
% then run in Simulink

% MKS units
M = 6.0;
m = 1.5;
L= 1.0; 
kv= 0.02;
b=  0.02;
g= 9.81;



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
SYS = ss(A,B,C,D);

Ts= 0.1;
sys= c2d(SYS,Ts);

CO= ctrb(A,B(:,1));

% check observability with theta and x only
Cm = [1 0 0 0 ;
      0 0 1 0];
OB= obsv(A,Cm);

% let lam vary 0.05 to 0.95
lam = 0.50;
Q= lam*diag([2 0.5 4 2]);
R= (1-lam);
[K, S, CLP] = lqr(SYS,Q,R);

Ad= sys.A;
Bd= sys.B;
Cd= sys.C;
Dd= sys.D;

figure(1); dstep(Ad-Bd*K,Bd,Cd,Dd);
figure(2); step(A-B*K,B,C,D);




function [phi_aug, Gamma_aug, T, K, cqlf_Ai] = control_model(Ts,delay,Q_multiplier,R)

%% model parameter setting
vx = 2.2; %longitudinal velocity(m/s)
LL = 0.55;  %look ahead distance(m) [less than 1s*Vx]

m = 1.599374; %mass of car(kg)
I_psi = 0.0705323934; %interia of CG(kg*m^2) [I=m*r^2]
l_f = 0.21; %distance from CG to front axle(m)
l_r = 0.21; %distance from CG to rear axle (m)
c_f = 2 * 60; %(N/rad)
c_r = 2 * 60; %cornering stiffness is increased by factor 2 -> 2 tires lumped together (N/rad)

%% original A B

a1 = c_f + c_r;
a2 = c_r * l_r - c_f * l_f;
a3 = -l_f * c_f + l_r * c_r;
a4 = l_f^2 * c_f + l_r^2 * c_r;
b1 = c_f / m;
b2 = l_f * c_f / I_psi;

%% state parameter description
% x1 is Vy
% x2 is yaw rate(rad/s)
% x3 is yL ==> F(S)=yL(S)/KL(S)
% x4 is epsilon_L

A = [-a1/(m*vx)     (a2-m*vx^2)/(m*vx)   0   0   0;
     a3/(I_psi*vx)  -a4/(I_psi*vx)       0   0   0;
     -1             -LL                  0   vx  0;
     0              -1                   0   0   vx;
     0              0                    0   0   0];
 
B = [b1; b2; 0; 0; 0];
C= [0 0 1 0 0];

%% cntinuous-time poles 
eigs(A)
%inv(A)

%% continuous-time state space
sysc = ss(A,B,C,0);

%% discrete sample data model for Delay<h
sysd1 = c2d(sysc,Ts-delay)
Gamma0 = sysd1.b

sysd2 = c2d(sysc, Ts)
Gamma1 = sysd2.b - Gamma0

phi = expm(A*Ts)

%% set augment model
phi_aug = [phi  Gamma1;zeros(1,6)]
Gamma_aug = [Gamma0; 1]

sysd = c2d(sysc,Ts);
C2c_d = sysd.c;
C_aug = [C 0];

%% check stabilizability
eig(phi_aug)

%% check controllablility
gamma = ctrb(phi_aug,Gamma_aug);
det(gamma)
if det(gamma) == 0
    disp('Uncontrollable');
else
    disp('Controllable');
end

%% controllable decomposition using example code
[phi2,Gamma2,C2,T,k] = ctrbf(phi_aug,Gamma_aug,C_aug)
sum(k) %sum(k) indicates number of controllable states

%% get conctrollable matix
phi2c= phi2(2:6,2:6)
Gamma2c = Gamma2(2:6)
C2c = C2(2:6)
                   
%% check controllability
gamma2 = ctrb(phi2c,Gamma2c)
rank(gamma2)
det(gamma2)
if det(gamma2) == 0
    disp('Uncontrollable');
else
    disp('Controllable');
end

Q = Q_multiplier*(C2c') * C2c;
[X,L,G]= dare(phi2c,Gamma2c,Q,R)
K = -G;
cqlf_Ai= phi2c + (K*Gamma2c);

end

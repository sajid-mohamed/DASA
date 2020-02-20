clc;
clear all;
format short;

%for 30 fps

%% A1
Ts = 0.035;
delay = 0.025; 
Q_multiplier=12.5;
R=1;
%settling time = 1.3979s, overshoot = 0.9797%
[phi{1},Gamma{1},K{1}, A1]=control_model2(Ts,delay,Q_multiplier,R);
% A1=power(A1,100);

%% A2
Ts = 0.065;
delay = 0.05; 
Q_multiplier=15;
R=1;
%settling time = 0.5865s, overshoot = 1.9437%
[phi{2},Gamma{2},K{2}, A2]=control_model2(Ts,delay,Q_multiplier,R);
% A2=power(A22,100);
% %% A3
% Ts = 0.035;
% delay = 0.03; 
% Q_multiplier=2;
% R=1;
% %settling time = 0.3843s, overshoot = 1.369%
% [phi{3},Gamma{3},K{3}, A3]=control_model2(Ts,delay,Q_multiplier,R);
% %A3=power(A33,10);

%% LMI Yalmip
Q=sdpvar(5,5);
L1=[[Q -Q*A1'; -A1*Q Q]>=0];
L2=[[Q -Q*A2'; -A2*Q Q]>=0];
% L3=[[Q -Q*A3'; -A3*Q Q]>=0];
L4=[Q >= 0];
% L=[L1, L2, L3, L4] %+ L5; %combine all constraints
L=[L1, L2, L4]

ops=sdpsettings('solver','sdpt3'); %mosek, sedumi, sdpt3
% ops=sdpsettings('verbose',1,'solver','sedumi','sedumi.eps',1e-12);
sol=optimize(L,[],ops); %Solving for P (matlab workspace)

Q=double(Q); 
eig(Q)
if (sol.problem==0)
    [~,r] = chol(Q); %to check for positive definiteness. r==0 for + def
    if r==0 %all(eig(P)>eps) %check for Positive Definite        
        display('Solution exists for Positive Definite Q matrix');
        eigenvalues = [ eig( [Q -Q*A1'; -A1*Q Q] ); 
%                         eig( value(Q) );
                        eig( [Q -Q*A2'; -A2*Q Q] )]; 
%                         eig( [Q -Q*A3'; -A3*Q Q] ) ];  
        if (any(eigenvalues < 0) == 0)
            display('Eigen value check (constraints) satisfied');
        else
            display('Eigen value check (constraints) NOT satisfied');
        end
    else
        display('Solution exists for Q, but it is not Positive Definite');        
        return;
    end
else
    display('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
    return;
end

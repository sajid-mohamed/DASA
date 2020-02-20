%"VISION-BASED LATERAL CONTROL OF VEHICLES"

%% A test code for controller analysis of various S-A delay & sample periods (3-test comparasion)
clc;
clear all;
format short;

%% Load reference vehicle track for 2 test situation
load('StraightTestTrack.mat');
% load('50mCurveTrackdata.mat');

%% set simulation step time and simulation test time
simstep = 0.005; %V-rep simulation step time
simulation_time = 7;

%% test parameters
vx = 2.2; % vehicle longitudinal speed is 2.2m/s to simulate ~80km/h in reality
LL = 0.55; % look-ahead distance is 0.25s*vx

%% initial controller parameter setting
delay = 0.075;
Ts = 0.1;
% clear the SSE
SSE_t = 0;

for loop = 1:3 %test for Q/R weight ratio value need set loop as 1:7
    clc;
    %% set various parameters for test : delay / Ts 
    if loop == 1
        delay = 0.025;
%         Ts = 0.035;
    else if loop == 2
            delay = 0.05;
%             Ts = 0.065;
        else if loop == 3;
                delay = 0.075;
%                 Ts = 0.1;
            end
        end
    end
    
%% model parameter setting
%% vehicle parameter setting
m = 1.599374; %mass of car(kg)
I_psi = 0.0705323934; %interia of CG(kg*m^2) [I=m*r^2]
l_f = 0.21; %distance from CG to front axle(m)
l_r = 0.21; %distance from CG to rear axle (m)
c_f = 2 * 60; %(N/rad)
c_r = 2 * 60; %cornering stiffness is increased by factor 2 -> 2 tires lumped together (N/rad)

%% parameters for original A B matrix of continuous SS model
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

%single meansurement output 'yL'
C= [0 0 1 0 0];

%% continuous-time poles 
eigs(A)
%inv(A)
 
%% continuous-time state space
sysc = ss(A,B,C,0);

%% set delay and sampling rate
delay = delay; % sensing delay
Ts = Ts; % sampling time

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
det(gamma);
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
det(gamma2);
if det(gamma2) == 0
    disp('Uncontrollable');
else
    disp('Controllable');
end

%% optimal control design - LQR feedback gain
R = 1;
Q = 12.5*(C2c') * C2c;

% % set various parameters for Q/R weight ratio test 
%     if loop == 1
%         R = 1;
%         Q = 2.5*(C2c') * C2c;
%     else if loop == 2
%             R = 1;
%             Q = 5*(C2c') * C2c;
%         else if loop ==3;
%                 R = 1;
%                 Q = 7.5*(C2c') * C2c;
%             else if loop == 4
%                     R = 1;
%                     Q = 10*(C2c') * C2c;
%                 else if loop == 5
%                         R = 1;
%                         Q = 12.5*(C2c') * C2c;
%                     else if loop == 6
%                             R = 1;
%                             Q = 15*(C2c') * C2c;
%                         else if loop == 7
%                                 R = 1;
%                                 Q = 17.5*(C2c') * C2c;
%                             end
%                         end
%                     end
%                 end
%             end
%         end
%     end

[X,L,G]= dare(phi2c,Gamma2c,Q,R)
K2c = -G

%% MATLAB controller with V-REP simulation
%transform set longitudinal velocity to desired joint speed in V-REP
desiredWheelRotSpeed = 2*vx/0.12681;

%parameter setting for Ackermann steering
d=0.135;%2*d=distance between left and right wheels
l=0.42;%l=distance between front and read wheels    

%% state parameter description
%z1 is vy
%z2 is yaw rate
%z3 is yL
%z4 is epsilon_L
%z5 is curvature at lookahead distance KL (which is K_ref of CoG)
%z6 is the input of the last sampling period.

%% initial vehicle state condition setting
%vehicle initial global corrdination
carx(2) = 0; carx(1) = 0;
cary(2) = 0; cary(1) = 0;
%vehecle initial yaw
yaw(2) = 0.0; yaw(1)= 0.0;

z1(2) = 0.0; z1(1) = 0.0;
z2(2) = 0.0; z2(1) = 0.0;
z3(2) = 0.0; z3(1) = 0.0;
z4(2) = 0.0; z4(1) = 0.0;
z5(2) = 0.0; z5(1) = 0.0; % initial KL value is 0
input(2) = 0.0; input(1) = 0.0;

time(2) = Ts; time(1) = 0;


%% Connecting V-REP

    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    
    if (clientID>-1)
        disp('Connected to remote API server');
        %switch on synchronous mode
        vrep.simxSynchronous(clientID,true);
        %% Commands before first simstep
        % start the co-simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

        %% initial co-simulation configuration
        % Handle V-REP objects
        % set navigation camera
        [returnCode,cam]=vrep.simxGetObjectHandle(clientID,'cam',vrep.simx_opmode_blocking);
        [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,cam,0,vrep.simx_opmode_streaming);
        % handle vehicle steering joint
        [returnCode,left_Steer]=vrep.simxGetObjectHandle(clientID,'nakedCar_steeringLeft',vrep.simx_opmode_blocking)
        [returnCode,right_Steer]=vrep.simxGetObjectHandle(clientID,'nakedCar_steeringRight',vrep.simx_opmode_blocking)
        % handle vehicle longitudinal velocity
        [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'nakedCar_motorLeft',vrep.simx_opmode_blocking)
        [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'nakedCar_motorRight',vrep.simx_opmode_blocking)
        % handle vehicle position
        [returnCode,car]=vrep.simxGetObjectHandle(clientID,'modAckermannSteeringCar',vrep.simx_opmode_blocking);
        [returnCode,floor]=vrep.simxGetObjectHandle(clientID,'ResizableFloor_5_25',vrep.simx_opmode_blocking);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_streaming);
        
        %% set initial steering angle to be zero
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
        %% set initial velocity to be zero
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,desiredWheelRotSpeed,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,desiredWheelRotSpeed,vrep.simx_opmode_blocking);

        %% let vehicle run 2.5s to reach the set longitudinal velocity
        for i=1:(2.5/simstep-1)
%         while position(1)<23 %start condition for entering curve case
            vrep.simxSynchronousTrigger(clientID);
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_streaming);
        end
        
        vrep.simxSynchronousTrigger(clientID);%trigger simulation step
        [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
        % get initial image for navigation
        [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,cam,0,vrep.simx_opmode_buffer);
%         imwrite(image,'initialposition.jpg');

        %% sensing initial parameters
        time(1) = 0.0;
        % sensing initial states
        [yL, out] = LateralDeviation(image,LL);
        imshow(out);
        z1(1) = 0.0;
        z3(1) = yL;
        z4(1) = 0.0;
        z5(1) = 2*z3(1)/((LL+l_r)^2);
        input(1) = 0.0;
   
        % calculate initial steering
        desiredSteeringAngle(1) = 0.0;
        % apply ackermann steering
        steeringAngleLeft(1)=atan(l/(-d+l/tan(desiredSteeringAngle(1))));%calculate left front tire steering angle according to desired steering angle
        steeringAngleRight(1)=atan(l/(d+l/tan(desiredSteeringAngle(1))));%calculate right front tire steering angle according to desired steering angle
        
        %% estimate the states for next iteration
        zkp1 = phi_aug*[z1(1);z2(1);z3(1);z4(1);z5(1);0] + Gamma_aug*desiredSteeringAngle(1); % original sample data augment system with delay
        z1(2) = zkp1(1);
        z2(2) = zkp1(2);
        z3(2) = zkp1(3);
        z4(2) = zkp1(4);
        z5(2) = zkp1(5);
        input(2) = desiredSteeringAngle(1);
        time(2) = time(1) + Ts;
        
        p=1;
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
        carx(p) = position(1);
        cary(p) = position(2);
        p = p+1;
        
        for j=1:(Ts/simstep-1)
            vrep.simxSynchronousTrigger(clientID);
            %set steering angle to keep zero-order hold input
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            %sensing vehicle global coordination
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
            carx(p) = position(1);
            cary(p) = position(2);
            p = p+1; 
        end
        vrep.simxSynchronousTrigger(clientID);
        %set steering angle to keep zero-order hold input
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);

        iteration = ceil(simulation_time/Ts+1);
        for i=2:iteration
            
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            % get update navigation camera image  %Always happens at V-rep simulation time: 0.1*(i-2)+0.02s
            [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,cam,0,vrep.simx_opmode_buffer);
            
            %% Sensing states
            [yL, out] = LateralDeviation(image,LL);
            imshow(out);
            z3(i) = yL;
%             % remove outlier when in entering curve situation [Need advanced filter mechanism]
%             if (yL > 0.01)
%                 z3(i) = z3(i-1);
%             end
            z5(i) = 2*z3(i)/((LL+l_r)^2);% estimate the curvature
            
            %% Computing input
            zt = T*[z1(i);z2(i);z3(i);z4(i);z5(i);input(i-1)]; %zt is the transferred state vector
            desiredSteeringAngle(i) = K2c*[zt(2);zt(3);zt(4);zt(5);zt(6)];% get input throught the controllable subsystem
            %apply ackemann steering
            steeringAngleLeft(i)=atan(l/(-d+l/tan(desiredSteeringAngle(i))));%calculate left front tire steering angle according to desired steering angle
            steeringAngleRight(i)=atan(l/(d+l/tan(desiredSteeringAngle(i))));%calculate right front tire steering angle according to desired steering angle

            %% Delay simulation in V-REP
            for j=1:(delay/simstep-1)
                % get car position
                [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
                carx(p) = position(1);
                cary(p) = position(2);
                p = p+1;
                
                vrep.simxSynchronousTrigger(clientID);
                % set steer angle=>zero order hold
                [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeft(i-1),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRight(i-1),vrep.simx_opmode_blocking);
                [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            end
            % get car position
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
            carx(p) = position(1);
            cary(p) = position(2);
            p = p+1;
            
            vrep.simxSynchronousTrigger(clientID);
            %% Actuating input
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,steeringAngleLeft(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,steeringAngleRight(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeft(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRight(i),vrep.simx_opmode_blocking);
            
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            % get car position
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
            carx(p) = position(1);
            cary(p) = position(2);
            p = p+1;
            
            %% Sample period simulation in V-REP
            for j=1:((Ts-delay)/simstep-1)
                vrep.simxSynchronousTrigger(clientID);
                %set steering angle to keep zero-order hold input
                [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeft(i),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRight(i),vrep.simx_opmode_blocking);
                
                [returnCode,pingTime]=vrep.simxGetPingTime(clientID);               
                % get car position
                [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
                carx(p) = position(1);
                cary(p) = position(2);
                p = p+1;
            end
            
            %% estimate the states for next iteration
            zkp1 = phi_aug*[z1(i);z2(i);z3(i);z4(i);z5(i);input(i-1)] + Gamma_aug*desiredSteeringAngle(i); % original sample data augment system with delay

            z1(i+1) = zkp1(1);
            z2(i+1) = zkp1(2);
%             z3(i+1) = zkp1(3);
            z4(i+1) = zkp1(4);
            z5(i+1) = zkp1(5);
            input(i+1) = desiredSteeringAngle(i);
            time(i+1) = time(i) + Ts;
            z3(i+1) = z3(i);
            desiredSteeringAngle(i+1) = desiredSteeringAngle(i);
            steeringAngleLeft(i+1)=steeringAngleLeft(i);
            steeringAngleRight(i+1)=steeringAngleRight(i);
            
            vrep.simxSynchronousTrigger(clientID);
            [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeft(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRight(i),vrep.simx_opmode_blocking);
        end
        
        % stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
        
        vrep.simxFinish(-1);
    end
    
    %% Calculate SSE (only for various S-A delay case & Q/R weight ratio case)
    klr=1/49.8395;%lane curvature
    z3r=((LL+l_r)^2)/(49.8395*2);%reference look-ahead lateral deviation stable value in entering curve case (for small straight bias case is zero)
    
    for i=1:(length(time)-1)
        SSE_t = SSE_t+((z3(i)-0)^2); %SSE for straight bias
%         SSE_t = SSE_t+((z3(i)-z3r)^2); %SSE for entering curve
    end
    SSE(loop)=SSE_t;
    %% store testing data (for S-A delay is 1-3, for Q/R weight ratio is 1-7)
        if loop == 1
            z30 = z3;
            z50 = z5;
            steer0 = desiredSteeringAngle;
            carx0 = carx;
            cary0 = cary;
            time0 = time;
        else if loop == 2
                z31 = z3;
                z51 = z5;
                steer1 = desiredSteeringAngle;
                carx1 = carx;
                cary1 = cary;
                time1 = time;
            else if loop ==3;
                    z32 = z3;
                    z52 = z5;
                    steer2 = desiredSteeringAngle;
                    carx2 = carx;
                    cary2 = cary;
                    time2 = time;
                else if loop == 4;
                        z33 = z3;
                        z53 = z5;
                        steer3 = desiredSteeringAngle;
                        carx3 = carx;
                        cary3 = cary;
                        time3 = time;
                    else if loop == 5;
                            z34 = z3;
                            z54 = z5;
                            steer4 = desiredSteeringAngle;
                            carx4 = carx;
                            cary4 = cary;
                            time4 = time;
                        else if loop == 6;
                                z35 = z3;
                                z55 = z5;
                                steer5 = desiredSteeringAngle;
                                carx5 = carx;
                                cary5 = cary;
                                time5 = time;
                            else if loop == 7;
                                    z36 = z3;
                                    z56 = z5;
                                    steer6 = desiredSteeringAngle;
                                    carx6 = carx;
                                    cary6 = cary;
                                    time6 = time;
                                else if loop == 8;
                                        z37 = z3;
                                        z57 = z5;
                                        steer7 = desiredSteeringAngle;
                                        carx7 = carx;
                                        cary7 = cary;
                                        time7 = time;
                                    else if loop == 9;
                                            z38 = z3;
                                            z58 = z5;
                                            steer8 = desiredSteeringAngle;
                                            carx8 = carx;
                                            cary8 = cary;
                                            time8 = time;
                                        end
                                    end
                                end
                            end
                        end
                    end                     
                end
            end
        end
        
        time = [];
        z1 = [];
        z2 = [];
        z3 = [];
        z4 = [];
        z5 = [];
        input = [];
        desiredSteeringAngle = [];
        steeringAngleLeft(i+1)=[];
        steeringAngleRight(i+1)=[];
        carx = [];
        cary = [];
        SSE_t = 0;
        
end

vrep.delete(); % call the destructor!
disp('Program ended');

%% plot the results 

%% plot performance of look-ahead lateral deviation
%array for reference look-ahead deviation stable value in entering curve case
for i=1:length(time2)
    yL_r(i)= z3r;
end

figure('name','Lateral Deviation(m)- original');

plot(time0,z30,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;

plot(time1,z31,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;

plot(time2,z32,'-s','MarkerSize',3,...
    'MarkerEdgeColor','g',...
    'Color','g');
hold on;

% plot reference stable value
plot(time2, time2*0, 'Color', [1,0.75,0]);
% plot(time2, yL_r, 'Color', [1,0.75,0]);

xlabel('time(s)');
ylabel('Look-ahead Lateral deviation(m)');
legend('delay=25ms','delay=50ms', 'delay=75ms','reference');
% legend('vx=2.2m/s;L=0.55m','vx=2.7m/s;L=0.675m','vx=3.6m/s;L=0.9m','reference');
% legend('Q=2.5*C2c^T*C2c;R=1','Q=3.5*C2c^T*C2c;R=1','Q=4.5*C2c^T*C2c;R=1','reference');
% legend('h=35ms','h=65ms', 'h=100ms','reference');
% legend('delay=30ms;h=50ms','delay=60ms;h=100ms', 'delay=90ms;h=150ms','reference');

z30_smooth(1:3)= z30(1:3);
z31_smooth(1:3)= z31(1:3);
z32_smooth(1:3)= z32(1:3);

z30_smooth(4:length(z30))= smooth(z30(4:length(z30)),0.1,'loess');
z31_smooth(4:length(z31))= smooth(z31(4:length(z31)),0.1,'loess');
z32_smooth(4:length(z32))= smooth(z32(4:length(z32)),0.1,'loess');

figure('name','Lateral Deviation(m)- smooth filtered');

plot(time0,z30_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;

plot(time1,z31_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;

plot(time2,z32_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','g',...
    'Color','g');
hold on;

plot(time2, time2*0, 'Color', [1,0.75,0]);
% plot(time2, yL_r, 'Color', [1,0.75,0]);
xlabel('time(s)');
ylabel('Lateral Deviation(m)');
legend('delay=25ms','delay=50ms', 'delay=75ms','reference');
% legend('vx=2.2m/s;L=0.55m','vx=2.7m/s;L=0.675m','vx=3.6m/s;L=0.9m','reference');
% legend('Q=2.5*C2c^T*C2c;R=1','Q=3.5*C2c^T*C2c;R=1','Q=4.5*C2c^T*C2c;R=1','reference');
% legend('h=35ms','h=65ms', 'h=100ms','reference');
% legend('delay=30ms;h=50ms','delay=60ms;h=100ms', 'delay=90ms;h=150ms')

%% plot estimated observed look-ahead lane curvature
%array for lane curvature
for i=1:length(time2)
    KL_r(i)= 1/49.8395;
end

figure('name','KL(m) - original');
plot(time0,z50,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;

plot(time1,z51,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;

plot(time2,z52,'-s','MarkerSize',3,...
    'MarkerEdgeColor','g',...
    'Color','g');
hold on;

% plot lane curvature as reference
plot(time2, time2*0, 'Color', [1,0.75,0]);
% plot(time2, KL_r, 'Color', [1,0.75,0]);

xlabel('time(s)');
ylabel('Look-ahead Curvature(m^-1)');
legend('delay=25ms','delay=50ms', 'delay=75ms','reference');
% legend('vx=2.2m/s;L=0.55m','vx=2.7m/s;L=0.675m','vx=3.6m/s;L=0.9m','reference');
% legend('Q=2.5*C2c^T*C2c;R=1','Q=3.5*C2c^T*C2c;R=1','Q=4.5*C2c^T*C2c;R=1','reference');
% legend('h=35ms','h=65ms', 'h=100ms','reference');
% legend('Centerline','delay=30ms;h=50ms','delay=60ms;h=100ms', 'delay=90ms;h=150ms');

z50_smooth(1:3)= z50(1:3);
z51_smooth(1:3)= z51(1:3);
z52_smooth(1:3)= z52(1:3);

z50_smooth(4:length(z50))= smooth(z50(4:length(z50)),0.1,'loess');
z51_smooth(4:length(z51))= smooth(z51(4:length(z51)),0.1,'loess');
z52_smooth(4:length(z52))= smooth(z52(4:length(z52)),0.1,'loess');

figure('name','KL(m) - smooth filtered');
plot(time0,z50_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;

plot(time1,z51_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;

plot(time2,z52_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','g',...
    'Color','g');
hold on;

% plot lane curvature as reference
plot(time2, time2*0, 'Color', [1,0.75,0]);
% plot(time2, KL_r, 'Color', [1,0.75,0]);

xlabel('time(s)');
ylabel('Look-ahead Curvature(m^-1)');
legend('delay=25ms','delay=50ms', 'delay=75ms','reference');
% legend('vx=2.2m/s;L=0.55m','vx=2.7m/s;L=0.675m','vx=3.6m/s;L=0.9m','reference');
% legend('Q=2.5*C2c^T*C2c;R=1','Q=3.5*C2c^T*C2c;R=1','Q=4.5*C2c^T*C2c;R=1','reference');
% legend('h=35ms','h=65ms', 'h=100ms','reference');
% legend('Centerline','delay=30ms;h=50ms','delay=60ms;h=100ms', 'delay=90ms;h=150ms');

%% plot control input - front tire steering angle
figure('name','steering angle(rad) - original');

plot(time0,steer0,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;

plot(time1,steer1,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;

plot(time2,steer2,'-s','MarkerSize',3,...
    'MarkerEdgeColor','g',...
    'Color','g');
hold on;

% plot expected stable value in small straight lane bias case
plot(time2, time2*0, 'Color', [1,0.75,0]);

xlabel('time(s)');
ylabel('Front wheel steering angle(rad)');
legend('delay=25ms','delay=50ms', 'delay=75ms','reference');
% legend('vx=2.2m/s;L=0.55m','vx=2.7m/s;L=0.675m','vx=3.6m/s;L=0.9m','reference');
% legend('Q=2.5*C2c^T*C2c;R=1','Q=3.5*C2c^T*C2c;R=1','Q=4.5*C2c^T*C2c;R=1','reference');
% legend('h=35ms','h=65ms', 'h=100ms','reference');
% legend('delay=30ms;h=50ms','delay=60ms;h=100ms', 'delay=90ms;h=150ms','reference');

steer0_smooth(1:3)= steer0(1:3);
steer1_smooth(1:3)= steer1(1:3);
steer2_smooth(1:3)= steer2(1:3);

steer0_smooth(4:length(z30))= smooth(steer0(4:length(steer0)),0.1,'loess');
steer1_smooth(4:length(z31))= smooth(steer1(4:length(steer1)),0.1,'loess');
steer2_smooth(4:length(z32))= smooth(steer2(4:length(steer2)),0.1,'loess');

figure('name','steering angle(rad) - smooth filtered');

plot(time0,steer0_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;

plot(time1,steer1_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;

plot(time2,steer2_smooth,'-s','MarkerSize',3,...
    'MarkerEdgeColor','g',...
    'Color','g');
hold on;

% plot expected stable value in small straight lane bias case
plot(time2, time2*0, 'Color', [1,0.75,0]);

xlabel('time(s)');
ylabel('Front wheel steering angle(rad)');
legend('delay=25ms','delay=50ms', 'delay=75ms','reference');
% legend('vx=2.2m/s;L=0.55m','vx=2.7m/s;L=0.675m','vx=3.6m/s;L=0.9m','reference');
% legend('Q=2.5*C2c^T*C2c;R=1','Q=3.5*C2c^T*C2c;R=1','Q=4.5*C2c^T*C2c;R=1','reference');
% legend('h=35ms','h=65ms', 'h=100ms','reference');
% legend('delay=30ms;h=50ms','delay=60ms;h=100ms', 'delay=90ms;h=150ms')

%% plot track of vehicle
figure('name','track of vehicle');
plot(trackx,tracky,'Color',[1,0.75,0]); %[r,g,b]
hold on;
plot(carx0,cary0,'r');
hold on;
plot(carx1,cary1,'b');
hold on;
plot(carx2,cary2,'g');
xlabel('Global X (m)');
ylabel('Global Y (m)');
legend('Centerline','delay=25ms','delay=50ms','delay=75ms');
% legend('Centerline','vx=2.2m/s;L=0.55m','vx=2.7m/s;L=0.675m','vx=3.6m/s;L=0.9m');
% legend('Centerline','Q=2.5*C2c^T*C2c;R=1','Q=3.5*C2c^T*C2c;R=1','Q=4.5*C2c^T*C2c;R=1');
% legend('Centerline','h=35ms','h=65ms','h=100ms');
% legend('Centerline','delay=30ms;h=50ms','delay=60ms;h=100ms', 'delay=90ms;h=150ms');

%% calculate min SSE
[minSSE,minIndex] = min(SSE)




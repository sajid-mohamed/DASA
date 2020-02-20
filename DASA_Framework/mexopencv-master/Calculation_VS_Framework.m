%"VISION-BASED" LATERAL CONTROL OF VEHICLES

%%   Detailed explanation goes here
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
delay = 0.05; % sensing delay
Ts = 0.065; % sampling time

%% discrete sample data model for Delay<h
sysd1 = c2d(sysc,Ts-delay);
Gamma0 = sysd1.b;

sysd2 = c2d(sysc, Ts);
Gamma1 = sysd2.b - Gamma0;

phi = expm(A*Ts);

%% set augment model
phi_aug = [phi  Gamma1;zeros(1,6)];
Gamma_aug = [Gamma0; 1];

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
[phi2,Gamma2,C2,T,k] = ctrbf(phi_aug,Gamma_aug,C_aug);
sum(k); %sum(k) indicates number of controllable states

%% get conctrollable matix
phi2c= phi2(2:6,2:6);
Gamma2c = Gamma2(2:6);
C2c = C2(2:6);
                   
%% check controllability
gamma2 = ctrb(phi2c,Gamma2c);
rank(gamma2);
det(gamma2);
if det(gamma2) == 0
    disp('Uncontrollable');
else
    disp('Controllable');
end

%% optimal control design - LQR feedback gain
R = 1;
Q = 12.5*(C2c') * C2c;

[X,L,G]= dare(phi2c,Gamma2c,Q,R);
K2c = -G;

eig(phi2c+Gamma2c*K2c)

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

%% initial conditions
carxz(2) = 0; carxz(1) = 0;
caryz(2) = 0; caryz(1) = 0;

yawz(2) = 0.0; yawz(1)= 0.0;

z1z(2) = 0.0; z1z(1) = 0.0;
z2z(2) = 0.0; z2z(1) = 0.0;
z3z(2) = 0.0; z3z(1) = 0.0;
z4z(2) = 0.0; z4z(1) = 0.0;
z5z(2) = 0.0; z5z(1) = 0.0; % initial KL value is 0
inputz(2) = 0.0; inputz(1) = 0.0;

timez(2) = Ts; timez(1) = 0;

%% Connecting V-REP
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    
    if (clientID>-1)
        disp('Connected to remote API server');
        
        vrep.simxSynchronous(clientID,true);
        % start the simulation:
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
        for j=1:(2.5/simstep-1)
            vrep.simxSynchronousTrigger(clientID);
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_streaming);
        end
        
        vrep.simxSynchronousTrigger(clientID);%trigger simulation step
        [returnCode,pingTime]=vrep.simxGetPingTime(clientID);

        %% sensing initial parameters
        % get initial image for navigation
        [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,cam,0,vrep.simx_opmode_buffer);
        [yL, out] = LateralDeviation(image,LL);
        z3z(1)=yL;
        imshow(out);
        z5z(1) = 2*z3z(1)/((LL+l_r)^2);
        inputz(1)=0;
        
        % calculate initial steering
        desiredSteeringAnglez(1) = 0;
        % apply ackermann steering
        steeringAngleLeftz(1)=atan(l/(-d+l/tan(desiredSteeringAnglez(1))));%calculate left front tire steering angle according to desired steering angle
        steeringAngleRightz(1)=atan(l/(d+l/tan(desiredSteeringAnglez(1))));%calculate right front tire steering angle according to desired steering angle
        
        [returnCode,SteerLz(1)]=vrep.simxGetJointPosition(clientID,left_Steer,vrep.simx_opmode_blocking);
        Steerz_F(1) = atan(l/(d+l/tan(SteerLz(1))));
        
        % calculate vehicle's yaw angle
        [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,car,floor,vrep.simx_opmode_blocking);
        if (eulerAngles(1)<=0)
                    yawz(1) = pi/2 - eulerAngles(2);
                else if (eulerAngles(1)>0)
                        yawz(1) = eulerAngles(2)-pi/2;
                    end
        end
        
        p=1;
        Steerz_F_hold(p) = Steerz_F(1);
        yawz_hold(1)=yawz(1);
        timez_hold(p)=0;
        p=p+1;
        
        % get vehicle global position coordination
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
        carxz(1) = position(1);
        caryz(1) = position(2);
        
        % estimate states for next loop according to control model
        zkp1 = phi_aug*[z1z(1);z2z(1);z3z(1);z4z(1);z5z(1);0] + Gamma_aug*desiredSteeringAnglez(1);
        z1z(2) = zkp1(1);
        z2z(2) = zkp1(2);
        z3ez(2) = zkp1(3);
        z4z(2) = zkp1(4);
        z5z(2) = zkp1(5);
        inputz(2) = desiredSteeringAnglez(1);
        timez(2) = timez(1) + Ts;
        
        for j=1:(Ts/simstep-1)
            vrep.simxSynchronousTrigger(clientID);
            %set steering angle to keep zero-order hold input
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
                
                [returnCode,SteerLz_hold]=vrep.simxGetJointPosition(clientID,left_Steer,vrep.simx_opmode_blocking);
                Steerz_F_hold(p) = atan(l/(d+l/tan(SteerLz_hold)));
                
                [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,car,floor,vrep.simx_opmode_blocking);
                if (eulerAngles(1)<=0)
                    yawz_hold(p) = pi/2 - eulerAngles(2);
                else if (eulerAngles(1)>0)
                        yawz_hold(p) = eulerAngles(2)-pi/2;
                    end
                end
                
                timez_hold(p)=timez_hold(p-1)+simstep;
                p=p+1;
            
        end
        vrep.simxSynchronousTrigger(clientID);
        %set steering angle to keep zero-order hold input
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
        
        iteration = ceil(simulation_time/Ts+1);
        for i=2:iteration
%           
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            %% sensing part
            %Get steering angle
            [returnCode,SteerLz(i)]=vrep.simxGetJointPosition(clientID,left_Steer,vrep.simx_opmode_blocking);
            Steerz_F(i) = atan(l/(d+l/tan(SteerLz(i))));
            %Get vehicle yaw
            [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,car,floor,vrep.simx_opmode_blocking);
                if (eulerAngles(1)<=0)
                    yawz(i) = pi/2 - eulerAngles(2);
                else if (eulerAngles(1)>0)
                        yawz(i) = eulerAngles(2)-pi/2;
                    end
                end
            
            %get update image
            [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,cam,0,vrep.simx_opmode_buffer);
            % cal lateral deviation
            [yL, out] = LateralDeviation(image,LL);
            imshow(out);
            
            %Assign measurement output yL
            z3z(i) = yL;
            
            z5z(i) = 2*z3z(i)/((LL+l_r)^2);% estimate the curvature

            %% Computing input
            zt = T*[z1z(i);z2z(i);z3z(i);z4z(i);z5z(i);inputz(i-1)]; %zt is the transferred state vector          
            desiredSteeringAnglez(i) = K2c*[zt(2);zt(3);zt(4);zt(5);zt(6)];% get input throught the controllable subsystem
            %apply ackemann steering
            steeringAngleLeftz(i)=atan(l/(-d+l/tan(desiredSteeringAnglez(i))));%calculate left front tire steering angle according to desired steering angle
            steeringAngleRightz(i)=atan(l/(d+l/tan(desiredSteeringAnglez(i))));%calculate right front tire steering angle according to desired steering angle

            %% Delay simulation in V-REP
            for j=1:(delay/simstep-1)
                
                [returnCode,SteerLz_hold]=vrep.simxGetJointPosition(clientID,left_Steer,vrep.simx_opmode_blocking);
                Steerz_F_hold(p) = atan(l/(d+l/tan(SteerLz_hold)));
                
                [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,car,floor,vrep.simx_opmode_blocking);
                if (eulerAngles(1)<=0)
                    yawz_hold(p) = pi/2 - eulerAngles(2);
                else if (eulerAngles(1)>0)
                        yawz_hold(p) = eulerAngles(2)-pi/2;
                    end
                end
                
                timez_hold(p)=timez_hold(p-1)+simstep;
                p=p+1;
                
                vrep.simxSynchronousTrigger(clientID);
                % set steer angle=>zero order hold
                [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeftz(i-1),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRightz(i-1),vrep.simx_opmode_blocking);
                [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            end
            
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            [returnCode,SteerLz_hold]=vrep.simxGetJointPosition(clientID,left_Steer,vrep.simx_opmode_blocking);
            Steerz_F_hold(p) = atan(l/(d+l/tan(SteerLz_hold)));
                
                [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,car,floor,vrep.simx_opmode_blocking);
                if (eulerAngles(1)<=0)
                    yawz_hold(p) = pi/2 - eulerAngles(2);
                else if (eulerAngles(1)>0)
                        yawz_hold(p) = eulerAngles(2)-pi/2;
                    end
                end
                
                timez_hold(p)=timez_hold(p-1)+simstep;
                p=p+1;

            vrep.simxSynchronousTrigger(clientID);
            %% set steering angle after delay (actuation)
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,steeringAngleLeftz(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,steeringAngleRightz(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeftz(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRightz(i),vrep.simx_opmode_blocking);
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            
            [returnCode,SteerLz_hold]=vrep.simxGetJointPosition(clientID,left_Steer,vrep.simx_opmode_blocking);
            Steerz_F_hold(p) = atan(l/(d+l/tan(SteerLz_hold)));
            
            [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,car,floor,vrep.simx_opmode_blocking);
            if (eulerAngles(1)<=0)
                    yawz_hold(p) = pi/2 - eulerAngles(2);
                else if (eulerAngles(1)>0)
                        yawz_hold(p) = eulerAngles(2)-pi/2;
                    end
                end
            
            timez_hold(p)=timez_hold(p-1)+simstep;
            p=p+1;

            %% Sample period simulation in V-REP
            for j=1:((Ts-delay)/simstep-1)
                vrep.simxSynchronousTrigger(clientID);
                %set steering angle to keep zero-order hold input
                [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeftz(i),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRightz(i),vrep.simx_opmode_blocking);
                [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
                 
                [returnCode,SteerLz_hold]=vrep.simxGetJointPosition(clientID,left_Steer,vrep.simx_opmode_blocking);
                Steerz_F_hold(p) = atan(l/(d+l/tan(SteerLz_hold)));
                
                [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,car,floor,vrep.simx_opmode_blocking);
                if (eulerAngles(1)<=0)
                    yawz_hold(p) = pi/2 - eulerAngles(2);
                else if (eulerAngles(1)>0)
                        yawz_hold(p) = eulerAngles(2)-pi/2;
                    end
                end
                timez_hold(p)=timez_hold(p-1)+simstep;
                p=p+1;
            end
            
            %% estimate the states for next iteration
            zkp1 = phi_aug*[z1z(i);z2z(i);z3z(i);z4z(i);z5z(i);inputz(i-1)] + Gamma_aug*desiredSteeringAnglez(i); % original sample data augment system with delay

            z1z(i+1) = zkp1(1);
            z2z(i+1) = zkp1(2);
%             z3ez(i+1) = zkp1(3);
            z4z(i+1) = zkp1(4);
            z5z(i+1) = zkp1(5);
            inputz(i+1) = desiredSteeringAnglez(i);
            z3z(i+1) = z3z(i);
            timez(i+1) = timez(i) + Ts;
            
            desiredSteeringAnglez(i+1)=desiredSteeringAnglez(i);
            steeringAngleLeftz(i+1)=steeringAngleLeftz(i);
            steeringAngleRightz(i+1)=steeringAngleRightz(i);
            
            SteerLz(i+1) = SteerLz_hold;
            Steerz_F(i+1) = Steerz_F_hold(p-1);
            yawz(i+1) = yawz_hold(p-1);

            %get absoulute position coordination
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_buffer);
            carxz(i) = position(1);
            caryz(i) = position(2);

            vrep.simxSynchronousTrigger(clientID);
            [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,steeringAngleLeftz(i),vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,steeringAngleRightz(i),vrep.simx_opmode_blocking);
        end
        
        % stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
        
        vrep.simxFinish(-1);
    end
        vrep.delete(); % call the destructor!

%% MATLAB calculation part

%% state parameter description
%z1s is vy
%z2s is yaws rate
%z3s is yL
%z4s is epsilon_L
%z5s is curvature at lookahead distance KL (which is K_ref of CoG)
%z6 is the inputs of the last sampling period.

%% initial conditions
carxs(1) = carxz(1);
carys(1) = caryz(1);
yaws(1)= 0;
carxsL(1) = carxs(1)+LL*cos(yaws(1));
carysL(1) = carys(1)+LL*sin(yaws(1));

z1s(1) = 0;
z2s(1) = 0;
z3s(1) = z3z(1);
z4s(1) = 0;
z5s(1) = 2*z3s(1)/((LL+l_r)^2); %z5s(1) = 2*z3s(1)/((LL+l_r)^2); % initial KL value is 0
inputs(1) = 0;

zkp1 = phi_aug*[z1s(1);z2s(1);z3s(1);z4s(1);z5s(1);0] + Gamma_aug*0; % original sample data augment system with delay
z1s(2) = zkp1(1);
z2s(2) = zkp1(2);
z4s(2) = zkp1(4);
inputs(2) = 0;
z3s(2) = zkp1(3);
%estimate position for next iteration
yaws(2) = yaws(1)+z2s(1)*Ts;
carxs(2) = carxs(1)+Ts*(vx*cos(yaws(1)) + z1s(1)*cos(yaws(1)+ pi/2));
carys(2) = carys(1)+Ts*(vx*sin(yaws(1)) + z1s(1)*sin(yaws(1)+ pi/2));

%estimate look-ahead point position for next iteration
carxsL(2) = carxs(2)+LL*cos(yaws(2));
carysL(2) = carys(2)+LL*sin(yaws(2));
z3s(2) = -0.1591 - carysL(2);
z5s(2) = 2*z3s(2)/((LL+l_r)^2);

p=1;
for j=1:(Ts/simstep)
    yaws_hold(p)=yaws(1);
    times_hold(p)=simstep*(j-1);
    p = p+1;
end

desiredSteeringAngles(1) = 0
times(2) = Ts; times(1) = 0;
        
iteration = ceil(simulation_time/Ts);
        for i=2:iteration
            
            for j=1:(Ts/simstep)
                yaws_hold(p)=yaws(i);
                times_hold(p)=times_hold(p-1)+simstep;
                p = p+1;
            end
            
            zts = T*[z1s(i);z2s(i);z3s(i);z4s(i);z5s(i);inputs(i-1)]; %zt is the transferred state vector
            desiredSteeringAngles(i) = K2c*[zts(2);zts(3);zts(4);zts(5);zts(6)];% get inputs throught the controllable subsystem
            
            
            %% estimate the states for next iteration
            zkp1 = phi_aug*[z1s(i);z2s(i);z3s(i);z4s(i);z5s(i);inputs(i-1)] + Gamma_aug*desiredSteeringAngles(i); % original sample data augment system with delay

            z1s(i+1) = zkp1(1);
            z2s(i+1) = zkp1(2);
%             z3s(i+1) = zkp1(3);
            z4s(i+1) = zkp1(4);
            z5s(i+1) = zkp1(5);
            inputs(i+1) = desiredSteeringAngles(i);
            times(i+1) = times(i) + Ts;
            desiredSteeringAngles(i+1) = desiredSteeringAngles(i);

            %estimate position for next iteration
            yaws(i+1) = yaws(i)+z2s(i)*Ts;
            carxs(i+1) = carxs(i)+Ts*(vx*cos(yaws(i)) + z1s(i)*cos(yaws(i)+ pi/2));
            carys(i+1) = carys(i)+Ts*(vx*sin(yaws(i)) + z1s(i)*sin(yaws(i)+ pi/2));
            
            %estimate look-ahead point position for next iteration
            carxsL(i+1) = carxs(i+1)+LL*cos(yaws(i+1));
            carysL(i+1) = carys(i+1)+LL*sin(yaws(i+1));
            z3s(i+1) = -0.1591 - carysL(i+1);
            z5s(i+1) = 2*z3s(i+1)/((LL+l_r)^2);
            
        end

%% plot the results 

% %% plot the zero-order hold yaw in framework
% figure('name','Framework zero-order hold Yaw(rad)');
% plot(timez_hold, yawz_hold, 'r');
% hold on;
% scatter(timez, yawz, 5, 'b');
% xlabel('time(s)');
% ylabel('Yaw(rad)');

%% plot comparison of lateral deviation
klr=1/49.8395;%lane curvature
z3r=((LL+l_r)^2)/(49.8395*2);%reference look-ahead lateral deviation stable value in entering curve case (for small straight bias case is zero)
for i=1:length(timez)
    yL_r(i)= z3r;
end
figure('name','Lateral Deviation(m)');
plot(timez,z3z,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;

plot(times,z3s,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;

plot(timez, z3z*0, 'Color', [1,0.75,0]);
% plot(timez, yL_r, 'Color', [1,0.75,0]);

xlabel('time(s)');
ylabel('yL(m)');
legend({'Framework','Calculation','reference'},'FontSize',20,'Location','northeast');

%% plot framework zero-order hold control input
figure('name','Framework input_zero hold(rad)');
plot(timez_hold, Steerz_F_hold, 'r');
hold on;
scatter(timez, Steerz_F, 5, 'b');
xlabel('time(s)');
ylabel('Set front steer angle(rad)');
legend('Framework ZOH steering angle','Framework steering angle sample point');

%% code end
disp('Program ended');




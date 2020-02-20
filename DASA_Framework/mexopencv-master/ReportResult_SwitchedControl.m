%"VISION-BASED LATERAL SWITCHED CONTROL OF VEHICLES"

%% A test code for controller analysis of various parameters (3-test comparasion)
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
l_r = 0.21; %distance from CG to rear axle (m)

%for 30 fps, period_s should be [0.033  0.066 0.1]; but the simulation step set as 5ms, so period_s=[0.035 0.065 0.1];
period_s=[0.035 0.065 0.1];
delay_s=[0.025 0.05 0.075]; %delay = 0.75*Ts

% Q weight for each pattern
Q_multiplier=[12.5 15 10];

timingPattern = [];
%set pattern sequence
timingPattern_init = [1 1;
                      1 2;
                      2 2];
%for 30 fps
Ts = period_s(1);
delay = delay_s(1);
% Q_multiplier=12.5;
%set R weight
R=1;
%A3=cqlf_Ai(Ts,delay,Q_multiplier,R)
[phi_aug{1}, Gamma_aug{1}, T{1}, K2c{1}, A1]=control_model(Ts,delay,Q_multiplier(1),R);

%% A2
Ts = period_s(2);
delay = delay_s(2); 
% Q_multiplier=15;
R=1;
[phi_aug{2}, Gamma_aug{2}, T{2}, K2c{2}, A2]=control_model(Ts,delay,Q_multiplier(2),R);

%% A3
Ts = period_s(3);
delay = delay_s(3); 
% Q_multiplier=10;
R=1;
[phi_aug{3}, Gamma_aug{3}, T{3}, K2c{3}, A3]=control_model(Ts,delay,Q_multiplier(3),R);

for loop = 1:3
    clc;
    timingPattern=timingPattern_init(loop,:);

%% MATLAB controller with V-REP simulation
%transform set longitudinal velocity to desired joint speed in V-REP
desiredWheelRotSpeed = 2*vx/0.12681;

%parameter setting for Ackermann steering
d=0.135;%2*d=distance between left and right wheels
l=0.42;%l=distance between front and read wheels    

%% initial vehicle state condition setting
%vehicle initial global corrdination
carx(2) = 0; carx(1) = 0;
cary(2) = 0; cary(1) = 0;
%vehecle initial yaw
yaw(2) = 0.0; yaw(1)= 0.0;

% clear the SSE
SSE_t = 0;

restart_flag = 1;

%% state parameter description
%z1 is vy
%z2 is yaw rate
%z3 is yL
%z4 is epsilon_L
%z5 is curvature at lookahead distance KL (which is K_ref of CoG)
%z6 is the input of the last sampling period.

z1(2) = 0.0; z1(1) = 0.0;
z2(2) = 0.0; z2(1) = 0.0;
z3(2) = 0.0; z3(1) = 0.0;
z4(2) = 0.0; z4(1) = 0.0;
z5(2) = 0.0; z5(1) = 0.0; % initial KL value is 0
input(2) = 0.0; input(1) = 0.0;

time(2) = period_s(timingPattern(restart_flag)); time(1) = 0;


%% Connecting V-REP

    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    
    clientID
    
    if (clientID>-1)
        disp('Connected to remote API server');
        %switch on synchronous mode
        vrep.simxSynchronous(clientID,true);
        %% Commands before first simstep [0-0.01s]
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

        %% run for reach set longitudinal velocity
        for i=1:(2.5/simstep-1)
%         while position(1)<23 %start condition for entering curve case
            vrep.simxSynchronousTrigger(clientID);
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,car,floor,vrep.simx_opmode_streaming);
        end
        
        vrep.simxSynchronousTrigger(clientID); % After this command, the V-rep simulation time becomes 0.01s
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
            zkp1 = phi_aug{timingPattern(restart_flag)}*[z1(1);z2(1);z3(1);z4(1);z5(1);0] + Gamma_aug{timingPattern(restart_flag)}*desiredSteeringAngle(1); % original sample data augment system with delay
            z1(2) = zkp1(1);
            z2(2) = zkp1(2);
            z3(2) = zkp1(3);
            z4(2) = zkp1(4);
            z5(2) = zkp1(5);
            input(2) = desiredSteeringAngle(1);
            time(2) = time(1) + period_s(timingPattern(restart_flag));
            restart_flag = restart_flag + 1;
            if restart_flag>size(timingPattern,2)
                restart_flag = 1;
            end
        
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
        
        vrep.simxSynchronousTrigger(clientID); % After this command, the V-rep simulation time becomes 0.02s
        %set steering angle to keep zero-order hold input
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,left_Steer,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointPosition(clientID,right_Steer,0,vrep.simx_opmode_blocking);

        i = 1;
        while (time(i+1) <= simulation_time)
            i = i + 1;
            
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
            %% get update navigation camera image
            [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,cam,0,vrep.simx_opmode_buffer);
            
            %% sensing states
            [yL, out] = LateralDeviation(image,LL);
            imshow(out);
            z3(i) = yL;
%             % remove outlier when in entering curve situation [Need advanced filter mechanism]
%             if (i>15)&&(abs(z3(i)-z3(i-1))>0.003)
%                 z3(i) = z3(i-1);
%             end
            z5(i) = 2*z3(i)/((LL+l_r)^2);% estimate the curvature
            
            %% Computing input
            zt = T{timingPattern(restart_flag)}*[z1(i);z2(i);z3(i);z4(i);z5(i);input(i-1)]; %zt is the transferred state vector
            desiredSteeringAngle(i) = K2c{timingPattern(restart_flag)}*[zt(2);zt(3);zt(4);zt(5);zt(6)];% get input throught the controllable subsystem
            steeringAngleLeft(i)=atan(l/(-d+l/tan(desiredSteeringAngle(i))));%calculate left front tire steering angle according to desired steering angle
            steeringAngleRight(i)=atan(l/(d+l/tan(desiredSteeringAngle(i))));%calculate right front tire steering angle according to desired steering angle

            %% Delay simulation in V-REP
            for j=1:(delay_s(timingPattern(restart_flag))/simstep-1)
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
            [returnCode,pingTime]=vrep.simxGetPingTime(clientID);
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
            for j=1:((period_s(timingPattern(restart_flag))-delay_s(timingPattern(restart_flag)))/simstep-1)
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
            
            restart_flag = restart_flag + 1;
            if restart_flag>size(timingPattern,2)
                restart_flag = 1;
            end
            
            %% estimate the states for next iteration
            zkp1 = phi_aug{timingPattern(restart_flag)}*[z1(i);z2(i);z3(i);z4(i);z5(i);input(i-1)] + Gamma_aug{timingPattern(restart_flag)}*desiredSteeringAngle(i); % original sample data augment system with delay
            z1(i+1) = zkp1(1);
            z2(i+1) = zkp1(2);
%             z3(i+1) = zkp1(3);
            z4(i+1) = zkp1(4);
            z5(i+1) = zkp1(5);
            z3(i+1) = z3(i);
            input(i+1) = desiredSteeringAngle(i);
            time(i+1) = time(i) + period_s(timingPattern(restart_flag));
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
    
    %% store testing data
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
        carx = [];
        cary = [];
        restart_flag = 1;
     
end

vrep.delete(); % call the destructor!
disp('Program ended');

%% plot the results

%array for reference yL & KL stable value in entering curve case
KL_ref = 1/49.8395;
yL_ref = ((LL+l_r)^2)*KL_ref/2;

for i=1:length(time2)
    yL_r(i) = yL_ref;
    KL_r(i) = KL_ref;
end

%% plot performance of look-ahead lateral deviation
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
legend('Pattern[1 1]','Pattern[1 2]','Pattern[2 2]','reference');
% legend('Pattern[2 2]','Pattern[2 3]','Pattern[3 3](WC)','reference');


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

% plot reference stable value
plot(time2, time2*0, 'Color', [1,0.75,0]);
% plot(time2, yL_r, 'Color', [1,0.75,0]);

xlabel('time(s)');
ylabel('Lateral Deviation(m)');
legend('Pattern[1 1]','Pattern[1 2]','Pattern[2 2]','reference');
% legend('Pattern[2 2]','Pattern[2 3]','Pattern[3 3](WC)','reference');

%% plot estimated observed look-ahead lane curvature
%array for lane curvature
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
ylabel('Estimated curvature(m^-1)');
% legend('Pattern[1 1]','Pattern[1 2]','Pattern[2 2]','reference');
legend('Pattern[2 2]','Pattern[2 3]','Pattern[3 3](WC)','reference');


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
ylabel('Estimated curvature(m^-1)');
% legend('Pattern[1 1]','Pattern[1 2]','Pattern[2 2]','reference');
legend('Pattern[2 2]','Pattern[2 3]','Pattern[3 3](WC)','reference');

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
legend('Pattern[1 1]','Pattern[1 2]','Pattern[2 2]','reference');
% legend('Pattern[2 2]','Pattern[2 3]','Pattern[3 3](WC)','reference');


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
legend('Pattern[1 1]','Pattern[1 2]','Pattern[2 2]','reference');
% legend('Pattern[2 2]','Pattern[2 3]','Pattern[3 3](WC)','reference');


%% plot track of vehicle
figure('name','track of vehicle');
plot(trackx,tracky,'Color',[1,0.75,0]); %[r,g,b]
hold on;
plot(carx0,cary0,'-s','MarkerSize',3,...
    'MarkerEdgeColor','r',...
    'Color','r');
hold on;
plot(carx1,cary1,'-s','MarkerSize',3,...
    'MarkerEdgeColor','b',...
    'Color','b');
hold on;
plot(carx2,cary2,'-s','MarkerSize',3,...
    'MarkerEdgeColor','g',...
    'Color','g');

legend('Pattern[1 1]','Pattern[1 2]','Pattern[2 2]','reference');
% legend('Pattern[2 2]','Pattern[2 3]','Pattern[3 3](WC)','reference');

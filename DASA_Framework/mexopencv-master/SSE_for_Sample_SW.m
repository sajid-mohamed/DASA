%"This code is for SSE calculation for various sample period test case of worst-case based controller and switched controller;
%The goal is to first pick same Nr of observation points to calculate SSE so that the result could be fair;
%The code is only fit for simulation step set to 5ms, simulation time is 7s;
%For worst-case based controller, the result store in array SSE_WC;
%For switched controller, the result store in variable SSE_BC, SSE_SW and SSE_WC."

%% Worst-case based controller: varying sample period
klr=1/49.8395;
z3r=((LL+l_r)^2)/(49.8395*2);
SSE_WC=[];
    
SSE_t=0;
for i=1:6:181
        SSE_t = SSE_t+((z30(i)-0)^2); %SSE for small straight lane bias
%         SSE_t = SSE_t+((z30(i)-z3r)^2); %SSE for entering curve
end
SSE_WC(1) = SSE_t*1000;

SSE_t=0;
for i=1:3:70
        SSE_t = SSE_t+((z31(i)-0)^2); %SSE for small straight lane bias
%         SSE_t = SSE_t+((z31(i)-z3r)^2); %SSE for entering curve
end
for i=74:4:98
        SSE_t = SSE_t+((z31(i)-0)^2); %SSE for small straight lane bias
%         SSE_t = SSE_t+((z31(i)-z3r)^2); %SSE for entering curve
end
SSE_WC(2) = SSE_t*1000;

SSE_t=0;
for i=1:2:55
        SSE_t = SSE_t+((z32(i)-0)^2); %SSE for small straight lane bias
%         SSE_t = SSE_t+((z32(i)-z3r)^2); %SSE for entering curve
end
for i=58:3:64
        SSE_t = SSE_t+((z32(i)-0)^2); %SSE for small straight lane bias
%         SSE_t = SSE_t+((z32(i)-z3r)^2); %SSE for entering curve
end
SSE_WC(3) = SSE_t*1000;

%% switched small straight lane bias
SSE_BC=0;
SSE_SW=0;
SSE_WC=0;
% SSE for best-case based controller
SSE_t=0;
for i=1:6:181
        SSE_t = SSE_t+((z30(i)-0)^2); %SSE for small straight lane bias
end
SSE_BC = SSE_t*1000;

% SSE for switched controller
SSE_t=0;
for i=1:4:113
        SSE_t = SSE_t+((z31(i)-0)^2); %SSE for straight bias
end
for i=118:5:128
        SSE_t = SSE_t+((z31(i)-0)^2); %SSE for straight bias
end
SSE_SW = SSE_t*1000;

% SSE for worst-case based controller
SSE_t=0;
for i=1:3:70
        SSE_t = SSE_t+((z32(i)-0)^2); %SSE for small straight lane bias
end
for i=74:4:98
        SSE_t = SSE_t+((z32(i)-0)^2); %SSE for small straight lane bias
end
SSE_WC = SSE_t*1000;

%% switch case curve
klr=1/49.8395;
z3r=((LL+l_r)^2)/(49.8395*2);
SSE_BC=0;
SSE_SW=0;
SSE_WC=0;
% SSE for best-case based controller
SSE_t=0;
for i=1:3:70
        SSE_t = SSE_t+((z31(i)-z3r)^2); %SSE for entering curve
end
for i=74:4:98
        SSE_t = SSE_t+((z31(i)-z3r)^2); %SSE for entering curve
end
SSE_BC = SSE_t*1000;

% SSE for switched controller
SSE_t=0;
for i=1:3:46
        SSE_t = SSE_t+((z31(i)-yL_ref)^2); %SSE for straight bias
end
for i=48:2:78
        SSE_t = SSE_t+((z31(i)-yL_ref)^2); %SSE for straight bias
end
SSE_SW = SSE_t*1000;

% SSE for worst-case based controller
SSE_t=0;
for i=1:2:55
        SSE_t = SSE_t+((z32(i)-z3r)^2); %SSE for entering curve
end
for i=58:3:64
        SSE_t = SSE_t+((z32(i)-z3r)^2); %SSE for entering curve
end
SSE_WC = SSE_t*1000;
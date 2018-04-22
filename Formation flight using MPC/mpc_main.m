clc;
clear all;
hold on;
%#########################################PARAMETERS DEFINITION


global wijx;
global wijy;
global no_agent;
global agent;
global Xcurrent;
global Xall;
global flag;
global aij;
global dii;   %DEGREE AGENT
global Nc;
Np=15;
Nc=8;
Ns=8;
agent = 1;
no_agent = 5;
flag = 0;
Xcurrent = zeros(Np,Ns);
Xall = zeros(Np,Ns,no_agent); %ALL AGENTS STATES TRACKING

Td = 0.2;
Abar = [1,Td,(Td^2)/2,((Td^3)/6); 0,1,Td,(Td^2)/2; 0,0,1,Td; 0,0,0,1];
Bbar = [(Td^4)/24; (Td^3)/6; (Td^2)/2; Td];
Cbar = eye(Ns);
A = [Abar,zeros(4);zeros(4),Abar];
B = [Bbar, zeros(4,1); zeros(4,1), Bbar];
C = eye(8);

%States of Agent
X0 = [20,0,0,0,50,0,0,0; 50,0,0,0,100,0,0,0; -10,0,0,0,-14,0,0,0; -3,0,0,0,8,0,0,0]'; 

%Leader is always last
%#########################################FORMATION SHAPE PROPERTIES

dii = [1,2,2,2,1];                       %DEGREE MATRIX PROPERTIES
wijx = [0,-5,-8,2,12; 5,0,6,-3,-4; 8,-6,0,-7,-5; -2,3,7,0,6; -12,4,0,5,-6];      %DESIRED DISTANCES IN FORMATION WRT TO LEADER
wijy = [0,-5,-3,2,4; 5,0,6,-3,-4; 2,-6,0,-7,-5; -2,3,7,0,6; -4,4,0,5,-6];
aij = [0,1,0,0,0; 1,0,1,0,0; 0,1,0,1,0; 0,0,1,0,1];          %ADJACENCY MATRIX
%#########################################PLANT MPC SETUP
Fsys = ss(A,B,C,0,0.2);     %MODIFY FOR 1D
Fmpc = mpc(Fsys, Td, Np, Nc);
%review(Fmpc);
%input('Enter a key');
%mpcDesigner(Fmpc);
Fmpc.MV(1).ScaleFactor = 1;
Fmpc.MV(2).ScaleFactor = 1;  %REMOVE FOR 1D
Fmpc.OV(1).ScaleFactor = 1;
Fmpc.OV(2).ScaleFactor = 1;
Fmpc.OV(3).ScaleFactor = 1;
Fmpc.OV(4).ScaleFactor = 1;
Fmpc.OV(5).ScaleFactor = 1;
Fmpc.OV(6).ScaleFactor = 1;
Fmpc.OV(7).ScaleFactor = 1;
Fmpc.OV(8).ScaleFactor = 1;
%Fmpc.DV.ScaleFactor = 1;

%Weights
Fmpc.Weights.OutputVariables = [100000 100000 100000 100000 100000 100000 100000 100000]; %{Q};
Fmpc.Weights.ManipulatedVariables = [1 1]; %{Ru};
%Fmpc.Weights.ManipulatedVariablesRate = 0.1; %{Rdu};
Fmpc.Optimizer.CustomCostFcn = true;


T = [0:0.2:20];
N = length(T);

for i=1:no_agent-1
    x(i) = mpcstate(Fmpc);
    x(i).Plant = X0(:,i);
end
y = zeros(no_agent-1,Ns,N);
u = zeros(no_agent-1,N,Ns/4);
r = zeros(no_agent-1,1,8);

start = cputime;
timelimit = 0;
itr=0;
while(timelimit < 140)    
    for i=1:N
    %% MPC Calculation
        for j=1:no_agent-1
            agent = j;
            y(j,:,i) = C*x(j).Plant;
            u(j,i,:) = mpcmove(Fmpc, x(j), y(j,:,i), r(j,:));
            Xall(:,:,agent) = Xcurrent;
        end
        
        %% Updating Leader Postion
        timelimit = cputime - start;
        if(timelimit < 30)
            %wij = [100,150;50,0];
            Xall(:,1,no_agent)=Xall(:,1,no_agent) + 1;
            if Ns==8
                Xall(:,5,no_agent)=Xall(:,5,no_agent)+ 0.5;
            end

        elseif(timelimit < 60)
            %wij = [100,150;50,0];
            Xall(:,1,no_agent)=Xall(:,1,no_agent) + 0.5;
            if Ns==8
                Xall(:,5,no_agent)=Xall(:,5,no_agent)+ 0.9;
            end
            
        elseif(timelimit < 90)
            %wij = [100,150;50,0];
            Xall(:,1,no_agent)=Xall(:,1,no_agent) + 0.5;
            if Ns==8
                Xall(:,5,no_agent)=Xall(:,5,no_agent)- 0.12;
            end

        elseif(timelimit < 120)
            %wij = [100,150;50,0];
            Xall(:,1,no_agent)=Xall(:,1,no_agent) - 0.5;
            if Ns==8
                Xall(:,5,no_agent)=Xall(:,5,no_agent)- 0.8;
            end
        end
        
        %% PLOTTINGS CODE
        figure(1);
        %hold on;
        plot(y(4,1,i),y(4,5,i),'m.',y(3,1,i),y(3,5,i),'c.',y(2,1,i),y(2,5,i),'b.',y(1,1,i),y(1,5,i),'g.',Xall(1,1,no_agent),Xall(1,5,no_agent),'r.');
        xlim([-10 80]);
        ylim([-10 80]);  
        % pause(0.05)
        %%
    end
    
    %{
    figure(2);
    subplot(2,1,1);
    plot(T, y1(1,:), 'r');
    subplot(2,1,2);
    plot(T, y2(1,:), 'b');
    figure(3);
    subplot(2,1,1);
    plot(T, y1(5,:), 'r');
    subplot(2,1,2);
    plot(T, y2(5,:), 'b');
    %}
end


%#################REFERENCE PATH CALCULATION

%function calcparamRef(Xall,



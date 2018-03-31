%============================================
%Developed by : Sushant Trivedi, Ninad Jadhav
%Paper Implemented : Y. Kuriki and T. Namerikawa: Formation control of UAVs with a fourth-order flight dynamics, Proc. 52nd IEEE Conference on Decision and Control, pp. 6706–6711, 2013.
%============================================

%Setting up UAV Dynamics
clc;
clear all;
c = ["red"; "yellow"; "green"; "brown"];

%======Update only these as per the number of clients============.
no_agent=5;
%Adj = [0,1,1,1; 0,0,0,1; 0,0,0,1; 0,0,0,0]; % Adjacency Matrix
Adj = [0,1,1,1,1; 0,0,1,0,0; 0,0,0,1,0; 0,0,0,0,1; 0,0,0,0,0]; % Adjacency Matrix
%Adj = [0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0,0,0;]; % Adjacency Matrix
%================================================================

%================================================================
%This code handles the chain connection failure scenarios where one of the
%agents in the chain is connected to all the agents. 

%Algorithm change. We basically ignore all other connections if the
%follower is directly connected to the leader since the most accurate
%information from the leader. to test this scenario run the following test case here
%direct implementation
%Adj = [0,1,1,1,1; 0,0,1,0,0; 0,0,0,1,0; 0,0,0,0,1; 0,0,0,0,0];
%no_agents=5;
%================================================================


%Need to add verification for smallest eigen values check and corresponding
%Beta values generation.
%------------------------------------------

%=======================VARIABLES REQUIRED ===============================
Td = 0.2;
t = 0:0.1:10;
Abar = [1,Td,(Td^2)/2,((Td^3)/6); 0,1,Td,(Td^2)/2; 0,0,1,Td; 0,0,0,1];
Bbar = [(Td^4)/24; (Td^3)/6; (Td^2)/2; Td];
A = [Abar,zeros(4);zeros(4),Abar];
B = [Bbar, zeros(4,1); zeros(4,1), Bbar];

fprintf("A: \n");
disp(A);
fprintf("B: \n");
disp(B)

%Beta parameters - Set initial conditions HERE %As per given in paper 'prev'
Beta = [1,5,20,3];

%Update this when number of agents changed
%-----------------------------------------
Prevpos = zeros(no_agent,2);

%Positions of all UAVs 1-Leader, rest Follower
Pos_def = [-2,-5; -5,0; -12,-10; 0,0]; 
des_def = [10,-5; 8,5; -5,0; 0,0]; 
Pos = zeros(no_agent,2);
des = zeros(no_agent,2);

%Give range for positions and desired positions for randomly generated values
rangex=50;
rangey=-50;
desx=10;
desy=-10;
if(no_agent < 5)
    Pos = Pos_def;
    des = des_def;
else
    %Keeping some default positions and desired states for simulations.
    for i=1:3
        Pos(i,1:2) = Pos_def(i,1:2); 
        des(i,1:2) = des_def(i,1:2); 
    end
    for i=4:no_agent-1
        r1 = randi(100,1,1);
        r2 = randi(100,1,1);
        Pos(i,1:2) = [r1,r2];
        
        r1 = randi(50,1,1);
        r2 = randi(50,1,1);
        des(i,1:2) = [r1,r2]; 
    end
    
    %Last is always leader.
    Pos(no_agent,1:2)=[0,0];
    des(no_agent,1:2)=[0,0];
end

Sk = zeros(4,2,no_agent);
Xk = zeros(8,no_agent);
Mk = zeros(no_agent,2);
d = zeros(4,2,no_agent); 

for i=1:no_agent
    Sk(:,:,i)= [Pos(i,1),Pos(i,2); 0,0; 0,0; 0,0];
    Xk(:,i)=(cat(1,Sk(:,1,i),Sk(:,2,i)));
    d(:,:,i) = [des(i,1),des(i,2); 0,0; 0,0; 0,0];
end

count=0;

%COMMENTS STATEMENTS 
fprintf("------------------------------\nINITIAL CONDITIONS\nM:\n");
disp(Mk);
fprintf("Xk:\n");
disp(Xk);
fprintf("------------------------------\nLOOP STARTED\n")

hold on;
flag = 0;
initial_time = cputime;
start = initial_time;
d_agent = cond(Xk,d,no_agent);
%d_sum=100;

while(cond(Xk,d,no_agent) > 0.01)
    %d_sum = cond(Xk,d,no_agent);
    current_time = cputime;
    timelimit = current_time-start;
    if ((initial_time + 0.5 <= current_time) && (timelimit < 15))
        %Update this when number of agents changed
        %-----------------------------------------
        Pos(no_agent,:) = Pos(no_agent,:)+ [1,1];
        Sk(:,:,no_agent) = [Pos(no_agent,:); 2,2; 0,0; 0,0];
        fprintf("-------------------------------\nLEADER POS UPDATED:\n");
        disp(Pos(no_agent,:));
        %-----------------------------------------
        initial_time = cputime;
        %input('give inp : ');
    
    elseif(timelimit>15 && timelimit<30)
        Pos(no_agent,:) = Pos(no_agent,:)+ [0,0.25];
        Sk(:,:,no_agent) = [Pos(no_agent,:); 0,0.2; 0,0; 0,0];
        fprintf("-------------------------------\nLEADER POS UPDATED:\n");
        disp(Pos(no_agent,:));
    
    elseif(timelimit > 30)
        Sk(:,:,no_agent) = [Pos(no_agent,:); 0,0; 0,0; 0,0];
        fprintf("LEADER STOPPED");
    end
    mvalue(Sk, Mk,d,Beta, A, B, Pos,Adj, no_agent);
end

endtime = cputime;
fprintf("==========FORMATION STABILIZED. END OF SIMULATION=========\n");
fprintf("Time for simulation = \n");
disp(endtime - start);

%#################################################################################
function mvalue(Sk,Mk,d,Beta,A,B,Pos,Adj, no_agent)

%Update this when number of agents changed
%-----------------------------------------
Sek = zeros(4,2,no_agent);
for i=1:4  %k state
    for j=1:no_agent %jth agent
        Sek(i,:,j) = Sk(i,:,j) - d(i,:,j);
    end
end

for i=1:(no_agent-1) %i-th agent 
    if Adj(i,no_agent) == 1
       j=no_agent;
       Mk(i,:)= Mk(i,:)-(Adj(i,j)*((Beta(1)*(Sek(1,:,i)-Sek(1,:,j)))+(Beta(2)*(Sek(2,:,i)-Sek(2,:,j)))+(Beta(3)*(Sek(3,:,i)-Sek(3,:,j)))+(Beta(4)*(Sek(4,:,i)-Sek(4,:,j)))));
    else
        for j=1:no_agent %M calculation over all agents
               Mk(i,:)= Mk(i,:)-(Adj(i,j)*((Beta(1)*(Sek(1,:,i)-Sek(1,:,j)))+(Beta(2)*(Sek(2,:,i)-Sek(2,:,j)))+(Beta(3)*(Sek(3,:,i)-Sek(3,:,j)))+(Beta(4)*(Sek(4,:,i)-Sek(4,:,j)))));
        end
    end
end
fprintf("------------------------------\nMk:\n");
disp(Mk);

% STATE CALCULATION
X = zeros(8,no_agent);
for i=1:no_agent %Each agent
    Xk = [Sk(:,1,i); Sk(:,2,i)];
    Xk1 = (A*Xk) + (B*transpose(Mk(i,:)));
    X(:,i) = Xk1;
end

fprintf("------------------------------\nX:\n");
disp(X);

%Update this when number of agents changed
%-----------------------------------------
Prevpos = Pos;
for i=1:no_agent
    Pos(i,1) = X(1,i);
    Pos(i,2) = X(5,i);
end
%fprintf("-----------------------\nPOSITIONS STATUS\nk-1:\n");
%disp(Prevpos);
%fprintf("k:\n");
%disp(Pos);

for i=1:no_agent
    %Some Default trajectories for the default agents.LEADER IS ALWAYS RED.
    if i==1
        color=':.b';
    elseif i==2
        color=':.g';
    elseif i==no_agent
        color=':.r';
    else
        out=randi(3,1,1);
        switch out
            case 1
                color=':.m';
            case 2
                color=':.k';
            case 3
                color=':.c';
        end
    end
    plot([Prevpos(i,1),Pos(i,1)],[Prevpos(i,2),Pos(i,2)],color);
end

xlim([-15 275]);
ylim([-15 275]);
pause(0.05);

%GET STATES FROM X VECTOR
S = zeros(4,2,no_agent);
for i=1:no_agent
    S(:,:,i) = cat(2, X(1:4,i), X(5:8,i));
end

assignin('base','Sk', S);   %RETURN VALUE
assignin('base','Xk', X);   %RETURN VALUE
assignin('base','Pos', Pos);   %RETURN VALUE
assignin('base','Prevpos', Prevpos);   %RETURN VALUE
%assignin('base','Mk',Mk) %Updating this causes error;     
fprintf("-----------------------\nLoop ended\n");
end


%################################################################################################

%%%FUNCTION DEFINITION
%Calculating While condition
function des_sum = cond(Xk,d,no_agent)
%d -> 4x2x2; Xk->8x2; X->4x2

% POSITION ARRAY
Vpos = zeros(no_agent,1);
Dpos = zeros(no_agent,1);
for i=1:no_agent
    Vpos(i) = pdist([0,0; Xk(1,i),Xk(5,i)]);
    Dpos(i) = pdist([0,0; d(1,1,i), d(1,2,i)]);
end
fprintf("------------DPOS--------\n");
disp(transpose(Vpos));

for i=1:(no_agent-1)
        X_vec(1,1) =    Xk(1,i) - Xk(1,no_agent) - d(1,1,i);   
        X_vec(1,2) =    Xk(5,i) - Xk(5,no_agent) - d(1,2,i);
        distsolo(i) = pdist([0,0; X_vec(1,1), X_vec(1,2)]);
end

fprintf("------------------------\nDeviation: ");
disp(distsolo);
des_sum=sum(distsolo);
end

function [f,dfdy,dfdu,dfddu,dfdslack] = mpcCustomCostFcn(y,yref,u,uref,du,v,slack,varargin)
% This is a template for specifying arbitrary cost "f" in MPC controller.  
% This cost function replaces the standard quadratic cost function.
% "fmincon" from Optimization Toolbox is used to solve the nonlinear programming problem.
% Set "Optimizer.CustomCostFcn = true" in MPC controller to enable this feature.
% Gradients are optional although having them would help with computational efficiency.
%
% Inputs:
%   y:      predicted plant outputs, p-by-ny, from k+1 to k+p 
%   yref:   previewed output reference, p-by-ny, from k+1 to k+p
%   u:      optimal control sequence, p-by-nmv, from k to k+p-1
%   uref:   previewed mv target, p-by-nmv, from k to k+p-1 (although we don’t provide preview for mv target now, this interface is general)
%   du:     rate of control changes, p-by-nmv, from k to k+p-1 (the first value is u(k)-u(k-1))
%   v:      previewed MD, p+1-by-nmd, from k to k+p ([] is passed in if MD does not exist)
%   slack:  global slack variable, scalar
%
% Outputs:
%           f:    nonlinear cost, scalar
%
%        dfdy:    df/dy, p-by-ny, (return [] if unknown)
%        dfdu:    df/du, p-by-nmv, (return [] if unknown)
%       dfddu:    df/ddu, p-by-nmv, (return [] if unknown)
%    dfdslack:    df/dslack, 1-by-1, (return [] if unknown)
%
% Note that:
%   (1) Prediction horizon can be obtained as "size(y,1)".
%   (3) The previous MV, u(k-1), can be computed as "u(1,:)-du(1,:)".
%   (3) Economic MPC does not support "block moves".
%
% In the template, standard quadratic cost function with default weights is implemented as an example.
% Dimension
p = size(y,1); %PREDICTION HORIZON
global Nc;
Nc = 8; %CONTROL HORIZON
Ns = 8; %NO OF STATES
nmv = size(u,2);
ny = size(y,2);
global wijx;
global wijy;
global agent;
global Xall;
global Xcurrent;
global no_agent;
global aij;
global dii;
Xcurrent = y;
Wij = zeros(p,Ns);
for i=1:p
    %X-axis
    temp = 0;
    for j=1:no_agent
       temp  =  temp + aij(agent,j)*(Xall(i,1,j)-wijx(agent,j));
    end
    Wij(i,1) = (1/dii(1,agent))*temp;
    %Y-axis
    if Ns==8
        temp1=0;
        for j=1:no_agent
            temp1 = temp1 + aij(agent,j)*(Xall(i,5,j)-wijy(agent,j));
        end
        Wij(i,5) = (1/dii(1,agent))*temp1;
    end
end

r = zeros(p,Ns);
r = y-Wij;
%With +sign
P = 1.0e+07*[0.5662, 0.4947, 0.3682, 0.0483; 0.4947, 1.5090, 0.7622, 0.1778; 0.3682, 0.7622, 1.1391, 0.1535; 0.0483, 0.1778, 0.1535, 0.1455];
%With -sign
%P = 1.0e+05*[1.2955,0.7087,0.3468,-0.1560; 0.7087,2.8074,1.2066,-0.4904; 0.3468,1.2066,1.5900, -0.4833; -0.1560,-0.4904,-0.4833,0.1554];
Q = 10^5*eye(4);
R = 1;
if Ns==8
    P = [P,zeros(4,4);zeros(4,4),P];
    Q = 10^5*eye(8);
    R = 1;
end
f = 0.5*((r(p,:)*P*transpose(r(p,:)))+trace(r*Q*transpose(r)) + sum(u(1:Nc).^2));

dfdy = [];
dfdu = [];
dfddu = [];
dfdslack = [];
end

% Default weights (MV weights are 0)
%{
Wy = diag(ones(ny,1));
Wdu = diag(ones(nmv,1))*0.1; 
Wecr = 1e5;
%}
% Quadratic cost
%f = sum(sum(((y-yref)*Wy).^2))+sum(sum((du*Wdu).^2))+Wecr*slack^2;
% Gradients
%{
dfdy = (y-yref)*(Wy.^2);
dfdu = zeros(p,nmv);
dfddu = du*(Wdu.^2);
dfdslack = Wecr*slack;
%}
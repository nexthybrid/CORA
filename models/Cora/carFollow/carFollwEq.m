function dx = carFollwEq(x,u)
% carFollwEq - system dynamics for the car-following example
%               (see [1])
%
% Syntax:  
%    dx = carFollwEq(x,u)
%
% Inputs:
%    x - state vector
%    u - input vector
%
% Outputs:
%    dx - time-derivate of the system state
% 
% References:
%    [1] Two_vehicle_model_car_follow.pdf (root folder in this branch)

% Author:        Tong Zhao
% Written:       04-Oct-2022
% Last update:   ---
% Last revision: ---

%------------- BEGIN CODE --------------
% The list of equations and parameters
% dot v1    = a0* (1 - (v1/v0)^delta - (s_star/s)^2)
% dot sx1  = v1
% dot v2    = a2
% dot sx2  = v2
%
% s = sx2 - sx1 - L
% s_star = s0 + max(0, v1*Th+0.5*v1*(v1-v2)/sqrt(a0*b0))
%
% parameters: v0,delta,s0, Th, a0, b0, L
%
% x = [v1,sx1,v2,sx2]
% u = a2

v0 = 15; delta = 4; s0 = 1; Th = 0.5; a0 = 2; b0 = 3; L = 5;

v1 = x(1); sx1 = x(2); v2 = x(3); sx2 = x(4);
a2 = u;

s =  sx2 - sx1 - L;
s_star = s0 + v1*Th+0.5*v1*(v1-v2)/sqrt(a0*b0);

% dx(1,1) = a0* (1 - (v1/v0)^delta - (s_star/(s+0.1))^2); % IDM
dx(1,1) = a0* (1 - (v1/v0)^delta +(0.1*s)^2);   % distance-keeping v1
% dx(1,1) = a0* (1 - (v1/v0)^delta);   % only constant speed cruise
dx(2,1) = v1;
dx(3,1) = a2;
dx(4,1) = v2;

    
%------------- END OF CODE --------------
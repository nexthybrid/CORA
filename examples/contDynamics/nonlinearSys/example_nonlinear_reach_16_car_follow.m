% function completed = example_nonlinear_reach_16_car_follow()
% example_nonlinear_reach_16_car_follow - example of non-linear 
%                                                     reachability analysis 
%
%    Example of reachability analysis for non-linear closed-loop car follow
%    system [1] using zonotopes and polynomial zonotopes.
%
% Syntax:  
%    completed = example_nonlinear_reach_16_car_follow()
%
% Inputs:
%    no
%
% Outputs:
%    completed - boolean 
%
% References: 
%   [1] One_vehicle_model_car_follow.pdf (root folder in this branch)

% Author:       Tong Zhao
% Written:      04-Oct-2022
% Last update:  ---
% Last revision:---


%------------- BEGIN CODE --------------

% Parameters --------------------------------------------------------------

params.tFinal = 3;                              % final time [s]
% R0 = zonotope([[1.4;2.4], diag([0.17, 0.06])]);    % initial set
R0 = zonotope([[15;1;15;20], diag([1,1,5,1])]);    % initial set

% Reachability Settings ---------------------------------------------------

% settings
options.timeStep = 0.005;                           
options.taylorTerms = 4;                            
options.zonotopeOrder = 50;       
options.intermediateOrder = 50;
options.errorOrder = 20;

% reachability algorithm
options.alg = 'poly';
options.tensorOrder = 3;

% special settings for polynomial zonotopes
polyZono.maxDepGenOrder = 50;
polyZono.maxPolyZonoRatio = 0.01;
polyZono.restructureTechnique = 'reducePca';



% System Dynamics ---------------------------------------------------------

carFollow = nonlinearSys(@carFollwEq);



% Reachability Analysis (zonotope) ----------------------------------------
      
% adapted options
params.R0 = R0;
% syntax for params.U (seemingly):
% params.U = zonotope([center,generator]); 
% Example:
% To represent a 1-dim control input ranging between -4 and -2,
% we can calculate the center as (-4 + -2)/2 = -3, and the amplitude
% as (-2 - -4)/2 = 1. So center = -3, generator = 1. The generator
% is a term used in zonotope definition. See Girard 2005.
params.U = zonotope([-3,1]); 

% compute reachable set
tic
R = reach(carFollow, params, options);
tComp = toc;
disp(['computation time (zonotope): ',num2str(tComp)]);



% % % % Reachability Analysis (polynomial zonotope) -----------------------------
% % %       
% % % % adapted options
% % % params.R0 = polyZonotope(R0);
% % % 
% % % options.polyZono = polyZono;
% % % 
% % % % compute reachable set
% % % tic
% % % Rpoly = reach(carFollow, params, options);
% % % tComp = toc;
% % % disp(['computation time (polynomial zonotope): ',num2str(tComp)]);



% Visualization -----------------------------------------------------------

%--- combined two vehicle states
figure; hold on;
% plot reachable set (zonotope)
handleVeh2 = plot(R,[3,4],'FaceColor',[0 1 0],'EdgeColor','none');
% alpha(handleVeh1,0.5);
handleVeh1 = plot(R,[1,2],'FaceColor',[0 0 1],'EdgeColor','none');
% alpha(handleVeh1,0.5);

% plot initial set
plot(R0,[3,4],'w','Filled',true,'EdgeColor','k');
plot(R0,[1,2],'w','Filled',true,'EdgeColor','k');
% label plot
xlabel('velocity (m/s)');
ylabel('position (m)');
legend([handleVeh1,handleVeh2],'vehicle 1','vehicle 2');
title()

%--- (TBD) visualize vehicle states over time

% example completed
completed = 1;


%------------- END OF CODE --------------
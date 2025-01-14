function verboseLog(step,t,options)
% verboseLog - standardized console output if options.verbose = true,
%    only used in reach-functions
%
% Syntax:  
%    verboseLog(step,t,options)
%
% Inputs:
%    step - current step
%    t - start time of current step
%    options - options struct containing fields:
%                   'verbose': (true/false)
%                   'tStart': start time (for start message)
%                   'tFinal': time horizon (for end message)
%
% Outputs:
%    - (to console)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: none

% Author:        Mark Wetzlinger
% Written:       05-March-2021
% Last update:   ---
% Last revision: ---

%------------- BEGIN CODE --------------

if options.verbose
    
    % start message
    if abs(t - options.tStart) < 1e-12
        disp(newline + "Start analysis...");
        disp("- step 1: " + t);
        return;
    end
    
    % end message
    if abs(t - options.tFinal) < 1e-12
        disp("...time horizon reached, analysis finished." + newline);
        return;
    end
	
    % every ... steps, information is logged
    cycle = 10;
    % shift by one to obtain clean start times
    if mod(step-1,cycle) == 0
        disp("- step " + step + ": " + t);
    end
    
end

end


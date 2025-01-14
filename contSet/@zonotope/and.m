function Z = and(Z1,Z2,varargin)
% and - overloads & operator, computes the intersection of two zonotopes
%
% Syntax:  
%    Z = and(Z1,Z2)
%    Z = and(Z1,Z2,method)
%
% Inputs:
%    Z1 - zonotope object
%    Z2 - zonotope-, halfspace-, or constrained hyperplane object
%    method - (optional) algorithm used to compute the intersection
%               - 'conZonotope' (default)
%               - 'averaging'
%
% Outputs:
%    Z - zonotope object enclosing the intersection 
%
% Example: 
%    zono1 = zonotope([4 2 2;1 2 0]);
%    zono2 = zonotope([3 1 -1 1;3 1 2 0]);
%
%    res = zono1 & zono2
%
%    figure; hold on;
%    plot(zono1,[1,2],'r');
%    plot(zono2,[1,2],'b');
%    plot(res,[1,2],'g');
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: conPolyZono/and

% Author:        Matthias Althoff, Niklas Kochdumper, Amr Alanwar
% Written:       29-June-2009
% Last update:   02-Sep-2019 (rename intersection -> and)
%                11-Nov-2019 (NK: added algorithm for general case)
%                12-Feb-2020 (Amr: adding averaging option)
% Last revision: ---

%------------- BEGIN CODE --------------

    % parse input arguments
    method = 'conZonotope';
    
    if nargin == 3
        method = varargin{1};
        if ~ismember(method,'conZonotope','averaging')
           error('Wrong value for input argument "method"!'); 
        end
    end
    
    % quick check: simpler function for intervals
    if isInterval(Z1) && isInterval(Z2)
        % conversion to intervals exact
        Z = zonotope(interval(Z1) & interval(Z2)); return
    end

    % call special algorithm if the two sets are two paralleloptoes with
    % the same center
    if isa(Z2,'levelSet') || isa(Z2,'conZonotope') || ...
       isa(Z2,'zonoBundle') || isa(Z2,'conPolyZono')
        
        Z = Z2 & Z1;
        
    elseif strcmp(method,'conZonotope')
        
        % convert sets to constrained zonotopes
        Z1 = conZonotope(Z1);
        
        if ~isa(Z2,'halfspace') && ~isa(Z2,'conHyperplane')
            Z2 = conZonotope(Z2);
        end
        
        % compute intersection
        Z = Z1 & Z2;
        
        % conclose resulting constrained zonotope by a zonotope
        Z = zonotope(Z);
    
    elseif strcmp(method,'averaging')
        
        zonol{1} = Z1;
        zonol{2} = Z2;
        Z = andAveraging(zonol);
        
    else
        % throw error for given arguments
        error(noops(Z1,Z2));
    end
end

%------------- END OF CODE --------------
function E = andHalfspace(E,h,mode)
% andHalfspace - Computes the inner or outer approximation of the
% intersection between an ellipsoid and a halfspace
%
% Syntax:  
%    E = andHalfspace(E,h)
%    E = andHalfspace(E,h,mode)
%
% Inputs:
%    E      - ellipsoid object
%    h      - halfspace object
%    mode   - mode ('i':inner approx; 'o': outer approx)
%
% Outputs:
%    E - ellipsoid after intersection
%
% References:
%   [1] Kurzhanskiy, A.A. and Varaiya, P., 2006, December. Ellipsoidal toolbox (ET).
% In Proceedings of the 45th IEEE Conference on Decision and Control (pp. 1498-1503). IEEE.
%
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: -

% Author:       Victor Gassmann
% Written:      10-March-2021
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------
if ~exist('mode','var')
    mode = 'o';
end
if ~any(mode==['i','o'])
   error('mode has to be either "i" (inner approx) or "o" (outer approx)');
end
%check if same dimension
if length(E.q)~=length(h.c)
    error('Ellipsoid and halfspace have to have matching dimensions');
end

% compute distance to hyperplane defined by halfspace
hyp = conHyperplane(h.c,h.d);
d = distance(E,hyp);

if d>=-E.TOL
    % touching, completely inside or outside
    if h.c'*E.q>h.d
        % completely outside or touching
        if withinTol(d,0,E.TOL)
            % touching
            % point on hyperplane
            xh = h.c*h.d;
            % direction resulting in touching point has positive 
            % inner product with xh
            v = sign((xh-E.q)'*h.c)*h.c;
            % compute touching point
            [~,x] = supportFunc(E,v);
            E = ellipsoid(zeros(E.dim),x);
        else 
            E = ellipsoid;
        end
    else
        % E completely inside (or touching)
        % E = E;
    end
    return;
end
% they are intersecting

n = dim(E);
T = eye(n);
x_rem = [];

if E.isdegenerate
    nt = rank(E);
    % check if E.Q all zero
    if nt==0
        % check if E in halfspace
        if in(h,E.q)
            E = ellipsoid(zeros(n),E.q);
        else
            E = ellipsoid;
        end
        return;
    end
    [T,~,~] = svd(E.Q);
    E = T'*E;
    % transform halfspace (possible since T invertible)
    h = halfspace(T'*h.c,h.d);
    % project
    x_rem = E.q(nt+1:end);
    E = project(E,1:nt);
    h = halfspace(h.c(1:nt),h.d-h.c(nt+1:end)'*x_rem);
end

n_nd = dim(E);
% normalize halfspace
% shift E and h such that h.d = 0 and transform such that c=e_1
c = h.c/norm(h.c); d = h.d/norm(h.c);
% compute transformation matrix so that I(:,1) = S*c;
I = eye(n_nd);
S = vecalign(I(:,1),c);
h = halfspace(I(:,1),0);
E = -d*I(:,1) + S*E;

n_rem = n-n_nd;
% now non-degenerate
W1 = inv(E.Q);
q1 = E.q;
[r_s,x] = supportFunc(ellipsoid(E.Q),I(:,1));

% that is alternative; but only really works for outer approximations
% place center in the middle
% r_s = r_s-E_nd.q(1);
% q2 = [-1/2*r_s;zeros(n_nd-1,1)];
% W2 = diag([1/abs(1/2*r_s)^2;zeros(n_nd-1,1)]);

% that is "ellipsoidal toolbox original"
lb = max(eig(E.Q));
q2 = -2*sqrt(lb)*h.c;
W2 = 1/(4*lb)*h.c*h.c';

if strcmp(mode,'o')
    p = compIntersectionParam(W1,q1,W2,q2);
    [~,Q_nd,q_nd] = rootfnc(p,W1,q1,W2,q2);
else
    % intersect hyperplane defined by halfspace with E_nd
    E_hyp = E & conHyperplane(h);
    x = -x+E.q;
    b1 = (q1-E_hyp.q)'*W1*(q1-E_hyp.q);
    b2 = (q2-x)'*W2*(q2-x);
    t1 = (1-b2)/(1-b1*b2);
    t2 = (1-b1)/(1-b1*b2);
    q_nd = inv(t1*W1+t2*W2)*(t1*W1*q1+t2*W2*q2);
    W = t1*W1+t2*W2;
    Q_nd = (1-t1*q1'*W1*q1-t2*q2'*W2*q2+q_nd'*W*q_nd)*inv(W);
end
if any(eig(Q_nd)<0)
    error('Something went wrong...');
else
    E_nd = ellipsoid(Q_nd,q_nd);
    % revert S transform + shift
    Et = S'*E_nd + d*c;
end

% restore original dimensions & backtransform
E_t = ellipsoid([Et.Q,zeros(n_nd,n_rem);zeros(n_rem,n)],[Et.q;x_rem]);
E = T*E_t;
%------------- END OF CODE --------------
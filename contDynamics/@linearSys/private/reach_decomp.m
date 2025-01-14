function [Rout,Rout_tp,res] = reach_decomp(obj,options)
% reach_decomp - implementation of decomposed approach for reachability
% analysis of linear systems, cf. [1]
%
% Syntax:  
%    [Rout,Rout_tp,res] = reach_decomp(obj,options)
%
% Inputs:
%    obj     - continuous system object
%    options - options for the computation of reachable sets
%
% Outputs:
%    Rout    - output set of time intervals
%    Rout_tp - output set of time points
%    res     - boolean (only if specification given)
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: none
%
% References: 
%   [1] S. Bogomolov, M. Forets, G. Frehse, A. Podelski, C. Schlling
%       "Decomposing Reach Set Computations with Low-dimensional Sets
%            and High-Dimensional Matrices"

% Author:       Mark Wetzlinger
% Written:      11-June-2019
% Last update:  14-Aug-2019
% Last revision:---

%------------- BEGIN CODE --------------


% blocks and output matrix ------------------------------------------------
options.blocks = length(options.partition);
if length(obj.A) ~= 1 && length(obj.C) == 1
    if obj.C == 1
        % output = states
        obj.C = speye(obj.dim);
    end
end
% -------------------------------------------------------------------------


% initialize reachable set computations -----------------------------------
% log information
verboseLog(1,options.tStart,options);
% init reach step
[Yhat0, options] = initReach_Decomp(obj, options.R0, options);
% quick exit if violation already
if isfield(options,'specification')
    % check safety property (only time interval)
    for b=1:options.blocks
        if options.Cno0(b)
            if ~check(options.specification,Yhat0.ti{b})
                Rout    = Yhat0.ti;
                Rout_tp = Yhat0.tp;
                res     = false;
                return
            end
        end
    end
end
% -------------------------------------------------------------------------


% quick access ------------------------------------------------------------
Rtrans   = options.Rtrans;
Raux     = options.Raux;
Yinhom   = options.Yinhom;
Rinhom   = options.Rinhom;
Rhom0    = options.Rhom;
Rhom_tp0 = options.Rhom_tp;
Cno0     = options.Cno0;
C        = options.Cblock;
% -------------------------------------------------------------------------


% init time related terms -------------------------------------------------
eAt   = obj.taylor.eAt; % used every step for update of P and Q
P     = speye(obj.dim);
Q     = obj.taylor.eAt;
tVec  = options.tStart:options.timeStep:options.tFinal;
% -------------------------------------------------------------------------


% init some variables -----------------------------------------------------
Rout       = cell(length(tVec)-1,1);
Rout{1}    = Yhat0.ti;
Rout_tp    = cell(length(tVec)-1,1);
Rout_tp{1} = Yhat0.tp;
Yhatk      = cell(options.blocks,1);
Yhatk_tp   = cell(options.blocks,1);
% -------------------------------------------------------------------------


tic;
% loop over all further time steps of reachability analysis ---------------
for k=2:length(tVec)-1
    
    % log information
    verboseLog(k,tVec(k),options);
    
    % compute next reachable set
    for bi=1:options.blocks
        
        if Cno0(bi) 
            s_i = options.partition(bi,1);
            f_i = options.partition(bi,2);
            Ytemp = 0;
            Ytemp_tp = 0;
            for bj=1:options.blocks
                s_j = options.partition(bj,1);
                f_j = options.partition(bj,2);
                Ytemp = Ytemp + (C{bi} * Q(s_i:f_i,s_j:f_j)) * Rhom0{bj};
                Ytemp_tp = Ytemp_tp + (C{bi} * Q(s_i:f_i,s_j:f_j)) * Rhom_tp0{bj};
            end
            Yhatk{bi} = Ytemp + Yinhom{bi};
            Yhatk_tp{bi} = Ytemp_tp + Yinhom{bi};
            
        end
    end
    
    % write to return variables
    Rout{k} = Yhatk;
    Rout_tp{k} = Yhatk_tp;
    
    if isfield(options,'specification')
        % check safety property (only time interval)
        for b=1:options.blocks
            if options.Cno0(b)
                if ~check(options.specification,Rout{k}{b})
                    Rout    = Rout(1:k);
                    Rout_tp = Rout_tp(1:k);
                    res     = false;
                    return
                end
            end
        end
    end
    
    
    % update expm
    P = Q; % ... propagation of Rtrans
    Q = Q * eAt; % ... propagation of Raux and next Rhom0
    
    
    % propagation of Rinhom
    % note: for now, input set constant V(k) = V(k+1) = V (no uTransVec)
    if options.isInhom
        for bi=1:options.blocks
            s_i = options.partition(bi,1);
            f_i = options.partition(bi,2);
            
            if Cno0(bi)
                Rinhomtemp = Rinhom{bi};
                Yinhomtemp = Yinhom{bi};

                % change representation if suitable
                if issparse(Rinhom{bi}.Z) && nnz(Rinhom{bi}.Z(:,2:end)) / numel(generators(Rinhom{bi})) > 0.5
                    % change to full representation
                    Rinhom{bi} = zonotope([full(center(Rinhom{bi})), full(generators(Rinhom{bi}))]);
                    Rtrans{bi} = zonotope([full(center(Rtrans{bi})), full(generators(Rtrans{bi}))]);
                    Raux{bi}   = zonotope([full(center(Raux{bi})), full(generators(Raux{bi}))]);
                end
                
            
                for bj=1:options.blocks
                    s_j = options.partition(bj,1);
                    f_j = options.partition(bj,2);
                    eAtRtrans = P(s_i:f_i,s_j:f_j);
                    eAtRaux   = Q(s_i:f_i,s_j:f_j);

                    if ~(~nnz(eAtRtrans) && ~nnz(eAtRaux))
                        % only if prop matrices not all-zeros
                        Yinhomtemp = Yinhomtemp + (C{bi} * eAtRtrans) * Rtrans{bj} + ...
                            (C{bi} * eAtRaux) * Raux{bj};
                        Rinhomtemp = Rinhomtemp + eAtRtrans * Rtrans{bj} + ...
                            eAtRaux * Raux{bj};
                    end
                end
                
                Rinhom{bi} = Rinhomtemp;
                Yinhom{bi} = Yinhomtemp;
                % order reduction
                Rinhom{bi} = reduce(Rinhom{bi},options.reductionTechnique,options.zonotopeOrder);
                if size(C{bi},1) == 1
                    % manual order reduction if only one output dimension
                    temp_YinhomsZ = interval(Yinhom{bi});
                    YinhomsZ_only_generator = center(Yinhom{bi}) - supremum(temp_YinhomsZ);
                    Yinhom{bi} = zonotope([center(Yinhom{bi}), YinhomsZ_only_generator]);
                else
                    Yinhom{bi} = reduce(Yinhom{bi},options.reductionTechnique,options.zonotopeOrder);
                end
            end
            
        end
    end
    
end
% -------------------------------------------------------------------------

% log information
verboseLog(length(tVec),tVec(end),options);

% no violation of specification
res = true;


end


%------------- END OF CODE --------------




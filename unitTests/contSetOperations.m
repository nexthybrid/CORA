function setOps = contSetOperations(varargin)
% isContSetFunction - check which contSet classes have implemented
%    certain functions (basic function in manual)
%
% Syntax:
%    setOps = contSetOperations
%    setOps = contSetOperations(contSetclass)
%    setOps = contSetOperations(contSetclass,type)
%
% Inputs:
%    contSetclass - (optional) class for which functions and unit tests
%           should be printed to console
%    type - (options) 'basic' (default), 'all' (all functions)
%
% Outputs:
%    setOps - struct containing four fields
%               'basicSetOps': basic set operations (manual 2.1.1)
%               'predicates': predicates (manual 2.1.2)
%               'setProps': set properties (manual 2.1.3)
%               'auxOps': auxiliary operations (manual 2.1.4)
%             each of which contains 'func'/'exists'/'unitTest' fields, where
%               'func': cell-array of function names in group
%               'exists': logical array of implemented functions
%               'unitTest': logical array of implemented unit tests
%
% Example: 
%    -

% Author:       Mark Wetzlinger
% Written:      09-March-2021
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------

% groups in manual
groups = {'basicSetOps','predicates','setProps','auxOps'};
% list function specified in manual
basicSetOps = {'mtimes','plus','cartProd','convHull','quadMap','and','or'}';
predicates = {'in','isIntersecting','isFullDim','isequal','isempty'}';
setProps = {'center','dim','norm','vertices','volume'}';
auxOps = {'cubMap','enclose','enclosePoints','generateRandom','linComb',...
    'randPoint','reduce','supportFunc','plot','project'}';

% content of contSet folder
allContSet = listFolderContent([coraroot filesep 'contSet']);
% content of unitTests/contSet folder
allunitTestsContSet = listFolderContent([coraroot filesep 'unitTests' filesep 'contSet']);

% list classes in contSet
nrClasses = length(allContSet.files);
classes = cell(nrClasses,1);
for i=1:nrClasses
    fullclasspath = allContSet.files{i}.dir;
    idxAtSign = strfind(fullclasspath,'@');
    classes{i} = fullclasspath(idxAtSign+1:end);
end

% list classes in unitTests/contSet
nrClasses_uT = length(allunitTestsContSet.files);
classes_uT = cell(nrClasses_uT,1);
for i=1:nrClasses_uT
    fullclasspath = allunitTestsContSet.files{i}.dir;
    idxLastBackslash = max(strfind(fullclasspath,'\'));
    classes_uT{i} = fullclasspath(idxLastBackslash+1:end);
end
% --- loading of CORA status finished ---


% check if user-provided class exists
contSetclass = [];
if nargin >= 1
    if ~ischar(varargin{1})
        error("contSet class name must be a char array!");
    elseif ~ismember(varargin{1},classes)
        error("Provided contSet class does not exist!");
    end
    contSetclass = varargin{1};
    type = 'basic';
    if nargin == 2
        if ~ischar(varargin{2})
            error("type specification name must be a char array!");
        elseif ~ismember(varargin{2},{'basic','all'})
            error("type specification name must be 'basic' or 'all'!");
        end
        type = varargin{2};
    end
end


% struct containing all groups + funcs
setOps = struct();
longestName = 0;
for i=1:length(groups)
    setOps.(groups{i}).func = eval(groups{i});
    setOps.(groups{i}).exists = false(nrClasses,length(setOps.(groups{i}).func));
    setOps.(groups{i}).unitTest = false(nrClasses,length(setOps.(groups{i}).func));
    longestName = max([longestName;strlength(setOps.(groups{i}).func)]);
end
% struct for left-over set operations / unit tests
leftovers.func = cell(nrClasses,1);
leftovers.unitTest = cell(nrClasses,1);


% check if functions are implemented for respective classes
for j=1:nrClasses
    funcsInClass = allContSet.files{j}.files;
    % check which set operations have unit tests
    idx = strcmp(classes{j},classes_uT);
    unitTestsForClass = {};
    if any(idx)
        unitTestsForClass = allunitTestsContSet.files{idx}.files;
        % pre-process unit test function names
        for t=1:length(unitTestsForClass)
            if ~ischar(unitTestsForClass{t}) || ...
                    ~contains(unitTestsForClass{t},classes{j})
                % delete entry if not a unit test of current class
                unitTestsForClass{t} = '';
            else
                % extract function name of unit test
                idxLastUnderline = max(strfind(unitTestsForClass{t},'_'));
                unitTestsForClass{t} = unitTestsForClass{t}(idxLastUnderline+1:end);
            end
        end
    end
        
    for i=1:length(groups)
        % check which set operations are implemented
        [setOps.(groups{i}).exists(j,:),setOpsIdx] = ismember(setOps.(groups{i}).func,funcsInClass); 
        % delete found set operations from list
        setOpsIdx(setOpsIdx == 0) = [];
        funcsInClass(setOpsIdx) = [];
        % check for which set operations there is a unit test
        [setOps.(groups{i}).unitTest(j,:),setOpsUnitTestsIdx] = ...
            ismember(setOps.(groups{i}).func,unitTestsForClass);
        % delete found set operations from list
        setOpsUnitTestsIdx(setOpsUnitTestsIdx == 0) = [];
        unitTestsForClass(setOpsUnitTestsIdx) = [];
    end
    
    % analyze left-over set operations and unit tests
    leftovers.func(j) = {funcsInClass};
    if any(idx)
        leftovers.unitTest(j) = {ismember(funcsInClass,unitTestsForClass)};
    else
        leftovers.unitTest(j) = {false(length(funcsInClass),1)};
    end
end


% skip to end if no class should be printed
if isempty(contSetclass)
    return;
end


% print information about given class -------------------------------------
classIdx = ismember(classes,contSetclass);

% print header
fprintf('-*---------------------------------*-\n');
fprintf(['--- Set Operations for ''' contSetclass ''' ---\n']);
fprintf('(implemented: function / unit test)\n\n');

headers = {'basic set operations','predicates',...
    'set properties','auxiliary operations'};

% adapt longest name
if strcmp(type,'all')
    remfuncs = leftovers.func(classIdx);
    remfuncs = remfuncs{1};
    if ~isempty(remfuncs)
        longestName = max([longestName;strlength(remfuncs)]);
    end
end

% implemented functions / unit tests for groups
for i=1:length(groups)
    fprintf([headers{i} '\n']);
    currfuncs = setOps.(groups{i}).func;
    charlengthdiff = longestName - strlength(currfuncs);
    for j=1:length(currfuncs)
        txtExists = 'no  ';
        if setOps.(groups{i}).exists(classIdx,j)
            txtExists = 'yes ';
        end
        txtUnitTest = 'no';
        if setOps.(groups{i}).unitTest(classIdx,j)
            txtUnitTest = 'yes';
        end
        fprintf(['.. ' currfuncs{j} ': ' ...
            repmat(' ',1,charlengthdiff(j)) txtExists '/ ' txtUnitTest '\n']);
    end
end

% additional group: all other implemented functions / corresponding unit tests
if strcmp(type,'all') && ~isempty(remfuncs)
    fprintf('remaining operations \n');
    
    charlengthdiff = longestName - strlength(remfuncs);
    for j=1:length(remfuncs)
        txtUnitTest = 'no';
        if leftovers.unitTest{classIdx}(j)
            txtUnitTest = 'yes';
        end
        fprintf(['.. ' remfuncs{j} ': ' ...
            repmat(' ',1,charlengthdiff(j)) 'yes / ' txtUnitTest '\n']);
    end
    
end

% print footer
fprintf('-*---------------------------------*-\n');

end

%------------- END OF CODE --------------
% Copyright 2020 The MathWorks, Inc.

disp('Begin call Model Advisor analysis')

% Move to working directory
prj = matlab.project.currentProject;
cd(prj.ProjectStartupFolder);

% Setup and open model
TopModel = 'LaneFollowingTestBenchExample';
load_system(TopModel);

% Create Model Advisor app
app = Advisor.Manager.createApplication();

% Set root for analysis.
setAnalysisRoot(app,'Root', TopModel);

% Clear all check instances from Model Advisor analysis.
deselectCheckInstances(app);

% Select check Identify unconnected lines, input ports, and output ports
instanceID = getCheckInstanceIDs(app, 'mathworks.design.UnconnectedLinesPorts');
checkinstanceID = instanceID(1);
selectCheckInstances(app, 'IDs', checkinstanceID);

%Run Model Advisor analysis.
run(app);

%Get analysis results.
result = getResults(app);

% Ensure that there are no unconnected lines
assert(~any([result.numFail]), 'Unconnected lines found in LaneFollowingTestBenchExample or referenced models');

%Generate and view the Model Advisor report. The Model Advisor runs the check on both mdlref_basic and mdlref_counter.
report = generateReport(app);

disp('End call Model Advisor analysis')
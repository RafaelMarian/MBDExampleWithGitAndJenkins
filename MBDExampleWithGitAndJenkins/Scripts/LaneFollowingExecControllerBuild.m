% Copyright 2020 The MathWorks, Inc.

disp('Begin controller build')

% Move to working directory
prj = matlab.project.currentProject;
cd(prj.ProjectStartupFolder);

% Build setup
mdl = 'LFRefMdl';
evalin('base', 'scenarioId = 1');
evalin('base', 'helperLFSetUp');
load_system(mdl);

% Build
slbuild(mdl);

disp('End controller build')
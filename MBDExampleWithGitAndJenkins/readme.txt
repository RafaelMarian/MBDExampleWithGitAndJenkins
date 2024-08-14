Copyright 2020 The MathWorks, Inc.

The project is composed of the following folders:

Data: Test case input data

Models: Simulink models used to model lane change logic

Requirements: Requirements for the lane change controller

Scripts: Scripts used to setup models, tests, and visualizations

Tests: Test cases we will execute as part of our CI pipeline

The Tests folder defines our pipeline and consists of three files.
They are, in order of execution:
1. LaneFollowingModelAdvisorChecks.m: Simulink Check tests
2. LaneFollowingControllerBuild.m: SIL code generation
3. LaneFollowingTestScenarios.mldatx: Simulink Test Test Cases
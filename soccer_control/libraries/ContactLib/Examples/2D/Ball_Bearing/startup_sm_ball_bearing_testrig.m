% Copyright 2018-2019 The MathWorks, Inc.

addpath(pwd)
addpath([pwd filesep 'Libraries']);
addpath([pwd filesep 'Scripts_Data']);
addpath([pwd filesep 'Images']);
addpath([pwd filesep 'Animations']);

% Documentation excluded for zip file on File Exchange
if(exist([pwd filesep 'html' filesep 'html'],'dir'))
    addpath([pwd filesep 'html' filesep 'html']);
end

sm_ball_bearing_testrig_param
sm_ball_bearing_testrig

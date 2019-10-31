% Startup for Simscape Multibody Contact Force Library
% Copyright 2014-2017 The MathWorks, Inc.

CFR_HomeDir = pwd;
addpath(genpath(pwd));

CFL_libname = 'Contact_Forces_Lib';
load_system(CFL_libname);
CFL_ver = get_param(CFL_libname,'Description');
disp(CFL_ver);
which(CFL_libname)

tempNumCFLLibs = which(CFL_libname,'-all');
if (length(tempNumCFLLibs)>1)
    tempCFLwarnStr = sprintf('Multiple copies of the Simscape Multibody Contact Force Library are on the MATLAB path. It is recommended you only have one copy on your path. Please consider adjusting your MATLAB path so that only one copy is present.\n\nLocations of Simscape Multibody Contact Force Library on your path:\n');
    tempCFLLibStr = sprintf('%s\n',tempNumCFLLibs{:});
    warning([tempCFLwarnStr tempCFLLibStr])
end
clear tempNumCFLLibs tempCFLwarnStr

if(exist('Cam_Follower.slx')==4)
    Cam_Follower
    web('Contact_Forces_Demo_Script.html');
end


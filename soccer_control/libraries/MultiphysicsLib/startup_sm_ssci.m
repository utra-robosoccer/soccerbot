% Copyright 2016-2017 The MathWorks, Inc.

SCI_HomeDir = pwd;
addpath(pwd)
addpath(genpath(pwd))

cd Libraries
if((exist('+forcesPS')==7) && ~exist('forcesPS_Lib'))
    ssc_build forcesPS
end
cd ..

if exist('Multibody_Multiphysics_Demo_Script.html')
    web('Multibody_Multiphysics_Demo_Script.html');
end
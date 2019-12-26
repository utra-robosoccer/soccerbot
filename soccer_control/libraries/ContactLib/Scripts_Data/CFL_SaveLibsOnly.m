% Code to create copy of Contact Force Library
% that excludes all examples and testing files
% Resulting folder CFL_Copy can be copy/pasted
% into other projects.
%
% Copyright 2014-2019 The MathWorks, Inc.

folder_list = {...
    'Libraries',...
    'Scripts_Data'};

file_list = {...
    'README.txt',...
    'startup_Contact_Forces.m'};

copyFolderName = 'CFL_Libs';
mkdir(copyFolderName)

for i= 1:length(folder_list)
    copyfile(folder_list{i},[copyFolderName filesep folder_list{i}]);
end

for i= 1:length(file_list)
    copyfile(file_list{i},[copyFolderName filesep file_list{i}]);
end

% Move to folder
cd(copyFolderName)

% Delete files
deleteFileList = {...
    ['Scripts_Data' filesep 'CFR_Results_REF.xlsx'],
    ['Scripts_Data' filesep 'CFR_Results.xlsx'],
    ['Scripts_Data' filesep 'Test_All_CFR_Models.m'],
    ['Scripts_Data' filesep 'CFL_SaveFEXCopy.m'],
    ['Scripts_Data' filesep 'CFL_SaveLibsOnly.m'],
    ['Scripts_Data' filesep 'CheckAnim.m'],
    ['Scripts_Data' filesep 'CheckHTMLDoc.m'],
    ['Scripts_Data' filesep 'temp_setvispar.m']
    ['Libraries' filesep 'Images' filesep 'Link_End_nx_IMAGE.png']
    ['Libraries' filesep 'Images' filesep 'Link_End_px_IMAGE.png']
    ['Libraries' filesep 'Images' filesep 'Link_Seg_2_Hole_IMAGE.png']
    ['Libraries' filesep 'Images' filesep 'Rod_nZ_IMAGE.png']
    ['Libraries' filesep 'Images' filesep 'Rod_pZ_IMAGE.png']
    ['Libraries' filesep 'Images' filesep 'Triangle_Link_IMAGE.png']
    };

for i=1:length(deleteFileList)
    delstr = [pwd filesep deleteFileList{i}];
    disp(delstr);
    delete(delstr);
end

%% Delete PowerPoint files

if(strcmp(pwd,['C:' filesep 'SMILLER' filesep 'TMW' filesep 'Simscape' filesep 'Demos' filesep 'sscMbody' filesep 'Contact_Forces']))
    error('Stop delete -- in source directory')
end

pptxFullList = dir('**/*.pptx');
for ppt_i=1:length(pptxFullList)
    delstr = [pptxFullList(ppt_i).folder '' filesep '' pptxFullList(ppt_i).name]; 
    disp(delstr);
    delete(delstr)
end



% Copyright 2014-2017 The MathWorks, Inc.

ds_list = dir('*Contact*.m');
for ds_i = 1:length(ds_list);
    publish(ds_list(ds_i).name);
    HTMLfilename = strrep(ds_list(ds_i).name,'.m','.html');
    movefile(['./html/' HTMLfilename],['./' HTMLfilename]);
end
rmdir('html');
clear ds_list ds_i HTMLfilename

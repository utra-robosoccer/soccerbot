function CheckAnim(animation_file)
% Verify if animation file exists before attempting to display 
% the animation in the MATLAB web browser.

% Copyright 2014-2019 The MathWorks, Inc.

if (exist(animation_file))
    % If exists, display in web browser
    web(animation_file)
else
    % If does not exist, display warning dialog
    % Some animations GIFs are too large to include in zip file for sharing
    warndlg(sprintf('%s\n%s','The animation file for this example was not included',... 
        'to reduce the size of the zip file.'),'Animation Not Included');
end

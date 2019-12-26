function CheckHTMLDoc(HTMLDoc_file)
% Verify if documentation for the example exists before attempting to
% display the animation in the MATLAB web browser.

% Copyright 2014-2019 The MathWorks, Inc.

if (exist(HTMLDoc_file))
    % If exists, display in web browser
    web(HTMLDoc_file)
else
    % If does not exist, display warning dialog
    warndlg(sprintf('%s\n%s','The documentation for this example was not included',... 
        'to reduce the size of the zip file.'),'Documentation Not Included');
end

function convertVideo(varargin)

%

function convertVideo()

% Converts 'rawVideo_640x480.avi'
to either
320x240 or 180x120 and/or
%
extracts a
section of
the video
.
%
%
This function
takes 1 or 2 inputs.
The first
input is
the resolution
. The
%
second input
is the
desired time
range from
the video
file;
%
convertVideo(resolution)
- converts 'rawVideo_640x480.avi' to
% the "resolution"
specified and
outputs it
as a
file called
% "rawVideo_resolution.avi"
%
%
The resolution
can be
one of
the 3 strings '640x480', '320x240', '180x120'
%
%
convertVideo(resolution, trange
) - converts 'rawVideo_640x480.avi' to
% the "resolution"
specified and
selects only
the video
corresponding to
%
the time
range "trange".
It is
labeled rawVideo_resolution_tbegin_tend
%
% Examples
% convertVideo('320x240') -
converts rawVideo_640x480
.
avi to
% 320x240
resolution and
calls it
'rawVideo_320x240.avi'
% convertVideo('640x480',[10 40]) -
extracts the
30
second video
%
corresponding to
the begin
and end
times 10 and 40
and calls
it
% rawVideo_640x480_10_40

if
isequal(nargin,
1)

resolution = varargin { 1 };

elseif isequal(nargin,

2)

resolution = varargin { 1 };

trange = varargin { 2 };
end

% Copyright 2013,
The MathWorks, Inc
.

%
define the
output name
of the
file and
the number
of output
rows and
% columns
if
strcmp(resolution,
'640x480')
outputName = 'rawVideo_640x480';
rows = 480;
cols = 640;

elseif strcmp(resolution,

'320x240')
outputName = 'rawVideo_320x240';
rows = 240;
cols = 320;

elseif strcmp(resolution,

'180x180')
outputName = 'rawVideo_180x120';
rows = 120;
cols = 180;
end

%
append the
time range
if specified
if
isequal(nargin,
2)
outputName = [outputName, '_', num2str(trange(1)), '_', num2str(trange(2))];
end

%
define video
objects
        videoReaderObject = VideoReader('2.mp4');
videoWriterObject = VideoWriter(outputName);

%
configure videoWriterObject
videoWriterObject.
FrameRate = 10;

%
open videoWriterObject
open(videoWriterObject);

%
find size
of raw
video
        nidx = get(videoReaderObject, 'NumberOfFrames');

%
loop through
frames and
convert video
if
isequal(nargin,
1)
ilow = 1;
ihigh = nidx;

elseif isequal(nargin,

2)
ilow = trange(1) * 10;
ihigh = trange(2) * 10;
end
for
idx = ilow
:
ihigh
        frame = read(videoReaderObject, idx);
frame = imresize(frame, [rows cols]);
writeVideo(videoWriterObject, frame
);
end

%
close videoWriterObject
close(videoWriterObject);
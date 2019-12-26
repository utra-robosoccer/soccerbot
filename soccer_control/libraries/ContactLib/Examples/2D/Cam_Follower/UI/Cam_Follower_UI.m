function varargout = Cam_Follower_UI(varargin)
% CAM_FOLLOWER_UI MATLAB code for Cam_Follower_UI.fig
%      CAM_FOLLOWER_UI, by itself, creates a new CAM_FOLLOWER_UI or raises the existing
%      singleton*.
%
%      H = CAM_FOLLOWER_UI returns the handle to a new CAM_FOLLOWER_UI or the handle to
%      the existing singleton*.
%
%      CAM_FOLLOWER_UI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CAM_FOLLOWER_UI.M with the given input arguments.
%
%      CAM_FOLLOWER_UI('Property','Value',...) creates a new CAM_FOLLOWER_UI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Cam_Follower_UI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Cam_Follower_UI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Copyright 2014-2019 The MathWorks, Inc.

% Edit the above text to modify the response to help Cam_Follower_UI

% Last Modified by GUIDE v2.5 14-Jul-2014 13:56:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Cam_Follower_UI_OpeningFcn, ...
                   'gui_OutputFcn',  @Cam_Follower_UI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before Cam_Follower_UI is made visible.
function Cam_Follower_UI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Cam_Follower_UI (see VARARGIN)

% Choose default command line output for Cam_Follower_UI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using Cam_Follower_UI.
if strcmp(get(hObject,'Visible'),'off')
    Plot_pushbutton_Callback(hObject,eventdata, handles);
end

% UIWAIT makes Cam_Follower_UI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Cam_Follower_UI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in Plot_pushbutton.
function Plot_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Plot_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
cla;

r_tip  = str2double(get(handles.Tip_Radius_edit, 'String'));
r_axis = str2double(get(handles.Axis_Radius_edit, 'String'));
l_cam  = str2double(get(handles.Length_edit, 'String'));

[xy_data]=Extr_Data_Cam_Circles(r_tip, r_axis, l_cam, 0, 0);
plot([xy_data(:,1);xy_data(1,1)],[xy_data(:,2);xy_data(1,2)])
t_h=get(gca,'Title');
set(t_h,'String','');
axis equal
    




% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});



function Tip_Radius_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Tip_Radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tip_Radius_edit as text
%        str2double(get(hObject,'String')) returns contents of Tip_Radius_edit as a double


% --- Executes during object creation, after setting all properties.
function Tip_Radius_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tip_Radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Axis_Radius_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Axis_Radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Axis_Radius_edit as text
%        str2double(get(hObject,'String')) returns contents of Axis_Radius_edit as a double


% --- Executes during object creation, after setting all properties.
function Axis_Radius_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Axis_Radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Height_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Height_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Height_text as text
%        str2double(get(hObject,'String')) returns contents of Height_text as a double


% --- Executes during object creation, after setting all properties.
function Height_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Height_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Update_pushbutton.
function Update_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Update_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

r_tip  = str2double(get(handles.Tip_Radius_edit, 'String'));
r_axis = str2double(get(handles.Axis_Radius_edit, 'String'));
l_cam  = str2double(get(handles.Length_edit, 'String'));
evalin('base',['Cam_PARAM.Cam.Radius_rev = ' num2str(r_axis) ';']);
evalin('base',['Cam_PARAM.Cam.Radius_tip = ' num2str(r_tip) ';']); % cm
evalin('base',['Cam_PARAM.Cam.Dist_ctrs = ' num2str(l_cam) ';']);  % cm
evalin('base',['Cam_PARAM.Cam.Theta1 = acos((Cam_PARAM.Cam.Radius_rev-Cam_PARAM.Cam.Radius_tip)/Cam_PARAM.Cam.Dist_ctrs)*180/pi;']);


set_param(bdroot,'SimulationCommand','Update')

function Length_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Length_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Length_edit as text
%        str2double(get(hObject,'String')) returns contents of Length_edit as a double


% --- Executes during object creation, after setting all properties.
function Length_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Length_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

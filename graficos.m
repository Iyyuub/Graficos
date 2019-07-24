function varargout = graficos(varargin)
% GRAFICOS MATLAB code for graficos.fig
%      GRAFICOS, by itself, creates a new GRAFICOS or raises the existing
%      singleton*.
%
%      H = GRAFICOS returns the handle to a new GRAFICOS or the handle to
%      the existing singleton*.
%
%      GRAFICOS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GRAFICOS.M with the given input arguments.
%
%      GRAFICOS('Property','Value',...) creates a new GRAFICOS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before graficos_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to graficos_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help graficos

% Last Modified by GUIDE v2.5 02-Jul-2019 18:58:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @graficos_OpeningFcn, ...
                   'gui_OutputFcn',  @graficos_OutputFcn, ...
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


% --- Executes just before graficos is made visible.
function graficos_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to graficos (see VARARGIN)
global CAMERAON
global RUN
CAMERAON=0;
RUN=0;
% Choose default command line output for graficos
handles.output = hObject;
load CalibrationData.mat
try
    rosinit('nfs11',11311);
catch,
    fprintf('Rosinit ja tinha corrido \n');
end
 topic_rgb = rossubscriber('/camera/rgb/image_color');%kinect 
%topic_rgb = rossubscriber('/camera/rgb/image_raw');%openni2 asus
topic_depth=rossubscriber('/camera/depth/image_rect');

data_rgb = receive(topic_rgb,10);
image_rgb = readImage(data_rgb);
data_depth = receive(topic_depth,10);
image_depth = readImage(data_depth);
%Copy rostopics to guidata
handles.topic_depth=topic_depth;
handles.topic_rgb=topic_rgb;
%Tracking variables

%OSC
try,
    fclose(handles.osc);
osc=udp('192.168.0.8',57120);
fopen(osc);
catch
    osc=udp('192.168.0.8',57120);
fopen(osc);
end

handles.osc=osc;
% Tamanho do rectangulo no chao
handles.chao=[-3.70 3.70 -3 3];
%Objects min size and max size (area) and mass(number pxls in the vertical)
handles.objminsize=10;
handles.objmaxsize=150;
handles.objmass=20;
handles.lowestobj=.5;% minimum height of an object
%SIZE_IM - size of ground plane (pixels)
handles.sizeIM=200;
% Começa com graficos on
handles.graphics_on=1;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes graficos wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = graficos_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%CALIBRATE CAMERA 1
global CAMERAON
CAMERAON=0;

fprintf('Vai calibrar')
calibra_cam1(hObject)
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% PUT CAMERAS 1 PASSTHRU
global CAMERAON
CAMERAON=1;
topic_rgb = handles.topic_rgb;
set(handles.figure1,'CurrentAxes',handles.axes1);
while CAMERAON
    data_rgb = receive(topic_rgb,10);
    image_rgb = readImage(data_rgb);
    imshow(imresize(image_rgb,[240 320]));
    drawnow;
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% RUN PROGRAM
global RUN
global CAMERAON
if RUN ==1,
    fprintf(' It is running already');
    return;
else,
RUN=1;
CAMERAON=0;
realtimego_graphics;
end
% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%STOP TRACKING
global RUN CAMERAON

CAMERAON=0;
RUN=0;


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%%TEST SOUND


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%GRAPHICS ON OFF
if handles.graphics_on,
    set(handles.pushbutton8,'String','Graphics off');
    handles.graphics_on=0;
    set(handles.pushbutton8,'BackgroundColor',[0 1 0]);
else
    set(handles.pushbutton8,'String','Graphics on');
    handles.graphics_on=1;
    set(handles.pushbutton8,'BackgroundColor',[0.5 0.5 0.5]);
end
guidata(hObject, handles);

function varargout = untitledGUI(varargin)
% UNTITLEDGUI MATLAB code for untitledGUI.fig
%      UNTITLEDGUI, by itself, creates a new UNTITLEDGUI or raises the existing
%      singleton*.
%
%      H = UNTITLEDGUI returns the handle to a new UNTITLEDGUI or the handle to
%      the existing singleton*.
%
%      UNTITLEDGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLEDGUI.M with the given input arguments.
%
%      UNTITLEDGUI('Property','Value',...) creates a new UNTITLEDGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitledGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitledGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitledGUI

% Last Modified by GUIDE v2.5 09-Nov-2023 15:23:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitledGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @untitledGUI_OutputFcn, ...
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
% --- Executes just before untitledGUI is made visible.
function untitledGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitledGUI (see VARARGIN)

% Choose default command line output for untitledGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using untitledGUI.
% if strcmp(get(hObject,'Visible'),'off')
%     plot(rand(5));
% end

% UIWAIT makes untitledGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);
cla
axes(handles.axes1);


% L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
% L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
% L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
% L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

L1 = Link('d',0,    'a',0,      'alpha',-pi/2,    'offset',0,     'qlim',[deg2rad(-270),deg2rad(270)]);
L2 = Link('d',0.180,    'a',0.6361,    'alpha',0,     'offset',0,     'qlim',[deg2rad(-180),deg2rad(180)]);
L3 = Link('d',-0.1297,     'a',0.5579,    'alpha',0,    'offset',0,     'qlim',[deg2rad(-166),deg2rad(166)]);
L4 = Link('d',0.106,     'a',0,'alpha',pi/2,     'offset',0,     'qlim',[deg2rad(-180),deg2rad(180)]);
L5 = Link('d',0.106,    'a',0,  'alpha',-pi/2,    'offset',0,     'qlim',[deg2rad(-180),deg2rad(180)]);
L6 = Link('d',0.11315,       'a',0,  'alpha',0,     'offset',0,     'qlim',[deg2rad(-270),deg2rad(270)]);



% link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
% link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);

model = SerialLink([L1 L2 L3 L4 L5 L6],'name','TM12');

for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>        
    model.faces{linkIndex+1} = faceData;
    model.points{linkIndex+1} = vertexData;
end

% Display robot
workspace = [-2 2 -2 2 -0.3 2];
model.plot(zeros(1,model.n),'noarrow','workspace',workspace);
% model.plot3d(zeros(1,model.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end  
model.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles,'UserData');
    try 
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                      , plyData{linkIndex+1}.vertex.green ...
                                                      , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end
    
data = guidata(hObject);
data.model = model;

guidata(hObject,data);


% --- Outputs from this function are returned to the command line.
function varargout = untitledGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
cla
axes(handles.axes1);



% L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
% L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
% L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
% L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

L1 = Link('d',0,    'a',0,      'alpha',-pi/2,    'offset',0,     'qlim',[deg2rad(-270),deg2rad(270)]);
L2 = Link('d',0.180,    'a',0.6361,    'alpha',0,     'offset',0,     'qlim',[deg2rad(-180),deg2rad(180)]);
L3 = Link('d',-0.1297,     'a',0.5579,    'alpha',0,    'offset',0,     'qlim',[deg2rad(-166),deg2rad(166)]);
L4 = Link('d',0.106,     'a',0,'alpha',pi/2,     'offset',0,     'qlim',[deg2rad(-180),deg2rad(180)]);
L5 = Link('d',0.106,    'a',0,  'alpha',-pi/2,    'offset',0,     'qlim',[deg2rad(-180),deg2rad(180)]);
L6 = Link('d',0.11315,       'a',0,  'alpha',0,     'offset',0,     'qlim',[deg2rad(-270),deg2rad(270)]);



% link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
% link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);

model = SerialLink([L1 L2 L3 L4 L5 L6],'name','TM12');

for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>        
    model.faces{linkIndex+1} = faceData;
    model.points{linkIndex+1} = vertexData;
end

% Display robot
workspace = [-2 2 -2 2 -0.3 2];
model.plot(zeros(1,model.n),'noarrow','workspace',workspace);
% model.plot3d(zeros(1,model.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end  
model.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles,'UserData');
    try 
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                      , plyData{linkIndex+1}.vertex.green ...
                                                      , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end
    
data = guidata(hObject);
data.model = model;
guidata(hObject,data);


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


% --- Executes on button press in plusX_pushbutton.
function plusX_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to plusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.but2Val == 0
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr.t(1) = tr.t(1) + 0.1;
%tr(1,4) = tr(1,4) + 0.01; <---- old line
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in minusX_pushbutton.
function minusX_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to minusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.but2Val == 0
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr.t(1) = tr.t(1) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in plusY_pushbutton.
function plusY_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to plusY_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr.t(3) = tr.t(3) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in minusY_pushbutton.
function minusY_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to minusY_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


if handles.but2Val == 0
    q = handles.model.getpos;
    tr = handles.model.fkine(q);
    tr.t(3) = tr.t(3) - 0.1;
    newQ = handles.model.ikcon(tr,q);
    handles.model.animate(newQ);
    pos = handles.model.fkine(q);
    disp(pos.t)
    else
    disp ('E-Stop Pressed')
end


% --- Executes on button press in plusZ_pushbutton.
function plusZ_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to plusZ_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr.t(2) = tr.t(2) + 0.1;
%tr(1,4) = tr(1,4) + 0.01; <---- old line
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in minusZ_pushbutton.
function minusZ_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to minusZ_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr.t(2) = tr.t(2) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% % % % % xminval=-pi/2;%set min value for x axis
% % % % % xmaxval=+pi/2;%set max value for x axis
% % % % % xsld_step = [0.1];%slider step
% % % % % axis([xminval xmaxval 0 1]);%set axis range for plot
% % % % % set(hObject,'Min',xminval);
% % % % % set(hObject,'Max',xmaxval);
% % % % % set(hObject, 'SliderStep', xsld_step);
% % % % % new_xmaxval=get(hObject,'Value')
% % % % % 
% % % % % 
% % % % % q = handles.model.getpos;
% % % % % q(1) = q(1) + 0.01




% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(1) = q(1) + 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(1) = q(1) - 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(2) = q(2) + 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(2) = q(2) - 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(3) = q(3) + 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(3) = q(3) - 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(4) = q(4) + 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end


% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(4) = q(4) - 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(5) = q(5) + 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(5) = q(5) - 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
q = handles.model.getpos;
q(6) = q(6) + 0.1
handles.model.animate(q)
pos = handles.model.fkine(q);
disp(pos.t)
else
    disp ('E-Stop Pressed')
end

% --- Executes on button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.but2Val == 0
    q = handles.model.getpos;
    q(6) = q(6) - 0.1
    handles.model.animate(q)
    pos = handles.model.fkine(q);
    disp(pos.t)
else
    disp ('E-Stop Pressed')
end





% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of togglebutton1

% Check if STOP buttons has been pressed

handles.but2Val = get(hObject,'Value');
if handles.but2Val == 1
    disp ('E-Stop Pressed')
end
    

% Responds to "stop" button.  Toggles the but2Val to TRUE
if handles.but2Val == 0
    disp('E-Stop Released')
end

guidata(hObject.Parent, handles)


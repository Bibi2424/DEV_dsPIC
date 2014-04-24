function varargout = GUI_Debug(varargin)
% GUI_DEBUG MATLAB code for GUI_Debug.fig
%      GUI_DEBUG, by itself, creates a new GUI_DEBUG or raises the existing
%      singleton*.
%
%      H = GUI_DEBUG returns the handle to a new GUI_DEBUG or the handle to
%      the existing singleton*.
%
%      GUI_DEBUG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_DEBUG.M with the given input arguments.
%
%      GUI_DEBUG('Property','Value',...) creates a new GUI_DEBUG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_Debug_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_Debug_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_Debug

% Last Modified by GUIDE v2.5 25-Oct-2013 14:59:57

%TODO :
%    -> Check if 'a' is correctly reeded
%           -> OK
%    -> Fix the timer stop problem
%           -> There wasn't any problem, but I add a response to stop the
%           µC (-1->reset, 0->stop, 1->run)
%    -> Fixed start issue
%           -> NOK
%    -> Reset when close Serial
%    -> Make it possible with multiple y datas
%    -> Auto detect possible Serial Port
%    -> Display Figure options (+ export)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_Debug_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_Debug_OutputFcn, ...
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


% --- Executes just before GUI_Debug is made visible.
function GUI_Debug_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_Debug (see VARARGIN)

% Choose default command line output for GUI_Debug
handles.output = hObject;

handles.x=0;
handles.y=0;
handles.z=0;

handles.s=0;

handles.osc = plot(handles.axes_osc, [0 1023 1023 0], [0 0 1023 1023]);

handles.tmr = timer('TimerFcn', {@TmrFcn,gcf},'BusyMode','Queue', 'ExecutionMode','FixedRate','Period',0.01);
% Update handles structure
guidata(hObject, handles);


% UIWAIT makes GUI_Debug wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = GUI_Debug_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
if exist('handles.tmr', 'var')
    stop(handles.tmr);
    delete(handles.tmr);
end
if exist('handles.s', 'var');
    fclose(handles.s);
end
delete(hObject);

% --- Timer Callback
function TmrFcn(hObject, eventdata, hfig)
handles = guidata(hfig);
if(handles.s.BytesAvailable > 0)

    a = fscanf(handles.s, '%8f %8f %8f');   %Read from serial

    if numel(a) == 3                        %If input data is correct
    
        handles.x = a(1);
        handles.y = a(2);
        handles.z = a(2);

        plot(handles.osc, a(1), a(2), 'x');
        drawnow;
    end
end
guidata(hfig,handles);

% --- Executes on selection change in popupmenu_port.
function popupmenu_port_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_port contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_port


% --- Executes during object creation, after setting all properties.
function popupmenu_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'COM1', 'COM2', 'COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8', 'COM9'});


% --- Executes on button press in pushbutton_Port.
function pushbutton_Port_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(instrfind);
popup_sel_index = get(handles.popupmenu_port, 'Value');
switch popup_sel_index
    case 1
        handles.s=serial('COM1');
    case 2
        handles.s=serial('COM2');
    case 3
        handles.s=serial('COM3');
    case 4
        handles.s=serial('COM4');
    case 5
        handles.s=serial('COM5');
    case 6
        handles.s=serial('COM6');
    case 7
        handles.s=serial('COM7');
    case 8
        handles.s=serial('COM8');
    case 9
        handles.s=serial('COM9');
end

%Configure Serial Port

set(handles.s, 'InputBufferSize', 256);     %number of bytes in inout buffer
set(handles.s, 'BaudRate', 115200);
set(handles.s, 'Parity', 'none');
set(handles.s, 'DataBits', 8);
set(handles.s, 'StopBit', 1);
set(handles.s, 'Timeout',10);

error = 0;
try                 %Try to open the serial port
    fopen(handles.s);                       
catch err
    error = 1;
end

if error == 1
    set(handles.edit_port, 'String', ['Error while opening ', get(handles.s,'Name')]);
    set(handles.uipanel1, 'Visible', 'off');
else
    set(handles.edit_port, 'String', [get(handles.s,'Name'), ' opened']);
    set(handles.uipanel1, 'Visible', 'on');
    flushinput(handles.s);                     %FlushSerial
end
guidata(hObject, handles);

% --- Executes on button press in pushbutton_portClose.
function pushbutton_portClose_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_portClose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.edit_port, 'String', [get(handles.s,'Name'), ' closed']);
set(handles.uipanel1, 'Visible', 'off');
delete(instrfind);

% --- Executes on button press in togglebutton_OnOff.
function togglebutton_OnOff_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton_OnOff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton_OnOff
state = get(hObject,'Value');
if state == 1                       %If start
    flushinput(handles.s);
    %fwrite(handles.s,2);                            %Send 2 (reset) on Serial Port
    %fwrite(handles.s,1);                            %Send 1 (run) on Serial Port
    start(handles.tmr);                             %Start timer
else                                %If Stop
    stop(handles.tmr);                              %Stop Timer
    %fwrite(handles.s,0);                            %Send 0 (stop) on Serial Port
end
guidata(hObject, handles);


% --- Executes on button press in pushbutton_reset.
function pushbutton_reset_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop(handles.tmr);          %Stop Timer
flushinput(handles.s);      %Flushing Input Buffer
set(handles.togglebutton_OnOff, 'Value', 0);    %Set Run/Stop button to Stop
handles.x = 0;%zeros(1,handles.length);         %Reset x
handles.y = 0;            %Reset y
handles.z = 0;                                  %Reset z
handles.osc = plot(handles.axes_osc, [0 1023 1023 0], [0 0 1023 1023]);    %Prepare first graph
guidata(hObject, handles);



function edit_length_Callback(hObject, eventdata, handles)
% hObject    handle to edit_length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_length as text
%        str2double(get(hObject,'String')) returns contents of edit_length as a double
%handles.length = str2double(get(hObject,'String'));     %Set the variable length to the value specified on the box
guidata(hObject, handles);

% --- Executes on button press in update_length.
function update_length_Callback(hObject, eventdata, handles)
% hObject    handle to update_length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.length = str2double(get(handles.edit_length,'String'));     %Set the variable length to the value specified on the box
handles.y = zeros(1,handles.length);            %Reset y
handles.osc = plot(handles.axes_osc, (1:handles.length), handles.y);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_length_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
    set(hObject,'String',100);
end


clear; clc
addpath '../Simulation/3D'
addpath '../Simulation/derivation'

% USER:: your serial port
% Open the Arduino IDE to find the appropriate value
% WINDOWS: 'COM#'
% MACOS: '/dev/cu.usbmodem######'
% SERIAL_PORT_ID = 'COM3';
SERIAL_PORT_ID = '/dev/cu.usbmodem65646201';
BAUD_RATE = 115200;

% Use this to save data:
% save('Data/!!!run_name.mat!!!', 't', 'xhatdata', 'eulerData');

% Use this between runs:
% clear all; close all; instrreset; clc;

% Make sure all instruments and serial connections are removed
instrreset
fclose('all');

% START_FLAG  indicates that the vehicle is now outputting valid data
% DATA_FLAG   indicates that the data should be saved
% VIS_FLAG    indicates that data should be displayed
global shouldRead START_FLAG STOP_FLAG VIS_FLAG;
global globalTic;
globalTic = tic;
shouldRead  =   true;

% USER:: Make sure to check that baudrate matches the arduino serial baudrate
% Init serial connection to arduino
global a_serial;
a_serial = serial(SERIAL_PORT_ID, 'BaudRate', BAUD_RATE, 'Terminator', 'CR/LF');
a_serial.BytesAvailableFcnMode = 'byte';
a_serial.BytesAvailableFcnMode = 'terminator';
a_serial.BytesAvailableFcn = @(obj, event) parse(obj, event); % other params?
fopen(a_serial);

global xhatdata index
xhatdata = zeros(1,10); 
index = 1;

function parse(obj, event)
global a_serial
global index xhatdata

% check if callback is passed a valid set of chars
if strcmp(event.Type, 'BytesAvailable')
						
	% is valid data
	output = fscanf(a_serial, '%s');
	xhatdata(index,:) = cellfun(@(x) str2num(x), split(output, '_'))';
	
% 	xhatdata(index)
% xhatdata(index, :)
	
	% convert data to row vector and update matrix
% 	xhatdata(index,:) = cellfun(@(x) str2num(x), split(output, '_'))'

	index = index + 1;
	
end

end

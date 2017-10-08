%% Import data from text file.
% Script for importing data from the following text file:
%
%    M:\Thesis\Matlab\Path Generation\LOG_16_03\LOG_2015_03_16_12_55_18_CraftPosition.log
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2015/03/18 14:32:32

%% Initialize variables.
[FileName,PathName] = uigetfile('*_CraftPosition.log','Select craft position file');
%filename = 'M:\Thesis\Matlab\Path Generation\LOG_16_03\LOG_2015_03_16_12_55_18_CraftPosition.log';
filename = strcat(PathName,FileName);
delimiter = {',',' '};
startRow = 5;

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
xMeas = dataArray{:, 1};
yMeas = dataArray{:, 2};
psiRef = dataArray{:, 3};
CTE = dataArray{:, 4};
deltaRudd = dataArray{:, 5};


%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;
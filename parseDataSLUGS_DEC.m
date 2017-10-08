%% Import data from text file.
% Script for importing data from the following text file:
%
%    M:\Thesis\Matlab\Path Generation\LOG_16_03\SLUGS_LOG_13_14_17_EQUIVALENT.log
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2015/03/18 16:39:01

%% Initialize variables.
filename = 'M:\Thesis\Matlab\Path Generation\LOG_16_03\SLUGS_LOG_13_14_17_EQUIVALENT.log';
delimiter = ',';

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
%   column13: double (%f)
%	column14: double (%f)
%   column15: double (%f)
%	column16: double (%f)
%   column17: double (%f)
%	column18: double (%f)
%   column19: double (%f)
%	column20: double (%f)
%   column21: double (%f)
%	column22: double (%f)
%   column23: double (%f)
%	column24: double (%f)
%   column25: double (%f)
%	column26: double (%f)
%   column27: double (%f)
%	column28: double (%f)
%   column29: double (%f)
%	column30: double (%f)
%   column31: double (%f)
%	column32: double (%f)
%   column33: double (%f)
%	column34: double (%f)
%   column35: double (%f)
%	column36: double (%f)
%   column37: double (%f)
%	column38: double (%f)
%   column39: double (%f)
%	column40: double (%f)
%   column41: double (%f)
%	column42: double (%f)
%   column43: double (%f)
%	column44: double (%f)
%   column45: double (%f)
%	column46: double (%f)
%   column47: double (%f)
%	column48: double (%f)
%   column49: double (%f)
%	column50: double (%f)
%   column51: double (%f)
%	column52: double (%f)
%   column53: double (%f)
%	column54: double (%f)
%   column55: double (%f)
%	column56: double (%f)
%   column57: double (%f)
%	column58: double (%f)
%   column59: double (%f)
%	column60: double (%f)
%   column61: double (%f)
%	column62: double (%f)
%   column63: double (%f)
%	column64: double (%f)
%   column65: double (%f)
%	column66: double (%f)
%   column67: double (%f)
%	column68: double (%f)
%   column69: double (%f)
%	column70: double (%f)
%   column71: double (%f)
%	column72: double (%f)
%   column73: double (%f)
%	column74: double (%f)
%   column75: double (%f)
%	column76: double (%f)
%   column77: double (%f)
%	column78: double (%f)
%   column79: double (%f)
%	column80: double (%f)
%   column81: double (%f)
%	column82: double (%f)
%   column83: double (%f)
%	column84: double (%f)
%   column85: double (%f)
%	column86: double (%f)
%   column87: double (%f)
%	column88: double (%f)
%   column89: double (%f)
%	column90: double (%f)
%   column91: double (%f)
%	column92: double (%f)
%   column93: double (%f)
%	column94: double (%f)
%   column95: double (%f)
%	column96: double (%f)
%   column97: double (%f)
%	column98: double (%f)
%   column99: double (%f)
%	column100: double (%f)
%   column101: double (%f)
%	column102: double (%f)
%   column103: double (%f)
%	column104: double (%f)
%   column105: double (%f)
%	column106: double (%f)
%   column107: double (%f)
%	column108: double (%f)
%   column109: double (%f)
%	column110: double (%f)
%   column111: double (%f)
%	column112: double (%f)
%   column113: double (%f)
%	column114: double (%f)
%   column115: double (%f)
%	column116: double (%f)
%   column117: double (%f)
%	column118: double (%f)
%   column119: double (%f)
%	column120: double (%f)
%   column121: double (%f)
%	column122: double (%f)
%   column123: double (%f)
%	column124: double (%f)
%   column125: double (%f)
%	column126: double (%f)
%   column127: double (%f)
%	column128: double (%f)
%   column129: double (%f)
%	column130: double (%f)
%   column131: double (%f)
%	column132: double (%f)
%   column133: double (%f)
%	column134: double (%f)
%   column135: double (%f)
%	column136: double (%f)
%   column137: double (%f)
%	column138: double (%f)
%   column139: double (%f)
%	column140: double (%f)
%   column141: double (%f)
%	column142: double (%f)
%   column143: double (%f)
%	column144: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
Timestamp = dataArray{:, 1};
ROLL = dataArray{:, 2};
PITCH = dataArray{:, 3};
YAW = dataArray{:, 4};
P = dataArray{:, 5};
Q = dataArray{:, 6};
R = dataArray{:, 7};
X = dataArray{:, 8};
Y = dataArray{:, 9};
Z = dataArray{:, 10};
VX = dataArray{:, 11};
VY = dataArray{:, 12};
VZ = dataArray{:, 13};
GPSYEAR = dataArray{:, 14};
GPSMONTH = dataArray{:, 15};
GPSDAY = dataArray{:, 16};
GPSHOUR = dataArray{:, 17};
GPSMIN = dataArray{:, 18};
GPSSEC = dataArray{:, 19};
GPSLAT = dataArray{:, 20};
GPSLONG = dataArray{:, 21};
GPSHEIGHT = dataArray{:, 22};
GPSCOG = dataArray{:, 23};
GPSSOG = dataArray{:, 24};
GPSHDOP = dataArray{:, 25};
VarName26 = dataArray{:, 26};
GPSSATSUSED = dataArray{:, 27};
VarName28 = dataArray{:, 28};
RAWGYROX = dataArray{:, 29};
RAWGYROY = dataArray{:, 30};
RAWGYROZ = dataArray{:, 31};
RAWACCX = dataArray{:, 32};
RAWACCY = dataArray{:, 33};
RAWACCZ = dataArray{:, 34};
RAWMAGX = dataArray{:, 35};
RAWMAGY = dataArray{:, 36};
RAWMAGZ = dataArray{:, 37};
VarName38 = dataArray{:, 38};
VarName39 = dataArray{:, 39};
VarName40 = dataArray{:, 40};
VarName41 = dataArray{:, 41};
BIASACCELX = dataArray{:, 42};
BIASACCELY = dataArray{:, 43};
BIASACCELZ = dataArray{:, 44};
BIASGYROP = dataArray{:, 45};
BIASGYROQ = dataArray{:, 46};
BIASGYROR = dataArray{:, 47};
VarName48 = dataArray{:, 48};
VarName49 = dataArray{:, 49};
VarName50 = dataArray{:, 50};
VarName51 = dataArray{:, 51};
VarName52 = dataArray{:, 52};
VarName53 = dataArray{:, 53};
VarName54 = dataArray{:, 54};
VarName55 = dataArray{:, 55};
VarName56 = dataArray{:, 56};
VarName57 = dataArray{:, 57};
VarName58 = dataArray{:, 58};
VarName59 = dataArray{:, 59};
VarName60 = dataArray{:, 60};
VarName61 = dataArray{:, 61};
VarName62 = dataArray{:, 62};
VarName63 = dataArray{:, 63};
VarName64 = dataArray{:, 64};
CMDTHROT = dataArray{:, 65};
CMD66 = dataArray{:, 66};
VarName67 = dataArray{:, 67};
CMDRUD = dataArray{:, 68};
CMD68 = dataArray{:, 69};
VarName70 = dataArray{:, 70};
VarName71 = dataArray{:, 71};
VarName72 = dataArray{:, 72};
VarName73 = dataArray{:, 73};
VarName74 = dataArray{:, 74};
VarName75 = dataArray{:, 75};
VarName76 = dataArray{:, 76};
VarName77 = dataArray{:, 77};
VarName78 = dataArray{:, 78};
VarName79 = dataArray{:, 79};
VarName80 = dataArray{:, 80};
PASSTHRRUD = dataArray{:, 81};
VarName82 = dataArray{:, 82};
VarName83 = dataArray{:, 83};
VarName84 = dataArray{:, 84};
VarName85 = dataArray{:, 85};
VarName86 = dataArray{:, 86};
VarName87 = dataArray{:, 87};
VarName88 = dataArray{:, 88};
VarName89 = dataArray{:, 89};
VarName90 = dataArray{:, 90};
VarName91 = dataArray{:, 91};
VarName92 = dataArray{:, 92};
VarName93 = dataArray{:, 93};
VarName94 = dataArray{:, 94};
VarName95 = dataArray{:, 95};
VarName96 = dataArray{:, 96};
VarName97 = dataArray{:, 97};
VarName98 = dataArray{:, 98};
VarName99 = dataArray{:, 99};
VarName100 = dataArray{:, 100};
THROTTLEHANDLE = dataArray{:, 101};
RUDDERCOMMANDDEG = dataArray{:, 102};
VarName103 = dataArray{:, 103};
VarName104 = dataArray{:, 104};
ACCELX = dataArray{:, 105};
ACCELY = dataArray{:, 106};
ACCELZ = dataArray{:, 107};
MAGNETX = dataArray{:, 108};
MAGNETY = dataArray{:, 109};
MAGNETZ = dataArray{:, 110};
NDELTA = dataArray{:, 111};
ANTIWINDUP = dataArray{:, 112};
VarName113 = dataArray{:, 113};
VarName114 = dataArray{:, 114};
VarName115 = dataArray{:, 115};
VarName116 = dataArray{:, 116};
VarName117 = dataArray{:, 117};
VarName118 = dataArray{:, 118};
VarName119 = dataArray{:, 119};
VarName120 = dataArray{:, 120};
VarName121 = dataArray{:, 121};
VarName122 = dataArray{:, 122};
VarName123 = dataArray{:, 123};
VarName124 = dataArray{:, 124};
VarName125 = dataArray{:, 125};
VarName126 = dataArray{:, 126};
VarName127 = dataArray{:, 127};
VarName128 = dataArray{:, 128};
VarName129 = dataArray{:, 129};
VarName130 = dataArray{:, 130};
VarName131 = dataArray{:, 131};
VarName132 = dataArray{:, 132};
VarName133 = dataArray{:, 133};
VarName134 = dataArray{:, 134};
VarName135 = dataArray{:, 135};
VarName136 = dataArray{:, 136};
VarName137 = dataArray{:, 137};
VarName138 = dataArray{:, 138};
VarName139 = dataArray{:, 139};
VarName140 = dataArray{:, 140};
VarName141 = dataArray{:, 141};
VarName142 = dataArray{:, 142};
VarName143 = dataArray{:, 143};
VarName144 = dataArray{:, 144};


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;
global u_craft x0 y0 Psi0 v_current phiBeta Tr K k1 k2 kp ki kpr kdr Tdelta lookAheadDistance acceptanceR wptList

initWpt;
%load streamline.mat
load CTRL.mat
load refSmoother.mat
refSmoother;
CreateLookUpTableSpeedCurvature;

% Define parameteres
x0 = 0; 
y0 = 0;
Psi0 = 0;
u_craft = 5;
v_current = 0;
phiBeta = 0;
acceptanceR = 10;
lookAheadDistance = 10;

% Dynamical parameters
% Nozzle (1st order)
Tdelta = 0.32; % "It takes 1 second to go from far left to far right" Thesis Casper Ole 2012
% Heading (Nomoto model)
Tr = 1.72;  % From thesis Lukas
K = 2.06;  % From thesis Lukas
k1 = 1/Tr;
k2 = K/Tr;

% Craft parameters
L = 3.25; % Craft length [m]
uMax = 30; % Max speed [m/s]

% Controller parameters
kp = 1/lookAheadDistance;   % Proportional action heading
ki = 0.1/lookAheadDistance; % Integral action heading
deltaMax = 0.26;
% kdr = 1.5;   % Obtained with pole placement
% kpr = 5*deltaMax/pi*(1+kdr*K/Tr); % Obtained with pole placement

obstacleList = [50, -5, 1,  pi-atan2(-5,50),15,15];
              %   40, -1, 1,pi,3,3;
              %   70,9,1,pi,3,3]; 
                %270, 4,1,pi,7,7;
                %240,-4,1,pi,7,7;
                %350, 10,1,pi,7,7
                %];
target = [100,0];
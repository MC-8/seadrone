%% Reference smoother
% I want a bandwidth  ~bandwidth of the system (not faster) that is about 5
% rad/s
s = tf('s');
omega0 = 2.5;
damping = 1;
referenceSmoother = (omega0^2)/(s^2+2*damping*omega0*s + omega0^2);
Aref = [0 1; -omega0^2 -2*damping*omega0];
Bref = [0 ; omega0^2];
Cref = [1 0];
Dref = 0;
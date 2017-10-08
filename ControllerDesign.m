clc
s = tf('s');
T1 = 0.32;
G1 = 1/(s*T1+1);
T2 = 1.72;
K = 2.06;
G2 = K/(T2*s+1);
G3 = 1/s;
F = G1*G2*G3;
% figure(1);
% bode(F);
%%
LC1 = (1+1*s)/(1+1*0.4*s);
LC2 = (1+1*s)/(1+1*0.1*s);

Kc = 0.8;
CTRL = Kc*LC1*LC2;

F2 = CTRL*F;

W2 = feedback(F2,1);
bode(W2)
hold on
bode(G1)
save('CTRL','CTRL');
%%
 bode(W2);
 hold on;
 bode(F2);
 bode(feedback(0.8*F2,1));
save('CTRL','CTRL');
%%
margin(F2);
%%
Kp = 1;


figure(5);
bode(G1*C*G2);
W = feedback(G1*C*G2,1);
figure(6);
margin(W);
figure(7);
bode(G1); hold on;
bode(G2);
bode(C*G2);

R2 = (1+s/1.2)/(1+s/12);
C2 = Kp*R2;
W2 = feedback(G1*C2*G2,1);
omega0 = 10;
damping = 1;
referenceSmoother = (omega0^2)/(s^2+2*damping*omega0*s + omega0^2);
%bandwidth(W)
bandwidth(W2)
figure();
W1 = feedback(G1*G2,1);
bode(G1,W2,W1);
legend('Actuator','Closed Loop w/ controller','Closed Loop w/o controller');
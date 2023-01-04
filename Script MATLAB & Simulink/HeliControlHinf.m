%% Script Matlab per il controllo di un elicottero a 2 gradi di libertà
%
% Ciro Arena − P38/53
% Vito Giura − P38/56

close all;
clear;
clc;

%% Imposto i parametri del sistema
Mheli=1.3872; %kg Massa totale dell'elicottero (corpo, motori etc.)
Lcm = 0.186; %m Distanza tra il centro di massa dell'elicottero e l'asse di pitch
Kpp = 0.204; %Nm/V costante della forza di spinta del motore di pitch
Kyy = 0.072; %Nm/V costante della forza di spinta del motore di yaw
Kpy = 0.0068; %Nm/V costante di coppia che agisce lungo l'asse di pitch dovuta alla forza del motore di yaw
Kyp = 0.0219; %Nm/V costante di coppia che agisce lungo l'asse di yaw dovuta alla forza del motore di pitch
Jeq_p = 0.0384; %Kg.m^2 momento di inerzia totale intorno l'asse di pitch
Jeq_y = 0.0432; %Kg.m^2 momento di inerzia totale intorno l'asse di yaw
Bp = 0.8; %N/V coefficiente di smorzamento viscoso relativo all'asse di pitch
By = 0.318; %N/V coefficiente di smorzamento viscoso relativo all'asse di yaw
g = 9.81; %m/s^2 costante gravitazionale terrestre

%% Linearizzazione

%Matrici del sistema linearizzato intorno al punto di equilibrio
Jtp = Jeq_p + Mheli*Lcm^2;
Jty = Jeq_y + Mheli*Lcm^2;

A=[0 0 1 0;
   0 0 0 1;
   0 0 -Bp/Jtp 0;
   0 0 0 -By/Jty];

B=[0 0;
   0 0;
   Kpp/Jtp Kpy/Jtp;
   Kyp/Jty Kyy/Jty];

 C=[1 0 0 0;
     0 1 0 0];

D=zeros(2,2);

x1=0;
x0=[x1;0;0;0]; %stato di equilibrio
a=[Kpp Kpy; Kyp Kyy];
b=[Mheli*g*Lcm*cos(x1);0];
u0=a\b; %ingresso di equilibrio

%matrici del sistema linearizzato
%[A,B,C,D]=linmod('LinearHeli');

%% Controllabilità e Osservabilità

% verifica della completa raggiungibilità
n=length(A); %dimensione del sistema
Mr=ctrb(A,B); %matrice di controllabilità
nr=rank(Mr); %dimensione del sottospazio di raggiungibilità
nnr=n-nr; % dimensione del sottospazio di NON raggiungibilità

if nnr==0
    disp('Il sistema è completamente raggiungibile')
else
    disp('Il sistema NON è completamente raggiungibile')
end

Mo=obsv(A,C); %matrice di osservabilità
no=rank(Mo); %dimensione del sottospazio di osservabilità
nno=n-no; %dimensione del sottospazio di NON osservabilità
if nno==0
    disp('Il sistema è completamente osservabile')
else
    disp('Il sistema NON è completamente osservabile')
end

%% Progettazione controllore H infinito
s = zpk('s');

G=ss(A,B,C,D);

%scelta delle matrici di peso Ws, Wk e Wt
W1 = 5000/((180*s+1));
Ws=append(4*W1,4*W1);
Wk = diag([48 45]);
Wt = [];

% %scelta delle matrici di peso Ws, Wk e Wt
% W1 = 5000/((300*s+1));
% Ws=append(4*W1,4*W1);
% Wk = diag([50 45]);
% Wt = [];

% %scelta delle matrici di peso Ws, Wk e Wt
% W1 = 4800/((170*s+1));
% Ws=append(4*W1,4*W1);
% Wk = diag([48 45]);
% Wt = [];c

%creazione della matrice di P(s)
P = augw(G,Ws,Wk,Wt);

%creazione del controllore
[K,CL,GAM] = hinfsyn(P);  
L = G*K; %funzione d'anello              
T = feedback(L,eye(2)); %funzione di sensività complementare
S = eye(2)-T; %funzione di sensività

figure(1)
subplot(2,1,1)
plot(out.t,out.pitch)
grid on
title('Angolo di Pitch')
legend('non lineare', 'lineare')
subplot(2,1,2)
plot(out.t,out.yaw)
grid on
title('Angolo di Yaw')
legend('non lineare', 'lineare')

figure(2)
subplot(2,1,1)
plot(out.t,out.motorpitch)
grid on
title('Motore di Pitch')
subplot(2,1,2)
plot(out.t,out.motoryaw)
grid on
title('Motore di Yaw')
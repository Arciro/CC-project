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

%% Progettazione LQR con azione integrale

[p, n]=size(C); %restituisce il numero di uscite

%matrici del sistema aumentato di ordine n+p
Ag=[A zeros(n,p); -C zeros(p,p)];
Bg=[B; zeros(p,p)];
Cg=[C zeros(p,p)];
Dg=D;

Q=diag([8 8 0.0006 0.1 14 14]);
R=diag([0.0003 0.0003]);
K=lqr(Ag,Bg,Q,R);

Kf=K(:,1:4);
Ki=K(:,5:end);


%% Progettazione Filtro di Kalman

T=(10/3)*1e-3;
v_x=8e-4;
v_y=8e-8;
Qtilde=v_x*diag([1 1 1 1]);
Rtilde=v_y*diag([1 1]);
L=lqr(A',C',Qtilde,Rtilde)';

Afk=A-L*C;
Bfk=[B-L*D L];
Cfk=eye(4);
Dfk=zeros(4,4);

% %% Simulazione
% open('KalmanFilterHeli.slx')
% sim('KalmanFilterHeli.slx')

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
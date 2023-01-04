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

%Linearizzazione intorno al punto di equilibrio
Jtp = Jeq_p + Mheli*Lcm^2;
Jty = Jeq_y + Mheli*Lcm^2;
x1=0;
x0=[x1;0;0;0]; %stato di equilibrio
a=[Kpp Kpy; Kyp Kyy];
b=[Mheli*g*Lcm*cos(x1);0];
u0=a\b; %ingresso di equilibrio

%matrici del sistema linearizzato
[A,B,C,D]=linmod('LinearHeli');

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

%% Progettazione in retroazione di stato con azione integrale

[p, n]=size(C); %restituisce il numero di uscite

%matrici del sistema aumentato di ordine n+p
Ag=[A zeros(n,p); -C zeros(p,p)];
Bg=[B; zeros(p,p)];
Cg=[C zeros(p,p)];
Dg=D;

p1=-0.9;
p2=p1-0.1;
p3=7.5*p1;
p4=7.6*p1;
p5=p3-0.5;
p6=p4-0.5;
p=[p1 p2 p3 p4 p5 p6];

%matrice di guadagno K
K=place(Ag,Bg,p);

Kf=K(:,1:4);
Ki=K(:,5:end);

%% Progettazione osservatore

%matrice di guadagno L dell'osservatore
po=[6.85*p4, 6.85*p4+0.1, 6.85*p4+0.2, 6.85*p4+0.3];
L=place(A',C',po)';
Aoss=A-L*C;
Boss=[B-L*D L];
Coss=eye(4);
Doss=zeros(4,4);

%% Simulazione
% open('PolePlaceHeliInteger.slx')
% sim('PolePlaceHeliInteger.slx')

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

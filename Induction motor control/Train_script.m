clc
clear all

%% DC machine parameters
load('Traction_e402B_online');
%parameters of 1 motor = equivalent motor
V_motor = 1860; %voltage of each motor
V_max=V_motor*sqrt(3);
pp=2; %pole pairs of each motor
cos_fi=0.85; %power factor of each motor
Rs=0.01; %p.u.
Rr=0.01; %p.u.
X_lock=0.1; %p.u.
X_noload=3.5; %p.u.
d=1250*10^-3; %diameter wheels
tr=23/91; %trasmission ratio
mass_tot=89*10^3+11*47*10^3+45*80+45*15; %kg

v_max=200*1000/3600; %max speed m/s
omega_max=v_max/tr/d*2; %rad/s
eff=0.9; %efficiency 
J=mass_tot*v_max^2/omega_max^2;
%J_eq=mass_tot*(tr*d)^2/4;

Pn=5.6*10^6; %tot mechanical power [W]
Pe_tot=Pn/eff;


I_motor=Pe_tot/sqrt(3)/V_motor/cos_fi; 
Z=V_motor/I_motor;

Rs_eq=Rs*Z; 
Rr_eq=Rr*Z; 
X_lock_eq=X_lock*Z; 
X_noload_eq=X_noload*Z;
Rks=Rs_eq+Rr_eq;
Lks=X_lock_eq/(2*pi*50);
M=(X_noload_eq)/(2*pi*50);

F_friction=62000; %N at v_max
T_friction=F_friction/2*d/2*tr; %at 100km/h base speed

omega_b=100*1000/3600/tr/d*2;
psi_r_max=V_motor*sqrt(3)*0.8/(omega_b*pp);

k=traction(1,2).*d/2.*tr./psi_r_max;
psi_rd_ref = traction(:,2).*d/2.*tr./k;
speed_ref = traction(:,1);
id_ref=psi_rd_ref./M;

Tn = Pn/omega_b; % Nominal torque provided by the machine
iq_max=1000*traction(1,2)*d/2*tr/pp/psi_r_max;

tau_s=Lks/Rs;
B=T_friction/omega_b;
tau_O=J/B;
tau_psi=M/Rr_eq;

% B = Tfriction/rated_speed_motor; % friction coefficient < Or
% tau_mec=J_eq/B;
%%  PI controller design parameters
s=tf('s');
%Gi
tau_s_desired=tau_s/100;
wc_s=2*pi/tau_s_desired;

%GO
tau_O_desired=tau_O/1000;
wc_O=2*pi/tau_O_desired;

%Gpsi
tau_psi_desired=tau_psi/1000;
wc_psi=2*pi/tau_psi_desired;

%tf
Gi = 1/(Rs+Lks*s);

GO = 1/(B+J*s);

Gpsi = (1/M)/(1+(M/Rr_eq)*s);
%% Zero Pole cancellation (90 phase margin)
% %PI parameters ia
% kp_a=wc_a*La;
% ki_a=wc_a*Ra;
% Regi=kp_a+ki_a/s
% Ti_a=kp_a/ki_a;
% %tf open loop
% Li=Regi*Gi;
% %tf close loop
% Fi=Li/(1+Li);
% % figure
% % bode(Li)
% % figure
% % bode(Fi)
% %PI parameters ie
% kp_e=wc_e*Le;
% ki_e=wc_e*Re;
% Rege=kp_e+ki_e/s
% Ti_e=kp_e/ki_e;
% %tf open loop
% L_e=Rege*Ge;
% %tf close loop
% Fe=L_e/(1+L_e);
% % figure
% % bode(L_e)
% % figure
% % bode(Fe)
% %PI parameters speed
% kp_O=wc_O*J_eq;
% ki_O=wc_O*B;
% RegO=kp_O+ki_O/s
% Ti_O=kp_O/ki_O;
% %tf open loop
% LO=RegO*GO;
% %tf close loop
% FO=LO/(1+LO);
% % figure
% % bode(LO)
% % figure
% % bode(FO)
% %saturation
% SatUp_Va = Van; % [A]
% SatLow_Va = -Van;
% SatUp_T = Tn*10; % [V]
% SatLow_T = -Tn*10;
% SatUp_Ve = Ven*1.1;
% SatLow_Ve = 0;
%% Pidtool
%otherwise use pidtool
phase_m=90;
%pidtool(Gi)
opt=pidtuneOptions('PhaseMargin', phase_m);
par_regi=pidtune(Gi,'PI',wc_s,opt);

ki_s=par_regi.Ki;
kp_s=par_regi.Kp;

Regi=kp_s+ki_s/s
Ti_s=kp_s/ki_s;
%tf open loop
Li=Regi*Gi;
%tf close loop
Fi=Li/(1+Li);
% figure
% bode(Li);
% figure
% bode(Fi);
% figure
% margin(Li);


%pidtool(GO)
par_reg_speed=pidtune(GO,'PI',wc_O,opt);
ki_O=par_reg_speed.Ki;
kp_O=par_reg_speed.Kp;

RegO=kp_O+ki_O/s
Ti_O=kp_O/ki_O;
%tf open loop
LO=RegO*GO;
%tf close loop
FO=LO/(1+LO);
% figure
% bode(LO);
% figure
% bode(FO);
% figure
% margin(LO);

%pidtool(Gpsi)
par_reg_speed=pidtune(Gpsi,'PI',wc_psi,opt);
ki_psi=par_reg_speed.Ki;
kp_psi=par_reg_speed.Kp;

Regpsi=kp_psi+ki_psi/s
Ti_psi=kp_psi/ki_psi;
%tf open loop
Lpsi=Regpsi*Gpsi;
%tf close loop
Fpsi=Lpsi/(1+Lpsi);
% figure
% bode(LO);
% figure
% bode(FO);
% figure
% margin(LO);


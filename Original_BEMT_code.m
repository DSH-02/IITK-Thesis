clear all;
clc

%------------------------Sabal Optimization Code for Hover--------------------------%
%-----------------------------------------------------------------------------------%

% Given Mission Parameters:
h_g = 0;             % Geometric Altitude in m
Range = 10;          % Range in km
Climb_alt = 0.5;     % Climb Altitude in km
endurance = 1;       % Endurance in hrs (Given: 60 min)
Payload = 25 ;       % Payload mass in kg

% Atmospheric Parameters Calculation:
P0 = 1.01325*(10^5); % Atmospheric Pressure at MSL in N/m^2                 
T0 = 288.16+15;      % Temperature in K (15 deg added for IRA)                              
rho0 = 1.225;        % Density in kg/m^3  
%[rho,P,T]=atmos(h,P0,T0,rho0);

% function [rho,P,T] = atmos(z,P0,T0,den0)
z1 = h_g;            % Geomentric Altitude in m
Re = 6356766;        % Radius of the earth in m
Rs = 287;            % Specific gas constant for Air in J/(kg*K) 
g_0 = 9.81;          % Acceleration due to gravity at sea level in m/s^2

if ((z1 >=0) && (z1 <= 11000))              % Gradient layer 0-11 km
    h = (Re*z1)/(Re + z1);                  % Geopotential altitute in m
    h1 = 0;                                 % Altitude at sea level in m 
    a = -0.0065;                            % Temperature Lapse Rate in K/m for the Altitude till 11 Km
    T = T0 + a*(h - h1);                    % Temperature at Altitude h in K
    P = P0*((T/T0)^(-g_0/(a*Rs))) ;         % Pressure at Altitude h in N/m^2
    rho = rho0*(T/T0)^((-g_0/(a*Rs)) - 1) ; % Density at Altitude h in kg/m^3  
end

g = 9.81;                % Acceleration due to gravity in m/s^2
gama = 1.4;              % Ratio of specific heats for air
cair = sqrt(gama*Rs*T);  % Speed of sound in air in m/s
Cd=0.011;                % Coefficient of profile drag
cl_alpha=5.73;           % Lift curve slope
%%
% Baseline Design Configuration Parameters:
R_fr = 1.2:0.1:2;                % Blade radius of front rotor in m                  %1.2:0.1:1.4;%:0.1:1.2;
Nb_fr = 2;                       % Number of blades of front rotor 
AR_fr = 10;                      % Aspect ratio of front rotor
Nb_br = 2;                       % Number of blades of back rotor
AR_br = 10;                      % Aspect ratio of back rotor
theta0  = (0:0.01:20)*pi/180;    % Collective Pitch Angle in rad
thetatw = 0*pi/180;              % Blade twist rate in rad
%Vtip_fr=(1300)*(2*pi*R_fr/60);  %(2*pi*1400/60)*R_fr;% tip velocity in m/s
%Vtip_fr=(1300)*(2*pi*R_fr/60);  % Tip speed of front rotor in m/s
Vtip_fr=110:10:150;

% Number of cells in series in a battery (44.4v/3.7v)
S_fr = 6;                        % For front rotor
S_br = 6;                        % For back rotor

batt_res =0.90;                  % 10% battery reserve taken
NoB = 2;                         % Number of batteries each side
Nom_volt=3.7*S_fr;               % Nominal voltage of the entire one battery (cells in series)

%rpm=(Vtip_fr*60/(2*pi*R_fr));%blade rpm
% Vc=1.5;%climb speed in m/s
% Vd=0.8;% descent speed in m/s

% Losses and Efficiencies Parameters:
trans_loss=1.02;                 % Transmission losses (2%)
elec_loss=1.02;                  % Electrical losses (2%)
motor_eff=0.85;                  % Motor efficiency (85%)
mu=1.09;

% Fixed Component Weights:


%%
% Design Iterative Code:

for i=1:size(R_fr,2)           % Loop for Rotor Radius

    for j=1:size(Vtip_fr,2)    % Loop for Tip Speed

        %fprintf('%4.3f %4.3f %4.3f %4.3f \n',Nb_fr, AR_fr, R_fr(i), Vtip_fr(j));

% Front Rotor Parameters:

        A=pi*(R_fr(i)^2);                   % Rotor disk area (Same for both rotors)
        c_fr(i,j) = R_fr(i)/AR_fr;          % Blade chord in m
        omega_fr(i,j) = Vtip_fr(j)/R_fr(i); % Angular velocity in rad/s
        sigma_fr(i,j) =(Nb_fr*c_fr(i,j)*(R_fr(i)-0.2*R_fr(i)))/(pi*R_fr(i).^2);% Solidity 

% Back Rotor Parameters:

        R_br(i) = R_fr(i);                  % Blade radius in m
        c_br(i,j) = R_br(i)/AR_br;          % Blade chord in m
        sigma_br(i,j) =(Nb_br*c_br(i,j)*(R_br(i)-0.2*R_br(i)))/(pi*R_br(i).^2);% Solidity
        Vtip_br(j) = Vtip_fr(j);            % Tip speed of in m/s
        omega_br(i,j)=Vtip_br(j)/R_br(i);   % Angular velocity in rad/s

        % For non-dimensionalizing Thrust and Pressure:

        nondt=rho*A*(omega_fr(i,j)*R_fr(i))^2; % For thrust terms
        nondp=rho*A*(omega_fr(i,j)*R_fr(i))^3; % For Pressure terms

% Distance between Front and Back Rotors:

        L = R_fr(i)+R_br(i)+(2*R_fr(i)/3);  % Clearance between Rotors = Rotor Diameter/3

% Initialization of Weights:

        m=1;                                  % First index initialization
        n=2;                                  % Second index initialization
        GTOW(m) = 1;                          % Initialized GTOW in kg 
        Empty = 40;                           % Empty weight in kg 
        GTOW(n) = Empty + Payload;            % Total gross weight in kg 
        pre_weight = GTOW(n);                 % Intial gross weight guess for iterations
        
        error = 100;                          % Error Initialization
        while( error > 0.1 )

% Calculation using Momentum Theory for Hover:

            CT_h = GTOW(n)*0.5*9.81/nondt;    % In hover taking thrust = weight
            % CPi = 1.15*(CT_h^1.5)/sqrt(2);    % Coefficient of induced power 
            % CP0 = sigma_fr(i,j)*Cd/8;         % Coefficient of profile power coefficient
            % 
            % % Collective Pitch Angle theta in degrees:
            % collective(i,j)=((6*CT_h/(sigma_fr(i,j)*cl_alpha)) + 1.5*sqrt(CT_h/2))*180/pi; 

% Calculation using BEMT for Hover:

            for k =1:length(theta0)

            [Thrust_bt(k),Power_bt(k),Power_h_mech_bt(k),Torque_bt(k),Theta_bt(k),err_bt(k),FM,BL]=BEMT(R_fr(i),CT_h,Nb_fr,c_fr(i,j),Vtip_fr(j),cair,GTOW(n),trans_loss,nondp,motor_eff,nondt,theta0(k),elec_loss);
            %err_bt
            if (abs(err_bt(k))<5)

                Thrust_hv(i,j)=Thrust_bt(k);
                Power_hv(i,j)=Power_bt(k);
                Torque_hv(i,j)=Torque_bt(k);
                Theta_hv(i,j)=theta0(k);
                Power_h_mech_hv(i,j)=Power_h_mech_bt(k);
                err(i,j)=err_bt(k);

                break;

            else

                Thrust_hv(i,j)=0; 
                Power_hv(i,j)=0;
                Torque_hv(i,j)=0;
                Theta_hv(i,j)=0;
                Profile_power(i,j)=0;
                Induced_power(i,j)=0;
                Power_h_mech_hv(i,j)=0;
              
            end

            end

            if (Thrust_hv(i,j)==0)

                fprintf("solution not possible");
                break;

            end
            % [thrust_h(i,j),power_h(i,j),torque_h(i,j),theta_h(i,j),err(i,j),FM,BL]=BEMT(R_fr(i),Ct,Nb_fr,c_fr(i,j),Vtip_fr(j),a,GW(n),trans_loss,nondp,motor_efficiency,nondt,theta_0,electrical_loss);
            %----Climb power---------------------------------------------
            %[Pclimb(i,j),Pdescent(i,j)]=climb(Thrust_hv(i,j)/4,rho,R_fr(i),Power_hv(i,j)/4,Vc,Vd);
            %energy_ver(i,j)=Pclimb(i,j)*(climb_alt/(Vc*3.6)) +Pdescent(i,j)*(climb_alt/(Vd*3.6));%climb and descent power(in watt-hour)
            %------------calculation using MT(forward-flight)---------------------   
            %[Prange(i,j),Vrange(i,j),Pendu(i,j),Vendu(i,j),Pft(i,j)]=forwardflight(R_fr(i),Vtip_fr(j),thrust_h(i,j)/2,nondp,solidity_fr(i,j));%velocities are in kmph
            %energy_ff(i,j)=2*Pft(i,j)*(range/54);%Pendu(i,j)*(range/Vendu(i,j))+in watt-hr

% Calculation of Auxillary Power:

            Power_servo = 6*10;                                % 60 W
            Power_camera = 25;                                 % 25 W
            aux_power = Power_servo + Power_camera;            % in W
            energy_aux = aux_power*endurance;                  % in Wh

% Total Power and Thrust required in Hover:

            Power_total(i,j) = Power_hv(i,j) + aux_power;      % in W
            Thrust_total(i,j) = Thrust_hv(i,j);                % in N

% Total Energy and Battery Capacity required on the basis of the mission:

            energy(i,j) =(Power_total(i,j)*(endurance)/batt_res); % in Wh %+energy_ver(i,j)+energy_ff(i,j))/batt_reserve;% energy supplied  by battery equals total power required for hover multiplied by the hover duration(in watt-hour)
            energy_MJ(i,j) = energy(i,j)*(3600)*10^(-6);          % Energy in mega -joules(MJ)
            C(i,j) =(((energy(i,j)/(NoB*Nom_volt))*1000));        % Total Battery Capacity, all 2 batteries combined, in mAh=E(energy)/V(voltage), 3.6 volt for 1 cell;(17% for battery reserve)
            %flight_time_min(i,j)= (batt_res*NoB*Nom_volt*(C(i,j))/ Power_total(i,j))*60*10^(-3);%flight time in min

            %C_h(i,j)= ((Power_total(i,j)*(15/60))/(2*S_fr*3.6))*1000;
            %C_ver_climb(i,j)=(Pclimb(i,j)*(climb_alt/(Vc*3.6))/(2*S_fr*3.6))*1000;
            %C_ver_descent(i,j)=(Pdescent(i,j)*(climb_alt/(Vd*3.6))/(2*S_fr*3.6))*1000;
            %C_ff_endu(i,j)= (Pendu(i,j)*(range/Vendu(i,j))/(2*S_fr*3.6))*1000;
            %C_ff_range(i,j)=(Pft(i,j)*(range/54)/(2*S_fr*3.6))*1000;
            %C_aux=(energy_aux/(2*S_fr*3.6))*1000;
            %C_total(i,j)=(C_h(i,j)+ C_ver_climb(i,j)+ C_ver_descent(i,j)+ 2*C_ff_range(i,j))/batt_reserve;
            %flight_time_min(i,j)=(batt_reserve)*(No_of_battery)*(Nominal_volt)*(60)*(10^-3)*((C_h(i,j)/Power_total(i,j))+ C_ver_climb(i,j)/Pclimb(i,j) + C_ver_descent(i,j)/Pdescent(i,j) + 2*C_ff_range(i,j)/Pft(i,j));

%%

% Weight Estimation:

% Front Rotor Group:

            mrotor_fr_pounds=(0.02638*(Nb_fr^0.6826)*((c_fr(i,j)*3.28)^0.9952)*((R_fr(i)*3.28)^1.3507)*((Vtip_fr(j)*3.28)^0.6563)*(mu^2.5231))/Nb_fr;% mass of rotor blades(in pounds)
            %mrotor_fr_prouty_pounds=0.026*(Nb_fr^0.66)*(c_fr(i,j)*3.28)*((R_fr(i)*3.28)^1.3)*((Vtip_fr(j)*3.28)^0.67);%prouty blade estimation
            mrotor_fr=mrotor_fr_pounds*0.4535;% mass of rotor in kg
            mhub_fr_pounds=0.0135*(Empty*2.2)*(R_fr(i)*3.28)^0.42;% mass of hub and hinge in pounds
            mhub_fr=mhub_fr_pounds*0.4535;%mass ofhub&hinge in kg
            
            RPM(i,j)=Vtip_fr(j)*60/(2*pi*R_fr(i));
            Kv(i,j) = RPM(i,j)/(2*S_fr*3.6);% rpm per volt of one side
            mmotor_fr = 1.07;%((10^4.0499)*(Kv(i,j)^-0.5329))*10^-3;%mass of motor(U15 XXL KV29)
            Imax_fr(i,j)=(Power_hv(i,j)/2)/(2*S_fr*3.6);% maximum current of one side
            mesc_fr =0.294;%0.8421*Imax_fr(i,j)*10^-3;%mass of esc(Thunder 300A 24S)
            m_front=mrotor_fr+mhub_fr+mmotor_fr+mesc_fr;%total front mass

% Fuselage:            
            nult=2; %ultimate load factor from vibhram
            Lf=(L)*3.28+2;% total length of fuselage in ft
            Sf=1.64;%fuselage wetted area in ft^2(22.38)
            Iramp = 1;% raming factor, 1 for no ramp
            mfuselage_pounds = 10.13*((0.001*GTOW(n)*2.20)^0.5719)*(nult^0.2238)*(Lf^0.5558)*(Sf^0.1534)*(Iramp^0.5242);% mass of fuselage using RTL method in pounds
            %mfuselage_pounds = 6.9*((GW(n)*2.2/1000)^0.49)*(Lf^0.61)*(Sf^0.25);
            mfuselage=mfuselage_pounds*0.4535;% conversion from pound to kg

% Trasmission:

            HP_mr=(Power_hv(i,j)/2)/746;%maximum drive system horse power(1.2 times take off horse power)
            a_mr=1;%adjustment factor
            %rpm(i,j)=Vtip_fr(j)*60/(2*pi*R_fr(i));
            z_mr=1;% number of stages in drive system
            kt=1.3;%configuration factor
            k_star=0.35;%weight coefficient value of CH-47C
            nmgb=1;% number of main gear boxes
            a_q=1;%coefficient reflecting excess torque
            m_transmission_pounds=250*a_mr*((HP_mr/RPM(i,j))*(z_mr^0.25)*kt)^0.67;%weight of drive system in pounds(boeing vertol)
            %m_transmission=k_star*nmgb*(a_q*torque_h(i,j)/9.8)^0.8;% tishenko estimation of drive system(considering main gear box only)
            m_transmission=m_transmission_pounds*0.4535;%conversion from pound to kg

% Landing Gear:

            %mlg = 0.015 * GW(n);%boeing vertol formula
            mlg = 0.010 * GTOW(n);% tishenko formula

% Controls and Electrical Components:

            Fcb=2;%1= mechanical type, 2=boosted type
            Fcp=1;%Flight control ballastic tolerance 1=no, 2=yes
            kmrc=26;
            %mcontrols_pounds=0.1657*(Fcb^1.3696)*((c_fr(i,j)*3.28)^0.4481)*(Fcp^0.4469)*((GW(n)*2.20)^0.6865);% weight of controls using RTL method in pounds
            %mcontrols_pounds=36*Nb_fr*((c_fr*3.28)^2.2)*((Vtip_fr*3.28/1000)^3.2);% prouty formula
            mcontrols_pounds=kmrc*((c_fr(i,j)*3.28)*((R_fr(i)*3.28)*Nb_fr*(mrotor_fr*2.2)*10^-3)^0.5)^1.1;% weight of rotor controls plus main actuators(boeing vertol formula)
            %mcontrols_pounds=30*((10^-3)*GW(n)*2.2/2)^0.84;% boeing vertol formula
            %mcontrols_pounds=20*(c_fr(i,j)*3.28*(R_fr(i)*3.28*mrotor_fr_pounds*10^-3)^0.5)^1.1;
            mcontrols=mcontrols_pounds*0.4535;%conversion from pound to kg
            melec=0.02*Empty;% electrical weights

% Batttery:

            sed = 1.2; %MJ/Kg(specific energy density of li-ion battery-260 Wh/Kg)
            Battery =energy_MJ(i,j) / sed;%kg

% Back Rotor Group:

            mrotor_br_pounds=(0.02638*(Nb_br^0.6826)*((c_br(i)*3.28)^0.9952)*((R_br(i)*3.28)^1.3507)*((Vtip_br(j)*3.28)^0.6563)*(mu^2.5231))/Nb_fr;% mass of rotor blades(in pounds)
            mrotor_br=mrotor_br_pounds*0.4535;%mass of rotor blades(in kg)
            mhub_br_pounds=0.0135*(Empty*2.2)*(R_br(i)*3.28)^0.42;%mass of hub and hinge(in pounds)
            mhub_br=mhub_br_pounds*0.4535;%mass of hub&hinge(in kg)
            mmotor_br=mmotor_fr;
            mesc_br=mesc_fr;
            m_back = mrotor_br + mhub_br + mesc_br + mmotor_br;%total back mass

% Fixed Weights:

            m_avionics=0.05*Empty;% mass of avionics
            manti_ice=8*(GTOW(n)/1000);%mass of anti ice equipments
            m_instruments=0.4535*3.5*(GTOW(n)*2.2/1000)^1.3;%mass of instruments
            mfixed=m_avionics+manti_ice+m_instruments;% includes mass of avionics and mass of ribs+rods+payload support, anti ice and equipments

% New Empty Weight Calculation:

            Empty = m_front + mfuselage + mcontrols + melec + mlg + m_back + mfixed + m_transmission;

% New Gross Weight Calculation:

            Gross_new(i,j) =Empty + Payload + Battery ;%new gross weight
            m=m+1;
            n=n+1;
            GTOW(n) = Gross_new(i,j);
            error= abs(GTOW(n)-GTOW(m));
 
        end
        
        % Plotting Parameters:

            Rp(i,j) = R_fr(i);% radius
            Vp(i,j) = Vtip_fr(j);%tip speed
            Thp(i,j) = Thrust_total(i,j);%total thrust
            Erp(i,j) = err(i,j);%error margin
            Tp(i,j) = Torque_hv(i,j);% total torque
            Op(i,j) = RPM(i,j);%rpm of blade
            PMp(i,j) = Power_h_mech_hv(i,j);
            PEp(i,j) = Power_hv(i,j);%total hover power
            Gp(i,j) = Gross_new(i,j);%gross weight
            Ep(i,j) = energy(i,j);%total energy
            Bp(i,j) = Battery;%total battery mass
            Colp(i,j) = Theta_hv(i,j)*180/pi;%collective in degrees
            PLp(i,j) = Thrust_total(i,j)./Power_hv(i,j);%power loading(N/W)
            DLp(i,j) = (Thrust_total(i,j)/2)./(pi.*Rp(i,j).^2);%disk loading(N/m^2)
            %O8(i,j) = power_mech(i,j);
    end
end


% Plotting of Design Parameters:
set(groot,'defaultLineLineWidth',3);
set(groot,'defaultAxesFontSize',16);

figure(1)
surf(Rp,Vp,PEp,PLp);
hold on;
title('Hover Power vs Tip speed vs Blade radius Plot')
xlabel("Blade radius(m)");2
ylabel("Tip speed in(m/s)");
zlabel("Hower Power(W)");
% dim = [0.2 0.5 0.3 0.3];
% str = {'AR:8','AR:10','AR:12'};
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
%legend("AR 8","AR 10", "AR 12");
%shading interp;
grid on;
cb=colorbar;
cb.Label.String="Power Loading(N/W)";
% 
% figure(2)
% surf(Rp,Vp,Gp,PLp);
% hold on;
% title('Gross weight vs Tip speed vs Blade radius Plot')
% xlabel("Blade radius(m)");
% ylabel("Tip speed in(m/s)");
% zlabel("Gross weight (kg)");
% %legend("AR 8","AR 10", "AR 12");
% %shading interp;
% grid on;
% cb=colorbar;
% cb.Label.String="Power Loading(N/W)";
% 
figure(3)
surf(Rp,Vp,DLp,PLp);
hold on;
title('Disk loading vs Tip speed vs Blade radius Plot')
xlabel("Blade radius(m)");
ylabel("Tip speed in(m/s)");
zlabel("Disk loading(N/m^2)");
%legend("AR 8","AR 10", "AR 12");
%shading interp;
grid on;
cb=colorbar;
cb.Label.String="Power Loading(N/W)";

figure(4)
surf(Rp,Vp,Colp,PLp);
hold on;
title('Collective vs Tip speed vs Blade radius  Plot')
xlabel("Blade radius(m)");
%xlabel("Tip speed in(m/s)");
ylabel("Tip speed in(m/s)");
%ylabel("Blade radius(m)");
zlabel("Collective \theta (deg)");
% dim = [0.2 0.5 0.3 0.3];
% str = {'AR:8','AR:10','AR:12'};
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
%legend("AR 8","AR 10", "AR 12");
%shading interp;
grid on;
cb=colorbar;
cb.Label.String="Power Loading(N/W)";

% % hold on;
% plot(Op, PEp);
% xlabel("rpm");
% ylabel("Power(W)");
% hold on;
% for z=1:size(R_fr,2)
% plot(Thp(z,:),Erp(z,:));
% hold on;

%%
% Defining BEMT Code as Function:
%%
function [Thrust_h, Power_h, Power_h_mech, Torque_h, Theta_h, err, FOM, BL]=BEMT(Rbt,CT_h,Nb,c,Vtip,sos,GTOW,trans_loss,nondp,motor_eff,nondt,theta_0,elec_loss)

%-------------------- BEMT Code -------------%
% Given data:
%rho = 1.225;                         % Density of air in kg/m^3
%Nb = 2;                              % Number of blades
%Rbt = 3.81;                          % Blade radius in m
Cd0 = 0.011;                          % Coefficient of profile drag
Cl_alpha = 5.73; %6.96;                % Lift curve slope
%vtip = 79.8576;                      % Tip speed in m/s   
%DL = 119.7006;                       % Disk Loading in N/m^2
%CT_hover = DL / (rho * vtip^2);      % CT value for hover case
R_cut = 0.2 * Rbt;                    % 20% of R (Root cutout) in m
A = pi * Rbt^2;                       % Rotor disc area
%sigma = 0.076;                       % Rotor solidity
sigma = Nb*c*(Rbt-R_cut)/(pi*Rbt^2);  % Rotor solidity
omega = Vtip/Rbt;                     % Angular Velocity in rad/s

% For non-dimensionalizing Thrust and Pressure:
% Nonct = rho * A * (vtip^2);
% Noncp = rho * A * (vtip^3);

% Initial lambda and theta0: 
%lambda_h = sqrt(CT_hover / 2);
%theta0 = (1:1:15) * pi / 180;

% Number of segments
nos = 20;
dr = (1 - 0.2) / nos;
er = 1e-6;

%% Preallocate Arrays
CT = zeros(nos, 1);
CP = zeros(nos, 1);
CT_TL = zeros(nos, 1);
CP_TL = zeros(nos, 1);

% Gaussian Quadrature - 6-point rule
x = [-0.9324695142, -0.6612093865, -0.2386191861, 0.2386191861, 0.6612093865, 0.9324695142]; % Points
w = [0.1713244924, 0.3607615730, 0.4679139346, 0.4679139346, 0.3607615730, 0.1713244924];    % Weights

%% Numerical Solution
%for t = 1:length(theta0)
    for N = 1:nos                                     % Loop over Number of segments
        a = 0.2 + dr * (N - 1);
        b = 0.2 + dr * N;
        r_f = ((b - a) / 2) * x + ((a + b) / 2);      % Non-dimensional radial positions
        theta_tw = 0*pi/180;                          % No twist case
        theta = theta_0 + theta_tw * (r_f - 0.75);    % Pitch angle

        for i = 1:6  % Loop over Gaussian points
            theta_fwot = theta(i) * r_f(i);           % Pitch angle times r_f
            lambda_fwot = (sigma * Cl_alpha / 16) * (sqrt(1 + (32 * theta_fwot / (sigma * Cl_alpha))) - 1);
            e = 1000;  % Error initialization
            lambda_wt = lambda_fwot;                  % Initial guess for F = 1

            while e > er
                f = (Nb / 2) * ((1 - r_f(i)) / lambda_wt);
                F = (2 / pi) * acos(exp(-f));         % Prandtl tip-loss factor
                lambda_new = (sigma * Cl_alpha / (16 * F)) * (sqrt(1 + (32 * F * theta_fwot / (sigma * Cl_alpha))) - 1);
                e = abs(lambda_wt - lambda_new);
                lambda_wt = lambda_new;
            end

            % Calculation of Parameters:

                phi(i) = lambda_wt/r_f(i);            % Inflow angle 
                alpha(i) = theta(i)-phi(i);           % Angle of Attack in radian
                AoA = alpha(i)*180/pi;                % Angle of Attack in degree
                Ut = r_f(i)*omega*Rbt;                % Tangential component of velocity
                Up = omega*lambda_wt*Rbt;             % Perpendicular component of velocity
                U = sqrt(Up^2+Ut^2);                  % Resultant velocity
                M = U/sos;                            % Mach speed

            % Calculate thrust and power distributions
            CT_f = 0.5 * sigma * Cl_alpha * (theta_fwot - lambda_fwot) * r_f(i);
            CT_TL_f = 0.5 * sigma * Cl_alpha * (theta_fwot - lambda_wt) * r_f(i);
            CP_f = (lambda_fwot * CT_f) + (0.5 * sigma * Cd0 * (r_f(i)^3));
            CP_TL_f = (lambda_wt * CT_TL_f) + (0.5 * sigma * Cd0 * (r_f(i)^3));

            % Numerical integration for CT and CP
            CT(N) = CT(N) + w(i) * CT_f * ((b - a) / 2);
            CT_TL(N) = CT_TL(N) + w(i) * CT_TL_f * ((b - a) / 2);
            CP(N) = CP(N) + w(i) * CP_f * ((b - a) / 2);
            CP_TL(N) = CP_TL(N) + w(i) * CP_TL_f * ((b - a) / 2);
        end
    end
%end

% Calculation of Total Parameters:

Total_CT=sum(CT_TL);                                         % Total CT with Tip loss
Total_CP=sum(CP_TL);                                         % Total CP with Tip loss
% sum_CP0=sum(CP0_TL);
% sum_CPi=sum(CPi_TL);
Thrust_h= 2*nondt*Total_CT;                                  % Total thrust required for Hover
Power_h=2*trans_loss*elec_loss*(Total_CP)*nondp/motor_eff;   % Total electrical power required for Hover
Power_h_mech=2*(Total_CP)*nondp;                             % Total mechanical power available for Hover
Torque_h=Power_h/omega;                                      % Total torque available for Hover
lambda_TL= sqrt(Total_CT/2);                                 % Inflow ratio with tip loss
Theta_h=((6*Total_CT/sigma/Cl_alpha)+(3*lambda_TL)/2)*180/pi;% Collective Pitch Angle with tip loss in degrees
err=Thrust_h-GTOW*9.81;                                      % Error margin for Gross Weight and Thrust
FOM=(Total_CT^(1.5)/sqrt(2))/((sigma*Cd0/8)+(1.15*Total_CT^(1.5))/sqrt(2));% Figure of merit
BL=Total_CT/sigma;                                           % Ratio of CT to Rotor Solidity

end
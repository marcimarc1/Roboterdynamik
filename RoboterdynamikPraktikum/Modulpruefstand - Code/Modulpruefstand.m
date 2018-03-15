%% Workspace bereinigen
clear all
close all

%% Konfiguration
% Signaltyp der Fuehrungsgroesse phi_g,soll
Signaltyp = 2;      % 0...Null
                    % 1...Sprung
                    % 2...Rampe
                    % 3...Sinus
                    
% (De-)Aktivierung der Last
Last = true;       % false...ohne Last
                    % true ...mit Last

% Messrauschen (und Wert-Diskretisierung)
Messrauschen = false;   % false...ohne Messrauschen
                        % true ...mit Messrauschen

% Regelungsvariante
Variante = 1;       % 1...P-Positionsregler
                    % 2...PI-Positionsregler

%% Parametrisierung Simulationsmodell
% Abtastzeit (= Zeitschrittweite fuer Simulation)
T_A = 1e-5;         % [s]

% Erdbeschleunigung
g = 9.81;           % [m/s^2]

% Anfangswerte
I_qstart = 0;       % [A]
omega_mstart = 0;   % [rad/s]
phi_mstart = 0;     % [rad]

%% Parametrisierung Leistungsverstaerker
% Maximalspannung
U_max = 100;    % [V]           G-WHI 20/100EE: Maximum DC Operating Voltage = 100V

% Zeitkonstante (PT1-Verhalten - als Approximation von Totzeit des Leistungsverstaerkers)
T_LV = 1e-4;

%% Parametrisierung Motor, Getriebe und Last
% Polpaarzahl
p = 4;          % []            PB K064025-GY_

% Statorinduktivitaet in q-Richtung
L_q = 6.503e-3; % [H]           PB K064025-GY_     

% Statorwiderstand
R = 9.641;      % [Ohm]         PB K064025-GY_

% magnetischer Fluss der Permanentmagnete
psi_PM = 0.028; % [Wb]          PB K064025-GY_

% Maximalstrom (thermisch begrenzt)
I_max = 11.6;   % [A]

% Getriebeuebersetzung
N = 100;        % []            HFUC-17

% Massentraegheitsmoment von Motor und Getriebe (auf Motorseite bezogen)
J = 1.385e-5;   % [kgm^2]       PB K064025-GD und HFUC-17

% Masse der Last
if Last == true
    m_Last = 1;     % [kg]
else
    m_Last = 0;     % [kg]
end

% Hebelarm der Last
l_Last = 0.3;   % [m]

% Massentraegheitsmoment der Last (auf Motorseite bezogen)
J_Last = m_Last*l_Last^2/(N^2);

% Reibkoeffizient (viskose Reibung)
if Last == true
    b_Reib = 5e-4;  % [Nms/rad]
else
    b_Reib = 0;     % [Nms/rad]
end

%% Parametrisierung Inkremental-Drehgeber
% Anzahl Schritte (Aufloesung)
n_ink = 11520;  % [Schritte/Umdrehung]

% Messrauschen (Signalstaerke)
r_ink = 1e-5;

%% Parametrisierung Absolut-Drehgeber
% Anzahl Schritte (Aufloesung)
n_abs = 2^(17); % [Schritte/Umdrehung]

% Messrauschen (Signalstaerke)
r_abs = 1e-5;
 
%% Parametrisierung Stromsensor
% Messrauschen (Signalstaerke)
r_strom = 1e-9;

%% Parametrisierung Messwertaufbereitung
% Tiefpassfilter Winkelgeschwindigkeit
% (zum Ausgleich von Fehlern durch numerische Differentiation)
T_omegaTP = 10*T_A;     

%% Reglerauslegung
% Berechnung von Hilfsgroessen
T_el = L_q/R;

% Stromregler
V_IR = L_q/(2*T_LV);
T_In = T_el;
Kp_I = V_IR;                        % P-Anteil
Ki_I = V_IR/T_In;                   % I-Anteil

% Drehzahlregler
V_omegaR = J/(6*p*psi_PM*T_LV);
T_omegan = 8*T_LV;
Kp_omega = V_omegaR;                % P-Anteil
Ki_omega = V_omegaR/T_omegan;       % I-Anteil

% Positionsregler
if Variante == 1
    % P-Positionsregler (Variante 1)
    V_phiR = 1/(16*T_LV);
    Kp_phi = V_phiR;                % P-Anteil
    Ki_phi = 0;                     % I-Anteil
end
if Variante == 2
    % PI-Positionsregler (Variante 2)
    V_phiR = 1/(16*T_LV);
    T_phin = 32*T_LV;
    Kp_phi = V_phiR;                % P-Anteil
    Ki_phi = V_phiR/T_phin;         % I-Anteil
end

%% Simulink Modell ausfuehren
% Modell oeffnen
open_system('Modulpruefstand_sim');

% Modell simulieren
sim('Modulpruefstand_sim');

%% Ausgabe von Statistiken
% Ueberpruefung Maximalstrom (I_q)
I_qmessmax=max(I_qmess(:,2));
if I_qmessmax>I_max
    disp('Warnung: Die Grenze fuer den Motorstrom (I_q) wurde ueberschritten!');
end

% Ueberpruefung Maximalspannung (U_d)
U_didealmax=max(U_dideal(:,2));
if U_didealmax>U_max
    disp('Warnung: Die Grenze fuer die Motorspannung (U_d) wurde ueberschritten!');
end

% Ueberpruefung Maximalspannung (U_q)
U_qidealmax=max(U_qideal(:,2));
if U_qidealmax>U_max
    disp('Warnung: Die Grenze fuer die Motorspannung (U_q) wurde ueberschritten!');
end

% Ausgabe des maximalen Gelenkwinkelfehlers
fprintf('Der maximale Gelenkwinkelfehler betraegt %f¡ã\n',max(phi_gfehler(:,2)));

% Ausgabe des maximalen Motorstroms
fprintf('Der maximale Motorstrom (I_q) betraegt %fA\n',max(I_qmess(:,2)));

%% Plots erzeugen
% Regelabweichung Gelenkwinkel 
figure('name','Gelenkwinkelfehler');
hold on
title('Gelenkwinkelfehler')
plot(phi_gfehler(:,1),phi_gfehler(:,2),'-r');
xlabel('Zeit [s]') 
ylabel('Gelenkwinkelfehler [grad]') 

% Vergleich von Soll- und gemessenem Gelenkwinkel
figure('name','Vergleich Gelenkwinkel (Messung-Soll)');
hold on
title('Vergleich Gelenkwinkel')
plot(phi_gmess(:,1),phi_gmess(:,2),'-r');
plot(phi_gsoll(:,1),phi_gsoll(:,2),'-b');
xlabel('Zeit [s]') 
ylabel('Gelenkwinkel [grad]') 
legend({'Messung','Soll'},'Location','southeast')

%Aufraeumen
close all;
clear all;

addpath('C:\Users\DELL\Documents\MATLAB\P_Roboterdynamik\Code_Direkte_Kinematik');
addpath('C:\Users\DELL\Documents\MATLAB\P_Roboterdynamik\Inverse_Kinematik_Code');
addpath('C:\Users\DELL\Documents\MATLAB\P_Roboterdynamik\Code_Bahnplanung');

%% 1. Parameter und Strukturen initialisieren

%Lade Sollbahn im Arbeitsraum in Workspace
load( 'Gelenkwinkelbahn.mat' );

%Erzeuge Roboterstruktur
rob = erstelle_roboter();

%Datenmatrix fuer Viewer - Sollbahn
V = zeros(3,4,6,length(T));


%% Inverse Dynamik

%Variable zur Speicherung der berechneten Gelenkmomente
Tau_id = zeros( size(Q) );

%Berechne Gelenkmomentenverlauf fuer gegebene Bahn
for i=1:length(T)
    
    %Setze Gelenkwinkel und Zeit
    rob.q = Q(:,i);
    rob.dot_q = dot_Q(:,i);
    rob.ddot_q = ddot_Q(:,i);
    
    rob.zeit = T(i);
 
    %Berechne inverse Dynamik
    rob = berechne_id(rob);
    
    %Gelenkmomente zur spaeteren Analyse speichern
    Tau_id(:,i) = rob.tau_id;   
    
    % Speichere Vektoren B0_r_i und Transformationsmatrizen A_i0 fuer Visualisierung im Viewer
    for l = 1:6
        V( :, 1, l, i ) = rob.kl(l).B0_r_i;
        V( :, 2:4, l, i ) = rob.kl(l).A_i0;
    end
end

%Drehmomentenverlauf plotten
figure();
plot( T, Tau_id(1,:), ...
      T, Tau_id(2,:), ...
      T, Tau_id(3,:), ...
      T, Tau_id(4,:), ...
      T, Tau_id(5,:), ...
      T, Tau_id(6,:) );
  
h=legend( '$\tau_1$','$\tau_2$','$\tau_3$','$\tau_4$','$\tau_5$','$\tau_6$');
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$\tau$ in [Nm]','Interpreter','latex');

%Winkelgeschwindigkeit plotten
figure();
plot( T, dot_Q(1,:), ...
      T, dot_Q(2,:), ...
      T, dot_Q(3,:), ...
      T, dot_Q(4,:), ...
      T, dot_Q(5,:), ...
      T, dot_Q(6,:) );

h=legend( '$\dot q_1$','$\dot q_2$','$\dot q_3$','$\dot q_4$','$\dot q_5$','$\dot q_6$' );
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$\dot{q}$ in [rad/s]','Interpreter','latex');

%% Direkte Dynamik

%Setze Roboter auf Anfangsposition
rob.q = Q(:,1);
rob.dot_q = dot_Q(:,1);
rob.ddot_q = ddot_Q(:,1);

%Variablen zur Speicherung der berechneten Bahn
Q_vd = zeros( size(Tau_id) );
dot_Q_vd = zeros( size(Tau_id) );
ddot_Q_vd = zeros( size(Tau_id) );

%Datenmatrix fuer Viewer - Istbahn
V_vd = zeros(3,4,rob.N_Q,length(T));

%Berechne Bahn aus Drehmomenten der inversen Dynamik
for j=1:length(T)
    
    %Setze Antriebsmoment
    rob.tau_reg = Tau_id(:,j);
    
    %Setze die aktuelle Zeit
    rob.zeit = T(j);
    
    %Berechne Winkelbeschleunigungen (direkte Dynamik)
    rob = berechne_bgl(rob);
    
    %Gelenkwinkelbahn abspeichern
    Q_vd(:,j) = rob.q;
    dot_Q_vd(:,j) = rob.dot_q;
    ddot_Q_vd(:,j) = rob.ddot_q;
    
    %Beschleunigungen und Geschwindigkeiten integrieren
    % Methode 1: mit Driftcompensation in Positionsebene
     if j == 1
        rob.dot_q = rob.dot_q;
        rob.q = rob.q;
     else
        rob.ddot_q = rob.ddot_q + 50*(dot_Q(:,j-1)-rob.dot_q);
        rob.dot_q = rob.dot_q+ rob.ddot_q*rob.dt + 50*(Q(:,j-1)-rob.q);
        rob.q = rob.q + rob.dot_q*rob.dt;
     end
%     % Methode 2: Semi explicit (Numerische Intgration)
%        rob.dot_q= dot_Q_vd(:,j)+rob.ddot_q*rob.dt; %explizit
%        rob.q = Q_vd(:,j)+0.5*(dot_Q_vd(:,j)+rob.dot_q)*rob.dt; %Semi explizit
end

%Speichere die Gelenkwinkel fuer den Viewer in .csv-Datei
write_data(T,V,6,'trajectory_Dynamik_Soll.csv');
write_data(T,V_vd,6,'trajectory_Dynamik_Ist.csv');

%Differenzen der Winkel zur Ueberpruefung plotten
e_Q = Q - Q_vd;
figure();
plot( T, e_Q(1,:), ...
      T, e_Q(2,:), ...
      T, e_Q(3,:), ...
      T, e_Q(4,:), ...
      T, e_Q(5,:), ...
      T, e_Q(6,:) );
  
h=legend( '$e_{q_1}$','$e_{q_2}$','$e_{q_3}$','$e_{q_4}$','$e_{q_5}$','$e_{q_6}$','Location','northwest');
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$e_q$ in [rad]','Interpreter','latex');






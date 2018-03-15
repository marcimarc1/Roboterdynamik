% Workspace aufraeumen
close all;
clear all;

%% 1. Parameter und Strukturen initialisieren
%  ------------------------------------------
% Erzeuge Roboterstruktur (Datenstruktur mit Geometriedaten)
rob = erstelle_roboter();

% Parameter der Bahn
q_a = zeros(rob.N_Q,1);         % Anfangswinkel (in Radiant)
q_e = pi/4 * ones(rob.N_Q,1);   % Endwinkel (in Radiant)
T_ges = 2.0;                    % Gesamtdauer der Bewegung (in Sekunden)

% Setze Anfangsgelenkwinkel
rob.q = q_a;

%% 2. Nacheinander Gelenkwinkel variieren und direkte Kinematik berechnen
%  ----------------------------------------------------------------------
% Definition des Zeitarrays T
% (Gelenke werden nacheinander auf Endposition gefahren)
T_Gelenk = T_ges/rob.N_Q;               % Verfuegbare Zeit pro Gelenk
N_Gelenk = floor(T_Gelenk/rob.dt);      % Anzahl Zeitschritte pro Gelenk
T = rob.dt*(0:(rob.N_Q*N_Gelenk-1));    % Zeitschritt-Vektor

% Initialisierung der Variablen zur Speicherung der TCP-Position und der Gelenkwinkel
Q = zeros(rob.N_Q,length(T));           % Gelenkwinkel-Matrix
W = zeros(3,length(T));                 % TCP-Matrix (Position)
dot_W = zeros(3,length(T));             % TCP-Matrix (Geschwindigkeit)
V = zeros(3,4,rob.N_Q,length(T));       % Datenmatrix fuer Viewer

% Nacheinander alle Gelenkwinkel linear auf Endposition fahren
tic % Zeitmessung: Start
for i = 1:rob.N_Q
    % Gelenkwinkelinkrement berechnen
    delta_q = (q_e(i)-q_a(i))/N_Gelenk;    
    
    % Aktuelle Geschwindigkeit der Gelenke setzen (nur ein Gelenk bewegt sich)
    rob.dot_q = zeros(6,1);
    rob.dot_q(i) = (q_e(i)-q_a(i))/T_Gelenk;

    % Ueber alle Zeitschritte dieses Gelenks iterieren
    for k = 1:N_Gelenk
        % Winkel von Gelenk i im Zeitschritt k setzen
        % (alle anderen Gelenke bleiben wo sie gerade sind)
        rob.q(i) = rob.q(i) + delta_q;
        
        % Simulationszeit aktualisieren
        rob.zeit = rob.zeit + rob.dt;
        
        % Hinweis:
        % --------
        % Die Koerper muessen entsprechend der Baumstruktur wie folgt sortiert werden:
        % ist Koerper A naeher an der Wurzel als Koerper B, so belegt der Freiheitsgrad
        % von A im Vektor der Freiheitsgrade q einen kleineren Index als B
        % ==> Dann kann die Kinematik in einer Schleife berechnet werden,
        %     wobei stets gewaehrleistet ist, dass die Kinematik des
        %     Vorgaenger-Koerpers bereits berechnet wurde

        %% --- ARBEITSBEREICH: --------------------------------------------
        % Kommentieren Sie hier die entsprechenden Zeilen aus, um Ihre
        % Funktionen zu testen
        % -----------------------------------------------------------------
        % Aufgabe 2.1: Direkte Kinematik mit Vektorkette
%         rob = berechne_dk_positionen_vektorkette(rob);

        % Aufgabe 2.2: Direkte Kinematik mit homogenen Transformationsmatrizen
%         rob = berechne_dk_positionen_dh_trafo(rob);

         % Aufgabe 2.3: Direkte Kinematik auf Geschwindigkeitsebene 
%          rob = berechne_dk_geschwindigkeiten(rob);
             % Position des TCPS fuer die Trajektorie(Zur Uerberpruefung der Geschwindigkeit)
             if k == 1 && i == 1
                 rob.w = [0.287;0;-0.289];         % Initialposition des TCPs
             else
                 rob.w = rob.w + rob.dot_w*rob.dt; % Numerische Intgration der Geschwindigkeit
             end

        % Aufgabe 3 Bonusaufgabe: Effiziente Berechnung der direkten Kinematik auf Positionsebene
%         rob = berechne_dk_positionen_effizient(rob,i,k);
        %% --- ENDE ARBEITSBEREICH ----------------------------------------

        % Gelenkwinkel, Arbeitsraumkoordinaten- und geschwindigkeiten speichern fuer Analyse
        Q(:,N_Gelenk*(i-1)+k) = rob.q;
        W(:,N_Gelenk*(i-1)+k) = rob.w;
        dot_W(:,N_Gelenk*(i-1)+k) = rob.dot_w;

        % Vektoren B0_r_i und Transformationsmatrizen A_i0 fuer Viewer speichern
        for l = 1:6
            V(:,1,l,N_Gelenk*(i-1)+k) = rob.kl(l).B0_r_i;
            V(:,2:4,l,N_Gelenk*(i-1)+k) = rob.kl(l).A_i0;
        end
    end
end
toc % Zeitmessung: Ende

%% 3. Plotte und visualisiere Bewegung
%  -----------------------------------
% Speichere die Gelenkwinkel fuer den Viewer
write_data(T,V,6,'trajectory_D.csv');

% Visualisierung der Bahn im Arbeitsraum
figure();
plot3(W(1,:),W(2,:),W(3,:));
xlabel('x /m')
ylabel('y /m')
zlabel('z /m')
grid on
axis square
legend('Bahn im Arbeitsraum');
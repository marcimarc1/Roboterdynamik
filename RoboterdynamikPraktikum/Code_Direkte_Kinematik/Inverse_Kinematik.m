% Workspace aufraeumen
close all;
clear all;

%% --- ARBEITSBEREICH: ------------------------------------------------
% Setzen Sie hier Parameter und waehlen Sie Ausfuehrungsoptionen aus
% ---------------------------------------------------------------------
%% 1. Optionen fuer Ausfuehrung setzen
%  -----------------------------------
PlotsErzeugen = true;   % Sollen Plots erzeugt werden?
PlotsSpeichern = false; % Sollen Plots gespeichert werden?
IKVerfahren = 'rmc';    % Verfahren der Inversen Kinematik
                        % 'rmc'     ...Resolved Motion Rate Control
                        % 'asc'     ...Automatic Supervisory Control
                        % 'asceff'  ...Automatic Supervisory Control (effizient)
                        % 'xxx'     ...Platzhalter
IKOption = 'driftcomp';     % Optionen fuer das IKVerfahren
                        % 'drift'     ...ohne Driftkompensation
                        % 'driftcomp' ...mit Driftkompensation
                        % 'comfort'   ...mit Komfort-Pose
                        % 'limit'     ...mit Gelenkwinkelbeschraenkungen
                        % 'both'      ...mit Komfort-Pose UND Gelenkwinkelbeschraenkungen
                        % 'VA'        ...Platzhalter
                        % 'VB'        ...Platzhalter
% Parameter Resolved Motion Rate Control
W_rmc = eye(6);                     % Wichtungsmatrix
K_rmc = 50*eye(3);                  % Driftkompensationsmatrix

% Parameter Automatic Supervisory Control
W_asc = diag([10,10,10,10,1,1]);    % Wichtungsmatrix
K_asc = 50*eye(3);                  % Driftkompensationsmatrix
alpha = 10;                         % Wichtungsfaktor
%% --- ENDE ARBEITSBEREICH --------------------------------------------

%% 2. Parameter und Strukturen initialisieren
%  ------------------------------------------
% Soll-Trajektorie in Workspace laden
if strcmp(IKVerfahren,'xxx') == true
    load('Solltrajektoriexxx.mat');
else
    load('Solltrajektorie.mat');
end
% Die Datei enthaelt:
% --> W_d       ...TCP Soll-Trajektorie (Position)
% --> dot_W_d   ...TCP Soll-Trajektorie (Geschwindigkeit)
% --> dt        ...Zeitschrittweite in Sekunden
% --> T         ...Zeitschritt-Vektor
% --> q0        ...Initiale Gelenkwinkel

% Erzeuge Roboterstruktur (Datenstruktur mit Geometriedaten)
rob = erstelle_roboter();
rob.dt = dt;                % Zeitschrittweite aus Solltrajektorie uebernehmen
rob.q = q0;                 % Initiale Gelenkwinkel uebernehmen

% Initiale Berechnung der Direkten Kinematik auf Positionsebene (notwendig fuer Berechnung der Jacobis)
rob = berechne_dk_positionen_vektorkette(rob);

% Initiale Berechnung der Jacobi-Matrizen
rob = berechne_dk_jacobis(rob,IKVerfahren);

% Initialisierung von Variablen fuer Analyse
Q = zeros(rob.N_Q,length(T));           % Gelenkwinkel-Matrix
dot_Q = zeros(rob.N_Q,length(T));       % Gelenkwinkelgeschwindigkeit-Matrix
W = zeros(3,length(T));                 % TCP Ist-Trajektorie (Position)
dot_W = zeros(3,length(T));             % TCP Ist-Trajektorie (Geschwindigkeit)
delta_W = zeros(1,length(T));           % Fehler von TCP Position (Norm(Ist-Soll))
V = zeros(3,4,rob.N_Q,length(T));       % Datenmatrix fuer Viewer

%% 3. Berechnung der Inversen Kinematik
%  ------------------------------------
tic % Zeitmessung: Start
% Iteration ueber alle Zeitschritte
for i = 1:length(T)
    % Aktuelle Werte aus Solltrajektorie uebernehmen
    rob.zeit = T(i);
    rob.w_d = W_d(:,i);
    rob.dot_w_d = dot_W_d(:,i);

    % Resolved Motion Rate Control
    % ----------------------------
    if strcmp(IKVerfahren,'rmc') == true
        rob = berechne_ik_rmc(rob,W_rmc,K_rmc,IKOption);
    end

    % Automatic Supervisory Control
    % -----------------------------
    if strcmp(IKVerfahren,'asc') == true
        rob = berechne_ik_asc(rob,W_asc,K_asc,alpha,IKOption);
    elseif strcmp(IKVerfahren,'asceff') == true
        rob = berechne_ik_asc_effizient(rob,W_asc,K_asc,alpha,IKOption);
    end

    % Platzhalter
    % -----------
    if strcmp(IKVerfahren,'xxx') == true
        rob = berechne_ik_xxx(rob,IKOption);
    end

    % Update der direkten Kinematik, Jacobis und TCP Geschwindigkeit
    % (fuer Analyse sowie Berechnung der Inversen Kinematik im naechsten Zeitschritt)
    rob = berechne_dk_positionen_vektorkette(rob);  % Direkte Kinematik
    rob = berechne_dk_jacobis(rob,IKVerfahren);     % Jacobi-Matrizen
    rob.dot_w = rob.Jw*rob.dot_q;                   % TCP Ist-Geschwindigkeit berechnen

    % Gelenkwinkel, Arbeitsraumkoordinaten- und geschwindigkeiten speichern fuer Analyse
    Q(:,i) = rob.q;
    dot_Q(:,i) = rob.dot_q;
    W(:,i) = rob.w(1:3);
    dot_W(:,i) = rob.dot_w(1:3);

    % Arbeitsraumfehler der TCP Position berechnen
    delta_W(i) = norm(rob.w-rob.w_d(1:3));

    % Vektoren B0_r_i und Transformationsmatrizen A_i0 fuer Viewer speichern
    for l = 1:rob.N_Q
        V(:,1,l,i) = rob.kl(l).B0_r_i;
        V(:,2:4,l,i) = rob.kl(l).A_i0;
    end
end
toc % Zeitmessung: Ende

%% 4. Plotte und visualisiere Bewegung
%  -----------------------------------
% Speichere die Gelenkwinkel fuer den Viewer
write_data(T,V,6,['trajectory_IK_',IKVerfahren,'_',IKOption,'.csv']);

% Plots erzeugen
if PlotsErzeugen == true
    %% Ausgabeordner erstellen
    if exist('plots','dir') ~= 7
        mkdir('plots');
    end

    %% Sollbahn und Istbahn plotten (3D)
    %  ---------------------------------
    h = figure();
    title('Soll- und Ist-Bahn im Arbeitsraum')
    hold on;
    plot3(W(1,:),W(2,:),W(3,:));        % Istbahn plotten
    plot3(W_d(1,:),W_d(2,:),W_d(3,:));  % Sollbahn plotten

    % Anfangs- und Endpunkt der Sollbahn markieren
    plot3(W_d(1,1),W_d(2,1),W_d(3,1),'o');
    plot3(W_d(1,end),W_d(2,end),W_d(3,end),'*');

    % Anfangs- und Endpunkt der Istbahn markieren
    plot3(W(1,1),W(2,1),W(3,1),'o');
    plot3(W(1,end),W(2,end),W(3,end),'*');
    hold off;

    % Legende plotten
    legend('Location','northeast')
    legend('Ist-Bahn','Soll-Bahn');
    xlabel('x /m')
    ylabel('y /m')
    zlabel('z /m')
    grid on
    axis square
    view(50,40)

    % Plot speichern
    if PlotsSpeichern == true
        set(h,'PaperPositionMode','Auto','PaperUnits','Centimeters','PaperSize',[10 10])
        print(h,['plots/Plot_Bahn3D_',IKVerfahren,'_',IKOption],'-fillpage','-dpdf')
    end

    %% Trajektorien des TCPs plotten (komponentenweise)
    %  ------------------------------------------------
    h = figure;
    title('Position des TCPs im Arbeitsraum')
    hold on;
    plot(T,W,'-');      % Ist-Trajektorie plotten
    plot(T,W_d,'-.');   % Soll-Trajektorie plotten
    hold off;
    legend('Location','northeastoutside')
    legend('x_{ist}(t)','y_{ist}(t)','z_{ist}(t)',...
           'x_{soll}(t)','y_{soll}(t)','z_{soll}(t)');
    xlabel('t /s')
    ylabel('X/Y/Z-Position /m')

    % Plot speichern
    if PlotsSpeichern == true
        set(h,'PaperPositionMode','Auto','PaperUnits','Centimeters','PaperSize',[15 10])
        print(h,['plots/Plot_TCP_Pos_',IKVerfahren,'_',IKOption],'-fillpage','-dpdf')
    end

    %% Geschwindigkeit des TCPs plotten (komponentenweise)
    %  ---------------------------------------------------
    h = figure;
    title('Geschwindigkeit des TCPs im Arbeitsraum')
    hold on;
    plot(T,dot_W,'-');      % Ist-Geschwindigkeit plotten
    plot(T,dot_W_d,'-.');   % Soll-Geschwindigkeit plotten
    hold off;
    legend('Location','northeastoutside')
    legend('u_{ist}(t)','v_{ist}(t)','w_{ist}(t)',...
           'u_{soll}(t)','v_{soll}(t)','w_{soll}(t)');
    xlabel('t /s')
    ylabel('X/Y/Z-Geschwindigkeit / m/s')

    % Plot speichern
    if PlotsSpeichern == true
        set(h,'PaperPositionMode','Auto','PaperUnits','Centimeters','PaperSize',[15 10])
        print(h,['plots/Plot_TCP_Geschw_',IKVerfahren,'_',IKOption],'-fillpage','-dpdf')
    end

    %% Abweichung des TCP von der Solltrajektorie plotten
    %  --------------------------------------------------
    h = figure;
    title('Positionsabweichung im Arbeitsraum')
    hold on;
    semilogy(T,delta_W,'--r');              % Fehler plotten
    [delta_w_max,ind] = max(delta_W);       % maximalen Fehler identifizieren
    semilogy(T(ind),delta_w_max,'x');       % maximalen Fehler markieren
    hold off;
    legend('Location','southeast')
    legend('Arbeitsraumfehler','Max. Arbeitsraumfehler');
    xlabel('t /s')
    ylabel('|\Delta w| /m')

    % Plot speichern
    if PlotsSpeichern == true
        set(h,'PaperPositionMode','Auto','PaperUnits','Centimeters','PaperSize',[10 10])
        print(h,['plots/Plot_Fehler_',IKVerfahren,'_',IKOption],'-fillpage','-dpdf')
    end

    %% Gelenkwinkelbahn plotten (komponentenweise)
    %  -------------------------------------------
    h = figure;
    title('Gelenkwinkel')
    hold on;
    plot(T,Q);              % Gelenkwinkel plotten
    hold off;
    legend('Location','northeastoutside')
    legend('q_{1}(t)','q_{2}(t)','q_{3}(t)','q_{4}(t)','q_{5}(t)','q_{6}(t)');
    xlabel('t /s')
    ylabel('q_i /rad')

    % Plot speichern
    if PlotsSpeichern == true
        set(h,'PaperPositionMode','Auto','PaperUnits','Centimeters','PaperSize',[15 10])
        print(h,['plots/Plot_Gelenkwinkel_',IKVerfahren,'_',IKOption],'-fillpage','-dpdf')
    end
end

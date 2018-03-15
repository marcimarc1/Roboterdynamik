function [Q] = berechne_gelenkwinkel(W)

N_points = size(W,2);

rob = erstelle_roboter();
rob.dt = 1;                 % Zeitschrittweite (egal, da IK nur als Newton-Verfahren genutzt)
rob.q = ones(rob.N_Q,1);    % initiale Gelenkwinkel fuer Newton-Verfahren

% Variablen fuer die Funktion berechne_ik_rmc
W_rmc = eye(6);                     % Wichtungsmatrix
K_rmc = eye(3);                     % Driftkompensationsmatrix
rob.dot_w_d = zeros(size(W(:,1)));  % fuer das Newton-Verfahren wird nur die Driftkompensation genutzt

% initialisiere Ergebnismatrix
Q = zeros(rob.N_Q,N_points);

% fuer alle Stuetzpunkte
for i=1:N_points

  % setzt w desired
  rob.w_d = W(:,i);

  % Initiale Berechnung der Direkten Kinematik auf Positionsebene
  % (notwendig fuer Berechnung der Jacobis)
  rob = berechne_dk_positionen_vektorkette(rob);


  % bis gut genug
  k = 0;
  while 1

    % Initiale Berechnung der Jacobi-Matrizenrob
    rob = berechne_dk_jacobis(rob,'rmc');

    % berechne die IK mit der Jacobi-Matrix des aktuellen q
    rob = berechne_ik_rmc(rob,W_rmc,K_rmc,'driftcomp');

    % Berechnung der Direkten Kinematik auf Positionsebene
    % => neues rob.w
    rob = berechne_dk_positionen_vektorkette(rob);

    % Abbruch des Newton-Verfahrens
    % vergleiche Arbeitsraum-Koordinaten und DK und W
    if norm(rob.w-rob.w_d) < 1e-3
      break;
    end

    k = k+1;
    if k>1e3
      error('Newton-Verfahren konvergiert nicht: Eventuell befinden sich die Arbeitsraum-Koordinaten nicht in Reichweite des Roboters.');
    end
  end

  % uebernehme konvergierte Gelenkwinkel
  Q(:,i) = rob.q;

end

end %function

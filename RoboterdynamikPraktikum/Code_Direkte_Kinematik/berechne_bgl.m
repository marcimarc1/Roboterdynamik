function rob=berechne_bgl(rob)
%Berechnung der Bewegungsgleichungen des Systems (M, h und ddot_q)
%
% Alle notwendigen Funktionen werden von hier aus aufgerufen
%


%% 1. Berechnung des h-Vektors:
% Tipp: Nutzen Sie zur Berechnung von h die Funktion "berechne_id.m"
rob.ddot_q = zeros(rob.N_Q,1);
rob = berechne_id(rob);
rob.h = rob.tau_id;

%% 2. Berechnung der Massenmatrix
rob.M=zeros(rob.N_Q,rob.N_Q);

% Beitraege aller Koerper addieren
for i=1:length(rob.kl)
    %Anteil diese Koerpers
    Moi = [rob.kl(i).m*eye(3),                 rob.kl(i).m*tilde(rob.kl(i).Bi_r_s)';...
          rob.kl(i).m*tilde(rob.kl(i).Bi_r_s), rob.kl(i).I_o ];
    dM = [rob.kl(i).Bi_Jt_o;rob.kl(i).Bi_Jr]'*Moi*[rob.kl(i).Bi_Jt_o ; rob.kl(i).Bi_Jr];

    %Anteil zur Gesamt-Massenmatrix addieren 
    rob.M = rob.M + dM;
end

%Die aktuellen Beschleunigungen berechnen
%Hier werden auch die Antriebsmomente der Regelung tau_reg beruecksichtigt
     rob.ddot_q= rob.M\(rob.tau_reg - rob.h);
end

 

function roboter = erstelle_roboterstruktur(n_Elemente)
    % Erzeugt eine (leere) Datenstruktur mit allen notwenigen Variablen

    %% --- Allgemeine Daten/Konstanten ------------------------------------
    roboter.N_Q = n_Elemente;   % Anzahl der Freiheitsgrade (Dimension von q)
    roboter.zeit = 0;           % Aktuelle Simulationszeit
    roboter.dt = 0;             % Zeitschrittweite
    roboter.B0_g = zeros(3,1);  % Erdbeschleunigung

    %% --- Geometriedaten -------------------------------------------------
    % Laengen, Breiten und Dicken der einzelnen Koerper
    roboter.l = zeros(1,6);
    roboter.b = zeros(1,6);
    roboter.d = zeros(1,6);

    %% --- Eigentliche RD-Variablen ---------------------------------------
    % Vektor der generalisierten Koordinaten, Geschwindigkeiten und Beschleunigungen
    roboter.q = zeros(roboter.N_Q,1);
    roboter.dot_q = zeros(roboter.N_Q,1);
    roboter.ddot_q = zeros(roboter.N_Q,1);

    % Arbeitsraum-Koordinaten (Endeffektor-Position): Istwert, Sollwert und Ableitungen
    roboter.w = zeros(3,1);
    roboter.dot_w = zeros(3,1);
    roboter.w_d = zeros(3,1);
    roboter.dot_w_d = zeros(3,1);

    % Vektor der generalisierten (Antriebs-)Kraefte
    roboter.tau_id = zeros(roboter.N_Q,1);
    roboter.tau_antrieb = zeros(roboter.N_Q,1);

    % Massenmatrix und h Vektor fuer Dynamik
    roboter.M = zeros(roboter.N_Q);
    roboter.h = zeros(roboter.N_Q,1);

    % Vektor zum TCP, relativ zum koerperfesten Koordinatensystem des letzten Koerpers
    roboter.BN_r_N_tcp = zeros(3,1);

    % Jacobi-Matrix der Endeffektor-Position
    roboter.Jw = zeros(3,roboter.N_Q);

    % Variablen der einzelnen Koerper anlegen
    for i = 1:roboter.N_Q
        % Vorgaengerkoerper
        roboter.kl(i).vorgaenger = 0;

        % Freiheitsgrad, der vom Koerper belegt wird (d.h. hier theta=q(fhg_no)
        roboter.kl(i).fhg_no = 0;

        % DH-Parameter (alpha, a und d sind Konstanten fuer rein rotatorische Gelenke)
        roboter.kl(i).alpha = 0.0;
        roboter.kl(i).a = 0;
        roboter.kl(i).d = 0;

        % Masse des i-ten Koerpers
        roboter.kl(i).m = 0;

        % Vektoren im inertialen Koordinatensystem
        roboter.kl(i).B0_r_i = zeros(3,1);          % Position des Ursprungs B0-System (inertial)

        % Vektoren im i-ten Koordinatensystem
        roboter.kl(i).Bi_r_s = zeros(3,1);          % Schwerpunkt relativ zum Koerper Kosy-Ursprung
        roboter.kl(i).Bi_dot_r_s = zeros(3,1);      % Absolutgeschwindigkeit des Schwerpunkts
        roboter.kl(i).Bi_ddot_r_s = zeros(3,1);     % Absolutbeschleunigung des Schwerpunkts
        roboter.kl(i).Bi_r_i = zeros(3,1);          % Position des Ursprungs des i-ten Koerpers im KOS des i-ten Koerpers
        roboter.kl(i).Bi_dot_r_i = zeros(3,1);      % Absolutgeschwindigkeit des Ursprungs des i-ten Koerpers im KOS des i-ten Koerpers
        roboter.kl(i).Bi_ddot_r_i = zeros(3,1);     % Absolutbeschleunigung des Ursprungs des i-ten Koerpers im KOS des i-ten Koerpers

        roboter.kl(i).Bi_omega = zeros(3,1);        % absolute Winkelgeschwindigkeit des i-ten Koerpers im i-ten KOS
        roboter.kl(i).Bi_dot_omega = zeros(3,1);    % absolute Winkelbeschleunigung des i-ten Koerpers im i-ten KOS
        roboter.kl(i).Bi_omega_rel = zeros(3,1);    % relativer Anteil
        roboter.kl(i).Bi_dot_omega_rel = zeros(3,1);% Ableitung des relativen Anteils

        roboter.kl(i).Bi_g = zeros(3,1);            % Erdbeschleunigung im System des i-ten Koerpers

        % Vektoren im Vorgaenger-Koordinatensystem
        roboter.kl(i).Bv_r_vi = zeros(3,1);         % Verschiebungsvektor vom Vorgaenger zum Koerper i im KOS des Vorgaengers (Bv)

        % Drehmatrizen
        roboter.kl(i).A_iv = eye(3);                % Drehmatrix vom Vorgaenger zum i-ten Koerper
        roboter.kl(i).A_i0 = eye(3);                % Drehmatrix vom i-ten Koerper ins Inertialsystem (=B0)

        % DH-Matrizen
        roboter.kl(i).D_vi = eye(4);                % Homogene Transformationsmatrix vom i-ten Koerper zum Vorgaenger
        roboter.kl(i).D_0i = eye(4);                % Homogene Transformationsmatrix vom i-ten Koerper ins Inertialsystem

        % Jacobi-Matrizen
        roboter.kl(i).Bi_Jr = zeros(3,length(roboter.q));   % Jacobi-Matrix der Rotation, dargestellt im Bi-KOS (d(Bi_omega)/d(dot(q))
        roboter.kl(i).Bi_Jt_o = zeros(3,length(roboter.q)); % Jacobi-Matrix der Translation, dargestellt im Bi-KOS

        % Traegheitstensoren
        roboter.kl(i).I_g = zeros(3);               % Traegheitstensor bezueglich Schwerpunkts des KOSY
        roboter.kl(i).I_g_Steiner = zeros(3);       % Steiner Anteil aus Offset
        roboter.kl(i).I_o = zeros(3);               % Traegheitstensor bezueglich Ursprung des KOSY
    end
end

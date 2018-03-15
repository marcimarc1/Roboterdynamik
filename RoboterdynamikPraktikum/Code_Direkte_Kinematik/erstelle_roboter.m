function rob = erstelle_roboter()
    % Erzeugt eine (befuellte) Datenstruktur des Roboters (Powercube)

    %% --- Initialisierung Datenstruktur ----------------------------------
    % Erzeuge leere Datenstruktur (Roboter mit sechs Einzelkoerpern/Gelenken)
    rob = erstelle_roboterstruktur(6);

    % Zeitschrittweite
    rob.dt = 0.001;

    % Erdbeschleunigung
    rob.B0_g = [0; 0; -9.81];

    %% --- ARBEITSBEREICH: DH-Parameter -----------------------------------
    % Tragen Sie die DH-Parameter in die folgende Struktur ein.
    % Einheiten:    a und d in Meter
    %               alpha in Radiant
    % ---------------------------------------------------------------------
    rob.kl(1).alpha = 0;
    rob.kl(1).a = 0;
    rob.kl(1).d = 0;

     rob.kl(2).alpha = pi/2;
     rob.kl(2).a = 0;
     rob.kl(2).d = 0;

     rob.kl(3).alpha = pi;
     rob.kl(3).a = 0.184;
     rob.kl(3).d = 0;

     rob.kl(4).alpha = -pi/2;
     rob.kl(4).a = 0.103;
     rob.kl(4).d = 0.196;

     rob.kl(5).alpha = pi/2;
     rob.kl(5).a = 0;
     rob.kl(5).d = 0;

     rob.kl(6).alpha = -pi/2;
     rob.kl(6).a = 0;
     rob.kl(6).d = 0.058;
    %% --- ENDE ARBEITSBEREICH --------------------------------------------

    %% --- Tool-Center-Point (TCP) ----------------------------------------
    % TCP dargestellt im KOSY des letzten Gelenks
    rob.BN_r_N_tcp = [0; 0; 0.035];

    %% --- Geometriedaten des Roboters ------------------------------------
    % Breiten der einzelnen Teilmodule die zu einem Roboterglied kombiniert werden koennen:
    % -------------------------------------------------------------------------------------
    % Grundwuerfel
    BM90 = 0.09;
    BM70 = 0.07;
    % Verbindungsmodule 1:3
    BA1  = 0.09;
    BA2  = 0.09;
    BA3  = 0.09;
    BA4  = 0.07;
    % Handmodul A und B
    BHand_A = 0.07;
    BHand_B = 0.07;

    % Laengen der einzelnen Teilmodule die zu einem Roboterglied kombiniert werden koennen:
    % -------------------------------------------------------------------------------------
    % Grundwuerfel
    LM90 = 0.0902;
    LM70 = 0.0701;
    % Verbindungsmodule 1:3
    LA1  = 0.030;
    LA2  = 0.090;
    LA3  = 0.020;
    LA4  = 0.010;
    % Handmodul A und B
    LHand_A = 0.07;
    LHand_B = 0.116;

    % Geometrie der Roboterglieder
    % ----------------------------
    % Laenge Roboterglied i
    l(1) = LM90+LA1+LM90;
    l(2) = LM90+LA2+LM90;
    l(3) = LM90+LA3+LM70;
    l(4) = LM70+LA4+BHand_A;
    l(5) = 0.196;
    l(6) = 0.035;
    rob.l = l; % Uebertragen in Roboter-Struktur

    % Breite Roboterglied i
    b(1) = BM90;
    b(2) = BM90;
    b(3) = BM90;
    b(4) = BHand_A;
    b(5) = BHand_B;
    b(6) = 0.05;
    rob.b = b; % Uebertragen in Roboter-Struktur

    % Hoehe Roboterglied i
    d = zeros(1,6);
    d(5) = 0.08;
    rob.d = d; % Uebertragen in Roboter-Struktur

    % Schwerpunkt
    % -----------
    x = [b(1); l(2); l(3); b(4); b(5); b(6)];
    y = [b(1); b(2); b(3); b(4); l(5); l(6)];
    z = [l(1); b(2); b(3); l(4); d(5); b(6)];
    x_off = [0; 0.5*l(2)-0.5*b(1); (l(3)-b(2))/2; 0; 0; 0];
    y_off = [0; 0; 0; 0; 0; 0];
    z_off = [-0.5*l(1)+b(2)/2; -(b(1)/2 + b(2)/2); 0; -l(4)/2; 0; b(6)/2];

    % Schwerpunkt relativ zum Koerper Kosy-Ursprung: Entspricht genau den Offset Werten
    % --> Vereinfachung, hier SP = Geom. Mittelpunkt
    for i = 1:rob.N_Q
        rob.kl(i).Bi_r_s= [x_off(i); y_off(i); z_off(i)];
    end

    %% --- Massen und Traegheitstensoren ----------------------------------
    % Massen der einzelnen Teilmodule
    % -------------------------------
    % Grundwuerfel
    MM90 = [2.215 1.194];   % [m_mech m_el]
    MM70 = [0.616 1.217];   % [m_mech m_el]

    % Verbindungsmodule
    MA1  = 0.16;
    MA2  = 0.356;
    MA3  = 0.196;
    MA4  = 0.062;

    % Handmodul A und B
    MHA = [0.238 0];
    MHBC= [1.200 0.303];

    % Massen der Roboterglieder
    % -------------------------
    m(1)= MM90(2) + MA1 + MM90(1);
    m(2)= MM90(2) + MA2 + MM90(1);
    m(3)= MM90(2) + MA3 + MM70(1);
    m(4)= MM70(2) + MA4 + MHA(1);
    m(5)= MHBC(1);
    m(6)= MHBC(2);
    for i = 1:rob.N_Q
        rob.kl(i).m = m(i); % Uebertragen in Roboter-Struktur
    end

    % Traegheitstensoren der Roboterglieder
    % -------------------------------------
    for i=1:rob.N_Q
        % Traegheitstensor bezueglich Schwerpunkts KOSY
        rob.kl(i).I_g = m(i)/12 * [y(i)^2 + z(i)^2,             0,              0; ...
                                                 0, x(i)^2+z(i)^2,              0; ...
                                                 0,             0, x(i)^2+y(i)^2];
        % Steiner Anteil aus Offset
        rob.kl(i).I_g_Steiner = m(i)*[y_off(i)^2+z_off(i)^2,                     0,                     0; ...
                                                          0, x_off(i)^2+z_off(i)^2,                     0; ...
                                                          0,                     0, x_off(i)^2+y_off(i)^2];

        % Traegheitstensor bezueglich der Schwerpunkts und Ursprungs des KOSY
        rob.kl(i).I_o = rob.kl(i).I_g + rob.kl(i).I_g_Steiner;
    end

    %% --- Sonstige Parameter ---------------------------------------------
    for i = 1:rob.N_Q
        if i == 1
            % Der erste Koerper hat keinen Vorgaenger ==> Index = -1
            rob.kl(i).vorgaenger = -1;
        else
            rob.kl(i).vorgaenger = i-1;
        end

        % Freiheitsgrad, der vom Koerper belegt wird (d.h. hier theta = q(fhg_no))
        rob.kl(i).fhg_no = i;
    end
end

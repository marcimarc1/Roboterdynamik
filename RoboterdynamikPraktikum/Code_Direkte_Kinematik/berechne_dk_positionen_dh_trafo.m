function rob = berechne_dk_positionen_dh_trafo(rob)
    % Berechnung der Positions-Groessen der direkten Kinematik (Lage und Orientierung)
    % ueber die DH Transformation

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % Ersetzen Sie die nachfolgenden Rueckgaben durch Ihre Berechnungen
    % ---------------------------------------------------------------------
    % Berechnung fuer alle Koerper
    for i = 1:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;

        % Relativkinematik: Position und Orientierung relativ zum Vorgaenger
        % ------------------------------------------------------------------
        % Homogene Transformationsmatrix vom i-ten Koerper zum Vorgaenger
        % (Bitte nutzen Sie hierfuer auch die Datei dh_trafo.m)
         rob.kl(i).D_vi = dh_trafo(rob.kl(i).alpha,rob.kl(i).a,rob.kl(i).d,rob.kl(i).q);

        % Absolute Position und Orientierung
        % ----------------------------------
        % Homogene Transformationsmatrix vom i ins 0-System
        if i == 1
            rob.kl(i).D_0i = rob.kl(i).D_vi;
        else
            rob.kl(i).D_0i = rob.kl(vor).D_0i*rob.kl(i).D_vi;
        end

        % Vektor vom Inertialsystem ins i-te KOS im B0-System (aus homogener Transformationsmatrix)
        rob.kl(i).B0_r_i = rob.kl(i).D_0i(1:3,4);

        % Rotationsmatrix vom B0 ins Bi-KOS (aus homogener Transformationsmatrix)
        rob.kl(i).A_i0 = rob.kl(i).D_0i(1:3,1:3)';
    end
    % Position des Endeffektors im B0-System (aus homogener Transformationsmatrix)
     rob.w = rob.kl(rob.N_Q).B0_r_i + rob.kl(rob.N_Q).A_i0'*rob.BN_r_N_tcp;
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end

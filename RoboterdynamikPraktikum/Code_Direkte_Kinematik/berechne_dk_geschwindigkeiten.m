function rob = berechne_dk_geschwindigkeiten(rob)
    % Berechnung der Geschwindigkeits-Groessen der direkten Kinematik

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % Ersetzen Sie die nachfolgenden Rueckgaben durch Ihre Berechnungen
    % ---------------------------------------------------------------------
    % Berechnung fuer alle Koerper
    for i = 1:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;
        
        % Verschiebungsvektor vom Vorgaenger zum Koerper i im KOS des Vorgaengers (Bv)
        rob.kl(i).Bv_r_vi = [                      rob.kl(i).a;...
                             -sin(rob.kl(i).alpha)*rob.kl(i).d;...
                              cos(rob.kl(i).alpha)*rob.kl(i).d];
                          
        % Drehmatrix vom Vorgaenger zum i-ten Koerper
        rob.kl(i).A_iv = Az(rob.q(i))*Ax(rob.kl(i).alpha); 
            
        % Drehmatrix vom B0-KOS ins Bi-KOS:
        if i == 1
            rob.kl(i).A_i0  = rob.kl(i).A_iv;
        else
            rob.kl(i).A_i0 = rob.kl(i).A_iv*rob.kl(vor).A_i0;
        end

        % Relative Winkelgeschwindigkeit berechnen
        % ----------------------------------------
         rob.kl(i).Bi_omega_rel = rob.dot_q(i)*[0;0;1];

        % Absolute Winkelgeschwindigkeit berechnen
        % ----------------------------------------
        if i == 1
            rob.kl(i).Bi_omega = rob.kl(i).Bi_omega_rel;
        else
            rob.kl(i).Bi_omega = rob.kl(i).A_iv*rob.kl(vor).Bi_omega + rob.kl(i).Bi_omega_rel;
        end

        % Absolute Translationsgeschwindigkeit berechnen
        % ----------------------------------------------
        if i == 1
          rob.kl(i).Bi_dot_r_i = zeros(3,1);
        else
          rob.kl(i).Bi_dot_r_i = rob.kl(i).A_iv*(rob.kl(vor).Bi_dot_r_i + tilde(rob.kl(i).Bv_r_vi)'*rob.kl(vor).Bi_omega);
        end
    end

    % Geschwindigkeit des TCP im B0-System berechnen
     rob.dot_w = rob.kl(rob.N_Q).A_i0'*(rob.kl(rob.N_Q).Bi_dot_r_i + tilde(rob.kl(rob.N_Q).Bi_omega)*rob.BN_r_N_tcp);
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end

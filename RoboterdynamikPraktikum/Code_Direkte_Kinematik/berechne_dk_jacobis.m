function rob = berechne_dk_jacobis(rob,Verfahren)
    % Berechnung der Jacobi-Matrizen der Rotation, Translation und Arbeitsraumkoordinaten
    % Hinweis: vor dem Aufruf dieser Funktion muss berechne_dk_positionen ausgefuehrt werden, um
    % Zugriff auf aktuellen Drehmatrizen zu haben. Dies ist bereits in Inverse_Kinematik.m
    % implementiert.

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % Ersetzen Sie die nachfolgenden Rueckgaben durch Ihre Berechnungen
    % ---------------------------------------------------------------------
    % Berechnung fuer alle Koerper
    for i=1:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;     

        % Jacobi-Matrix der Rotation des Koerpers i
        % -----------------------------------------
        rob.kl(i).Bi_Jr_rel = zeros(3,rob.N_Q);
        rob.kl(i).Bi_Jr_rel(3,i) = 1;
        if i == 1
            rob.kl(i).Bi_Jr = rob.kl(i).Bi_Jr_rel;
        else
            rob.kl(i).Bi_Jr = rob.kl(i).A_iv*rob.kl(vor).Bi_Jr + rob.kl(i).Bi_Jr_rel;

        % Jacobi-Matrix der Translation des Koerpers i
        % --------------------------------------------
        if i ==1
            rob.kl(i).Bi_Jt_o = zeros(3,rob.N_Q);
        else
            rob.kl(i).Bi_Jt_o_rel = rob.kl(i).A_iv*(tilde(rob.kl(i).Bv_r_vi)'*rob.kl(vor).Bi_Jr);
            rob.kl(i).Bi_Jt_o = rob.kl(i).A_iv*rob.kl(vor).Bi_Jt_o + rob.kl(i).Bi_Jt_o_rel;
        end    
    end

    % Jacobi-Matrizen fuer TCP
    % ------------------------
    % Jacobi-Matrix der Rotation des TCP dargestellt im B0-KOS
     B0_Jr = rob.kl(i).A_i0'*rob.kl(rob.N_Q).Bi_Jr;

    % Jacobi-Matrix der Translation des TCP dargestellt im B0-KOS
     Bi_Jt_o = rob.kl(rob.N_Q).Bi_Jt_o + tilde(rob.BN_r_N_tcp)'*rob.kl(rob.N_Q).Bi_Jr;
     B0_Jt_o = rob.kl(i).A_i0'*Bi_Jt_o;

    % Jacobi-Matrix der Arbeitsraum-Koordinaten
    % -----------------------------------------
    rob.Jw = B0_Jt_o;

    %% --- ENDE ARBEITSBEREICH --------------------------------------------

    % Platzhalter (noch nicht zu bearbeiten)
    if strcmp(Verfahren,'xxx') == true
        rob.Jw = B0_Jt_o;
    end
end

function rob = berechne_ik_rmc(rob,W,K,Option)
    % Berechnung der Inversen Kinematik ueber Resolved Motion Rate Control
    % W     ...positiv definite Wichtungsmatrix
    % K     ...Driftkompensationsmatrix
    % Option...dem Verfahren uebergebene Optionen

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % ---------------------------------------------------------------------
    % Berechnung der mit W gewichteten Pseudoinversen der Arbeitsraum-Jacobimatrix
     Jw_pseudo = inv(W)*rob.Jw'*inv(rob.Jw*inv(W)*rob.Jw');

    % Berechnung der Gelenkwinkelgeschwindigkeit ueber Resolved Motion Rate Control
    if strcmp(Option,'drift') == true
        % Aufgabe 2.1 RMC - ohne Driftkompensation
        dot_q_new = Jw_pseudo*rob.dot_w_d;

    elseif strcmp(Option,'driftcomp') == true
        % Aufgabe 2.2 RMC - mit Driftkompensation
        dot_q_new = Jw_pseudo*(rob.dot_w_d + K*(rob.w_d - rob.w));
        
    else
        % Ungueltige Option
        error('Ungueltige Option gewaehlt!')
    end
    %% --- ENDE ARBEITSBEREICH --------------------------------------------

    % Berechnung der Gelenkwinkelbeschleunigung aus Differenzenquotient
    rob.ddot_q = (dot_q_new-rob.dot_q)/rob.dt;

    % Uebernehmen der berechneten Gelenkwinkelgeschwindigkeit
    rob.dot_q = dot_q_new;

    % Gelenkwinkel ueber explizites Euler-Verfahren berechnen
    rob.q = rob.q+rob.dot_q*rob.dt;
end

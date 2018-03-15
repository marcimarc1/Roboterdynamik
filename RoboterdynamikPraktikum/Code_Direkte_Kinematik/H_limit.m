function [H,grad_H] = H_limit(q)
    % Berechnung der Guetefunktion sowie dessen Gradienten
    % in Abhaengigkeit von der aktuellen Konfiguration (Gelenkwinkel)
    % zur Vermeidung von Gelenkwinkelbeschraenkungen

    % Definition der Gelenkwinkelbeschraenkungen
    q_max = [    pi/2.0; ...
                     pi; ...
                 pi/2.0; ...
                 pi/2.0; ...
             3.0/4.0*pi; ...
                     pi];
    q_min = -q_max;

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % ---------------------------------------------------------------------
    % Straffunktion
     H = 1/(q-q_min).^2 + 1/(q-q_max).^2;

    % Gradient der Straffunktion
     grad_H = -2./(q-q_min).^3 -2./(q-q_max).^3; 
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end

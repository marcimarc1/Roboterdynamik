function [H,grad_H] = H_comfort(q)
    % Berechnung der Guetefunktion sowie dessen Gradienten
    % in Abhaengigkeit von der aktuellen Konfiguration (Gelenkwinkel)
    % zur Bevorzugung einer Komfort-Pose

    % Komfort-Pose
    q_star = [     0; ...
                pi/4; ...
                pi/4; ...
                   0; ...
                   0; ...
                pi/4];

    % Guetefunktion H
    H = 1.0/2.0*(q-q_star)'*(q-q_star);

    % Gradient der Guetefunktion
    grad_H = q-q_star;
end

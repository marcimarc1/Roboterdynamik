function rob = berechne_dk_beschleunigungen( rob )
%Berechnung der Beschleunigungsgroessen, direkte Kinematik 
% Ergebnisse werden in rob gespeichert
%
% Es muessen zuvor Positionen und Geschwindigkeiten berechnet werden
%
% Allgemeine Anmerkungen: Siehe berechne_dk_positionen
%
%Parameter: rob  (Roboter Daten)

n_Elemente = rob.N_Q;

% Berechnung fuer alle Koerper
for i=1:n_Elemente
    % Index des Vorgaengers merken
    vor=rob.kl(i).vorgaenger;
    %% absolute Winkelbeschleunigung berechnen
    phi=rob.q(rob.kl(i).fhg_no);
    dot_phi=rob.dot_q(rob.kl(i).fhg_no);
    ddot_phi=rob.ddot_q(rob.kl(i).fhg_no);
    
    
    %Im Folgenden wird die Zeitableitung der Drehmatrix ueber den
    %Tilde-Operator gebildet und nicht explizit berechnet. 
    
    if vor ~= -1 % hat Vorgaenger
        %Variante ueber tilde(omega).
        %Es wird ausgenutzt, dass
        %A_BI*d(A_IB)/dt=-d(A_BI)*A_IB=tilde(B_omega)
        % ==> d(A_BI)/dt = -tilde(B_omega)*A_BI = tilde(B_omega)^T*A_BI
        % Wobei hier gilt: A_BI ~ A_iv
        rob.kl(i).Bi_dot_omega=rob.kl(i).A_iv*rob.kl(vor).Bi_dot_omega...
            +tilde(rob.kl(i).Bi_omega_rel)'*rob.kl(i).A_iv*rob.kl(vor).Bi_omega;
    else
        rob.kl(i).Bi_dot_omega=zeros(3,1);
    end
    %relativer Anteil
    rob.kl(i).Bi_dot_omega_rel=[0;0;ddot_phi];
    rob.kl(i).Bi_dot_omega=rob.kl(i).Bi_dot_omega+rob.kl(i).Bi_dot_omega_rel;
    
    
    %% absolute Translationsbeschleunigung berechnen
    if vor ~= -1 % hat Vorgaenger
        rob.kl(i).Bi_ddot_r_i=rob.kl(i).A_iv*(rob.kl(vor).Bi_ddot_r_i+...
            tilde(rob.kl(vor).Bi_dot_omega)*rob.kl(i).Bv_r_vi+...
            tilde(rob.kl(vor).Bi_omega)*tilde(rob.kl(vor).Bi_omega)*rob.kl(i).Bv_r_vi);
    else
        rob.kl(i).Bi_ddot_r_i=zeros(3,1);
    end
    
end
end


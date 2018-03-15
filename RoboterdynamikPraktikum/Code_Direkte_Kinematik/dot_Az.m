function dotA = dot_Az(phi,dot_phi)
    % Zeitableitung der Elementardrehung um die z-Achse mit Winkel phi
    dotA = [ -sin(phi)*dot_phi,  cos(phi)*dot_phi, 0;
             -cos(phi)*dot_phi, -sin(phi)*dot_phi, 0;
                             0,                 0, 0];
end

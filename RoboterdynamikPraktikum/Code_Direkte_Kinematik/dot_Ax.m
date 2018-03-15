function dotA = dot_Ax(phi,dot_phi)
    % Zeitableitung der Elementardrehung um die x-Achse mit Winkel phi
    dotA=[0,                0,                 0;
          0,-sin(phi)*dot_phi,  cos(phi)*dot_phi;
          0,-cos(phi)*dot_phi, -sin(phi)*dot_phi];
end
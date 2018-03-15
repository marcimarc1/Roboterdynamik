function dotA = dot_Ay(phi,dot_phi)
    % Zeitableitung Elementardrehung um die y-Achse mit Winkel phi
    dotA = [ -sin(phi)*dot_phi, 0, -cos(phi)*dot_phi;
                             0, 0,                 0;
              cos(phi)*dot_phi, 0, -sin(phi)*dot_phi];
end

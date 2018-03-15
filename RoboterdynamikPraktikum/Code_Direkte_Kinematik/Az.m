function A = Az(phi)
    % Elementardrehung um die z-Achse mit Winkel phi
    A = [ cos(phi), sin(phi), 0;
         -sin(phi), cos(phi), 0;
                 0,        0, 1];
end

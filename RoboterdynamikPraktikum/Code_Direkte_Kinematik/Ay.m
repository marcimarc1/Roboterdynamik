function A = Ay(phi)
    % Elementardrehung um die y-Achse mit Winkel phi
    A = [cos(phi), 0, -sin(phi);
                0, 1,         0;
         sin(phi), 0,  cos(phi)];
end

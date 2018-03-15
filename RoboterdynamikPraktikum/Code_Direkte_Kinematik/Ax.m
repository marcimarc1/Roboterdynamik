function A = Ax(phi)
    % Elementardrehung um die x-Achse mit Winkel phi
    A = [1,         0,        0;
         0,  cos(phi), sin(phi);
         0, -sin(phi), cos(phi)];
end
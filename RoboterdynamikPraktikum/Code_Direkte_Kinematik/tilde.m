function T = tilde(v)
    % tilde-Operator: Berechnung des "Schlangentensors" aus Vektor v 
    % (schiefsymmetrische Matrix, sodass tilde(v)*w = cross(v,w)

    T = [    0, -v(3),  v(2);...
          v(3),     0, -v(1);...
         -v(2),  v(1),    0];
end


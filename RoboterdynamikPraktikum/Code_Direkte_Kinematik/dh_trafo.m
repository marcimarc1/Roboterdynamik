function D_vi = dh_trafo(alpha, a, d, theta)
    % alpha(i-1), a(i-1), d(i), theta(i): DH-Parameter
    % D_vi ist homogene Transformationsmatrix von i -> i-1

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % DH-Matrix
    D_vi = [ Ax(alpha)'*Az(theta)', Ax(alpha)'*( a*[1;0;0]+d*[0;0;1]);...
                  zeros(1,3)      ,                1                     ];
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end

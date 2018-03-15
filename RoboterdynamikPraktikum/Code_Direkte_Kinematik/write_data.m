function write_data(T,V,N_q,filename)
    % T         ...Zeitschritt-Vektor
    % V         ...Datensatz, Aufbau:
    %                         V( :, 1, Gelenk, Zeit ) = rob.kl(l).B0_r_i;
    %                         V( :, 2:4, Gelenk, Zeit ) = rob.kl(l).A_i0;
    % N_q       ...Anzahl Freiheitsgrade
    % filename  ...Dateiname 
    
    %% Oeffne die Datei um sie (neu) zu beschreiben
    file = fopen(filename,'w');

    %% Erklaere Spaltennamen
    fprintf(file,'# time\tB0_r_i(1)\tB0_r_i(2)\tB0_r_i(3)\t0_A_i(1,1)\t0_A_i(1,2)\t0_A_i(1,3)\t0_A_i(2,1)\t...\n');

    %% Fuege Daten ein
    % Iteration ueber Zeit
    for t=1:length(T)
        % Zeit
        fprintf(file,'%f\t',T(t));
        % Vektor Drehmatrix Vektor Drehmarix ...
        for q=1:N_q
            fprintf(file,'%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f', V(:,1,q,t), V(:,2:4,q,t) );
        end
        % Zeilenende
        fprintf(file,'\n');
    end

    %% Schliesse Datei
    fclose(file);

end % function

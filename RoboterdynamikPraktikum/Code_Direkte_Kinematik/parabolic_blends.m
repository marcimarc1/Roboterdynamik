function [Q, dot_Q, ddot_Q, T, Q_I, T_I] = parabolic_blends( W_stuetz, delta_T )
  % Erzeugt aus N_I Stuetzvektoren in W_stuetz eine Trajektorie mit konstanten Bescheunigungen
  % Q         := Gelenkwinkeltrajektorie auf Positionsebene
  % dot_Q     := Gelenkwinkeltrajektorie auf Geschwindigkeitsebene
  % ddot_Q    := Gelenkwinkeltrajektorie auf Beschleunigungsebene
  % T         := Zeitvektor der Trajektorie

  % W_stuetz  := Stuetzpunkte
  % delta_T   := Taktzeit


  %% Einstellungen
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Maximalgeschwindigkeiten
  dot_Q_I_max = 2.4*[ 1, 1, 1, 1, 1, 1 ]; % [rad/s]
  ddot_Q_I_max = 32*[ 1, 1, 1, 1, 1, 1 ]; % [rad/s^2]


  %% indirekte Kinematik Stuetzpunkte aus W nach Q
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Q_I = berechne_gelenkwinkel( W_stuetz );


  % Anzahl der Stuetzpunkte
  N_I = length(Q_I(1,:));
  % Anzahl der Freiheitsgrade
  N_Q = length(Q_I(:,1));



  %% Berechnung der Schaltzeiten je Gelenk
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Initialisierung
  dot_Q_I  = zeros(N_Q,N_I-1);
  ddot_Q_I = zeros(N_Q,N_I);
  t_b = zeros(N_Q,N_I);
  t_g = zeros(N_Q,N_I-1);

  for g = 1:N_Q

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen.
    % Achten Sie auf die Laenge der benoetigten Schleifen.
    % Bedenken Sie auch, dass Nenner = 0 sein koennen.
    % ---------------------------------------------------------------------
for i = 1: N_I
    %try    % Berechnung der Geschwindigkeiten
    if i == 1
     dot_Q_I(g,i) = sign(Q_I(g,i+1)-Q_I(g,i))*dot_Q_I_max(g);
    % Berechnung der Bescheunigungen
     ddot_Q_I(g,i) = sign(dot_Q_I(g,i))*ddot_Q_I_max(g); 
    % Berechnung der Beschleunigungszeiten 
    if ddot_Q_I(g,i) == 0
       t_b(g,i) = 0;
    else
       t_b(g,i) = dot_Q_I(g,i)/ddot_Q_I(g,i);
    end
    % Berechnung der benoetigten Zeiten
    if dot_Q_I(g,i) == 0
        t_g(g,i) = 0.5 *t_b(g,i);
    else
       t_g(g,i) = (Q_I(g,i+1)-Q_I(g,i))/dot_Q_I(g,i) + 0.5 *t_b(g,i); 
    end
    elseif i == N_I 
    % Berechnung der Bescheunigungen
     ddot_Q_I(g,i) = sign(-dot_Q_I(g,i-1))*ddot_Q_I_max(g);
    % Berechnung der Beschleunigungszeiten 
    if ddot_Q_I(g,i) == 0
       t_b(g,i) = 0;
    else
       t_b(g,i) = -dot_Q_I(g,i-1)/ddot_Q_I(g,i);
    end
    % Berechnung der benoetigten Zeiten
    if dot_Q_I(g,i-1) == 0
        t_g(g,i-1) = 0.5 *t_b(g,i);
    else
        t_g(g,i-1) = (Q_I(g,i)-Q_I(g,i-1))/dot_Q_I(g,i-1) + 0.5 * t_b(g,i);
    end
    
    else
     dot_Q_I(g,i) = sign(Q_I(g,i+1)-Q_I(g,i))*dot_Q_I_max(g);


    % Berechnung der Bescheunigungen
     ddot_Q_I(g,i) = sign(dot_Q_I(g,i)-dot_Q_I(g,i-1))*ddot_Q_I_max(g);


    % Berechnung der Beschleunigungszeiten 
    if ddot_Q_I(g,i) == 0
        t_b(g,i) =0;
    else
        t_b(g,i) = (dot_Q_I(g,i)-dot_Q_I(g,i-1))/ddot_Q_I(g,i);
    end


    % Berechnung der benoetigten Zeiten
    if dot_Q_I(g,i) == 0
        t_g(g,i) = 0;
    else
        t_g(g,i) = (Q_I(g,i+1)-Q_I(g,i))/dot_Q_I(g,i);
    end
    
    end
    
end
    %% --- ENDE ARBEITSBEREICH --------------------------------------------

  end



  %% Synchronisation
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % laengste noetige Zeit je Stuetzpunkt
  t = max(t_g,[],1) ;
  if min(t) <= 1e-3
    error('Zwei aufeinanderfolgende Stuetzpunkte sind zu nahe beieinander.')
  end




  %% Berechnung der Switching-Times
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Nullen der Arrays
  dot_Q_I       = zeros(N_Q,N_I-1);
  t_b           = zeros(N_Q,N_I);

  % Initialisierung neuer Arrays
  t_accelerate  = zeros(N_Q,N_I);
  t_const       = zeros(N_Q,N_I);
  t_total       = zeros(1,N_I-1);

  % Berechnung der absoluten Zeiten aus den relativen Zeiten
  t_total(1) = t(1);
  for i = 2:N_I-1
    t_total(i) = t_total(i-1)+t(i);
  end
  T_I = [0,t_total];

  for g = 1:N_Q

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen.
    % Bedenken Sie wieder die Laenge der benoetigten Schleifen und die
    % Division durch 0.
    % ---------------------------------------------------------------------

    % erste und letzte Beschleunigung bleiben gleich
    
    %fuer i = 1
    % Neuberechnung der Geschwindigkeiten
    dot_Q_I(g,1) = ddot_Q_I(g,1)*t(1)-sign(ddot_Q_I(g,1))* sqrt(ddot_Q_I(g,1)^2 * t(1)^2-2*ddot_Q_I(g,1)*(Q_I(g,2)-Q_I(g,1)));
    % Neuberechnung der Beschleunigungszeiten
    if ddot_Q_I(g,1) ==0
        t_b(g,1) = 0;
    else
        t_b(g,1) = (dot_Q_I(g,1)-dot_Q_I(g,i-1))/ddot_Q_I(g,1);
    end
     
    for i= 2:N_I-1
    % Neuberechnung der Geschwindigkeiten
     dot_Q_I(g,i) = ( Q_I(g,i+1)-Q_I(g,i) )/t(i);


    % Neuberechnung der n-ten Bescheunigung
    % mittlere koennen sich aufgrund der Synchronisation aendern
      ddot_Q_I(g,i) = sign( dot_Q_I(g,i) - dot_Q_I(g,i-1) ) * ddot_Q_I_max(g);

    % Neuberechnung der Beschleunigungszeiten
    if ddot_Q_I(g,i) == 0
        t_b(g,i) = 0;
    else
        t_b(g,i) = (dot_Q_I(g,i)-dot_Q_I(g,i-1))./ddot_Q_I(g,i);
    end
    
    end
   
     %fuer den letzten
     % Neuberechnung der Geschwindigkeiten
    dot_Q_I(g,N_I-1) = -ddot_Q_I(g,N_I)*t(N_I-1)+sign(ddot_Q_I(g,N_I))* sqrt(ddot_Q_I(g,N_I).^2*t(N_I-1)^2+2*ddot_Q_I(g,N_I)*(Q_I(g,N_I)-Q_I(g,N_I-1))) ;
    % Neuberechnung der Beschleunigungszeiten
    if ddot_Q_I(g,N_I-1) == 0
       t_b(g,N_I) = 0;
    else
       t_b(g,N_I) = -dot_Q_I(g,N_I-1)/ddot_Q_I(g,N_I);
    end
    
    
     %% --- ENDE ARBEITSBEREICH --------------------------------------------


    % Berechnung der Schaltzeiten
    t_accelerate(g,1)   = 0;
    t_const(g,1)        = t_b(g,1);
    for i=2:N_I-1
      t_accelerate(g,i) = t_total(i-1)-0.5*t_b(g,i);
      t_const(g,i)      = t_total(i-1)+0.5*t_b(g,i);
    end
    t_accelerate(g,N_I) = t_total(N_I-1)-t_b(g,N_I);
    t_const(g,N_I)      = t_total(N_I-1);

  end


  %% Abbruch, falls gegebene Stuetzpunkte nicht erreicht werden koennen
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for g = 1:N_Q
    for j = 1:N_I-1
      if t_const(g,j) > t_accelerate(g,j+1)
        error('Diese Trajektorie kann mit den gegebenen Bedingungen nicht abgefahren werden! Bitte aendern Sie Punkte oder Maximalgeschwindigkeit/-beschleunigung.');
      end
    end
  end



  %% berechne Steuertrajektorien
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  t_end   = max(t_const(:,N_I));
  T       = 0:delta_T:t_end;


  t_ddot_q_I = zeros(N_Q,N_I*4 -2);
  ddot_q_I   = zeros(N_Q,N_I*4 -2);
  t_dot_q_I  = zeros(N_Q,N_I*2);
  dot_q_I    = zeros(N_Q,N_I*2);

  Q       = zeros(N_Q,length(T));
  dot_Q   = zeros(size(Q));
  ddot_Q  = zeros(size(Q));


  t_ddot_q_I(:,1:4:end)  = 0.001+t_accelerate;
  t_ddot_q_I(:,2:4:end)  = 0.002+t_const;
  t_ddot_q_I(:,3:4:end)  = 0.003+t_const(:,1:end-1);
  t_ddot_q_I(:,4:4:end)  = t_accelerate(:,2:end);

  ddot_q_I(:,1:4:end)    = ddot_Q_I;
  ddot_q_I(:,2:4:end)    = ddot_Q_I;
  ddot_q_I(:,3:4:end)    = zeros(N_Q,N_I-1);
  ddot_q_I(:,4:4:end)    = zeros(N_Q,N_I-1);

  t_dot_q_I(:,1:2:end)   = t_accelerate;
  t_dot_q_I(:,2:2:end)   = 0.001+t_const;

  dot_q_I(:,1)           = zeros(N_Q,1);
  dot_q_I(:,2:2:end-1)   = dot_Q_I;
  dot_q_I(:,3:2:end-1)   = dot_Q_I;
  dot_q_I(:,end)         = zeros(N_Q,1);

  for g=1:N_Q
    ddot_Q(g,:) = interp1(t_ddot_q_I(g,:),ddot_q_I(g,:),T,'previous');
    dot_Q(g,:)  = interp1(t_dot_q_I(g,:),dot_q_I(g,:),T);
    Q(g,:)      = trapz_int(t_dot_q_I(g,:),dot_q_I(g,:),T,Q_I(g,1));
                  % interpoliert und integriert
  end
  ddot_Q(:,1) = zeros(N_Q,1);
end % Funktion parabolic_blends
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Hilfsfunktionen

% interpolieren und integrieren von dot_Q nach Q
function Q = trapz_int( t_dot_Q_I, dot_Q_I, T, Q_0 )

  % initialisieren
  Q = zeros(1,length(T));

  % Initialwerte
  Q(:,1) = Q_0;
  dot_Q  = dot_Q_I(1);

  % interpolieren und integrieren ueber die Zeit
  for t=2:length(T)
    dot_Q_last  = dot_Q;
    dot_Q       = interp1(t_dot_Q_I,dot_Q_I,T(t));
    Q(t)        = Q(t-1) + (dot_Q_last+dot_Q)/2*(T(t)-T(t-1));
  end

end

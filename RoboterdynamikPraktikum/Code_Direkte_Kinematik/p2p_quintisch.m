function [ S, dot_S, ddot_S, T ] = p2p_quintisch( W_stuetz, T_ges, delta_T)
  % Erzeugt aus N_I Stuetzpunkten in W_stuetz je mit Anfangs- und Endpunkt N_I-1 Trajektorien je in Form eines quintischen Polynoms
  % S         := Trajektorie auf Positionsebene
  % dot_S     := Trajektorie auf Geschwindigkeitsebene
  % ddot_S    := Trajektorie auf Beschleunigungsebene
  % T         := Zeitvektor der Trajektorie

  % W_stuetz  := Stuetzpunkte
  % T_I(end)     := Dauer der Bewegung/Interpolation
  % delta_T   := Taktzeit

  % Anzahl der Freiheitsgrade
  N_Q       = size( W_stuetz,1 );

  % Anzahl der Stuetzpunkte
  N_I       = size( W_stuetz,2 );

  % Zeitintervall fuer Interpolation
  T_I       = 0:delta_T:(T_ges/(N_I-1));  % Zeitintervall fuer ein Teilstueck
  N_T_I     = length(T_I);          % Anzahl der Zeitpunkte eines Teilstuecks

  %% Berechnung der Trajektorie

  % Initialisierung fuer Teilstuecke
  S_I       = zeros( N_Q, N_T_I );
  dot_S_I   = zeros(size(S_I));
  ddot_S_I  = zeros(size(S_I));

  % Initialisierung fuer Gesamttrajektorie
  S         = [];
  dot_S     = [];
  ddot_S    = [];
  T         = [];
  %% --- ARBEITSBEREICH: ------------------------------------------------
% Erzeuge Trajektorie 

% Schleife ueber Stuetzpunktepaare
  for i = 1:N_I-1
      pos1 = W_stuetz(:,i);
      pos2 = W_stuetz(:,i+1);
      den = [ T_I(1)^5*eye(N_Q),         T_I(1)^4*eye(N_Q),         T_I(1)^3*eye(N_Q),         T_I(1)^2*eye(N_Q),       T_I(1)*eye(N_Q),     1*eye(N_Q);   % Position des Anfangspunktes
              T_I(N_T_I)^5*eye(N_Q),     T_I(N_T_I)^4*eye(N_Q),     T_I(N_T_I)^3*eye(N_Q),     T_I(N_T_I)^2*eye(N_Q),   T_I(N_T_I)*eye(N_Q), 1*eye(N_Q); % Position des Endpunktes
              5*T_I(1)^4*eye(N_Q),       4*T_I(1)^3*eye(N_Q),       3*T_I(1)^2*eye(N_Q),       2*T_I(1)*eye(N_Q),       1*eye(N_Q),          0*eye(N_Q); % Geschwindgkeit des Anfangspunktes
              5*T_I(N_T_I)^4*eye(N_Q),   4*T_I(N_T_I)^3*eye(N_Q),   3*T_I(N_T_I)^2*eye(N_Q),   2*T_I(N_T_I)*eye(N_Q),   1*eye(N_Q),          0*eye(N_Q); % Geschwindgkeit des Endpunktes
              20*T_I(1)^3*eye(N_Q),      12*T_I(1)^2*eye(N_Q),      6*T_I(1)*eye(N_Q),         2*eye(N_Q),              0*eye(N_Q),          0*eye(N_Q); % Beschleunigung des Anfangspunktes
              20*T_I(N_T_I)^3*eye(N_Q),  12*T_I(N_T_I)^2*eye(N_Q),  6*T_I(N_T_I)*eye(N_Q),     2*eye(N_Q),              0*eye(N_Q),          0*eye(N_Q)];% Beschleunigung des Endpunktes
      num = [pos1; pos2;zeros(N_Q,1);zeros(N_Q,1);zeros(N_Q,1);zeros(N_Q,1)];  % Geschwindigkeit sind  0 fur Anfangs- und Endpunkte
      para = den\num;                                                         % Para = [ax,ay,az,bx,by,bz,cx,cy,cz,dx,dy,dz], Koeffizienten des Polynoms
      for j = 1: N_T_I
          pos_mat = [T_I(j)^5,T_I(j)^4,T_I(j)^3, T_I(j)^2, T_I(j), 1];
          vel_mat = [5*T_I(j)^4,4*T_I(j)^3,3*T_I(j)^2, 2*T_I(j), 1, 0];
          acc_mat = [20*T_I(j)^3,12*T_I(j)^2,6*T_I(j), 2, 0, 0];
          para_x = [para(1);para(4);para(7);para(10);para(13);para(16)];
          para_y = [para(2);para(5);para(8);para(11);para(14);para(17)];
          para_z = [para(3);para(6);para(9);para(12);para(15);para(18)];
          S_I(:,j) = [pos_mat*para_x;pos_mat*para_y;pos_mat*para_z];
          dot_S_I(:,j) = [vel_mat*para_x;vel_mat*para_y;vel_mat*para_z];
          ddot_S_I(:,j) = [acc_mat*para_x;acc_mat*para_y;acc_mat*para_z];
      end
      T = [T,i*T_I];
      S = [S,S_I];
      dot_S = [dot_S,dot_S_I];
      ddot_S = [ddot_S,ddot_S_I];  
  end
  %% --- ENDE ARBEITSBEREICH --------------------------------------------
end % function

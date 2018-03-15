function [ S, dot_S, ddot_S, T ] = kubischer_spline( W_stuetz, T_ges, delta_T )
% Erzeugt aus N_I Stuetzvektoren in W_stuetz eine Trajektorie in Form kubischer Splines
% S         := Trajektorie auf Positionsebene
% dot_S     := Trajektorie auf Geschwindigkeitsebene
% ddot_S    := Trajektorie auf Beschleunigungsebene
% T         := Zeitvektor der Trajektorie

% W_stuetz  := Stuetzpunkte
% T_ges     := Dauer der Bewegung/Interpolation
% delta_T   := Taktzeit

% Erzeugt aus n Stuetzvektoren p_i und einer Zeit T eine Trajektorie aus
%stueckweise zusammengesetzten Polynomen dritten Grades (Spline)


%% --- ARBEITSBEREICH: ------------------------------------------------
%% Dimensionen pruefen/festlegen
    N_Q = size( W_stuetz,1 );
    N_I = size( W_stuetz,2 );
    N_T_I = ceil((T_ges/(N_I-1))/delta_T); % Anzahl von Zeitschritt zwieschen 
    N_T = N_T_I*(N_I-1);

% T_ges neu setzen, damit T_ges exakt bei N_T erreicht ist
     T_ges = delta_T*(N_T-1);
%% --- ENDE ARBEITSBEREICH --------------------------------------------

% Aequidistanter h-Vektor
h       = T_ges / ( N_I - 1 ) * ones( 1, N_I - 1 );

%% --- ARBEITSBEREICH: ------------------------------------------------
     S = [];
     dot_S = [];
     ddot_S = [];
     T = 0:delta_T:T_ges;

    for i = 1:N_Q
         [s,dot_s,ddot_s] = kubischer_spline_skalar(W_stuetz(i,:), h ,N_I, N_T, N_Q, T, N_T_I);
         S = [S;s];
         dot_S = [dot_S;dot_s];
         ddot_S = [ddot_S;ddot_s];
    end
%% --- ENDE ARBEITSBEREICH --------------------------------------------

%% Erzeuge Spline fuer jede Komponente

end


%% Hilfsfunktion

%% --- ARBEITSBEREICH: ------------------------------------------------
function [ s, dot_s, ddot_s] = kubischer_spline_skalar( p,h, N_I,N_T,N_Q,T,N_T_I)
%% Variablen fuer Gleichungssystem A* ddot_p = r anlegen
    A = zeros( N_I);
    r = zeros( N_I,1);

% Eigentliche Spline Koeffizienten
    a = zeros( 1,N_I-1 );
    b = zeros( 1,N_I-1 );
    c = zeros( 1,N_I-1 );
    d = zeros( 1,N_I-1 );

%% Erstelle A-Matrix und r-Vektor
for i = 2:N_I-1
   A(i,i) = 2*(h(i)+h(i-1)); % Diagonale Elemente von A-Matrix
   A(i,i-1) = h(i-1); % Schiefdiagonale Elemente von A-Matrix
   A(i-1,i) = h(i-1);
   r(i) = -6*(p(i)-p(i-1))/h(i-1) + 6*(p(i+1)-p(i))/h(i); 
end
A(1,1) = 2*h(1);
A(N_I,N_I) = 2*h(N_I-1);
A(N_I-1,N_I) = h(N_I-1);
A(N_I,N_I-1) = h(N_I-1);
r(1) = 6*(p(2)-p(1))/h(1);
r(N_I) = -6*(p(N_I)-p(N_I-1))/h(N_I-1);
%% Gleichungssystem loesen
ddot_p = A\r;

%% Koeffizientenberechnung
    for i = 1:N_I-1
        a(i) = (ddot_p(i+1)-ddot_p(i))/(6*h(i));
        b(i) = ddot_p(i)/2;
        c(i) = (p(i+1)-p(i))/h(i) - h(i)*(ddot_p(i+1)+2*ddot_p(i))/6;
        d(i) = p(i);
    end

%% Spline generieren
    s = zeros(1,N_T);
    dot_s = zeros(1,N_T);
    ddot_s = zeros(1,N_T);
    for j = 1:N_T
        i = ceil(j/N_T_I);
        t = T(j) - T((i-1)*N_T_I+1);
        s(j) = a(i)*t^3 + b(i)*t^2 + c(i)*t + d(i);
        dot_s(j) = 3*a(i)*t^2 + 2*b(i)*t + c(i);
        ddot_s(j) = 6*a(i)*t + 2*b(i);
    end

% Schleife ueber alle Intervalle

%% --- ENDE ARBEITSBEREICH --------------------------------------------

end




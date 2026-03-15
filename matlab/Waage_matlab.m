% Waage berechnungen
clc;
clear;
close all;
clear vars;

%% parameters
g = 9.81;              % Erdbeschleunigung
hohe_m = 0.020;         % Höhe der Magnet in m
durchmesser_m = 0.035;  % Durchmesser der Magnet in m
pi = 3.14;
mw = 0.0665;            % Masse der Waage Plateau 
m1 = 0.00;             % Gewicht Beispiele 
m = m1 + mw;            % Gesamte Masse ist Gewicht zu messen + Plateau Gewicht 

% Spule 
Id_s = 0.0127;          % Innendurchmesser der Spule in m
Ad_s = 0.033;           % Außendurchmesser der Spule in m
hohe_s = 0.008;         % Höhe der Spulen in m
breite_s = 0.016;       % Breite der Spulen in m
D_0 = 0.0003;           % D_Spulen in m
D_1 = 0.0004;           % D_Spulen lackiert, in m
Vspule = pi * (Ad_s/2)^2 * hohe_s - pi * (Id_s/2)^2 * hohe_s; % Volumen der Spule 
n_Lagen = 62;           % Anzahl der Lagen 
B = 0.6;                % mag. Fluss in Tesla
I_max = 2;              % max Strom in Ampere
U_max = 12;             % Max Spannung
Kd = 0.1;             % Dämpfungskonstante 
k = 11.64;              % Motorkonstante
Kf=0.2;                %Federkonstante
Windung_n = (hohe_s / D_1) * (breite_s / D_1); % Total Windungszahl 
R_L = 10.75;            % Widerstand der Spule 

% Konstanten Übertragungsfunktion

T1 = Kd / Kf;            % Zeitkonstante T1 
T2= sqrt(m/Kf);          % Zeitkonstante T2
T=0.003;                   % Zeitkonstante T
%% PID
% Transfer function plant model 
s = tf('s');
G = ((U_max*k)/(Kf*R_L))/((m/Kf)*s^2+(Kd/Kf)*s+1); %Übertragungsfunktion PT2 
Gr=1/(T*s*G); %GR-Übertragungsfunktion
C = pidtune(G, 'PID', 100); 

% Polstellen und Nullstellen berechnen
p = pole(G);  % Polstellen
disp(p);
z = zero(G);  % Nullstellen

% Polstellen und Nullstellen separat anzeigen
figure;
MARKERSIZE = 10;
FONTSIZE = 12;
LINEWIDTH = 2;

% Nullstellen (blau, Kreis)
subplot(2, 1, 1);  % Nullstellen plotten im oberen Teil
plot(real(z), imag(z), 'bo', 'MarkerSize', MARKERSIZE);
hold on;
grid on;
xlabel('Re', 'FontSize', FONTSIZE);
ylabel('Im', 'FontSize', FONTSIZE);
title('Nullstellen (o)', 'FontSize', FONTSIZE);
set(gca, 'XAxisLocation', 'origin');
set(gca, 'YAxisLocation', 'origin');
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
box off;

% Polstellen (rot, Kreuz)
subplot(2, 1, 2);  % Polstellen plotten im unteren Teil
plot(real(p), imag(p), 'rx', 'MarkerSize', MARKERSIZE);
hold on;
grid on;
xlabel('Re', 'FontSize', FONTSIZE);
ylabel('Im', 'FontSize', FONTSIZE);
title('Polstellen (x)', 'FontSize', FONTSIZE);
set(gca, 'XAxisLocation', 'origin');
set(gca, 'YAxisLocation', 'origin');
xlim([-1.0 0.5]);
ylim([-2.5 2.5]);
box off;

% Impulsantwort anzeigen
figure;
impulse(G);  % Impulsantwort
title('Impulsantwort des Systems', 'FontSize', FONTSIZE);
xlabel('Zeit (s)', 'FontSize', FONTSIZE);
ylabel('Antwort', 'FontSize', FONTSIZE);
grid on;

% Sprungantwort anzeigen
t=0:0.01:0.1;
[~,t_out]=step(G,t);
figure;
step(G,100);  % Sprungantwort
title('Sprungantwort des Systems', 'FontSize', FONTSIZE);
xlabel('Zeit (s)', 'FontSize', FONTSIZE);
ylabel('Antwort', 'FontSize', FONTSIZE);
grid on;
%xlim([0 0.1]);  % Zeitbereich anpassen für schnelle Betrachtung
%ylim([0 0.003]); % Höhenbereich auf 1-2 mm fokussieren
% Skalierung der Achsen
xlim([0 10]);  % Zeitbereich anpassen (von 0 bis 10 Sekunden, z. B.)

open("Waage1_simulink.slx");

Data = sim('Waage1_simulink.slx');
y = Data.logsout.get('y');
u = Data.logsout.get('u');
m = Data.logsout.get('e');

figure('Name','Gs - C','NumberTitle','off');
subplot (2,3,1);
plot(y.Values.Time, y.Values.Data, 'r-', 'Linewidth', 1);
grid on;
ylabel('Höhe [m]');
xlabel('Zeit [s]');
ylim([-0.001 0.003])
xlim([-1 4])
title('y(t)')


subplot (2,3,2);
plot(u.Values.Time, u.Values.Data, 'b-', 'Linewidth', 1);
grid on;
ylabel('Pulsweite [-]');
xlabel('Zeit [s]');
ylim([-0.1 0.3])
xlim([-1 4])
title('u(t)')

subplot (2,3,3);
plot(m.Values.Time, m.Values.Data, 'g-', 'Linewidth', 2);
grid on;
ylabel('Höhe[m]');
xlabel('Zeit [s]');
ylim([-0.001 0.003])
xlim([-1 4])
title('e(t)')


subplot (2,3,4);
step(G, t)
Kp = C.Kp;
Kd = C.Kd;
Ki = C.Ki;
xlabel('Zeit [s]');
ylabel('Strecke [m]');
xlim([0 t(end)])
ylim auto
title('Übertragungsfunktion')
grid off;

subplot (2,3,5);
P = pole(G);
plot(real(P),imag(P),"bx");
xlabel('Real');
ylabel('Imaginär');
xlim([-0.05 0.01])
ylim([-1 1])
title('Polstellen')
grid off;

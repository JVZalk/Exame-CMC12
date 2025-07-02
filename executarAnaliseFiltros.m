clear; clc; close all;

% 1: Sem Filtro
% 2: Filtro Passa-Baixas
% 3: Filtro de Kalman
MODO_FILTRO = 3;

% variância do ruído do sensor
VARIANCIA_RUIDO = 0.01; 

% requisitos de desempenho para o controlador PD
requisitos.tr = 2.0;  % Tempo de subida 
requisitos.Mp = 0.05; % Sobressinal

% referência de altitude
referencia.amplitude = 2.0; % Altitude desejada em metros
referencia.instante = 1.0;  % Drone começa a subir em t=1s

model_name = 'drone'; 

% parâmetros da planta
planta = obterPlantaDrone();

% controlador PD
controlador = projetarControladorDrone(requisitos, planta);

% parâmetros do sistema digital e do filtro
T = 0.01;

% variáveis necessárias para o Simulink
Kp = controlador.Kp;
Kd = controlador.Kd;
modo = MODO_FILTRO;
varz = VARIANCIA_RUIDO;
z_referencia = referencia.amplitude;
tempo_degrau = referencia.instante;
Forca_Gravidade = planta.m * planta.g;



%Execução
out = sim(model_name, 'StopTime', '15');


% Gráficos
nomes_modos = {'Sem Filtro', 'Filtro Passa-Baixas', 'Filtro de Kalman'};
titulo_base = ['Análise de Controle de Altitude: ', nomes_modos{modo}];

% Desempenho do Rastreamento de Altitude
figure('Name', [titulo_base, ' - Rastreamento'], 'NumberTitle', 'off');
hold on; grid on;
plot(out.z.time, out.z.signals.values, 'b-', 'LineWidth', 2);
plot(out.zf.time, out.zf.signals.values, 'g--', 'LineWidth', 2);
yline(z_referencia, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Referência');
title('Desempenho do Rastreamento de Altitude');
xlabel('Tempo (s)');
ylabel('Altitude (m)');
legend('Altitude Real (z)', 'Altitude Usada para Controle (zf)');
hold off;

% Esforço de Controle
figure('Name', [titulo_base, ' - Controle'], 'NumberTitle', 'off');
hold on; grid on;
plot(out.u.time, out.u.signals.values, 'r-');
title('Esforço de Controle (Força de Empuxo)');
xlabel('Tempo (s)');
ylabel('Força (N)');
legend('Comando de Controle u(t)');
if modo == 1
    text(0.5, 0.8, 'Controle MUITO ruidoso!', 'Units', 'normalized', 'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold');
end
hold off;

% Análise da Qualidade da Filtragem
figure('Name', [titulo_base, ' - Filtragem'], 'NumberTitle', 'off');
hold on; grid on;
plot(out.z.time, out.z.signals.values, 'b-', 'LineWidth', 2);
plot(out.zm.time, out.zm.signals.values, 'r.', 'MarkerSize', 2);
plot(out.zf.time, out.zf.signals.values, 'g-', 'LineWidth', 2);
title('Análise da Qualidade da Filtragem');
xlabel('Tempo (s)');
ylabel('Altitude (m)');
legend('Altitude Real (z)', 'Medida Ruidosa (zm)', 'Altitude Filtrada (zf)');
hold off;
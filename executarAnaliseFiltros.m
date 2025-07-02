clear; clc; close all;

% 1: Sem Filtro
% 2: Filtro Passa-Baixas
% 3: Filtro de Kalman
MODO_FILTRO = 1;

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
T_controlador = 0.01; % Período de amostragem do controlador 
T_Filtro = 0.01;      % Período de amostragem do filtro

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
titulo_principal = ['Análise de Controle de Altitude: ', nomes_modos{modo}];

figure('Name', titulo_principal, 'NumberTitle', 'off', 'Position', [100, 100, 800, 800]);

% Subplot 1: Desempenho do Rastreamento de Altitude
subplot(3, 1, 1);
hold on; grid on;
plot(out.z.time, out.z.signals.values, 'b-', 'LineWidth', 2);
plot(out.zf.time, out.zf.signals.values, 'g--', 'LineWidth', 2);
yline(z_referencia, 'k:', 'LineWidth', 1.5);
title('Desempenho do Rastreamento de Altitude');
xlabel('Tempo (s)');
ylabel('Altitude (m)');
legend('Altitude Real (z)', 'Altitude Filtrada (zf)', 'Referência');
hold off;

% Subplot 2: Esforço de Controle
subplot(3, 1, 2);
hold on; grid on;
plot(out.u.time, out.u.signals.values, 'r-');
title('Esforço de Controle (Força de Empuxo)');
xlabel('Tempo (s)');
ylabel('Força (N)');
legend('Comando de Controle u(t)');
% Destaque visualmente como o filtro suaviza o controle
if modo == 1
    ylim_current = ylim;
    text(0.5, 0.5, 'Controle muito ruidoso!', 'Units', 'normalized', 'Color', 'red', 'FontSize', 12);
end
hold off;

% Subplot 3: Análise da Qualidade da Filtragem 
subplot(3, 1, 3);
hold on; grid on;
plot(out.z.time, out.z.signals.values, 'b-', 'LineWidth', 2);
plot(out.zm.time, out.zm.signals.values, 'r.', 'MarkerSize', 2);
plot(out.zf.time, out.zf.signals.values, 'g-', 'LineWidth', 2);
title('Análise da Filtragem');
xlabel('Tempo (s)');
ylabel('Altitude (m)');
legend('Altitude Real (z)', 'Medida Ruidosa (zm)', 'Altitude Filtrada (zf)');
hold off;

sgtitle(titulo_principal, 'FontSize', 14, 'FontWeight', 'bold');
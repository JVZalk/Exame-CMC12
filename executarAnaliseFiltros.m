clear; clc; close all;

% Inclua no vetor os modos a serem testados
% 1: Sem Filtro
% 2: Filtro Passa-Baixas
% 3: Filtro de Kalman
MODO_FILTRO = [1, 2, 3];

% variância do ruído do sensor
VARIANCIA_RUIDO = 0.01; 

% requisitos de desempenho para o controlador PD
requisitos.tr = 2.0;  % Tempo de subida 
requisitos.Mp = 0.05; % Sobressinal

% referência de altitude
referencia.amplitude = 1.0; % Altitude desejada em metros
referencia.instante = 0;  % Drone começa a subir em t=1s

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
modo = MODO_FILTRO(1);
varz = VARIANCIA_RUIDO;
z_referencia = referencia.amplitude;
tempo_degrau = referencia.instante;
Forca_Gravidade = planta.m * planta.g;


% Gráficos
formato = 'eps'; % selecione 'png' ou 'eps'
nomes_modos = {'Sem Filtro', 'Filtro Passa-Baixas', 'Filtro de Kalman'};

z = cell(3, 1);
zm = cell(3, 1);
u = cell(3, 1);
zf = cell(3, 1);

varzs = [0.1, 0.01, 0.001, 0];

for j=1:length(MODO_FILTRO)
    modo = MODO_FILTRO(j);
    titulo_base = ['Análise de Controle de Altitude: ', nomes_modos{modo}];

    for i=1:length(varzs)
        varz = varzs(i);
    
        % Rodando a simulacao
        out = sim(model_name, 'StopTime', '10');
        
        z{i} = out.z;
        u{i} = out.u;
        zm{i} = out.zm;
        zf{i} = out.zf;
    end
    
    colors = get(0, 'DefaultAxesColorOrder');
    
    legs = {};
    for i=1:length(varzs)
        legs{i} = sprintf('\\sigma_h^2 = %g ft^2', varzs(i));
    end
    
    figure('Name', [titulo_base, ' - Desempenho'], 'NumberTitle', 'off');
    hold on;
    grid on;
    for i=1:length(varzs)
        plot(z{i}.time, z{i}.signals.values, 'Color', colors(i, :));
    end
    title('Desempenho do Controlador de Altitude com Vários Ruídos');
    xlabel('Tempo (s)', 'FontSize', 14);
    ylabel('Altura (m)', 'FontSize', 14);
    legend(legs, 'FontSize', 12, 'Location', 'Southeast');
    set(gca, 'FontSize', 14);
    salvarGrafico(sprintf('altitude_modo_%d', modo), formato);
    
    figure('Name', [titulo_base, ' - Filtragem'], 'NumberTitle', 'off');
    hold on;
    grid on;
    plot(zm{1}.time, zm{1}.signals.values, 'r.', 'MarkerSize', 2);
    plot(zf{1}.time, zf{1}.signals.values, 'g-', 'LineWidth', 2);
    plot(z{1}.time, z{1}.signals.values, 'b-', 'LineWidth', 2);
    title('Análise da Qualidade da Filtragem');
    xlabel('Tempo (s)');
    ylabel('Altitude (m)');
    legend('Medida Ruidosa (zm)', 'Altitude Filtrada (zf)', 'Altitude Real (z)');
    set(gca, 'FontSize', 14);
    salvarGrafico(sprintf('filtragem_modo_%d', modo), formato);
    
    figure('Name', [titulo_base, ' - Comando'], 'NumberTitle', 'off');
    hold on;
    grid on;
    for i=1:length(varzs)
        plot(u{i}.time, u{i}.signals.values, 'Color', colors(i, :));
    end
    title('Comando do Controlador de Altitude com Vários Ruídos');
    xlabel('Tempo (s)', 'FontSize', 14);
    ylabel('Força Comandada (N)', 'FontSize', 14);
    legend(legs, 'FontSize', 12, 'Location', 'Southeast');
    set(gca, 'FontSize', 14);
    salvarGrafico(sprintf('comando_modo_%d', modo), formato);
end

function salvarGrafico(nome, formato)
% Salva o grafico corrente.
% nome: nome do arquivo.
% formato: formato do arquivo ('png' ou 'eps').

if strcmp(formato, 'png')
    print('-dpng', '-r400', nome);
else
    print('-depsc2', nome);
end

end
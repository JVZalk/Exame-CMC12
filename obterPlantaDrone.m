function planta = obterPlantaDrone()
% define e retorna os parâmetros físicos do drone.
% SAÍDA: struct com os campos m, b, g.

planta.m = 1.0;     % Massa do drone em kg
planta.b = 0.25;    % Coeficiente de arrasto viscoso em Ns/m
planta.g = 9.81;    % Aceleração da gravidade em m/s^2

end
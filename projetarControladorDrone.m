function controlador = projetarControladorDrone(requisitos, planta)
% projeta os ganhos do controlador PD.
% ENTRADAS: requisitos-> struct com os campos tr (tempo de subida) e Mp (sobressinal).
%              planta -> struct com os parâmetros físicos do drone.
% SAÍDA: struct com os campos Kp e Kd.

m = planta.m;
b = planta.b;

xi = -log(requisitos.Mp) / sqrt(pi^2 + log(requisitos.Mp)^2);
wn = pi / (requisitos.tr * sqrt(1 - xi^2));

controlador.Kp = m * wn^2;
controlador.Kd = 2 * xi * wn * m - b;

end
%% ELE410 - CODIGO PREDICAO PARA MERCADO IMOBILIARIO/DEMANDA DE ENERGIA:
%  FREDERICO CASARA ANTONIAZZI - 26/03/2025
%  UFRGS DOUTORADO - PROFESSOR BAZANELLA

% DESCRIÇÃO:
% CODIGO PARA LER E REALIZAR FITTING COM MINIMOS QUADRADOS DOS DADOS
% FORNECIDOS PELO PROFESSOR, OBJETIVO ESTIMAR O VALOR DO IMOVEL EM NOVO
% HAMBURGO BASEADO NOS DADOS EM mercadoImobiliario_table E PREDIZER A 
% DEMANDA(TOTAL, REGIONAL, ESTADUAL) DE ENERGIA ELETRICA NO BRASIL USANDO 
% OS DADOS EM demandaEletrica_table.


%% INICIANDO O MATLAB:

clc;
clear;
close all;

format short;

%% CARACTERIZACAO DO SOLVER:

% 1 - MINIMOS QUADRADOS TRADICIONAL
% 2 - MINIMOS QUADRADOS RECURSIVOS
solver = 1;

% 1 - SEM ESQUECIMENTO
% 2 - COM ESQUECIMENTO
esquecimento = 1;

% 1 - SEM PONDERACAO
% 2 - COM PONDERACAO
ponderacao = 1;

%% VARIAVEIS GLOBAIS:



%% INPUT DOS DADOS: IMPORTANDO OS DADOS EM FORMATO CSV

% CARREGANDO DADOS IMOBILIARIOS:
mercadoImobiliario_table = readtable('DadosMercadoImobiliarioNH.csv', 'VariableNamingRule', 'preserve');

% CARREGANDO DADOS DA DEMANDA DE ENERGIA ELETRICA:
% demandaEletrica_table = readtable('DadosDemandaEletricaBrasil.csv', 'VariableNamingRule', 'preserve');

%% CONSTRUINDO O MODELO DE MINIMOS QUADRADOS:

% valorImovel = theta_0 + theta_1 * Area + theta_2 * Frente + gamma * theta_3 * IA + theta_4 * (Diatancia_1 + Distancia_2)/2

Area_cell =        mercadoImobiliario_table{4:end, 4};  % AREA TOTAL DO IMOVEL
Frente_cell =      mercadoImobiliario_table{4:end, 5};  % FRENTE DO IMOVEL
Renda_cell =       mercadoImobiliario_table{4:end, 10}; % FRENTE DO IMOVEL
IA_cell =          mercadoImobiliario_table{4:end, 13}; % INDICE DE APROVEITAMENTO
Distancia_1_cell = mercadoImobiliario_table{4:end, 11}; % DISTANCIA ATE POLO 1
Distancia_2_cell = mercadoImobiliario_table{4:end, 12}; % DISTANCIA ATE POLO 2
PrecoImovel_cell = mercadoImobiliario_table{4:end, 14}; % PRECO TOTAL DO IMOVEL

% CORRIGINDO AS CELULAS PARA DOUBLE:
Area = NaN(length(Area_cell), 1);
for i = 1:length(Area_cell)
    temp = strrep(Area_cell{i, 1}, '.', '');
    temp = strrep(temp, ',', '.');
    Area(i, 1) = str2double(temp);
end

Frente = NaN(length(Frente_cell), 1);
for i = 1:length(Frente_cell)
    temp = strrep(Frente_cell{i, 1}, '.', '');
    temp = strrep(temp, ',', '.');
    Frente(i, 1) = str2double(temp);
end

Renda = NaN(length(Renda_cell), 1);
for i = 1:length(Renda_cell)
    temp = strrep(Renda_cell{i, 1}, '.', '');
    temp = strrep(temp, ',', '.');
    Renda(i, 1) = str2double(temp);
end

IA = NaN(length(IA_cell), 1);
for i = 1:length(IA_cell)
    temp = strrep(IA_cell{i, 1}, '.', '');
    temp = strrep(temp, ',', '.');
    IA(i, 1) = str2double(temp);
end

Distancia_1 = NaN(length(Distancia_1_cell), 1);
for i = 1:length(Distancia_1_cell)
    temp = strrep(Distancia_1_cell{i, 1}, '.', '');
    temp = strrep(temp, ',', '.');
    Distancia_1(i, 1) = str2double(temp);
end

Distancia_2 = NaN(length(Distancia_2_cell), 1);
for i = 1:length(Distancia_2_cell)
    temp = strrep(Distancia_2_cell{i, 1}, '.', '');
    temp = strrep(temp, ',', '.');
    Distancia_2(i, 1) = str2double(temp);
end

Distancia = (Distancia_1 + Distancia_2)/2;

precoImovel = NaN(length(PrecoImovel_cell), 1);
for i = 1:length(PrecoImovel_cell)
    temp = strrep(PrecoImovel_cell{i, 1}, '.', '');
    temp = strrep(temp, ',', '.');
    precoImovel(i, 1) = str2double(temp);
end

%% ANALISANDO DADOS:

% figure;
% plot(Area, precoImovel./min(precoImovel), '*');
% grid on;

% figure;
% plot(Frente, precoImovel, '*');
% grid on;

% figure;
% plot(IA, precoImovel, '*');
% grid on;

% figure;
% plot(Distancia, precoImovel, '*');
% grid on;

% labels = {'Area','Front Length','IA','Mean Distance'};
% dataParallelPlot = [Area./max(Area) Frente./max(Frente) IA./max(IA) Distancia./max(Distancia)];
% parallelcoords(dataParallelPlot, 'Labels', labels);

% for i = 1:10:length(precoImovel)
% 
%     labels = {'Area','Front Length', 'Renda', 'IA','Mean Distance'};
%     dataParallelPlot = [Area(i, 1) Frente(i, 1) Renda(i, 1) IA(i, 1) Distancia(i, 1)];
%     figure;
%     parallelcoords(dataParallelPlot, 'Labels', labels);
%     title(['In ', i]);
%     legend(PrecoImovel_cell{i, 1});
% 
% end

%% CONTRUINDO AS MATRIZES PARA O MINIMOS QUADRADOS:

gamma = min(Area);

X = [Area.^2 Frente Renda.^(2/3) .3*gamma.*IA (.2*gamma + Distancia).^2];

theta = (X'*X)\X'*precoImovel;

%% VERIFICANDO OS RESULTADOS:

theta_0 = min(Area);

e = zeros(size(precoImovel, 1), 1);
modeloFitado = zeros(size(precoImovel, 1), 1);
valorImovel_hat = zeros(size(precoImovel, 1), 1);

j = 0;
for i = 1:length(precoImovel)

    valorImovel_hat(i, 1) = .8*theta_0 + theta'*X(i, :)';
    e(i, 1) = precoImovel(i, 1) - valorImovel_hat(i, 1);

    if abs(e(i, 1)) < 500

        modeloFitado(i, 1) = e(i, 1);

    else

        j = j + 1;

    end
end

%% VERFICANDO:

figure;
plot(e, 'r*', 'LineWidth', 1.5);

figure;
plot(modeloFitado, 'ro', 'LineWidth', 1.5);

disp('Valores Descartados:');
disp(j);

%% SALVANDO OS RESULTADOS:

% Define the filename
filename = 'dadoErroEstimacao_precoImovel.mat';

% Define the new variable to save (replace with your actual data variable)
newData = e;  % Example data to append

% Check if the file exists
if exist(filename, 'file') == 2

    % File exists, load the existing data from the file
    load(filename, 'allData');  % Load 'allData' if it exists (or create new if not)
    
    % If 'allData' doesn't exist (on the first run), initialize it
    if ~exist('allData', 'var')
        allData = [];
    end
    
    % Append the new data to the existing data
    allData = [allData; newData];  % Appending newData row-wise
    
    % Save the updated 'allData' variable back into the file
    save(filename, 'allData');
    disp('Data appended to the existing file.');

else

    % File doesn't exist, create it and save the new data as 'allData'
    allData = newData;
    save(filename, 'allData');
    disp('New file created and data saved.');

end

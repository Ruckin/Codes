% Período de amostragem
T = 1.0; % Período de amostragem em segundos

% Dados da simulação para a resposta ao degrau
tempo_degrau = saida_degrau.Time;  % Tempo da simulação
dados_degrau = saida_degrau.Data;  % Valores do sinal

% Verificar o valor final da resposta para degrau
y_final_degrau = dados_degrau(end);

% Caso y_final_degrau seja muito próximo de zero, isso pode causar problemas no cálculo do P.O.
if abs(y_final_degrau) < 1e-6
    y_final_degrau = 1; % Assumimos que o valor esperado é 1 para degrau unitário
end

% Valor máximo da resposta (para calcular P.O)
y_max_degrau = max(dados_degrau);

% Percentual de Overshoot (P.O) para degrau
PO_degrau = ((y_max_degrau - y_final_degrau) / abs(y_final_degrau)) * 100;

% Tempo de acomodação (t_s) para degrau
% Considerando acomodação dentro de 1% do valor final
tolerancia = 0.01;
indices_acomodacao_degrau = find(abs(dados_degrau - y_final_degrau) <= tolerancia * abs(y_final_degrau));
if ~isempty(indices_acomodacao_degrau)
    t_s_index_degrau = indices_acomodacao_degrau(1);  % Pegamos o primeiro índice onde a resposta está dentro da tolerância
    t_s_degrau = tempo_degrau(t_s_index_degrau);
else
    t_s_degrau = NaN; % Caso o sistema não tenha acomodado dentro da tolerância
end

% Erro de regime permanente (e_ss) para degrau
e_ss_degrau = abs(1 - y_final_degrau);  % Considerando degrau unitário

% Exibir os resultados para degrau
fprintf('Percentual de Overshoot (P.O) para saida_degrau: %.2f%%\n', PO_degrau);
fprintf('Tempo de Acomodação (t_s) para saida_degrau: %.2f segundos\n', t_s_degrau);
fprintf('Erro de Regime Permanente (e_ss) para saida_degrau: %.5f\n', e_ss_degrau);



% Dados da simulação para a resposta à rampa
tempo_rampa = saida_rampa.Time;  % Tempo da simulação
dados_rampa = saida_rampa.Data;  % Valores do sinal

% Verificar o valor final da resposta para rampa
y_final_rampa = dados_rampa(end);

% Valor máximo da resposta (para calcular P.O) para rampa
y_max_rampa = max(dados_rampa);

% Percentual de Overshoot (P.O) para rampa
% Valor final esperado para a rampa é o valor no último instante mais a inclinação da rampa (aqui é 1)
valor_final_esperado_rampa = tempo_rampa(end) + 1;
PO_rampa = ((y_max_rampa - valor_final_esperado_rampa) / valor_final_esperado_rampa) * 100;

% Tempo de acomodação (t_s) para rampa
% Considerando acomodação dentro de 1% do valor final
indices_acomodacao_rampa = find(abs(dados_rampa - valor_final_esperado_rampa) <= tolerancia * valor_final_esperado_rampa);
if ~isempty(indices_acomodacao_rampa)
    t_s_index_rampa = indices_acomodacao_rampa(1);  % Pegamos o primeiro índice onde a resposta está dentro da tolerância
    t_s_rampa = tempo_rampa(t_s_index_rampa);
else
    t_s_rampa = NaN; % Caso o sistema não tenha acomodado dentro da tolerância
end

% Erro de regime permanente (e_ss) para rampa
e_ss_rampa = abs(dados_rampa(end) - tempo_rampa(end));  % Considerando rampa unitária

% Exibir os resultados para rampa
fprintf('Percentual de Overshoot (P.O) para saida_rampa: %.2f%%\n', PO_rampa);
fprintf('Tempo de Acomodação (t_s) para saida_rampa: %.2f segundos\n', t_s_rampa);
fprintf('Erro de Regime Permanente (e_ss) para saida_rampa: %.5f\n', e_ss_rampa);


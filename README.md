# sicite-o3_uv
Código da unidade controladora de um medidor de ozônio baseado em fotometria UV

A unidade microcontroladora é responsável pelos seguintes trabalhos:

- Controlar o chaveamento do sistema pneumático, definindo em que momentos será feita a medição da amostra e em que momentos se medirá o ar filtrado;
- Realizar a medição da intensidade de luz em ambos os casos;
- Usar essas medições para calcular a concentração de ozônio em microgramas por metro cúbico a partir da lei de Beer-Lambert, e convertê-los para partes por bilhão com base em medidas de temperatura e pressão atmosférica;
- Registrar e/ou mostrar para o usuário os valores de concentração medidos.

Para esse protótipo a unidade é baseada em um microcontrolador de 8 bits ATmega328P, o mesmo da famosa placa de desenvolvimento \textit{Arduino}, a um clock de 12 MHz.
Essa escolha foi feita no início do projeto pela facilidade de trabalho e pela familiaridade que o ator já tinha com esse microcontrolador, mas ao final desse desenvolvimento inicial ele já se provou limitado para essa aplicação, pelas pequenas memórias RAM e de programa para realizar todas as tarefas acima com uma interface ao usuário satisfatória.

## Chaveamento

Para determinar quando deve ativar e desativar cada uma das válvulas, o código do microcontrolador faz uso de um dos \textit{timers} integrados (especificamente o \textit{timer 1}). \cite{atmega328p_datasheet}

O \textit{timer} é configurado no modo CTC, de modo que quando a contagem alcança o valor do registrador OCR1A ele reinicia a contagem, ao mesmo tempo em que gera uma interrupção (COMPA).
Fazemos também uso do registrador OCR1B, para gerar uma interrupção (COMPB) quando deve ser iniciada a medida do ADC.
A contagem máxima, foi definida para o valor máximo possível (0xffff no registrador OCR1A) para a maior parte dos testes, para que o período de chaveamento seja o maior possível (cerca de 5,6 segundos com um clock de 12 MHz).

Sempre que a interrupção COMPA ocorre o código muda a válvula que está acionada.
Quando há essa mudança de entrada de ar gera-se um transiente no sinal a ser medido, que impossibilita uma medição exata nesse primeiro momento.
Por isso, precisamos esperar um certo período de tempo antes de iniciar o ADC para realizar a medida, e esse instante quando se iniciará a medir é definido pelo valor de OCR1B.
Durante os testes, foi usado metade do valor de OCR1A no registrador OCR1B (0x7fff), para um delay de cerca de 2,8 segundos antes da partida do ADC.

## Conversor AD

Para descobrir o nível médio do sinal dentro da janela de medida, realizamos várias medidas com o ADC ao longo da janela, e ao final tiramos a média de todos os valores registrados.
Essa abordagem é feita para tentar eliminar o efeito de ruídos que estejam sobrepostos ao sinal, e medir somente o nível médio da intensidade.

O ADC é então configurado no modo de disparo contínuo; ativamos ele e damos o disparo inicial no momento da interrupção COMPB, e ele realizará medidas repetidamente até o desativarmos na próxima interrupção COMPA.
Sempre que o ADC conclui uma medida, ele gerará uma interrupção para que possamos lidar com o valor resultante da conversão.
Como vamos apenas realizar a média dos valores, adicionamos o resultado a uma variável de soma e incrementamos a contagem de conversões realizadas; assim no final da janela de medição basta dividir a soma pela contagem para obter a média.

Mas como o código realiza a medição de dois sinais (o de baixa concentração, vindo do circuito de processamento, e o de alta concentração, vindo direto do amplificador de trans impedância), é necessário realizar uma multiplexação, para que o único ADC que temos seja utilizado para medir dois sinais.
Isso também é realizado dentro da interrupção do ADC; sempre que a medida de um dos sinais é finalizada, nós alteramos o registrador ADMUX para que o outro sinal seja selecionado.

Ativamos também o pino PD2 enquanto o ADC está habilitado, para fins de \textit{debug}.

## Cálculo da Concentração

Assim que temos as duas medidas em mãos (a média da intensidade medida com o ar filtrado e a mesma média com o ar da amostra), precisamos obter a diferença entre elas para repassar à função \texttt{calcConc()}, que retorna o valor da concentração de acordo com a lei de Beer-Lambert.
Essa função, além da diferença entre os dois sinais, precisa do valor da intensidade de referência (isto é, com o ar filtrado), que é armazenada em uma variável global para que possa ser acessada depois do final da rotina de serviço da interrupção.

## Interface com o usuário

O código para cálculo e exibição da concentração e feito pela função \texttt{printValue()}, que gera saídas tanto pela interface serial do microcontrolador quanto no display LCD do equipamento.
Por via serial ele envia os valores brutos da medição (intensidade de referência e diferença entre as medidas), bem como os valores medidos de temperatura e pressão por um sensor BMP280, além da concentração em µg/m³ e em ppb.
Já no LCD ele mostra apenas a concentração nas duas unidades.

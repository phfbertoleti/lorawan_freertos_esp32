/* Header com definições da leitura da bateria */

#define GPIO_ADC_BATERIA                 37
#define BITS_RESOLUCAO_ADC               12
#define MAX_VALOR_ADC_BAT                4095
#define FATOR_DIVISOR_TENSAO_BAT         2.0
#define TENSAO_MINIMA_DE_BATERIA         3.2 //V
#define NUMERO_LEITURAS_BATERIA          10
#define MAX_TENSAO_ADC                   3.9
#define TEMPO_ENTRE_LEITURAS_BATERIA        500 //ms

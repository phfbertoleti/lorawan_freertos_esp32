#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>

/* Projeto: estação de medições (temperatura, pressão barométrica e nível de bateria) com LoRaWAN 
 *          (modo: ABP) usando FreeRTOS. Projeto já configurado para operar na rede LoRaWAN 
 *          da ATC / Everynet no Brasil.
 *          Este programa NÃO faz uso do modo sleep (light sleep ou deep sleep) do ESP32. Dessa forma,
 *          não se trata de um projeto low power.
 *          
 * Autor: Pedro Bertoleti e agradecimentos ao professor Marcelus Guirardello por toda a ajuda na codificação da 
 *        comunicação LoRaWAN.
 * 
 * Placa de desenvolvimento utilizada: Heltec LoRa wifi v1 (https://www.curtocircuito.com.br/placa-wifi-lora-32-esp32-lora-display-oled.html?gclid=EAIaIQobChMIqOLj9ZG-6gIVBL7ACh1xvQorEAkYASABEgKLOfD_BwE)
 * 
 * Sensor utilizado: BMP180 (SDA: GPIO4; SCL: GPIO15)
 * 
 * IMPORTANTE:
 * 1) Este projeto considera a tensão da bateria lida no GPIO37 (ADC1_1), onde a tensão é lida num divisor de tensão 
 *    (resistor de 470k / 0,25W e resistor de 100k / 0,25W). 
 * 
 * VBAT -----------
 *                |                               R1: resistor de 470k / 0,25W
 *               ---                              R2: resistor de 100k / 0,25W
 *                R1                              RL: impedância do ADC (calculado: 13M)
 *               --- 
 *                |       ADC1_1 (Vmax: 0.73V)
 *                |---------
 *                |        |
 *               ---      --- 
 *                R2       RL 
 *               ---      --- 
 *                |        |  
 * GND ---------------------
 * 
 *    NÃO SE ESQUEÇA DE USAR O DIVISOR DE TENSÃO AQUI!! O ADC do ESP32 suporta, no máximo, 1,1V (0dB), 
 *    enquanto a tensão de bateria pode chegar a 4,2V.
 * 
 * 2) Esse projeto faz uso da biblioteca "MCCI LoRaWAN LMIC Library". 
 *    Este projeto foi testado com a versão 2.3.2 da mesma.
 *    
 * 3) Antes de compilar, é preciso deixar o arquivo lmic_project_config.h   
 *    (dentro na pasta da biblioteca: project_config/lmic_project_config.h) com o 
 *     conteúdo conforme abaixo:
 *
 *    // project-specific definitions
 *    //#define CFG_eu868 1
 *    //#define CFG_us915 1
 *    #define CFG_au921 1
 *    //#define CFG_as923 1
 *    // #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP      
 *    //#define CFG_in866 1
 *    #define CFG_sx1276_radio 1
 *    //#define LMIC_USE_INTERRUPTS
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include "OLED_defs.h"
#include "BATERIA_defs.h"
#include "RADIO_LORA_defs.h"
#include "LORAWAN_defs.h"
#include "TEMPORIZACOES_defs.h"

/* Definições gerais */
#define BAUDRATE_SERIAL_DEBUG               115200
#define TEMPO_ENTRE_LEITURAS_BMP180         500 //ms

/* Constantes do rádio LoRa: GPIOs utilizados para comunicação
   com rádio SX1276 */
const lmic_pinmap lmic_pins = {
  .nss = RADIO_NSS_PORT,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RADIO_RESET_PORT,
  .dio = {RADIO_DIO_0_PORT, RADIO_DIO_1_PORT, LMIC_UNUSED_PIN},  //dio2 não é utilizado.
};

/* Estruturas */

/* Estrutura que contem leituras de temperatura e pressao do sensor BMP180 */
typedef struct 
{
    float temperatura;
    float pressao;
}TTemp_pressao;

/* Estrutura que contem cada linha que pode ser escrita no display OLED da placa */
typedef struct
{
    char linha2[OLED_LINE_MAX_SIZE];
    char linha3[OLED_LINE_MAX_SIZE];
    char linha4[OLED_LINE_MAX_SIZE];
}TTela_display;

/* Estrutura com os dados já no formado que são enviados via LoRaWAN */
typedef struct __attribute__((__packed__))
{
   char  temperatura;     //1 byte   
   char  tensao_mult_10;  //1 byte
   short pressao_hpa;     //2 byte  
   char  carga_bateria;   //1 byte
}TDados_lorawan;

#define TAMANHO_DADOS_LORAWAN   sizeof(TDados_lorawan)

/* Objetos globais */
Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, 
                         OLED_SCREEN_HEIGHT, 
                         &Wire, 
                         OLED_RESET, 
                         100000UL, 
                         100000UL);

Adafruit_BMP085 bmp180;

/* Semáforo */
SemaphoreHandle_t xI2C_semaphore;
SemaphoreHandle_t xSerial_semaphore;

/* Filas */
QueueHandle_t xQueue_temp_pressao;
QueueHandle_t xQueue_bateria;
QueueHandle_t xQueue_display;

/* Variaveis e objetos globais */
static osjob_t sendjob; //objeto para job de envio de dados via ABP
esp_adc_cal_characteristics_t adc_cal; //Estrutura que contem as informacoes para calibracao

/* Relação de tensão x carga da bateria */
#define PONTOS_MAPEADOS_BATERIA   11
char cargas_mapeadas[PONTOS_MAPEADOS_BATERIA] = { 0,
                                                  3, 
                                                  13,
                                                  22,
                                                  39,
                                                  53,
                                                  62,
                                                  74,
                                                  84,
                                                  94,
                                                  100 };
                             
float tensao_x_carga[PONTOS_MAPEADOS_BATERIA] = {3.2,   //0%
                                                 3.3,   //3%
                                                 3.4,   //13%
                                                 3.5,   //22%
                                                 3.6,   //39%
                                                 3.7,   //53%
                                                 3.8,   //62%
                                                 3.9,   //74%
                                                 4.0,   //84%
                                                 4.1,   //94%
                                                 4.2 }; //100%



/* Tarefas */
void task_oled( void *pvParameters );                         
void task_mede_pressao_temperatura( void *pvParameters );
void task_formata_medicoes_display( void *pvParameters );
void task_envio_lorawan( void *pvParameters );

/* Protótipos */
void init_bmp180(void);
void escreve_alerta_bateria_baixa(void);
char calculo_carga_bateria(float tensao_bateria);
float le_temperatura(void);
float le_pressao(void);
void configura_adc_bateria(void);
float le_tensao_bateria(void);
unsigned long diferenca_tempo(unsigned long tstamp);

/* Função: calcula diferença de tempo (ms) entre tempo atual e referência passada
 * Parâmetros: referÊncia de tempo passada
 * Retorno:  diferença de tempo
 */
unsigned long diferenca_tempo(unsigned long tstamp)
{
    return (millis() - tstamp);
}


/* Função: le temperatura (do sensor BMP180)
 * Parâmetros: nenhum
 * Retorno:  temperatura lida
 */
float le_temperatura(void)
{   
    return bmp180.readTemperature();
}

/* Função: le sensor de pressão barométrica
 * Parâmetros: nenhum
 * Retorno:  pressao lida
 */
float le_pressao(void)
{
    return bmp180.readPressure();
}

/* Função: configura ADC para leitura da tensão de bateria
 * Parâmetros: nenhum
 * Retorno:  nenhum
 */
void configura_adc_bateria(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_0);
    
    esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);
    
    if (adc_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {
            Serial.println("ADC CALV ref eFuse encontrado: ");
            Serial.print(adc_cal.vref);
            Serial.print("mV");
            xSemaphoreGive(xSerial_semaphore);
        }
    }
    else if (adc_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {
            Serial.println("ADC CAL Two Point eFuse encontrado");
            xSemaphoreGive(xSerial_semaphore);
        }
    }
    else
    {
        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {  
            Serial.println("ADC CAL Nada encontrado, utilizando Vref padrao: ");
            Serial.print(adc_cal.vref);
            Serial.print("mV");
            xSemaphoreGive(xSerial_semaphore);
        }
    }
}

/* Função: le tensão da bateria
 * Parâmetros: nenhum
 * Retorno:  tensão da bateria (V)
 */
float le_tensao_bateria(void)
{
    unsigned long leitura_adc_bateria = 0;
    unsigned long soma_leitura_adc_bateria = 0;
    float tensao_bateria = 0.0;
    float tensao_adc = 0.0;
    int i;

    for(i=0; i<NUMERO_LEITURAS_BATERIA; i++)
    {
        leitura_adc_bateria = adc1_get_raw(ADC1_CHANNEL_1);
        soma_leitura_adc_bateria = soma_leitura_adc_bateria + leitura_adc_bateria;
    }

    leitura_adc_bateria = soma_leitura_adc_bateria / NUMERO_LEITURAS_BATERIA;
    tensao_adc = esp_adc_cal_raw_to_voltage(leitura_adc_bateria, &adc_cal);  //unidade: mV
    tensao_adc = tensao_adc / 1000.0; //unidade: V

    /*         bateria                 adc
                  4.1V     -------     0.73V 
           tensao_bateria  -------     tensao_adc  

       tensao_bateria = tensao_adc*(4.1/0.73)
    */

    tensao_bateria = tensao_adc*(4.1/0.73);
    
    return tensao_bateria;
}


/* Função: calcula a porcentagem de carga restante da bateria
 * Parâmetros: tensão da bateria 
 * Retorno: porcentagem de carga restante da bateria (0..100%)
 */
char calculo_carga_bateria(float tensao_bateria)
{
    char carga_bateria = 0;
    char carga_bateria_float = 0.0;
    int i;
    int idx_menor_distancia;
    bool carga_calculada = false;
    float distancias[(PONTOS_MAPEADOS_BATERIA-1)] = {0.0};
    float menor_distancia;
    float x0, y0, x1, y1, m;

    /* Verifica se a tensão da bateria já está nos niveis mapeados */
    for (i=0; i<PONTOS_MAPEADOS_BATERIA; i++)
    {
        if (i < (PONTOS_MAPEADOS_BATERIA - 1))
            distancias[i] = abs(tensao_bateria - tensao_x_carga[i]);
        
        if (tensao_x_carga[i] == tensao_bateria)
        {
            carga_bateria_float = (float)cargas_mapeadas[i];
            carga_calculada = true;
            break;
        }
    }

    if ( carga_calculada == false)
    {
        /* Se a tensão da bateria não está nos níveis mapeados,
           calcula a carga com base na interpolação linear entre os dois níveis
           mapeados mais próximos */
        menor_distancia = distancias[0];
        idx_menor_distancia = 0;
        
        for (i=1; i<(PONTOS_MAPEADOS_BATERIA - 1); i++)
        {
            if ( distancias[i] < menor_distancia )
            {
                menor_distancia = distancias[i];
                idx_menor_distancia = i;                
            }
        }

        //tensão: eixo x
        //carga: eixo y
        //tensao mais prox da mapeada: x0
        //carga mais prox da mapeada: y0
        //tensão mapeada imediatamente acima: x1
        //carga mapeada imediatamente acima: y1
        //Coeficiente angular da reta que passa pelos dois níveis mapeados mais próximos: m = (y1-y0) / (x1 - x0)
        //equação de reta: y = m*(x-x0) + y0 -> y = m*tensoa_bateria -m*x0 + y0

        x0 = tensao_x_carga[idx_menor_distancia];
        y0 = cargas_mapeadas[idx_menor_distancia];
        x1 = tensao_x_carga[idx_menor_distancia + 1];
        y1 = cargas_mapeadas[idx_menor_distancia + 1];        
        m = ( (y1-y0) / (x1 - x0) );       
        carga_bateria_float = ((m*tensao_bateria) - (m*x0) + y0);        
    }

    /* Caso a bateria esteja totalmente carregada, ainterpolação seja feita nos dois últimos níveis mapeados. 
     * Nesse caso, se o ADC apresentar algum erro de leitura para cima, a carga calculada poderá ser ligeiramente
     * maior que 100%. Nesse caso, trava-se a carga em 100%.  
     */
    if (carga_bateria_float > 100.0)
        carga_bateria_float = 100.0;

    carga_bateria = (char)carga_bateria_float;
    return carga_bateria;
}

/* Callbacks para uso cpm OTAA apenas (por este projeto usar ABP, isso, eles 
 *  estão vazios) */
void os_getArtEui (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

void os_getDevEui (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

void os_getDevKey (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

/* Callback de evento: todo evento do LoRaAN irá chamar essa
   callback, de forma que seja possível saber o status da 
   comunicação com o gateway LoRaWAN. */
void onEvent (ev_t ev) 
{
    if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
    {
        Serial.print(os_getTime());
        Serial.print(": ");
        Serial.println(ev);
        xSemaphoreGive(xSerial_semaphore);
    }
    
    switch(ev) 
    {
        case EV_SCAN_TIMEOUT:
            break;
        case EV_BEACON_FOUND:
            break;
        case EV_BEACON_MISSED:
            break;
        case EV_BEACON_TRACKED:
            break;
        case EV_JOINING:
            break;
        case EV_JOINED:
            break;
        case EV_JOIN_FAILED:
            break;
        case EV_REJOIN_FAILED:
            break;
        case EV_TXCOMPLETE:
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) != pdTRUE )
            {
                break;
            }

            /* COntrole do semáforo serial obtido. Printa na serial as informações do evento. */
            Serial.println (millis());
            Serial.println(F("EV_TXCOMPLETE (incluindo espera pelas janelas de recepção)"));

            /* Verifica se ack foi recebido do gateway */
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Ack recebido"));

            /* Verifica se foram recebidos dados do gateway */  
            if (LMIC.dataLen) 
            {
                Serial.println(F("Recebidos "));
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes (payload) do gateway"));
              
                /* Como houve recepção de dados do gateway, os coloca
                   em um array para uso futuro. */
                if (LMIC.dataLen == 1) 
                {
                    uint8_t dados_recebidos = LMIC.frame[LMIC.dataBeg + 0];
                    Serial.print(F("Dados recebidos: "));
                    Serial.write(dados_recebidos);
                }
            }

            /* Devolve o controle do semáforo da serial */
            xSemaphoreGive(xSerial_semaphore);
            
            break;

        case EV_LOST_TSYNC:
            break;
        case EV_RESET:
            break;
        case EV_RXCOMPLETE:
            break;
        case EV_LINK_DEAD:
            break;
        case EV_LINK_ALIVE:
            break;
        case EV_TXSTART:
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
            {
                Serial.println(F("EV_TXSTART"));
                Serial.println (millis());
                Serial.println(LMIC.freq);
                xSemaphoreGive(xSerial_semaphore);
            }
            break;
        default:
            break;
    }
}

/* Função para envio de dados ao gateway LoRaWAN */
void do_send(osjob_t* j)
{
    static uint8_t mydata[TAMANHO_DADOS_LORAWAN];    
    float tensao_bateria_mult_10;
    float tensao_bateria;
    char carga_bateria;
    TDados_lorawan dados_lorawan;
    TTemp_pressao temp_pressao;

    /* le temperatura e pressao */
    xQueuePeek(xQueue_temp_pressao, (void *)&temp_pressao, portMAX_DELAY);

    /* le tensão e calcula carga da bateria */
    xQueuePeek(xQueue_bateria, (void *)&tensao_bateria, portMAX_DELAY);
    tensao_bateria_mult_10 = tensao_bateria*10.0;
    carga_bateria = calculo_carga_bateria(tensao_bateria);

    /* Formata dados a serem enviados na estrutura */    
    dados_lorawan.temperatura =  (char)temp_pressao.temperatura;
    dados_lorawan.tensao_mult_10 = (char)(tensao_bateria_mult_10);
    dados_lorawan.pressao_hpa = (short)(temp_pressao.pressao/100.0);
    dados_lorawan.carga_bateria = carga_bateria;    
    memcpy(mydata, (uint8_t *)&dados_lorawan, TAMANHO_DADOS_LORAWAN);
    
    /* Verifica se já há um envio sendo feito.
       Em caso positivo, o envio atual é suspenso. */
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("OP_TXRXPEND: ha um envio ja pendente, portanto o envio atual nao sera feito"));
            xSemaphoreGive(xSerial_semaphore);
        }
    } 
    else 
    {
        /* Aqui, o envio pode ser feito. */
        /* O pacote LoRaWAN é montado e o coloca na fila de envio. */
        LMIC_setTxData2(4, mydata, sizeof(mydata), 0);

        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Pacote LoRaWAN na fila de envio.")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
    }
}


/* Função: inicializa sensor BMP180
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void init_bmp180(void)
{
    if (!bmp180.begin()) 
    {
        Serial.println("[BMP180] Sensor nao encontrado.");
        while (1) 
        {
            delay(1);  
        }
    }
}

void setup() 
{
    /* Inicializa serial de debug */
    Serial.begin(BAUDRATE_SERIAL_DEBUG);

    /* Inicializa I²C (para comunicação com OLED e BMP180)*/
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    
    /* Inicializa display */
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) 
    { 
        Serial.println("[ERRO] não foi possivel inicializar display. O NodeMCU será reiniciado em 1s...");
        delay(1000);
        ESP.restart();
    }
    else
    {
        Serial.println("Display inicializado.");
        
        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(0,OLED_LINHA_1);
        display.println("Inicializando...");
        display.display();
    }

    /* Inicializa comunicação com sensor BMP180 */
    init_bmp180();

    /* Criação dos semáforos para serial e I²C (usados para comunicação com OLED e BMP180) */
    xSerial_semaphore = xSemaphoreCreateMutex();
    xI2C_semaphore = xSemaphoreCreateMutex();

    if ( (xI2C_semaphore == NULL) || (xSerial_semaphore == NULL) )
    {
        Serial.println("Falha ao criar semáforos.");
        delay(1000);
        ESP.restart(); 
    }

    /* Criação das filas */
    xQueue_temp_pressao = xQueueCreate( 1, sizeof( TTemp_pressao ) );
    xQueue_display = xQueueCreate( 1, sizeof( TTela_display ) );
    xQueue_bateria = xQueueCreate( 1, sizeof( float ) );

    if ( (xQueue_temp_pressao == NULL) || (xQueue_display == NULL) || (xQueue_bateria == NULL) )
    {
        Serial.println("Falha ao criar filas.");
        delay(1000);
        ESP.restart();  
    } 

    /* Agenda execução das tarefas */
    xTaskCreatePinnedToCore(task_oled,  
                           "oled", 
                           4096, 
                           NULL,  
                           5, 
                           NULL, 
                           1);     

    xTaskCreate(task_envio_lorawan ,
                "lorawan",
                4096,   
                NULL,
                6,  
                NULL );
                           
    xTaskCreate(task_mede_pressao_temperatura ,
                "temp_pressao",
                4096,   
                NULL,
                7,  
                NULL );   

    xTaskCreate(task_formata_medicoes_display ,
                "formata_tela",
                4096,   
                NULL,
                8,  
                NULL );                                                     

    xTaskCreate(task_bateria ,
                "bateria",
                4096,   
                NULL,
                9,  
                NULL );                                                                 
}

void loop() 
{
    /* Nada é feito aqui. As tarefas cuidam de tudo. */     
}

/* Tarefa responsável por atualizar display OLED */
void task_oled( void *pvParameters )
{
    TTela_display tela_display;
            
    while(1)
    {
        xSemaphoreTake(xI2C_semaphore, portMAX_DELAY );
        
        if (xQueueReceive(xQueue_display, (void *)&tela_display, TEMPO_PARA_LER_FILAS) == pdTRUE) 
        {
            display.clearDisplay();
            display.setCursor(0,OLED_LINHA_1);
            display.println("       Leituras");
            display.setCursor(0,OLED_LINHA_2);
            display.print(tela_display.linha2);
            display.setCursor(0,OLED_LINHA_3);
            display.print(tela_display.linha3);
            display.setCursor(0,OLED_LINHA_4);
            display.print(tela_display.linha4);
            display.display();
        }

        xSemaphoreGive(xI2C_semaphore);

        vTaskDelay( TEMPO_REFRESH_DISPLAY / portTICK_PERIOD_MS ); 
    }
}

/* Tarefa responsavel por receber todas as medições e colocá-las (formatadas para o display)
   em uma estrutura */
void task_formata_medicoes_display( void *pvParameters )
{
    TTemp_pressao temp_pressao;
    TTela_display tela_display;
    float tensao_bateria;

    /* Tempo para comunciação I²C com display acontecer sem problemas */
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); 

    while(1)
    {
        xQueuePeek(xQueue_temp_pressao, (void *)&temp_pressao, TEMPO_PARA_LER_FILAS);        
        xQueuePeek(xQueue_bateria, (void *)&tensao_bateria, TEMPO_PARA_LER_FILAS);

        sprintf(tela_display.linha2, "T: %.1fC/P: %.1fkPa", temp_pressao.temperatura, (temp_pressao.pressao/1000));
        sprintf(tela_display.linha3, "Bat: %.2fV", tensao_bateria);
        sprintf(tela_display.linha4, "-4-");

        xQueueSend(xQueue_display, (void *)&tela_display, portMAX_DELAY);

        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

/* Tarefa responsavel por medir pressao e temperatura (BMP180) */
void task_mede_pressao_temperatura( void *pvParameters )
{
    TTemp_pressao temp_pressao;

    vTaskDelay( 1000 / portTICK_PERIOD_MS ); 
  
    while(1)
    {
        if( xSemaphoreTake( xI2C_semaphore, TEMPO_PARA_OBTER_SEMAFORO ) == pdTRUE )
        {
            temp_pressao.temperatura = le_temperatura();
            temp_pressao.pressao = le_pressao();
            xQueueOverwrite(xQueue_temp_pressao, (void *)&temp_pressao);
            xSemaphoreGive(xI2C_semaphore);
        }
        
        vTaskDelay( TEMPO_ENTRE_LEITURAS_BMP180 / portTICK_PERIOD_MS );
    }   
}

/* Tarefa responsavel por enviar via LoRaWAN as medições */
void task_envio_lorawan( void *pvParameters )
{
    int b;
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    unsigned long timestamp_envio;
    
    /* Inicializa comunicação SPI com rádio LoRa */
    SPI.begin(RADIO_SCLK_PORT, RADIO_MISO_PORT, RADIO_MOSI_PORT);

    /* Inicializa stack LoRaWAN */
    os_init();
    LMIC_reset();

    /* Inicializa chaves usadas na comunicação ABP */
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

    /* Faz inicializações de rádio pertinentes a região do 
       gateway LoRaWAN (ATC / Everynet Brasil) */
    for (b=0; b<8; ++b) 
        LMIC_disableSubBand(b);

    LMIC_enableChannel(0); // 915.2 MHz
    LMIC_enableChannel(1); // 915.4 MHz
    LMIC_enableChannel(2); // 915.6 MHz
    LMIC_enableChannel(3); // 915.8 MHz
    LMIC_enableChannel(4); // 916.0 MHz
    LMIC_enableChannel(5); // 916.2 MHz
    LMIC_enableChannel(6); // 916.4 MHz
    LMIC_enableChannel(7); // 916.6 MHz

    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);

    /* Data rate para janela de recepção RX2 */
    LMIC.dn2Dr = DR_SF12CR;

    /* Configura data rate de transmissão e ganho do rádio
       LoRa (dBm) na transmissão */
    LMIC_setDrTxpow(DR_SF12, GANHO_LORA_DBM);

    /* Força envio do primeiro pacote */
    do_send(&sendjob);

    /* Inicializa temporização para envio LoRaWAN */
    timestamp_envio = millis();

    while(1)
    {
        /* Envia um pacote LoRaWAN de acordo com periodicidade definida em TX_INTERVAL */
        if (diferenca_tempo(timestamp_envio) >= TX_INTERVAL*1000)
        {
            do_send(&sendjob);
            timestamp_envio = millis();
        }

        os_runloop_once();

        vTaskDelay( 10 / portTICK_PERIOD_MS );
    }
}

/* Tarefa responsavel por ler a tensão da bateria */
void task_bateria( void *pvParameters )
{
    float tensao_bateria;
        
    /* Configura ADC da bateria */
    configura_adc_bateria(); 

    while(1)
    {
        /* Le o ADC considerando sua calibração */
        tensao_bateria = le_tensao_bateria();
        xQueueOverwrite(xQueue_bateria, (void *)&tensao_bateria);
        vTaskDelay( TEMPO_ENTRE_LEITURAS_BATERIA / portTICK_PERIOD_MS );
    }
}

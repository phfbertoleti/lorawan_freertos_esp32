# Projeto - comunicação LoRaWAN, usando SX1276 e ESP32

Este repositório contém um projeto de end-device LoRaWAN (ABP), utilizando como placa de desenvolvimento o heltec WiFi LoRa v1 (alimentado a bateria ou via cabo USB) e como sensor o BMP180. Esta placa de desenvolvimento conta com o SX1276 como rádio LoRa e o ESP32 como SoC. Todo o software embarcado é feito utilizando o FreeRTOS como sistema operacional embarcado. Esse projeto deve ser aberto e compilado na Arduino IDE. O projeto possui as seguintes finalidades:

* Leitura periódica da pressão barométrica (hPa) e da temperatura ambiente (°C) medidos pelo sensor BMP180
* Leitura periódica da tensão de bateria (V) e carga da bateria (0 .. 100%)
* Envio periódico (30 em 30 minutos) das medições para o gateway LoRaWAN (o software embarcado já está preparado para funcionar com a operadora ATC)
* Exibição das medições no display OLED 128x64 contido na placa de desenvolvimento.

**IMPORTANTE:**
1) Este projeto considera a tensão da bateria lida no GPIO37 (ADC1_1), onde a tensão é lida num divisor de tensão  (dois resistores de 220k / 0,25W). 
NÃO SE ESQUEÇA DE USAR O DIVISOR DE TENSÃO AQUI!! O ADC do ESP32 suporta, no máximo, 3,9V (quando em 11dB de atenuação), enquanto a tensão de bateria pode chegar a 4,2V.
 
2) Esse projeto faz uso da biblioteca "MCCI LoRaWAN LMIC Library". Este projeto foi testado com a versão 2.3.2 da mesma.
3) Antes de compilar, é preciso deixar o arquivo lmic_project_config.h (dentro na pasta da biblioteca: project_config/lmic_project_config.h) com o conteúdo conforme abaixo:
```
// project-specific definitions
//#define CFG_eu868 1
//#define CFG_us915 1
#define CFG_au921 1
//#define CFG_as923 1
// #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP      
//#define CFG_in866 1
#define CFG_sx1276_radio 1
//#define LMIC_USE_INTERRUPTS
```
4) Você precisará da network session key, application session key (definidos por você ou pela operadora da ATC) e do Device Address (fornecido pela operadora em caráter experimental ou adquirido por meios oficiais). Substitua estas informações no arquivo LORAWAN_defs.h.
Para obtenção das chaves e tudo mais em termos de conectividade 
LoRaWAN, entre em contato com uma das empresas credenciadas pela ATC:
 
https://iotopenlabs.io/home/catalogo-de-solucoes/conectividade-lorawan/

5) O circuito esquemático encontra-se na pasta "ckt_esquematico", tanto como figura como no formato Fritzing.

Este projeto é de autoria de Pedro Bertoleti. 
Agradecimentos ao professor Marcelus Guirardello (ETEC - Bento Quirino - Campinas-SP) por toda a ajuda na codificação da comunicação LoRaWAN.

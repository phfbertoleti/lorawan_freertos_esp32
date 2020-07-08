/* Header com definições e constantes do LoRaWAN */

/* Constantes do LoraWAN */
/* - Chaves (network e application keys) */
static const PROGMEM u1_t NWKSKEY[16] = {  }; //coloque aqui sua network session key
static const u1_t PROGMEM APPSKEY[16] = {  }; //coloque aqui sua application session key

/* - Device Address */
static const u4_t DEVADDR = 0xcecd7ccd;

/* - Tempo entre envios de pacotes LoRa */
const unsigned TX_INTERVAL = 1800; //tempo (em segundos) entre duas transmissoes LoRaWAN

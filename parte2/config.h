/************************ Configurações do Adafruit IO *******************************/

//Insira seu nome de usuário e chave. Você encontra essas informações acessando
//sua conta no Adafruit IO
#define IO_USERNAME  "user_name"
#define IO_KEY       "aio_key"

/********************* Configuração do WIFI **************************************/

//Insira o SSID e Senha da rede WIFI a qual você irá conectar

#define WIFI_SSID "ssid"
#define WIFI_PASS "password"


/***************Configurações do comunicaçaão************/
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

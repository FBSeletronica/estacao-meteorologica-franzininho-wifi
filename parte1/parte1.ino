/*
 * Estação Meteorológica Franzininho WiFi
 * publicação de valores no Adafruit IO
 * Por: Fábio Souza
 * 
 * DHT11
 */

/************************** Configuração***********************************/
#include "config.h"

/**************************   DHT 11    **********************************/
#include "DHT.h"

#define DHTPIN 2         //pino para leitura do DHT11
#define DHTTYPE DHT11

DHT dht(DHTPIN,DHTTYPE);


/************************ Configuração dos feeds *******************************/

// configura o tópico "fs_embarcados/feeds/sensorUmidade"
AdafruitIO_Feed *feedSensorUmidade = io.feed("sensorUmidade");

// configura o tópico "fs_embarcados/feeds/sensorTemperatura"
AdafruitIO_Feed *feedSensorTemperatura = io.feed("sensorTemperatura");


/***********************Variáveis Globais*****************************************/
long ultimaLeituraSensor = 0;

/************************ Função setup *******************************/

void setup() {
  // configura comunicação serial
  Serial.begin(115200);
  dht.begin();
  conectaAdafruitIO(); //função para conectar ao broker

}

/************************ Função loop *******************************/

void loop() {

  // processa as mensagens e mantêm a conexão ativa
  byte state = io.run();

  //verifica se está conectado
  if(state == AIO_NET_DISCONNECTED | state == AIO_DISCONNECTED){
    conectaAdafruitIO(); //função para conectar ao broker
  }
  
  if((millis() - ultimaLeituraSensor)> 30000)
  {
    enviaDHT();
    ultimaLeituraSensor = millis();    
  }  

}

void enviaDHT(void){

  float umidade = dht.readHumidity();
  float temperatura = dht.readTemperature();

  if(isnan(temperatura) || isnan(umidade)){

    Serial.println("Falha na leitura do DHT11");
  }
  else
  {
    Serial.print("Leitura DHT11 - umidade: ");
    Serial.print(umidade);
    Serial.print("  temperatura: ");
    Serial.print(temperatura);
    Serial.println(" °C");

    feedSensorUmidade-> save(umidade);
    delay(1000);
    feedSensorTemperatura-> save(temperatura);
  }

  
}

/****** Função para conectar ao WIFI e Broker***************/

void conectaAdafruitIO(){
  
  //mensagem inicial
  Serial.print("Conectando ao Adafruit IO");

  // chama função de conexão io.adafruit.com
  io.connect();

  // Aguarda conexação ser estabelecida
  while(io.status() < AIO_CONNECTED) {
    Serial.println("Aguardando a conexão ser estabelecida");
    delay(500);
  }

  // Conectado
  Serial.println();
  Serial.println(io.statusText());

}

/*
 * Estação Meteorológica Franzininho WiFi
 * publicação de valores no Adafruit IO
 * Por: Fábio Souza
 * 
 * DHT11
 * LED
 * Sensor de Chuva
 * Sensor de Luz
 */

/************************** Configuração***********************************/
#include "config.h"

/**************************   DHT 11    **********************************/
#include "DHT.h"

#define DHTPIN 2         //pino para leitura do DHT11
#define DHTTYPE DHT11

DHT dht(DHTPIN,DHTTYPE);

/************************ Mapeamento de IO *******************************/
#define L1  3 // pino onde o LED 1 está ligado
#define L2  4 // pino onde o LED 2 está ligado
#define L3  5 // pino onde o LED 3 está ligado
#define S1  6 //pino de entrada do sensor 1 - digital

#define S_LUZ A0


/************************ Configuração dos feeds *******************************/

// configura o tópico "fs_embarcados/feeds/sensorUmidade"
AdafruitIO_Feed *feedSensorUmidade = io.feed("sensorUmidade");

// configura o tópico "fs_embarcados/feeds/sensorTemperatura"
AdafruitIO_Feed *feedSensorTemperatura = io.feed("sensorTemperatura");

// configura o tópico "fs_embarcados/feeds/L1"
AdafruitIO_Feed *feedL1 = io.feed("L1");

// configura o tópico "fs_embarcados/feeds/L2"
AdafruitIO_Feed *feedL2 = io.feed("L2");

// configura o tópico "fs_embarcados/feeds/L3"
AdafruitIO_Feed *feedL3 = io.feed("L3");

// configura o tópico "fs_embarcados/feeds/S1"
AdafruitIO_Feed *feedS1 = io.feed("S1");

// configura o tópico "fs_embarcados/feeds/SensorLuz"
AdafruitIO_Feed *feedS_LUZ = io.feed("SensorLuz");

/***********************Variáveis Globais*****************************************/
long ultimaLeituraSensor = 0;
bool statusAntS1 = 0;

/************************ Função setup *******************************/

void setup() {

  pinMode(L1,OUTPUT);
  pinMode(L2,OUTPUT);
  pinMode(L3,OUTPUT);
  
  //confiugura pino do sensor como entrada
  pinMode(S1,INPUT_PULLUP);
  
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
    leituraSensorDigital();
    enviaDHT();
    enviaSensorLuz();
    ultimaLeituraSensor = millis();    
  }  

}

void enviaSensorLuz(){
  int valorSensor = analogRead(S_LUZ);
  feedS_LUZ -> save(valorSensor);
  Serial.print("Publicado em fabiosouza_io/feeds/S_LUZ: ");
  Serial.println(valorSensor);
}

void leituraSensorDigital(){
  bool statusS1 = digitalRead(S1);
  
  if(statusS1 != statusAntS1)
  {
      statusAntS1 = statusS1;

      if(statusS1)
        feedS1->save("sun-o");
      else
        feedS1->save("w:rain");
        
      Serial.print("Publicado em fabiosouza_io/feeds/S1: ");
      Serial.println(statusS1);

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

//***********Função para atualizar LED 1 ************
void trataL1(AdafruitIO_Data *data){

  //mesagem que chegou
  Serial.print("Recebido <- ");
  Serial.print(data->feedName());
  Serial.print(" :  ");
  Serial.println(data->value());

  if(data->isTrue()){
    digitalWrite(L1,HIGH);
  }
  else{
    digitalWrite(L1,LOW);
  }
  
}

//***********Função para atualizar LED 2 ************
void trataL2(AdafruitIO_Data *data){

  //mesagem que chegou
  Serial.print("Recebido <- ");
  Serial.print(data->feedName());
  Serial.print(" :  ");
  Serial.println(data->value());

  if(data->isTrue()){
    digitalWrite(L2,HIGH);
  }
  else{
    digitalWrite(L2,LOW);
  }
  
}

//***********Função para atualizar LED 3 ************
void trataL3(AdafruitIO_Data *data){

  //mesagem que chegou
  Serial.print("Recebido <- ");
  Serial.print(data->feedName());
  Serial.print(" :  ");
  Serial.println(data->value());

  if(data->isTrue()){
    digitalWrite(L3,HIGH);
  }
  else{
    digitalWrite(L3,LOW);
  }
  
}

/****** Função para conectar ao WIFI e Broker***************/

void conectaAdafruitIO(){
  
  //mensagem inicial
  Serial.print("Conectando ao Adafruit IO");

  // chama função de conexão io.adafruit.com
  io.connect();

  feedL1->onMessage(trataL1);
  feedL2->onMessage(trataL2);
  feedL3->onMessage(trataL3);

  // Aguarda conexação ser estabelecida
  while(io.status() < AIO_CONNECTED) {
    Serial.println("Aguardando a conexão ser estabelecida");
    delay(500);
  }

  // Conectado
  Serial.println();
  Serial.println(io.statusText());

  feedL1->get();
  feedL2->get();
  feedL3->get();

}

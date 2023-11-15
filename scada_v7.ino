//-----BIBLIOTECAS------//
#include <SimpleModbusSlave.h>
#include <OneWire.h>  
#include <DallasTemperature.h>


//-----PINOS------//
const int P_PH = A0; 
#define P_TEMP 8
#define P_ON 4
#define P_OFF 5
#define P_VAZAO1 2
#define P_VAZAO2 3
#define P_BOMBA1 6
#define P_BOMBA2 7 
#define P_RES 9
#define P_MOTOR 10


//-----VARIAVEIS------//
float temp; //ph
int buf[10]; //ph
int tempo=0; //ph
unsigned long int inValue; //ph
int F_ON = 0; //botao
int F_OFF = 0; //botao
double erro = 0; //controle
double soma_erro = 0; //controle
int saida = 0; //controle
unsigned long startTime1 = 0; //bomba
bool timerRunning1 = false;   //bomba
unsigned long currentTime1 = 0; //bomba
unsigned long elapsedTime1 = 0; //bomba
float elapsedSeconds1 = 0; //bomba
unsigned long startTime2 = 0; //bomba
bool timerRunning2 = false;   //bomba
unsigned long currentTime2 = 0; //bomba
unsigned long elapsedTime2 = 0; //bomba
float elapsedSeconds2 = 0; //bomba
float TimeValue = 3.72; //bomba
float TimeValue2 = 3.72;//3.87; //bomba
float VolumeA = 0.02; //bomba
float VolumeB = 0.02; //bomba
int condicao = 0; //bomba
//definicao das variaveis de fluxo e volume
const int INTERRUPCAO_SENSOR = 1;
const int INTERRUPCAO_SENSOR2 = 0;
unsigned long timevazao1 = 0;
unsigned long timevazao2 = 0;
float fluxo = 0;
float volume = 0;
float volume_total = 0;
unsigned long tempo_antes = 0;
float fluxo2 = 0;
float volume2 = 0;
float volume_total2 = 0;
unsigned long tempo_antes2 = 0;
volatile long contador = 0;
volatile long contador2 = 0;
float fluxo_sensor1, volume_sensor1, fluxo_sensor2, volume_sensor2;

//TEMPERATURA
OneWire oneWire(P_TEMP); 
DallasTemperature sensors(&oneWire);


////////////////REGISTRADORES VARIAVEIS COMUNICACAO SCABABR///////////////////
enum 
{     
  TEMP_VAL,
  PH_VAL,
  VOL_VAL,
  RES,
  LED_ON,
  LED_OFF,
  MIX,
  MIXALL,
  MOTOR,
  ST,
  MODO,
  KP,
  KI,
  QUANTA,
  QUANTB,
  BOMBA,
  BOMBB,
  RES_ON,
  MOTOR_ON,
  VAZAO1,
  VAZAO2,
  ESVAZIAR,
  HOLDING_REGS_SIZE
};
unsigned int holdingRegs[HOLDING_REGS_SIZE];
////////////////////////////////////////////////////////////

void setup()
{
  modbus_configure(&Serial, 9600, SERIAL_8N1, 1, 2, HOLDING_REGS_SIZE, holdingRegs);
  modbus_update_comms(9600, SERIAL_8N1, 1);
  sensors.begin();
  pinMode(P_ON,INPUT);
  pinMode(P_OFF,INPUT);
  pinMode(P_VAZAO1, INPUT);
  pinMode(P_VAZAO2, INPUT_PULLUP);
  pinMode(P_BOMBA1,OUTPUT);
  pinMode(P_BOMBA2,OUTPUT);
  pinMode(P_RES,OUTPUT);
  pinMode(P_MOTOR,OUTPUT);
}

void loop()
{
  modbus_update();
//////////////////////////BOTAO DE LIGA E DESLIGA///////////////////////////////////
  if(digitalRead(P_ON) == 1){
    F_ON = 1;
    F_OFF = 0;
    holdingRegs[LED_ON] = 1;
    holdingRegs[LED_OFF] = 0;
  }
  if(digitalRead(P_OFF) == 1){
    F_ON = 0;
    F_OFF = 1;
    holdingRegs[LED_ON] = 0;
    holdingRegs[LED_OFF] = 1;   
  }
//////////////////////////LEITURA DOS SENSORES/////////////////////////////////// 
   TEMPERATURA();
   PH();
   vazao1(volume_sensor1);
   //vazao2(volume_sensor2);
   holdingRegs[VOL_VAL] = (volume_sensor1)*100; //+volume_sensor2
   
///////////////////////////ESVAZIAR TANQUE//////////////////////////////////
   if (holdingRegs[ESVAZIAR] == 1 && (F_ON == 1)){
    holdingRegs[VOL_VAL] = 0;
    volume_sensor1 = 0;
    volume_sensor2 = 0;
   }
/////////////////////////////BOMBEAR LIQUIDOS////////////////////////////////
   if ((holdingRegs[MIX] == 1) && (F_ON == 1) && (F_OFF == 0) && !timerRunning1 && !timerRunning2 && condicao == 0){ 
    digitalWrite(P_BOMBA1,HIGH);
    digitalWrite(P_BOMBA2,HIGH);
    holdingRegs[BOMBA] = 1;
    holdingRegs[BOMBB] = 1;
    startTime1 = millis();          // Grava o tempo de início
    timerRunning1 = true;   // Indica que o cronômetro está em execução
    startTime2 = millis();          // Grava o tempo de início
    timerRunning2 = true;
    VolumeA = holdingRegs[QUANTA]/100;
    VolumeB = holdingRegs[QUANTB]/100;
   }
   if (timerRunning1) {
    currentTime1 = millis();
    elapsedTime1 = currentTime1 - startTime1;
    // Converte o tempo decorrido de milissegundos para segundos
    elapsedSeconds1 = elapsedTime1 / 1000.0;
  }
    if (elapsedSeconds1 > (VolumeA*TimeValue)){
    digitalWrite(P_BOMBA1,LOW);
    holdingRegs[BOMBA] = 0;
    holdingRegs[MIX] == 0;
    timerRunning1 = false;
    condicao = 1;
  }
   if (timerRunning2) {
    currentTime2 = millis();
    elapsedTime2 = currentTime2 - startTime2;
    // Converte o tempo decorrido de milissegundos para segundos
    elapsedSeconds2 = elapsedTime2 / 1000.0;
  }
    if (elapsedSeconds2 > (VolumeB*TimeValue2)){
    digitalWrite(P_BOMBA2,LOW);
    holdingRegs[BOMBB] = 0;
    holdingRegs[MIX] == 0;
    timerRunning2 = false;
    condicao = 1;
  }
   if ((holdingRegs[MIX] == 0) && (F_ON == 1) && (F_OFF == 0)){
    elapsedTime1 = 0;
    elapsedSeconds1 = 0;
    elapsedTime2 = 0;
    elapsedSeconds2 = 0;
    condicao = 0;
   }
//////////////////////////LIGAR MOTOR///////////////////////////////////
   if((holdingRegs[MOTOR_ON] == 1)&& (F_ON == 1) ){
    analogWrite(P_MOTOR, 1);
   }
   if((holdingRegs[MOTOR_ON] == 0)&& (F_ON == 1) ){
    analogWrite(P_MOTOR, 0);
   }
//////////////////////////LIGAR AQUECIMENTO///////////////////////////////////
   if ((holdingRegs[MODO] == 1) &&(holdingRegs[ST] != 0) && (F_ON == 1) && (F_OFF == 0) ){ //&& (holdingRegs[VOL_VAL] >= 350)
    controle(50,0.05);
   }
   if ((holdingRegs[MODO] == 0) &&(holdingRegs[ST] != 0) && (F_ON == 1) && (F_OFF == 0)  ){ //&& (holdingRegs[VOL_VAL] >= 350
    controle(holdingRegs[KP],(holdingRegs[KI]/100));
   }
////////////////////////DESLIGAR/////////////////////////////////////
   if (F_OFF == 1){
    holdingRegs[BOMBA] = 0;
    holdingRegs[BOMBB] = 0;
    holdingRegs[ST] = 0;
    holdingRegs[KP] = 0;
    holdingRegs[KI] = 0;
    holdingRegs[RES_ON] = 0;
    holdingRegs[MIXALL] = 0;
    holdingRegs[MIX] = 0;
    holdingRegs[RES] = 0;
    holdingRegs[MOTOR_ON] = 0;
    saida = 0;
    erro = 0;
    soma_erro = 0;
    condicao = 0;
    volume_sensor1 = 0;
    volume_sensor2 = 0;
    analogWrite(P_RES, 0);
    analogWrite(P_MOTOR, 0);
    digitalWrite(P_BOMBA1,LOW);
    digitalWrite(P_BOMBA2,LOW);
   }
}



////////////////////////FUNCAO PH/////////////////////////////////////
void PH(){
   for(int i=0; i<10;i++){
      buf[i]= analogRead(P_PH);
      delay(10);
   }
    for(int i=0; i<9; i++){
      for(int j=i+1;j<10;j++){
        tempo= buf[i];
        buf[i]=buf[j];
        buf[j]=tempo;
      }  
    }
    inValue=0;
    for(int i=2; i<8; i++){
        inValue= inValue + buf[i];
    }
    float PHVol= (float)inValue*100*5/1024/6;
    float pH = (float)-0.0541*PHVol+30.541;
    if (pH <0) {
      pH = 0;
    }
    if (pH >14) {
      pH = 14;
    }
    holdingRegs[PH_VAL] = pH*100;
}
////////////////////////FUNCAO TEMPERATURA/////////////////////////////////////
void TEMPERATURA(){
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
  if (temp == -127){
    temp = 0;
  }
  holdingRegs[TEMP_VAL] = temp*100;
} 
////////////////////////FUNCAO CONTROLE DE TEMPERATURA/////////////////////////////////////
void controle(double kp, double ki){
  erro = holdingRegs[ST] - temp;
  saida = kp*(erro+ki*(erro+soma_erro));
  soma_erro = soma_erro + erro;
  if (saida < 0){
    saida =0;
  }
  if (saida > 200){
    saida = 200;
    soma_erro = 0;
  }
   analogWrite(P_RES,saida);
   holdingRegs[RES] = saida;
   if (saida != 0){
    holdingRegs[RES_ON] = 1;
   }
   else{
    holdingRegs[RES_ON] = 0;
   }
}
////////////////////////FUNCAO VAZAO/////////////////////////////////////
  float vazao1(float& volume_total) {
  timevazao1 = millis();
  // Executa a contagem de pulsos uma vez por segundo
  if ((timevazao1 - tempo_antes) > 1000) {

    // Desativa as interrupções apenas para o sensor 1
    
 
    detachInterrupt(INTERRUPCAO_SENSOR);
    

    // Calcula a taxa de fluxo e o volume para o sensor 1
    fluxo = ((1000.0 / (timevazao1 - tempo_antes)) * contador) / 10;
    holdingRegs[VAZAO1] = fluxo*100;
    volume = fluxo / 60;
    volume_total += volume;
    tempo_antes = timevazao1;
    contador = 0;
    // Reativa a interrupção do sensor 1

    attachInterrupt(INTERRUPCAO_SENSOR, contador_pulso, FALLING);
    // Continuar com o armazenamento de dados e reinicialização de contadores
    // Retorna os valores calculados
    return;
  }
}

float vazao2(float& volumetotal2) {
  timevazao2 = millis();
  
   if ((timevazao2 - tempo_antes2) > 1000) {
   
 // Desativa as interrupções apenas para o sensor 2

    detachInterrupt(INTERRUPCAO_SENSOR2);

    // Calcula a taxa de fluxo e o volume para o sensor 2
    fluxo2 = ((1000.0 / (timevazao2 - tempo_antes2)) * contador2) * 2.25 / 1000;
    holdingRegs[VAZAO2] = fluxo2*100;
    volume2 = fluxo2 / 60;
    volumetotal2 += volume2;
    tempo_antes2 = timevazao2;
    contador2 = 0;
    // Reativa a interrupção do sensor 2
   
    attachInterrupt(INTERRUPCAO_SENSOR2, contador_pulso2, FALLING);
    // Continuar com o armazenamento de dados e reinicialização de contadores
    
    // Retorna os valores calculados
    return;
  }
}

void contador_pulso() {
  contador++;
}
void contador_pulso2() {
  contador2++;
}

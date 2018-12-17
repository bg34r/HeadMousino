/*
* HeadMousino Receptor
* Codigo que recebe os valores dos sensores (acel, giro, piezo)
* e os usa para movimentar o cursor do mouse e realizar o click esquerdo
*                
*Baseado nos seguintes códigos:                
* https://github.com/adafruit/Adafruit_CircuitPlayground/blob/master/examples/accel_mouse/accel_mouse.ino 
* foi retirado a funcao  lerp  de interpolacao linear          
* 
* https://github.com/jrowberg/i2cdevlib 
* como tratar os dados (acelerometro  e  giroscopio)
* 
* http://www.arduino.cc/en/Tutorial/JoystickMouseControl 
* exemplo de movimentacao do mouse e interpolacao
* 
* https://github.com/tmrh20/RF24/
* exemplo de comunicacao dos radios RF24
* by Bruno Araujo, @github
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Mouse.h>

// ================================================================
// ===                           RADIO                          ===
// ================================================================

RF24 radio(7, 8); // CE, CSN Default: 7,8 (como no lado do receptor não estamos usando o pino 7, deixamos como está)
const byte address[6] = "Mouse";  //código do mouse

// ================================================================
// ===                           MOUSE                          ===
// ================================================================
// set pin numbers for switch, and LED:
const int switchPin = 10;      // Botao/Switch para ligar/desligar a função do mouse
const int ledPin = 13;         // LED indicativo do controle do mouse
//variables to turn on/off the mouse
bool mouseIsActive = false;    // quando ou não ativar o mouse
int lastSwitchState = HIGH;        // estado anterior do botao/switch
// threshold para decidir quando e ou nao efetuado o click do mouse
const int threshold = 60;
// configuracao para ajustar a sensitividade e velocidade do mouse.
// X axis (esquerda/direita) configuracao:
#define XACCEL_MIN 0.1      // Valor minimo da aceleracao no eixo X , valores abaixo
                            // nao irao mexer o mouse.
#define XACCEL_MAX 30.0      // Valor maximo da aceleracao no eixo X, valores acima
                            // irao mover o mouse o mais rápido possível.
#define XMOUSE_RANGE 120   // Amplitude de velocidade para o movimento do mouse. Quanto maior
                            // este valor, mais rapidamente o mouse ira se mover.
#define XMOUSE_SCALE 1      // Scala para aplicar o movimento do  mouse, isto é
                            // util ao setarmos para -1 para inverter o movimento no eixo X.

// Eixo Y (Cima/Baixo) configuracao:
// Veja que os valores sao os mesmo de  X,
// apenas aplicado ao eixo Y e movimento cima/baixo. E desejavel manter
// estes valores iguais ao de X (o que e feito por padrao, eles apenas
// leem os valores de X, mas  voce pode alteralos com outros valores abaixo.
#define YACCEL_MIN XACCEL_MIN
#define YACCEL_MAX XACCEL_MAX
#define YMOUSE_RANGE XMOUSE_RANGE
#define YMOUSE_SCALE 1

// Funcao de interpolacao linear que pega um valor dentro de um limite/intercecao
// e mapeia em outro limite/intersecao. Isto e usado para transformar o valor medido
// no giroscopia na velocidade/aceleracao do mouse. Mais detalhes em:
// https://en.wikipedia.org/wiki/Linear_interpolation
// ou https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
int int_lerp(int x, int  x0, int x1, int y0, int y1){
    if (x <= x0){
      return y0;
    }else if( x >= x1){
      return y1;  
    }
    return y0 + (y1-y0)*((x-x0)/(x1-x0));
  }

float lerp(float x, float x0, float x1, float y0, float y1) {
  // Verifica se o  valor de entrada (x) esta fora da amplitude desejada e, sendo  assim, o corta para
  // os valores max/min em y.
  if (x <= x0) {
    return y0;
  }
  else if (x >= x1) {
    return y1;
  }
  // Se nao, computa o  valor desejado y baseado na posicao x, dentro dos  seus  limites  e 
  // min/max y desejados.
  return y0 + (y1-y0)*((x-x0)/(x1-x0));
}

//Recebe valor cru do giroscópio e devolve em graus por segundo
int gyro_motion(int16_t g){
    int j = g/131;  //-+250 graus/segundo
    //Serial.println(j);
    return j;
  }

void setup() {
  //Serial.begin(9600);
  //while (!Serial);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  pinMode(switchPin,INPUT);
  digitalWrite(switchPin, HIGH);
  pinMode(ledPin, OUTPUT);
  Mouse.begin();
}
void loop() {
  int switchState = digitalRead(switchPin);
  //se o estado do switch   for alterado, muda o estado
  if (switchState != lastSwitchState) {
    if (switchState == LOW) {
      mouseIsActive = !mouseIsActive;
      // liga o led  para indicar o funcionamento do mouse
      digitalWrite(ledPin, mouseIsActive);
      //Serial.println("Mouse active");
      delay(20);
    }
  }
  // salva o estado do switch para próxima comparacao
  lastSwitchState = switchState;
  
  if (radio.available()) {//Se receber dados, move o mouse
    int16_t payload[7] = {};
    radio.read(&payload, sizeof(payload));
    /*
    Serial.print(payload[0]);
    Serial.print("\t");
    Serial.print(payload[1]);
    Serial.print("\t");
    Serial.print(payload[2]);
    Serial.print("\t");
    Serial.print(payload[3]);
    Serial.print("\t");
    Serial.print(payload[4]);
    Serial.print("\t");
    Serial.print(payload[5]);
    Serial.print("\t");
    Serial.println(payload[6]);
    */
    if (payload[6] >= threshold && mouseIsActive) {//analisa o valor do sensor piezo, e sendo maior que nosso threshold, ativa o click do  mouse
    // toggle the status of the ledPin:
    //ledState = !ledState;
    // update the LED pin itself:
    //digitalWrite(ledPin, ledState);
    // send the string "Knock!" back to the computer, followed by newline
    // se o mouse-click nao esta pressionado, pressione-o:
    if (!Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.press(MOUSE_LEFT);
      //Serial.println("Pressed!");
       
      }
    //Serial.println("Pressed!");
    }
    else {
      // se ele estava presionado, libere-o:
      if (Mouse.isPressed(MOUSE_LEFT)) {
        Mouse.release(MOUSE_LEFT);
        //Serial.println("End of click!");
      }
    }
    
    float x = gyro_motion(payload[4]);
    float y = gyro_motion(payload[5]);
    // Usa a magnitude da acceleration/gyro para interpolar a velocidade do mouse.
    float x_mag = abs(x);
    float x_mouse = lerp(x_mag, XACCEL_MIN, XACCEL_MAX, 0, XMOUSE_RANGE);
    float y_mag = abs(y);
    float y_mouse = lerp(y_mag, YACCEL_MIN, YACCEL_MAX, 0, YMOUSE_RANGE);
    //mudar a direcao do mouse baseado na direcao do gyro.
    if (x < 0) {
      x_mouse *= -1;
    }
    if (y < 0) {
      y_mouse *= -1;
    }
    x_mouse = floor(x_mouse*XMOUSE_SCALE);
    y_mouse = floor(y_mouse*YMOUSE_SCALE);
    
    if(mouseIsActive){
      //Serial.println("Mouse ativo");
      Mouse.move(int(-x_mouse), int(y_mouse), 0);
    }
  }
  delay(10);
}

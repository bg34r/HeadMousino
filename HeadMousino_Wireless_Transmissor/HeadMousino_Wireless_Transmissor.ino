/* 
 * HeadMousino Transmissor 
 * Codigo para trasmissao dos valores lidos no sensores (acel.,  giro,  piezo)
 * e transmissao.
 * 
 * Baseado nos codigos:
 * https://github.com/jrowberg/i2cdevlib 
 * para aquisicao dos dados (acelerometro  e  giroscopio) da MPU6050
 * 
 * https://github.com/tmrh20/RF24/
 * exemplo de comunicacao dos radios RF24
 * 
 * 
 * by Bruno Araujo, https://github.com/bg34r/HeadMousino
 */

#include <Wire.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include <MPU6050.h>
//#include <Mouse.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ================================================================
// ===                         MPU6050                          ===
// ================================================================

MPU6050 mpu;
#define INTERRUPT_PIN 7
int16_t ax, ay, az, gx, gy, gz;
// MPU controle/status variaveis
bool dmpReady = false;  // set true se a DMP foi inicializada corretamente
uint8_t mpuIntStatus;   // segura o byte  de  status atual da MPU
uint8_t devStatus;      // retorna status  apos cada operacao do disposivo (0 = successo, !0 = error)
uint16_t packetSize;    // tamanho esperado do pacote DMP (default e 42 bytes)
uint16_t fifoCount;     // contagem de todos os bytes atualmente na fila FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int MPUOffsets[6] = {  -1836,  -3755,  3327,  43,  4,  94}; //Run MPU6050_Calibration to obtain these

volatile bool mpuInterrupt = false;     // indica quando o pino  de interrupcao da MPU foi nivel alto/high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                          PIEZO                           ===
// ================================================================
// constantes que  nao  mudam
const int knockSensor = A0; // o sensor  esta conectos ao analog. 0  (A0)
// variaveis  que irao  mudar
int sensorReading = 0;      // variable to store the value read from the sensor pin

// ================================================================
// ===                           RADIO                          ===
// ================================================================
RF24 radio(6, 8); // CE, CSN Default: 7,8(Mudamos para o pino 6, já que o 7 já está sendo usado em nosso projeto, como interrupção no nó transmissor)
const byte address[6] = "Mouse";

void setup() {
    // conectar ao bus  I2C(I2Cdev biblioteca nao  faz automaticamente)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // As informações abaixo sao para serial, a qual nao usamos, mas ficam como referencia 
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // inicializa MPU6050
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // usado para verificar conexao com o MPU6050
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // valores de offset da nossa mpu, podem ser obtidos atraves do codigo: MPU6050_calibration
    mpu.setXAccelOffset(MPUOffsets[0]);
    mpu.setYAccelOffset(MPUOffsets[1]);
    mpu.setZAccelOffset(MPUOffsets[2]);
    mpu.setXGyroOffset(MPUOffsets[3]);
    mpu.setYGyroOffset(MPUOffsets[4]);
    mpu.setZGyroOffset(MPUOffsets[5]);
    
    // testando a conexao (returns 0 se sim)
    if (devStatus == 0) {
        // liga a DMP, agora que temos  uma conexao
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Habilitar interrupcao do Arduino ao pino de inturrupcao do MPU6050
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
    radio.begin();//iniciar radio
    radio.openWritingPipe(address);//endereco em que nossos dispositivos se comunicam
    radio.setPALevel(RF24_PA_MIN); //define que nossa comunicacao se da no menor nivel de alcance 
    radio.stopListening();  //como somente transmistimos, paramos de escutar
}

void loop() {
  // falhando a comunicacao com o dispositivo i2c, nao fazer nada
  if (!dmpReady) return;
  //com a comunicacao estabelecida, podemos fazer a leitura
  sensorReading = analogRead(knockSensor);  //leitura do sensor piezo
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //leitura de valores crus da MPU6050
  int16_t payload[7] = {ax, ay, az, gx, gy,  gz, sensorReading}; //pacote que sera  transmitido
  radio.write(&payload, sizeof(payload));   //pacote transmitido pela nrf24l01
}

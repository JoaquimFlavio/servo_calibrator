#include <Servo.h>
#include <math.h>

// ======================================================
// SERVOS
// ======================================================

const uint8_t LEFT_SERVO_PIN = 6;
const uint8_t RIGHT_SERVO_PIN = 5;

Servo leftServo;
Servo rightServo;

// Valores já calibrados anteriormente
const int LEFT_STOP = 1450;
const int RIGHT_STOP = 1450;

// Sentidos físicos dos servos
const int LEFT_DIRECTION = 1;
const int RIGHT_DIRECTION = -1;

const int MAX_SERVO_SPEED = 300;


// ======================================================
// ENCODERS
// ======================================================
//
// Arduino UNO:
// D2 = INT0
// D3 = INT1
//

const uint8_t LEFT_ENCODER_PIN = 2;
const uint8_t RIGHT_ENCODER_PIN = 3;

volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;


// ======================================================
// PULSOS POR VOLTA
// ======================================================
//
// Ajuste conforme seus encoders.
//
// Se os dois forem iguais, use o mesmo valor.
//
// IMPORTANTE:
// Considere apenas os pulsos contados na borda RISING,
// pois é assim que este código trabalha.
//

const float LEFT_ENCODER_PPR = 20.0;
const float RIGHT_ENCODER_PPR = 20.0;


// ======================================================
// GANHOS
// ======================================================

float leftGainPositive = 1.0;
float rightGainPositive = 1.0;

float leftGainNegative = 1.0;
float rightGainNegative = 1.0;


// ======================================================
// CONFIGURAÇÃO DA CALIBRAÇÃO
// ======================================================

// Velocidade lógica utilizada no teste (max 300)
const int CALIBRATION_SPEED = 300;

// Tempo para o motor atingir velocidade estável
const unsigned long SETTLE_TIME_MS = 500;

// Tempo efetivo de medição
const unsigned long MEASURE_TIME_MS = 3000;

// Pausa entre medições
const unsigned long PAUSE_TIME_MS = 1000/4;

// Quantidade máxima de tentativas ajustando um único motor
const int MAX_ITERATIONS_PER_MOTOR = 50;

// A calibração só é aceita com erro estritamente abaixo de 5%
const float TOLERANCE = 0.05;

// Se após 10 tentativas o erro continuar acima de 50%,
// troca o motor de referência e passa a ajustar o outro.
const float SWITCH_MOTOR_ERROR = 0.50;

// Quanto corrigir a cada tentativa.
//
// 1.0 = correção total
// 0.5 = correção mais suave
//
// Recomendo 0.5~0.7 para evitar oscilações.

const float ADJUST_STRENGTH = 0.15;

// Limites de segurança para os ganhos
const float MIN_GAIN = 0.50;
const float MAX_GAIN = 1.50;


// ======================================================
// INTERRUPÇÕES DOS ENCODERS
// ======================================================

void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}


// ======================================================
// ZERA ENCODERS
// ======================================================

void resetEncoders() {
  noInterrupts();

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  interrupts();
}


// ======================================================
// OBTÉM CONTAGEM
// ======================================================

void getEncoderCounts(
  unsigned long &left,
  unsigned long &right) {
  noInterrupts();

  left = leftEncoderCount;
  right = rightEncoderCount;

  interrupts();
}


// ======================================================
// PARA OS MOTORES
// ======================================================

void stopMotors() {
  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);
}


// ======================================================
// CONTROLE DOS MOTORES
// ======================================================

void setMotorSpeed(
  int leftSpeed,
  int rightSpeed,
  float leftGain,
  float rightGain) {
  int adjustedLeft =
    leftSpeed * leftGain;

  int adjustedRight =
    rightSpeed * rightGain;

  adjustedLeft = constrain(
    adjustedLeft,
    -MAX_SERVO_SPEED,
    MAX_SERVO_SPEED);

  adjustedRight = constrain(
    adjustedRight,
    -MAX_SERVO_SPEED,
    MAX_SERVO_SPEED);

  int leftPulse =
    LEFT_STOP + LEFT_DIRECTION * adjustedLeft;

  int rightPulse =
    RIGHT_STOP + RIGHT_DIRECTION * adjustedRight;

  leftServo.writeMicroseconds(leftPulse);
  rightServo.writeMicroseconds(rightPulse);
}


// ======================================================
// EXECUTA UMA MEDIÇÃO
// ======================================================

bool measureMotors(
  int speed,
  float leftGain,
  float rightGain,
  float &leftTurns,
  float &rightTurns) {
  // Começa a movimentação
  setMotorSpeed(
    speed,
    speed,
    leftGain,
    rightGain);

  // Aguarda estabilizar
  delay(SETTLE_TIME_MS);

  // Só começamos a contar depois da estabilização
  resetEncoders();

  delay(MEASURE_TIME_MS);

  unsigned long leftCount;
  unsigned long rightCount;

  getEncoderCounts(
    leftCount,
    rightCount);

  stopMotors();

  leftTurns =
    leftCount / LEFT_ENCODER_PPR;

  rightTurns =
    rightCount / RIGHT_ENCODER_PPR;


  Serial.print("Pulsos L: ");
  Serial.print(leftCount);

  Serial.print(" | Pulsos R: ");
  Serial.print(rightCount);

  Serial.print(" | Voltas L: ");
  Serial.print(leftTurns, 3);

  Serial.print(" | Voltas R: ");
  Serial.println(rightTurns, 3);


  // Detecta encoder sem leitura

  if (
    leftCount == 0 || rightCount == 0) {
    Serial.println(
      "ERRO: um dos encoders nao registrou pulsos.");

    return false;
  }

  return true;
}


// ======================================================
// CALCULA ERRO ENTRE OS MOTORES
// ======================================================

float calculateError(float leftTurns, float rightTurns) {
  float maximum = max(leftTurns, rightTurns);

  if (maximum <= 0.0) {
    return 1.0;
  }

  return abs(leftTurns - rightTurns) / maximum;
}


// ======================================================
// AJUSTA APENAS UM DOS MOTORES
// ======================================================

bool calibrateSingleMotor(
  int speed,
  bool adjustLeft,
  float &leftGain,
  float &rightGain,
  float &lastError) {
  if (adjustLeft) {
    rightGain = 1.0;
  } else {
    leftGain = 1.0;
  }

  for (
    int iteration = 1;
    iteration <= MAX_ITERATIONS_PER_MOTOR;
    iteration++) {
    Serial.println();
    Serial.print("Tentativa ");
    Serial.print(iteration);
    Serial.print("/");
    Serial.println(MAX_ITERATIONS_PER_MOTOR);

    Serial.print("Motor sendo ajustado: ");
    Serial.println(adjustLeft ? "ESQUERDO" : "DIREITO");

    Serial.print("LEFT_GAIN  = ");
    Serial.println(leftGain, 4);

    Serial.print("RIGHT_GAIN = ");
    Serial.println(rightGain, 4);

    float leftTurns;
    float rightTurns;

    bool success =
      measureMotors(
        speed,
        leftGain,
        rightGain,
        leftTurns,
        rightTurns);

    if (!success) {
      stopMotors();
      lastError = 1.0;
      return false;
    }

    lastError =
      calculateError(
        leftTurns,
        rightTurns);

    Serial.print("Erro: ");
    Serial.print(lastError * 100.0, 2);
    Serial.println("%");

    if (lastError < TOLERANCE) {
      Serial.println();
      Serial.println("*** CALIBRACAO CONCLUIDA (< 5%) ***");
      return true;
    }

    if (adjustLeft) {
      float ratio =
        rightTurns / leftTurns;

      float correction =
        pow(
          ratio,
          ADJUST_STRENGTH);

      leftGain *= correction;

      leftGain = constrain(
        leftGain,
        MIN_GAIN,
        MAX_GAIN);

      rightGain = 1.0;
    } else {
      float ratio =
        leftTurns / rightTurns;

      float correction =
        pow(
          ratio,
          ADJUST_STRENGTH);

      rightGain *= correction;

      rightGain = constrain(
        rightGain,
        MIN_GAIN,
        MAX_GAIN);

      leftGain = 1.0;
    }

    delay(PAUSE_TIME_MS);
  }

  return false;
}


// ======================================================
// CALIBRA UMA DIREÇÃO
// ======================================================

void calibrateDirection(
  int speed,
  float &leftGain,
  float &rightGain) {
  leftGain = 1.0;
  rightGain = 1.0;

  Serial.println();
  Serial.println("Medicao inicial para escolher o motor a ajustar...");

  float leftTurns;
  float rightTurns;

  bool success =
    measureMotors(
      speed,
      leftGain,
      rightGain,
      leftTurns,
      rightTurns);

  if (!success) {
    stopMotors();
    return;
  }

  float initialError =
    calculateError(
      leftTurns,
      rightTurns);

  Serial.print("Erro inicial: ");
  Serial.print(initialError * 100.0, 2);
  Serial.println("%");

  if (initialError < TOLERANCE) {
    Serial.println("*** JA ESTA CALIBRADO (< 5%) ***");
    return;
  }

  // Ajusta primeiro o motor mais rápido e mantém o outro em ganho 1.0.
  bool adjustLeft =
    leftTurns > rightTurns;

  Serial.print("Primeiro motor a ajustar: ");
  Serial.println(adjustLeft ? "ESQUERDO" : "DIREITO");

  float lastError = initialError;

  bool calibrated =
    calibrateSingleMotor(
      speed,
      adjustLeft,
      leftGain,
      rightGain,
      lastError);

  if (calibrated) {
    return;
  }

  Serial.println();
  Serial.print("Erro apos 10 tentativas: ");
  Serial.print(lastError * 100.0, 2);
  Serial.println("%");

  if (lastError > SWITCH_MOTOR_ERROR) {
    Serial.println();
    Serial.println("Erro ainda acima de 50%.");
    Serial.println(
      "Trocando o motor de referencia e ajustando o outro.");

    leftGain = 1.0;
    rightGain = 1.0;

    adjustLeft = !adjustLeft;

    calibrated =
      calibrateSingleMotor(
        speed,
        adjustLeft,
        leftGain,
        rightGain,
        lastError);

    if (calibrated) {
      return;
    }
  }

  Serial.println();
  Serial.println("*** CALIBRACAO NAO CONVERGIU ***");
  Serial.print("Erro final: ");
  Serial.print(lastError * 100.0, 2);
  Serial.println("%");
}


// ======================================================
// EXIBE RESULTADO
// ======================================================

void printResults() {
  Serial.println();
  Serial.println();
  Serial.println("==============================");
  Serial.println(" RESULTADO FINAL");
  Serial.println("==============================");
  Serial.println();

  Serial.print(
    "const float LEFT_GAIN_POSITIVE = ");
  Serial.print(leftGainPositive, 4);
  Serial.println(";");

  Serial.print(
    "const float RIGHT_GAIN_POSITIVE = ");
  Serial.print(rightGainPositive, 4);
  Serial.println(";");

  Serial.println();

  Serial.print(
    "const float LEFT_GAIN_NEGATIVE = ");
  Serial.print(leftGainNegative, 4);
  Serial.println(";");

  Serial.print(
    "const float RIGHT_GAIN_NEGATIVE = ");
  Serial.print(rightGainNegative, 4);
  Serial.println(";");

  Serial.println();
}


// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);

  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  

  // stopMotors();
  // while(1);

  pinMode(
    LEFT_ENCODER_PIN,
    INPUT_PULLUP);

  pinMode(
    RIGHT_ENCODER_PIN,
    INPUT_PULLUP);


  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_PIN),
    leftEncoderISR,
    RISING);

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_PIN),
    rightEncoderISR,
    RISING);


  stopMotors();

  delay(2000);


  // ====================================================
  // POSITIVO
  // ====================================================

  Serial.println();
  Serial.println("==============================");
  Serial.println(" CALIBRANDO VELOCIDADE POSITIVA");
  Serial.println("==============================");

  calibrateDirection(
    CALIBRATION_SPEED,
    leftGainPositive,
    rightGainPositive);


  delay(2000);


  // ====================================================
  // NEGATIVO
  // ====================================================

  Serial.println();
  Serial.println("==============================");
  Serial.println(" CALIBRANDO VELOCIDADE NEGATIVA");
  Serial.println("==============================");

  calibrateDirection(
    -CALIBRATION_SPEED,
    leftGainNegative,
    rightGainNegative);


  stopMotors();

  printResults();
}


// ======================================================
// LOOP
// ======================================================

void loop() {
  // Calibração executada apenas uma vez no setup().
}
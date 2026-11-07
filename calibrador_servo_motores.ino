#include <Servo.h>
#include <math.h>

// Única configuração específica da montagem (UNO: encoders em D2/D3).
const uint8_t LEFT_SERVO_PIN = 6;
const uint8_t RIGHT_SERVO_PIN = 5;
const uint8_t LEFT_ENCODER_PIN = 2;
const uint8_t RIGHT_ENCODER_PIN = 3;

// Limites de exploração: não é possível descobrir limites mecânicos por encoder.
const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;
const int SCAN_STEP_US = 25;
const unsigned long SETTLE_MS = 500;
const unsigned long MEASURE_MS = 1000;
const float TOLERANCE = 0.05;
const int SAMPLE_COUNT = (PULSE_MAX_US - PULSE_MIN_US) / SCAN_STEP_US + 1;

Servo servos[2];
volatile unsigned long encoderCounts[2] = {0, 0};

struct DirectionParameters {
  int minimumUs;
  int maximumUs;
  unsigned long maximumPulses;
};

struct MotorParameters {
  int stopUs;
  int stopLowUs;
  int stopHighUs;
  DirectionParameters direction[2]; // 0: pulso menor; 1: pulso maior que parada
};

struct CalibrationResult {
  bool valid;
  int pulseUs[2];
  float error;
};

MotorParameters motors[2];
CalibrationResult results[2];
// Buffer reutilizado para não ocupar a pilha do UNO.
unsigned long samples[2][SAMPLE_COUNT];

void leftEncoderISR() { encoderCounts[0]++; }
void rightEncoderISR() { encoderCounts[1]++; }

void resetEncoders() {
  noInterrupts();
  encoderCounts[0] = encoderCounts[1] = 0;
  interrupts();
}

void readEncoders(unsigned long &left, unsigned long &right) {
  noInterrupts();
  left = encoderCounts[0];
  right = encoderCounts[1];
  interrupts();
}

void stopMotors() {
  for (int m = 0; m < 2; m++) servos[m].writeMicroseconds(motors[m].stopUs);
}

void halt(const __FlashStringHelper *reason) {
  // Remove o sinal mesmo se a parada ainda não foi descoberta.
  servos[0].detach();
  servos[1].detach();
  Serial.print(F("FALHA: "));
  Serial.println(reason);
  Serial.println(F("Execucao interrompida. Corrija a montagem e reinicie."));
  while (true) delay(1000);
}

void measure(unsigned long &left, unsigned long &right) {
  delay(SETTLE_MS);
  resetEncoders();
  delay(MEASURE_MS);
  readEncoders(left, right);
}

void printMeasurement(int motor, int pulse, unsigned long left, unsigned long right) {
  Serial.print(F("motor=")); Serial.print(motor == 0 ? F("L") : F("R"));
  Serial.print(F(" pulso_us=")); Serial.print(pulse);
  Serial.print(F(" pulsos_L=")); Serial.print(left);
  Serial.print(F(" pulsos_R=")); Serial.println(right);
}

// Varredura simultânea, com detecção independente da faixa de parada.
void discoverStops() {
  int runStart[2] = {-1, -1};
  int bestStart[2] = {-1, -1};
  int bestEnd[2] = {-1, -1};
  int bestLength[2] = {0, 0};
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int pulse = PULSE_MIN_US + i * SCAN_STEP_US;
    for (int m = 0; m < 2; m++) servos[m].writeMicroseconds(pulse);
    unsigned long counts[2];
    measure(counts[0], counts[1]);
    for (int m = 0; m < 2; m++) {
      printMeasurement(m, pulse, counts[0], counts[1]);
      if (counts[m] == 0) {
        if (runStart[m] < 0) runStart[m] = i;
        int length = i - runStart[m] + 1;
        if (length > bestLength[m]) {
          bestLength[m] = length;
          bestStart[m] = runStart[m];
          bestEnd[m] = i;
        }
      } else {
        runStart[m] = -1;
      }
    }
  }
  for (int m = 0; m < 2; m++) {
    if (bestStart[m] <= 0 || bestEnd[m] >= SAMPLE_COUNT - 1) {
      halt(F("parada nao identificada: confira encoder, alimentacao e faixa de pulsos."));
    }
    motors[m].stopLowUs = PULSE_MIN_US + bestStart[m] * SCAN_STEP_US;
    motors[m].stopHighUs = PULSE_MIN_US + bestEnd[m] * SCAN_STEP_US;
    motors[m].stopUs = (motors[m].stopLowUs + motors[m].stopHighUs) / 2;
  }
  stopMotors();
}

void attachMotor(int motor, int pulse) {
  // Precarrega o comando antes de habilitar os pulsos.
  servos[motor].writeMicroseconds(pulse);
  servos[motor].attach(motor == 0 ? LEFT_SERVO_PIN : RIGHT_SERVO_PIN);
}

void setupHardware() {
  Serial.println(F("ETAPA 1: CORRESPONDENCIA MOTORES / ENCODERS"));
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

  // Somente o motor sob teste recebe sinal. Nenhuma parada é buscada aqui.
  for (int m = 0; m < 2; m++) {
    attachMotor(m, PULSE_MAX_US);
    unsigned long left, right;
    measure(left, right);
    printMeasurement(m, PULSE_MAX_US, left, right);
    servos[m].detach();
    unsigned long wanted = m == 0 ? left : right;
    unsigned long other = m == 0 ? right : left;
    if (!wanted || other) halt(F("motor/encoder sem correspondencia ou motor inativo girando."));
    Serial.println(F("Correspondencia motor/encoder confirmada."));
    delay(SETTLE_MS);
  }
}

int pulseAt(int motor, int direction, int offset) {
  return motors[motor].stopUs + (direction == 0 ? -offset : offset);
}

void discoverDirections(int direction) {
  stopMotors();
  int sampleCount[2];
  int firstOffset[2];
  int boundary = direction == 0 ? PULSE_MIN_US : PULSE_MAX_US;
  int steps = 0;
  for (int m = 0; m < 2; m++) {
    int limit = abs(boundary - motors[m].stopUs);
    sampleCount[m] = (limit - 1) / SCAN_STEP_US + 1;
    firstOffset[m] = limit - (sampleCount[m] - 1) * SCAN_STEP_US;
    steps = max(steps, sampleCount[m]);
    motors[m].direction[direction].maximumPulses = 0;
    motors[m].direction[direction].maximumUs = boundary;
  }

  // Uma janela para ambos. Cada array é ordenado pelo desvio da sua parada.
  // Se um motor terminar antes, fica parado enquanto o outro conclui.
  for (int step = 0; step < steps; step++) {
    int pulses[2];
    for (int m = 0; m < 2; m++) {
      int i = sampleCount[m] - 1 - step;
      pulses[m] = i >= 0
        ? pulseAt(m, direction, firstOffset[m] + i * SCAN_STEP_US)
        : motors[m].stopUs;
      servos[m].writeMicroseconds(pulses[m]);
    }
    unsigned long counts[2];
    measure(counts[0], counts[1]);
    for (int m = 0; m < 2; m++) {
      int i = sampleCount[m] - 1 - step;
      if (i < 0) {
        if (counts[m]) halt(F("motor concluido recebeu pulsos no comando de parada."));
        continue;
      }
      printMeasurement(m, pulses[m], counts[0], counts[1]);
      samples[m][i] = counts[m];
      DirectionParameters &p = motors[m].direction[direction];
      if (counts[m] > p.maximumPulses) {
        p.maximumPulses = counts[m];
        p.maximumUs = pulses[m];
      }
    }
  }
  stopMotors();

  for (int m = 0; m < 2; m++) {
    DirectionParameters &p = motors[m].direction[direction];
    if (!p.maximumPulses) halt(F("nenhum movimento neste sentido."));
    // O mínimo delimita a busca: exige duas amostras vizinhas com pulsos.
    p.minimumUs = p.maximumUs;
    bool minimumFound = false;
    int maximumOffset = abs(p.maximumUs - motors[m].stopUs);
    for (int i = 0; i + 1 < sampleCount[m]; i++) {
      int offset = firstOffset[m] + i * SCAN_STEP_US;
      if (offset > maximumOffset) break;
      if (samples[m][i] > 0 && samples[m][i + 1] > 0) {
        p.minimumUs = pulseAt(m, direction, offset);
        minimumFound = true;
        break;
      }
    }
    if (!minimumFound) {
      Serial.println(F("AVISO: sem amostras vizinhas com movimento; limite inferior igual ao maximo medido."));
    }
    Serial.print(F("Motor=")); Serial.print(m == 0 ? F("L") : F("R"));
    Serial.print(F(" limite inferior da calibracao (us): "));
    Serial.println(p.minimumUs);
  }
}

void discoverParameters() {
  Serial.println(F("ETAPA 2: PARAMETROS DOS MOTORES / MEDICAO SIMULTANEA"));
  for (int m = 0; m < 2; m++) attachMotor(m, PULSE_MIN_US);
  discoverStops();
  unsigned long left, right;
  measure(left, right);
  if (left || right) halt(F("ha rotacao com ambos os comandos de parada."));
  for (int d = 0; d < 2; d++) {
    Serial.print(F("Ambos os motores: sentido_pulso="));
    Serial.println(d == 0 ? F("negativo") : F("positivo"));
    discoverDirections(d);
  }
}

// Comparação de pulsos pressupõe encoders com a mesma resolução.
float relativeError(unsigned long left, unsigned long right) {
  unsigned long largest = max(left, right);
  return largest ? (float)(left > right ? left - right : right - left) / largest : 1.0;
}

void measurePair(const int pulses[2], unsigned long &left, unsigned long &right) {
  servos[0].writeMicroseconds(pulses[0]);
  servos[1].writeMicroseconds(pulses[1]);
  measure(left, right);
  stopMotors();
}

void calibrateDirection(int direction) {
  // Montagem espelhada: pulsos opostos produzem o mesmo sentido lógico.
  int dirs[2] = {direction, 1 - direction};
  CalibrationResult &result = results[direction];
  int pulses[2];
  for (int m = 0; m < 2; m++) {
    pulses[m] = motors[m].direction[dirs[m]].maximumUs;
    result.pulseUs[m] = pulses[m];
  }
  unsigned long left, right;
  measurePair(pulses, left, right);
  if (!left || !right) halt(F("sem pulsos na medicao inicial da calibracao."));
  result.error = relativeError(left, right);
  result.valid = result.error < TOLERANCE;
  if (result.valid) return;

  int adjusted = left > right ? 0 : 1;
  DirectionParameters &p = motors[adjusted].direction[dirs[adjusted]];
  int low = abs(p.minimumUs - motors[adjusted].stopUs);
  int high = abs(p.maximumUs - motors[adjusted].stopUs);
  while (low <= high) {
    int mid = low + (high - low) / 2;
    pulses[adjusted] = pulseAt(adjusted, dirs[adjusted], mid);
    measurePair(pulses, left, right);
    unsigned long target = adjusted == 0 ? right : left;
    unsigned long actual = adjusted == 0 ? left : right;
    if (!target) halt(F("referencia sem pulsos durante calibracao."));
    float error = relativeError(left, right);
    Serial.print(F("CAL sentido=")); Serial.print(direction);
    Serial.print(F(" pulso_L=")); Serial.print(pulses[0]);
    Serial.print(F(" pulso_R=")); Serial.print(pulses[1]);
    Serial.print(F(" pulsos_L=")); Serial.print(left);
    Serial.print(F(" pulsos_R=")); Serial.print(right);
    Serial.print(F(" erro_pct=")); Serial.println(error * 100.0, 2);
    if (error < result.error) {
      result.error = error;
      result.pulseUs[0] = pulses[0];
      result.pulseUs[1] = pulses[1];
    }
    if (error < TOLERANCE) { result.valid = true; return; }
    if (actual > target) high = mid - 1;
    else low = mid + 1;
  }
  Serial.println(F("AVISO: tolerancia nao atingida; melhor medicao mantida."));
}

// Trecho copiável, como no log original do repositório.
void printCodeResults() {
  Serial.println(F("\n// ===== CODIGO DOS PARAMETROS E DA CALIBRACAO ====="));
  Serial.println(F("// POSITIVE: L aumenta pulso, R diminui; NEGATIVE: inverso."));
  Serial.println(F("// Ganhos multiplicam o desvio entre MAX e STOP de cada sentido."));
  Serial.println(F("// MIN, MAX e CALIBRATED sao pulsos absolutos em microssegundos."));
  for (int m = 0; m < 2; m++) {
    Serial.print(F("const int "));
    Serial.print(m == 0 ? F("LEFT") : F("RIGHT"));
    Serial.print(F("_STOP = ")); Serial.print(motors[m].stopUs);
    Serial.println(';');
  }
  for (int d = 1; d >= 0; d--) {
    Serial.print(F("const bool CALIBRATION_VALID_"));
    Serial.print(d == 1 ? F("POSITIVE") : F("NEGATIVE"));
    Serial.print(F(" = "));
    Serial.print(results[d].valid ? F("true") : F("false"));
    Serial.println(';');
    if (!results[d].valid) Serial.println(F("// ATENCAO: melhor ajuste abaixo nao atingiu a tolerancia."));
    for (int m = 0; m < 2; m++) {
      DirectionParameters &p = motors[m].direction[m == 0 ? d : 1 - d];
      for (int field = 0; field < 4; field++) {
        Serial.print(field == 3 ? F("const float ") : F("const int "));
        Serial.print(m == 0 ? F("LEFT_") : F("RIGHT_"));
        switch (field) {
          case 0: Serial.print(F("MIN_")); break;
          case 1: Serial.print(F("MAX_")); break;
          case 2: Serial.print(F("CALIBRATED_")); break;
          case 3: Serial.print(F("GAIN_")); break;
        }
        Serial.print(d == 1 ? F("POSITIVE") : F("NEGATIVE"));
        Serial.print(F(" = "));
        if (field == 0) Serial.print(p.minimumUs);
        else if (field == 1) Serial.print(p.maximumUs);
        else if (field == 2) Serial.print(results[d].pulseUs[m]);
        else Serial.print((float)abs(results[d].pulseUs[m] - motors[m].stopUs) /
                              abs(p.maximumUs - motors[m].stopUs), 4);
        Serial.println(';');
      }
    }
  }
  Serial.println(F("// ===== FIM DO CODIGO =====\n"));
}

void printResults() {
  Serial.println(F("ETAPA 4: RESULTADOS"));
  Serial.print(F("Unidades: comandos em us; velocidade em pulsos por janela de "));
  Serial.print(MEASURE_MS); Serial.println(F(" ms."));
  Serial.println(F("MOTOR,SENTIDO_PULSO,PARADA_US,MIN_US,MAX_US,PULSOS_MAX"));
  for (int m = 0; m < 2; m++) {
    for (int d = 0; d < 2; d++) {
      DirectionParameters &p = motors[m].direction[d];
      Serial.print(m == 0 ? F("L,") : F("R,"));
      Serial.print(d == 0 ? F("-,") : F("+,"));
      Serial.print(motors[m].stopUs); Serial.print(',');
      Serial.print(p.minimumUs); Serial.print(',');
      Serial.print(p.maximumUs); Serial.print(',');
      Serial.println(p.maximumPulses);
    }
  }
  Serial.println(F("CAL_SENTIDO,VALIDO,PULSO_L_US,PULSO_R_US,GANHO_L,GANHO_R,ERRO_PCT"));
  for (int d = 0; d < 2; d++) {
    CalibrationResult &r = results[d];
    Serial.print(d); Serial.print(','); Serial.print(r.valid); Serial.print(',');
    for (int m = 0; m < 2; m++) { Serial.print(r.pulseUs[m]); Serial.print(','); }
    for (int m = 0; m < 2; m++) {
      int motorDirection = m == 0 ? d : 1 - d;
      float gain = (float)abs(r.pulseUs[m] - motors[m].stopUs) /
        abs(motors[m].direction[motorDirection].maximumUs - motors[m].stopUs);
      Serial.print(gain, 4); Serial.print(',');
    }
    Serial.println(r.error * 100.0, 2);
  }
  printCodeResults();
  Serial.println(F("Plot de validacao a seguir. Envie p para repetir o plot."));
}

// Plot de validação: duas séries com pulsos medidos após a calibração.
// Executa ao final do setup; envie 'p' para repetir. Termina com ambos parados.
void plotResults() {
  for (int d = 0; d < 2; d++) {
    if (!results[d].valid) continue;
    servos[0].writeMicroseconds(results[d].pulseUs[0]);
    servos[1].writeMicroseconds(results[d].pulseUs[1]);
    delay(SETTLE_MS);
    for (int i = 0; i < 10; i++) {
      resetEncoders();
      delay(MEASURE_MS);
      unsigned long left, right;
      readEncoders(left, right);
      Serial.print(F("L:")); Serial.print(left);
      Serial.print(F("\tR:")); Serial.println(right);
    }
    stopMotors();
    delay(SETTLE_MS);
  }
}

// Teste independente: mantém os comandos fixos até reiniciar a placa.
// Inicializa os encoders sem executar as etapas de descoberta/calibração.
void debugMotorPulses(int leftPulseUs, int rightPulseUs) {
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);
  attachMotor(0, leftPulseUs);
  attachMotor(1, rightPulseUs);

  Serial.println(F("DEBUG: comandos fixos; reinicie a placa para encerrar."));
  Serial.print(F("pulso_L_us=")); Serial.print(leftPulseUs);
  Serial.print(F(" pulso_R_us=")); Serial.print(rightPulseUs);
  Serial.print(F(" janela_ms=")); Serial.println(MEASURE_MS);
  delay(SETTLE_MS);
  while (true) {
    resetEncoders();
    delay(MEASURE_MS);
    unsigned long left, right;
    readEncoders(left, right);
    Serial.print(F("pulsos_L=")); Serial.print(left);
    Serial.print(F(" pulsos_R=")); Serial.println(right);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  // Descomente para testar a configuração da falha; o helper não retorna.
  // Comente novamente para executar o fluxo de calibração abaixo.
  // debugMotorPulses(1000, 1550);

  setupHardware();
  discoverParameters();
  Serial.println(F("ETAPA 3: CALIBRACAO POR BUSCA BINARIA"));
  calibrateDirection(0);
  calibrateDirection(1);
  stopMotors();
  printResults();
  plotResults();
}

void loop() {
  if (Serial.available() && Serial.read() == 'p') plotResults();
}

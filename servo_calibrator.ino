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

// Colunas de largura fixa: legíveis mesmo em monitores sem tab stops.
void logSection(const __FlashStringHelper *title) {
  Serial.println(F("\n======================================================================"));
  Serial.println(title);
  Serial.println(F("======================================================================"));
}

void logText(const __FlashStringHelper *value) {
  Serial.print(F("| "));
  size_t length = Serial.print(value);
  while (length++ < 12) Serial.print(' ');
  Serial.print(' ');
}

void logNumber(unsigned long value) {
  char buffer[11];
  ultoa(value, buffer, 10);
  Serial.print(F("| "));
  for (size_t i = strlen(buffer); i < 12; i++) Serial.print(' ');
  Serial.print(buffer);
  Serial.print(' ');
}

void logFloat(float value, uint8_t decimals) {
  char buffer[24];
  dtostrf(value, 12, decimals, buffer);
  Serial.print(F("| ")); Serial.print(buffer); Serial.print(' ');
}

void logMeasurementHeader() {
  logText(F("L cmd [us]")); logText(F("R cmd [us]"));
  logText(F("L pulsos")); logText(F("R pulsos")); Serial.println('|');
  Serial.println(F("+--------------+--------------+--------------+--------------+"));
}

void logMeasurementCells(int leftPulse, int rightPulse,
                         unsigned long left, unsigned long right) {
  if (leftPulse >= 0) logNumber(leftPulse); else logText(F("sem sinal"));
  if (rightPulse >= 0) logNumber(rightPulse); else logText(F("sem sinal"));
  logNumber(left); logNumber(right);
}

void logMeasurement(int leftPulse, int rightPulse,
                    unsigned long left, unsigned long right) {
  logMeasurementCells(leftPulse, rightPulse, left, right);
  Serial.println('|');
}

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
  logSection(F("[FALHA] EXECUCAO INTERROMPIDA"));
  Serial.print(F("Motivo:\t"));
  Serial.println(reason);
  Serial.println(F("Revise os dados acima e reinicie a placa."));
  while (true) delay(1000);
}

void measure(unsigned long &left, unsigned long &right) {
  delay(SETTLE_MS);
  resetEncoders();
  delay(MEASURE_MS);
  readEncoders(left, right);
}

// Varredura simultânea, com detecção independente da faixa de parada.
void discoverStops() {
  Serial.println(F("\n> Busca dos pontos de parada"));
  logMeasurementHeader();
  int runStart[2] = {-1, -1};
  int bestStart[2] = {-1, -1};
  int bestEnd[2] = {-1, -1};
  int bestLength[2] = {0, 0};
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int pulse = PULSE_MIN_US + i * SCAN_STEP_US;
    for (int m = 0; m < 2; m++) servos[m].writeMicroseconds(pulse);
    unsigned long counts[2];
    measure(counts[0], counts[1]);
    logMeasurement(pulse, pulse, counts[0], counts[1]);
    for (int m = 0; m < 2; m++) {
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
  logSection(F("ETAPA 1 / HARDWARE - CORRESPONDENCIA DOS ENCODERS"));
  Serial.print(F("Janela de medicao:\t")); Serial.print(MEASURE_MS); Serial.println(F(" ms"));
  Serial.println(F("L = esquerdo; R = direito. Apenas um motor recebe sinal por teste."));
  logMeasurementHeader();
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

  // Somente o motor sob teste recebe sinal. Nenhuma parada é buscada aqui.
  for (int m = 0; m < 2; m++) {
    attachMotor(m, PULSE_MAX_US);
    unsigned long left, right;
    measure(left, right);
    logMeasurement(m == 0 ? PULSE_MAX_US : -1, m == 1 ? PULSE_MAX_US : -1, left, right);
    servos[m].detach();
    unsigned long wanted = m == 0 ? left : right;
    unsigned long other = m == 0 ? right : left;
    if (!wanted || other) halt(F("motor/encoder sem correspondencia ou motor inativo girando."));
    Serial.print(F("[OK]\t"));
    Serial.print(m == 0 ? F("Esquerdo") : F("Direito"));
    Serial.println(F(" -> encoder correspondente confirmado."));
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
    logMeasurement(pulses[0], pulses[1], counts[0], counts[1]);
    for (int m = 0; m < 2; m++) {
      int i = sampleCount[m] - 1 - step;
      if (i < 0) {
        if (counts[m]) halt(F("motor concluido recebeu pulsos no comando de parada."));
        continue;
      }
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
      Serial.println(F("[AVISO] Sem amostras vizinhas com movimento; minimo igual ao maximo medido."));
    }
    Serial.print(F("[OK]\tMinimo ")); Serial.print(m == 0 ? F("L") : F("R"));
    Serial.print(F(":\t"));
    Serial.print(p.minimumUs); Serial.println(F(" us"));
  }
}

void discoverParameters() {
  logSection(F("ETAPA 2 / PARAMETROS - MEDICAO SIMULTANEA"));
  for (int m = 0; m < 2; m++) attachMotor(m, PULSE_MIN_US);
  discoverStops();
  unsigned long left, right;
  measure(left, right);
  if (left || right) halt(F("ha rotacao com ambos os comandos de parada."));
  for (int d = 0; d < 2; d++) {
    Serial.print(F("\n> Varredura: pulsos "));
    Serial.println(d == 0 ? F("abaixo da parada (-)") : F("acima da parada (+)"));
    logMeasurementHeader();
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
  Serial.print(F("\n> Sentido logico: "));
  Serial.println(direction == 1 ? F("POSITIVO (L+, R-)") : F("NEGATIVO (L-, R+)"));
  logText(F("L cmd [us]")); logText(F("R cmd [us]"));
  logText(F("L pulsos")); logText(F("R pulsos")); logText(F("Erro [%]")); Serial.println('|');
  Serial.println(F("+--------------+--------------+--------------+--------------+--------------+"));
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
  logMeasurementCells(pulses[0], pulses[1], left, right);
  logFloat(relativeError(left, right) * 100.0, 2); Serial.println('|');
  if (!left || !right) halt(F("sem pulsos na medicao inicial da calibracao."));
  result.error = relativeError(left, right);
  result.valid = result.error < TOLERANCE;
  if (result.valid) { Serial.println(F("[OK]\tMedicao inicial dentro da tolerancia (< 5%).")); return; }

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
    logMeasurementCells(pulses[0], pulses[1], left, right);
    logFloat(error * 100.0, 2); Serial.println('|');
    if (error < result.error) {
      result.error = error;
      result.pulseUs[0] = pulses[0];
      result.pulseUs[1] = pulses[1];
    }
    if (error < TOLERANCE) {
      result.valid = true;
      Serial.println(F("[OK]\tCalibracao concluida (< 5%)."));
      return;
    }
    if (actual > target) high = mid - 1;
    else low = mid + 1;
  }
  Serial.println(F("[AVISO] Tolerancia nao atingida; melhor medicao mantida."));
}

float calibratedGain(int motor, int direction) {
  int motorDirection = motor == 0 ? direction : 1 - direction;
  return (float)abs(results[direction].pulseUs[motor] - motors[motor].stopUs) /
    abs(motors[motor].direction[motorDirection].maximumUs - motors[motor].stopUs);
}

// Somente os pontos de parada e os ganhos, prontos para copiar.
void printCodeResults() {
  Serial.println(F("\n--- CODIGO PARA COPIAR ------------------------------------------------"));
  for (int m = 0; m < 2; m++) {
    Serial.print(m == 0 ? F("const int LEFT_STOP  = ") : F("const int RIGHT_STOP = "));
    Serial.print(motors[m].stopUs); Serial.println(';');
  }
  Serial.println();
  for (int d = 1; d >= 0; d--) {
    for (int m = 0; m < 2; m++) {
      Serial.print(m == 0 ? F("const float LEFT_GAIN_") : F("const float RIGHT_GAIN_"));
      Serial.print(d == 1 ? F("POSITIVE") : F("NEGATIVE"));
      Serial.print(m == 0 ? F("  = ") : F(" = "));
      Serial.print(calibratedGain(m, d), 4); Serial.println(';');
    }
  }
  Serial.println(F("----------------------------------------------------------------------"));
}

void printResults() {
  logSection(F("ETAPA 4 / RESULTADOS"));
  Serial.print(F("Janela:\t")); Serial.print(MEASURE_MS); Serial.println(F(" ms"));
  Serial.println(F("Sentido +/- dos parametros = pulso acima/abaixo da parada."));
  Serial.println(F("\n> Parametros dos motores"));
  logText(F("Motor")); logText(F("Pulso +/-")); logText(F("Parada [us]"));
  logText(F("Min [us]")); logText(F("Max [us]")); logText(F("Pulsos max")); Serial.println('|');
  Serial.println(F("+--------------+--------------+--------------+--------------+--------------+--------------+"));
  for (int m = 0; m < 2; m++) {
    for (int d = 0; d < 2; d++) {
      DirectionParameters &p = motors[m].direction[d];
      logText(m == 0 ? F("Esquerdo") : F("Direito"));
      logText(d == 0 ? F("-") : F("+"));
      logNumber(motors[m].stopUs); logNumber(p.minimumUs);
      logNumber(p.maximumUs); logNumber(p.maximumPulses); Serial.println('|');
    }
  }
  Serial.println(F("\n> Calibracao (sentido logico; motores espelhados)"));
  logText(F("Sentido")); logText(F("Status")); logText(F("L cmd [us]"));
  logText(F("R cmd [us]")); logText(F("Ganho L")); logText(F("Ganho R")); logText(F("Erro [%]")); Serial.println('|');
  Serial.println(F("+--------------+--------------+--------------+--------------+--------------+--------------+--------------+"));
  for (int d = 1; d >= 0; d--) {
    CalibrationResult &r = results[d];
    logText(d == 1 ? F("POSITIVO") : F("NEGATIVO"));
    logText(r.valid ? F("OK") : F("NAO CONVERGIU"));
    logNumber(r.pulseUs[0]); logNumber(r.pulseUs[1]);
    logFloat(calibratedGain(0, d), 4); logFloat(calibratedGain(1, d), 4);
    logFloat(r.error * 100.0, 2); Serial.println('|');
  }
  for (int d = 0; d < 2; d++) {
    if (!results[d].valid) Serial.println(F("[AVISO] Ha ganhos sem convergencia; consulte o status antes de utilizar."));
  }
  Serial.println(F("Ganhos: multiplicadores do desvio entre MAX e PARADA de cada sentido."));
  printCodeResults();
  Serial.println(F("[PLOT] Validacao a seguir: L/R = pulsos por janela. Envie 'p' para repetir."));
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

  logSection(F("DEBUG / COMANDOS FIXOS"));
  Serial.println(F("Reinicie a placa para encerrar. Contagens por janela, sem calibracao."));
  Serial.print(F("Janela:\t")); Serial.print(MEASURE_MS); Serial.println(F(" ms"));
  logMeasurementHeader();
  delay(SETTLE_MS);
  while (true) {
    resetEncoders();
    delay(MEASURE_MS);
    unsigned long left, right;
    readEncoders(left, right);
    logMeasurement(leftPulseUs, rightPulseUs, left, right);
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
  logSection(F("ETAPA 3 / CALIBRACAO POR BUSCA BINARIA"));
  calibrateDirection(0);
  calibrateDirection(1);
  stopMotors();
  printResults();
  plotResults();
}

void loop() {
  if (Serial.available() && Serial.read() == 'p') plotResults();
}

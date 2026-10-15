/* ESP32 Dev Module + L298N. Управление только digitalWrite(), без LEDC/ШИМ.
   IN1: 18,17,16,4; IN2: 26,25,33,32.
   Порядок: передний левый (FL), передний правый (FR), задний левый (RL),
   задний правый (RR). MOTOR_SIGN определяет, какой вход вращает колесо вперёд.

   MOTOR_TEST_ONLY=false, AUTO_START=true: автоматический маршрут через 5 с.
   MOTOR_TEST_ONLY=true: тест каждого входа HIGH 2 с, пауза LOW 700 мс,
   один проход после задержки 3 с. Проверять с вывешенными колёсами.
   AUTO_START=false: ручные команды в мониторе порта 115200, Newline.
   f/b — вперёд/назад; l/r — боком; q/e — поворот влево/вправо;
   1..4, -1..-4 — один мотор; go — маршрут; load — реальная погрузка;
   p/s — PING/STATUS; x — остановка платформы до RESET.

   Вращение: один IN HIGH, второй LOW; стоп: оба LOW. Полная мощность,
   плавного разгона нет. ENA/ENB должны быть постоянно включены (перемычки).
   Не путать с перемычкой питания 5V-EN. Общая GND ESP32/драйверов обязательна.
   Для 4 независимых моторов нужны 4 мостовых канала, обычно 2 модуля L298N.
   На втором канале модуля пара входов может называться IN3/IN4.

   Траектория при старте передом влево на карте:
   1) вперёд 1.78 м;
   2) поворот влево на MAIN_ANGLE_DEG, вперёд 6.25 м, поворот обратно;
   3) влево боком 0.35 м (вниз на карте), остановка, LOAD, ожидание DONE;
   4) вправо боком 0.35 м, поворот вправо на угол финишной диагонали;
   5) вперёд по гипотенузе 2.91 м, поворот обратно, вперёд 0.35 м.
   Проекция финишной диагонали влево = 2.70 - 0.35 = 2.35 м,
   вверх = sqrt(2.91^2 - 2.35^2), угол = atan2(вверх, влево).
   Курс корпуса восстанавливается до въезда под манипулятор и перед финишем.
   Повороты на месте требуют свободного места вокруг корпуса.

   ВСЕ скорости ниже — ЗАГЛУШКИ для полного HIGH, не измерения.
   Не использовать старую калибровку PWM=100. В ручном режиме импульс 0.8 с:
   путь / 0.8 — начальная оценка м/с; угол / 0.8 — начальная оценка град/с.
   Уточнить по целым участкам с остановками; отдельно проверить с кубиком.
   MAIN_ANGLE_DEG=45 тоже требует проверки по карте. Без энкодеров/IMU
   координаты и углы оцениваются по времени, точность не гарантируется.

   Протокол фиксирован организатором: LOAD / STARTED / BUSY / DONE.
   Повтор LOAD каждые 3 с без STARTED/BUSY, всего 1+3 отправки.
   DONE ждём максимум 30 с от первого LOAD, после тайм-аута не едем.
   Без идентификаторов нельзя полностью исключить задержанный старый DONE.
   Общий лимит попытки 180 с, затем остановка до RESET. x не останавливает
   сам манипулятор. Исправная загрузка ESP32 обязательна: ошибка таблицы
   разделов flash не исправляется изменением этого скетча.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

const bool MOTOR_TEST_ONLY = false;
const bool AUTO_START = true;
const uint8_t IN1[4] = {18, 17, 16, 4};
const uint8_t IN2[4] = {26, 25, 33, 32};
const int MOTOR_SIGN[4] = {1, 1, 1, 1}; // Set -1 if this wheel runs backwards.

// Calibrate at full digital HIGH with ENA/ENB enabled.
const float FORWARD_MPS_EMPTY = 0.20f;
const float FORWARD_MPS_LOADED = 0.20f;
const float STRAFE_LEFT_MPS_EMPTY = 0.20f;
const float STRAFE_RIGHT_MPS_LOADED = 0.20f;
const float TURN_LEFT_DPS_EMPTY = 90.0f;
const float TURN_RIGHT_DPS_EMPTY = 90.0f;
const float TURN_LEFT_DPS_LOADED = 90.0f;
const float TURN_RIGHT_DPS_LOADED = 90.0f;

const float START_LEFT_M = 1.78f;
const float MAIN_DIAGONAL_M = 6.25f;
const float MAIN_ANGLE_DEG = 45.0f; // Not measured.
const float DOCK_DOWN_M = 0.35f;
const float FINISH_TOTAL_LEFT_M = 2.70f;
const float FINISH_ENTRY_M = 0.35f;
const float FINISH_DIAGONAL_M = 2.91f;

const uint32_t BOOT_DELAY_MS = 5000;
const uint32_t SETTLE_MS = 700;
const uint32_t PULSE_MS = 800;
const uint32_t ATTEMPT_LIMIT_MS = 180000;
const uint32_t WIFI_WAIT_MS = 15000;
const uint32_t ACK_TIMEOUT_MS = 3000;
const uint32_t DONE_TIMEOUT_MS = 30000;
const uint32_t RX_QUIET_MS = 100;
const uint8_t MAX_LOAD_SENDS = 4; // 1 first request + 3 retries.
const char *SSID = "ROBOT_ARM";
const char *PASSWORD = "12345678";
const IPAddress ARM_IP(192, 168, 4, 1);
const uint16_t ARM_PORT = 4210, LOCAL_PORT = 4211;

enum Motion { STOPPED, FORWARD, BACKWARD, STRAFE_LEFT, STRAFE_RIGHT,
              TURN_LEFT, TURN_RIGHT, LOAD_CUBE };
enum Mission { IDLE, RUN_ROUTE, WAIT_ARM, FINISHED, ABORTED };
enum LoadState { READY, PREPARE_LOAD, WAIT_STARTED, WAIT_DONE, LOAD_FAULT };
struct RouteStep { uint8_t motion; uint32_t durationMs; };
RouteStep route[11];
uint8_t routeCount = 0, routeIndex = 0;
Mission mission = IDLE;
LoadState loadState = READY;
WiFiUDP udp;
bool configOK = false, udpReady = false, loadComplete = false;
bool motionActive = false, settling = false;
int wheelCommand[4] = {0, 0, 0, 0};
uint32_t bootAt = 0, attemptAt = 0, motionAt = 0, motionDuration = 0;
uint32_t settleAt = 0, connectAt = 0, prepareAt = 0, rxQuietAt = 0;
uint32_t loadStart = 0, lastLoad = 0;
uint8_t loadSends = 0;
char input[33] = {0};
uint8_t inputLength = 0;
uint8_t pinTestPhase = 0, pinTestIndex = 0;
uint32_t pinTestAt = 0;

// Same working principle as motor_start.ino: one HIGH, the other LOW.
void writeMotor(uint8_t i, int direction) {
  const int d = direction * MOTOR_SIGN[i];
  if (d > 0) {
    digitalWrite(IN2[i], LOW); // Inactive input FIRST.
    digitalWrite(IN1[i], HIGH);
  } else if (d < 0) {
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], HIGH);
  } else {
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);
  }
  wheelCommand[i] = direction > 0 ? 1 : (direction < 0 ? -1 : 0);
}

void stopAll() {
  for (uint8_t i = 0; i < 4; ++i) writeMotor(i, 0);
  motionActive = false;
}

void abortMission() {
  stopAll();
  mission = ABORTED;
  loadState = LOAD_FAULT;
  loadComplete = false;
  Serial.println("ABORTED: motors stopped. Reset to repeat.");
}

void setWheels(int fl, int fr, int rl, int rr) {
  stopAll(); // Explicit all-LOW interval before changing direction.
  delayMicroseconds(100);
  writeMotor(0, fl); writeMotor(1, fr);
  writeMotor(2, rl); writeMotor(3, rr);
}

bool setMotion(uint8_t kind) {
  switch (kind) {
    case FORWARD:      setWheels( 1, 1, 1, 1); break;
    case BACKWARD:     setWheels(-1,-1,-1,-1); break;
    case STRAFE_LEFT:  setWheels(-1, 1, 1,-1); break;
    case STRAFE_RIGHT: setWheels( 1,-1,-1, 1); break;
    case TURN_LEFT:    setWheels(-1, 1,-1, 1); break;
    case TURN_RIGHT:   setWheels( 1,-1, 1,-1); break;
    default: stopAll(); return false;
  }
  return true;
}

bool appendStep(uint8_t kind, float seconds) {
  if (routeCount >= 11 || !isfinite(seconds) || seconds < 0 || seconds > 120) return false;
  route[routeCount].motion = kind;
  route[routeCount].durationMs = (uint32_t)ceilf(seconds * 1000.0f);
  ++routeCount;
  return true;
}

bool buildRoute() {
  routeCount = 0;
  const float speeds[] = {FORWARD_MPS_EMPTY, FORWARD_MPS_LOADED,
    STRAFE_LEFT_MPS_EMPTY, STRAFE_RIGHT_MPS_LOADED, TURN_LEFT_DPS_EMPTY,
    TURN_RIGHT_DPS_EMPTY, TURN_LEFT_DPS_LOADED, TURN_RIGHT_DPS_LOADED};
  for (float s : speeds) if (!isfinite(s) || s <= 0) return false;
  const float dimensions[] = {START_LEFT_M, MAIN_DIAGONAL_M, MAIN_ANGLE_DEG,
    DOCK_DOWN_M, FINISH_TOTAL_LEFT_M, FINISH_ENTRY_M, FINISH_DIAGONAL_M};
  for (float d : dimensions) if (!isfinite(d) || d < 0) return false;
  if (MAIN_ANGLE_DEG > 90 || MAIN_DIAGONAL_M <= 0 || FINISH_DIAGONAL_M <= 0 ||
      DOCK_DOWN_M <= 0 || FINISH_ENTRY_M <= 0) return false;
  for (int s : MOTOR_SIGN) if (s != 1 && s != -1) return false;
  // No GPIO may be assigned to more than one motor input.
  for (uint8_t a = 0; a < 8; ++a) {
    uint8_t pa = a < 4 ? IN1[a] : IN2[a - 4];
    for (uint8_t b = 0; b < a; ++b) {
      uint8_t pb = b < 4 ? IN1[b] : IN2[b - 4];
      if (pa == pb) return false;
    }
  }
  const float left = FINISH_TOTAL_LEFT_M - FINISH_ENTRY_M;
  if (left < 0 || left > FINISH_DIAGONAL_M) return false;
  const float up = sqrtf(fmaxf(0.0f, FINISH_DIAGONAL_M * FINISH_DIAGONAL_M - left * left));
  const float finishAngle = atan2f(up, left) * 180.0f / PI;
  if (!isfinite(finishAngle)) return false;
  // Each diagonal is a straight full-drive run AFTER rotating the chassis.
  return appendStep(FORWARD, START_LEFT_M / FORWARD_MPS_EMPTY) &&
    appendStep(TURN_LEFT, MAIN_ANGLE_DEG / TURN_LEFT_DPS_EMPTY) &&
    appendStep(FORWARD, MAIN_DIAGONAL_M / FORWARD_MPS_EMPTY) &&
    appendStep(TURN_RIGHT, MAIN_ANGLE_DEG / TURN_RIGHT_DPS_EMPTY) &&
    appendStep(STRAFE_LEFT, DOCK_DOWN_M / STRAFE_LEFT_MPS_EMPTY) &&
    appendStep(LOAD_CUBE, 0) &&
    appendStep(STRAFE_RIGHT, DOCK_DOWN_M / STRAFE_RIGHT_MPS_LOADED) &&
    appendStep(TURN_RIGHT, finishAngle / TURN_RIGHT_DPS_LOADED) &&
    appendStep(FORWARD, FINISH_DIAGONAL_M / FORWARD_MPS_LOADED) &&
    appendStep(TURN_LEFT, finishAngle / TURN_LEFT_DPS_LOADED) &&
    appendStep(FORWARD, FINISH_ENTRY_M / FORWARD_MPS_LOADED);
}

bool sendArm(const char *message) {
  if (!udpReady || WiFi.status() != WL_CONNECTED) return false;
  if (!udp.beginPacket(ARM_IP, ARM_PORT)) return false;
  if (udp.write((const uint8_t *)message, strlen(message)) != strlen(message)) return false;
  const bool ok = udp.endPacket() == 1;
  Serial.print(ok ? "TX: " : "TX failed: "); Serial.println(message);
  return ok;
}

void beginLoad() {
  if (loadState != READY || !udpReady || WiFi.status() != WL_CONNECTED) return;
  stopAll();
  loadComplete = false;
  loadState = PREPARE_LOAD;
  prepareAt = rxQuietAt = millis();
  loadSends = 0;
}

void loadFault() {
  stopAll(); loadState = LOAD_FAULT; loadComplete = false;
  Serial.println("LOAD ERROR: timeout. Staying stopped until reset.");
}

void serviceNetwork() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (!connected && udpReady) { udp.stop(); udpReady = false; }
  if (connected && !udpReady) udpReady = udp.begin(LOCAL_PORT) == 1;
  if ((loadState == WAIT_STARTED || loadState == WAIT_DONE) &&
      millis() - loadStart >= DONE_TIMEOUT_MS) loadFault();
  for (uint8_t count = 0; udpReady && count < 8; ++count) {
    const int size = udp.parsePacket();
    if (size <= 0) break;
    const bool fromArm = udp.remoteIP() == ARM_IP && udp.remotePort() == ARM_PORT;
    char buffer[80];
    int n = udp.read(buffer, sizeof(buffer) - 1);
    if (n < 0) n = 0;
    buffer[n] = '\0';
    udp.flush();
    if (loadState == PREPARE_LOAD) { rxQuietAt = millis(); continue; }
    if (!fromArm || size >= (int)sizeof(buffer) || n != size || (int)strlen(buffer) != n) continue;
    String reply(buffer); reply.trim();
    Serial.print("RX: "); Serial.println(reply);
    const bool active = loadState == WAIT_STARTED || loadState == WAIT_DONE;
    if (!active) continue;
    if (millis() - loadStart >= DONE_TIMEOUT_MS) { loadFault(); continue; }
    if (reply == "STARTED" || reply == "BUSY") loadState = WAIT_DONE;
    else if (reply == "DONE") {
      stopAll(); loadState = READY; loadComplete = true;
      Serial.println("DONE received. Delivery allowed.");
    }
  }
  const uint32_t now = millis();
  if (loadState == PREPARE_LOAD) {
    if (now - prepareAt >= WIFI_WAIT_MS) { loadFault(); return; }
    if (now - prepareAt >= SETTLE_MS && now - rxQuietAt >= RX_QUIET_MS &&
        connected && udpReady) {
      loadState = WAIT_STARTED;
      loadStart = lastLoad = now; loadSends = 1;
      sendArm("LOAD");
    }
  } else if (loadState == WAIT_STARTED || loadState == WAIT_DONE) {
    if (now - loadStart >= DONE_TIMEOUT_MS) loadFault();
    else if (loadState == WAIT_STARTED && now - lastLoad >= ACK_TIMEOUT_MS &&
             loadSends < MAX_LOAD_SENDS) {
      lastLoad = now; ++loadSends; sendArm("LOAD");
    }
  }
}

void startMission() {
  if (!configOK || mission != IDLE || motionActive || loadState != READY || loadComplete) {
    Serial.println("Cannot start: check config/motion/loading state."); return;
  }
  stopAll();
  attemptAt = AUTO_START ? bootAt : millis();
  routeIndex = 0; settling = false; mission = RUN_ROUTE;
  Serial.println("MISSION START: digital HIGH/LOW, front initially points map-left.");
}

void serviceAuto() {
  if (AUTO_START && mission == IDLE && millis() - bootAt >= BOOT_DELAY_MS) {
    startMission();
    if (mission == IDLE) abortMission();
    return; // Start the first step next loop with a fresh timestamp.
  }
  if (mission == IDLE || mission == ABORTED || mission == FINISHED) return;
  if (millis() - attemptAt >= ATTEMPT_LIMIT_MS || loadState == LOAD_FAULT) {
    abortMission(); return;
  }
  if (mission == WAIT_ARM) {
    if (loadState == READY && loadComplete) {
      loadComplete = false; ++routeIndex; mission = RUN_ROUTE;
      settling = true; settleAt = millis();
    } else if (loadState == READY) {
      if (udpReady && WiFi.status() == WL_CONNECTED) beginLoad();
      else if (millis() - connectAt >= WIFI_WAIT_MS) abortMission();
    }
    return;
  }
  if (motionActive) {
    if (millis() - motionAt >= motionDuration) {
      stopAll(); ++routeIndex; settling = true; settleAt = millis();
    }
    return;
  }
  if (settling) {
    if (millis() - settleAt < SETTLE_MS) return;
    settling = false;
  }
  if (routeIndex >= routeCount) {
    stopAll(); mission = FINISHED;
    Serial.println("FINISHED. All motor inputs LOW."); return;
  }
  const uint8_t kind = route[routeIndex].motion;
  if (kind == LOAD_CUBE) {
    stopAll(); mission = WAIT_ARM; connectAt = millis();
    Serial.println("PARKED: waiting for arm connection/loading."); return;
  }
  motionDuration = route[routeIndex].durationMs;
  if (motionDuration == 0) { ++routeIndex; return; }
  Serial.print("Step "); Serial.print(routeIndex + 1);
  Serial.print(" motion="); Serial.print((int)kind);
  Serial.print(" duration(ms)="); Serial.println(motionDuration);
  if (!setMotion(kind)) { abortMission(); return; }
  motionAt = millis(); motionActive = true;
}

void manualPulse(int fl, int fr, int rl, int rr) {
  if (mission != IDLE || loadState != READY || loadComplete) {
    Serial.println("Manual motion locked during/after loading."); return;
  }
  setWheels(fl, fr, rl, rr);
  motionAt = millis(); motionDuration = PULSE_MS; motionActive = true;
}

void help() {
  Serial.println("DIGITAL MODE: motors run at full HIGH. x: stop until reset.");
  Serial.println("AUTO_START=false: go | f b l r q e | 1..4 -1..-4 | load p s ?");
}

void command(char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') ++cmd;
  size_t len = strlen(cmd);
  while (len && (cmd[len-1] == ' ' || cmd[len-1] == '\t')) cmd[--len] = '\0';
  if (!len) return;
  if (!strcmp(cmd, "x")) { abortMission(); return; }
  if (!strcmp(cmd, "?")) { help(); return; }
  if (AUTO_START || mission != IDLE) {
    Serial.println("Manual control locked; use AUTO_START=false for calibration."); return;
  }
  if (!strcmp(cmd, "go")) { startMission(); return; }
  if (!strcmp(cmd, "load")) {
    if (loadState == READY && udpReady && WiFi.status() == WL_CONNECTED) beginLoad();
    else Serial.println("LOAD unavailable: check Wi-Fi/loading state.");
    return;
  }
  if (!strcmp(cmd, "p") || !strcmp(cmd, "s")) {
    if (loadState == READY) sendArm(cmd[0] == 'p' ? "PING" : "STATUS");
    return;
  }
  if (!strcmp(cmd, "f")) manualPulse( 1, 1, 1, 1);
  else if (!strcmp(cmd, "b")) manualPulse(-1,-1,-1,-1);
  else if (!strcmp(cmd, "l")) manualPulse(-1, 1, 1,-1);
  else if (!strcmp(cmd, "r")) manualPulse( 1,-1,-1, 1);
  else if (!strcmp(cmd, "q")) manualPulse(-1, 1,-1, 1);
  else if (!strcmp(cmd, "e")) manualPulse( 1,-1, 1,-1);
  else {
    const bool reverse = cmd[0] == '-';
    const char *number = reverse ? cmd + 1 : cmd;
    if (strlen(number) == 1 && number[0] >= '1' && number[0] <= '4') {
      int v[4] = {0,0,0,0}; v[number[0]-'1'] = reverse ? -1 : 1;
      manualPulse(v[0],v[1],v[2],v[3]);
    } else Serial.println("Unknown command. Send ?");
  }
}

void serviceSerial() {
  for (uint8_t count = 0; Serial.available() && count < 64; ++count) {
    const char c = (char)Serial.read();
    if (c == 'x' || c == 'X') { inputLength = 0; abortMission(); }
    else if (c == '\r' || c == '\n') {
      input[inputLength] = '\0'; command(input); inputLength = 0;
    } else if (inputLength < sizeof(input) - 1) input[inputLength++] = (char)tolower((unsigned char)c);
    else { inputLength = 0; abortMission(); Serial.println("Serial input too long."); }
  }
}

void startPinTestStep() {
  stopAll();
  const uint8_t motor = pinTestIndex / 2;
  const uint8_t pin = (pinTestIndex % 2) == 0 ? IN1[motor] : IN2[motor];
  Serial.print("PIN TEST motor "); Serial.print(motor + 1);
  Serial.print(" GPIO="); Serial.print(pin); Serial.println(" HIGH for 2 s");
  digitalWrite(pin, HIGH); // Raw input test intentionally ignores MOTOR_SIGN.
  pinTestAt = millis(); pinTestPhase = 1;
}

void servicePinTest() {
  for (uint8_t count = 0; Serial.available() && count < 64; ++count) {
    const char c = (char)Serial.read();
    if (c == 'x' || c == 'X') {
      stopAll(); pinTestPhase = 3; Serial.println("PIN TEST STOPPED until reset."); return;
    }
  }
  const uint32_t now = millis();
  if (pinTestPhase == 0 && now - pinTestAt >= 3000) startPinTestStep();
  else if (pinTestPhase == 1 && now - pinTestAt >= 2000) {
    stopAll(); pinTestAt = millis(); pinTestPhase = 2;
  } else if (pinTestPhase == 2 && now - pinTestAt >= 700) {
    if (++pinTestIndex < 8) startPinTestStep();
    else { stopAll(); pinTestPhase = 3; Serial.println("PIN TEST FINISHED until reset."); }
  }
}

void setup() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(IN1[i], OUTPUT); pinMode(IN2[i], OUTPUT);
    digitalWrite(IN1[i], LOW); digitalWrite(IN2[i], LOW);
  }
  Serial.begin(115200);
  stopAll();
  Serial.println("ROBOT DIGITAL: IN1=18,17,16,4; IN2=26,25,33,32");
  if (MOTOR_TEST_ONLY) {
    pinTestPhase = 0; pinTestIndex = 0;
    Serial.println("PIN TEST ONLY: wheels off ground; starts in 3 s; x stops.");
    pinTestAt = millis(); return;
  }
  configOK = buildRoute();
  if (!configOK) { Serial.println("ERROR: invalid route/configuration."); abortMission(); return; }
  WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true); WiFi.begin(SSID, PASSWORD);
  bootAt = millis();
  Serial.println(AUTO_START ? "AUTO START after 5 seconds!" : "MANUAL: send go after calibration.");
  help();
}

void loop() {
  if (MOTOR_TEST_ONLY) { servicePinTest(); delay(1); return; }
  serviceSerial();
  serviceNetwork();
  serviceAuto();
  if (mission == IDLE && motionActive && millis() - motionAt >= motionDuration) {
    stopAll(); Serial.println("Manual pulse finished.");
  }
  delay(1);
}

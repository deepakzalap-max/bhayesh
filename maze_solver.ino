/*
  Left-Hand Maze Solver  |  Arduino UNO + TB6612FNG + 3x HC-SR04
  Fully corrected & optimized edition with crosstalk filtering,
  speed-compensated timing, debounced buttons, complete path simplification,
  and junction alignment controls.
*/

// ---------- PINS ----------
const uint8_t TRIG_L = A0, ECHO_L = A1;
const uint8_t TRIG_F = A2, ECHO_F = A3;
const uint8_t TRIG_R = A4, ECHO_R = A5;
const uint8_t AIN1 = 4, AIN2 = 7, PWMA = 5;   // left motors  (A01/A02)
const uint8_t BIN1 = 8, BIN2 = 12, PWMB = 6;  // right motors (B01/B02)
const uint8_t STBY = 9;
const uint8_t BTN = 2, LED = 13;

// ---------- TUNING (adjust on your maze) ----------
const bool INV_LEFT = false, INV_RIGHT = false;  // flip if a side runs backwards
const int SEARCH_SPEED = 150, FAST_SPEED = 210, TURN_SPEED = 150;  // 0-255
const int WALL_SET   = 8;    // cm target from left wall
const int LEFT_OPEN  = 18;   // cm: left wall missing
const int RIGHT_OPEN = 18;   // cm: right wall missing
const int FRONT_STOP = 9;    // cm: wall ahead
const int FRONT_OPEN = 28;   // cm: corridor continues past junction
const int FINISH_DIST = 50;  // cm: open space distance for goal area
const float KP = 6.0, KD = 12.0;
const int TURN_90_MS  = 330, TURN_180_MS = 650;  // turn duration at TURN_SPEED
const int ADVANCE_MS  = 180;  // base centre robot before turning
const int ENTER_MS    = 250;  // base drive into new corridor after turn
const int CELL_MS     = 350;  // base pass a side opening when going straight

int curSpeed = SEARCH_SPEED;
int lastErr = 0;
bool firstWallFollow = true;
char path[64];
int plen = 0;

// ---------- HELPER FOR SPEED-SCALED TIMING ----------
// Scales move duration so linear travel distance stays constant across speeds
int getScaledMs(int baseMs) {
  return (int)((long)baseMs * SEARCH_SPEED / curSpeed);
}

// ---------- SENSORS ----------
int readCm(uint8_t t, uint8_t e) {
  digitalWrite(t, LOW);  delayMicroseconds(2);
  digitalWrite(t, HIGH); delayMicroseconds(10);
  digitalWrite(t, LOW);
  // Pulse timeout ~7000us (~120cm max distance)
  unsigned long d = pulseIn(e, HIGH, 7000UL);
  // Return 120 on timeout, but caller must handle timeout vs actual space
  return d == 0 ? 120 : (int)(d / 58);
}

void sample(int &L, int &F, int &R) {
  // 15ms delays between pings to avoid ultrasonic echo crosstalk
  L = readCm(TRIG_L, ECHO_L); delay(15);
  F = readCm(TRIG_F, ECHO_F); delay(15);
  R = readCm(TRIG_R, ECHO_R); delay(15);
}

// ---------- MOTORS ----------
void setMotor(uint8_t in1, uint8_t in2, uint8_t pwm, int s, bool inv) {
  if (inv) s = -s;
  s = constrain(s, -255, 255);
  digitalWrite(in1, s > 0 ? HIGH : LOW);
  digitalWrite(in2, s < 0 ? HIGH : LOW);
  analogWrite(pwm, abs(s));
}

void drive(int l, int r) {
  setMotor(AIN1, AIN2, PWMA, l, INV_LEFT);
  setMotor(BIN1, BIN2, PWMB, r, INV_RIGHT);
}

void halt() {                       // short brake
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255);
  delay(60);
}

void turn(int dir, int ms) {        // dir: -1 left, +1 right
  halt();
  drive(dir * TURN_SPEED, -dir * TURN_SPEED);
  delay(ms);
  halt();
}

void advance(int ms) {              // straight, stops early if wall detected ahead
  unsigned long t = millis();
  while (millis() - t < (unsigned long)ms) {
    if (readCm(TRIG_F, ECHO_F) <= FRONT_STOP) break;
    drive(curSpeed, curSpeed);
    delay(8);
  }
  halt();                           // brake at end of advance to avoid coasting
}

void followWalls(int L, int R) {    // PD centring
  int err = 0;
  if (L < LEFT_OPEN && R < RIGHT_OPEN) err = (L - R) / 2;
  else if (L < LEFT_OPEN)              err = L - WALL_SET;
  else if (R < RIGHT_OPEN)             err = WALL_SET - R;

  if (firstWallFollow) {
    lastErr = err;                  // prevent derivative spike on state entry
    firstWallFollow = false;
  }

  int corr = constrain((int)(KP * err + KD * (err - lastErr)), -70, 70);
  lastErr = err;
  drive(curSpeed - corr, curSpeed + corr);
}

// ---------- PATH & JUNCTION CLASSIFICATION ----------
char classify(int L, int F, int R) {
  // Finish detection: require valid distances > FINISH_DIST but < 120 (timeout)
  if (L > FINISH_DIST && L < 120 &&
      F > FINISH_DIST && F < 120 &&
      R > FINISH_DIST && R < 120) {
    return 'X';
  }
  if (L > LEFT_OPEN) return 'L';
  if (F <= FRONT_STOP) return (R > RIGHT_OPEN) ? 'R' : 'B';
  if (R > RIGHT_OPEN && F > FRONT_OPEN) return 'S';
  return 'C';                       // plain corridor
}

void simplify() {
  // Continuously reduce path whenever 'B' (dead end) is encountered
  while (plen >= 3 && path[plen - 2] == 'B') {
    char a = path[plen - 3], c = path[plen - 1], r = ' ';
    if      (a == 'L' && c == 'R') r = 'B';
    else if (a == 'L' && c == 'S') r = 'R';
    else if (a == 'R' && c == 'L') r = 'B';
    else if (a == 'S' && c == 'L') r = 'R';
    else if (a == 'S' && c == 'S') r = 'B';
    else if (a == 'L' && c == 'L') r = 'S';
    else if (a == 'R' && c == 'R') r = 'B';
    else if (a == 'S' && c == 'R') r = 'L';
    else if (a == 'R' && c == 'S') r = 'L';
    else break;

    plen -= 3;
    path[plen++] = r;
  }
}

void doMove(char m) {
  int advMs = getScaledMs(ADVANCE_MS);
  int enterMs = getScaledMs(ENTER_MS);
  int cellMs = getScaledMs(CELL_MS);

  switch (m) {
    case 'L':
      advance(advMs);
      turn(-1, TURN_90_MS);
      advance(enterMs);
      break;
    case 'R':
      advance(advMs);               // advance to junction centre before right turn
      turn(+1, TURN_90_MS);
      advance(enterMs);
      break;
    case 'B':
      turn(+1, TURN_180_MS);
      advance(enterMs);
      break;
    case 'S':
      advance(cellMs);
      break;
  }
  firstWallFollow = true;           // reset wall follow derivative flag
}

void celebrate() {
  halt(); drive(0, 0);
  for (int i = 0; i < 10; i++) { digitalWrite(LED, i % 2); delay(150); }
  digitalWrite(LED, LOW);
}

void waitButton() {
  // Wait for button release (HIGH with debouncing)
  while (digitalRead(BTN) == LOW) delay(20);
  // Wait for button press (LOW with debouncing)
  while (digitalRead(BTN) == HIGH) delay(20);
  delay(50); // debounce delay

  // Blink LED signal
  for (int i = 0; i < 6; i++) { digitalWrite(LED, i % 2); delay(250); }
  digitalWrite(LED, LOW);
}

// ---------- RUN MAZE ----------
void runMaze(bool fast) {
  curSpeed = fast ? FAST_SPEED : SEARCH_SPEED;
  int idx = 0, L, F, R;
  firstWallFollow = true;
  if (!fast) plen = 0;

  while (true) {
    sample(L, F, R);
    char ev = classify(L, F, R);

    // Normal wall follow mode inside plain corridors
    if (ev == 'C') {
      followWalls(L, R);
      continue;
    }

    halt();                         // stop to confirm junction
    delay(30);
    sample(L, F, R);
    if (classify(L, F, R) != ev) continue; // ignore false sensor spikes

    if (ev == 'X') { celebrate(); return; }

    char mv = ev;
    if (fast) {
      if (idx < plen) {
        mv = path[idx++];
      } else {
        // Fallback safety if path index exceeds recorded length
        mv = ev;
      }
    } else {
      if (plen < 60) {
        path[plen++] = mv;
        simplify();
      }
    }

    doMove(mv);
  }
}

void setup() {
  uint8_t out[] = {TRIG_L, TRIG_F, TRIG_R, AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY, LED};
  for (uint8_t p : out) pinMode(p, OUTPUT);
  pinMode(ECHO_L, INPUT); pinMode(ECHO_F, INPUT); pinMode(ECHO_R, INPUT);
  pinMode(BTN, INPUT_PULLUP);
  digitalWrite(STBY, HIGH);
  Serial.begin(9600);

  waitButton();  runMaze(false);   // run 1: explore
  Serial.print("Shortest path: ");
  for (int i = 0; i < plen; i++) Serial.print(path[i]);
  Serial.println();

  waitButton();  runMaze(true);    // run 2: fast run
}

void loop() {}

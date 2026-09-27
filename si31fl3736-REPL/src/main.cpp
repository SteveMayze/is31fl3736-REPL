// IS31FL3736 bring-up / trim REPL
// Target: Teensy 3.1/3.2, Arduino framework, Wire (I2C)
//
// Serial commands:
//   help
//   mode pwm                              -> global PWM mode (B_EN=0)
//   mode abm                              -> global Auto Breath mode (B_EN=1)
//   load pwm BBGGRR BBGGRR ...             -> one 6-hex-digit BBGGRR token per physical RGB
//                                             LED (dot), starting at LED 0, e.g.
//                                             load pwm 0a0b30 0a0b3f Fa7bC3
//                                             (non-zero channel also turns that dot's On/Off bit on)
//   load abm m0,m1,m2, ...                 -> flat list of ABM modes (0-3), assigned to dots
//                                             starting at dot 0, in order
//   assign abm <n> <dot...>               -> same assignment, but by explicit dot index
//                                             (n = 0 for PWM control, 1-3 for ABM-1..3)
//   define abm <n> <T1> <T2> <T3> <T4>    -> program ABM-n timing (raw register codes, not
//                                             seconds - see Table 15/16 of the datasheet) and
//                                             commit per Figure 16 (clear/set B_EN, update 0Eh)
//   gcc <0-255>                           -> Global Current Control (PG3, 01h)
//   reset                                 -> trigger IC reset (read PG3, 11h) and re-init
//   dump                                  -> print current shadow state
//
// I2C address assumes ADDR1 = ADDR2 = GND (7-bit address 0x50). Change I2C_ADDR below
// if ADDR1/ADDR2 are strapped differently (see Table 1 in the datasheet).

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Chip constants
// ---------------------------------------------------------------------------

static const uint8_t I2C_ADDR = 0x50; // ADDR1=ADDR2=GND -> A4:A3=00, A2:A1=00

// Top-level registers (not behind a page select)
static const uint8_t REG_CMD        = 0xFD; // Command register (page select), write-only
static const uint8_t REG_CMD_LOCK   = 0xFE; // Command register write lock
static const uint8_t REG_INT_MASK   = 0xF0; // Interrupt Mask Register
static const uint8_t REG_INT_STATUS = 0xF1; // Interrupt Status Register (read)
static const uint8_t CMD_UNLOCK_KEY = 0xC5;

// Pages selected via REG_CMD
static const uint8_t PAGE_LEDCTRL = 0x00; // On/Off, Open, Short
static const uint8_t PAGE_PWM     = 0x01; // PWM duty, 00h~BEh
static const uint8_t PAGE_ABM     = 0x02; // per-dot Auto Breath Mode select, 00h~BEh
static const uint8_t PAGE_FUNC    = 0x03; // Function Register

// Function Register (PG3) offsets
static const uint8_t FN_CONFIG    = 0x00;
static const uint8_t FN_GCC       = 0x01;
static const uint8_t FN_ABM1_BASE = 0x02; // 02h..05h
static const uint8_t FN_ABM2_BASE = 0x06; // 06h..09h
static const uint8_t FN_ABM3_BASE = 0x0A; // 0Ah..0Dh
static const uint8_t FN_TIME_UPD  = 0x0E;
static const uint8_t FN_SWY_PUR   = 0x0F;
static const uint8_t FN_CSX_PDR   = 0x10;
static const uint8_t FN_RESET     = 0x11;

// Configuration Register (PG3, 00h) bit masks
static const uint8_t CFG_SSD_BIT  = 0x01; // 0=shutdown, 1=normal operation
static const uint8_t CFG_BEN_BIT  = 0x02; // 0=PWM mode, 1=Auto Breath mode
static const uint8_t CFG_OSD_BIT  = 0x04; // open/short detect trigger
static const uint8_t CFG_SYNC_MASK = 0xC0;

static const uint8_t NUM_SW = 12;
static const uint8_t NUM_CS = 8;
static const uint8_t NUM_DOTS = NUM_SW * NUM_CS; // 96

// ---------------------------------------------------------------------------
// Shadow state (registers on this chip are write-only, so we track them here)
// ---------------------------------------------------------------------------

static uint8_t g_config = 0x00;
static uint8_t g_gcc    = 0x00;
static uint8_t g_pwm[NUM_DOTS];      // last PWM value written per dot
static uint8_t g_onoff[24];          // LED On/Off Register shadow, 00h~17h
static uint8_t g_abmAssign[NUM_DOTS]; // per-dot ABM selection shadow (0..3)

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------

static bool i2cWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool i2cWriteBlock(uint8_t startReg, const uint8_t *data, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(startReg);
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

static int i2cReadReg(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1; // repeated start, keep bus
  if (Wire.requestFrom(I2C_ADDR, (uint8_t)1) != 1) return -1;
  if (!Wire.available()) return -1;
  return Wire.read();
}

// Unlock FDh (one-shot) and point it at the requested page.
static bool selectPage(uint8_t page) {
  if (!i2cWriteReg(REG_CMD_LOCK, CMD_UNLOCK_KEY)) return false;
  return i2cWriteReg(REG_CMD, page);
}

static bool writePageReg(uint8_t page, uint8_t reg, uint8_t value) {
  if (!selectPage(page)) return false;
  return i2cWriteReg(reg, value);
}

static bool writePageBlock(uint8_t page, uint8_t startReg, const uint8_t *data, size_t len) {
  if (!selectPage(page)) return false;
  return i2cWriteBlock(startReg, data, len);
}

// ---------------------------------------------------------------------------
// Dot <-> register address mapping (12 SW rows x 8 CS columns, per datasheet
// Figure 9 for PWM and Table 6/7 for On/Off)
// ---------------------------------------------------------------------------

static uint8_t pwmAddrForDot(uint8_t sw /*1-12*/, uint8_t cs /*1-8*/) {
  return (uint8_t)((sw - 1) * 0x10 + (cs - 1) * 2);
}

static uint8_t abmAddrForDot(uint8_t sw, uint8_t cs) {
  // Auto Breath Mode select register shares the same layout as the PWM page.
  return pwmAddrForDot(sw, cs);
}

static void onoffAddrForDot(uint8_t sw, uint8_t cs, uint8_t &addr, uint8_t &bitPos) {
  uint8_t half = (uint8_t)((cs - 1) / 4);       // 0 = CS1-4, 1 = CS5-8
  addr = (uint8_t)(2 * (sw - 1) + half);
  uint8_t localCs = (uint8_t)((cs - 1) % 4);    // 0..3
  bitPos = (uint8_t)(2 * localCs);              // bit 0,2,4,6
}

static void dotToSwCs(uint8_t dot, uint8_t &sw, uint8_t &cs) {
  sw = (uint8_t)(dot / NUM_CS + 1);
  cs = (uint8_t)(dot % NUM_CS + 1);
}

// Write one dot's PWM value and keep its On/Off bit in sync (on if value>0).
static void setDotPwm(uint8_t dot, uint8_t value) {
  if (dot >= NUM_DOTS) return;
  uint8_t sw, cs;
  dotToSwCs(dot, sw, cs);

  g_pwm[dot] = value;
  writePageReg(PAGE_PWM, pwmAddrForDot(sw, cs), value);

  uint8_t addr, bitPos;
  onoffAddrForDot(sw, cs, addr, bitPos);
  if (value > 0) g_onoff[addr] |= (uint8_t)(1 << bitPos);
  else           g_onoff[addr] &= (uint8_t)~(1 << bitPos);
}

// Push the whole On/Off shadow (00h~17h, 24 bytes) out in one block write.
static void flushOnOff() {
  writePageBlock(PAGE_LEDCTRL, 0x00, g_onoff, sizeof(g_onoff));
}

static void writeConfig(uint8_t value) {
  g_config = value;
  writePageReg(PAGE_FUNC, FN_CONFIG, value);
}

static void updateConfigBit(uint8_t mask, bool set) {
  if (set) g_config |= mask;
  else     g_config &= (uint8_t)~mask;
  writeConfig(g_config);
}

// ---------------------------------------------------------------------------
// High level operations
// ---------------------------------------------------------------------------

static void chipReset() {
  selectPage(PAGE_FUNC);
  i2cReadReg(FN_RESET); // reading this register resets all registers to POR state

  g_config = 0x00;
  g_gcc = 0x00;
  memset(g_pwm, 0, sizeof(g_pwm));
  memset(g_onoff, 0, sizeof(g_onoff));
  memset(g_abmAssign, 0, sizeof(g_abmAssign));

  // Bring the chip out of software shutdown so it actually drives the matrix.
  updateConfigBit(CFG_SSD_BIT, true);
}

static void setGcc(uint8_t value) {
  g_gcc = value;
  writePageReg(PAGE_FUNC, FN_GCC, value);
}

static void setGlobalMode(bool abm) {
  updateConfigBit(CFG_BEN_BIT, abm);
}

// Assign a dot's per-dot Auto Breath Mode selection register (00=PWM control,
// 01/10/11 = ABM-1/2/3). Required for a dot to actually run any ABM timing.
static void assignDotAbm(uint8_t dot, uint8_t mode /*0-3*/) {
  if (dot >= NUM_DOTS || mode > 3) return;
  uint8_t sw, cs;
  dotToSwCs(dot, sw, cs);
  g_abmAssign[dot] = mode;
  writePageReg(PAGE_ABM, abmAddrForDot(sw, cs), mode & 0x03);
}

// Program one ABM slot's timing and commit it per the datasheet's Figure 16
// flow: write 02h~0Dh -> clear B_EN -> set B_EN -> write 0Eh=0x00.
static void defineAbm(uint8_t n /*1-3*/, uint8_t T1, uint8_t T2, uint8_t T3, uint8_t T4) {
  if (n < 1 || n > 3) return;
  uint8_t base = (n == 1) ? FN_ABM1_BASE : (n == 2) ? FN_ABM2_BASE : FN_ABM3_BASE;

  uint8_t regs[4];
  regs[0] = (uint8_t)(((T1 & 0x07) << 5) | ((T2 & 0x0F) << 1)); // fade-in / hold
  regs[1] = (uint8_t)(((T3 & 0x07) << 5) | ((T4 & 0x0F) << 1)); // fade-out / off
  regs[2] = 0x00; // LE=00 (end at off), LB=00 (begin at T1), LTA=0000 (endless loop)
  regs[3] = 0x00; // LTB=0 -> combined with LTA=0 this means endless loop

  writePageBlock(PAGE_FUNC, base, regs, sizeof(regs));

  updateConfigBit(CFG_BEN_BIT, false); // clear B_EN
  updateConfigBit(CFG_BEN_BIT, true);  // set B_EN
  writePageReg(PAGE_FUNC, FN_TIME_UPD, 0x00); // commit 02h~0Dh
}

// ---------------------------------------------------------------------------
// Command parsing helpers
// ---------------------------------------------------------------------------

static const uint8_t MAX_TOKENS = 100;

// Split on space, tab and comma. Empty tokens are skipped.
static uint8_t tokenize(char *line, char *tokens[], uint8_t maxTokens) {
  uint8_t count = 0;
  char *p = strtok(line, " \t,");
  while (p != nullptr && count < maxTokens) {
    tokens[count++] = p;
    p = strtok(nullptr, " \t,");
  }
  return count;
}

static bool parseByte(const char *s, uint8_t &out) {
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s || v < 0 || v > 255) return false;
  out = (uint8_t)v;
  return true;
}

// Parses a 6-hex-digit BBGGRR token, e.g. "0a0b30" or "Fa7bC3".
static bool parseHexTriplet(const char *s, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (strlen(s) != 6) return false;
  char *end;
  long value = strtol(s, &end, 16);
  if (end != s + 6 || value < 0) return false;
  b = (uint8_t)((value >> 16) & 0xFF);
  g = (uint8_t)((value >> 8) & 0xFF);
  r = (uint8_t)(value & 0xFF);
  return true;
}

// ---------------------------------------------------------------------------
// Command implementations
// ---------------------------------------------------------------------------

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  mode pwm"));
  Serial.println(F("  mode abm"));
  Serial.println(F("  load pwm BBGGRR BBGGRR ...         (one hex triplet per LED, LED 0 upward)"));
  Serial.println(F("  load abm m0,m1,m2, ...             (dot 0 upward, 0=PWM control, 1-3=ABM-1..3)"));
  Serial.println(F("  assign abm <n 0-3> <dot...>        (same, but by explicit dot index)"));
  Serial.println(F("  define abm <n 1-3> <T1> <T2> <T3> <T4>   (raw codes, see datasheet Table 15/16)"));
  Serial.println(F("  gcc <0-255>"));
  Serial.println(F("  reset"));
  Serial.println(F("  dump"));
}

static void cmdMode(uint8_t argc, char *argv[]) {
  if (argc < 2) { Serial.println(F("usage: mode pwm|abm")); return; }
  if (strcmp(argv[1], "pwm") == 0) {
    setGlobalMode(false);
    Serial.println(F("OK global mode = PWM"));
  } else if (strcmp(argv[1], "abm") == 0) {
    setGlobalMode(true);
    Serial.println(F("OK global mode = ABM"));
  } else {
    Serial.println(F("usage: mode pwm|abm"));
  }
}

// Each token is one RGB LED as a 6-hex-digit RRGGBB value, e.g.
//   load pwm 0a0b30 0a0b3f Fa7bC3
// LED n: R=SW(n%4*3+1)/CS(n/4+1), G=next SW row, B=next SW row after that.
static void cmdLoadPwm(uint8_t argc, char *argv[]) {
  static const uint8_t NUM_LEDS = (NUM_SW / 3) * NUM_CS; // 32
  uint8_t led = 0;
  for (uint8_t i = 2; i < argc; i++, led++) {
    if (led >= NUM_LEDS) {
      Serial.println(F("ran out of dots, stopping"));
      break;
    }
    uint8_t r, g, b;
    if (!parseHexTriplet(argv[i], r, g, b)) {
      Serial.print(F("bad BBGGRR value: "));
      Serial.println(argv[i]);
      return;
    }
    uint8_t cs  = (uint8_t)(led / 4 + 1);
    uint8_t swR = (uint8_t)((led % 4) * 3 + 1);
    setDotPwm((uint8_t)((swR - 1) * NUM_CS + (cs - 1)), r);
    setDotPwm((uint8_t)((swR    ) * NUM_CS + (cs - 1)), g);
    setDotPwm((uint8_t)((swR + 1) * NUM_CS + (cs - 1)), b);
  }
  flushOnOff();
  Serial.print(F("OK loaded "));
  Serial.print(led);
  Serial.println(F(" LED(s)"));
}

// Assign ABM mode per dot in sequence, starting at dot 0 - e.g.
//   load abm 1, 1, 1, 2, 2, 0, 0, ...
// gives dot 0-2 ABM-1, dots 3-4 ABM-2, dots 5-6 back to PWM control, etc.
static void cmdLoadAbm(uint8_t argc, char *argv[]) {
  uint8_t dot = 0;
  for (uint8_t i = 2; i < argc && dot < NUM_DOTS; i++, dot++) {
    uint8_t mode;
    if (!parseByte(argv[i], mode) || mode > 3) {
      Serial.print(F("bad mode (0-3): "));
      Serial.println(argv[i]);
      return;
    }
    assignDotAbm(dot, mode);
  }
  Serial.print(F("OK assigned "));
  Serial.print(dot);
  Serial.println(F(" abm dot(s)"));
}

static void cmdLoad(uint8_t argc, char *argv[]) {
  if (argc < 2) {
    Serial.println(F("usage: load pwm BBGGRR...  |  load abm m0,m1,m2,..."));
    return;
  }
  if (strcmp(argv[1], "pwm") == 0) {
    cmdLoadPwm(argc, argv);
  } else if (strcmp(argv[1], "abm") == 0) {
    cmdLoadAbm(argc, argv);
  } else {
    Serial.println(F("usage: load pwm BBGGRR...  |  load abm m0,m1,m2,..."));
  }
}

static void cmdAssign(uint8_t argc, char *argv[]) {
  if (argc < 3 || strcmp(argv[1], "abm") != 0) {
    Serial.println(F("usage: assign abm <n 0-3> <dot...>"));
    return;
  }
  uint8_t mode;
  if (!parseByte(argv[2], mode) || mode > 3) {
    Serial.println(F("mode must be 0-3"));
    return;
  }
  for (uint8_t i = 3; i < argc; i++) {
    uint8_t dot;
    if (!parseByte(argv[i], dot) || dot >= NUM_DOTS) {
      Serial.print(F("bad dot index: "));
      Serial.println(argv[i]);
      continue;
    }
    assignDotAbm(dot, mode);
  }
  Serial.println(F("OK"));
}

static void cmdDefineAbm(uint8_t argc, char *argv[]) {
  if (argc < 7 || strcmp(argv[1], "abm") != 0) {
    Serial.println(F("usage: define abm <n 1-3> <T1> <T2> <T3> <T4>"));
    return;
  }
  uint8_t n, t1, t2, t3, t4;
  if (!parseByte(argv[2], n) || n < 1 || n > 3 ||
      !parseByte(argv[3], t1) || !parseByte(argv[4], t2) ||
      !parseByte(argv[5], t3) || !parseByte(argv[6], t4)) {
    Serial.println(F("usage: define abm <n 1-3> <T1> <T2> <T3> <T4>"));
    return;
  }
  defineAbm(n, t1, t2, t3, t4);
  Serial.print(F("OK ABM-"));
  Serial.println(n);
}

static void cmdGcc(uint8_t argc, char *argv[]) {
  if (argc < 2) { Serial.println(F("usage: gcc <0-255>")); return; }
  uint8_t value;
  if (!parseByte(argv[1], value)) { Serial.println(F("bad value")); return; }
  setGcc(value);
  Serial.print(F("OK gcc="));
  Serial.println(value);
}

static void cmdDump() {
  Serial.print(F("config=0x")); Serial.println(g_config, HEX);
  Serial.print(F("gcc=")); Serial.println(g_gcc);
  Serial.println(F("pwm (dot:value), non-zero only:"));
  for (uint8_t d = 0; d < NUM_DOTS; d++) {
    if (g_pwm[d] != 0) {
      Serial.print(d); Serial.print(':'); Serial.print(g_pwm[d]); Serial.print(' ');
    }
  }
  Serial.println();
  Serial.println(F("abm assignment (dot:mode), non-zero only:"));
  for (uint8_t d = 0; d < NUM_DOTS; d++) {
    if (g_abmAssign[d] != 0) {
      Serial.print(d); Serial.print(':'); Serial.print(g_abmAssign[d]); Serial.print(' ');
    }
  }
  Serial.println();
}

static void handleLine(char *line) {
  char *argv[MAX_TOKENS];
  uint8_t argc = tokenize(line, argv, MAX_TOKENS);
  if (argc == 0) return;

  if (strcmp(argv[0], "help") == 0) {
    printHelp();
  } else if (strcmp(argv[0], "mode") == 0) {
    cmdMode(argc, argv);
  } else if (strcmp(argv[0], "load") == 0) {
    cmdLoad(argc, argv);
  } else if (strcmp(argv[0], "assign") == 0) {
    cmdAssign(argc, argv);
  } else if (strcmp(argv[0], "define") == 0) {
    cmdDefineAbm(argc, argv);
  } else if (strcmp(argv[0], "gcc") == 0) {
    cmdGcc(argc, argv);
  } else if (strcmp(argv[0], "reset") == 0) {
    chipReset();
    Serial.println(F("OK reset"));
  } else if (strcmp(argv[0], "dump") == 0) {
    cmdDump();
  } else {
    Serial.print(F("unknown command: "));
    Serial.println(argv[0]);
  }
}

// ---------------------------------------------------------------------------
// Serial line buffering
// ---------------------------------------------------------------------------

static char g_lineBuf[160];
static uint8_t g_lineLen = 0;

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      Serial.println();
      g_lineBuf[g_lineLen] = '\0';
      if (g_lineLen > 0) handleLine(g_lineBuf);
      g_lineLen = 0;
      Serial.print(F("> "));
    } else if (c == 0x08 || c == 0x7F) { // backspace or delete
      if (g_lineLen > 0) {
        g_lineLen--;
        Serial.print(F("\b \b")); // erase the character on the terminal
      }
    } else if (g_lineLen < sizeof(g_lineBuf) - 1) {
      g_lineBuf[g_lineLen++] = c;
      Serial.write(c); // local echo, since most serial terminals don't echo for you
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { /* wait for host on native USB */ }

  Wire.begin();
  Wire.setClock(400000);

  chipReset();

  Serial.println(F("IS31FL3736 REPL ready. Type 'help' for commands."));
  Serial.print(F("> "));
}

void loop() {
  pollSerial();
}

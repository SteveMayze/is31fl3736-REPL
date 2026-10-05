// IS31FL3736 bring-up / trim REPL
// Target: Teensy 3.1/3.2, Arduino framework, Wire (I2C)
//
// Serial commands:
//   help
//   panel [<n 1-3>]                       -> number of panels incl. the master (default 1).
//                                             Panels stack in Y: 8x4, 8x8 or 8x12 RGB LEDs,
//                                             with the master at the BOTTOM (see MASTER_AT_TOP).
//                                             Changing it resets all panels and sets SYNC
//                                             (master=01, slaves=10) when n > 1
//   mode pwm                              -> global PWM mode (B_EN=0), all panels
//   mode abm                              -> global Auto Breath mode (B_EN=1), all panels
//   load pwm RRGGBB RRGGBB ...             -> one 6-hex-digit RRGGBB token per physical RGB
//                                             LED (dot), starting at LED 0 = bottom-left of
//                                             the whole display, e.g.
//                                             load pwm 0a0b30 0a0b3f Fa7bC3
//                                             (non-zero channel also turns that dot's On/Off bit on)
//   load abm mR mG mB mR mG mB ...         -> one ABM mode (0-3) per colour channel: R,G,B of
//                                             LED 0, then R,G,B of LED 1, ... (LEDs X first).
//                                             ABM dots always breathe 0 -> full (their PWM
//                                             value does not scale the peak)
//   fill pwm from <s> to <e> with RRGGBB ...  -> repeat the RRGGBB pattern over RGB LEDs s..e
//                                             (X first, then upward through the whole stack,
//                                             whichever chip each LED is on), truncated at e
//   fill abm from <s> to <e> with m m ...  -> repeat the mode pattern over the R,G,B channels
//                                             of RGB LEDs s..e, truncated at e's B channel
//   assign abm <n> <dot...>               -> same assignment, but by explicit dot index
//                                             (panel*96 + chip dot; n = 0 for PWM control,
//                                             1-3 for ABM-1..3)
//   define abm <n> <T1> <T2> <T3> <T4> [start <1-4>] [end on|off] [loop <0-4095>]
//                                         -> program ABM-n timing on all panels (raw register
//                                             codes, not seconds - see Table 15/16 of the
//                                             datasheet) and commit per Figure 16. Optional,
//                                             any order: 'start' sets LB, the phase the loop
//                                             begins at (default T1); 'end' sets LE, finish
//                                             on (end of T1) or off (end of T3, default);
//                                             'loop' sets LTA:LTB, the repeat count (0 = endless)
//   gcc <0-255>                           -> Global Current Control (PG3, 01h), all panels
//   reset                                 -> trigger IC reset (read PG3, 11h) and re-init
//   wave <RRGGBB ...|rainbow|rainbow-bump> [width <leds>] [plateau <leds>] [gap <leds>] [speed <leds/s>] [dir <d>]
//                         [sharp <1-8>] [fps <5-60>]
//                                         -> smooth travelling wave generated on the Teensy with PWM
//                                             (not ABM): every LED fades between black and its colour.
//                                             One bump is 'width' LEDs long (rise + 'plateau' held at
//                                             full + fall), bumps are 'gap' dark LEDs apart. 'rainbow'
//                                             instead of colours is a continuous scrolling rainbow:
//                                             hue follows the position along <d>, 'width' LEDs per full
//                                             red-to-red cycle, no black ('gap' adds dark LEDs between
//                                             cycles, 'plateau' and 'sharp' are ignored). 'rainbow-bump'
//                                             paints the spectrum (red -> violet) across each bump
//                                             instead. Colours tile over
//                                             the LEDs in display order (short = repeat, long = truncate,
//                                             bad entry = black). <d> = right left up down bl-tr br-tl tl-br
//                                             tr-bl. Never an error: bad or missing values fall back to
//                                             defaults/clamps. 'wave off' (or any command that changes the
//                                             LEDs) stops it
//   dump                                  -> print current shadow state
//
// I2C addresses per panel are in PANEL_ADDR below (see Table 1 in the datasheet).

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Chip constants
// ---------------------------------------------------------------------------

// Panel 0 is the SYNC master; the others are SYNC slaves. Each board's address is set by its
// ADDR1/ADDR2 solder jumpers, so a board's position on the shared I2C ribbon does not matter.
// 7-bit addresses for Wire (8-bit write addresses 0xA0/0xA2/0xA4 >> 1).
static const uint8_t MAX_PANELS = 3;

// Physical assembly: the master (panel 0) is the BOTTOM panel of the stack and the slaves sit above it.
// LED numbers, however, are display coordinates (LED 0 = bottom-left of the whole display, X first,
// then upward), so rgbDot() below maps them onto the chips. Set this to true if the master is ever
// the top panel; nothing else needs to change.
static const bool MASTER_AT_TOP = false;

static const uint8_t PANEL_ADDR[MAX_PANELS] = {
  0x50, // master:  ADDR2=GND, ADDR1=GND -> A4:A3=00, A2:A1=00
  0x51, // slave 1: ADDR2=GND, ADDR1=SCL -> A4:A3=00, A2:A1=01
  0x52, // slave 2: ADDR2=GND, ADDR1=SDA -> A4:A3=00, A2:A1=10
};

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
static const uint8_t CFG_SYNC_MASK   = 0xC0;
static const uint8_t CFG_SYNC_MASTER = 0x40; // SYNC=01, SYNC pin outputs the clock
static const uint8_t CFG_SYNC_SLAVE  = 0x80; // SYNC=10, SYNC pin takes the clock

static const uint8_t NUM_SW = 12;
static const uint8_t NUM_CS = 8;
static const uint8_t NUM_DOTS = NUM_SW * NUM_CS; // 96 per chip
static const uint8_t LEDS_PER_PANEL = (NUM_SW / 3) * NUM_CS; // 32 RGB LEDs per chip

// ---------------------------------------------------------------------------
// Shadow state (registers on this chip are write-only, so we track them here)
// ---------------------------------------------------------------------------

static uint8_t g_numPanels = 1;
static uint8_t g_addr = PANEL_ADDR[0];                 // chip the I2C helpers talk to
static uint8_t g_config[MAX_PANELS];
static uint8_t g_gcc = 0x00;                           // same on every panel
static uint8_t g_pwm[MAX_PANELS][NUM_DOTS];            // last PWM value written per dot
static uint8_t g_onoff[MAX_PANELS][24];                // LED On/Off Register shadow, 00h~17h
static uint8_t g_abmAssign[MAX_PANELS][NUM_DOTS];      // per-dot ABM selection shadow (0..3)

static uint8_t numLeds() { return (uint8_t)(g_numPanels * LEDS_PER_PANEL); }
static uint16_t numDots() { return (uint16_t)(g_numPanels * NUM_DOTS); }

// ---------------------------------------------------------------------------
// Low-level I2C helpers (all talk to g_addr, set via selectChip)
// ---------------------------------------------------------------------------

static void selectChip(uint8_t panel) {
  g_addr = PANEL_ADDR[panel];
}

static bool chipPresent(uint8_t panel) {
  Wire.beginTransmission(PANEL_ADDR[panel]);
  return Wire.endTransmission() == 0;
}

static bool i2cWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(g_addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool i2cWriteBlock(uint8_t startReg, const uint8_t *data, size_t len) {
  Wire.beginTransmission(g_addr);
  Wire.write(startReg);
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

static int i2cReadReg(uint8_t reg) {
  Wire.beginTransmission(g_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1; // repeated start, keep bus
  if (Wire.requestFrom(g_addr, (uint8_t)1) != 1) return -1;
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

// A global dot index is panel * 96 + the chip's own dot index.
static void splitDot(uint16_t gdot, uint8_t &panel, uint8_t &dot) {
  panel = (uint8_t)(gdot / NUM_DOTS);
  dot = (uint8_t)(gdot % NUM_DOTS);
}

// Physical RGB LED layout: each panel is 8 LEDs in X (CS1..CS8) x 4 LEDs in Y (SW row groups of 3),
// panels stacked in Y. LEDs are numbered as display coordinates, X first: LED n is at x = n % 8 and
// display row n / 8, where row 0 is the BOTTOM row of the whole stack, whichever chip that row is on.
// The master (panel 0) is the top panel only when MASTER_AT_TOP; otherwise it is the bottom panel and
// holds the stack's bottom row; within a panel, local row 0 (SW1-3) is its bottom row. A channel on CS(x+1) is wired
// B=SW(y'*3+1), G=SW(y'*3+2), R=SW(y'*3+3) (y' = local row), i.e. the panel has R and B in reverse SW
// order - this is the one place that swap lives.
static uint16_t rgbDot(uint8_t led, uint8_t channel /*0=R,1=G,2=B*/) {
  uint8_t x = (uint8_t)(led % NUM_CS);
  uint8_t row = (uint8_t)(led / NUM_CS);                    // 0 = bottom of the display
  uint8_t panelFromBottom = (uint8_t)(row / (NUM_SW / 3));
  uint8_t y = (uint8_t)(row % (NUM_SW / 3));                // row within the panel
  uint8_t panel = MASTER_AT_TOP ? (uint8_t)(g_numPanels - 1 - panelFromBottom) : panelFromBottom;
  return (uint16_t)(panel * NUM_DOTS + (y * 3 + (2 - channel)) * NUM_CS + x);
}

// A dot must be on if it has a PWM value or is running an ABM pattern.
static void syncDotOnOff(uint8_t panel, uint8_t dot) {
  uint8_t sw, cs, addr, bitPos;
  dotToSwCs(dot, sw, cs);
  onoffAddrForDot(sw, cs, addr, bitPos);
  if (g_pwm[panel][dot] > 0 || g_abmAssign[panel][dot] != 0)
    g_onoff[panel][addr] |= (uint8_t)(1 << bitPos);
  else
    g_onoff[panel][addr] &= (uint8_t)~(1 << bitPos);
}

// Write one dot's PWM value and keep its On/Off bit in sync.
static void setDotPwm(uint16_t gdot, uint8_t value) {
  if (gdot >= numDots()) return;
  uint8_t panel, dot, sw, cs;
  splitDot(gdot, panel, dot);
  dotToSwCs(dot, sw, cs);

  g_pwm[panel][dot] = value;
  selectChip(panel);
  writePageReg(PAGE_PWM, pwmAddrForDot(sw, cs), value);
  syncDotOnOff(panel, dot);
}

// Push each active panel's On/Off shadow (00h~17h, 24 bytes) out in one block write.
static void flushOnOff() {
  for (uint8_t p = 0; p < g_numPanels; p++) {
    selectChip(p);
    writePageBlock(PAGE_LEDCTRL, 0x00, g_onoff[p], sizeof(g_onoff[p]));
  }
}

static void writeConfig(uint8_t panel) {
  selectChip(panel);
  writePageReg(PAGE_FUNC, FN_CONFIG, g_config[panel]);
}

// Set or clear a config bit on every active panel.
static void updateConfigBit(uint8_t mask, bool set) {
  for (uint8_t p = 0; p < g_numPanels; p++) {
    if (set) g_config[p] |= mask;
    else     g_config[p] &= (uint8_t)~mask;
    writeConfig(p);
  }
}

// Write the same Function Register value to every active panel.
static void writeFuncAll(uint8_t reg, uint8_t value) {
  for (uint8_t p = 0; p < g_numPanels; p++) {
    selectChip(p);
    writePageReg(PAGE_FUNC, reg, value);
  }
}

// ---------------------------------------------------------------------------
// High level operations
// ---------------------------------------------------------------------------

static uint8_t syncBitsFor(uint8_t panel) {
  if (g_numPanels == 1) return 0x00; // SYNC pin high impedance
  return panel == 0 ? CFG_SYNC_MASTER : CFG_SYNC_SLAVE;
}

// Resets every possible panel (so a panel dropped by 'panel <n>' is blanked -
// POR leaves it in software shutdown), then brings the active ones up with
// their SYNC role. Slaves are configured before the master so they are all
// waiting for its clock when it starts.
static void chipReset() {
  for (uint8_t p = 0; p < MAX_PANELS; p++) {
    selectChip(p);
    selectPage(PAGE_FUNC);
    i2cReadReg(FN_RESET); // reading this register resets all registers to POR state
  }

  g_gcc = 0x00;
  memset(g_pwm, 0, sizeof(g_pwm));
  memset(g_onoff, 0, sizeof(g_onoff));
  memset(g_abmAssign, 0, sizeof(g_abmAssign));

  for (int8_t p = (int8_t)(g_numPanels - 1); p >= 0; p--) {
    // SSD=1 brings the chip out of software shutdown so it drives the matrix.
    g_config[p] = (uint8_t)(syncBitsFor((uint8_t)p) | CFG_SSD_BIT);
    writeConfig((uint8_t)p);
  }
}

static void setGcc(uint8_t value) {
  g_gcc = value;
  writeFuncAll(FN_GCC, value);
}

static void setGlobalMode(bool abm) {
  updateConfigBit(CFG_BEN_BIT, abm);
}

// Assign a dot's per-dot Auto Breath Mode selection register (00=PWM control,
// 01/10/11 = ABM-1/2/3). Required for a dot to actually run any ABM timing.
// Also updates the On/Off shadow - caller must flushOnOff() afterwards.
// Note: an ABM dot always breathes 0 -> full intensity; its PWM register does not scale the peak.
static void assignDotAbm(uint16_t gdot, uint8_t mode /*0-3*/) {
  if (gdot >= numDots() || mode > 3) return;
  uint8_t panel, dot, sw, cs;
  splitDot(gdot, panel, dot);
  dotToSwCs(dot, sw, cs);
  g_abmAssign[panel][dot] = mode;
  selectChip(panel);
  writePageReg(PAGE_ABM, abmAddrForDot(sw, cs), mode & 0x03);
  syncDotOnOff(panel, dot);
}

// Count dots assigned to an ABM slot whose PWM is still 0 (they still breathe, but keep a non-zero PWM
// anyway so the On/Off shadow and 'dump' show them as lit).
static uint16_t countDarkAbmDots() {
  uint16_t n = 0;
  for (uint8_t p = 0; p < g_numPanels; p++) {
    for (uint8_t d = 0; d < NUM_DOTS; d++) {
      if (g_abmAssign[p][d] != 0 && g_pwm[p][d] == 0) n++;
    }
  }
  return n;
}

static void warnDarkAbmDots() {
  uint16_t n = countDarkAbmDots();
  if (n == 0) return;
  Serial.print(F("warning: "));
  Serial.print(n);
  Serial.println(F(" ABM dot(s) have PWM=0 - they still breathe to full intensity; assign 0 to keep a channel dark"));
}

// Program one ABM slot's timing on every panel and commit it per the
// datasheet's Figure 16 flow: write 02h~0Dh -> clear B_EN -> set B_EN ->
// write 0Eh=0x00. Each step is done on all panels before the next, so the
// panels start their breath cycle as close together as the bus allows.
static const uint16_t MAX_ABM_LOOPS = 4095; // 12-bit LTA:LTB

// Loop characters (Table 17/18):
//   startT (1-4) -> LB = startT-1, the phase the loop begins at
//   endOn        -> LE = 01 (end at on state, end of T1) else 00 (end at off, end of T3)
//   loops        -> LTA:LTB = loop count, 0 = endless
static void defineAbm(uint8_t n /*1-3*/, uint8_t T1, uint8_t T2, uint8_t T3, uint8_t T4,
                      uint8_t startT /*1-4*/, bool endOn, uint16_t loops) {
  if (n < 1 || n > 3 || startT < 1 || startT > 4 || loops > MAX_ABM_LOOPS) return;
  uint8_t base = (n == 1) ? FN_ABM1_BASE : (n == 2) ? FN_ABM2_BASE : FN_ABM3_BASE;

  uint8_t regs[4];
  regs[0] = (uint8_t)(((T1 & 0x07) << 5) | ((T2 & 0x0F) << 1)); // fade-in / hold
  regs[1] = (uint8_t)(((T3 & 0x07) << 5) | ((T4 & 0x0F) << 1)); // fade-out / off
  regs[2] = (uint8_t)(((endOn ? 1 : 0) << 6) |          // LE
                      (((startT - 1) & 0x03) << 4) |    // LB
                      ((loops >> 8) & 0x0F));           // LTA, loop count bits 8-11
  regs[3] = (uint8_t)(loops & 0xFF);                    // LTB, loop count bits 0-7

  for (uint8_t p = 0; p < g_numPanels; p++) {
    selectChip(p);
    writePageBlock(PAGE_FUNC, base, regs, sizeof(regs));
  }

  updateConfigBit(CFG_BEN_BIT, false); // clear B_EN
  updateConfigBit(CFG_BEN_BIT, true);  // set B_EN
  writeFuncAll(FN_TIME_UPD, 0x00);     // commit 02h~0Dh
}

// ---------------------------------------------------------------------------
// Command parsing helpers
// ---------------------------------------------------------------------------

// Enough for a full 'load abm' on 3 panels (96 LEDs x 3 channels + 2).
static const uint16_t MAX_TOKENS = 300;

// Split on space, tab and comma. Empty tokens are skipped.
static uint16_t tokenize(char *line, char *tokens[], uint16_t maxTokens) {
  uint16_t count = 0;
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

static bool parseU16(const char *s, uint16_t &out) {
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s || v < 0 || v > 65535) return false;
  out = (uint16_t)v;
  return true;
}

// Parses a 6-hex-digit RRGGBB token, e.g. "0a0b30" or "Fa7bC3".
static bool parseHexTriplet(const char *s, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (strlen(s) != 6) return false;
  char *end;
  long value = strtol(s, &end, 16);
  if (end != s + 6 || value < 0) return false;
  r = (uint8_t)((value >> 16) & 0xFF);
  g = (uint8_t)((value >> 8) & 0xFF);
  b = (uint8_t)(value & 0xFF);
  return true;
}

// ---------------------------------------------------------------------------
// PWM wave: a smooth travelling bump generated here and written as PWM frames
// ---------------------------------------------------------------------------
// ABM only has 3 timers (3 phases along the travel direction) and each dot can only fade 0 -> full, so it
// cannot draw a smooth bump. This mode instead computes every LED's brightness each frame and writes the
// changed PWM rows, so any width, speed and RGB colour works. The host (REPL now, DMX later) only supplies a
// few parameters. Nothing here ever reports an error: bad values fall back to defaults or are clamped.

static const float GAMMA_EXP = 2.2f;   // PWM = 255 * (intensity/255)^GAMMA_EXP, so steps look even to the eye

// rainbow-bump: hue runs red -> violet (this fraction of the colour wheel) across one bump, so its two ends
// differ. rainbow: hue runs round the whole wheel (red -> red) over one width.
static const float RAINBOW_SPAN = 5.0f / 6.0f;
// Rainbow colours are mixed directly in PWM values (LED light is proportional to PWM, so a hue ramp is a linear
// ramp in PWM; gamma would squash the minor channel and leave red/orange dominating the wheel). The green LEDs
// are much brighter than the red and blue ones, so each channel's PWM at full hue is scaled to this peak, the
// same calibration the ABM rainbow used (ff30a0), which keeps yellow, cyan and violet balanced.
static const uint8_t RAINBOW_PEAK[3] = {255, 48, 160};   // R, G, B PWM at full channel
// How far (as a fraction of the wheel) each primary stays at full strength before it starts to fade, measured
// either side of its centre; it reaches zero 1/3 of the wheel away. The standard HSV wheel holds each primary
// for 1/6 either side (a 1/3-wide plateau, which makes red look stuck because it also leads into both
// neighbours); a shorter plateau gives the colours between the primaries room, so each hue lasts about equally.
static const float RAINBOW_PLATEAU = 1.0f / 9.0f;

struct WaveState {
  bool active = false;
  uint8_t rainbow = 0;                      // 0 = colour pattern, 1 = continuous rainbow, 2 = rainbow in the bump
  uint8_t r[MAX_PANELS * LEDS_PER_PANEL];   // per-LED colour, the pattern already tiled
  uint8_t g[MAX_PANELS * LEDS_PER_PANEL];
  uint8_t b[MAX_PANELS * LEDS_PER_PANEL];
  float width = 8.0f;                       // LEDs in one whole bump: rise + plateau + fall
  float plateau = 0.0f;                     // LEDs held at full intensity inside the bump
  float gap = 0.0f;                         // dark LEDs between one bump and the next
  float speed = 4.0f;                       // LEDs per second along the direction
  int8_t ax = 1, ay = 0;                    // travel direction in display coordinates (x right, y up)
  uint8_t sharp = 1;                        // bump exponent: higher = narrower bright core
  uint16_t frameMs = 33;
  uint32_t startMs = 0, lastMs = 0;
  uint32_t frames = 0, lastFrameUs = 0;
};
static WaveState g_wave;
static uint8_t g_edge[256];                 // raised-cosine rise 0..255 (the fall is its mirror)
static uint8_t g_gamma[256];                // linear intensity -> PWM
static uint8_t g_hue[256][3];               // position in the bump (0..255) -> red..violet R,G,B PWM
static uint8_t g_wheel[256][3];             // position in the cycle (0..255) -> full-wheel R,G,B PWM

// Hue (0..1 round the whole wheel) -> R,G,B PWM values scaled to RAINBOW_PEAK. Red is centred on 0, green on
// 1/3 and blue on 2/3; each is full within RAINBOW_PLATEAU of its centre and fades linearly to 0 at 1/3.
static void hueToRgb(float hue, uint8_t out[3]) {
  const float fade = 1.0f / 3.0f - RAINBOW_PLATEAU;
  for (uint8_t c = 0; c < 3; c++) {
    float dh = fabsf(hue - (float)c / 3.0f);
    if (dh > 0.5f) dh = 1.0f - dh;                           // distance round the wheel
    float v = (1.0f / 3.0f - dh) / fade;
    v = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    out[c] = (uint8_t)(v * (float)RAINBOW_PEAK[c] + 0.5f);
  }
}

static void buildWaveTables(uint8_t sharp) {
  for (uint16_t i = 0; i < 256; i++) {
    float c = 0.5f * (1.0f - cosf(3.14159265f * (float)i / 255.0f));   // 0 -> 1 across the edge
    float v = c;
    for (uint8_t k = 1; k < sharp; k++) v *= c;
    g_edge[i] = (uint8_t)(v * 255.0f + 0.5f);
    g_gamma[i] = (uint8_t)(powf((float)i / 255.0f, GAMMA_EXP) * 255.0f + 0.5f);

    hueToRgb((float)i / 256.0f * RAINBOW_SPAN, g_hue[i]);   // i/256 so a full cycle doesn't hit red twice
    hueToRgb((float)i / 256.0f, g_wheel[i]);
  }
}

// PWM shadow update without the per-dot I2C write; flushPwmRows() writes whole rows.
static void setDotPwmShadow(uint16_t gdot, uint8_t value) {
  uint8_t panel, dot;
  splitDot(gdot, panel, dot);
  g_pwm[panel][dot] = value;
}

// Write the PWM shadow of one panel, one SW row (15 bytes: CS1..CS8 at stride 2) per block write, and only
// the rows that differ from what was last sent (prev).
static void flushPwmRows(uint8_t panel, const uint8_t *prev) {
  selectChip(panel);
  if (!selectPage(PAGE_PWM)) return;
  uint8_t row[2 * NUM_CS - 1];
  for (uint8_t sw = 0; sw < NUM_SW; sw++) {
    const uint8_t *cur = &g_pwm[panel][sw * NUM_CS];
    if (memcmp(cur, prev + sw * NUM_CS, NUM_CS) == 0) continue;
    memset(row, 0, sizeof(row));
    for (uint8_t cs = 0; cs < NUM_CS; cs++) row[cs * 2] = cur[cs];
    i2cWriteBlock((uint8_t)(sw * 0x10), row, sizeof(row));
  }
}

static void waveRender() {
  uint32_t t0 = micros();
  uint8_t prev[MAX_PANELS][NUM_DOTS];
  memcpy(prev, g_pwm, sizeof(prev));

  float shift = g_wave.speed * (float)(millis() - g_wave.startMs) * 0.001f;
  float period = g_wave.width + g_wave.gap;
  float edge = (g_wave.width - g_wave.plateau) * 0.5f;      // LEDs in the rise, and in the fall
  uint8_t n = numLeds();
  for (uint8_t led = 0; led < n; led++) {
    int16_t x = led % NUM_CS, y = led / NUM_CS;
    float d = ((float)(g_wave.ax * x + g_wave.ay * y) - shift) / period;
    d = (d - floorf(d)) * period;                            // LEDs into this bump, 0 .. period
    uint8_t level = 0;
    if (g_wave.rainbow == 1) {
      if (d < g_wave.width) level = 255;                     // continuous rainbow: full brightness, no fade
    } else if (d < g_wave.width) {
      if (edge < 0.001f)               level = 255;                          // no fade: a hard block
      else if (d < edge)               level = g_edge[(uint8_t)(d / edge * 255.0f)];
      else if (d < edge + g_wave.plateau) level = 255;
      else                             level = g_edge[(uint8_t)((g_wave.width - d) / edge * 255.0f)];
    }
    uint8_t pr = 0, pg = 0, pb = 0;
    if (!g_wave.rainbow) {                                   // colour pattern: scale, then gamma
      pr = g_gamma[(uint16_t)level * g_wave.r[led] / 255];
      pg = g_gamma[(uint16_t)level * g_wave.g[led] / 255];
      pb = g_gamma[(uint16_t)level * g_wave.b[led] / 255];
    } else if (level) {                                      // hue follows the position inside the cycle / bump
      uint16_t pos = (uint16_t)(d / g_wave.width * 256.0f);
      if (pos > 255) pos = 255;
      const uint8_t *h = (g_wave.rainbow == 1) ? g_wheel[pos] : g_hue[pos];
      uint16_t gl = g_gamma[level];                          // brightness (fade of the bump) in PWM terms
      pr = (uint8_t)(h[0] * gl / 255);
      pg = (uint8_t)(h[1] * gl / 255);
      pb = (uint8_t)(h[2] * gl / 255);
    }
    setDotPwmShadow(rgbDot(led, 0), pr);
    setDotPwmShadow(rgbDot(led, 1), pg);
    setDotPwmShadow(rgbDot(led, 2), pb);
  }
  for (uint8_t p = 0; p < g_numPanels; p++) flushPwmRows(p, prev[p]);
  g_wave.frames++;
  g_wave.lastFrameUs = micros() - t0;
}

// Stop the wave and leave the LEDs dark with the shadows consistent, so other commands start clean.
static void stopWave() {
  if (!g_wave.active) return;
  g_wave.active = false;
  uint8_t prev[MAX_PANELS][NUM_DOTS];
  memcpy(prev, g_pwm, sizeof(prev));
  memset(g_pwm, 0, sizeof(g_pwm));
  for (uint8_t p = 0; p < g_numPanels; p++) flushPwmRows(p, prev[p]);
  memset(g_onoff, 0, sizeof(g_onoff));
  flushOnOff();
}

static void startWave() {
  setGlobalMode(false);                       // B_EN=0: PWM mode
  uint8_t zeros[2 * NUM_CS - 1];
  memset(zeros, 0, sizeof(zeros));
  for (uint8_t p = 0; p < g_numPanels; p++) {  // no dot may stay on an ABM timer
    selectChip(p);
    if (!selectPage(PAGE_ABM)) continue;
    for (uint8_t sw = 0; sw < NUM_SW; sw++) i2cWriteBlock((uint8_t)(sw * 0x10), zeros, sizeof(zeros));
  }
  memset(g_abmAssign, 0, sizeof(g_abmAssign));
  for (uint8_t p = 0; p < g_numPanels; p++) memset(g_onoff[p], 0xFF, sizeof(g_onoff[p]));
  flushOnOff();                               // every dot on; zero PWM keeps it dark
  g_wave.frames = 0;
  g_wave.startMs = g_wave.lastMs = millis();
  g_wave.active = true;
}

static bool isWaveKeyword(const char *s) {
  return !strcmp(s, "width") || !strcmp(s, "plateau") || !strcmp(s, "gap") || !strcmp(s, "speed") || !strcmp(s, "dir") || !strcmp(s, "sharp") ||
         !strcmp(s, "fps") || !strcmp(s, "off");
}

static bool equalsNoCase(const char *s, const char *word) {
  for (; *word; s++, word++)
    if (tolower((unsigned char)*s) != *word) return false;
  return *s == '\0';
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void cmdWave(uint16_t argc, char *argv[]) {
  if (argc >= 2 && strcmp(argv[1], "off") == 0) {
    stopWave();
    Serial.println(F("OK wave off"));
    return;
  }
  // Colours come first; anything that is not a valid RRGGBB is shown as black.
  static uint8_t cr[MAX_TOKENS], cg[MAX_TOKENS], cb[MAX_TOKENS];
  uint16_t nCol = 0, i = 1;
  uint8_t rainbow = 0;
  for (; i < argc && !isWaveKeyword(argv[i]); i++) {
    if (equalsNoCase(argv[i], "rainbow"))      { rainbow = 1; continue; }   // wins over any colours given
    if (equalsNoCase(argv[i], "rainbow-bump")) { rainbow = 2; continue; }
    uint8_t r = 0, g = 0, b = 0;
    if (!parseHexTriplet(argv[i], r, g, b)) r = g = b = 0;
    cr[nCol] = r; cg[nCol] = g; cb[nCol] = b;
    nCol++;
  }
  if (nCol == 0) { cr[0] = cg[0] = cb[0] = 255; nCol = 1; }   // no colour given: white

  float width = 8.0f, plateau = 0.0f, gap = 0.0f, speed = 4.0f;
  int8_t ax = 1, ay = 0;
  uint8_t sharp = 1;
  uint16_t fps = 30;
  while (i < argc) {
    const char *key = argv[i];
    const char *val = (i + 1 < argc) ? argv[i + 1] : nullptr;
    if (!isWaveKeyword(key) || val == nullptr || !strcmp(key, "off")) { i++; continue; }   // unknown word: skip
    if (!strcmp(key, "dir")) {
      static const struct { const char *n; int8_t x, y; } DIRS[] = {
        {"right", 1, 0}, {"left", -1, 0}, {"up", 0, 1}, {"down", 0, -1},
        {"bl-tr", 1, 1}, {"br-tl", -1, 1}, {"tl-br", 1, -1}, {"tr-bl", -1, -1}};
      for (const auto &d : DIRS) if (!strcmp(val, d.n)) { ax = d.x; ay = d.y; }
    } else {
      char *end;
      float v = strtof(val, &end);
      if (end != val) {   // not a number: keep the default
        if (!strcmp(key, "width"))      width = clampf(v, 1.0f, 64.0f);
        else if (!strcmp(key, "plateau")) plateau = clampf(v, 0.0f, 64.0f);
        else if (!strcmp(key, "gap"))   gap = clampf(v, 0.0f, 64.0f);
        else if (!strcmp(key, "speed")) speed = clampf(v, -60.0f, 60.0f);
        else if (!strcmp(key, "sharp")) sharp = (uint8_t)clampf(v, 1.0f, 8.0f);
        else if (!strcmp(key, "fps"))   fps = (uint16_t)clampf(v, 5.0f, 60.0f);
      }
    }
    i += 2;
  }

  // Tile the colour pattern over the LEDs in display order (short = repeat, long = truncate).
  for (uint16_t led = 0; led < (uint16_t)numLeds(); led++) {
    g_wave.r[led] = cr[led % nCol];
    g_wave.g[led] = cg[led % nCol];
    g_wave.b[led] = cb[led % nCol];
  }
  if (plateau > width) plateau = width;                       // the hold can't be longer than the bump
  g_wave.rainbow = rainbow;
  g_wave.width = width; g_wave.plateau = plateau; g_wave.gap = gap; g_wave.speed = speed; g_wave.ax = ax; g_wave.ay = ay;
  g_wave.sharp = sharp; g_wave.frameMs = (uint16_t)(1000 / fps);
  buildWaveTables(sharp);
  if (!g_wave.active) startWave();
  g_wave.startMs = millis();
  Serial.print(F("OK wave "));
  if (rainbow) Serial.print(rainbow == 1 ? F("rainbow") : F("rainbow-bump")); else { Serial.print(nCol); Serial.print(F(" colour(s)")); }
  Serial.print(F(", width ")); Serial.print(width);
  Serial.print(F(" plateau ")); Serial.print(plateau); Serial.print(F(" gap ")); Serial.print(gap);
  Serial.print(F(" speed ")); Serial.print(speed); Serial.print(F(" fps ")); Serial.println(fps);
}

// ---------------------------------------------------------------------------
// Command implementations
// ---------------------------------------------------------------------------

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  panel [<n 1-3>]                    (panels incl. master, stacked in Y; resets all)"));
  Serial.println(F("  mode pwm"));
  Serial.println(F("  mode abm"));
  Serial.println(F("  load pwm RRGGBB RRGGBB ...         (one hex triplet per LED, LED 0 upward)"));
  Serial.println(F("  load abm mR mG mB mR mG mB ...     (one mode per R,G,B channel, LED 0 upward, 0=PWM, 1-3=ABM-1..3)"));
  Serial.println(F("  fill pwm from <led> to <led> with RRGGBB ...   (pattern repeats, truncated at end)"));
  Serial.println(F("  fill abm from <led> to <led> with m m m ...    (one mode per R,G,B channel, repeats)"));
  Serial.println(F("  assign abm <n 0-3> <dot...>        (same, but by explicit dot index, panel*96 + dot)"));
  Serial.println(F("  define abm <n 1-3> <T1> <T2> <T3> <T4> [start <1-4>] [end on|off] [loop <0-4095>]"));
  Serial.println(F("                                     (T codes: datasheet Table 15/16; start = begin at T1..T4,"));
  Serial.println(F("                                      default T1; end = finish on or off, default off;"));
  Serial.println(F("                                      loop = repeat count, default 0 = endless)"));
  Serial.println(F("  gcc <0-255>"));
  Serial.println(F("  reset"));
  Serial.println(F("  wave <RRGGBB ...|rainbow|rainbow-bump> [width <leds>] [plateau <leds>] [gap <leds>] [speed <leds/s>]"));
  Serial.println(F("       [dir right|left|up|down|bl-tr|br-tl|tl-br|tr-bl]"));
  Serial.println(F("       [sharp <1-8>] [fps <5-60>]   (smooth PWM wave made on the Teensy; 'wave off' stops it)"));
  Serial.println(F("  dump"));
}

static void printPanelSize() {
  Serial.print(g_numPanels);
  Serial.print(F(" panel(s), 8x"));
  Serial.print(g_numPanels * 4);
  Serial.print(F(" RGB LEDs (0-"));
  Serial.print(numLeds() - 1);
  Serial.println(')');
}

static void cmdPanel(uint16_t argc, char *argv[]) {
  if (argc < 2) {
    printPanelSize();
    return;
  }
  uint8_t n;
  if (!parseByte(argv[1], n) || n < 1 || n > MAX_PANELS) {
    Serial.println(F("usage: panel <1-3>"));
    return;
  }
  g_numPanels = n;
  chipReset();
  Serial.print(F("OK "));
  printPanelSize();
  Serial.println(F("all panels reset"));
  for (uint8_t p = 0; p < g_numPanels; p++) {
    if (!chipPresent(p)) {
      Serial.print(F("warning: panel "));
      Serial.print(p);
      Serial.print(F(" not answering at 0x"));
      Serial.println(PANEL_ADDR[p], HEX);
    }
  }
}

static void cmdMode(uint16_t argc, char *argv[]) {
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
// LEDs are filled X first (see rgbDot).
static void cmdLoadPwm(uint16_t argc, char *argv[]) {
  uint8_t led = 0;
  for (uint16_t i = 2; i < argc; i++, led++) {
    if (led >= numLeds()) {
      Serial.println(F("ran out of dots, stopping"));
      break;
    }
    uint8_t r, g, b;
    if (!parseHexTriplet(argv[i], r, g, b)) {
      Serial.print(F("bad RRGGBB value: "));
      Serial.println(argv[i]);
      return;
    }
    setDotPwm(rgbDot(led, 0), r);
    setDotPwm(rgbDot(led, 1), g);
    setDotPwm(rgbDot(led, 2), b);
  }
  flushOnOff();
  Serial.print(F("OK loaded "));
  Serial.print(led);
  Serial.println(F(" LED(s)"));
}

// Assign one ABM mode per colour channel, R,G,B of LED 0, then R,G,B of LED 1,
// and so on, LEDs filled X first (see rgbDot) - e.g.
//   load abm 1 2 3 3 2 1
// gives LED (0,0) R=ABM-1 G=ABM-2 B=ABM-3 and LED (1,0) R=ABM-3 G=ABM-2 B=ABM-1.
// 0 puts a channel back under plain PWM control.
static void cmdLoadAbm(uint16_t argc, char *argv[]) {
  uint16_t n = 0; // channel index: led = n/3, channel = n%3
  for (uint16_t i = 2; i < argc; i++, n++) {
    if (n >= (uint16_t)numLeds() * 3) {
      Serial.println(F("ran out of dots, stopping"));
      break;
    }
    uint8_t mode;
    if (!parseByte(argv[i], mode) || mode > 3) {
      Serial.print(F("bad mode (0-3): "));
      Serial.println(argv[i]);
      break; // still flush what was assigned so far
    }
    assignDotAbm(rgbDot((uint8_t)(n / 3), (uint8_t)(n % 3)), mode);
  }
  flushOnOff();
  Serial.print(F("OK assigned "));
  Serial.print(n);
  Serial.println(F(" abm channel(s)"));
  warnDarkAbmDots();
}

static void cmdLoad(uint16_t argc, char *argv[]) {
  if (argc < 2) {
    Serial.println(F("usage: load pwm RRGGBB...  |  load abm mR mG mB ..."));
    return;
  }
  if (strcmp(argv[1], "pwm") == 0) {
    cmdLoadPwm(argc, argv);
  } else if (strcmp(argv[1], "abm") == 0) {
    cmdLoadAbm(argc, argv);
  } else {
    Serial.println(F("usage: load pwm RRGGBB...  |  load abm mR mG mB ..."));
  }
}

// fill pwm from <start> to <end> with RRGGBB RRGGBB ...
// fill abm from <start> to <end> with m m m ...
// <start>/<end> are RGB LED indices (X first across all panels, see rgbDot),
// inclusive. The pattern repeats from <start> and is truncated at <end>. For
// pwm each pattern entry is one whole RGB LED; for abm each entry is one R, G
// or B channel, taken in R,G,B order across the LEDs in the range.
static void cmdFill(uint16_t argc, char *argv[]) {
  static const char USAGE[] =
      "usage: fill pwm|abm from <led> to <led> with <pattern...>";
  if (argc < 8 || strcmp(argv[2], "from") != 0 || strcmp(argv[4], "to") != 0 ||
      strcmp(argv[6], "with") != 0) {
    Serial.println(USAGE);
    return;
  }
  bool isPwm = strcmp(argv[1], "pwm") == 0;
  if (!isPwm && strcmp(argv[1], "abm") != 0) {
    Serial.println(USAGE);
    return;
  }
  uint8_t start, end;
  if (!parseByte(argv[3], start) || !parseByte(argv[5], end) ||
      start >= numLeds() || end >= numLeds() || start > end) {
    Serial.print(F("start/end must be LED 0-"));
    Serial.print(numLeds() - 1);
    Serial.println(F(" with start <= end"));
    return;
  }

  char **pattern = &argv[7];
  uint16_t patLen = (uint16_t)(argc - 7);

  // Validate the whole pattern before touching the chip.
  for (uint16_t p = 0; p < patLen; p++) {
    uint8_t r, g, b, mode;
    bool ok = isPwm ? parseHexTriplet(pattern[p], r, g, b)
                    : (parseByte(pattern[p], mode) && mode <= 3);
    if (!ok) {
      Serial.print(isPwm ? F("bad RRGGBB value: ") : F("bad mode (0-3): "));
      Serial.println(pattern[p]);
      return;
    }
  }

  uint8_t count = (uint8_t)(end - start + 1);
  if (isPwm) {
    for (uint8_t i = 0; i < count; i++) {
      uint8_t r, g, b;
      parseHexTriplet(pattern[i % patLen], r, g, b);
      uint8_t led = (uint8_t)(start + i);
      setDotPwm(rgbDot(led, 0), r);
      setDotPwm(rgbDot(led, 1), g);
      setDotPwm(rgbDot(led, 2), b);
    }
  } else {
    uint16_t numChannels = (uint16_t)(count * 3);
    for (uint16_t i = 0; i < numChannels; i++) {
      uint8_t mode;
      parseByte(pattern[i % patLen], mode);
      assignDotAbm(rgbDot((uint8_t)(start + i / 3), (uint8_t)(i % 3)), mode);
    }
  }
  flushOnOff();

  Serial.print(F("OK filled LED "));
  Serial.print(start);
  Serial.print(F(".."));
  Serial.println(end);
  if (!isPwm) warnDarkAbmDots();
}

static void cmdAssign(uint16_t argc, char *argv[]) {
  if (argc < 3 || strcmp(argv[1], "abm") != 0) {
    Serial.println(F("usage: assign abm <n 0-3> <dot...>"));
    return;
  }
  uint8_t mode;
  if (!parseByte(argv[2], mode) || mode > 3) {
    Serial.println(F("mode must be 0-3"));
    return;
  }
  for (uint16_t i = 3; i < argc; i++) {
    uint16_t dot;
    if (!parseU16(argv[i], dot) || dot >= numDots()) {
      Serial.print(F("bad dot index: "));
      Serial.println(argv[i]);
      continue;
    }
    assignDotAbm(dot, mode);
  }
  flushOnOff();
  Serial.println(F("OK"));
  warnDarkAbmDots();
}

// define abm <n> <T1> <T2> <T3> <T4> [start <1-4>] [end on|off] [loop <0-4095>]
// The optional keyword pairs may come in any order.
static void cmdDefineAbm(uint16_t argc, char *argv[]) {
  static const char USAGE[] =
      "usage: define abm <n 1-3> <T1> <T2> <T3> <T4> [start <1-4>] [end on|off] [loop <0-4095>]";
  if (argc < 7 || (argc - 7) % 2 != 0 || strcmp(argv[1], "abm") != 0) {
    Serial.println(USAGE);
    return;
  }
  uint8_t n, t1, t2, t3, t4;
  if (!parseByte(argv[2], n) || n < 1 || n > 3 ||
      !parseByte(argv[3], t1) || !parseByte(argv[4], t2) ||
      !parseByte(argv[5], t3) || !parseByte(argv[6], t4)) {
    Serial.println(USAGE);
    return;
  }

  uint8_t startT = 1;
  bool endOn = false;
  uint16_t loops = 0;
  for (uint16_t i = 7; i < argc; i += 2) {
    const char *key = argv[i];
    const char *val = argv[i + 1];
    bool ok;
    if (strcmp(key, "start") == 0) {
      ok = parseByte(val, startT) && startT >= 1 && startT <= 4;
    } else if (strcmp(key, "end") == 0) {
      ok = true;
      if (strcmp(val, "on") == 0)       endOn = true;
      else if (strcmp(val, "off") == 0) endOn = false;
      else                              ok = false;
    } else if (strcmp(key, "loop") == 0) {
      ok = parseU16(val, loops) && loops <= MAX_ABM_LOOPS;
    } else {
      ok = false;
    }
    if (!ok) {
      Serial.print(F("bad option: "));
      Serial.print(key);
      Serial.print(' ');
      Serial.println(val);
      Serial.println(USAGE);
      return;
    }
  }

  defineAbm(n, t1, t2, t3, t4, startT, endOn, loops);
  Serial.print(F("OK ABM-"));
  Serial.print(n);
  Serial.print(F(" start T"));
  Serial.print(startT);
  Serial.print(endOn ? F(" end on") : F(" end off"));
  Serial.print(F(" loop "));
  if (loops == 0) Serial.println(F("endless"));
  else            Serial.println(loops);
}

static void cmdGcc(uint16_t argc, char *argv[]) {
  if (argc < 2) { Serial.println(F("usage: gcc <0-255>")); return; }
  uint8_t value;
  if (!parseByte(argv[1], value)) { Serial.println(F("bad value")); return; }
  setGcc(value);
  Serial.print(F("OK gcc="));
  Serial.println(value);
}

static void cmdDump() {
  printPanelSize();
  Serial.print(F("gcc=")); Serial.println(g_gcc);
  for (uint8_t p = 0; p < g_numPanels; p++) {
    Serial.print(F("panel ")); Serial.print(p);
    Serial.print(F(" @0x")); Serial.print(PANEL_ADDR[p], HEX);
    Serial.print(F(" config=0x")); Serial.println(g_config[p], HEX);
  }
  if (g_wave.active) {
    Serial.print(F("wave: active, frames=")); Serial.print(g_wave.frames);
    Serial.print(F(" lastFrameUs=")); Serial.println(g_wave.lastFrameUs);
  }
  Serial.println(F("pwm (dot:value), non-zero only:"));
  for (uint8_t p = 0; p < g_numPanels; p++) {
    for (uint8_t d = 0; d < NUM_DOTS; d++) {
      if (g_pwm[p][d] != 0) {
        Serial.print(p * NUM_DOTS + d); Serial.print(':'); Serial.print(g_pwm[p][d]); Serial.print(' ');
      }
    }
  }
  Serial.println();
  Serial.println(F("abm assignment (dot:mode), non-zero only:"));
  for (uint8_t p = 0; p < g_numPanels; p++) {
    for (uint8_t d = 0; d < NUM_DOTS; d++) {
      if (g_abmAssign[p][d] != 0) {
        Serial.print(p * NUM_DOTS + d); Serial.print(':'); Serial.print(g_abmAssign[p][d]); Serial.print(' ');
      }
    }
  }
  Serial.println();
}

static void handleLine(char *line) {
  static char *argv[MAX_TOKENS];
  uint16_t argc = tokenize(line, argv, MAX_TOKENS);
  if (argc == 0) return;

  // Anything that rewrites the LEDs or the chip mode takes over from a running wave.
  static const char *const TAKEOVER[] = {"panel", "mode", "load", "fill", "assign", "define", "reset"};
  for (const char *c : TAKEOVER) if (strcmp(argv[0], c) == 0) { stopWave(); break; }

  if (strcmp(argv[0], "help") == 0) {
    printHelp();
  } else if (strcmp(argv[0], "panel") == 0) {
    cmdPanel(argc, argv);
  } else if (strcmp(argv[0], "mode") == 0) {
    cmdMode(argc, argv);
  } else if (strcmp(argv[0], "load") == 0) {
    cmdLoad(argc, argv);
  } else if (strcmp(argv[0], "fill") == 0) {
    cmdFill(argc, argv);
  } else if (strcmp(argv[0], "assign") == 0) {
    cmdAssign(argc, argv);
  } else if (strcmp(argv[0], "define") == 0) {
    cmdDefineAbm(argc, argv);
  } else if (strcmp(argv[0], "gcc") == 0) {
    cmdGcc(argc, argv);
  } else if (strcmp(argv[0], "reset") == 0) {
    chipReset();
    Serial.println(F("OK reset"));
  } else if (strcmp(argv[0], "wave") == 0) {
    cmdWave(argc, argv);
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

// Enough for a full 'load pwm' on 3 panels (96 x "RRGGBB ").
static char g_lineBuf[1024];
static uint16_t g_lineLen = 0;

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

static void waveTick() {
  if (!g_wave.active) return;
  uint32_t now = millis();
  if (now - g_wave.lastMs < g_wave.frameMs) return;
  g_wave.lastMs = now;     // a slow frame just delays the next one; frames are never queued
  waveRender();
}

void loop() {
  pollSerial();
  waveTick();
}

/*
 * nPM1300 + Nordic nRF Fuel Gauge test with persistence and a low-battery
 * trigger (XIAO nRF54LM20A). PlatformIO: src/main.cpp. UNTESTED draft.
 *
 * Reads battery voltage and current from the nPM1300, feeds them to Nordic's
 * fuel gauge library once a second and prints the state of charge.
 *
 * Persistence: a small record (state of health, cycle count, charge since the
 * last full cycle) is kept in the core's EEPROM library (CRC-protected RRAM).
 * It is written only when it has changed meaningfully, not on every wake.
 *
 * Low battery: when running on battery with a low state of charge or voltage
 * for several readings in a row, the record is saved once.
 *
 * Needs:
 *  - lib/nrf_fuel_gauge with libnrf_fuel_gauge.a (cortex-m33, soft-float,
 *    secondary cell variant).
 *  - src/battery_model.inc: the LP503030 model.
 *  - npm1300_read_ibat_signed_ma() in the nPM1300 driver.
 *
 * Assumptions to check:
 *  - Current sign: charging positive, discharging negative (CURRENT_SIGN).
 *  - No battery NTC: a fixed temperature is used.
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include "npm1300.h"
#include "nrf_fuel_gauge.h"

#define OutputSerial Serial1   // change to Serial for USB output

static const struct battery_model battery_model = {
#include "battery_model.inc"
};

constexpr float    BATTERY_TEMP_C    = 25.0f;   // no NTC: fixed estimate
constexpr float    CURRENT_SIGN      = 1.0f;    // +1: charge positive. -1 to flip.
constexpr uint32_t INTERVAL_MS       = 1000;    // 1 s while active (Nordic's suggestion)
constexpr float    MIN_BATTERY_V     = 2.5f;    // below this, assume no battery is connected
constexpr uint16_t CHARGE_CURRENT_MA = 100;     // about 0.25C for a 400 mAh cell
constexpr uint16_t CHARGE_TERM_MV    = 4200;    // Li-Po termination voltage

// Persistence
constexpr uint32_t SAVE_CHECK_MS   = 10UL * 60UL * 1000UL;  // check for changes every 10 min
constexpr float    SOH_SAVE_STEP   = 0.5f;                  // save if SoH moved this much (%)

// Low battery (only while VBUS is absent)
constexpr float    CRITICAL_SOC    = 5.0f;      // %
constexpr float    CRITICAL_V      = 3.3f;      // V, backstop for a mismatched model
constexpr uint8_t  CRITICAL_COUNT  = 5;         // consecutive readings (about 5 s)
constexpr bool     SHIP_ON_LOW_BATTERY = false; // true: enter ship mode after saving

// ---------------------------------------------------------------------------
// Persistent record
// ---------------------------------------------------------------------------
struct GaugeRecord {
  uint32_t magic;       // 'FGRC'
  uint16_t version;     // bump if this layout changes
  uint16_t cycles;
  float    soh;         // percent
  float    cumMah;      // mAh since the last full cycle
  uint32_t modelHash;   // hash of battery_model.name
};
constexpr uint32_t REC_MAGIC   = 0x43524746UL;
constexpr uint16_t REC_VERSION = 1;

static GaugeRecord savedRec;
static bool        haveSaved = false;

static struct nrf_fuel_gauge_state_info lastInfo;
static bool haveInfo = false;

static uint32_t hashName(const char *s) {            // FNV-1a
  uint32_t h = 2166136261UL;
  while (*s) { h ^= (uint8_t)*s++; h *= 16777619UL; }
  return h;
}

static bool loadRecord(GaugeRecord &r) {
  if (!EEPROM.begin(sizeof(GaugeRecord))) return false;
  uint8_t *p = (uint8_t *)&r;
  for (size_t i = 0; i < sizeof(r); i++) p[i] = EEPROM.read(i);
  return r.magic == REC_MAGIC && r.version == REC_VERSION &&
         r.modelHash == hashName(battery_model.name) &&
         isfinite(r.soh) && r.soh >= 50.0f && r.soh <= 120.0f &&
         isfinite(r.cumMah) && r.cumMah >= 0.0f && r.cycles < 10000;
}

static bool saveRecord(const GaugeRecord &r) {
  const uint8_t *p = (const uint8_t *)&r;
  for (size_t i = 0; i < sizeof(r); i++) EEPROM.write(i, p[i]);   // marks dirty only if a byte changed
  const uint32_t t0 = micros();
  bool ok = EEPROM.commit();
  if (!ok) ok = EEPROM.commit();                                   // one retry: the RRAM lock may be busy
  OutputSerial.print("record save ");
  OutputSerial.print(ok ? "ok" : "FAILED");
  OutputSerial.print(" in ");
  OutputSerial.print(micros() - t0);
  OutputSerial.print(" us: soh=");
  OutputSerial.print(r.soh, 1);
  OutputSerial.print(" cycles=");
  OutputSerial.println(r.cycles);
  return ok;
}

// Saves the record if SoH, cycle count (or, when forced, the partial-cycle
// charge) changed meaningfully since the last save.
static void maybeSaveRecord(bool force) {
  if (!haveInfo) return;
  float soh = 0;
  if (nrf_fuel_gauge_soh_get(&soh) < 0 || !isfinite(soh)) return;
  if (!isfinite(lastInfo.cycle_count_cumulative_mah)) return;

  GaugeRecord r;
  memset(&r, 0, sizeof(r));
  r.magic     = REC_MAGIC;
  r.version   = REC_VERSION;
  r.cycles    = lastInfo.cycle_count;
  r.soh       = soh;
  r.cumMah    = lastInfo.cycle_count_cumulative_mah;
  r.modelHash = hashName(battery_model.name);

  const bool changed = !haveSaved ||
                       r.cycles != savedRec.cycles ||
                       fabsf(r.soh - savedRec.soh) >= SOH_SAVE_STEP ||
                       (force && fabsf(r.cumMah - savedRec.cumMah) >= 1.0f);
  if (!changed) return;
  if (saveRecord(r)) { savedRec = r; haveSaved = true; }
}

// ---------------------------------------------------------------------------
// Charger and library helpers
// ---------------------------------------------------------------------------
// Decodes BCHGCHARGESTATUS (charger block 0x34).
static const char *chargeName(uint8_t s) {
  if (s & 0x40) return "paused (die temp)";
  if (s & 0x02) return "complete";
  if (s & 0x10) return "CV";
  if (s & 0x08) return "CC";
  if (s & 0x04) return "trickle";
  if (s & 0x20) return "recharge needed";
  return "idle";
}

// Tells the library about VBUS and the charger state. The header says this is
// optional, but the library uses it to know when the battery is charging.
static enum nrf_fuel_gauge_charge_state libChargeState(uint8_t s) {
  if (s & 0x02) return NRF_FUEL_GAUGE_CHARGE_STATE_COMPLETE;
  if (s & 0x10) return NRF_FUEL_GAUGE_CHARGE_STATE_CV;
  if (s & 0x08) return NRF_FUEL_GAUGE_CHARGE_STATE_CC;
  if (s & 0x04) return NRF_FUEL_GAUGE_CHARGE_STATE_TRICKLE;
  return NRF_FUEL_GAUGE_CHARGE_STATE_IDLE;
}

static void updateLibraryChargeInfo(uint8_t chg, uint8_t vbus) {
  static int lastVbus  = -1;
  static int lastState = -1;

  const int present = (vbus & NPM1300_VBUS_STATUS_PRESENT) ? 1 : 0;
  if (present != lastVbus) {
    lastVbus = present;
    nrf_fuel_gauge_ext_state_update(present ? NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED
                                            : NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED,
                                    nullptr);
    if (present) {
      union nrf_fuel_gauge_ext_state_info_data d;
      memset(&d, 0, sizeof(d));
      d.charge_current_limit = CHARGE_CURRENT_MA / 1000.0f;   // amperes
      nrf_fuel_gauge_ext_state_update(NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_LIMIT, &d);
    }
  }

  const int state = (int)libChargeState(chg);
  if (state != lastState) {
    lastState = state;
    union nrf_fuel_gauge_ext_state_info_data d;
    memset(&d, 0, sizeof(d));
    d.charge_state = (enum nrf_fuel_gauge_charge_state)state;
    nrf_fuel_gauge_ext_state_update(NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_STATE_CHANGE, &d);
  }
}

// Reads battery voltage [V] and current [A] from one ADC burst.
static bool readBattery(float &v, float &i) {
  const int32_t mv = npm1300_read_vbat_mv();
  int32_t ma = 0;
  if (mv < 0 || !npm1300_read_ibat_signed_ma(&ma)) return false;
  v = mv / 1000.0f;
  i = CURRENT_SIGN * (ma / 1000.0f);
  return true;
}

// ---------------------------------------------------------------------------
static uint32_t lastMs        = 0;
static uint32_t lastSaveCheck = 0;

void setup() {
  OutputSerial.begin(115200);
  delay(1000);

  if (!npm1300_is_present()) {
    OutputSerial.println("nPM1300 not found");
    while (true) delay(1000);
  }

  // The charger does nothing until the host enables it.
  npm1300_charger_set_term_voltage(CHARGE_TERM_MV);
  npm1300_charger_set_current(CHARGE_CURRENT_MA);
  npm1300_charger_enable(true);

  OutputSerial.print("fuel gauge library ");
  OutputSerial.print(nrf_fuel_gauge_version);
  OutputSerial.print(", variant ");
  OutputSerial.println(nrf_fuel_gauge_variant == NRF_FUEL_GAUGE_VARIANT_SECONDARY
                     ? "secondary (rechargeable)" : "primary");

  // Wait for a battery. Initialising the gauge from a no-battery reading
  // would start it from a wrong state. For the most accurate start, this
  // reading should be taken before the radio or LEDs draw current.
  float v = 0, i = 0;
  while (true) {
    if (readBattery(v, i) && v >= MIN_BATTERY_V) break;
    OutputSerial.print("waiting for battery, V=");
    OutputSerial.println(v, 3);
    delay(1000);
  }

  // Optional configuration, restored from the stored record when it is valid.
  static struct nrf_fuel_gauge_config_parameters cfg;   // keep valid while the library is in use
  nrf_fuel_gauge_config_params_default_get(&cfg);
  cfg.soh_estimate_enable = true;

  GaugeRecord rec;
  if (loadRecord(rec)) {
    cfg.soh                        = rec.soh;
    cfg.cycle_count                = rec.cycles;
    cfg.cycle_count_cumulative_mah = rec.cumMah;
    savedRec  = rec;
    haveSaved = true;
    OutputSerial.print("restored record: soh=");
    OutputSerial.print(rec.soh, 1);
    OutputSerial.print(" cycles=");
    OutputSerial.println(rec.cycles);
  } else {
    OutputSerial.println("no valid record, starting with library defaults");
  }

  // Filled in by hand: the init macro uses designated initializers in an
  // order that g++ can reject.
  struct nrf_fuel_gauge_init_parameters p;
  memset(&p, 0, sizeof(p));
  p.v0 = v;
  p.i0 = i;
  p.t0 = BATTERY_TEMP_C;
  p.model = &battery_model;
  p.opt_params = &cfg;
  p.state = nullptr;

  const int ret = nrf_fuel_gauge_init(&p, nullptr);
  OutputSerial.print("nrf_fuel_gauge_init: ");
  OutputSerial.println(ret);
  if (ret < 0) while (true) delay(1000);

  lastMs = lastSaveCheck = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastMs < INTERVAL_MS) { delay(10); return; }

  float v = 0, i = 0, soc = 0;
  if (!readBattery(v, i)) {
    OutputSerial.println("battery read failed");
    lastMs = now;
    return;
  }

  const float dt = (now - lastMs) / 1000.0f;   // seconds since the previous call
  lastMs = now;

  uint8_t chg = 0, vbus = 0;
  const bool haveChg  = npm1300_charger_status(&chg);
  const bool haveVbus = npm1300_vbus_status(&vbus);
  if (haveChg && haveVbus) updateLibraryChargeInfo(chg, vbus);

  const int ret = nrf_fuel_gauge_process(v, i, BATTERY_TEMP_C, dt, &soc, &lastInfo);
  if (ret == 0) haveInfo = true;

  OutputSerial.print("V=");   OutputSerial.print(v, 3);
  OutputSerial.print(" I=");  OutputSerial.print(i * 1000.0f, 1);
  OutputSerial.print(" mA SoC=");
  if (ret == 0) OutputSerial.print(soc, 1); else OutputSerial.print("err");
  OutputSerial.print(" %");

  uint8_t err = 0;
  if (haveChg) {
    OutputSerial.print(" chg=");
    OutputSerial.print(chargeName(chg));
    OutputSerial.print(" (0x");
    OutputSerial.print(chg, HEX);
    OutputSerial.print(")");
  }
  if (npm1300_charger_error(&err) && err != 0) {
    OutputSerial.print(" err=0x");
    OutputSerial.print(err, HEX);
  }
  OutputSerial.println();

  // --- Persistence -------------------------------------------------------
  static bool wasComplete = false;
  const bool isComplete = haveChg && (chg & 0x02);
  if (isComplete && !wasComplete) maybeSaveRecord(true);   // end of a charge
  wasComplete = isComplete;

  if (now - lastSaveCheck >= SAVE_CHECK_MS) {
    lastSaveCheck = now;
    maybeSaveRecord(false);                                 // saves only if it changed
  }

  // --- Low battery (running on battery only) -----------------------------
  static uint8_t lowCount   = 0;
  static bool    lowLatched = false;
  const bool onBattery = haveVbus && !(vbus & NPM1300_VBUS_STATUS_PRESENT);
  const bool low = onBattery && ((ret == 0 && soc <= CRITICAL_SOC) || v <= CRITICAL_V);

  if (low) {
    if (lowCount < 255) lowCount++;
  } else {
    lowCount = 0;
    // re-arm once the battery has recovered or VBUS is back
    if (lowLatched && (!onBattery || (ret == 0 && soc > CRITICAL_SOC + 5.0f && v > CRITICAL_V + 0.1f))) {
      lowLatched = false;
    }
  }

  if (lowCount >= CRITICAL_COUNT && !lowLatched) {
    lowLatched = true;
    OutputSerial.println("CRITICAL BATTERY: saving gauge record");
    maybeSaveRecord(true);
    if (SHIP_ON_LOW_BATTERY) {
      OutputSerial.println("entering ship mode");
      OutputSerial.flush();
      npm1300_enter_ship_mode();    // fails (returns false) if VBUS is present
    }
  }
}
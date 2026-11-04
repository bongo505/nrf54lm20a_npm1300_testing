/*
 * nPM1300 + Nordic nRF Fuel Gauge feasibility test (XIAO nRF54LM20A)
 *
 * UNTESTED first draft (PlatformIO src/main.cpp). Reads battery voltage and current from the nPM1300,
 * feeds them to Nordic's fuel gauge library once a second and prints the
 * state of charge.
 *
 * Needs:
 *  - nrf_fuel_gauge.h and the secondary-cell (rechargeable) library build
 *    linked in (cortex-m33, hard-float).
 *  - battery_model.inc: the LP503030 model, renamed or copied next to this file.
 *  - npm1300_read_ibat_signed_ma() added to the nPM1300 driver (see chat).
 *
 * Assumptions to check:
 *  - Current sign: charging positive, discharging negative. If the state of
 *    charge rises under load and falls on charge, flip CURRENT_SIGN.
 *  - There is no battery NTC read here, so a fixed temperature is used.
 *    The PMIC die temperature is NOT the battery temperature.
 */
#include <Arduino.h>
#include "npm1300.h"
#include "nrf_fuel_gauge.h"

#define OutputSerial Serial1   // change to Serial for USB output

static const struct battery_model battery_model = {
#include "battery_model.inc"
};

constexpr float    BATTERY_TEMP_C   = 25.0f;   // no NTC: fixed estimate
constexpr float    CURRENT_SIGN     = 1.0f;    // +1: charge positive. -1 to flip.
constexpr uint32_t INTERVAL_MS      = 1000;    // 1 s while active (Nordic's suggestion)
constexpr float    MIN_BATTERY_V    = 2.5f;    // below this, assume no battery is connected
constexpr uint16_t CHARGE_CURRENT_MA = 100;     // about 0.25C for a 400 mAh cell
constexpr uint16_t CHARGE_TERM_MV    = 4200;    // Li-Po termination voltage

static uint32_t lastMs = 0;

uint8_t ledFlag = 1;

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

static void blinkBootMarker(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);

  for (int i = 0; i < 5; ++i) {
    digitalWrite(pin, LOW);
    delay(100);
    digitalWrite(pin, HIGH);
    delay(100);
  }
}

void setup() {
  blinkBootMarker(LED_BUILTIN);
  OutputSerial.begin(115200);
  delay(1000);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, ledFlag);
  digitalWrite(LED_RED, !ledFlag);

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
  // would start it from a wrong state.
  float v = 0, i = 0;
  while (true) {
    if (readBattery(v, i) && v >= MIN_BATTERY_V) break;
    OutputSerial.print("waiting for battery, V=");
    OutputSerial.println(v, 3);
    delay(1000);
  }

  // Filled in by hand: the init macro uses designated initializers in an
  // order that g++ can reject.
  struct nrf_fuel_gauge_init_parameters p;
  memset(&p, 0, sizeof(p));
  p.v0 = v;
  p.i0 = i;
  p.t0 = BATTERY_TEMP_C;
  p.model = &battery_model;
  p.opt_params = nullptr;
  p.state = nullptr;

  const int ret = nrf_fuel_gauge_init(&p, nullptr);
  OutputSerial.print("nrf_fuel_gauge_init: ");
  OutputSerial.println(ret);
  if (ret < 0) while (true) delay(1000);

  lastMs = millis();
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
  const bool haveChg = npm1300_charger_status(&chg);
  if (haveChg && npm1300_vbus_status(&vbus)) updateLibraryChargeInfo(chg, vbus);

  const int ret = nrf_fuel_gauge_process(v, i, BATTERY_TEMP_C, dt, &soc, nullptr);

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

  ledFlag = !ledFlag;
  digitalWrite(LED_GREEN, ledFlag);
  digitalWrite(LED_RED, !ledFlag);

}
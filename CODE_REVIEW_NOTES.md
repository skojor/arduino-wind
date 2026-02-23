# Wind node review notes (ATmega328P @ 8 MHz, battery/solar, long-range MySensors)

## High-priority findings

1. **Test pulse injection is active in production loop**
   - `loop()` currently injects random wind pulses by directly calling `wspdISR()`.
   - This will bias all speed/gust calculations and traffic patterns if left enabled.
   - Suggested action: guard this with `#ifdef TEST_MODE` or remove.

2. **Radio TX power is set to max (`20 dBm`)**
   - For a battery/solar node, this can dominate average current.
   - Suggested action: do a link-budget check and step power down (e.g. 17/14/10 dBm) while monitoring delivery reliability.

3. **Temperature conversion is blocking at 12-bit resolution**
   - `DallasTemperature` at 12-bit with `setWaitForConversion(true)` blocks up to ~750 ms.
   - Suggested action: use 10–11 bit resolution and/or async conversion to reduce awake time.

## Medium-priority findings

4. **Direction max calibration only moves upward**
   - `WDIR_ADC_MAX` is increased when a larger sample appears, but never decays.
   - Over time, this can compress mapped direction values.
   - Suggested action: add bounded decay/recalibration strategy.

5. **No explicit error handling for send failures**
   - `send(...)` return value is ignored.
   - Suggested action: use return value to count failed sends and consider retry/backoff policy.

6. **Potentially expensive repeated transport init/disable**
   - `transportReInitialise()` and `transportDisable()` are called around each send.
   - Suggested action: keep as-is if measured beneficial, but profile energy/latency vs. batching when multiple values are due.

## Low-priority tuning ideas

7. **Battery read delay can likely be reduced**
   - Fixed `delay(20)` before ADC reads may be longer than needed depending on divider impedance.
   - Suggested action: empirically lower delay (e.g. 2–5 ms) and verify ADC stability.

8. **Watchdog reset strategy**
   - WDT is enabled for 8 s reset protection.
   - Suggested action: keep it, but add diagnostics counter for reset cause when practical.

## Suggested measurement plan

- Measure average current in 3 modes:
  1. Current firmware (baseline)
  2. Lower TX power + non-blocking temp conversion
  3. Lower TX power + slower telemetry intervals (if acceptable)
- Track packet delivery at gateway and RSSI while tuning TX power.
- Validate wind accuracy after removing test injection and after direction recalibration logic changes.

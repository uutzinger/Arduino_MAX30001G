# Interrupt Handling

The driver keeps default behavior in `serviceAllInterrupts()` (updates global flags), and also supports user callbacks.
For IRQ-driven sketches, prefer `servicePendingInterrupts()` in `loop()`.

## Interrupt Usage

Wire MAX30001G `INTB` (and optional `INT2B`) to interrupt-capable GPIO pins and pass those pins to the constructor.

```cpp
const uint8_t AFE_CS_PIN  = 10;
const int AFE_INT1_PIN = 2;   // INTB
const int AFE_INT2_PIN = -1;  // INT2B optional, -1 if unused

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

void setup() {
  // Constructor attaches ISR handlers for valid interrupt pins.
  // Choose which status events drive INT1/INT2:
  afe.setInterrupt1(true,  true,  true,  false, false);  // ECG + BIOZ + RTOR on INT1
  afe.setInterrupt2(false, false, false, false, false);  // INT2 disabled
}

void loop() {
  if (afe.servicePendingInterrupts()) {
    // STATUS was serviced and global flags/callbacks are up to date.
  }
}
```

`servicePendingInterrupts()` uses Arduino core APIs `noInterrupts()` and `interrupts()` to atomically read/clear the software pending flags. These APIs are available on AVR, SAMD, ESP8266, and ESP32 Arduino cores, so no API change is needed for ESP-type boards.

If your board does not wire interrupt pins, call `serviceAllInterrupts()` periodically instead.
If you provide your own ISR, set library globals `afe_irq_pending` and `afe_irq1_pending`/`afe_irq2_pending` (do not use a separate local pending flag).
If you use latched event/fault globals, clear them explicitly with `clearLatchedStatusFlags()` after handling.

## Configure which events drive INT1/INT2

`setInterrupt1(...)` and `setInterrupt2(...)` program MAX30001G `EN_INT1` and `EN_INT2`.

Example:

```cpp
// INT1: ECG + BIOZ FIFO related interrupts
afe.setInterrupt1(true,  true,  false, false, false);
// INT2: R-to-R interrupt
afe.setInterrupt2(false, false, true,  false, false);
```

## Available per-event callbacks (`InterruptEvent`)

- `MAX30001G::IRQ_ECG_FIFO`    -> `MAX30001_STATUS_EINT`
- `MAX30001G::IRQ_ECG_OVF`     -> `MAX30001_STATUS_EOVF`
- `MAX30001G::IRQ_ECG_FAST`    -> `MAX30001_STATUS_FSTINT`
- `MAX30001G::IRQ_ECG_LEADOFF` -> `MAX30001_STATUS_DCLOFFINT`
- `MAX30001G::IRQ_BIOZ_FIFO`   -> `MAX30001_STATUS_BINT`
- `MAX30001G::IRQ_BIOZ_OVF`    -> `MAX30001_STATUS_BOVF`
- `MAX30001G::IRQ_BIOZ_OVER`   -> `MAX30001_STATUS_BOVER`
- `MAX30001G::IRQ_BIOZ_UNDER`  -> `MAX30001_STATUS_BUNDR`
- `MAX30001G::IRQ_BIOZ_CGMON`  -> `MAX30001_STATUS_BCGMON`
- `MAX30001G::IRQ_LEADS_ON`    -> `MAX30001_STATUS_LONINT`
- `MAX30001G::IRQ_RTOR`        -> `MAX30001_STATUS_RRINT`
- `MAX30001G::IRQ_PLL_UNLOCK`  -> `MAX30001_STATUS_PLLINT`

## Example: Individual ECG and BIOZ FIFO callbacks

```cpp
void onEcgFifo(MAX30001G* dev, uint32_t status, void* ctx) {
  (void)status;
  (void)ctx;
  dev->handleECGFifoInterrupt(false);   // pushes samples into global ECG_data
}

void onBiozFifo(MAX30001G* dev, uint32_t status, void* ctx) {
  (void)status;
  (void)ctx;
  dev->handleBIOZFifoInterrupt(false);  // pushes samples into global BIOZ_data
}

void setup() {
  afe.setInterruptEventCallback(MAX30001G::IRQ_ECG_FIFO,  onEcgFifo);
  afe.setInterruptEventCallback(MAX30001G::IRQ_BIOZ_FIFO, onBiozFifo);
}
```

## Example: One aggregate callback

```cpp
void onAfeAggregate(MAX30001G* dev, uint32_t status, void* ctx) {
  (void)ctx;

  if (status & MAX30001_STATUS_EINT)  { dev->handleECGFifoInterrupt(false); }
  if (status & MAX30001_STATUS_BINT)  { dev->handleBIOZFifoInterrupt(false); }
  if (status & MAX30001_STATUS_RRINT) { dev->handleRtoRInterrupt(); }
}

void setup() {
  const uint32_t mask = MAX30001_STATUS_EINT | MAX30001_STATUS_BINT | MAX30001_STATUS_RRINT;
  afe.setInterruptAggregateCallback(onAfeAggregate, nullptr, mask);
}
```

If aggregate and individual callbacks are both registered, aggregate runs first.

## Interrupt Callback API summary

- `setInterruptEventCallback(event, cb, context)`
- `clearInterruptEventCallback(event)`
- `setInterruptAggregateCallback(cb, context, statusMask)`
- `clearInterruptAggregateCallback()`
- `clearAllInterruptCallbacks()`
- `clearLatchedStatusFlags()`

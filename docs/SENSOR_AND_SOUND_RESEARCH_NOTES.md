# Research Notes: Hit Sensing, FSR, Pitch/Treble Bug, and Magnetic Sensors

Date: 2026-08-16
Context: `genesis` firmware currently uses Piezo (`A2`) for hit trigger, FSR (`A0`) for
fade/sustain, and an analog Hall sensor (`A1`) for pitch, per [src/play.cpp](../src/play.cpp)
and [docs/PIN_CONFIGURATION.md](PIN_CONFIGURATION.md).

---

## 1. Hit-trigger sensor: why the piezo feels "not sensitive to hit"

The piezo disc itself *does* produce a voltage roughly proportional to strike force, but
the current firmware only uses it as a **threshold-crossing trigger**
(`PIEZO_THRESHOLD 190`, edge detection in `piezoISR()`), not as a calibrated amplitude
source. Two separate problems usually get confused here:

- **No analog front-end conditioning.** A bare piezo into a Teensy ADC pin gives a spiky,
  ringy, DC-drifting signal. Without a peak-detector/envelope circuit (diode + RC hold,
  or an op-amp precision rectifier), you're sampling a noisy instantaneous voltage, not
  a clean "hit energy" value — so velocity/amplitude estimates from `piezoCenterPeak`
  will be unreliable even though the sensor is physically capable.
- **Sampling rate.** A hard hit peaks in under ~1 ms. If the ISR/poll rate isn't fast
  enough relative to that, you can miss the true peak and only catch a partial ring-up,
  which reads as "insensitive."

### Recommended fix path (in order of effort)
1. **Keep the piezo, add a peak-hold circuit**: piezo → protection diode clamp (to
   ±3.3V) → simple envelope follower (diode + small RC, ~1–2 ms time constant) → ADC.
   This alone typically fixes "can't compute amplitude" complaints and is the cheapest
   option (no new sensor, ~$1 in passives).
2. **Better piezo element**: swap the generic piezo disc for a purpose-built **drum
   trigger piezo** (e.g., the type used in Roland/Alesis mesh-head pads — a large
   ~35–50 mm piezo disc bonded to a foam/rubber damping layer). These are specifically
   tuned for wide dynamic range and fast, clean decay, and are drop-in electrically
   compatible with what you have.
3. **Add a low-g MEMS accelerometer** on the same striking surface (e.g., Analog Devices
   ADXL345/ADXL372, or the high-bandwidth ADXL1001/1005). Accelerometers give a much
   more linear, low-noise force/velocity signal than a piezo and are what most modern
   electronic drum triggers use internally alongside a piezo. Downside: needs SPI/I2C
   or an extra ADC channel, plus firmware work to read the peak of a short burst.
4. **Dual-piezo (mechanically decoupled) trigger**, i.e., separate small piezo(s) per
   zone (center/rim), each with its own peak detector — this is what most electronic
   drum pads use for crosstalk-free hit detection and would pair naturally with the
   TODO item about adding a second piezo/FSR pair.

### On using a microphone instead
A microphone is **not recommended** as the primary hit sensor here, for exactly the
reason you flagged: it picks up ambient sound, other hits, and room noise, and needs a
gate/threshold that's hard to tune reliably in a live setting. If you want a
microphone-based path anyway:
- Use a **contact/piezo-diaphragm microphone** (a mic capsule mechanically coupled to
  the surface, e.g., a boundary/pickup mic like the type used on acoustic drum shells)
  rather than an open-air mic — this rejects most ambient sound by design since it only
  picks up structure-borne vibration.
- If an air mic is required, use a **cardioid or shotgun electret MEMS mic** placed very
  close to the strike point with a noise gate + short attack envelope follower, and
  expect to need per-venue threshold calibration. This is strictly worse than options
  1–3 above for reliability, so treat it as a fallback only.

**Bottom line recommendation:** add peak-hold conditioning to the existing piezo first
(near-zero cost, likely fixes most of the amplitude problem); if you still need better
dynamic range/velocity resolution, add a MEMS accelerometer (ADXL345 class) rather than
a microphone.

---

## 2. FSR: best option for clean "sound cutoff" (mute) and sustain control

Current use: FSR (`A0`) maps `[0,1023]` ADC → fade time `[0.1s, 2.0s]` (see
`fsrToDamping()`/fade-time mapping in [src/play.cpp](../src/play.cpp)). Two different
things are being asked for FSR, and they want different sensor behavior:

- **Sustain/expression control (continuous)** — current use case — wants a sensor with a
  smooth, monotonic, low-noise response across its full range.
- **Cutoff / mute (discrete, "stop the sound now")** — wants a sensor with a *fast, sharp*
  transition at a settable threshold, not necessarily fine linearity.

### Recommendations
- **For continuous sustain control:** Interlink Electronics **FSR 402/406** (round,
  standard hobbyist FSRs) are fine but notoriously non-linear and drift under sustained
  pressure — usable for what you're already doing (they've been "good enough" in most
  DIY drum-trigger builds) but not great for fine dynamics.
- **Better linearity option:** a **Velostat/Linqstat DIY pressure sensor** (two
  conductive-foil electrodes with a Velostat layer) can be tuned in size/shape for your
  pad and is cheaper per unit if you're scaling to more sensors, at the cost of some
  build consistency.
- **If you want a sharp, fast cutoff specifically:** don't rely on the FSR's analog
  curve for the cutoff — instead read the FSR normally for sustain, but drive the
  "cutoff" event off a **comparator-style threshold with hysteresis** (Schmitt trigger
  in firmware: fast release when FSR crosses back below e.g. 10% with a debounce), so
  the mute feels sharp and repeatable regardless of the FSR's non-linearity. This is a
  firmware change, not a new sensor, and is the most reliable way to get a "sharp"
  cutoff.
- **If budget allows and you want genuinely better dynamic range + faster response than
  Interlink FSRs:** look at **piezoresistive force sensors** like the **Tekscan FlexiForce
  A201**, which have a faster response time and better repeatability than Interlink FSRs,
  at higher unit cost. Reasonable upgrade if the mute/sustain feel is the main complaint.

**Bottom line recommendation:** keep FSR for sustain, but implement the mute/cutoff as a
hysteresis threshold in firmware rather than expecting the raw analog curve to give a
sharp cutoff; if you want more linear dynamics too, FlexiForce A201 is the upgrade path.

---

## 3. The treble/pitch bug — root cause and what to ask the sound engineer for

### Root cause (confirmed in code)
In [src/play.cpp](../src/play.cpp), `TREBLE_POT` does **not** feed any spectral/EQ
parameter — it only sets `trebleFreq` (150–250 Hz), which is blended with the Hall
sensor value to produce `hallFreq`, and that result is passed straight into
`ganGan.setBFreq(hallFreq)` — the synth's **fundamental body frequency**. So "treble"
is currently wired as *"the upper end of the pitch range,"* not as *brightness/harmonic
content*. That's exactly what your sound engineer is hearing: turning the knob shifts
overall pitch, not the treble/high-frequency timbre.

There is no separate spectral-shaping stage (filter cutoff, harmonic mix, or EQ curve)
tied to that pot at all — `bFreq` is the only thing it touches.

### What to request from the sound engineer
Ask him for a **synth patch/algorithm that exposes pitch and brightness as two
independent parameters**, so the firmware can drive them from two different sensors
without one leaking into the other. Concretely, ask for a SuperCollider (or equivalent)
patch structured like:

```supercollider
// Requested structure: pitch and brightness fully decoupled
SynthDef(\ganGan, { |pitchHz = 200, brightness = 0.5, gate = 1, amp = 0.4|
    var pitchSig, body, filtered, env;

    env = EnvGen.kr(Env.perc(0.001, 0.4), gate, doneAction: 2);

    // pitch: ONLY controls fundamental frequency
    body = SinOsc.ar(pitchHz) + (0.3 * LFNoise2.ar(pitchHz * 1.5));

    // brightness: ONLY controls harmonic/spectral content,
    // independent of pitchHz (e.g. filter cutoff scaled relative to pitch,
    // or a harmonic/waveshaping mix — NOT another frequency value)
    filtered = BLowPass4.ar(body, pitchHz * (1 + brightness * 8), 0.7);

    Out.ar(0, filtered * env * amp);
}).add;
```

Specifically ask him to define/return:
1. **Two separate control parameters** — one named clearly for pitch (Hz or MIDI note)
   and one for brightness/tone (a 0–1 or dB value), with brightness implemented via
   filter cutoff, harmonic amplitude ratio, or waveshaping — **never** as a second raw
   frequency that gets blended with pitch.
2. **The mapping curve he wants for brightness** (linear 0–1 → cutoff Hz, or dB shelf
   boost/cut like the existing `eqBands()` bass/treble implementation in
   [lib/Audio/control_sgtl5000.cpp](../lib/Audio/control_sgtl5000.cpp), which already
   does real EQ band shelving and is a good reference for what "treble" should mean).
3. **Confirmation of update rate** — whether brightness should be updated continuously
   (per-block, like the current Hall pitch modulation) or only at note-on (like a fixed
   per-hit timbre).

Once he provides that, the firmware-side fix is straightforward: replace the
`trebleFreq`/`hallFreq` blend with two independent calls — `ganGan.setBFreq(pitchHz)`
from Hall, and a new `ganGan.setBrightness(brightnessNorm)` from the treble pot that
drives a filter cutoff or the SGTL5000's `eqBands()` treble band — instead of both
sensors fighting over the same frequency variable.

---

## 4. Scaling to 6 magnetic pitch sensors + more magnets: best sensor choice

Current Hall setup uses a single **analog linear Hall sensor** on `A1` with a very
narrow usable ADC window (`HALL_MIN_ADC 516` / `HALL_MAX_ADC 535` — only ~19 ADC counts
of range out of 1023), then quantizes into 7 steps. That narrow window is why
sensitivity/resolution feels marginal — you're relying on a tiny slice of the sensor's
output range, so noise and placement tolerance matter a lot.

For **6 independent pitch zones with more magnets**, two real architectures to choose
between:

### Option A — One discrete digital Hall switch per note zone (recommended for crisp, reliable switching)
Use 6 separate **omnipolar digital Hall-effect switches**, one per magnet/note position
(e.g., Melexis US5881, Diodes AH3572/AH49E, or Honeywell SS451A). Each fires cleanly
high/low when a magnet crosses its threshold — no analog drift, no calibration window
to tune, and adding more magnets/zones is just adding more identical sensor+GPIO pins.
This is the more "sharp" and scalable option and matches what you described (discrete
note positions), but loses continuous pitch-bend/glide behavior between notes.

### Option B — Higher-sensitivity linear analog Hall sensor(s), if you want continuous glide to remain
If you want to keep the smooth pitch-sweep behavior (like the current re-excite/glide
logic) but with better sensitivity/resolution than the current sensor:
- **Allegro A1301/A1302** — linear Hall-effect sensors with ~2.5 mV/Gauss sensitivity
  (much more sensitive than typical low-cost ratiometric Hall ICs), giving you a wider,
  more usable ADC swing for the same magnet distance, instead of the current ~19-count
  window.
- **Honeywell SS495A/SS496A** — ratiometric linear Hall sensors, ratiometric to Vcc,
  commonly used in precision proximity/position sensing, good alternative if A1301/1302
  isn't available.
- If you need genuinely high resolution and are open to a different sensing principle:
  **AS5600** (12-bit magnetic rotary position sensor, I²C) or a **GMR-based angle
  sensor (e.g., TLE5010)** give much finer, sharper position resolution than a plain
  analog Hall IC, but they're designed for a single rotating magnet over one sensor, not
  6 independent linear zones — good only if the mechanical design is one rotating
  element rather than 6 discrete magnet positions.

**Bottom line recommendation:** if the 6 zones are meant to be discrete notes (which the
existing quantization code suggests), switch to **6 individual digital Hall switches**
(Option A) — it removes the calibration fragility entirely and scales cleanly with more
magnets. Only choose the analog route (Option B, A1301/A1302) if continuous pitch glide
between the 6 zones is a feature you want to keep.

---

## Summary of concrete next actions
1. Add a piezo peak-hold/envelope circuit (diode + small RC) before the ADC to fix
   amplitude sensing without buying new hardware.
2. Keep FSR for sustain; implement mute/cutoff as a firmware hysteresis threshold, not a
   raw analog cutoff. Consider Tekscan FlexiForce A201 if dynamics still feel poor.
3. Send the sound engineer the SuperCollider parameter request in §3 (separate `pitchHz`
   vs `brightness`), and update firmware to drive two independent synth parameters
   instead of blending them into one frequency.
4. For 6-zone pitch input, prefer 6 discrete digital Hall switches (e.g., Melexis
   US5881) unless continuous glide between zones must be preserved, in which case use
   Allegro A1301/A1302 linear Hall sensors for better sensitivity than the current
   sensor.

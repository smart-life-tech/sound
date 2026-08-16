# Research Notes: Hit Sensing, FSR, Pitch/Treble Behavior, and Magnetic Pitch Sensing

Date: 2026-08-16
Scope: `genesis` firmware currently uses a piezo (`A2`) for hit trigger, an FSR (`A0`)
for fade/sustain, and an analog Hall sensor (`A1`) for pitch, per
[src/play.cpp](../src/play.cpp) and [docs/PIN_CONFIGURATION.md](PIN_CONFIGURATION.md).
This document records findings and action items for three parties: the mechanical
engineer, the sound engineer, and the firmware owner (self).

---

## 1. Hit-trigger sensing: applicability of standard electronic drum pad architecture

### 1.1 Is the electronic-drum-pad architecture applicable here?

Commercial electronic drum triggers (Roland, Yamaha, Alesis) are the correct reference
architecture for this problem and are directly transferable to this project. Their
standard design has three parts, per the general drum-trigger literature
(Wikipedia: [Electronic drum](https://en.wikipedia.org/wiki/Electronic_drum), "Sensors
and pads" section):

1. A piezoelectric sensor bonded to (or embedded in) a rubber/silicone/mesh striking
   surface, converting mechanical strain into a voltage.
2. An analog conditioning stage in the "drum brain" that shapes the raw piezo pulse into
   a usable peak/velocity value.
3. A trigger algorithm (threshold, peak-hold, retrigger/mask timing) that turns the
   conditioned signal into a note-on event with a velocity value.

The piezo module in use already includes a diode clamp that conditions its output into
the Teensy's 0–3.3 V ADC range. That clamp addresses signal protection only — it
prevents over/under-voltage at the ADC pin — and is not the same function as the second
stage described above. The firmware still implements only the third stage (a fixed
threshold edge trigger, `PIEZO_THRESHOLD`), and the signal presented to the ADC is
still the raw, unrectified, ringing piezo waveform, only voltage-limited rather than
shaped into a stable peak value. This is the direct cause of the reported problem: the
piezo can physically sense force, and the existing module keeps that signal within a
safe voltage range, but nothing in the current signal path converts a strike into a
single, stable amplitude reading. Adding the missing peak/envelope stage is sufficient
to resolve the amplitude/velocity issue without changing sensor hardware.

### 1.2 Recommended architecture change (conditioning stage)

Add an envelope/peak-hold stage after the existing clamp module and before the ADC
input: piezo → existing diode-clamp module (0–3.3 V protection, already in place) →
envelope follower (rectifying diode into a small RC, time constant on the order of
1–2 ms) → ADC. The existing module should be retained for protection; the addition is
a small peak-detector stage, not a replacement of what is already built.

### 1.3 Sensor element itself

The physical piezo disc does not need to be replaced to fix amplitude sensing, and the
existing clamp module does not need to be removed; the missing piece is the envelope
stage described in §1.2. If greater dynamic range is still required after adding that
stage, two escalation paths exist, in order of how closely they follow standard
drum-trigger practice:

1. **Foam/rubber-backed piezo trigger element**, of the type used under mesh drum
   heads, for better mechanical decoupling and cleaner decay than a bare piezo disc
   taped to a surface.
2. **Add a low-g MEMS accelerometer** on the same striking surface as a secondary
   sensor (e.g., Analog Devices ADXL345-class part) for a more linear force signal.
   This requires an additional ADC/communication channel and is a larger firmware
   change than option 1.

### 1.4 On using an open-air microphone

An open-air microphone is not recommended as the primary hit sensor, for the reason
already identified: it cannot distinguish the intended strike from ambient sound,
adjacent hits, or room noise, and requires a noise gate that is difficult to keep
stable across different rooms and volumes. This is consistent with how commercial
"acoustic triggered kits" are built — they use a piezo/contact microphone mechanically
coupled to the drum shell rather than an air microphone
(Wikipedia: [Electronic drum § Variations](https://en.wikipedia.org/wiki/Electronic_drum#Variations)).
If a microphone-based approach is required for another reason, it must be a **contact
microphone** coupled directly to the striking surface, not a free-standing air
microphone.

### For the mechanical engineer
- Add a small envelope/peak-hold stage (rectifying diode + RC) in line after the
  existing diode-clamp module and before the Teensy ADC pin; the existing clamp module
  is retained as-is for protection.
- If further dynamic range is needed later, reserve mounting space for a MEMS
  accelerometer on the same striking surface as a secondary sensor.

### For self (firmware)
- Read the conditioned (enveloped) signal as a peak value per hit rather than relying
  on a single instantaneous threshold crossing; retain the current debounce/retrigger
  logic on top of that peak.

---

## 2. FSR: sound cutoff (mute) and sustain control

Current use: FSR (`A0`) maps `[0, 1023]` ADC to a fade time of `[0.1 s, 2.0 s]`, per the
mapping in [src/play.cpp](../src/play.cpp). Two distinct behaviors are being requested
from the same sensor and should be treated separately:

- **Sustain/expression control (continuous):** the existing use case, requiring a
  smooth, monotonic response.
- **Cutoff/mute (discrete):** requires a fast, repeatable transition at a settable
  threshold; fine linearity is not required for this behavior.

### Findings
- The currently used class of sensor (Interlink-style round FSR) is adequate for
  continuous sustain control but has known non-linearity and pressure-drift
  characteristics.
- A sharp, repeatable cutoff is better achieved as a firmware behavior — a threshold
  with hysteresis (Schmitt-trigger logic) applied to the existing FSR reading — rather
  than by changing sensor hardware. This is the recommended fix for the "sharp cutoff"
  requirement and does not require new parts.
- If sustain dynamics themselves need to be more linear and repeatable, a
  piezoresistive sensor such as the Tekscan
  [FlexiForce A201](https://www.tekscan.com/products-solutions/force-sensors/a201) is a
  documented upgrade path over Interlink-style FSRs, with datasheet-specified force
  ranges (0–4.4 N / 0–111 N / 0–445 N variants) and a trimmable sensing area.

### For the mechanical engineer
- No change required to the FSR mounting for the cutoff behavior; current sensor
  placement is retained.
- If FlexiForce A201 is adopted for improved sustain linearity, confirm mounting area
  against the available trimmed lengths (51 mm, 102 mm, 152 mm from the Mouser-listed
  variants).

### For self (firmware)
- Implement mute/cutoff as a hysteresis threshold on the existing FSR reading (fast
  release below a low threshold, with a short debounce) rather than expecting the
  cutoff to emerge naturally from the analog curve.
- Evaluate FlexiForce A201 only if sustain dynamics remain unsatisfactory after the
  hysteresis fix.

---

## 3. Treble/pitch coupling: root cause and request to the sound engineer

### 3.1 Root cause, confirmed in code

In [src/play.cpp](../src/play.cpp), `TREBLE_POT` does not feed any spectral or EQ
parameter. It sets `trebleFreq` (150–250 Hz), which is blended with the Hall sensor
reading to produce `hallFreq`, and that value is passed directly to
`ganGan.setBFreq(hallFreq)` — the synthesizer's fundamental body frequency. The control
currently wired as "treble" therefore sets the upper end of the pitch range, not the
spectral brightness of the sound. This matches the sound engineer's observation:
turning the control changes overall pitch rather than high-frequency tonal content.
There is no filter, harmonic-mix, or EQ stage tied to this control in the current
signal path.

### 3.2 Request to the sound engineer

Request a synthesis patch that exposes **pitch** and **brightness** as two independent
parameters, so the firmware can drive them from two separate sensors without one
affecting the other. The following SuperCollider structure illustrates the requested
separation and can be shared as a starting point for discussion:

```supercollider
// Requested structure: pitch and brightness fully decoupled
SynthDef(\ganGan, { |pitchHz = 200, brightness = 0.5, gate = 1, amp = 0.4|
    var body, filtered, env;

    env = EnvGen.kr(Env.perc(0.001, 0.4), gate, doneAction: 2);

    // pitch: controls fundamental frequency only
    body = SinOsc.ar(pitchHz) + (0.3 * LFNoise2.ar(pitchHz * 1.5));

    // brightness: controls spectral/harmonic content only,
    // independent of pitchHz (filter cutoff or harmonic mix — not a second frequency)
    filtered = BLowPass4.ar(body, pitchHz * (1 + brightness * 8), 0.7);

    Out.ar(0, filtered * env * amp);
}).add;
```

Specific items to request from the sound engineer:
1. Two independently named parameters — one for pitch (Hz or MIDI note) and one for
   brightness/tone (a 0–1 or dB value) — with brightness implemented through filter
   cutoff, harmonic amplitude ratio, or waveshaping, and never as a second raw
   frequency blended with pitch.
2. The intended mapping curve for brightness (linear 0–1 to cutoff frequency, or a dB
   shelf boost/cut). The existing `eqBands()` bass/treble shelving implementation in
   [lib/Audio/control_sgtl5000.cpp](../lib/Audio/control_sgtl5000.cpp) is a useful
   reference for what a proper treble control should do.
3. Confirmation of the intended update rate for brightness — continuous (per audio
   block, matching the current Hall pitch modulation) or fixed per note-on (a static
   per-hit timbre).

### For self (firmware)
- Once the sound engineer confirms the parameter structure, replace the
  `trebleFreq`/`hallFreq` blend with two independent calls: `ganGan.setBFreq(pitchHz)`
  driven by the Hall sensor, and a new `ganGan.setBrightness(brightnessNorm)` driven by
  the treble control, mapped to a filter cutoff or to the SGTL5000 `eqBands()` treble
  band.

---

## 4. Magnetic pitch sensing: scaling to six zones with continuous glide

### 4.1 Requirement

The intended use of the additional magnets and sensors is continuous pitch selection by
magnet position (glide), not discrete note switching. This rules out digital Hall
switches (which only provide on/off zone detection) and confirms that the correct
architecture is a set of **linear analog Hall sensors**, consistent with how the
current single-channel Hall pitch input already works.

### 4.2 Current limitation

The existing sensor is used over a very narrow window
(`HALL_MIN_ADC 516`, `HALL_MAX_ADC 535` — approximately 19 ADC counts out of 1023),
which is the direct cause of the reported lack of sensitivity: only a small slice of
the sensor's usable output range is being read, so noise and magnet-placement tolerance
have an outsized effect on the result.

### 4.3 Recommended sensor family

Use higher-sensitivity **linear analog Hall-effect ICs**, one per pitch zone, in place
of the current sensor, to obtain a wider and more usable ADC swing for the same magnet
distance:

- **Allegro A1301/A1302** — linear Hall-effect sensors with an output sensitivity on
  the order of 2.5 mV/Gauss, substantially more sensitive than typical low-cost
  ratiometric Hall ICs, giving a wider ADC swing for the same magnet spacing than the
  current sensor. Available through standard distributors, e.g.
  [Mouser](https://www.mouser.com/ProductDetail/Allegro-MicroSystems/A1301ELHLT-T) and
  [DigiKey](https://www.digikey.com/en/products/detail/allegro-microsystems/a1301elhlt-t/812331).
- **Honeywell SS495A/SS496A** — ratiometric linear Hall sensors, a suitable alternative
  if the Allegro parts are unavailable, commonly used in precision proximity/position
  sensing applications.

This recommendation deliberately excludes discrete digital Hall switches (e.g.,
Melexis US5881, or the SparkFun
[US1881 latching Hall switch](https://www.sparkfun.com/hall-effect-sensor-us1881-latching.html)
evaluated during this review) because those parts only report which zone a magnet is
in, not the continuous position within a zone, and would remove the glide behavior the
project requires.

### 4.4 Recommendation for six-zone scaling

Deploy six independent A1301/A1302 (or equivalent) linear Hall channels, one per magnet
zone, each feeding its own ADC input, and widen the active sensing window per channel
(replacing the current ~19-count window) so each channel uses a larger portion of its
output range. This preserves and improves the continuous glide behavior already present
in the firmware (the Hall-driven `bFreq` sweep and re-excite logic), while giving each
of the six zones the sensitivity margin the current single-sensor implementation lacks.

### For the mechanical engineer
- Plan for six discrete linear Hall sensor placements plus a corresponding increase in
  the number and/or strength of magnets, with attention to magnet spacing so that
  adjacent zones do not saturate each other's sensor readings.
- Confirm the target air gap and magnet grade with the electrical calibration range in
  mind, since sensitivity and usable ADC window are directly affected by both.

### For self (firmware)
- Add five additional ADC channels for the new Hall sensors and replace the single
  narrow calibration window (`HALL_MIN_ADC`/`HALL_MAX_ADC`) with a per-channel
  calibrated range once the new sensors and magnets are in place.
- Re-tune the smoothing (`HALL_SMOOTH_ALPHA`) and quantization thresholds per channel
  after the wider ADC swing is confirmed, since the current constants were tuned for
  the narrow single-sensor window.

---

## Summary of action items

| Owner | Action |
|---|---|
| Mechanical engineer | Add an envelope/peak-hold stage (rectifying diode + RC) after the existing diode-clamp module and before the Teensy ADC; retain the existing clamp module for protection. |
| Mechanical engineer | Reserve mounting space for six A1301/A1302-class linear Hall sensors and additional magnets, with attention to magnet spacing and air gap. |
| Sound engineer | Provide a SuperCollider (or equivalent) patch exposing independent `pitchHz` and `brightness` parameters, per §3.2. |
| Self (firmware) | Add piezo envelope reading in place of raw instantaneous threshold sampling. |
| Self (firmware) | Implement FSR mute/cutoff as a hysteresis threshold; evaluate FlexiForce A201 only if sustain dynamics remain unsatisfactory. |
| Self (firmware) | Replace the `trebleFreq`/`hallFreq` blend with independent pitch and brightness calls once the sound engineer's patch is available. |
| Self (firmware) | Add per-channel ADC reads and calibration ranges for six linear Hall sensors; re-tune smoothing and quantization constants accordingly. |

## References
- Wikipedia, [Electronic drum](https://en.wikipedia.org/wiki/Electronic_drum) — sensor and drum-module architecture background.
- Tekscan, [FlexiForce A201 Sensor](https://www.tekscan.com/products-solutions/force-sensors/a201) — force sensor specifications and force-range variants.
- SparkFun, [Hall-Effect Sensor US1881 (Latching)](https://www.sparkfun.com/hall-effect-sensor-us1881-latching.html) — reference part for digital Hall switching, evaluated and not adopted (see §4.3).
- Mouser, [Allegro A1301ELHLT-T](https://www.mouser.com/ProductDetail/Allegro-MicroSystems/A1301ELHLT-T) and DigiKey, [Allegro A1301ELHLT-T](https://www.digikey.com/en/products/detail/allegro-microsystems/a1301elhlt-t/812331) — linear Hall-effect sensor listings.

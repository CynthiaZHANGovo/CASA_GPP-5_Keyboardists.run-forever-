
# Introduction and Motivation

Exercise using boxing is becoming an increasingly popular type of training, as it incorporates high levels of physical exertion, coordination and prolonged involvement. Recent studies suggest that boxing exercise offers benefits beyond general fitness, including improvements in strength, balance, mobility, and exercise adherence, making it suitable for a wide range of individuals (Wang _et al_., 2025). However, effective punching training should not rely solely on repetition, but also emphasise feedback on movement quality, timing, and performance.

The role of feedback is particularly significant in the development of motor skills. Research in physical education has demonstrated that appropriate feedback can substantially enhance motor learning outcomes compared to practice alone, particularly when learners receive clear and timely input during or immediately after performing a movement (Han, Syed Ali and Ji, 2022). This is especially relevant in home-based or solitary exercise contexts, where individuals may lack coaching, structured guidance, or consistent motivation. Emerging research on home exercise highlights that technologies such as real-time feedback systems, wearable devices, and interconnected platforms can improve training environments by making them more interactive, informative, and engaging (Zhou _et al_., 2025).

These developments align with the broader concept of embodied interaction, where physical actions and digital responses are closely integrated in meaningful ways. Rather than viewing exercise as purely physical, this perspective emphasises how interactive systems can transform movement into responsive and motivating experiences (Lee-Cultura and Giannakos, 2020). To address these challenges, our project, _Punch Reach_, explores an interactive punching training prototype that integrates pressure sensors, LED feedback, and networked communication to support engaging individual and remote training. This project demonstrates how a simple connected training system can enhance feedback, increase user engagement, and create a more responsive home training experience.

# System Design and Implementation

This chapter outlines the design and implementation of the Punch Reach system, including materials, enclosure, hardware, and software, with a focus on integration, safety, and responsiveness.

## Materials Selection

The Punch Reach materials were chosen with regard to the balance between impact protection, user satisfaction, stability and durability of materials in a household exercising environment. As the prototype has been developed to handle repetitive punching interaction, it was necessary that the striking surface could absorb the force effectively and be lightweight and easy to assemble. Considering the given context, EVA and EPE foam boards were selected as the primary cushioning materials. Polymer foams are common in sports applications due to their excellent energy-absorption properties and their capability to minimize transmitted force upon impact (Tomin and Kmetty, 2022). They are then suitable to be used on a punching interface where the safety of the user and the protection of the wall are essential. Besides, studies on low-velocity impact behavior indicate that polymeric foams may enhance passive safety by reducing the impact and spreading the forces more efficiently (Penta _et al_., 2018). To our prototype, this supports the application of EVA as the striking surface and EPE as the wall-contact layer.

The soft, flexible and more comfortable material that would be used on repeated use by the device was elastic nylon fabric which was chosen as contact surface material. As indicated by sports textiles studies, fabrics that are utilized in active situations should have both mechanical flexibility and breathability and comfort particularly when they come into frequent contact with the body (Cui and Wang, 2025). This is why elastic nylon became a good outer layer to enhance tactile experience of the prototype.

In order to house the microcontroller and wiring, a 3D-printed enclosure was used in support of the electronics. This decision mattered not only because it would protect the components against damage during punching but also because it would enhance the neatness of the cables and the overall quality of the build. According to MacDonald _et al_. (2014), it is highly suitable to use 3D printing to rapid prototype as it enables designing structural parts based on electronic systems with a high degree of integration. Taken together, those material selections contributed to increasing the safety, durability and usability of Punch Reach with respect to an interactive home workout.

## Enclosure Design

The enclosure is the physical interface between the user and the system, not just a protective shell, but a structurally engineered component, which brings together interaction, protection and ergonomics. The design has gone through four iterations, as shown below:

<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/improvement.png" alt="Enclosure Iterations">

The initial prototypes were aimed at defining a working surface of a strike with a simple sensor arrangement. The second generation substituted a traditional screen with a ring of LEDs cut into the panel, spatially aligned to the point of impact and ensuring that the reaction of the system was much more immediate and legible. The third version was the most drastic structural change: MCU and core control circuitry were moved to an external 3D-printed control box, which is physically disconnected to the strike body. This isolation allows percussion of high frequency to avoid loosening connections and weakening delicate components, significantly enhancing the long-term reliability, similar in spirit to lightweight shock-absorbing load-bearing components designed using 3D printing (Zhang _et al_., 2022). The last design enhanced the cable routing by guiding the wires through special exit holes into the control box and getting rid of the dangers of exposed or trapped cords.

The panel itself is made of a layered composite structure. A visual representation is provided below. The innermost layer is a rigid wooden substrate, which evenly spreads the impact force over sensor array. It is then covered with high-density EVA and EPE foam layers, forming a graduated buffer which both isolates fine pressure differentiation and absorbs high-frequency percussion. A stretchy outer fabric cover balances between the resistance to abrasion and the sense of comfort with each repeated strike. This multilayer, energy-absorbing approach aligns with current trends in 3D-printed energy absorbing structures for crashworthiness applications (Isaac and Duddeck, 2022).

The issue of mounting was considered an ergonomic problem. The rear side has a hook and loop system, which enables the user to install the wall without using tools and easily adjust the height up and down to suit users of all sizes. The panel angle was also optimized to match the natural path of a straight punch and this decreased the strain on the wrist during longer sessions.

<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/Materia.png" width="300" alt="Materia">

Through these iterations, the enclosure began a change to an active, load-carrying subsystem that conditions the quality of all interactions.

# Boxing Duel — Two-Node Real-Time Interactive System

A two-player reaction/force "boxing" duel built on two **Arduino MKR WiFi 1010**
boards (Microchip **SAMD21G18A**, ARM Cortex-M0+). Each node reads four force
pads, detects valid hits, and synchronises game state with the opposite node
over MQTT, while driving four 7-LED WS2812B strips as force feedback.

Built with **PlatformIO** (not the Arduino IDE): a single `main.cpp` targets both
boards, dependencies are pinned in `platformio.ini`, and the custom SPI+DMA LED
driver is a proper library under `lib/`.

The headline engineering problem was **keeping the WiFi link alive while
refreshing addressable LEDs** — solved by moving LED output onto a spare
**SERCOM in SPI mode fed by the DMA controller**, so LED refresh never disables
global interrupts and never starves the NINA-W102 SPI handshake.

---

## System Architecture

```
   ┌─────────────────────  NODE A (MKR WiFi 1010)  ─────────────────────┐
   │                                                                    │
   │  4x FSR ─ADC+DMA scan─►  EMA filter ─►  Hit FSM ─►  hitQueue        │
   │           (free-running, no CPU)     (IDLE/ARMED/REFRACT)  │        │
   │                                                         ▼          │
   │  4x WS2812B ◄── SERCOM0 SPI + DMA (non-blocking) ◄── paintPad      │
   │                                                         │  JSON    │
   │                                                    TaskNetwork      │
   └─────────────────────────────────────────────────────────┼─────────┘
                                                              │ SERCOM1 SPI
                                                       NINA-W102 (WiFi)
                                                              │
                                                       MQTT broker
                                                  (mqtt.cetools.org)
                                                              │
                                                       NODE B (mirror)
```

Two FreeRTOS tasks per node, **equal priority (2)** so the scheduler
time-slices them fairly:

| Task          | Job                                            | Stack (words) |
|---------------|------------------------------------------------|---------------|
| `TaskSensing` | reads DMA-filled ADC buffer, EMA filter, hit FSM | 256         |
| `TaskNetwork` | WiFi keepalive, MQTT pub/sub, JSON encode      | 1024          |

Sampling is offloaded from the CPU: a **free-running ADC** scans the four FSR
pads in hardware and **DMA (channel 1)** copies each result into a RAM buffer,
so `TaskSensing` reads the latest values without ever busy-waiting inside
`analogRead()`. Two DMA channels are in use: **ch0** feeds the LEDs
(SRAM→SPI), **ch1** fills the ADC buffer (ADC→SRAM).

Decoupled by `hitQueue` (force events); shared `gameState[]` guarded by
`stateMutex`.

**Interrupt priorities** are restructured by business rule (Cortex-M0+ has only
4 levels): NINA handshake (EIC) = highest, NINA SPI (SERCOM1) = high, LED DMA
completion (DMAC) = lowest — so WiFi responsiveness is never delayed by LED
housekeeping.

---

## Why SAMD21 / MKR WiFi 1010

Driven by **peripherals, not headline memory**:

- **12-channel DMAC + 12-channel Event System** — streams the LED waveform from
  SRAM to SPI with zero CPU involvement (the core fix below).
- **6× SERCOM** — one (SERCOM1) is reserved by WiFiNINA for the NINA link; a
  spare (SERCOM0) drives the LEDs as SPI. Without free serial blocks this design
  is impossible.
- **32 KB SRAM** is comfortable: the whole DMA LED buffer is ~292 bytes
  (28 LEDs × 9 bytes + reset gap). WiFi/TLS heavy lifting runs on the NINA-W102
  (ESP32), *not* the SAMD21.

---

## The Core Problem & Fix: SPI + DMA WS2812B

WS2812B is an 800 kHz return-to-zero protocol with ±150 ns timing tolerance.
A bit-banged driver must `__disable_irq()` for the whole frame. On this board
that masks the **NINA SPI handshake interrupt**, so the WiFi transaction times
out and the link drops.

**Fix:** encode each WS2812B bit into 3 SPI bits at ~2.4 MHz —
`1 → 0b110`, `0 → 0b100` — and let the **DMAC** copy the encoded buffer into the
SPI DATA register on the "TX-empty" trigger. The CPU fires the transfer and
returns; **interrupts are never disabled**. See `lib/DmaNeoPixel/` for the
encoding theory and register-level setup.

```
1 colour byte  -> 24 SPI bits = 3 SPI bytes
1 LED (GRB)    -> 9 SPI bytes
28 LEDs        -> 252 bytes + ~40 reset bytes = 292-byte DMA buffer
```

---

## Known Trade-offs (honest engineering notes)

- **MQTT QoS 0**: no PUBACK round-trip → lowest latency; cost is no delivery
  guarantee, acceptable because hit/release events are idempotent and re-asserted
  by the sender's FSM.
- **Equal task priority + time-slicing** trades strict preemption for fairness;
  safe because the sensing task is short and bounded.
- **Latency (~100 ms)** is one-way application latency on a campus-LAN broker,
  not a hard real-time bound and not an internet RTT. See the LATENCY NOTE in
  `src/main.cpp`.
- **DMA channels**: the LED driver uses DMAC channel 0 and the ADC driver uses
  channel 1. The SAMD21 DMAC keeps all channel descriptors in one base-address
  table indexed by channel number; in a fully integrated build both drivers must
  share that single descriptor array (each writing only its own channel's slot).
  The drivers are written as separate modules for clarity — merging their
  descriptor tables is the one integration step to verify on real hardware.

---

## Wiring (per node)

| Signal        | MKR 1010 pin | Notes                                  |
|---------------|--------------|----------------------------------------|
| FSR 1–4       | A1, A2, A3, A4 | voltage-divider to 3V3, 10k to GND   |
| WS2812B data  | SERCOM0 MOSI pad | route with `pinPeripheral(pin, PIO_SERCOM_ALT)` |
| WS2812B 5V/GND| 5V / GND     | inject power at the strip, common GND  |
| NINA WiFi     | internal SERCOM1 | reserved by WiFiNINA — do not reuse |

---

## Build & Flash (PlatformIO)

```bash
# install PlatformIO Core (once)
pip install platformio

# copy the credential template and fill it in
cp src/secrets.h.example src/secrets.h    # then edit src/secrets.h

# build + flash node A
pio run -e boardA -t upload

# build + flash node B
pio run -e boardB -t upload
```

One codebase, two identities via `-D BOARD_ID` in `platformio.ini` — no
duplicated `boardA_code` / `boardB_code` files.

> Dependency note: `WiFiNINA` + `FreeRTOS_SAMD21` version resolution on
> PlatformIO can need manual pinning; the versions in `platformio.ini` are a
> starting point.

---

## Repository Layout

```
platformio.ini            # board, framework, deps, boardA/boardB envs
src/
  main.cpp                # single codebase (A/B via -D BOARD_ID)
  secrets.h.example       # credential template (real secrets.h is gitignored)
lib/
  DmaNeoPixel/
    DmaNeoPixel.h         # SPI+DMA WS2812B driver — interface + encoding theory
    DmaNeoPixel.cpp       # DMAC + SERCOM register-level implementation
    library.json          # PlatformIO library manifest
  DmaAdc/
    DmaAdc.h              # free-running multi-channel ADC + DMA driver
    DmaAdc.cpp            # ADC scan + DMAC channel 1 register-level setup
    library.json          # PlatformIO library manifest
Documents/
  pictures/               # architecture diagram, wiring, scope captures
  GPP_Minutes_*.md        # team meeting minutes
```

# Reflection and Future Work
During development, there were some differences between the sketched design and the physical prototype, particularly in appearance and finishing. The way interaction and presentation were structured could be further refined to improve clarity and usability. The LED strip implementation is relatively simple; while safety was considered, the lighting effect could be enhanced, for example by reflecting light onto the wall instead of exposing it directly.

&#x20;In terms of future work, the interaction between the two devices is still limited, and Wi-Fi settings are hardcoded, making network switching inconvenient. A dedicated application could allow flexible Wi-Fi connection and add features such as a time-limited competitive mode and a memory function to record and review past training sessions.

The current fabric contact surface is too smooth, which may cause lateral slipping during inaccurate punches and increase the risk of wrist strain. Replacing it with artificial leather would provide better friction and more stable interaction.

# Contributions

- **Xinyuan Sun**: Enclosure assembly, 3D modeling and 3D printing, video editing

- **Jiahua He**: Circuit assembly, initial testing code writing, enclosure assembly, video shooting

- **Xinyi Zhang**: Trimming materials, enclosure design, enclosure assembly, video shooting, scriptwriting for videos

- **Cheng Zhong**: Purchase of materials and components, scriptwriting for videos, main actor of the video, enclosure assembly

- **Qingshan Luo**: Component soldering, final code writing, main actor of the video

# Conclusion

This project presented *Punch Reach*, an interactive punching training system combining sensing, real-time feedback, and networked communication. As an ambient device connecting users across distance, it demonstrates how integrating physical interaction with digital feedback can enhance engagement and responsiveness in home exercise.

---

# References

Cui, G. and Wang, C. (2025) ‘Applications and development trends of textile materials in sports: A review’, *Alexandria Engineering Journal*, 126, pp. 491–506.

Han, Y., Syed Ali, S.K.B. and Ji, L. (2022) ‘Feedback for promoting motor skill learning in physical education: A trial sequential meta-analysis’, *International Journal of Environmental Research and Public Health*, 19(22), p. 15361.

Isaac, C.W. and Duddeck, F. (2022) ‘Current trends in additively manufactured (3D printed) energy absorbing structures for crashworthiness application – a review’, *Virtual and Physical Prototyping*, 17(3), pp. 672–708. doi:10.1080/17452759.2022.2073792.

Lee-Cultura, S. and Giannakos, M. (2020) ‘Embodied interaction and spatial skills: A systematic review of empirical studies’, *Interacting with Computers*, 32(4), pp. 331–366.

MacDonald, E., Salas, R., Espalin, D., Perez, M., Aguilera, E., Muse, D. and Wicker, R.B. (2014) ‘3D printing for the rapid prototyping of structural electronics’, *IEEE Access*, 2, pp. 234–242.

Penta, F., Amodeo, G., Gloria, A., Martorelli, M., Odenwald, S. and Lanzotti, A. (2018) ‘Low-velocity impacts on a polymeric foam for the passive safety improvement of sports fields: meshless approach and experimental validation’, *Applied Sciences*, 8(7), p. 1174.

Tomin, M. and Kmetty, Á. (2022) ‘Polymer foams as advanced energy absorbing materials for sports applications—A review’, *Journal of Applied Polymer Science*, 139(9), p. 51714.

Wang, Z., Song, B., Liu, C., Ma, H., Bai, Z., Carneiro, M.A.S., Youssef, L., Chen, C., Zhang, L. and Wang, D. et al. (2025) ‘Effects of boxing exercise in people with Parkinson’s disease: a systematic review’, *Frontiers in Aging Neuroscience*, 17, p. 1505326.

Zhang, G., Feng, R., Li, J., Zhou, Y., Zhou, X. and Wang, A. (2022) ‘Lightweight design of shock-absorbing and load-bearing components based on 3D printing technology’, *Coatings*, 12(6), pp. 1–16. doi:10.3390/coatings12060799.

Zhou, T., Zhang, S., Liu, S. and Yu, J. (2025) ‘Digital technology integration in home-based exercise: a systematic review of research evolution, applications, and impact mechanisms’, *BMC Public Health*, 25(1), p. 3528.


---

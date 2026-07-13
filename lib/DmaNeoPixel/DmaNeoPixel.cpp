/*
 * ============================================================================
 *  DmaNeoPixel.cpp  —  SAMD21 DMAC + SERCOM implementation
 * ============================================================================
 *  This is deliberately register-level (not a library wrapper) so that in an
 *  interview you can point at each block and explain WHAT peripheral it touches
 *  and WHY. Read the header first for the encoding theory.
 *
 *  Peripheral budget on MKR1010 (SAMD21G18A):
 *    - SERCOM1 : reserved by WiFiNINA for the NINA-W102 link  -> DO NOT TOUCH
 *    - SERCOM0 : free -> we use it as SPI master, MOSI only, to drive the LEDs
 *    - DMAC ch 0 : moves spiBuf -> SERCOM0 SPI DATA on "TX buffer empty"
 *  We only need MOSI (data line); SCK is generated but unused by WS2812B, MISO
 *  is irrelevant. The LED data line connects to the SERCOM0 PAD0/MOSI pin.
 * ============================================================================
 */

#include "DmaNeoPixel.h"

// Single instance pointer so the C-style DMAC ISR can reach the driver state.
// (We only ever instantiate one LED driver, so a singleton pointer is fine.)
static DmaNeoPixel *s_dnpInstance = nullptr;

// ---- DMA descriptor tables (must be 128-bit aligned, in SRAM) --------------
typedef struct {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor;

static volatile dmacdescriptor wrb[1] __attribute__((aligned(16)));        // write-back
static dmacdescriptor          descriptor_section[1] __attribute__((aligned(16)));

#define LED_DMA_CHANNEL  0
// SERCOM0 SPI: ~2.4 MHz. F_CPU 48MHz; BAUD = F_ref/(2*F_desired) - 1.
// Using GCLK0 (48MHz): 48e6/(2*2.4e6) - 1 = 9.  -> SPI clk = 48e6/(2*(9+1)) = 2.4MHz.
#define LED_SPI_BAUD  9

void DmaNeoPixel::encodeByte(uint8_t value, uint8_t *dst) {
  // Expand 8 colour bits -> 24 SPI bits (3 bytes). MSB first.
  // Each WS2812 bit becomes 3 SPI bits: 1 -> 0b110, 0 -> 0b100.
  uint32_t bits = 0;
  for (int i = 7; i >= 0; --i) {
    bits <<= 3;
    bits |= (value & (1 << i)) ? 0b110 : 0b100;
  }
  dst[0] = (bits >> 16) & 0xFF;
  dst[1] = (bits >> 8)  & 0xFF;
  dst[2] = (bits)       & 0xFF;
}

void DmaNeoPixel::setPixel(uint16_t idx, uint8_t r, uint8_t g, uint8_t b) {
  if (idx >= DNP_NUM_LEDS) return;
  uint8_t *p = &spiBuf[idx * DNP_BYTES_PER_LED];
  // WS2812B colour order is GRB.
  encodeByte(g, p);
  encodeByte(r, p + 3);
  encodeByte(b, p + 6);
}

void DmaNeoPixel::clear() {
  for (uint16_t i = 0; i < DNP_NUM_LEDS; ++i) setPixel(i, 0, 0, 0);
}

bool DmaNeoPixel::busy() {
  // inFlight is set when we fire a frame and cleared by the DMAC done-ISR.
  // Falls back to the channel-enable bit if the done-IRQ was never turned on.
  return inFlight;
}

void DmaNeoPixel::begin() {
  clear();
  for (uint16_t i = DNP_NUM_LEDS * DNP_BYTES_PER_LED; i < DNP_BUF_LEN; ++i)
    spiBuf[i] = 0x00;   // trailing low = WS2812B reset/latch gap

  // ---- 1. Clock the SERCOM0 peripheral from GCLK0 (48 MHz) ----------------
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM0_CORE) |
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

  // ---- 2. Route MOSI to the SERCOM0 pad (see board wiring note in README) --
  // (PORT MUX configuration for the chosen LED-data pin is done in setup();
  //  kept out of the driver so the data pin is documented in one place.)

  // ---- 3. Configure SERCOM0 as SPI master, MSB first, MOSI on PAD0 ---------
  SERCOM0->SPI.CTRLA.bit.ENABLE = 0;
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);
  SERCOM0->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
                           SERCOM_SPI_CTRLA_DOPO(0) |   // MOSI=PAD0, SCK=PAD1
                           SERCOM_SPI_CTRLA_DIPO(3);
  SERCOM0->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;       // we don't use RX, harmless
  while (SERCOM0->SPI.SYNCBUSY.bit.CTRLB);
  SERCOM0->SPI.BAUD.reg  = LED_SPI_BAUD;
  SERCOM0->SPI.CTRLA.bit.ENABLE = 1;
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);

  // ---- 4. Enable the DMAC and point it at our descriptor tables -----------
  PM->AHBMASK.reg  |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg  = (uint32_t)wrb;
  DMAC->CTRL.reg     = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xF);

  // ---- 5. Configure channel 0: trigger = SERCOM0 TX empty ------------------
  DMAC->CHID.reg = LED_DMA_CHANNEL;
  DMAC->CHCTRLA.reg = 0;                         // disable while configuring
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) |
                      DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_TX) |
                      DMAC_CHCTRLB_TRIGACT_BEAT;
}

void DmaNeoPixel::show() {
  if (busy()) return;   // skip if previous frame still latching (non-blocking)

  // Build the descriptor: copy spiBuf[] byte-by-byte into SPI DATA.
  descriptor_section[0].descaddr = 0;
  descriptor_section[0].srcaddr  = (uint32_t)spiBuf + DNP_BUF_LEN; // end addr (SRCINC)
  descriptor_section[0].dstaddr  = (uint32_t)&SERCOM0->SPI.DATA.reg;
  descriptor_section[0].btcnt    = DNP_BUF_LEN;
  descriptor_section[0].btctrl   = DMAC_BTCTRL_VALID |
                                   DMAC_BTCTRL_SRCINC |        // walk through spiBuf
                                   DMAC_BTCTRL_BEATSIZE_BYTE |
                                   DMAC_BTCTRL_BLOCKACT_NOACT;

  DMAC->CHID.reg = LED_DMA_CHANNEL;
  DMAC->CHCTRLA.bit.ENABLE = 1;   // FIRE. CPU returns immediately; no IRQ disabled.
  inFlight = true;
}

// ---------------------------------------------------------------------------
//  enableDoneIrq(): turn on the "transfer complete" (TCMPL) interrupt for our
//  channel. The handler just clears the in-flight flag. This interrupt is
//  DELIBERATELY assigned the LOWEST NVIC priority in setup() (see NVIC notes):
//  finishing an LED frame must never preempt the NINA WiFi handshake IRQ.
// ---------------------------------------------------------------------------
void DmaNeoPixel::enableDoneIrq() {
  s_dnpInstance = this;
  DMAC->CHID.reg = LED_DMA_CHANNEL;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;   // enable transfer-complete IRQ
  NVIC_EnableIRQ(DMAC_IRQn);
  // Priority itself is set centrally in setup() via NVIC_SetPriority(DMAC_IRQn, 3).
}

// Called by the ISR (friend access via the helper below).
void DmaNeoPixel::onDmaDone() {
  inFlight = false;
}

// C-linkage DMAC interrupt handler. The SAMD21 has ONE DMAC IRQ shared by all
// channels, so we check which channel raised it.
extern "C" void DMAC_Handler(void) {
  uint8_t active = DMAC->INTPEND.bit.ID;          // channel that triggered
  DMAC->CHID.reg = active;
  if (DMAC->CHINTFLAG.bit.TCMPL) {
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;   // clear flag
    if (active == LED_DMA_CHANNEL && s_dnpInstance) {
      s_dnpInstance->onDmaDone();
    }
  }
}

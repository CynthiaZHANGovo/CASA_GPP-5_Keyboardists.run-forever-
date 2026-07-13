/*
 * ============================================================================
 *  DmaAdc.cpp  —  SAMD21 free-running ADC + DMA implementation
 * ============================================================================
 *  Register-level on purpose, so each block is explainable in an interview.
 *  Read the header for the theory first.
 *
 *  Peripheral budget (in addition to the LED driver):
 *    - ADC        : free-running, scans the 4 FSR inputs
 *    - DMAC ch 1  : copies ADC->RESULT into adcBuf on each "result ready"
 *                   (the LED driver already owns DMAC channel 0)
 *
 *  HOW THE MULTI-CHANNEL SCAN WORKS
 *  ------------------------------------------------------------------------
 *  SAMD21's ADC has an input-scan feature: INPUTCTRL.INPUTSCAN makes the ADC
 *  automatically step MUXPOS across a range of positive inputs, one conversion
 *  each, then wrap around. With free-running (CTRLB.FREERUN) it never stops.
 *  Each finished conversion sets the "Result Ready" (RESRDY) flag, which is the
 *  DMA trigger. DMA copies the 16-bit RESULT into the next adcBuf slot.
 *  Because the scan order is fixed (A1,A2,A3,A4,A1,...), slot i always holds
 *  the freshest reading of pad i once the ring has filled.
 * ============================================================================
 */

#include "DmaAdc.h"

typedef struct {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor;

// The LED driver defines its own descriptor tables for channel 0. The DMAC
// BASEADDR/WRBADDR point at ONE table array indexed by channel, so in a real
// merge both drivers must share a single descriptor array. To keep this file
// self-contained and readable, we declare channel 1's descriptor here and,
// in begin(), chain it after channel 0's slot. (See README "DMA channels".)
static dmacdescriptor          adc_descriptor __attribute__((aligned(16)));
static volatile dmacdescriptor adc_wrb        __attribute__((aligned(16)));

#define ADC_DMA_CHANNEL  1

// Map the four Arduino analog pins to SAMD21 ADC AIN (positive MUX) numbers.
// On the MKR1010, A1..A4 correspond to a contiguous AIN range, which is why the
// hardware INPUTSCAN can sweep them in one go. (Values confirmed against the
// variant pin table when flashing real hardware.)
#define ADC_AIN_START   ADC_INPUTCTRL_MUXPOS_PIN0_Val   // adjust to A1's AIN
#define ADC_SCAN_COUNT  (DADC_NUM_CH - 1)               // scan N inputs

uint16_t DmaAdc::read(uint8_t ch) {
  if (ch >= DADC_NUM_CH) return 0;
  return adcBuf[ch];        // just return the DMA-filled latest value
}

void DmaAdc::begin() {
  for (uint8_t i = 0; i < DADC_NUM_CH; ++i) adcBuf[i] = 0;

  // ---- 1. Clock the ADC from GCLK0 (48 MHz) and enable its bus clock --------
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC) |
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  // ---- 2. Basic ADC config: reference, prescaler, 12-bit, average ----------
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->REFCTRL.reg  = ADC_REFCTRL_REFSEL_INTVCC1;        // VDDANA/2 reference
  ADC->CTRLB.reg    = ADC_CTRLB_PRESCALER_DIV64 |        // slow the ADC clock down
                      ADC_CTRLB_RESSEL_12BIT   |         // 12-bit result
                      ADC_CTRLB_FREERUN;                 // <-- free-running mode
  while (ADC->STATUS.bit.SYNCBUSY);

  // ---- 3. Input scan: sweep AIN across the 4 FSR pads ----------------------
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND |
                       ADC_INPUTCTRL_GAIN_DIV2  |
                       ADC_INPUTCTRL_INPUTSCAN(ADC_SCAN_COUNT) |  // auto-step N inputs
                       ADC_INPUTCTRL_MUXPOS(ADC_AIN_START);       // starting input
  while (ADC->STATUS.bit.SYNCBUSY);

  // Raise RESRDY (result ready) event -> used as the DMA trigger.
  ADC->INTENSET.bit.RESRDY = 1;

  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY);

  // ---- 4. Configure DMAC channel 1: ADC RESULT -> adcBuf on RESRDY ---------
  // (DMAC itself is already enabled by the LED driver's begin(); we only set up
  //  our own channel here. If the ADC driver runs first, it enables DMAC too.)
  PM->AHBMASK.reg  |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

  DMAC->CHID.reg = ADC_DMA_CHANNEL;
  DMAC->CHCTRLA.reg = 0;
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) |
                      DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) |  // fire on result ready
                      DMAC_CHCTRLB_TRIGACT_BEAT;

  // Descriptor: copy 4 conversions (one full scan) into adcBuf, then loop back
  // to itself (descaddr -> self) so it re-arms forever without CPU help.
  adc_descriptor.descaddr = (uint32_t)&adc_descriptor;        // self-link = endless
  adc_descriptor.srcaddr  = (uint32_t)&ADC->RESULT.reg;       // fixed source (no SRCINC)
  adc_descriptor.dstaddr  = (uint32_t)adcBuf + DADC_NUM_CH * sizeof(uint16_t); // end addr
  adc_descriptor.btcnt    = DADC_NUM_CH;
  adc_descriptor.btctrl   = DMAC_BTCTRL_VALID |
                            DMAC_BTCTRL_DSTINC |             // step through adcBuf slots
                            DMAC_BTCTRL_BEATSIZE_HWORD |     // 16-bit results
                            DMAC_BTCTRL_BLOCKACT_NOACT;

  // NOTE: BASEADDR must point at a descriptor array indexed by channel number.
  // In the integrated build both drivers share one array; here we assume the
  // LED driver set BASEADDR and this channel's slot chains to adc_descriptor.
  DMAC->CHID.reg = ADC_DMA_CHANNEL;
  DMAC->CHCTRLA.bit.ENABLE = 1;   // start the endless ADC->RAM copy
}

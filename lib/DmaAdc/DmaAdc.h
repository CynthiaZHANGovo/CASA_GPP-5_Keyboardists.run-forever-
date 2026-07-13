/*
 * ============================================================================
 *  DmaAdc.h  —  Free-running multi-channel ADC with DMA on SAMD21 (MKR1010)
 * ============================================================================
 *
 *  WHY THIS EXISTS
 *  ------------------------------------------------------------------------
 *  The original sketch read the four FSR pads with blocking analogRead() in a
 *  loop. analogRead() starts one conversion, then BUSY-WAITS (spins the CPU)
 *  until that one conversion finishes (~tens of microseconds each), four times
 *  per sample tick. The CPU is stalled the whole time and does nothing useful.
 *
 *  This driver instead runs the ADC in FREE-RUNNING mode and lets the DMA
 *  controller copy each finished conversion straight into a RAM buffer:
 *
 *    ADC auto-sequences pads A1->A2->A3->A4->A1...  (hardware scan, no CPU)
 *        every time one conversion completes, it raises a "result ready" event
 *        DMA copies ADC->RESULT into adcBuf[channel]                (no CPU)
 *    TaskSensing just reads the latest adcBuf[] values               (instant)
 *
 *  So sampling stops being "CPU waits for the ADC" and becomes "ADC fills a
 *  buffer in the background, CPU reads whenever it likes". This is the same
 *  DMA idea as the LED driver, but in the opposite direction:
 *      LED : SRAM buffer -> SPI DATA   (memory -> peripheral)
 *      ADC : ADC RESULT  -> SRAM buffer (peripheral -> memory)
 *
 *  It also lays the groundwork for low power (SleepWalking): a free-running ADC
 *  driven by the event system can keep sampling while the CPU is asleep.
 *
 *  NOTE ON CHANNELS: SAMD21's ADC has ONE result register; to scan several
 *  inputs it steps an input MUX (INPUTCTRL.MUXPOS) across pins. We advance the
 *  MUX after each conversion and route each result to the matching adcBuf slot.
 * ============================================================================
 */

#ifndef DMA_ADC_H
#define DMA_ADC_H

#include <Arduino.h>

#define DADC_NUM_CH   4        // four FSR pads

class DmaAdc {
public:
  void begin();                       // configure ADC free-run + DMA channel
  uint16_t read(uint8_t ch);          // latest value for channel 0..3 (non-blocking)

private:
  volatile uint16_t adcBuf[DADC_NUM_CH];   // DMA writes finished conversions here
};

#endif // DMA_ADC_H

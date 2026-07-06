/*
 * ============================================================================
 *  DmaNeoPixel.h  —  Non-blocking WS2812B driver for SAMD21 (Arduino MKR1010)
 * ============================================================================
 *
 *  WHY THIS FILE EXISTS (the load-bearing story of this whole project)
 *  ------------------------------------------------------------------------
 *  The stock Adafruit_NeoPixel::show() bit-bangs the WS2812B waveform on a GPIO
 *  pin and MUST call __disable_irq() (noInterrupts) for the entire transfer,
 *  because the WS2812B timing tolerance is +/-150ns and any interrupt would
 *  stretch a pulse and corrupt the frame.
 *
 *  On the MKR WiFi 1010 the NINA-W102 WiFi module talks to the SAMD21 over SPI
 *  (SERCOM1) and relies on an interrupt (NINA ACK line) to handshake each SPI
 *  transaction. So every time the LEDs refreshed by bit-bang, global interrupts
 *  were masked for ~tens to hundreds of microseconds, the NINA handshake IRQ was
 *  lost, the WiFiNINA SPI transaction timed out, and the WiFi link dropped.
 *
 *  THE FIX: drive WS2812B from a SECOND SPI peripheral (SERCOM) whose MOSI line
 *  is fed BY THE DMA CONTROLLER (DMAC), not the CPU. The DMA copies a pre-encoded
 *  byte buffer from SRAM into the SPI DATA register, triggered by "SPI TX buffer
 *  empty". The CPU kicks off the transfer and returns immediately — interrupts
 *  are NEVER disabled — so the NINA handshake IRQ is never starved.
 *
 *  HOW WS2812B IS ENCODED INTO SPI BYTES (this is what "DMA hardware offload" means)
 *  ------------------------------------------------------------------------
 *  WS2812B is an 800kHz return-to-zero code. Each bit = 1.25us, distinguished by
 *  HIGH pulse width:  '0' ~= 0.4us high,  '1' ~= 0.8us high.
 *
 *  We run SPI at ~2.4MHz, so 1 SPI bit ~= 0.4167us. Then:
 *      WS2812 "0"  ->  SPI bits  100   (0.42us high, 0.83us low)  [valid 0 code]
 *      WS2812 "1"  ->  SPI bits  110   (0.83us high, 0.42us low)  [valid 1 code]
 *  => 3 SPI bits encode 1 WS2812 bit, so 1 colour byte (8 bits) -> 24 SPI bits
 *     = exactly 3 SPI bytes. One LED = 3 colour bytes (GRB) = 9 SPI bytes.
 *
 *  28 LEDs * 9 bytes = 252 bytes of waveform + a trailing reset gap (>=50us low,
 *  emitted as ~40 zero bytes). Total buffer ~292 bytes — trivial against 32KB SRAM.
 *
 *  WHAT THE DMA ACTUALLY MOVES: the pre-encoded SPI byte buffer (the 'spiBuf'
 *  below), SRAM -> SERCOM SPI DATA, one byte per "TX empty" trigger, fully in
 *  hardware. If an interview asks "what does your DMA carry?" — it carries this.
 * ============================================================================
 */

#ifndef DMA_NEOPIXEL_H
#define DMA_NEOPIXEL_H

#include <Arduino.h>

#define DNP_NUM_LEDS      28          // 4 strips * 7 LEDs (physical hardware)
#define DNP_BYTES_PER_LED 9           // 3 colour bytes * 3 SPI bytes/colour byte
#define DNP_RESET_BYTES   40          // ~40 * 0.42us/byte*8... padded low for >50us reset
#define DNP_BUF_LEN       (DNP_NUM_LEDS * DNP_BYTES_PER_LED + DNP_RESET_BYTES)

// One linear 8-bit colour value (0..255) expands to 3 SPI bytes (24 SPI bits).
// We precompute the 3-byte pattern for any input byte at runtime by walking bits.

class DmaNeoPixel {
public:
  void begin();                               // configure SERCOM-SPI + DMAC + descriptor
  void enableDoneIrq();                        // turn on the "transfer complete" interrupt
  void setPixel(uint16_t idx, uint8_t r, uint8_t g, uint8_t b);
  void clear();
  void show();                                // kick the DMA; returns immediately (non-blocking)
  bool busy();                                // true while a DMA frame is still transmitting
  void onDmaDone();                           // called from the DMAC ISR only (clears in-flight)

private:
  uint8_t  spiBuf[DNP_BUF_LEN];               // encoded waveform fed to DMA
  volatile bool inFlight = false;

  void encodeByte(uint8_t value, uint8_t *dst);  // 1 colour byte -> 3 SPI bytes
};

#endif // DMA_NEOPIXEL_H

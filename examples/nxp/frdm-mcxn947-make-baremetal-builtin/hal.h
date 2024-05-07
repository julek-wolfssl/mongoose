// Copyright (c) 2023 Cesanta Software Limited
// All rights reserved

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define LED1 PIN(0, 10)
#define LED2 PIN(0, 27)
#define LED3 PIN(1, 2)

#ifndef UART_DEBUG
#define UART_DEBUG LPUART4
#endif

#include "MCXN947_cm33_core0.h"

#define BIT(x) (1UL << (x))
#define CLRSET(reg, clear, set) ((reg) = ((reg) & ~(clear)) | (set))
#define PIN(bank, num) ((bank << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

void hal_init(void);
size_t hal_ram_free(void);
size_t hal_ram_used(void);

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

#define SYS_FREQUENCY 150000000UL

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_HIGH };
enum { GPIO_PULL_NONE, GPIO_PULL_DOWN, GPIO_PULL_UP };
static inline GPIO_Type *gpio_bank(uint16_t pin) {
  static GPIO_Type *const g[] = GPIO_BASE_PTRS;
  return g[PINBANK(pin)];
}

static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  static PORT_Type *const p[] = PORT_BASE_PTRS;
  PORT_Type *port = p[PINBANK(pin)];
  GPIO_Type *gpio = gpio_bank(pin);
  uint32_t mask = (uint32_t) BIT(PINNO(pin));
  bool dopull = pull > 0;
  if (dopull) --pull;
  if (gpio != GPIO5) {
    SYSCON->AHBCLKCTRL0 |=
        (1 << (SYSCON_AHBCLKCTRL0_GPIO0_SHIFT + PINBANK(pin))) |
        (1 << (SYSCON_AHBCLKCTRL0_PORT0_SHIFT + PINBANK(pin)));
  };
  port->PCR[PINNO(pin)] = PORT_PCR_IBE(1) | PORT_PCR_MUX(af) | PORT_PCR_DSE(1) |
                          PORT_PCR_ODE(type) |
                          PORT_PCR_SRE(speed != GPIO_SPEED_HIGH) |
                          PORT_PCR_PE(dopull) | PORT_PCR_PS(pull);
  gpio->ICR[PINNO(pin)] = GPIO_ICR_ISF_MASK;
  if (mode == GPIO_MODE_INPUT) {
    gpio->PDDR &= ~mask;
  } else {
    gpio->PDDR |= mask;
  }
}

static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_LOW,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_LOW,
            GPIO_PULL_NONE, 0);
}

static inline bool gpio_read(uint16_t pin) {
  GPIO_Type *gpio = gpio_bank(pin);
  return gpio->PDR[PINNO(pin)];
}

static inline void gpio_write(uint16_t pin, bool value) {
  GPIO_Type *gpio = gpio_bank(pin);
  if (value) {
    gpio->PDR[PINNO(pin)] = 1;
  } else {
    gpio->PDR[PINNO(pin)] = 0;
  }
}

static inline void gpio_toggle(uint16_t pin) {
  GPIO_Type *gpio = gpio_bank(pin);
  uint32_t mask = (uint32_t) BIT(PINNO(pin));
  gpio->PTOR = mask;
}

// MCU-Link UART (P1_9/8; FC4_P1/0)
// Arduino J1_2/4 UART (P4_3/2; FC2_P3/2)
// 33.3.23 LP_FLEXCOMM clocking
// 66.2.4 LP_FLEXCOMM init
// 66.5 LPUART
static inline void uart_init(LPUART_Type *uart, unsigned long baud) {
  static LP_FLEXCOMM_Type *const f[] = LP_FLEXCOMM_BASE_PTRS;
  uint8_t af = 2, fc = 0;    // Alternate function, FlexComm instance
  uint16_t pr = 0, pt = 0;   // pins
  uint32_t freq = 12000000;  // fro_12_m
  if (uart == LPUART2) fc = 2, pt = PIN(4, 3), pr = PIN(4, 2);
  if (uart == LPUART4) fc = 4, pt = PIN(1, 9), pr = PIN(1, 8);

  SYSCON->AHBCLKCTRL1 |= (1 << (SYSCON_AHBCLKCTRL1_FC0_SHIFT + fc));
  SYSCON->PRESETCTRL1 |= (1 << (SYSCON_PRESETCTRL1_FC0_RST_SHIFT + fc));
  SYSCON->PRESETCTRL1 &= ~(1 << (SYSCON_PRESETCTRL1_FC0_RST_SHIFT + fc));
  SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_FRO_HF_ENA_MASK;  // enable FRO_12M
  SYSCON->FCCLKSEL[fc] = SYSCON_FCCLKSEL_SEL(2);  // clock from FRO_12M / 1
  SYSCON->FLEXCOMMCLKDIV[fc] = SYSCON_FLEXCOMMXCLKDIV_FLEXCOMMCLKDIV_DIV(0);
  LP_FLEXCOMM_Type *flexcomm = f[fc];
  flexcomm->PSELID = LP_FLEXCOMM_PSELID_PERSEL(1);  // configure as UART
  gpio_init(pt, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_LOW,
            GPIO_PULL_UP, af);
  gpio_init(pr, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_LOW,
            GPIO_PULL_UP, af);

  uart->GLOBAL |= LPUART_GLOBAL_RST_MASK;  // reset, CTRL = 0, defaults
  uart->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;
  // use a weird oversample ratio of 26x to fit specs, standard 16x won't do
  CLRSET(uart->BAUD,
         LPUART_BAUD_OSR_MASK | LPUART_BAUD_SBR_MASK | LPUART_BAUD_SBNS_MASK,
         LPUART_BAUD_OSR(26 - 1) | LPUART_BAUD_SBR(freq / (26 * baud)));
  CLRSET(uart->CTRL,
         LPUART_CTRL_PE_MASK | LPUART_CTRL_M_MASK | LPUART_CTRL_ILT_MASK |
             LPUART_CTRL_IDLECFG_MASK,
         LPUART_CTRL_IDLECFG(1) | LPUART_CTRL_ILT(1) |
             LPUART_BAUD_SBNS(0));  // no parity, idle 2 chars after 1 stop bit
  uart->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

static inline void uart_write_byte(LPUART_Type *uart, uint8_t byte) {
  uart->DATA = byte;
  while ((uart->STAT & LPUART_STAT_TDRE_MASK) == 0) spin(1);
}
static inline void uart_write_buf(LPUART_Type *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

static inline void rng_init(void) {
}
static inline uint32_t rng_read(void) {
  return 0;
}

// - PHY and MAC clocked via a 50MHz oscillator, P1_4 (ENET0_TXCLK)
// - 33.3.30 ENET clocking
// - PHY RST connected to P5_8
// - PHY RXD0,1,DV = 1 on RST enable autonegotiation, no hw pull-ups
static inline void ethernet_init(void) {
  SYSCON->AHBCLKCTRL2 |= SYSCON_AHBCLKCTRL2_ENET_MASK;  // enable bus clk
  // no internal clock in clk_rmii, set for RMII mode
  SYSCON->ENETRMIICLKSEL = SYSCON_ENETRMIICLKSEL_SEL(7);
  SYSCON->ENETRMIICLKDIV = SYSCON_ENETRMIICLKDIV_DIV(0);
  SYSCON0->ENET_PHY_INTF_SEL |= SYSCON_ENET_PHY_INTF_SEL_PHY_SEL_MASK;
  SYSCON0->PRESETCTRL2 = SYSCON_PRESETCTRL2_ENET_RST_MASK;  // reset MAC
  SYSCON0->PRESETCTRL2 &= ~SYSCON_PRESETCTRL2_ENET_RST_MASK;
  gpio_init(PIN(5, 8), GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_LOW,
            GPIO_PULL_NONE, 0);  // set P5_8 as GPIO (PHY \RST)
  gpio_write(PIN(5, 8), 0);      // reset PHY
  gpio_init(PIN(1, 4), GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 9);  // set P1_4 as ENET0_TXCLK
  gpio_init(PIN(1, 5), GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 9);  // set P1_5 as ENET0_TXEN
  gpio_init(PIN(1, 6), GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 9);  // set P1_6 as ENET0_TXD0
  gpio_init(PIN(1, 7), GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 9);  // set P1_7 as ENET0_TXD1
  gpio_init(PIN(1, 13), GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_UP, 9);  // set P1_13 as ENET0_RXDV
  gpio_init(PIN(1, 14), GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_UP, 9);  // set P1_14 as ENET0_RXD0
  gpio_init(PIN(1, 15), GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_UP, 9);  // set P1_15 as ENET0_RXD1
  gpio_init(PIN(1, 20), GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 9);  // set P1_20 as ENET0_MDC
  gpio_init(PIN(1, 21), GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 9);   // set P1_21 as ENET0_MDIO
  spin(10000);                    // keep PHY RST low for a while
  gpio_write(PIN(5, 8), 1);       // deassert RST
  NVIC_EnableIRQ(ETHERNET_IRQn);  // Setup Ethernet IRQ handler
}

// 33.2 Figure 127 SCG main clock
static inline void clock_init(void) {
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  // Enable FPU
  asm("DSB");
  asm("ISB");
}

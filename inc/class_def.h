#ifndef CLASS_DEFS
#define CLASS_DEFS
// Generic DEFs
#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 16
#define BIT5 32
#define BIT6 64
#define BIT7 128

#define LED3 0x2000  // Bit 13
#define LED4 0x1000  // Bit 12
#define LED5 0x4000  // Bit 14
#define LED6 0x8000  // Bit 15


// SysTick Timer Registers DEFs
#define STK_BASE    (volatile uint32_t) 0xE000E010 // Base address for the SysTick Timer Registers
#define STK_CTRL  (volatile uint32_t *) 0xE000E010 // SysTick control and status register
#define STK_LOAD  (volatile uint32_t *) 0xE000E014 // SysTick reload value register
#define STK_VALUE (volatile uint32_t *) 0xE000E018 // SysTick current value register
#define STK_CALIB (volatile uint32_t *) 0xE000E01C // SysTick calibration value register
// SysTick Timer Registers DEFs End

// System Register DEFs
#define RCC     (volatile uint32_t*) 0x40023800
#define AHB1ENR (volatile uint32_t*) 0x40023830
// System Register DEFs Ends

// GPIO DEFs
#define GPIOD	(volatile uint32_t)  0x40020C00	// GPIOD Base Address
#define MODER	(volatile uint32_t*) 0x40020C00	// 
#define OTYPER	(volatile uint32_t*) 0x40020C04	//
#define OSPEEDR	(volatile uint32_t*) 0x40020C08	//
#define PUPDR	(volatile uint32_t*) 0x40020C0C	//
#define BSRRL	(volatile uint16_t*) 0x40020C18	//
#define BSRRH	(volatile uint16_t*) 0x40020C1A	//


// GPIO DEFS ends

// Mask definition
#define PUPDR_AND_MASK 0x00FFFFFF
#define PUPDR_OR_MASK 0x55000000

#define OTYPER_MASK 0xFFFF0FFF
#define MODER_AND_MASK 0x00FFFFFF
#define MODER_OR_MASK 0x55000000

#endif

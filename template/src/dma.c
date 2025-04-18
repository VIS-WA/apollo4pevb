#include <stdio.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_gpio.h"
#include "am_hal_iom.h"
#include "am_util_delay.h"
#include "uart.h"

// IOM module and SPI configuration
#define IOM_MODULE       1              // using IOM1
#define SPI_CLOCK_FREQ   AM_HAL_IOM_16MHZ  // 1 MHz SPI clock (adjust as needed)
#define SPI_MODE         AM_HAL_IOM_SPI_MODE_0 // CPOL=0, CPHA=0 for ADS7042
#define ADC_CS_NUM       AM_BSP_IOM1_CS_CHNL              // using chip select 0 (IOM1 nCE0 for ADS7042)

// Buffer for IOM Command Queue (DMA transactions) in internal SRAM (TCM), 256 words (1024 bytes), 16-byte aligned
static uint32_t g_IOMCQBuffer[256] __attribute__((aligned(16)));

// Buffer for received data (2 bytes). Use a 32-bit variable for aligned DMA access.
static uint32_t g_ui32RxData;

// Flag and value for ADC result
static volatile bool g_bDmaDone = false;
static volatile uint16_t g_ui16AdcValue = 0;

// IOM handle (global for ISR use)
static void *g_IOMHandle;

void iom_set_up(uint32_t iomModule) {
    // Initialize IOM1
    uint32_t status;
    status = am_hal_iom_initialize(iomModule, &g_IOMHandle);
    if (status != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error - IOM initialize failed (0x%X)\n", status);
        return;
    }
    // Power on IOM1
    status = am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    if (status != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error - IOM power-up failed (0x%X)\n", status);
        return;
    }
}

// DMA transfer completion callback
void iom_dma_callback(void *pCallbackCtxt, uint32_t transactionStatus) {
    if (transactionStatus != AM_HAL_STATUS_SUCCESS) {
        // Print error status if transaction failed
        am_util_stdio_printf("DMA transaction error: 0x%X\n", transactionStatus);
    }
    // Combine the two received bytes into a 12-bit value
    // uint16_t rawValue = (uint16_t)(g_ui32RxData & 0xFFFF);  // lower 16 bits of rx buffer
    // g_ui16AdcValue = (rawValue >> 2) & 0x0FFF;  // shift down 2 dummy bits to get 12-bit result

    uint8_t *pRx = (uint8_t *)&g_ui32RxData;
    uint16_t raw = ((uint16_t)pRx[0] << 8) | pRx[1];

    // Discard the 2 extra bits (the two most significant bits), leaving a 14-bit frame.
    // Extract the 12-bit ADC conversion result.
    g_ui16AdcValue = (raw >> 2) & 0x0FFF;
    g_bDmaDone = true;  // signal completion to main loop
}

// IOM Interrupt Service Routine (for IOM1)
void am_iomaster1_isr(void) {
    uint32_t ui32Status;
    // Get and clear enabled interrupts
    am_hal_iom_interrupt_status_get(g_IOMHandle, false, &ui32Status);
    am_hal_iom_interrupt_clear(g_IOMHandle, ui32Status);
    // Service the interrupts: will invoke callback if transaction completed&#8203;:contentReference[oaicite:10]{index=10}
    am_hal_iom_interrupt_service(g_IOMHandle, ui32Status);
}

int main(void) {

    uart_init();
    iom_set_up(IOM_MODULE);
    

    // Configure IOM1 for SPI with DMA (non-blocking)
    am_hal_iom_config_t spiConfig = {
        .eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq        = SPI_CLOCK_FREQ,
        .eSpiMode             = SPI_MODE,
        .ui32NBTxnBufLength   = 256,               // length in words of CQ buffer
        .pNBTxnBuf            = g_IOMCQBuffer      // pointer to our CQ buffer in SRAM
    };

    uint32_t status;
    status = am_hal_iom_configure(g_IOMHandle, &spiConfig);
    if (status != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error - IOM configure failed (0x%X)\n", status);
        return -1;
    }

    // Enable IOM1 (this initializes the command queue for DMA)
    status = am_hal_iom_enable(g_IOMHandle);
    if (status != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error - IOM enable failed (0x%X)\n", status);
        return -1;
    }

    // Configure SPI pins (using Apollo4 Plus GPIO configurations - replace with actual pins)
    // For example purposes, assume predefined macros for IOM1 pins:
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK,  g_AM_BSP_GPIO_IOM1_SCK);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, g_AM_BSP_GPIO_IOM1_MISO);
    // am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MOSI, g_AM_BSP_IOM1_MOSI);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS, g_AM_BSP_GPIO_IOM1_CS);
    // (Actual pin numbers and configs depend on board layout and Ambiq BSP)

    // Enable IOM1 interrupts for DMA complete and errors
    am_hal_iom_interrupt_enable(g_IOMHandle, AM_HAL_IOM_INT_DCMP | AM_HAL_IOM_INT_CQERR | AM_HAL_IOM_INT_DERR);
    NVIC_EnableIRQ(IOMSTR1_IRQn);  // Enable NVIC interrupt for IOM1 (Apollo4 IOMSTR0 interrupt)

    am_util_stdio_printf("Starting ADS7042 DMA SPI read...\n");
    while (1) {
        g_bDmaDone = false;

        // Prepare a DMA transaction for 2-byte SPI read
        uint8_t rxBytes = 2;
        am_hal_iom_transfer_t Transaction = {
            .uPeerInfo.ui32SpiChipSelect = ADC_CS_NUM,  // CS0 for ADC
            .ui32InstrLen    = 0,          // No command bytes (just read data)
            .ui64Instr       = 0,          // Not used since InstrLen=0
            .ui32NumBytes    = rxBytes,    // read 2 bytes (14-bit frame)
            .eDirection      = AM_HAL_IOM_RX,
            .pui32TxBuffer   = NULL,       // no TX data (just clock out zeros)
            .pui32RxBuffer   = &g_ui32RxData, // receive buffer (32-bit aligned)
            .bContinue       = false,      // deassert CS after this transfer
            .ui8RepeatCount  = 0,
            .ui8Priority     = 1,          // high priority DMA
            .ui32PauseCondition = 0,
            .ui32StatusSetClr   = 0
        };

        status = am_hal_iom_nonblocking_transfer(g_IOMHandle, &Transaction, iom_dma_callback, NULL);
        am_util_stdio_printf("IOM nonblocking transfer status: 0x%X\n", status);
        if (status != AM_HAL_STATUS_SUCCESS) {
            am_util_stdio_printf("Error - DMA transfer start failed (0x%X)\n", status);
        } else {

            // Wait for DMA to complete (flag set in callback)
            while (!g_bDmaDone) {
                // Enter sleep to wait for IRQ (optional):
                // am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
            }
            // At this point, g_ui16AdcValue has the 12-bit ADC result
            am_util_stdio_printf("ADS7042 ADC 12-bit value = %u\n", g_ui16AdcValue);
        }

        am_util_delay_ms(1000);  // delay 1s between readings (adjust as needed)
    }
}


/*
16th April 2025

#include "am_mcu_apollo.h"
#include "uart.h"

#define NUM_SAMPLES 1000000  // 1,000,000 samples

static volatile uint16_t adc_buffer_mram[NUM_SAMPLES] __attribute__((section(".shared"))) ;  // >1M then shared memory, else TCM
static uint32_t iom_dma_buffer[256] ;  // DMA transaction buffer for IOM (command queue)

// Handles IOM
static void *IOMHandle;

// Flags and counters for transfer status
static volatile bool transfer_done = false;
static volatile uint32_t sample_count = 0;

// IOM1 interrupt service routine (ISR)
void am_iomaster1_isr(void) {
    uint32_t ui32Status;
    // Get and clear the interrupt status
    am_hal_iom_interrupt_status_get(IOMHandle, false, &ui32Status);
    am_hal_iom_interrupt_clear(IOMHandle, ui32Status);
    // Service the interrupt (will call the callback if a transaction completed)
    am_hal_iom_interrupt_service(IOMHandle, ui32Status);
}

uint32_t spi_transaction(uint32_t count);
// DMA transfer completion callback for each sample
void iom_dma_callback(void *pCallbackCtxt, uint32_t transactionStatus) {
    if (transactionStatus != AM_HAL_STATUS_SUCCESS) {
        // Handle error (optional: set an error flag, etc.)
        transfer_done = true;
        am_util_stdio_printf("DMA transaction error: 0x%X\n", transactionStatus);
        return;
    }
    // Process the just-received sample: extract 12-bit value from 14-bit frame
    // According to ADS7042 datasheet, first two bits of SDO frame are 0, next 12 bits are data
    // The 16-bit word has data in bits [13:2] (bits 15,14,1,0 are 0). Right-shift by 2 to get 12-bit result.
    // uint16_t raw_value = adc_buffer_mram[sample_count]; 
    // adc_buffer_mram[sample_count] = (raw_value >> 2) & 0x0FFF;  // Keep only the 12-bit ADC result

    // uint8_t *pRx = (uint8_t *)&adc_buffer_mram[sample_count];
    // uint16_t raw = ((uint16_t)pRx[0] << 8) | pRx[1];
    // raw = (raw >> 2) & 0x0FFF;
    // adc_buffer_mram[sample_count] = raw;  // Store the 12-bit ADC result in the buffer

    sample_count++;
    if (sample_count < NUM_SAMPLES) {
        // Schedule the next sample read (1 transfer = 1 sample) using DMA
        spi_transaction(sample_count);  // Start the next sample read via DMA
    } else {
        // All samples captured
        transfer_done = true;
        am_util_stdio_printf("DMA transfer complete. %u samples captured.\n", NUM_SAMPLES);
    }
}

uint32_t spi_transaction(uint32_t count){
    am_hal_iom_transfer_t Transaction;
        Transaction.uPeerInfo.ui32SpiChipSelect = 0;     // Chip select 0 (configured for ADS7042)
        Transaction.ui32InstrLen = 0;                    // No instruction bytes
        Transaction.ui64Instr    = 0;                    // (Not used for SPI read here)
        Transaction.eDirection   = AM_HAL_IOM_RX;        // Receive from ADC
        Transaction.ui32NumBytes = 2;                    // 2 bytes per sample (16 bits, contains 12-bit data)
        Transaction.pui32TxBuffer = NULL;
        Transaction.pui32RxBuffer = (uint32_t *)&adc_buffer_mram[count];  // store next sample
        Transaction.bContinue    = false;                // De-assert CS after each transfer (no continuous chip select)
        Transaction.ui8Priority  = 1;                    // High priority (if applicable)
        Transaction.ui32PauseCondition = 0;
        Transaction.ui32StatusSetClr   = 0;

        // am_util_stdio_printf("Count: %u\n", count);
        return am_hal_iom_nonblocking_transfer(IOMHandle, &Transaction, iom_dma_callback, NULL);

}


// Set up IOM1 for SPI communication with ADS7042 (16MHz, mode 0, DMA enabled)
void iom_set_up(void) {
    // Initialize IOM1 (SPI master)
    am_hal_iom_initialize(1, &IOMHandle);
    am_hal_iom_power_ctrl(IOMHandle, AM_HAL_SYSCTRL_WAKE, false);

    // Configure IOM1 for SPI mode 0, 16 MHz, with command queue (DMA) enabled
    am_hal_iom_config_t iom_cfg = {
        .eInterfaceMode    = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq     = AM_HAL_IOM_16MHZ,
        .eSpiMode          = AM_HAL_IOM_SPI_MODE_0,  // SPI mode 0 (CPOL=0, CPHA=0)
        .pNBTxnBuf         = iom_dma_buffer,
        .ui32NBTxnBufLength= sizeof(iom_dma_buffer) / 4  // Length in words
    };
    am_hal_iom_configure(IOMHandle, &iom_cfg);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK,  g_AM_BSP_GPIO_IOM1_SCK);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, g_AM_BSP_GPIO_IOM1_MISO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS,   g_AM_BSP_GPIO_IOM1_CS);

    // Enable IOM1 interrupts for DMA complete (and errors)
    am_hal_iom_interrupt_enable(IOMHandle, AM_HAL_IOM_INT_DCMP);
    NVIC_SetPriority(IOMSTR1_IRQn, 2);
    NVIC_EnableIRQ(IOMSTR1_IRQn);

    // Enable the IOM1 module
    am_hal_iom_enable(IOMHandle);
}


int main(void) {
    // Initialize UART for output (disable printing in loop for speed, but for now just init for final prints)
    uart_init();
    am_hal_pwrctrl_low_power_init(); // helps to reduce power consumption
    // Set up SPI interface on IOM1 for ADC
    iom_set_up();

    // Begin ADC sampling loop using DMA
    am_util_stdio_printf("Starting ADC capture of %d samples...\r\n", NUM_SAMPLES);
    // Prepare first SPI DMA transaction (1 sample)
    sample_count = 0;
    transfer_done = false;

    spi_transaction(sample_count);  // Start the first sample read via DMA
    am_util_delay_ms(1000);  // Optional delay to allow for setup time (if needed)
    // Wait for the DMA sampling to complete (no debug printing in this loop for performance)
    while (!transfer_done) {
        // Enter sleep mode while waiting for interrupts (to reduce CPU load during sampling)
        // am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }

    // Capture complete, print the first few samples for verification
    am_util_stdio_printf("ADC capture complete. First 10 samples:\r\n");
        for (uint32_t i = 0; i < 10 && i < NUM_SAMPLES; i++) {
        uint8_t *pRx = (uint8_t *)&adc_buffer_mram[i];
        uint16_t raw = ((uint16_t)pRx[0] << 8) | pRx[1];
        raw = (raw >> 2) & 0x0FFF;
        am_util_stdio_printf("Sample %u: %d\r\n", i, raw);
    }
    return 0;

}

*/
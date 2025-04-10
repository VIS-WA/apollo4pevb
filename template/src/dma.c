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

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "am_mcu_apollo.h"
#include "am_hal_gpio.h"
#include "am_hal_iom.h"
#include "am_util_delay.h"
#include "uart.h"
#include "am_bsp.h"

//*****************************************************************************
// Configuration
//*****************************************************************************

// IOM instance where the ADC is connected (you switched to IOM1).
#define IOM_MODULE         1

// 16MHz SPI clock to allow 16Mbits/s throughput (the max for many Apollo4 boards).
#define SPI_CLOCK_FREQ     AM_HAL_IOM_16MHZ

// ADS7042 is SPI mode 0 (CPOL=0, CPHA=0).
#define SPI_MODE           AM_HAL_IOM_SPI_MODE_0

// Chip Select defined by your BSP for IOM1. (AM_BSP_IOM1_CS_CHNL).
#define ADC_CS_NUM         AM_BSP_IOM1_CS_CHNL

// For 1 MSPS * 1 second = 1,000,000 samples.
// Each sample is 2 bytes, so we need 2 million bytes total.
#define NUM_SAMPLES        10
#define BYTES_PER_SAMPLE   2
#define TOTAL_TRANSFER_BYTES (NUM_SAMPLES * BYTES_PER_SAMPLE)

//*****************************************************************************
// Globals
//*****************************************************************************

// Command Queue buffer for IOM DMA, 256 words (1024 bytes) aligned to 16 bytes.
static uint32_t g_IOMCQBuffer[256] __attribute__((aligned(16)));

// Allocate a 2MB buffer for raw 2-byte samples. Must be in available SRAM.
// Make sure your linker script places this in valid memory.
static uint8_t g_AdcSamples[TOTAL_TRANSFER_BYTES] __attribute__((aligned(16)));
// static uint32_t g_ui32RxData;
// Flag to indicate DMA completion
static volatile bool g_bDmaComplete = false;

// IOM handle
static void *g_IOMHandle = NULL;

//*****************************************************************************
// IOM Interrupt Service Routine for IOM1
//*****************************************************************************
void am_iomaster1_isr(void)
{
    uint32_t ui32Status = 0;
    // Read and clear the interrupt status
    am_hal_iom_interrupt_status_get(g_IOMHandle, false, &ui32Status);
    am_hal_iom_interrupt_clear(g_IOMHandle, ui32Status);
    // Service the interrupts: will call our callback upon DMA completion
    am_hal_iom_interrupt_service(g_IOMHandle, ui32Status);
}

//*****************************************************************************
// DMA callback
//*****************************************************************************
static void
iom_dma_callback(void *pCallbackCtxt, uint32_t transactionStatus)
{
    if (transactionStatus != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("DMA Error: 0x%X\n", transactionStatus);
    }
    else
    {
        // Indicate transfer complete
        g_bDmaComplete = true;
    }
}

//*****************************************************************************
// Configure and Enable the IOM
//*****************************************************************************
static void
iom_setup(uint32_t iomModule)
{
    uint32_t status;

    // Initialize the IOM
    status = am_hal_iom_initialize(iomModule, &g_IOMHandle);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM initialize failed (0x%X)\n", status);
        return;
    }

    // Power on the IOM
    status = am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM power-up failed (0x%X)\n", status);
        return;
    }

    // Configure the IOM for SPI with DMA (non-blocking)
    am_hal_iom_config_t spiConfig =
    {
        .eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq        = SPI_CLOCK_FREQ,
        .eSpiMode             = SPI_MODE,
        .ui32NBTxnBufLength   = sizeof(g_IOMCQBuffer) / 4, // length in words
        .pNBTxnBuf            = g_IOMCQBuffer
    };

    status = am_hal_iom_configure(g_IOMHandle, &spiConfig);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM configure failed (0x%X)\n", status);
        return;
    }

    // Enable the IOM (this sets up the command queue)
    status = am_hal_iom_enable(g_IOMHandle);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM enable failed (0x%X)\n", status);
        return;
    }

    // Enable SPI pins (MISO, SCK, CS, etc.) for IOM1 as needed
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK,  g_AM_BSP_GPIO_IOM1_SCK);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, g_AM_BSP_GPIO_IOM1_MISO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MOSI, g_AM_BSP_GPIO_IOM1_MOSI);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS,   g_AM_BSP_GPIO_IOM1_CS);

    // Enable IOM interrupts for DMA completion & errors
    am_hal_iom_interrupt_enable(g_IOMHandle,
        AM_HAL_IOM_INT_DCMP | AM_HAL_IOM_INT_CQERR | AM_HAL_IOM_INT_DERR);
    NVIC_EnableIRQ(IOMSTR1_IRQn);
}

//*****************************************************************************
// Main
//*****************************************************************************
int
main(void)
{
    // Initialize UART for prints
    uart_init();

    // Setup the IOM for SPI reads
    iom_setup(IOM_MODULE);
    am_util_stdio_printf("\n\n===== Starting 1MSPS Capture =====\n");


    uint32_t g_ui32RxData;
    for (uint32_t i = 0; i < NUM_SAMPLES; i++)
    
    {

        // Prepare a single large DMA transaction for 2M bytes (1 million samples).
        am_hal_iom_transfer_t Transaction =
        {
            .uPeerInfo.ui32SpiChipSelect = ADC_CS_NUM,
            .ui32InstrLen    = 0,
            .ui64Instr       = 0,
            .ui32NumBytes    = 2,  // 2,000,000 bytes
            .eDirection      = AM_HAL_IOM_RX,
            .pui32TxBuffer   = NULL,
            .pui32RxBuffer   = &g_ui32RxData, 
            .pui32RxBuffer   = &g_AdcSamples, 
            .bContinue       = true,
            .ui8RepeatCount  = 0,     // Single transaction
            .ui8Priority     = 1,     // High priority DMA
            .ui32PauseCondition = 0,
            .ui32StatusSetClr   = 0
        };
    
        // Start the transaction
        uint32_t status = am_hal_iom_nonblocking_transfer(g_IOMHandle,
                                                          &Transaction,
                                                          iom_dma_callback,
                                                          NULL);
    
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            am_util_stdio_printf("Error - DMA transfer start failed (0x%X)\n", status);
            while(1);
        }
    
        // Wait for completion
        while (!g_bDmaComplete)
        {
            // Optionally sleep here
            // am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        }
    
        // Done capturing 1 million samples
        // am_util_stdio_printf("DMA complete! Collected %d samples.\n", NUM_SAMPLES);
    
        // -----------------------------------------------------
        // Optionally parse the raw data in g_AdcSamples here
        // to convert each 2-byte reading into a 12-bit value.
        //

        uint8_t *pRx = (uint8_t *)&g_ui32RxData;
        uint16_t raw = ((uint16_t)pRx[0] << 8) | pRx[1];
        uint16_t adc12 = (raw >> 2) & 0x0FFF; // Discard the 2 extra bits
        am_util_stdio_printf("Sample %d: %u\n", i, adc12);
            // do something with adc12...
    }
        // -----------------------------------------------------
        return 0;
    }
//*****************************************************************************
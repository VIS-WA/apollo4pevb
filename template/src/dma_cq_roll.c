#include "am_mcu_apollo.h"
#include "uart.h"           // Assume uart is already initialized externally.
#include "am_util_stdio.h"  // For am_util_stdio_printf


// -----------------------------------------------------------------------------
// Configuration parameters
// -----------------------------------------------------------------------------
#define NUM_SAMPLES  1000  // 1,000,000 samples to capture.
#define BATCH_SIZE   1000     // User-defined batch size (can be any multiple of 256).
                              // (Note: BATCH_SIZE may exceed 256; the CQ index registers roll over modulo 256.)
#define SPI_CLOCK_FREQ       AM_HAL_IOM_16MHZ    // 16 MHz SPI clock (suitable for 1 MSPS on ADS7042)
#define SPI_MODE             AM_HAL_IOM_SPI_MODE_0 // SPI Mode 0: CPOL=0, CPHA=0
#define ADC_CS_NUM           AM_BSP_IOM1_CS_CHNL   // Use chip‐select 0 on IOM1
#define IOM_MODULE          1                 // Using IOM1 for SPI

// -----------------------------------------------------------------------------
// Global buffers and variables
// -----------------------------------------------------------------------------

// Allocate a buffer for the ADC data (each sample is read as a 16‑bit word).
// Place it in shared SRAM (or TCM) as required and align to 4 bytes (for DMA).
static volatile uint16_t g_ADCBuffer[NUM_SAMPLES] __attribute__((section(".shared"), aligned(4)));

// We also need a buffer for the IOM command queue (CQ) – it must be 16-byte aligned.
// Its size must be large enough to store (BATCH_SIZE * #words/transfer). Choose conservatively.
static uint32_t g_CQBuffer[(int)BATCH_SIZE] __attribute__((section(".shared"), aligned(16)));

// Global IOM handle – used with IOM1.
static void *g_IOMHandle = 0;

// Counters to track progress.
static volatile uint32_t g_totalSamplesAcquired = 0;  // Total samples acquired so far.
static volatile uint32_t g_samplesRemaining = NUM_SAMPLES; // Samples left to schedule.

// Flag indicating that all samples have been acquired.
static volatile bool g_captureDone = false;

// -----------------------------------------------------------------------------
// IOM1 ISR: Must call the HAL interrupt service routine for the IOM module.
// -----------------------------------------------------------------------------
void am_iomaster1_isr(void)
{
    uint32_t ui32Status;
    am_hal_iom_interrupt_status_get(g_IOMHandle, false, &ui32Status);
    am_hal_iom_interrupt_clear(g_IOMHandle, ui32Status);
    am_hal_iom_interrupt_service(g_IOMHandle, ui32Status);
}

// -----------------------------------------------------------------------------
// Callback function for the last transfer in each batch.
// This callback is called only once per batch.
// If more samples remain to be acquired, it refills the CQ with another batch.
// -----------------------------------------------------------------------------
void adc_batch_callback(void *pCallbackCtxt, uint32_t transactionStatus)
{
    if (transactionStatus != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("CQ batch error: 0x%X\n", transactionStatus);
        g_captureDone = true;
        return;
    }
    // A whole batch has just completed.
    // The last transfer of the batch has finished executing.
    // g_totalSamplesAcquired has already been incremented with each transfer.
    // If more samples are needed, enqueue the next batch.
    if (g_samplesRemaining > 0)
    {
        // Enqueue next batch.
        uint32_t batchCount = (g_samplesRemaining >= BATCH_SIZE) ? BATCH_SIZE : g_samplesRemaining;
        for (uint32_t i = 0; i < batchCount; i++)
        {
            // Calculate the index in the ADC data buffer.
            uint32_t index = g_totalSamplesAcquired;
            // Prepare a transfer descriptor.
            am_hal_iom_transfer_t txn = {0};
            txn.uPeerInfo.ui32SpiChipSelect = ADC_CS_NUM;  // Use chip-select 0.
            txn.ui32InstrLen = 0;         // No instruction phase.
            txn.ui64Instr    = 0;
            txn.ui32NumBytes = 2;         // Read 2 bytes per sample.
            txn.eDirection   = AM_HAL_IOM_RX;
            // Set the DMA destination pointer into our ADC buffer.
            txn.pui32RxBuffer = (uint32_t *)&g_ADCBuffer[index];
            txn.bContinue    = false;     // Ensure chip-select toggles.
            txn.ui8Priority  = 1;

            // For all transfers except the last in the batch, no callback.
            // For the last one, set our batch callback.
            am_hal_iom_callback_t cb = (i == (batchCount - 1)) ? adc_batch_callback : NULL;

            uint32_t status = am_hal_iom_nonblocking_transfer(g_IOMHandle, &txn, cb, NULL);
            if (status != AM_HAL_STATUS_SUCCESS)
            {
                am_util_stdio_printf("Enqueue error at sample %u: 0x%X\n", index, status);
            }
            g_totalSamplesAcquired++;
            g_samplesRemaining--;
        }
    }
    else
    {
        // No more samples remaining: signal overall capture complete.
        g_captureDone = true;
    }
}

// -----------------------------------------------------------------------------
// Function to enqueue the first batch into the IOM Command Queue (CQ).
// This function starts the acquisition process.
// -----------------------------------------------------------------------------
void start_adc_capture(void)
{
    // Determine the number of transfers to enqueue in the first batch.
    uint32_t batchCount = (g_samplesRemaining >= BATCH_SIZE) ? BATCH_SIZE : g_samplesRemaining;
    for (uint32_t i = 0; i < batchCount; i++)
    {
        uint32_t index = g_totalSamplesAcquired;
        am_hal_iom_transfer_t txn = {0};
        txn.uPeerInfo.ui32SpiChipSelect = ADC_CS_NUM;
        txn.ui32InstrLen = 0;
        txn.ui64Instr    = 0;
        txn.ui32NumBytes = 2;
        txn.eDirection   = AM_HAL_IOM_RX;
        txn.pui32RxBuffer = (uint32_t *)&g_ADCBuffer[index];
        txn.bContinue    = false;
        txn.ui8Priority  = 1;
        // Only the last transaction in the batch gets the callback.
        am_hal_iom_callback_t cb = (i == (batchCount - 1)) ? adc_batch_callback : NULL;
        uint32_t status = am_hal_iom_nonblocking_transfer(g_IOMHandle, &txn, cb, NULL);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            // am_util_stdio_printf("Enqueue error at sample %u: 0x%X\n", index, status);
        }
        g_totalSamplesAcquired++;
        g_samplesRemaining--;
    }
}

// -----------------------------------------------------------------------------
// IOM and SPI initialization function
// -----------------------------------------------------------------------------
void iom_set_up(void)
{
    uint32_t status;

    // Initialize IOM1.
    status = am_hal_iom_initialize(IOM_MODULE, &g_IOMHandle);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM initialize failed (0x%X)\n", status);
        return;
    }
    // Power on IOM1.
    status = am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM power control failed (0x%X)\n", status);
        return;
    }
    // Configure IOM1 for SPI.
    am_hal_iom_config_t iomCfg = {
        .eInterfaceMode    = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq     = SPI_CLOCK_FREQ,
        .eSpiMode          = SPI_MODE,
        .pNBTxnBuf         = g_CQBuffer,
        .ui32NBTxnBufLength = sizeof(g_CQBuffer)/4
    };
    status = am_hal_iom_configure(g_IOMHandle, &iomCfg);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM configure failed (0x%X)\n", status);
        return;
    }
    // Configure SPI pins (adjust these to your board).
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK,  g_AM_BSP_GPIO_IOM1_SCK);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, g_AM_BSP_GPIO_IOM1_MISO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS,   g_AM_BSP_GPIO_IOM1_CS);
    
    // Enable IOM1 interrupts for command queue completion (CQ events).
    am_hal_iom_interrupt_enable(g_IOMHandle, AM_HAL_IOM_INT_DCMP);
    NVIC_SetPriority(IOMSTR1_IRQn, 1);
    NVIC_EnableIRQ(IOMSTR1_IRQn);
    
    // Finally, enable IOM1.
    status = am_hal_iom_enable(g_IOMHandle);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Error - IOM enable failed (0x%X)\n", status);
    }
}

// -----------------------------------------------------------------------------
// Main function
// -----------------------------------------------------------------------------
int main(void)
{
    // Assume uart_init is defined in uart.h and has already been called externally.
    // (If not, you may call it here.)
    uart_init();
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();
    am_bsp_low_power_init();

    uint32_t ui32retval = am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE);
    if ( ui32retval != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Error with am_hal_pwrctrl_mcu_mode_select(), returned %d.\n", ui32retval);
        return 0;
    }
    am_util_stdio_printf("Set High Performance (HP) Mode.\n");
    // am_bsp_low_power_init();

    // Set up IOM1 for SPI acquisition.
    iom_set_up();
    
    am_util_stdio_printf("Starting ADC capture: %d samples in batches of %d...\r\n", NUM_SAMPLES, BATCH_SIZE);
    
    // Start the first batch.
    start_adc_capture();
    
    // Wait for capture to complete. In the meanwhile, the CQ is refilled in the callbacks.
    while (!g_captureDone)
    {
        // Enter low power sleep to save CPU power.
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
    
    // Capture complete. Post-process and print the first 10 samples.
    am_util_stdio_printf("Capture complete. last few samples (12-bit values):\r\n");
    for (uint32_t i = 0; i < 4; i++)
    {
        uint8_t *pRx = (uint8_t *)&g_ADCBuffer[NUM_SAMPLES - i];
        uint16_t raw = ((uint16_t)pRx[0] << 8) | pRx[1];
        uint16_t adc_value = (raw >> 2) & 0x0FFF;
        am_util_stdio_printf("Sample %u: 0x%03X (%u)\r\n", i, adc_value, adc_value);
    }
    
    // // Optionally, disable IOM and go to sleep.
    // am_hal_iom_disable(g_IOMHandle);
    // while(1)
    // {
    //     am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
    // }
    
    // Return (should never reach here).
    return 0;
}

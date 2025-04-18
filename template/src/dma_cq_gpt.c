// added caching and HPC

#include "am_mcu_apollo.h"
#include "am_hal_iom.h"
#include "am_hal_gpio.h"
#include "am_util_stdio.h"
#include "uart.h"


// Constants for ADC capture
#define IOM_MODULE          1                 // Using IOM1 for SPI
#define ADC_SPI_CS          0                 // Chip-select index for ADS7042 (use CS0)
#define SPI_CLOCK_FREQ      AM_HAL_IOM_16MHZ  // 16 MHz SPI clock
#define SPI_MODE            AM_HAL_IOM_SPI_MODE_0  // SPI mode 0 (CPOL=0, CPHA=0)
#define NUM_SAMPLES         100000           // Total samples to capture 
#define BLOCK_SIZE          255               // CQ block size (max transactions per batch is 255)

// Buffer for ADC samples (2 bytes each). Placed in .shared for DMA, aligned to 4 bytes.
static volatile uint16_t g_ADCBuffer[NUM_SAMPLES] __attribute__((section(".shared")));
// static uint16_t g_ADCBuffer[NUM_SAMPLES] __attribute__((section(".shared"), aligned(4)));

// Global variables to track capture progress
static volatile uint32_t g_samples_to_capture = NUM_SAMPLES;
static volatile bool g_capture_done = false;
void *g_IOMHandle;

// Interrupt-driven callback after each block of transactions
void adc_block_callback(void *pCallbackCtxt, uint32_t status)
{
    // Check if last block completed successfully
    if (status != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("SPI CQ Error: 0x%X\n", status);
    }

    // If more samples remain, queue the next block of SPI transactions
    if (g_samples_to_capture > 0) 
    {
        uint32_t transactions = (g_samples_to_capture > BLOCK_SIZE) ? BLOCK_SIZE : g_samples_to_capture;
        // Enqueue the next batch of SPI reads
        for (uint32_t i = 0; i < transactions; i++) {
            // Calculate buffer offset for this sample (each sample is 2 bytes)
            uint32_t index = NUM_SAMPLES - g_samples_to_capture;  // index of next sample (0-based)
            am_hal_iom_transfer_t txn = {0};
            txn.uPeerInfo.ui32SpiChipSelect = ADC_SPI_CS;
            txn.ui32InstrLen = 0;                 // No command bytes (just direct read)
            txn.ui64Instr    = 0;                 // No address/offset needed
            txn.ui32NumBytes = 2;                 // 2 bytes per ADC sample (16-bit frame)
            txn.eDirection   = AM_HAL_IOM_RX;     // Read from SPI (ADC to MCU)
            txn.pui32RxBuffer = (uint32_t *)&g_ADCBuffer[index];  // Destination in buffer
            txn.bContinue   = false;              // De-assert CS after each transfer
            txn.ui8Priority = 1;                  // High-priority DMA for prompt service
            // For all but the last transaction in this block, no callback (NULL).
            // For the last transaction in the block, use this callback again.
            am_hal_iom_callback_t cb = (i == transactions - 1) ? adc_block_callback : NULL;
            uint32_t status = am_hal_iom_nonblocking_transfer(g_IOMHandle, &txn, cb, NULL);
            if (status != AM_HAL_STATUS_SUCCESS) {
                am_util_stdio_printf("Transfer queue error: 0x%X\n", status);
            }
            // Update index for next sample
            g_samples_to_capture--;  // mark one sample scheduled
        }
    } 
    else {
        // All samples have been captured. Signal completion.
        g_capture_done = true;
        // Invalidate DCache (if applicable) to ensure CPU sees fresh DMA data (optional).
        // am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);
        // Extract 12-bit ADC results from raw data and print first 10 samples.
        am_util_stdio_printf("Capture complete. First 2 samples:\n");
        for (int i = 0; i < 2; i++) {

            uint8_t *pRx = (uint8_t *)&g_ADCBuffer[i];
            uint16_t raw = ((uint16_t)pRx[0] << 8) | pRx[1];
            uint16_t adc_value = (raw >> 2) & 0x0FFF;
            am_util_stdio_printf("%d: 0x%03X (%d)\n", i, adc_value, adc_value);
        }
    }
}

void iom_set_up(uint32_t iomModule){
        // System initializations (clock, etc.) – assume already done in startup code.
    // Initialize IOM1 for SPI
    if (am_hal_iom_initialize(IOM_MODULE, &g_IOMHandle) != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error: IOM initialize failed\n");
        return;
    }
    // Power on the IOM
    if (am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false) != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error: IOM power-up failed\n");
        return;
    }
}

// IOM1 interrupt service routine -- ensures HAL gets called to handle CQ events
void am_iomaster1_isr(void){
    uint32_t status;
    // Read and clear interrupt status from IOM1
    am_hal_iom_interrupt_status_get(g_IOMHandle, true, &status);
    am_hal_iom_interrupt_clear(g_IOMHandle, status);
    // Service the interrupt (this will invoke callbacks for completed transactions)
    am_hal_iom_interrupt_service(g_IOMHandle, status);
}

int main(void)
{
    uart_init();  // Initialize UART for output (optional)
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
    
    // am_hal_pwrctrl_low_power_init();  // Optional: helps to reduce power consumption
    iom_set_up(IOM_MODULE);


    // Configure IOM1 for SPI mode0 @16MHz, and allocate a CQ buffer
    static uint32_t iom_cq_buffer[6152];  // Command queue buffer (size ~24.6KB for 256 transactions)
    am_hal_iom_config_t iomConfig = {
        .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq  = SPI_CLOCK_FREQ,
        .eSpiMode       = SPI_MODE,
        .pNBTxnBuf      = iom_cq_buffer,
        .ui32NBTxnBufLength = sizeof(iom_cq_buffer) / 4  // length in words
    };
    if (am_hal_iom_configure(g_IOMHandle, &iomConfig) != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error: IOM configure failed\n");
        return -1;
    }
    // Configure SPI pins for IOM1 (MISO, MOSI, SCK, CS). **Replace with actual pin numbers for your hardware.**
    // Example (using Ambiq board support macros if available):
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, g_AM_BSP_GPIO_IOM1_MISO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK,  g_AM_BSP_GPIO_IOM1_SCK);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS,  g_AM_BSP_GPIO_IOM1_CS);
    // If not using BSP, use actual pad numbers and GPIO functions to configure them for IOM1.
    
    // Enable the IOM to start operation
    am_hal_iom_enable(g_IOMHandle);
    // Enable relevant IOM interrupts (CQ mode updates and errors). Use CQ pause (CQCMP) interrupt.
    am_hal_iom_interrupt_enable(g_IOMHandle, AM_HAL_IOM_INT_CQERR | AM_HAL_IOM_INT_CQUPD);
    NVIC_EnableIRQ(IOMSTR1_IRQn);  // Enable IOM1 interrupt in NVIC (vector must call am_hal_iom_isr)
    
    // create a variable to store the current time
    // uint32_t start_time = am_hal_sysctrl_time_get();

    am_util_stdio_printf("Starting now\n");
    // *** Start ADC Sampling using CQ ***
    // Prime the first block of transactions
    g_samples_to_capture = NUM_SAMPLES;
    g_capture_done = false;
    uint32_t first_block = (g_samples_to_capture > BLOCK_SIZE) ? BLOCK_SIZE : g_samples_to_capture;
    g_samples_to_capture -= first_block;  // reserve these samples for the first block
    for (uint32_t i = 0; i < first_block; i++) {
        uint32_t index = i;  // starting at buffer index 0
        am_hal_iom_transfer_t txn = {0};
        txn.uPeerInfo.ui32SpiChipSelect = ADC_SPI_CS;
        txn.ui32InstrLen = 0;
        txn.ui64Instr    = 0;
        txn.ui32NumBytes = 2;
        txn.eDirection   = AM_HAL_IOM_RX;
        txn.pui32RxBuffer = (uint32_t *)&g_ADCBuffer[index];
        txn.bContinue   = false;
        txn.ui8Priority = 1;
        am_hal_iom_callback_t cb = (i == first_block - 1) ? adc_block_callback : NULL;
        uint32_t status = am_hal_iom_nonblocking_transfer(g_IOMHandle, &txn, cb, NULL);
        if (status != AM_HAL_STATUS_SUCCESS) {
            am_util_stdio_printf("Transfer queue error: 0x%X\n", status);
        }
    }
    // Wait for capture to complete (enter low-power sleep in the meantime)
    while (!g_capture_done) {
        // am_util_stdio_printf("%d",i++);
        // am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
    // Capture complete, print elapsed time
    // uint32_t end_time = am_hal_sysctrl_time_get();
    // am_util_stdio_printf("Capture complete. Elapsed time: %u ms\n", (end_time - start_time) / 1000);
    am_util_stdio_printf("Capture complete\n");

    // (Optional) Disable IOM or cleanup if needed
    am_hal_iom_disable(g_IOMHandle);
    am_hal_iom_uninitialize(g_IOMHandle);
    // while(1);  // End of program
    return 0;
}



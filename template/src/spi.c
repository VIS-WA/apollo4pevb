// Description: SPI code for ADS7042 ADC and Apollo4 Plus

#include "am_mcu_apollo.h" 
#include "am_util_stdio.h" 
#include "am_bsp.h" 
#include "uart.h" 
#include <string.h>

#define IOM_MODULE 1

// Global IOM handle. 
void *g_IOMHandle = NULL;

// Global SPI configuration for IOM1. 
static am_hal_iom_config_t g_sIOMSpiConfig = { 
    .eInterfaceMode = AM_HAL_IOM_SPI_MODE, 
    .ui32ClockFreq = AM_HAL_IOM_16MHZ, 
    .eSpiMode = AM_HAL_IOM_SPI_MODE_0, };

void iom_set_up(uint32_t iomModule) { 
    if (am_hal_iom_initialize(iomModule, &g_IOMHandle) != AM_HAL_STATUS_SUCCESS) { 
        am_util_stdio_printf("IOM%d initialization failed\n", iomModule); 
    } 
    
    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false); 
    if (am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig) != AM_HAL_STATUS_SUCCESS) { 
        am_util_stdio_printf("IOM%d configuration failed\n", iomModule); 
    } 
    // Enable the IOM pins (this configures the physical pins for SPI). 
    am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_SPI_MODE); 
    am_hal_iom_enable(g_IOMHandle); 
}

// Read the ADC conversion result from the sensor via SPI. 
// The sensor continuously outputs a 14-bit frame (2 zero bits followed by 12 data bits) 
// once CS is asserted low. We perform a 2-byte transfer and then discard the extra bits. 
uint16_t spi_read_sensor(void) { 
    uint32_t tx_data = 0; // Dummy transmit data. 
    uint32_t rx_data = 0; // Buffer for received data.

    am_hal_iom_transfer_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.ui32InstrLen = 0;       // No instruction bytes.
    transaction.ui32NumBytes = 2;         // 2 bytes (16 clock pulses).
    transaction.pui32TxBuffer = &tx_data;
    transaction.pui32RxBuffer = &rx_data;
    // Use the chip select defined by the BSP for IOM1.
    transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM1_CS_CHNL;
    // Use RX direction so that clocks are generated and data is read.
    transaction.eDirection = AM_HAL_IOM_RX;

    if (am_hal_iom_blocking_transfer(g_IOMHandle, &transaction) != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("SPI transfer failed\n");
    }

    // The received data is stored in rx_data. Cast it as a byte pointer to extract two bytes.
    uint8_t *pRx = (uint8_t *)&rx_data;
    uint16_t raw = ((uint16_t)pRx[0] << 8) | pRx[1];

    // Discard the 2 extra bits (the two most significant bits), leaving a 14-bit frame.
    uint16_t frame = raw >> 2;
    // Extract the 12-bit ADC conversion result.
    uint16_t adc_value = frame & 0x0FFF;
    return adc_value;
}

int main(){
    am_hal_pwrctrl_low_power_init(); 
    uart_init();

    // SPI Connections - default for IOM1
    // CS: 11
    // SCLK: 8
    // MISO: 10

    // Initialize IOM1 as SPI master.
    iom_set_up(IOM_MODULE);
    am_util_stdio_printf("%d",AM_BSP_IOM1_CS_CHNL);
    while (1)
    {
        uint16_t adc_val = spi_read_sensor();
        am_util_stdio_printf("ADC Conversion Result: %u (0x%03X)\n", adc_val, adc_val);
        am_util_delay_ms(4000);
    }
    return 0;

}

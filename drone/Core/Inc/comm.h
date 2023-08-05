/*
 * comm.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_COMM_H_
#define SRC_COMM_H_

#include <stm32f7xx_hal.h>

#include "bool.h"
#include "basic_io.h"

typedef enum {
    BUS_TYPE_NONE = 0,
    BUS_TYPE_I2C,
    BUS_TYPE_SPI,
    BUS_TYPE_MPU_SLAVE, // Slave I2C on SPI master
    BUS_TYPE_GYRO_AUTO,  // Only used by acc/gyro bus auto detection code
} busType_e;

struct spiDevice_s;

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;

typedef struct dmaResource_s dmaResource_t;

typedef enum {
    OWNER_FREE = 0,
    OWNER_PWMINPUT,
    OWNER_PPMINPUT,
    OWNER_MOTOR,
    OWNER_SERVO,
    OWNER_LED,
    OWNER_ADC,
    OWNER_ADC_BATT,
    OWNER_ADC_CURR,
    OWNER_ADC_EXT,
    OWNER_ADC_RSSI,
    OWNER_SERIAL_TX,
    OWNER_SERIAL_RX,
    OWNER_PINDEBUG,
    OWNER_TIMER,
    OWNER_SONAR_TRIGGER,
    OWNER_SONAR_ECHO,
    OWNER_SYSTEM,
    OWNER_SPI_SCK,
    OWNER_SPI_MISO,
    OWNER_SPI_MOSI,
    OWNER_I2C_SCL,
    OWNER_I2C_SDA,
    OWNER_SDCARD,
    OWNER_SDIO_CK,
    OWNER_SDIO_CMD,
    OWNER_SDIO_D0,
    OWNER_SDIO_D1,
    OWNER_SDIO_D2,
    OWNER_SDIO_D3,
    OWNER_SDCARD_CS,
    OWNER_SDCARD_DETECT,
    OWNER_FLASH_CS,
    OWNER_BARO_CS,
    OWNER_GYRO_CS,
    OWNER_OSD_CS,
    OWNER_RX_SPI_CS,
    OWNER_SPI_CS,
    OWNER_GYRO_EXTI,
    OWNER_BARO_EOC,
    OWNER_COMPASS_EXTI,
    OWNER_USB,
    OWNER_USB_DETECT,
    OWNER_BEEPER,
    OWNER_OSD,
    OWNER_RX_BIND,
    OWNER_INVERTER,
    OWNER_LED_STRIP,
    OWNER_TRANSPONDER,
    OWNER_VTX_POWER,
    OWNER_VTX_CS,
    OWNER_VTX_DATA,
    OWNER_VTX_CLK,
    OWNER_COMPASS_CS,
    OWNER_RX_BIND_PLUG,
    OWNER_ESCSERIAL,
    OWNER_CAMERA_CONTROL,
    OWNER_TIMUP,
    OWNER_RANGEFINDER,
    OWNER_RX_SPI,
    OWNER_PINIO,
    OWNER_USB_MSC_PIN,
    OWNER_MCO,
    OWNER_RX_SPI_BIND,
    OWNER_RX_SPI_LED,
    OWNER_PREINIT,
    OWNER_RX_SPI_EXTI,
    OWNER_RX_SPI_CC2500_TX_EN,
    OWNER_RX_SPI_CC2500_LNA_EN,
    OWNER_RX_SPI_CC2500_ANT_SEL,
    OWNER_QUADSPI_CLK,
    OWNER_QUADSPI_BK1IO0,
    OWNER_QUADSPI_BK1IO1,
    OWNER_QUADSPI_BK1IO2,
    OWNER_QUADSPI_BK1IO3,
    OWNER_QUADSPI_BK1CS,
    OWNER_QUADSPI_BK2IO0,
    OWNER_QUADSPI_BK2IO1,
    OWNER_QUADSPI_BK2IO2,
    OWNER_QUADSPI_BK2IO3,
    OWNER_QUADSPI_BK2CS,
    OWNER_BARO_XCLR,
    OWNER_PULLUP,
    OWNER_PULLDOWN,
    OWNER_DSHOT_BITBANG,
    OWNER_SWD,
    OWNER_RX_SPI_EXPRESSLRS_RESET,
    OWNER_RX_SPI_EXPRESSLRS_BUSY,
    OWNER_TOTAL_COUNT
} resourceOwner_e;

typedef struct resourceOwner_s {
    resourceOwner_e owner;
    uint8_t resourceIndex;
} resourceOwner_t;

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
    dmaResource_t               *ref;
    uint8_t                     stream;
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    resourceOwner_t             owner;
    uint8_t                     resourceIndex;
    uint32_t                    completeFlag;
} dmaChannelDescriptor_t;

typedef enum I2CDevice {
    I2CINVALID = -1,
    I2CDEV_1   = 0,
    I2CDEV_2,
    I2CDEV_3,
    I2CDEV_4,
} I2CDevice;

#define I2C_CFG_TO_DEV(x)   ((x) - 1)
#define I2C_DEV_TO_CFG(x)   ((x) + 1)

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4,
    SPIDEV_5,
    SPIDEV_6
} SPIDevice;

#define SPI_CFG_TO_DEV(x)   ((x) - 1)
#define SPI_DEV_TO_CFG(x)   ((x) + 1)

struct busSegment_s;

// Bus interface, independent of connected device
typedef struct busDevice_s {
    busType_e busType;
    union {
        struct busSpi_s {
            SPI_TypeDef *instance;
            uint16_t speed;
            bool leadingEdge;
        } spi;
        struct busI2C_s {
            I2CDevice device;
        } i2c;
        struct busMpuSlave_s {
            struct extDevice_s *master;
        } mpuSlave;
    } busType_u;
    bool useDMA;
    uint8_t deviceCount;
    dmaChannelDescriptor_t *dmaTx;
    dmaChannelDescriptor_t *dmaRx;
    // Use a reference here as this saves RAM for unused descriptors
    DMA_InitTypeDef             *initTx;
    DMA_InitTypeDef             *initRx;
    volatile struct busSegment_s* volatile curSegment;
    bool initSegment;
} busDevice_t;

// External device has an associated bus and bus dependent address
typedef struct extDevice_s {
    busDevice_t *bus;
    union {
        struct extSpi_s {
            uint16_t speed;
            IO_t csnPin;
            bool leadingEdge;
        } spi;
        struct extI2C_s {
            uint8_t address;
        } i2c;
        struct extMpuSlave_s {
            uint8_t address;
        } mpuSlave;
    } busType_u;
    // Cache the init structure for the next DMA transfer to reduce inter-segment delay
    DMA_InitTypeDef             initTx;
    DMA_InitTypeDef             initRx;
    // Support disabling DMA on a per device basis
    bool useDMA;
    // Per device buffer reference if needed
    uint8_t *txBuf, *rxBuf;
    // Connected devices on the same bus may support different speeds
    uint32_t callbackArg;
} extDevice_t;

typedef struct busSegment_s {
    union {
        struct {
            // Transmit buffer
            uint8_t *txData;
            // Receive buffer, or in the case of the final segment to
            uint8_t *rxData;
        } buffers;
        struct {
            // Link to the device associated with the next transfer
            const extDevice_t *dev;
            // Segments to process in the next transfer.
            volatile struct busSegment_s *segments;
        } link;
    } u;
    int len;
    bool negateCS; // Should CS be negated at the end of this segment
    busStatus_e (*callback)(uint32_t arg);
} busSegment_t;

bool i2cBusSetInstance(extDevice_t *dev, uint32_t device);

#endif /* SRC_COMM_H_ */

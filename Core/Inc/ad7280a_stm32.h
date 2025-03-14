#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usart.h"

#define AD7280A_SLAVE_ID            1

//define your CS port and pin name
#define AD7280_CS_Port	SPI2_CS_GPIO_Port
#define AD7280_CS_Pin	SPI2_CS_Pin

/* Acquisition time */
#define AD7280A_ACQ_TIME_400ns      0
#define AD7280A_ACQ_TIME_800ns      1
#define AD7280A_ACQ_TIME_1200ns     2
#define AD7280A_ACQ_TIME_1600ns     3

/* Conversion averaging */
#define AD7280A_CONV_AVG_DIS        0
#define AD7280A_CONV_AVG_2          1
#define AD7280A_CONV_AVG_4          2
#define AD7280A_CONV_AVG_8          3

/* Alert register bits */
#define AD7280A_ALERT_REMOVE_VIN5       (1 << 2)
#define AD7280A_ALERT_REMOVE_VIN4_VIN5  (2 << 2)
#define AD7280A_ALERT_REMOVE_AUX5       (1 << 0)
#define AD7280A_ALERT_REMOVE_AUX4_AUX5  (2 << 0)

/* Registers */
#define AD7280A_CELL_VOLTAGE_1          0x0  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_2          0x1  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_3          0x2  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_4          0x3  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_5          0x4  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_6          0x5  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_1               0x6  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_2               0x7  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_3               0x8  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_4               0x9  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_5               0xA  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_6               0xB  /* D11 to D0, Read only */
#define AD7280A_SELF_TEST               0xC  /* D11 to D0, Read only */
#define AD7280A_CONTROL_HB              0xD  /* D15 to D8, Read/write */
#define AD7280A_CONTROL_LB              0xE  /* D7 to D0, Read/write */
#define AD7280A_CELL_OVERVOLTAGE        0xF  /* D7 to D0, Read/write */
#define AD7280A_CELL_UNDERVOLTAGE       0x10 /* D7 to D0, Read/write */
#define AD7280A_AUX_ADC_OVERVOLTAGE     0x11 /* D7 to D0, Read/write */
#define AD7280A_AUX_ADC_UNDERVOLTAGE    0x12 /* D7 to D0, Read/write */
#define AD7280A_ALERT                   0x13 /* D7 to D0, Read/write */
#define AD7280A_CELL_BALANCE            0x14 /* D7 to D0, Read/write */
#define AD7280A_CB1_TIMER               0x15 /* D7 to D0, Read/write */
#define AD7280A_CB2_TIMER               0x16 /* D7 to D0, Read/write */
#define AD7280A_CB3_TIMER               0x17 /* D7 to D0, Read/write */
#define AD7280A_CB4_TIMER               0x18 /* D7 to D0, Read/write */
#define AD7280A_CB5_TIMER               0x19 /* D7 to D0, Read/write */
#define AD7280A_CB6_TIMER               0x1A /* D7 to D0, Read/write */
#define AD7280A_PD_TIMER                0x1B /* D7 to D0, Read/write */
#define AD7280A_READ                    0x1C /* D7 to D0, Read/write */
#define AD7280A_CNVST_N_CONTROL         0x1D /* D7 to D0, Read/write */

/* Bits and Masks */
#define AD7280A_CTRL_HB_CONV_INPUT_ALL                  (0 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_6CELL_AUX1_3_4       (1 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_6CELL                (2 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_SELF_TEST            (3 << 6)
#define AD7280A_CTRL_HB_CONV_RES_READ_ALL               (0 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_6CELL_AUX1_3_4    (1 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_6CELL             (2 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_NO                (3 << 4)
#define AD7280A_CTRL_HB_CONV_START_CNVST                (0 << 3)
#define AD7280A_CTRL_HB_CONV_START_CS                   (1 << 3)
#define AD7280A_CTRL_HB_CONV_AVG_DIS                    (0 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_2                      (1 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_4                      (2 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_8                      (3 << 1)
#define AD7280A_CTRL_HB_CONV_AVG(x)                     ((x) << 1)
#define AD7280A_CTRL_HB_PWRDN_SW                        (1 << 0)

#define AD7280A_CTRL_LB_SWRST                           (1 << 7)
#define AD7280A_CTRL_LB_ACQ_TIME_400ns                  (0 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_800ns                  (1 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_1200ns                 (2 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_1600ns                 (3 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME(x)                     ((x) << 5)
#define AD7280A_CTRL_LB_MUST_SET                        (1 << 4)
#define AD7280A_CTRL_LB_THERMISTOR_EN                   (1 << 3)
#define AD7280A_CTRL_LB_LOCK_DEV_ADDR                   (1 << 2)
#define AD7280A_CTRL_LB_INC_DEV_ADDR                    (1 << 1)
#define AD7280A_CTRL_LB_DAISY_CHAIN_RB_EN               (1 << 0)

#define AD7280A_ALERT_GEN_STATIC_HIGH                   (1 << 6)
#define AD7280A_ALERT_RELAY_SIG_CHAIN_DOWN              (3 << 6)

#define AD7280A_ALL_CELLS                               (0xAD << 16)

#define AD7280A_DEVADDR_MASTER                  0
#define AD7280A_DEVADDR_ALL                     0x1F

/* Value to be sent when readings are performed */
#define AD7280A_READ_TXVAL                      0xF800030A

#define NUMBITS_READ        22   // Number of bits for CRC when reading
#define NUMBITS_WRITE       21   // Number of bits for CRC when writing

#define BUFSIZE				1024
//PR
#define BATTERY_SERIES_CON		48


/*****************************************************************************/
/************************ Functions Declarations *****************************/
/*****************************************************************************/
/* Initializes the communication with the device. */
char AD7280A_Init(void);

/* Transmits 32 data bits from/to AD7280A. */
unsigned long AD7280A_Transmit_32bits(uint32_t data);

/* Transmits then receives 32 data bits to/from AD7280A. */
unsigned long AD7280A_TransmitReceive_32bits(uint32_t data,unsigned int datasize);

/* Computes the CRC value for a write transmission, and prepares the complete
 write codeword */
unsigned long AD7280A_CRC_Write(unsigned int data);

/* Checks the received message if the received CRC and computed CRC are
the same. */
long AD7280A_CRC_Read(uint32_t data);

/* Performs a read from all registers on 2 devices. */
char AD7280A_Convert_Read_All(unsigned int dataSize);

/* Converts acquired data to float values. */
char AD7280A_Convert_Data_All(void);

unsigned int AD7280A_CBxTim(unsigned int cellNum);

void AD7280A_CBOutAll(int totalCell);

void AD7280A_CBOffAll(void);

unsigned int AD7280A_addressConv(unsigned int addressIn);

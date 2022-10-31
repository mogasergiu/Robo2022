#include <i2c.h>
#include <position.h>
#include <gpio.h>
#include "hardware/i2c.h"

#ifndef LSM6DSOX_H

#define BITMASK(a, b) (((unsigned) -1 >> (31 - (a - b + 1))) << b)

#define LSM6DSOX_I2C_BUS_ADDR 0x6a

#define FUNC_CFG_ACCESS                            0x1
#define      FUNC_CFG_ACCESS_FUNC_CFG_ACCESS       0b10000000
#define      FUNC_CFG_ACCESS_SHUB_REG_ACCESS       0b01000000
#define      FUNC_CFG_ACCESS_OIS_CTRL_FROM_UI      0b00000001
#define PIN_CTRL                                   0x2
#define      PIN_CTRL_OIS_PU_DS                    0b10111111
#define      PIN_CTRL_SDO_PU_EN                    0b01111111
#define S4S_TPH_L                                  0x4
#define      S4S_TPH_L_TPH_H_SEL                   0b10000000
#define      S4S_TPH_L_TPH_L_MASK                  BITMASK(6, 0)
#define      S4S_TPH_L_TPH_L_OFFSET                0x0
#define S4S_TPH_H                                  0x5
#define S4S_RR                                     0x6
#define      S4S_RR_RR_MASK                        BITMASK(1, 0)
#define      S4S_RR_RR_OFFSET                      0x0
#define FIFO_CTRL1                                 0x7
#define FIFO_CTRL2                                 0x8
#define      FIFO_CTRL2_STOP_ON_WTM                0b10000000
#define      FIFO_CTRL2_FIFO_COMPR_RT_EN           0b01000000
#define      FIFO_CTRL2_ODRCHG_EN                  0b00010000
#define      FIFO_CTRL2_UNCOPTR_RATE_1             0b00000100
#define      FIFO_CTRL2_UNCOPTR_RATE_0             0b00000010
#define      FIFO_CTRL2_WTM8                       0b00000001
#define FIFO_CTRL_3                                0x9
#define      FIFO_CTRL_3_BDR_GY_MASK               BITMASK(7, 4)
#define      FIFO_CTRL_3_BDR_GY_OFFSET             0x4
#define      FIFO_CTRL_3_BDR_XL_MASK               BITMASK(3, 0)
#define      FIFO_CTRL_3_BDR_XL_OFFSET             0x0
#define FIFO_CTRL_4                                0xA
#define      FIFO_CTRL_4_DEC_TS_BATCH_MASK         BITMASK(7, 6)
#define      FIFO_CTRL_4_DEC_TS_BATCH_OFFSET       0x6
#define      FIFO_CTRL_4_ODR_T_BATCH_MASK          BITMASK(5, 4)
#define      FIFO_CTRL_4_ODR_T_BATCH_OFFSET        0x4
#define      FIFO_CTRL_4_FIFO_MODE_MASK            BITMASK(2, 0)
#define      FIFO_CTRL_4_FIFO_MODE_OFFSET          0x0
#define COUNTER_BDR_REG1                           0xB
#define      COUNTER_BDR_REG1_dataready_pulsed     0b10000000
#define      COUNTER_BDR_REG1_RST_COUNTER_BDR      0b01000000
#define      COUNTER_BDR_REG1_TRIG_COUNTER_BDR     0b00100000
#define      COUNTER_BDR_REG1_CNT_BDR_TH_MASK      BITMASK(2, 0)
#define      COUNTER_BDR_REG1_CNT_BDR_TH_OFFSET    0x0
#define COUNTER_BDR_REG2                           0xC
#define INT1_CTRL                                  0xD
#define      INT1_CTRL_DEN_DRDY_flag               0b10000000
#define      INT1_CTRL_INT1_CNT_BDR                0b01000000
#define      INT1_CTRL_INT1_FIFO_FULL              0b00100000
#define      INT1_CTRL_INT1_FIFO_OVR               0b00010000
#define      INT1_CTRL_INT1_FIFO_TH                0b00001000
#define      INT1_CTRL_INT1_BOOT                   0b00000100
#define      INT1_CTRL_INT1_DRDY_G                 0b00000010
#define      INT1_CTRL_INT1_DRDY_XL                0b00000001
#define INT2_CTRL                                  0xE
#define      INT2_CTRL_INT2_CNT_BDR                0b01000000
#define      INT2_CTRL_INT2_FIFO_FULL              0b00100000
#define      INT2_CTRL_INT2_FIFO_OVR               0b00010000
#define      INT2_CTRL_INT_FIFO_TH                 0b00001000
#define      INT2_CTRL_INT2_DRDY_TEMP              0b00000100
#define      INT2_CTRL_INT2_DRDY_G                 0b00000010
#define      INT2_CTRL_INT2_DRDY_XL                0b00000001
#define WHO_AM_I                                   0xF
#define CTRL1_XL                                   0x10
#define      CTRL1_XL_ODR_CL_MASK                  BITMASK(7, 4)
#define      CTRL1_XL_ODR_CL_OFFSET                0x4
#define      CTRL1_XL_FS_XL_MASK                   BITMASK(3, 2)
#define      CTRL1_XL_FS_XL_MASK_OFFSET            0x2
#define      CTRL1_XL_LPF2_XL_EN                   0b00000010
#define CTRL2_G                                    0x11
#define      CTRL2_G_ODR_MASK_G                    BITMASK(7, 4)
#define      CTRL2_G_ODR_G_OFFSET                  0x4
#define      CTRL2_G_FS_MASK_G                     BITMASK(3, 2)
#define      CTRL2_G_FS_G_OFFSET                   0x2
#define      CTRL2_G_FS_125                        0b00000010
#define CTRL3_C                                    0x12
#define      CTRL3_C_BOOT                          0b10000000
#define      CTRL3_C_BDU                           0b01000000
#define      CTRL3_C_H_LACTIVE                     0b00100000
#define      CTRL3_C_PP_OD                         0b00010000
#define      CTRL3_C_SIM                           0b00001000
#define      CTRL3_C_IF_INC                        0b00000100
#define      CTRL3_C_SW_RESET                      0b00000001
#define CTRL4_C                                    0x13
#define      CTRL4_C_SLEEP_G                       0b01000000
#define      CTRL4_C_INT2_on_INT1                  0b00100000
#define      CTRL4_C_DRDY_MASK                     0b00001000
#define      CTRL4_C_I2C_disable                   0b00000100
#define      CTRL4_C_LPF1_SEL_G                    0b00000010
#define CTRL5_C                                    0x14
#define     CTRL5_C_XL_ULP_EN                      0b10000000
#define     CTRL5_C_ROUNDING_MASK                  BITMASK(6, 5)
#define     CTRL5_C_ROUNDING_OFFSET                0x5
#define     CTRL5_C_ROUNDING_STATUS                0b00010000
#define     CTRL5_C_ST_MASK_G                      BITMASK(3, 2)
#define     CTRL5_C_ST_G_OFFSET                    0x2
#define     CTRL5_C_ST_MASK_XL                     BITMASK(1, 0)
#define     CTRL5_C_ST_MASK_XL_OFFSET              0x0
#define CTRL6_C                                    0x15
#define      CTRL6_C_TRIG_EN                       0b10000000
#define      CTRL6_C_LVL1_EN                       0b01000000
#define      CTRL6_C_LVL2_EN                       0b00100000
#define      CTRL6_C_XL_HM_MODE                    0b00010000
#define      CTRL6_C_USR_OFF_W                     0b00001000
#define      CTRL6_C_FTYPE_MASK                    BITMASK(2, 0)
#define      CTRL6_C_FTYPE_OFFSET                  0x0
#define CTRL7_C                                    0x16
#define      CTRL7_C_G_HM_MODE                     0b10000000
#define      CTRL7_C_HP_EN_G                       0b01000000
#define      CTRL7_C_HPM_G_MASK                    BITMASK(5, 4)
#define      CTRL7_C_HPM_G_OFFSET                  0x4
#define      CTRL7_C_OIS_ON_EN                     0b00000100
#define      CTRL7_C_USR_OFF_ON_OUT                0b00000010
#define      CTRL7_C_OIS_ON                        0b00000001
#define CTRL8_XL                                   0x17
#define      CTRL8_XL_HPCF_XL_MASK                 BITMASK(7, 5)
#define      CTRL8_XL_HPCF_XL_OFFSET               0x5
#define      CTRL8_XL_HP_REF_MODE_XL               0b00010000
#define      CTRL8_XL_FASTSETTL_MODE_XL            0b00001000
#define      CTRL8_XL_HP_SLOPE_XL_EN               0b00000100
#define      CTRL8_XL_XL_FS_MODE                   0b00000010
#define      CTRL8_XL_LOW_PASS_ON_6D               0b00000001
#define CTRL9_XL                                   0x18
#define      CTRL9_XL_DEN_X                        0b10000000
#define      CTRL9_XL_DEN_Y                        0b01000000
#define      CTRL9_XL_DEN_Z                        0b00100000
#define      CTRL9_XL_DEN_XL_G                     0b00010000
#define      CTRL9_XL_DEN_XL_EN                    0b00001000
#define      CTRL9_XL_DEN_LH                       0b00000100
#define      CTRL9_XL_I3C_disable                  0b00000010
#define CTRL19_C                                   0x19
#define      CTRL19_C_TIMESTAMP_EN                 0b00100000
#define ALL_INT_SRC                                0x1A
#define      ALL_INT_SRC_TIMESTAMP_ENDCOUNT        0b10000000
#define      ALL_INT_SRC_SLEEP_CHANGE_IA           0b00100000
#define      ALL_INT_SRC_D6D_IA                    0b00010000
#define      ALL_INT_SRC_DOUBLE_TAP                0b00001000
#define      ALL_INT_SRC_SINGLE_TAP                0b00000100
#define      ALL_INT_SRC_WU_IA                     0b00000010
#define      ALL_INT_SRC_FF_IA                     0b00000001
#define WAKE_UP_SRC                                0x1B
#define      WAKE_UP_SRC_SLEEP_CHANGE_IA           0b01000000
#define      WAKE_UP_SRC_FF_IA                     0b00100000
#define      WAKE_UP_SRC_SLEEP_STATE               0b00010000
#define      WAKE_UP_SRC_WU_IA                     0b00001000
#define      WAKE_UP_SRC_X_WU                      0b00000100
#define      WAKE_UP_SRC_Y_WU                      0b00000010
#define      WAKE_UP_SRC_Z_WU                      0b00000001
#define TAP_SRC                                    0x1C
#define      TAP_SRC_TAP_IA                        0b01000000
#define      TAP_SRC_SINGLE_TAP                    0b00100000
#define      TAP_SRC_DOUBLE_TAP                    0b00010000
#define      TAP_SRC_TAP_SIGN                      0b00001000
#define      TAP_SRC_X_TAP                         0b00000100
#define      TAP_SRC_Y_TAP                         0b00000010
#define      TAP_SRC_Z_TAP                         0b00000001
#define D6D_SRC                                    0x1D
#define      D6D_SRC_DEN_DRDY                      0b10000000
#define      D6D_SRC_D6D_IA                        0b01000000
#define      D6D_SRC_ZH                            0b00100000
#define      D6D_SRC_ZL                            0b00010000
#define      D6D_SRC_YH                            0b00001000
#define      D6D_SRC_YL                            0b00000100
#define      D6D_SRC_XH                            0b00000010
#define      D6D_SRC_XL                            0b00000001
#define STATUS_REG                                 0x1E
#define      STATUS_REG_TDA                        0b00000100
#define      STATUS_REG_GDA                        0b00000010
#define      STATUS_REG_XLDA                       0b00000001
#define OUT_TEMP_L                                 0x20
#define OUT_TEMP_H                                 0x21
#define OUTX_L_G                                   0x22
#define OUTX_H_G                                   0x23
#define OUTY_L_G                                   0x24
#define OUTY_H_G                                   0x25
#define OUTZ_L_G                                   0x26
#define OUTZ_H_G                                   0x27
#define OUTX_L_A                                   0x28
#define OUTX_H_A                                   0x29
#define OUTY_L_A                                   0x2A
#define OUTY_H_A                                   0x2B
#define OUTZ_L_A                                   0x2C
#define OUTZ_H_A                                   0x2D

typedef struct {
    const position_ops_t * const acc;
    position_t apos;
    const position_ops_t * const gyro;
    position_t gpos;
    const i2c_slave_ops_t * const ops;
    const gpio_t * const gpios;
} lsm6dsox_t;

extern lsm6dsox_t lsm6dsox;

#endif  /* LSM6DSOX_H */

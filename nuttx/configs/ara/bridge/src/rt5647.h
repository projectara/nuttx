#ifndef _RT5647_H_
#define _RT5647_H_

#include <nuttx/device.h>
#include <nuttx/device_codec.h>

#define RT5647_RESET                    0x00

#define RT5647_SPKOUT                   0x01
#define RT5647_HPOUT                    0x02
#define RT5647_LOUT_VOL                 0x03
#define RT5647_LOUT_CTL                 0x05
#define RT5647_MONO                     0x04
#define RT5647_IN1_CTL1                 0x0A
#define RT5647_IN1_CTL2                 0x0B
#define RT5647_IN1_CTL3                 0x0C
#define RT5647_IN2_CTL                  0x0D
#define RT5647_IN3_CTL                  0x0E
#define RT5647_IN_LR_VOL                0x0F

#define RT5647_SIDETONE                 0x18

#define RT5647_DAC1_DIGI_VOL            0x19
#define RT5647_DAC2_DIGI_VOL            0x1A
#define RT5647_DAC2_DIGI_MUTE           0x1B
#define RT5647_STO1_ADC_DIGI_VOL        0x1C
#define RT5647_MONO_ADC_DIGI_VOL        0x1D
#define RT5647_STO1_ADC_BST_GAIN        0x1E
#define RT5647_MONO_ADC_BST_GAIN        0x20

#define RT5647_STO1_ADC_DIGI_MIXER      0x27
#define RT5647_MONO_ADC_DIGI_MIXER      0x28
#define RT5647_ADC_DAC_DIGI_MIXER       0x29
#define RT5647_DAC_STO_DIGI_MIXER       0x2A
#define RT5647_DAC_MONO_DIGI_MIXER      0x2B
#define RT5647_DAC_DD_DIGI_MIXER        0x2C
#define RT5647_DATA_COPY_MODE           0x2F

#define RT5647_PWR_CTRL1                0x61
#define RT5647_PWR_CTRL2                0x62
#define RT5647_PWR_CTRL3                0x63
#define RT5647_PWR_CTRL4                0x64
#define RT5647_PWR_CTRL5                0x65
#define RT5647_PWR_CTRL6                0x66

#define EN_I2S1                         BIT(15)

#endif /* _RT5647_H_ */

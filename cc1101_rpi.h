/* rf1101-linux: Linux kernel driver for the CC1101 RF module
*  Copyright (C) 2013 George Wang
*
*  rf1101-linux is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*
*  rf1101-linux is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with rf1101-linux.  If not, see <http://www.gnu.org/licenses/>.
*/

#define     u8   unsigned char
#define     u16  unsigned int


// Configuration Registers
#define CC1101_IOCFG2           0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1           0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0           0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR          0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1            0x04        // Sync word, high byte
#define CC1101_SYNC0            0x05        // Sync word, low byte
#define CC1101_PKTLEN           0x06        // Packet length
#define CC1101_PKTCTRL1         0x07        // Packet automation control
#define CC1101_PKTCTRL0         0x08        // Packet automation control
#define CC1101_ADDR             0x09        // Device address
#define CC1101_CHANNR           0x0A        // Channel number
#define CC1101_FSCTRL1          0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0          0x0C        // Frequency synthesizer control
#define CC1101_FREQ2            0x0D        // Frequency control word, high byte
#define CC1101_FREQ1            0x0E        // Frequency control word, middle byte
#define CC1101_FREQ0            0x0F        // Frequency control word, low byte
#define CC1101_MDMCFG4          0x10        // Modem configuration
#define CC1101_MDMCFG3          0x11        // Modem configuration
#define CC1101_MDMCFG2          0x12        // Modem configuration
#define CC1101_MDMCFG1          0x13        // Modem configuration
#define CC1101_MDMCFG0          0x14        // Modem configuration
#define CC1101_DEVIATN          0x15        // Modem deviation setting
#define CC1101_MCSM2            0x16        // Main Radio Cntrl State Machine config
#define CC1101_MCSM1            0x17        // Main Radio Cntrl State Machine config
#define CC1101_MCSM0            0x18        // Main Radio Cntrl State Machine config
#define CC1101_FOCCFG           0x19        // Frequency Offset Compensation config
#define CC1101_BSCFG            0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2         0x1B        // AGC control
#define CC1101_AGCCTRL1         0x1C        // AGC control
#define CC1101_AGCCTRL0         0x1D        // AGC control
#define CC1101_WOREVT1          0x1E        // High byte Event 0 timeout
#define CC1101_WOREVT0          0x1F        // Low byte Event 0 timeout
#define CC1101_WORCTRL          0x20        // Wake On Radio control
#define CC1101_FREND1           0x21        // Front end RX configuration
#define CC1101_FREND0           0x22        // Front end TX configuration
#define CC1101_FSCAL3           0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2           0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1           0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0           0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1          0x27        // RC oscillator configuration
#define CC1101_RCCTRL0          0x28        // RC oscillator configuration
#define CC1101_FSTEST           0x29        // Frequency synthesizer cal control
#define CC1101_PTEST            0x2A        // Production test
#define CC1101_AGCTEST          0x2B        // AGC test
#define CC1101_TEST2            0x2C        // Various test settings
#define CC1101_TEST1            0x2D        // Various test settings
#define CC1101_TEST0            0x2E        // Various test settings


// 13 command strobe registers
#define CC1101_SRES             0x30        // Reset chip.
#define CC1101_SFSTXON          0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                            // If in RX/TX: Go to a wait state where only the synthesizer is
                                            // running (for quick RX / TX turnaround).
#define CC1101_SXOFF            0x32        // Turn off crystal oscillator.
#define CC1101_SCAL             0x33        // Calibrate frequency synthesizer and turn it off
                                            // (enables quick start).
#define CC1101_SRX              0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                            // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX              0x35        // In IDLE state: Enable TX. Perform calibration first if
                                            // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                            // Only go to TX if channel is clear.
#define CC1101_SIDLE            0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                            // Wake-On-Radio mode if applicable.
//#define CC1101_SAFC             0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR             0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD             0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX             0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX             0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST          0x3C        // Reset real time clock.
#define CC1101_SNOP             0x3D        // No operation. May be used to pad strobe commands to two
                                            // bytes for simpler software.

#define CC1101_PARTNUM      0x30
#define CC1101_VERSION      0x31
#define CC1101_FREQEST      0x32
#define CC1101_LQI          0x33
#define CC1101_RSSI         0x34
#define CC1101_MARCSTATE    0x35
#define CC1101_WORTIME1     0x36
#define CC1101_WORTIME0     0x37
#define CC1101_PKTSTATUS    0x38
#define CC1101_VCO_VC_DAC   0x39
#define CC1101_TXBYTES      0x3A
#define CC1101_RXBYTES      0x3B

#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F



//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

// Bit fields in the chip status byte
#define CC1101_STATUS_CHIP_RDYn_BM             0x80
#define CC1101_STATUS_STATE_BM                 0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

// Chip states
#define CC1101_STATE_IDLE                      0x00
#define CC1101_STATE_RX                        0x10
#define CC1101_STATE_TX                        0x20
#define CC1101_STATE_FSTXON                    0x30
#define CC1101_STATE_CALIBRATE                 0x40
#define CC1101_STATE_SETTLING                  0x50
#define CC1101_STATE_RX_OVERFLOW               0x60
#define CC1101_STATE_TX_UNDERFLOW              0x70


//----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
#define CC1101_LQI_CRC_OK_BM                   0x80
#define CC1101_LQI_EST_BM                      0x7F

// RF_SETTINGS is a data structure which contains all relevant CC1101 registers
typedef struct {
    u8 iocfg0;     // GDO0 Output Pin Configuration
    u8 fifothr;    // RX FIFO and TX FIFO Thresholds
    u8 pktctrl0;   // Packet Automation Control
    u8 fsctrl1;    // Frequency Synthesizer Control
    u8 freq2;      // Frequency Control Word, High Byte
    u8 freq1;      // Frequency Control Word, Middle Byte
    u8 freq0;      // Frequency Control Word, Low Byte
    u8 mdmcfg4;    // Modem Configuration
    u8 mdmcfg3;    // Modem Configuration
    u8 mdmcfg2;    // Modem Configuration
    u8 deviatn;    // Modem Deviation Setting
    u8 mcsm0;      // Main Radio Control State Machine Configuration
    u8 foccfg;     // Frequency Offset Compensation Configuration
    u8 worctrl;    // Wake On Radio Control
    u8 fscal3;     // Frequency Synthesizer Calibration
    u8 fscal2;     // Frequency Synthesizer Calibration
    u8 fscal1;     // Frequency Synthesizer Calibration
    u8 fscal0;     // Frequency Synthesizer Calibration
    u8 test2;      // Various Test Settings
    u8 test1;      // Various Test Settings
    u8 test0;      // Various Test Settings
} RF_SETTINGS;


RF_SETTINGS rfSettings = {
    0x06,  // IOCFG0        GDO0 Output Pin Configuration
    0x47,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
    0x05,  // PKTCTRL0      Packet Automation Control
    0x06,  // FSCTRL1       Frequency Synthesizer Control
    0x10,  // FREQ2         Frequency Control Word, High Byte
    0xB1,  // FREQ1         Frequency Control Word, Middle Byte
    0x3B,  // FREQ0         Frequency Control Word, Low Byte
    0xF6,  // MDMCFG4       Modem Configuration
    0x83,  // MDMCFG3       Modem Configuration
    0x13,  // MDMCFG2       Modem Configuration
    0x15,  // DEVIATN       Modem Deviation Setting
    0x18,  // MCSM0         Main Radio Control State Machine Configuration
    0x16,  // FOCCFG        Frequency Offset Compensation Configuration
    0xFB,  // WORCTRL       Wake On Radio Control
    0xE9,  // FSCAL3        Frequency Synthesizer Calibration
    0x2A,  // FSCAL2        Frequency Synthesizer Calibration
    0x00,  // FSCAL1        Frequency Synthesizer Calibration
    0x1F,  // FSCAL0        Frequency Synthesizer Calibration
    0x81,  // TEST2         Various Test Settings
    0x35,  // TEST1         Various Test Settings
    0x09,  // TEST0         Various Test Settings
};


/*
enum cc1101_registers{
    CC1101_IOCFG0,
    CC1101_FIFOTHR,
    CC1101_PKTCTRL0,
    CC1101_FSCTRL1,
    CC1101_FREQ2,
    CC1101_FREQ1,
    CC1101_FREQ0,
    CC1101_MDMCFG4,
    CC1101_MDMCFG3,
    CC1101_MDMCFG2,
    CC1101_DEVIATN,
    CC1101_MCSM0,
    CC1101_FOCCFG,
    CC1101_WORCTRL,
    CC1101_FSCAL3,
    CC1101_FSCAL2,
    CC1101_FSCAL1,
    CC1101_FSCAL0,
    CC1101_TEST2,
    CC1101_TEST1,
    CC1101_TEST0
}

enum cc1101_register_values{
    0x06,  // IOCFG0        GDO0 Output Pin Configuration
    0x47,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
    0x05,  // PKTCTRL0      Packet Automation Control
    0x06,  // FSCTRL1       Frequency Synthesizer Control
    0x10,  // FREQ2         Frequency Control Word, High Byte
    0xB1,  // FREQ1         Frequency Control Word, Middle Byte
    0x3B,  // FREQ0         Frequency Control Word, Low Byte
    0xF6,  // MDMCFG4       Modem Configuration
    0x83,  // MDMCFG3       Modem Configuration
    0x13,  // MDMCFG2       Modem Configuration
    0x15,  // DEVIATN       Modem Deviation Setting
    0x18,  // MCSM0         Main Radio Control State Machine Configuration
    0x16,  // FOCCFG        Frequency Offset Compensation Configuration
    0xFB,  // WORCTRL       Wake On Radio Control
    0xE9,  // FSCAL3        Frequency Synthesizer Calibration
    0x2A,  // FSCAL2        Frequency Synthesizer Calibration
    0x00,  // FSCAL1        Frequency Synthesizer Calibration
    0x1F,  // FSCAL0        Frequency Synthesizer Calibration
    0x81,  // TEST2         Various Test Settings
    0x35,  // TEST1         Various Test Settings
    0x09,  // TEST0         Various Test Settings

}

*/




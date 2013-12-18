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

#define     CC1101_IOCTL_RESETCHIP      _IOW(CC1101_DEV_MAJOR, 0, int)

/*
Reset chip
*/

#define     CC1101_IOCTL_SFSTXON        _IOW(CC1101_DEV_MAJOR, 1, int)

/*
Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
If in RX (with CCA):Go to a wait state where only the synthesizer
is running (for quick RX / TX turnaround).
*/

#define     CC1101_IOCTL_SXOFF          _IOW(CC1101_DEV_MAJOR, 2, int)
/*
Turn off crystal oscillator.
*/

#define     CC1101_IOCTL_SCAL           _IOW(CC1101_DEV_MAJOR, 3, int)

/*
Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
*/

#define     CC1101_IOCTL_SRX            _IOW(CC1101_DEV_MAJOR, 4, int)

/*
Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
*/

#define     CC1101_IOCTL_STX            _IOW(CC1101_DEV_MAJOR, 5, int)

/*
In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
If in RX state and CCA is enabled: Only go to TX if channel is clear.
*/

#define     CC1101_IOCTL_SIDLE          _IOW(CC1101_DEV_MAJOR, 6, int)

/*
Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
*/

#define     CC1101_IOCTL_SWOR           _IOW(CC1101_DEV_MAJOR, 7, int)

/*
Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
WORCTRL.RC_PD=0.
*/

#define     CC1101_IOCTL_SPWD           _IOW(CC1101_DEV_MAJOR, 8, int)

/*
Enter power down mode when CSn goes high.
*/

#define     CC1101_IOCTL_SFRX           _IOW(CC1101_DEV_MAJOR, 9, int)

/*
Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
*/


#define     CC1101_IOCTL_SFTX           _IOW(CC1101_DEV_MAJOR, 10, int)

/*
Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
*/


#define     CC1101_IOCTL_SWORRST        _IOW(CC1101_DEV_MAJOR, 11, int)

/*
Reset real time clock to Event1 value.
*/


#define     CC1101_IOCTL_SNOP           _IOW(CC1101_DEV_MAJOR, 12, int)

/*
No operation. May be used to get access to the chip status byte.
*/

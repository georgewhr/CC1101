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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <asm/uaccess.h>

#include "cc1101_rpi.h"

#define CC1101_DEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			0	/* ... up to 256 */
#define SPI_BUFF_SIZE	        8
#define SEMAPHORE_NUM           13

const char this_driver_name[] = "cc1101";
static long open_times = 0;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static u8 sync_busy = 0;
static long callbacks_ctr = 0;




struct cc1101_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;//is used to interchange data between an SPI slave and CPU memory
	struct list_head	device_entry;
    struct semaphore fop_sem;
	struct cdev cdev;
	//struct class *class;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
};


struct cc1101_control{
    struct spi_message msg;
	struct spi_transfer transfer;
	u32 spi_callbacks;
	u8 *tx_buff;
}

//static struct cc1101_control cc1101_ctl;
//static struct cc1101_data cc1101_dev;
static struct class *cc1101_class;


static const struct file_operations cc1101_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	cc1101_write,
	.read =		cc1101_read,
    #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = cc1101_ioctl
    #else
    .unlocked_ioctl = cc1101_ioctl
    #endif
	.open =		cc1101_open,
	.release =	cc1101_release,
	//.llseek =	no_llseek,
};


static int __init cc1101_init(void)
{
    int status;

    /*
    memset(&cc1101_dev, 0, sizeof(cc1101_dev));
	memset(&cc1101_ctl, 0, sizeof(cc1101_ctl));
	*/

    status = register_chrdev(cc1101_dev_MAJOR, "spi", &cc1101_fops);
    if (status < 0)
        return status;

	if (cc1101_init_class() < 0)
		goto fail_0;

	if (cc1101_init_spi() < 0)
		goto fail_0;
/*
fail_0:
	device_destroy(cc1101_class, cc1101_dev.devt);
	class_destroy(cc1101_class);
*/

}





static int __init cc1101_init_class()
{

    cc1101_class = class_create(THIS_MODULE, this_driver_name);

	if (!cc1101_class) {
		printk(KERN_ALERT "class_create() failed in cc1101_init_class()\n");
		return -1;
	}

	return 0;

}


static int __init c1101_init_spi()
{
    int status;


    status = spi_register_driver(&cc1101_driver);
	if (error < 0) {
		printk(KERN_ALERT "cc1101_register_driver() failed %d\n", status);
	}


	error = add_cc1101_device_to_bus();//if you have implemented in board level, no need to call this function
	if (error < 0) {
		printk(KERN_ALERT "add_cc1101_to_bus() failed\n");
		spi_unregister_driver(&cc1101_driver);
	}

	return error;

}


static struct spi_driver cc1101_driver = {
	.driver = {
		.name =	this_driver_name,
		.owner = THIS_MODULE,
	},
	.probe = cc1101_probe,
	.remove = __devexit_p(cc1101_remove),
};


/*The driver's init function calls spi_register_driver() in cc1101_init() which gives the kernel
a list of devices it is able to service, along with a pointer to the probe() function.
The kernel then calls the driver's probe() function once for each device.
will be checked in spi ccore module spi.c
*/

static int __devinit cc1101_probe(struct spi_device *spi)
{
    struct cc1101_data *cc1101_dev;

    cc1101_dev = kzalloc(sizeof(*cc1101_dev), GFP_KERNEL);

	if (!cc1101)
		return -ENOMEM;

    cc1101_dev -> spi = spi;

    pin_lock_init(cc1101_dev -> spi_lock);
    mutex_init(cc1101_dev -> buf_lock);
	sema_init(cc1101_dev -> fop_sem, SEMAPHORE_NUM);

	INIT_LIST_HEAD(&cc1101_dev->device_entry)

    //device_create() will generate /dev/this_driver_name
    if (!device_create(cc1101_class, NULL, cc1101_dev -> devt, NULL,
			this_driver_name)) {
		printk(KERN_ALERT "device_create(..., %s) failed\n",
			this_driver_name);
		class_destroy(cc1101_class);
		return -1;
	}

	return 0;

}

static int __devexit cc1101_remove(struct spi_device *spi)
{
	struct cc1101_data	*cc1101_dev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&cc1101_dev->spi_lock);
	cc1101_dev->spi = NULL  ;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&cc1101_dev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&cc1101_dev->device_entry);
	device_destroy(cc1101_class, cc1101_dev->devt);
	//clear_bit(MINOR(cc1101_dev->devt), minors);
	if (cc1101_dev->users == 0)
		kfree(cc1101_dev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static int cc1101_setup(struct cc1101_data *cc1101_dev)
{
    struct spi_message msg;
    struct spi_transfer *tr = kmalloc(sizeof(struct spi_transfer) * 20, GFP_KERNEL);
    u8 buffer[40];
    memset(tr, NULL, 20);
    spi_message_init(&msg);

    tr[0] = cc1101_make_transfer(CC1101_IOCFG0, rfSettings.iocfg0, buffer, NULL);
    tr[0].cs_change = 1;   //cs_change when set to 1 causes the CS line to go high between transfers in a spi_message series
    spi_message_add_tail(tr, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[1] = cc1101_make_transfer(CC1101_FIFOTHR,   rfSettings.fifothr, buffer + 2, NULL);
    tr[1].cs_change = 1;
    spi_message_add_tail(tr + 1, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[2] = cc1101_make_transfer(CC1101_PKTCTRL0,   rfSettings.pktctrl0, buffer + 4, NULL);
    tr[2].cs_change = 1;
    spi_message_add_tail(tr + 2, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[3] = cc1101_make_transfer(CC1101_FSCTRL1,   rfSettings.fsctrl1, buffer + 6, NULL);
    tr[3].cs_change = 1;
    spi_message_add_tail(tr + 3, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[4] = cc1101_make_transfer(CC1101_FREQ2,   rfSettings.freq2, buffer + 8, NULL);
    tr[4].cs_change = 1;
    spi_message_add_tail(tr + 4, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[5] = cc1101_make_transfer(CC1101_FREQ1,   rfSettings.freq1, buffer + 10, NULL);
    tr[5].cs_change = 1;
    spi_message_add_tail(tr + 5, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[6] = cc1101_make_transfer(CC1101_FREQ0,   rfSettings.freq0, buffer + 12, NULL);
    tr[6].cs_change = 1;
    spi_message_add_tail(tr + 6, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[7] = cc1101_make_transfer(CC1101_MDMCFG4,   rfSettings.mdmcfg4, buffer + 14, NULL);
    tr[7].cs_change = 1;
    spi_message_add_tail(tr + 7, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[8] = cc1101_make_transfer(CC1101_MDMCFG3,   rfSettings.mdmcfg3, buffer + 16, NULL);
    tr[8].cs_change = 1;
    spi_message_add_tail(tr + 8, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[9] = cc1101_make_transfer(CC1101_MDMCFG2,   rfSettings.mdmcfg2, buffer + 18, NULL);
    tr[9].cs_change = 1;
    spi_message_add_tail(tr + 9, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[10] = cc1101_make_transfer(CC1101_DEVIATN,   rfSettings.deviatn ,buffer + 20, NULL);
    tr[10].cs_change = 1;
    spi_message_add_tail(tr + 10, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[11] = cc1101_make_transfer(CC1101_MCSM0,   rfSettings.mcsm0 ,buffer + 22, NULL);
    tr[11].cs_change = 1;
    spi_message_add_tail(tr + 11, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[12] = cc1101_make_transfer(CC1101_FOCCFG,   rfSettings.foccfg ,buffer + 24, NULL);
    tr[12].cs_change = 1;
    spi_message_add_tail(tr + 12, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[13] = cc1101_make_transfer(CC1101_WORCTRL,   rfSettings.worctrl ,buffer + 26, NULL);
    tr[13].cs_change = 1;
    spi_message_add_tail(tr + 13, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[14] = cc1101_make_transfer(CC1101_FSCAL3,   rfSettings.fscal3 ,buffer + 28, NULL);
    tr[14].cs_change = 1;
    spi_message_add_tail(tr + 14, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[15] = cc1101_make_transfer(CC1101_FSCAL2,   rfSettings.fscal2 ,buffer + 30, NULL);
    tr[15].cs_change = 1;
    spi_message_add_tail(tr + 15, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[16] = cc1101_make_transfer(CC1101_FSCAL1,   rfSettings.fscal1 ,buffer + 32, NULL);
    tr[16].cs_change = 1;
    spi_message_add_tail(tr + 16, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[17] = cc1101_make_transfer(CC1101_FSCAL0,   rfSettings.fscal0 ,buffer + 34, NULL);
    tr[17].cs_change = 1;
    spi_message_add_tail(tr + 17, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[18] = cc1101_make_transfer(CC1101_TEST2,   rfSettings.test2 ,buffer + 36, NULL);
    tr[18].cs_change = 1;
    spi_message_add_tail(tr + 18, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[19] = cc1101_make_transfer(CC1101_TEST1,   rfSettings.test1 ,buffer + 38, NULL);
    tr[19].cs_change = 1;
    spi_message_add_tail(tr + 19, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);

    tr[20] = cc1101_make_transfer(CC1101_TEST0,   rfSettings.test0 ,buffer + 40, NULL);
    tr[20].cs_change = 1;
    spi_message_add_tail(tr + 20, &msg);
    spi_sync(cc1101_dev -> spi, &msg)
    spi_message_init(&msg);
}

struct spi_transfer cc1101_make_transfer(u8 rgstr, u8 cmd, u8 *buf, u8 *rx )
{

    struct spi_transfer tr{
     .tx_buf           = buf,
     .rx_buf           = rx_buf,
     .len              = 2,
     .cs_change        = 0,
     .bits_per_word    = 0,
     .delay_usecs      = 0,
     .speed_hz         = 0
    };

    *buf = rgstr;
    *(buf + 1) = cmd;

    return tr;

}


struct spi_transfer cc1101_strobe_transfer(u8 *buf)
{

    struct spi_transfer tr{
     .tx_buf           = buf,
     .rx_buf           = NULL,
     .len              = 2,
     .cs_change        = 0,
     .bits_per_word    = 0,
     .delay_usecs      = 0,
     .speed_hz         = 0
    };

    return tr;

}


static int cc1101_open(struct inode *inode, struct file *filp)
{
    struct cc1101_data *cc1101_dev;
    int			status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(cc1101_dev, &device_list, device_entry)
    {
        if (cc1101_dev -> devt == inode -> i_rdev)
        {
            status = 0;
			break;
		}
	}

    if (status == 0)
    {
		if (!cc1101_dev -> buffer)
		{
			cc1101_dev->buffer = kmalloc(4012, GFP_KERNEL);
			if (!cc1101_dev -> buffer)
			{
				printk(KERN_ALERT "CC1101: Can't allocate memory\n");
				status = -ENOMEM;
			}
		}

		if (status == 0)
		{
		    printk(KERN_ALERT "CC1101 device driver open successfully %d times\n", open_times);
			open_times ++;
			filp->private_data = cc1101_dev;
			nonseekable_open(inode, filp);
		}
	}


	else
		printk(KERN_ALERT "CC1101 device driver can't open\n");


	mutex_unlock(&device_list_lock);
	return status;


}

static long cc1101_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

    struct cc1101_data *cc1101_dev = (struct cc1101_data*)filp->private_data;
    struct spi_message *message;

    struct spi_transfer tr;

    spi_message_init(&msg);

    cc1101_setup(cc1101_dev);

     switch (cmd)
     {
         case   CC1101_IOCTL_RESETCHIP:{
             *(cc1101_dev -> buffer) = CC1101_SRES;
             tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
             break;
         }

         case   CC1101_IOCTL_SFSTXON:{
            *(cc1101_dev -> buffer) = CC1101_SFSTXON;
             tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
             break;
         }

         case   CC1101_IOCTL_SXOFF:{
            *(cc1101_dev -> buffer) = CC1101_SXOFF;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SCAL:{
            *(cc1101_dev -> buffer) = CC1101_SCAL;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SRX:{
            *(cc1101_dev -> buffer) = CC1101_SRX;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_STX:{
            *(cc1101_dev -> buffer) = CC1101_STX;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SIDLE :{
            *(cc1101_dev -> buffer) = CC1101_SIDLE;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SWOR  :{
            *(cc1101_dev -> buffer) = CC1101_SWOR;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SPWD  :{
            *(cc1101_dev -> buffer) = CC1101_SPWD;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SFRX  :{
            *(cc1101_dev -> buffer) = CC1101_SFRX;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SFTX  :{
            *(cc1101_dev -> buffer) = CC1101_SFTX;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SWORRST  :{
            *(cc1101_dev -> buffer) = CC1101_SWORRST;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SNOP  :{
            *(cc1101_dev -> buffer) = CC1101_SNOP;
            tr = cc1101_strobe_transfer(cc1101_dev -> buffer);
            break;
         }

       /*  default:
         {

         }*/

     }

     spi_message_add_tail(tr, &msg);
     spi_sync(cc1101_dev -> spi, &msg)
     spi_message_init(&msg);

}





//This is called somewhere in kernel space
static void cc1101_complete(void *arg)
{
    sync_busy = 0;
    callbacks_ctr ++;
    //complete(arg);
}

static ssize_t cc1101_sync(struct cc1101_data *cc1101_dev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message -> complete = cc1101_complete;
	message -> context = &done;

	spin_lock_irq(&cc1101_dev -> spi_lock);
	if (cc1101_dev -> spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(cc1101_dev -> spi, message);

	spin_unlock_irq(&cc1101_dev -> spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
		{
			status = message->actual_length;
			sync_busy = 1;
		}
	}
	return status;
}

static inline ssize_t cc1101_sync_write(struct cc1101_data *cc1101_dev, size_t len)
{
    u8 send_cmd0 = CC1101_TXFIFO;
    u8 send_cmd1 = CC1101_STX;
    u8 send_cmd2 = CC1101_SFTX;

	struct spi_transfer	t0 = {
			.tx_buf		= &send_cmd0,
			.len		= 1,
		};

    struct spi_transfer	t1 = {
			.tx_buf		= cc1101_dev->buffer,
			.len		= len,
		};


    struct spi_transfer	t2 = {
			.tx_buf		= &send_cmd1,
			.len		= 1,
		};

    struct spi_transfer	t3 = {
			.tx_buf		= &send_cmd12,
			.len		= 1,
		};


	struct spi_message	m;

	spi_message_init(&m);

	spi_message_add_tail(&t0, &m);

	spi_message_add_tail(&t1, &m);

	spi_message_add_tail(&t2, &m); //this is linked list, FIFO.

	spi_message_add_tail(&t3, &m);

	return cc1101_sync(cc1101_dev, &m);	// The core method for submitting any message to the SPI system is spi_async()
}



static ssize_t cc1101_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    struct cc1101_data *cc1101_dev = (struct cc1101_data*) filp -> private_data;
    ssize_t			status = 0;
	unsigned long		missing;

	mutex_lock(&cc1101_dev -> buf_lock);

	missing = copy_from_user(cc1101_dev -> buffer, buf, count);

	if (missing == 0)
	    status = cc1101_sync_write(cc1101_dev, count);

    else
        status = -EFAULT;

    mutex_unlock(&cc1101_dev -> buf_lock);

	return status;

}

static inline ssize_t cc1101_sync_read(struct cc1101_data *cc1101_dev, size_t len)
{
    u8 *read_cmd = kmalloc(sizeof(u8) * 2, GFP_KERNEL);
    *read_cmd = CC1101_SRX;
    *(read_cmd + 1) = CC1101_RXFIFO;

	struct spi_transfer	t = {
			.tx_buf		= read_cmd,
			.rx_buf     = &cc1101_dev -> buffer,
			.len		= len,
		};

	struct spi_message	m;

	spi_message_init(&m);

	spi_message_add_tail(&t, &m); //this is linked list, FIFO.

	return cc1101_sync(cc1101_dev, &m);	// The core method for submitting any message to the SPI system is spi_async()
}

static ssize_t cc1101_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct cc1101_data* cc1101_dev = (struct cc1101_data*)filp->private_data;
    short status = 0;

    mutex_lock(&cc1101_dev->buf_lock);

    status = cc1101_sync_read(cc1101_dev, count);

    if (copy_to_user(buf, cc1101_dev -> buffer, count))  {
		printk(KERN_ALERT "cc1101_read(): copy_to_user() failed\n");
		status = -EFAULT;
	}

	mutex_unlock(&cc1101_dev -> buf_lock);

	return status;
}

module_init(cc1101_init);



MODULE_AUTHOR("George Wang, <georgewhr@gmail.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cc1101");


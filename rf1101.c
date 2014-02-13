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
*  Youf should have received a copy of the GNU General Public License
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
#include "rf1101_ioctl.h"

#define CC1101_DEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32
#define SPI_BUFF_SIZE	        8
#define SEMAPHORE_NUM           13
static DECLARE_BITMAP(minors, N_SPI_MINORS);

//Define device hot plugable
#ifdef CONFIG_HOTPLUG
#  define __devinit
#else
#  define __devinit __init
#endif

// 3.8 kernels and later make these obsolete due to changes in HOTPLUG
// we keep them for compatibility with earlier kernel versions...
#ifndef __devexit
#define __devexit
#endif
#ifndef __devexit_p
#define __devexit_p(x)   x
#endif


const char this_driver_name[] = "spidev";
static long open_times = 0;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static u8 sync_busy = 0;
static long callbacks_ctr = 0;

static ssize_t cc1101_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static int __devinit cc1101_probe(struct spi_device *spi);
static long cc1101_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static ssize_t cc1101_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static int cc1101_release(struct inode *inode, struct file *filp);
static int __devexit cc1101_remove(struct spi_device *spi);
static int cc1101_open(struct inode *inode, struct file *filp);
static int __init cc1101_init_class();
static int __init cc1101_init_spi();
//static ssize_t cc1101_setup(struct cc1101_data *cc1101_dev);



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
};


static struct spi_driver cc1101_driver = {
	.driver = {
		.name =	this_driver_name,
		.owner = THIS_MODULE,
	},
	.probe = cc1101_probe,
	.remove = __devexit_p(cc1101_remove),
};


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
   // .unlocked_ioctl = cc1101_ioctl,
	.open =		cc1101_open,
	.release =	cc1101_release,
	//.llseek =	no_llseek,
};

struct spi_transfer cc1101_strobe_transfer_make(u8 *rxbuffer, u8 *buf, int length)
{

    struct spi_transfer tr = {
     .tx_buf           = buf,
     .rx_buf           = rxbuffer,
     .len              = length,
     .cs_change        = 0,
     .bits_per_word    = 0,
     .delay_usecs      = 0,
     .speed_hz         = 0,
    };

    return tr;

}

static void cc1101_complete(void *arg)
{
    sync_busy = 0;
    callbacks_ctr ++;
    //complete(arg);
}


static int cc1101_send_patable(struct cc1101_data *cc1101_dev, u8 *buf)
{
    u8 tablergstr = CC1101_PATABLE | WRITE_BURST;
    struct spi_transfer tr_table = cc1101_strobe_transfer_make(cc1101_dev->buffer, &tablergstr, 1);
    struct spi_transfer tr = cc1101_strobe_transfer_make(cc1101_dev->buffer, buf, 8);

    struct spi_message msg;

    spi_message_init(&msg);

  //  tr_table.cs_change = 1;
    spi_message_add_tail(&tr_table, &msg);

   //tr.cs_change = 1;
    spi_message_add_tail(&tr, &msg);

    return spi_sync(cc1101_dev -> spi, &msg);
}

static int cc1101_send_strobe(struct cc1101_data *cc1101_dev, u8 *buf, int length)
{
    struct spi_transfer tr = cc1101_strobe_transfer_make(cc1101_dev->buffer, buf, length);

    struct spi_message msg;

    spi_message_init(&msg);

    spi_message_add_tail(&tr, &msg);

    return spi_sync(cc1101_dev -> spi, &msg);
}


static int __init cc1101_init(void)
{
    int status;

    /*
    memset(&cc1101_dev, 0, sizeof(cc1101_dev));
	memset(&cc1101_ctl, 0, sizeof(cc1101_ctl));
	*/
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(CC1101_DEV_MAJOR, "spi", &cc1101_fops);
    if (status < 0)
        return status;

    cc1101_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(cc1101_class)) {
		unregister_chrdev(CC1101_DEV_MAJOR, cc1101_driver.driver.name);
		return PTR_ERR(cc1101_class);
	}


	status = spi_register_driver(&cc1101_driver);
	if (status < 0) {
		class_destroy(cc1101_class);
		unregister_chrdev(CC1101_DEV_MAJOR, cc1101_driver.driver.name);
		printk(KERN_ALERT "register driver failed\n");
	}

    else
       printk(KERN_ALERT "INIT SUCCESFULLY\n");

	return status;



}





static int __init cc1101_init_class()
{

    cc1101_class = class_create(THIS_MODULE, this_driver_name);

	if (!cc1101_class) {
		printk(KERN_ALERT "class_create() failed in cc1101_init_class()\n");
		return -1;
	}

	else
	   printk(KERN_ALERT "class_create() ok\n");

	return 0;

}


static int __init cc1101_init_spi()
{
    int error;

    error = spi_register_driver(&cc1101_driver);
	if (error < 0) {
		printk(KERN_ALERT "cc1101_register_driver() failed %d\n", error);
	}




    else
	   printk(KERN_ALERT "cc1101_init_spi ok\n");
    cc1101_init_class();

	/*error = add_cc1101_device_to_bus();//if you have implemented in board level, no need to call this function
	if (error < 0) {
		printk(KERN_ALERT "add_cc1101_to_bus() failed\n");
		spi_unregister_driver(&cc1101_driver);
	}*/

	return error;

}

struct spi_transfer cc1101_make_transfer(u8 rgstr, u8 cmd, char *buf, u8 *rx )
{

    //u8 tx[] = {rgstr, cmd,};


/*
    struct spi_transfer *tr =  kmalloc(sizeof(struct spi_transfer),GFP_KERNEL);
    tr -> tx_buf = (char) tx;
    tr -> rx_buf = rx;
    tr -> len = 2;
    tr -> cs_change = 0;
    tr -> bits_per_word  = 0;
    tr -> delay_usecs = 0;
    tr -> speed_hz = 0;
*/
   // printk(KERN_ALERT "rgstr : %.2x\n",tx[0]);
   // printk(KERN_ALERT "cmd : %.2x\n",tx[1]);
   // printk(KERN_ALERT "whole : %.4x\n",tx);

    struct spi_transfer tr = {
     .tx_buf           =  buf,
     .rx_buf           = rx,
     .len              = 2,
     .cs_change        = 0,
     .bits_per_word    = 0,
     .delay_usecs      = 0,
     .speed_hz         = 0,
    };

  //  printk(KERN_ALERT "cc1101_setup, tr.tx_buf : %.2x\n",(u8 *)(tr.tx_buf));

    buf[0] = rgstr;
    buf[1] = cmd;
    printk(KERN_ALERT "tx_buf 1 is %x \n", buf[0]);
    printk(KERN_ALERT "tx_buf 2 is %x \n", buf[1]);

    return tr;

}


static int cc1101_clear_buffer(struct cc1101_data *cc1101_dev)
{
    u8 cmd = CC1101_SFRX;
    struct spi_transfer tr = cc1101_strobe_transfer_make(cc1101_dev->buffer, &cmd,1);
    struct spi_message msg;
    spi_message_init(&msg);
    tr.cs_change = 1;
    spi_message_add_tail(&tr, &msg);
    return spi_sync(cc1101_dev -> spi, &msg);
}

static int cc1101_read_status(struct cc1101_data *cc1101_dev, char* buffer, int size)
{
    struct spi_transfer tr = {
       .rx_buf      = buffer,
       .len         = size,
    };

    struct spi_message msg;
    spi_message_init(&msg);
    spi_message_add_tail(&tr, &msg);
    return spi_sync(cc1101_dev -> spi, &msg);

}


static ssize_t cc1101_setup(struct cc1101_data *cc1101_dev)
{
    int err, i;
    printk(KERN_ALERT "cc1101_setup, cc1101_dev -> spi : %x\n",(int)(cc1101_dev -> spi));
    struct spi_message msg;


    //struct spi_transfer *tr = kmalloc(sizeof(struct spi_transfer) * 20, GFP_KERNEL);
    struct spi_transfer tr0, tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9, tr10, tr11, tr12, tr13, tr14, tr15, tr16, tr17, tr18, tr19, tr20, tr21;
    //struct spi_transfer tr;

    char buffer[46];

    u8 *ptr = kmalloc(16, GFP_KERNEL);
    memset(ptr,16,0);
    char *ptr2;
    memset(buffer,0,46);
   // memset(tr, NULL, 20);

    spi_message_init(&msg);
    //msg.complete = cc1101_complete;
   // msg.context = NULL;
    u8 resetcmd = CC1101_SRES;


    tr0 = cc1101_strobe_transfer_make(cc1101_dev -> buffer, &resetcmd,1);

    //tr0.cs_change = 1;
    spi_message_add_tail(&tr0, &msg);

    tr1 = cc1101_make_transfer(CC1101_IOCFG0, rfSettings.iocfg0, buffer + 0, ptr);


    //printk(KERN_ALERT "tr1.tx_buf[0] is %x \n", *buff);
    //printk(KERN_ALERT "tr1.tx_buf[1] is %x \n", *(buff+1));
   // printk(KERN_ALERT "in cc1101_setup, tr[0].tr_buf : %d\n",(int )tr1.tx_buf);
    //printk(KERN_ALERT "4 \n");
    //tr1.cs_change = 1;   //cs_change when set to 1 causes the CS line to go high between transfers in a spi_message series
    spi_message_add_tail(&tr1, &msg);
    //spi_sync(cc1101_dev -> spi, &msg);
    // spi_message_init(&msg);


    tr2 = cc1101_make_transfer(CC1101_FIFOTHR,   rfSettings.fifothr, buffer + 2, ptr+2);



    //printk(KERN_ALERT "tr2.tx_buf[0] is %x \n", *buff);
    //printk(KERN_ALERT "tr2.tx_buf[1] is %x \n", *(buff+1));


   // printk(KERN_ALERT "in cc1101_setup, tr[1].tr_buf : %d\n",(int)tr2.tx_buf);
    //tr2.cs_change = 1;
    spi_message_add_tail(&tr2, &msg);
    //spi_sync(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);


    tr3 = cc1101_make_transfer(CC1101_PKTCTRL0,   rfSettings.pktctrl0, buffer + 4, ptr+4);
    //  printk(KERN_ALERT "in cc1101_setup, tr[2].tr_buf : %d\n",(intr[2].tx_buf);
    //tr3.cs_change = 1;


    spi_message_add_tail(&tr3, &msg);
    // spi_sync(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr4 = cc1101_make_transfer(CC1101_FSCTRL1,   rfSettings.fsctrl1, buffer + 6, ptr+6);


   // printk(KERN_ALERT "in cc1101_setup, tr[3].tr_buf : %d\n",(int)tr[3].tx_buf);
    //tr4.cs_change = 1;
    spi_message_add_tail(&tr4, &msg);
   // spi_sync(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr5= cc1101_make_transfer(CC1101_FREQ2,   rfSettings.freq2, buffer + 8, ptr+8);

   // buff +=2;
    //tr5.cs_change = 1;
    spi_message_add_tail(&tr5, &msg);
    err = spi_sync(cc1101_dev -> spi, &msg);
    if(err == 0)
    {
      // printk(KERN_ALERT "in cc1101_setup, 1st spi_sync sucessfully\n");

       for(i = 0; i < 16; i ++)
          printk(KERN_ALERT "rx_buf is %x \n", *(ptr+i));

    }

    else
       printk(KERN_ALERT "in cc1101_setup, 1st spi_sync fail\n");

    spi_message_init(&msg);

    tr6 = cc1101_make_transfer(CC1101_FREQ1,   rfSettings.freq1, buffer + 10, NULL);
   // tr6.cs_change = 1;
    spi_message_add_tail(&tr6, &msg);
    //  spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr7 = cc1101_make_transfer(CC1101_FREQ0,   rfSettings.freq0, buffer + 12, NULL);
   // tr7.cs_change = 1;
    spi_message_add_tail(&tr7, &msg);
   // spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr8 = cc1101_make_transfer(CC1101_MDMCFG4,   rfSettings.mdmcfg4, buffer + 14, NULL);
    //tr8.cs_change = 1;
    spi_message_add_tail(&tr8, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr9 = cc1101_make_transfer(CC1101_MDMCFG3,   rfSettings.mdmcfg3, buffer + 16, NULL);
    //tr9.cs_change = 1;
    spi_message_add_tail(&tr9, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr10 = cc1101_make_transfer(CC1101_MDMCFG2,   rfSettings.mdmcfg2, buffer + 18, NULL);
    //tr10.cs_change = 1;
    spi_message_add_tail(&tr10, &msg);
    err = spi_sync(cc1101_dev -> spi, &msg);
    if(err == 0)
       printk(KERN_ALERT "in cc1101_setup, 2nd spi_sync sucessfully\n");

    else
       printk(KERN_ALERT "in cc1101_setup, 2nd spi_sync fail\n");
    spi_message_init(&msg);

    tr11 = cc1101_make_transfer(CC1101_DEVIATN,   rfSettings.deviatn ,buffer + 20, NULL);
    //tr11.cs_change = 1;
    spi_message_add_tail(&tr11, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr12 = cc1101_make_transfer(CC1101_MCSM0,   rfSettings.mcsm0 ,buffer + 22, NULL);
    //tr12.cs_change = 1;
    spi_message_add_tail(&tr12, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr13 = cc1101_make_transfer(CC1101_FOCCFG,   rfSettings.foccfg ,buffer + 24, NULL);
    //tr13.cs_change = 1;
    spi_message_add_tail(&tr13, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr14 = cc1101_make_transfer(CC1101_WORCTRL,   rfSettings.worctrl ,buffer + 26, NULL);
    //tr14.cs_change = 1;
    spi_message_add_tail(&tr14, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr15 = cc1101_make_transfer(CC1101_FSCAL3,   rfSettings.fscal3 ,buffer + 28, NULL);
    //tr15.cs_change = 1;
    spi_message_add_tail(&tr15, &msg);
    err = spi_sync(cc1101_dev -> spi, &msg);
    if(err == 0)
       printk(KERN_ALERT "in cc1101_setup, 3rd spi_sync sucessfully\n");

    else
       printk(KERN_ALERT "in cc1101_setup, 3rd spi_sync fail\n");
    spi_message_init(&msg);

    tr16 = cc1101_make_transfer(CC1101_FSCAL2,   rfSettings.fscal2 ,buffer + 30, NULL);
    //tr16.cs_change = 1;
    spi_message_add_tail(&tr16, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr17 = cc1101_make_transfer(CC1101_FSCAL1,   rfSettings.fscal1 ,buffer + 32, NULL);
    //tr17.cs_change = 1;
    spi_message_add_tail(&tr17, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr18 = cc1101_make_transfer(CC1101_FSCAL0,   rfSettings.fscal0 ,buffer + 34, NULL);
    //tr18.cs_change = 1;
    spi_message_add_tail(&tr18, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr19 = cc1101_make_transfer(CC1101_TEST2,   rfSettings.test2 ,buffer + 36, NULL);
    //tr19.cs_change = 1;
    spi_message_add_tail(&tr19, &msg);
    // spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr20 = cc1101_make_transfer(CC1101_TEST1,   rfSettings.test1 ,buffer + 38, NULL);
    //tr20.cs_change = 1;
    spi_message_add_tail(&tr20, &msg);
    //spi_async(cc1101_dev -> spi, &msg);
    //spi_message_init(&msg);

    tr21 = cc1101_make_transfer(CC1101_TEST0,   rfSettings.test0 ,buffer + 40, NULL);

    tr21.rx_buf = cc1101_dev -> buffer;
    //tr21.cs_change = 1;
    spi_message_add_tail(&tr21, &msg);
    err = spi_sync(cc1101_dev -> spi, &msg);
    if(err == 0)
    {
       printk(KERN_ALERT "in cc1101_setup, 4th spi_sync sucessfully\n");
       printk(KERN_ALERT "in cc1101_setup, buffer message is %x\n",*(cc1101_dev -> buffer));
    }

    else
       printk(KERN_ALERT "in cc1101_setup, 4th spi_sync fail\n");
    spi_message_init(&msg);

    printk(KERN_ALERT "all transfer done, setup done \n");

}



/*The driver's init function calls spi_register_driver() in cc1101_init() which gives the kernel
a list of devices it is able to service, along with a pointer to the probe() function.
The kernel then calls the driver's probe() function once for each device.
will be checked in spi ccore module spi.c
*/

static int __devinit cc1101_probe(struct spi_device *spi)
{
    struct cc1101_data *cc1101_dev;
    	int			status;
    unsigned long		minor;

    cc1101_dev = kzalloc(sizeof(*cc1101_dev), GFP_KERNEL);

	if (!cc1101_dev)
		return -ENOMEM;

    cc1101_dev -> spi = spi;
    printk(KERN_ALERT "in probe, cc1101_dev -> spi : %x\n",(int)(cc1101_dev -> spi));

  /*  spin_lock_init(&cc1101_dev -> spi_lock);
    mutex_init(&cc1101_dev -> buf_lock);
	sema_init(& cc1101_dev -> fop_sem, SEMAPHORE_NUM);*/

	INIT_LIST_HEAD(&cc1101_dev->device_entry);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	//cc1101_dev -> devt = MKDEV(CC1101_DEV_MAJOR, minor);
/*
    //device_create() will generate /dev/this_driver_name
    if (!device_create(cc1101_class, NULL, cc1101_dev -> devt, NULL,
			this_driver_name)) {
		printk(KERN_ALERT "device_create(..., %s) failed\n",
			this_driver_name);
		class_destroy(cc1101_class);
		return -1;
	}


    else
        printk(KERN_ALERT "device_create ok, in probe,next step is cc1101_setup\n");

	cc1101_setup(cc1101_dev);

	printk(KERN_ALERT "after cc1101_setup done \n");

	return 0;*/


		if (minor < N_SPI_MINORS) {
		struct device *dev;

		cc1101_dev->devt = MKDEV(CC1101_DEV_MAJOR, minor);
		dev = device_create(cc1101_class, &spi->dev, cc1101_dev->devt,
				    cc1101_dev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&cc1101_dev->device_entry, &device_list);
	}
//	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, cc1101_dev);

	else
		kfree(cc1101_dev);

	return status;

}

static int __devexit cc1101_remove(struct spi_device *spi)
{
    printk(KERN_ALERT "at cc1101_remove method \n");
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





static int cc1101_open(struct inode *inode, struct file *filp)
{
    struct cc1101_data *cc1101_dev;
    int			status = -ENXIO;
    printk(KERN_ALERT "In Open\n");

    mutex_lock(&device_list_lock);

    list_for_each_entry(cc1101_dev, &device_list, device_entry)
    {
        printk(KERN_ALERT "status is %d, %d\n", status, inode->i_rdev);
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
			memset(cc1101_dev->buffer,0,4012);
			filp->private_data = cc1101_dev;
			nonseekable_open(inode, filp);
			printk("<1>open filp->private_data : %x\n",(int)(&filp->private_data));
		}
	}


	else
		printk(KERN_ALERT "CC1101 device driver can't open\n");

    if (status == 0)
        cc1101_setup(cc1101_dev);
    printk(KERN_ALERT "in open, spidev -> spi : %x\n",(int)(cc1101_dev -> spi));


	mutex_unlock(&device_list_lock);
	return status;


}
/*
static long cc1101_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

    struct cc1101_data *cc1101_dev = (struct cc1101_data*)filp->private_data;
    struct spi_message *msg;

    struct spi_transfer tr;

    spi_message_init(msg);

    //cc1101_setup(cc1101_dev);

     switch (cmd)
     {
         case   CC1101_IOCTL_RESETCHIP:{
             *(cc1101_dev -> buffer) = CC1101_SRES;
             tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
             break;
         }

         case   CC1101_IOCTL_SFSTXON:{
            *(cc1101_dev -> buffer) = CC1101_SFSTXON;
             tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
             break;
         }

         case   CC1101_IOCTL_SXOFF:{
            *(cc1101_dev -> buffer) = CC1101_SXOFF;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SCAL:{
            *(cc1101_dev -> buffer) = CC1101_SCAL;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SRX:{
            *(cc1101_dev -> buffer) = CC1101_SRX;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_STX:{
            *(cc1101_dev -> buffer) = CC1101_STX;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SIDLE :{
            *(cc1101_dev -> buffer) = CC1101_SIDLE;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SWOR  :{
            *(cc1101_dev -> buffer) = CC1101_SWOR;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SPWD  :{
            *(cc1101_dev -> buffer) = CC1101_SPWD;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SFRX  :{
            *(cc1101_dev -> buffer) = CC1101_SFRX;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SFTX  :{
            *(cc1101_dev -> buffer) = CC1101_SFTX;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SWORRST  :{
            *(cc1101_dev -> buffer) = CC1101_SWORRST;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

         case   CC1101_IOCTL_SNOP  :{
            *(cc1101_dev -> buffer) = CC1101_SNOP;
            tr = cc1101_strobe_transfer_make(cc1101_dev -> buffer);
            break;
         }

        default:
         {

         }

     }

     spi_message_add_tail(&tr, msg);
     spi_async(cc1101_dev -> spi, msg);
     spi_message_init(msg);

}

*/



//This is called somewhere in kernel space


static ssize_t cc1101_sync(struct cc1101_data *cc1101_dev, struct spi_message *message)

{
    printk(KERN_ALERT "in cc1101_sync \n");
	DECLARE_COMPLETION_ONSTACK(done);
	int status = -1;
    int i;
	u8 tx_mode[2] =  {0xff, 0xfa};
    char buffer[32] = {0};

    struct spi_transfer t = {
        .tx_buf = tx_mode,
        .rx_buf = buffer,
        .len            = 2,
    };

	//struct spi_message	m;

	spi_message_add_tail(&t, message);



	//message -> complete = cc1101_complete;
	//message -> context = &done;

	//spin_lock_irq(&cc1101_dev -> spi_lock);
	if (cc1101_dev -> spi == NULL)
		status = -ESHUTDOWN;
	else
	{
	    printk(KERN_ALERT "about to spi_async\n");
		status = spi_sync(cc1101_dev -> spi, message);//this will wait until the message sucesffuly sent out
		printk(KERN_ALERT "finish spi_async, status is %d\n",status);
	}
	//spin_unlock_irq(&cc1101_dev -> spi_lock);
/*
	if (status == 0) {
	    printk(KERN_ALERT "status is 0\n");

		//wait_for_completion(&done);
		//status = message->status;
		if (status == 0)
		{
			status = message->actual_length;
			sync_busy = 1;
		}
	}*/
   /* struct spi_transfer t = {
        .tx_buf         = &tx_mode,
        .rx_buf = buffer,
        .len            = 8,
    };


    spi_message_init(&m);

	spi_message_add_tail(&t, &m);

	spi_sync(cc1101_dev -> spi, &m);*/

	for(i = 0; i<32;i++)
     printk(KERN_ALERT "in last spi_sync %x\n",buffer[i]);



	return status;
}

static inline ssize_t cc1101_sync_write(struct cc1101_data *cc1101_dev, size_t len)
{
    u8 send_cmd0 = CC1101_TXFIFO;
   // u8 send_cmd1 = CC1101_STX;
    u8 send_cmd1 = CC1101_TXFIFO | WRITE_BURST;
    u8 send_cmd2 = CC1101_SIDLE;
    u8 send_cmd3 = CC1101_STX;

    struct spi_message	m;

	spi_message_init(&m);
    //u8 send_cmd2 = CC1101_SFTX;

	struct spi_transfer	t0 = {
			.tx_buf		= &send_cmd0,
			.len		= 1,
		};




    struct spi_transfer	t1 = {
			.tx_buf		= &send_cmd1,
			.len		= 1,
    };



  //  spi_message_add_tail(&t0, &m);
    //spi_message_add_tail(&t1, &m);

    //cc1101_sync(cc1101_dev, &m);

    //spi_message_init(&m);

    struct spi_transfer	t2 = {
			.tx_buf		= cc1101_dev->buffer,
			.len		= 8,
    };

  /*  spi_message_add_tail(&t2, &m);
    cc1101_sync(cc1101_dev, &m);

    spi_message_init(&m);*/



    struct spi_transfer t3 = {
        .tx_buf         = &send_cmd2,
        .len            = 1,
    };

    struct spi_transfer t4 = {
        .tx_buf         = &send_cmd3,
        .len            = 1,
    };

    spi_message_add_tail(&t0, &m);
    spi_message_add_tail(&t1, &m);
    spi_message_add_tail(&t2, &m);
    spi_message_add_tail(&t3, &m);
	spi_message_add_tail(&t4, &m);



  /*  struct spi_transfer	t3 = {
			.tx_buf		= &send_cmd2,
			.len		= 1,
		};*/




/*

	spi_message_add_tail(&t1, &m);

	spi_message_add_tail(&t2, &m); //this is linked list, FIFO.
	spi_message_add_tail(&t3, &m);
	spi_message_add_tail(&t4, &m);*/

	//spi_message_add_tail(&t3, &m);

	return cc1101_sync(cc1101_dev, &m);	// The core method for submitting any message to the SPI system is spi_async()
}



static ssize_t cc1101_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    struct cc1101_data *cc1101_dev = (struct cc1101_data*) filp -> private_data;
    ssize_t			status = 0;
	unsigned long		missing;
	char buf_status[8] = {0};
	int i;
	u8 PaTabel[8] = {0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60};

	u8 reset = CC1101_SRES;

	cc1101_send_strobe(cc1101_dev, &reset, 1);

    cc1101_send_patable(cc1101_dev, PaTabel);


	//mutex_lock(&cc1101_dev -> buf_lock);
	printk(KERN_ALERT "in write,before copy_from_user\n");

    cc1101_dev->buffer = kmalloc(8, GFP_KERNEL);
    memset(cc1101_dev->buffer,0,8);

	missing = copy_from_user(cc1101_dev -> buffer, buf, count);

	for(i = 0; i < count;i++)
	 printk(KERN_ALERT "in write,cc1101_dev -> buffer is %d\n",*(cc1101_dev->buffer + i));

	printk(KERN_ALERT "in write,after copy_from_user\n");

	if (missing == 0)
	{
	    	printk(KERN_ALERT "in write,before cc1101_sync_write\n");


	    status = cc1101_sync_write(cc1101_dev, count);



	/*cc1101_read_status(cc1101_dev, buf_status, 64);

    for(i = 0; i < 8;i++)
	 printk(KERN_ALERT "buf_status is %x\n",*(buf_status + i));*/
	}

    else
        status = -EFAULT;

   // mutex_unlock(&cc1101_dev -> buf_lock);
   kfree(cc1101_dev->buffer);

	return status;

}

static inline ssize_t cc1101_sync_read(struct cc1101_data *cc1101_dev, size_t len)
{
    /*u8 *read_cmd = kmalloc(sizeof(u8) * 2, GFP_KERNEL);
    *read_cmd = CC1101_SRX;
    *(read_cmd + 1) = CC1101_RXFIFO;
*/
    int status;
    printk(KERN_ALERT "in sync_read \n");
    u8 read_cmd[2];
    read_cmd[0] = CC1101_SRX;
    read_cmd[1] = CC1101_RXFIFO;

    //status = cc1101_send_strobe(cc1101_dev, read_cmd);
    if(status == 0)
      printk(KERN_ALERT "in sync_read, cc1101_send_strobe successfully \n");

    else
      printk(KERN_ALERT "in sync_read, cc1101_send_strobe fail \n");

	struct spi_transfer	t = {
			.tx_buf		= read_cmd,
			.rx_buf     = cc1101_dev -> buffer,
			.len		= 2,
		};

	struct spi_message	m;

	spi_message_init(&m);

	spi_message_add_tail(&t, &m); //this is linked list, FIFO.
	//return 0;

	return cc1101_sync(cc1101_dev, &m);	// The core method for submitting any message to the SPI system is spi_async()
}

static ssize_t cc1101_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    printk(KERN_ALERT "in read,<1>open filp->private_data : %x\n",(int)(&filp->private_data));
    //struct cc1101_data* cc1101_dev = (struct cc1101_data*)filp->private_data;
    struct cc1101_data* cc1101_dev;
    short status = 0;
    u8 cmd;
    int i;

    cc1101_dev = filp->private_data;

    cc1101_dev->buffer = kmalloc(64, GFP_KERNEL);
    memset(cc1101_dev->buffer,0,64);
        for(i = 0; i<count;i++)
        printk(KERN_ALERT "just before cc1101_sync_read,cc1101_dev -> buffer is %x\n", *(cc1101_dev -> buffer+i));;

    //mutex_lock(&cc1101_dev->buf_lock);



    printk(KERN_ALERT "in read,before sync\n");

    status = cc1101_sync_read(cc1101_dev, count);


    for(i = 0; i<count;i++)
        printk(KERN_ALERT "after cc1101_sync_read,cc1101_dev -> buffer is %x\n", *(cc1101_dev -> buffer+i));;


    if (copy_to_user(buf, cc1101_dev -> buffer, count))  {
		printk(KERN_ALERT "cc1101_read(): copy_to_user() failed\n");
		status = -EFAULT;
	}

//    status = cc1101_clear_buffer(cc1101_dev);
  //  status = cc1101_clear_buffer(cc1101_dev);

  //  for(i = 0; i<count;i++)
    //    printk(KERN_ALERT "clear buffer,cc1101_dev -> buffer is %x\n", *(cc1101_dev -> buffer+i));;

    kfree(cc1101_dev -> buffer);

	//mutex_unlock(&cc1101_dev -> buf_lock);

	return status;
}


static int cc1101_release(struct inode *inode, struct file *filp)
{
	struct cc1101_data	*cc1101;
	int			status = 0;

	mutex_lock(&device_list_lock);
	cc1101 = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	cc1101->users--;
	if (!cc1101->users) {
		int		dofree;

		kfree(cc1101->buffer);
		cc1101->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&cc1101->spi_lock);
		dofree = (cc1101->spi == NULL);
		spin_unlock_irq(&cc1101->spi_lock);

		if (dofree)
			kfree(cc1101);
	}
	mutex_unlock(&device_list_lock);

	return status;
}
MODULE_LICENSE("GPL");


module_init(cc1101_init);

static void __exit cc1101_exit(void)
{
	spi_unregister_driver(&cc1101_driver);
	class_destroy(cc1101_class);
	unregister_chrdev(CC1101_DEV_MAJOR, cc1101_driver.driver.name);
}
module_exit(cc1101_exit);



MODULE_AUTHOR("George Wang, <georgewhr@gmail.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cc1101");


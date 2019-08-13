/*
 * Driver for the Epson RTC module RX-8731 SJ
 *
 * Copyright(C) Timesys Corporation 2015
 * Copyright(C) General Electric Company 2015
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bcd.h>
#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rtc.h>

#define RX8731_SEC     0x00
#define RX8731_MIN     0x01
#define RX8731_HOUR    0x02
#define RX8731_WDAY    0x03
#define RX8731_MDAY    0x04
#define RX8731_MONTH   0x05
#define RX8731_YEAR    0x06
#define RX8731_YEAR    0x06
#define RX8731_IO_CTRL 0x07
#define RX8731_ALMIN   0x08
#define RX8731_ALHOUR  0x09
#define RX8731_ALWDAY  0x0A
#define RX8731_TCOUNT0 0x0B
#define RX8731_TCOUNT1 0x0C
#define RX8731_EXT     0x0D
#define RX8731_FLAG    0x0E
#define RX8731_CTRL    0x0F

#define RX8731_EXT_TSEL0  	 BIT(0)
#define RX8731_EXT_TSEL1  	 BIT(1)
#define RX8731_EXT_TSEL 	 (RX8731_EXT_TSEL0|RX8731_EXT_TSEL1)
#define RX8731_EXT_TE  	 BIT(4)
#define RX8731_EXT_WADA  BIT(6)

#define RX8731_FLAG_VLF  BIT(1)
#define RX8731_FLAG_AF   BIT(3)
#define RX8731_FLAG_TF   BIT(4)
#define RX8731_FLAG_UF   BIT(5)
#define RX8731_FLAG_TEST BIT(7)

#define RX8731_CTRL_STOP BIT(1)
#define RX8731_CTRL_AIE  BIT(3)
#define RX8731_CTRL_TIE  BIT(4)
#define RX8731_CTRL_UIE  BIT(5)
#define RX8731_CTRL_TWP	 BIT(6)
#define RX8731_CTRL_TEST BIT(7)

#define RX8731_ALARM_AE  BIT(7)

static const struct i2c_device_id rx8731_id[] = {
	{ "rx8731", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rx8731_id);

static const struct of_device_id rx8731_of_match[] = {
	{ .compatible = "epson,rx8731" },
	{ }
};
MODULE_DEVICE_TABLE(of, rx8731_of_match);

struct rx8731_data {
	struct i2c_client *client;
	struct rtc_device *rtc;
	u8 ctrlreg;
};

/*
*P03 	PO2	PO1	POO
*0	1	1	0	normal rs-232
*0	1	0	1	normal rs-422
*1	0	1	0	bypass rs-232
*1	0	0	1	bypass rs-422
*/
#define RX8731_NORMAL_RS232	0XF6	/*normal RS-232*/
#define RX8731_NORMAL_RS422	0XF4	/*normal RS-422*/
#define RX8731_BYPASS_RS232	0XFA	/*bypass RS-232*/
#define RX8731_BYPASS_RS422	0XF9	/*bypass RS-422*/
#define UART_MODE0	0	/*normal RS-232*/
#define UART_MODE1	1	/*normal RS-422*/
#define UART_MODE2	2	/*bypass RS-232*/
#define UART_MODE3	3	/*bypass RS-422*/


/*
*TSEL1,0 table
*TSEL1		TSEL0
*0		0		4096HZ
*0		1		64Hz
*1		0		1HZ
*1		1		1/60HZ	
*/

struct timer_counter_setting
{
	unsigned int 	ms_per_tick;	
	unsigned int  	range;
	unsigned char 	config; /*TSEL1,0*/
};

#define RX8731_MAX_TCOUNT	4095
#define RX8731_MIN_TCOUNT	1


static struct timer_counter_setting  timer_setting[4]={
	{
		.ms_per_tick = 244,
		.range = 999,
		.config = 0,
	},
	{
		.ms_per_tick = 15625,
		.range = 63984,
		.config = 1,
	},
	{
		.ms_per_tick = 1000,
		.range = 4095000,
		.config = 2,
	},
	{
		.ms_per_tick = 60000,
		.range = 245700000,
		.config = 3,
	},
};
static unsigned int timeout = 0;

static ssize_t show_uart_mode(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	int  mode = 0;
	int  value = 0;
	unsigned char io_reg = 0;

	value = i2c_smbus_read_byte_data(rx8731->client, RX8731_IO_CTRL);
	if(value < 0)
	{
		dev_warn(dev, "read error.\n");	
		return value;
	}
	io_reg = value;
	switch(io_reg)
	{
		case RX8731_NORMAL_RS232: 
			mode = UART_MODE0;
			break;
		case RX8731_NORMAL_RS422:
			mode = UART_MODE1;
			break;
		case RX8731_BYPASS_RS232:
			mode = UART_MODE2;
			break;
		case RX8731_BYPASS_RS422:
			mode = UART_MODE3;
			break;
		default:
			mode = -1;
			break;
	}

	return sprintf(buf, "%d\n", mode);	
}
static ssize_t store_uart_mode(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int mode = 0;
	unsigned char io_reg = 0;

	ret = kstrtouint(buf, 10, &mode);
	if(ret < 0)
	{
		dev_warn(dev, "switch string error!.\n");
		return ret;
	}
	switch(mode)
	{
		case UART_MODE0: 
			io_reg = RX8731_NORMAL_RS232;
			break;
		case UART_MODE1:
			io_reg = RX8731_NORMAL_RS422;
			break;
		case UART_MODE2:
			io_reg = RX8731_BYPASS_RS232;
			break;
		case UART_MODE3:
			io_reg = RX8731_BYPASS_RS422;
			break;
		default:
			return -1;
	}

	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_IO_CTRL, io_reg);
	if(ret < 0)
	{
		dev_warn(dev, "write byte data failed!\n\n");	
		return ret;
	}

	return count;
}

static ssize_t show_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", timeout);	
}
static ssize_t store_timeout(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int 	value = 0;
	int 		cur_value = 0;
	unsigned char  	config = 0;
	unsigned short	counter = 0;
	unsigned char 	ext_reg = 0;
	unsigned char 	timer_count0 = 0;
	unsigned char 	timer_count1 = 0;

	ret = kstrtouint(buf, 10, &value);
	if(ret < 0)
	{
		dev_warn(dev, "switch string error!.\n");
		return ret;
	}
	timeout = value;
	//dev_warn(dev, "timeout:%u\n", timeout);	

	if(value <= timer_setting[0].range)
	{
		config = timer_setting[0].config;	
		counter = (value*1000)/timer_setting[0].ms_per_tick;
		
	}else if(value <= timer_setting[1].range)
	{
		config = timer_setting[1].config;
		counter = (value*1000)/timer_setting[1].ms_per_tick;
	}else if(value <= timer_setting[2].range)
	{
		config = timer_setting[2].config;
		counter = value/timer_setting[2].ms_per_tick;
	}else if(value <= timer_setting[3].range)
	{
		config = timer_setting[3].config;
		counter = value/timer_setting[3].ms_per_tick;
	}else{
		dev_warn(dev, "error: over the max timeout.\n");
		return -1;
	}

	if(counter < RX8731_MIN_TCOUNT)
	{
		counter = RX8731_MIN_TCOUNT;
	}

	if(counter > RX8731_MAX_TCOUNT)
	{
		counter = RX8731_MAX_TCOUNT;
	}

	//dev_warn(dev, "config:%hhu\n", config);	
	//dev_warn(dev, "counter:%hu\n", counter);	
	

	cur_value = i2c_smbus_read_byte_data(rx8731->client, RX8731_EXT); 
	if(cur_value < 0)
	{
		return cur_value;
	}
	ext_reg = cur_value;	
	ext_reg = ext_reg&(~RX8731_EXT_TSEL);
	ext_reg = ext_reg+config;

	timer_count0 = counter&0xff;
	timer_count1 = (counter>>8)&0xff;

	//dev_warn(dev, "ext reg:%hhu\n", ext_reg);
	//dev_warn(dev, "count0:%hhu\n", timer_count0);
	//dev_warn(dev, "count1:%hhu\n", timer_count1);

	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_EXT, ext_reg);
	if(ret < 0)
	{
		dev_warn(dev, "write TIMER COUNT1 data failed!\n\n");	
		return ret;
	}

	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_TCOUNT0, timer_count0);
	if(ret < 0)
	{
		dev_warn(dev, "write TIMER COUNT0 data failed!\n\n");	
		return ret;
	}
	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_TCOUNT1, timer_count1);
	if(ret < 0)
	{
		dev_warn(dev, "write TIMER COUNT1 data failed!\n\n");	
		return ret;
	}

	return count;
}

static ssize_t show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	unsigned char ext_reg = 0;
	unsigned char ctrl_reg = 0;
	unsigned int  enable = 0;
	int value = 0;

	value = i2c_smbus_read_byte_data(rx8731->client, RX8731_EXT);
	if(value < 0)
	{
		dev_warn(dev, "read error.\n");	
		return value;
	}
	ext_reg = value;

	value = i2c_smbus_read_byte_data(rx8731->client, RX8731_CTRL);
	if(value < 0)
	{
		dev_warn(dev, "read error.\n");	
		return value;
	}
	ctrl_reg = value;
	if((ext_reg & RX8731_EXT_TE) && (ctrl_reg & RX8731_CTRL_TIE))
	{
		enable = 1;
	}

	
	return sprintf(buf, "%u\n", enable);	
}
static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	int  value = 0;
	int  ret = 0;
	unsigned char ext_reg = 0;
	unsigned char ctrl_reg = 0;
	unsigned int  enable = 0;

	ret = kstrtouint(buf, 10, &enable);
	if(ret < 0)
	{
		dev_warn(dev, "switch string error!.\n");
		return ret;
	}


	value = i2c_smbus_read_byte_data(rx8731->client, RX8731_EXT);
	if(value < 0)
	{
		dev_warn(dev, "read error.\n");	
		return value;
	}
	ext_reg = value;
	/*start timer or stop timer*/
	if(enable != 0)	
	{
		ext_reg = ext_reg | RX8731_EXT_TE;
	}else{
		ext_reg = ext_reg & (~RX8731_EXT_TE);
	}

	value = i2c_smbus_read_byte_data(rx8731->client, RX8731_CTRL);
	if(value < 0)
	{
		dev_warn(dev, "read error.\n");	
		return value;
	}

	ctrl_reg = value;
	/*enable interrupt or disable interrupt*/
	if(enable != 0)
	{
		ctrl_reg = ctrl_reg | RX8731_CTRL_TIE;
	}else{
		ctrl_reg = ctrl_reg & (~RX8731_CTRL_TIE);
	}

	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_CTRL, ctrl_reg);
	if(ret < 0)
	{
		dev_warn(dev, "write control register data failed!\n\n");	
		return ret;
	}

	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_EXT, ext_reg);
	if(ret < 0)
	{
		dev_warn(dev, "write control register data failed!\n\n");	
		return ret;
	}
	
	return count;
}


static DEVICE_ATTR(uart_mode, 0644, show_uart_mode, store_uart_mode);
static DEVICE_ATTR(timeout, 0644, show_timeout, store_timeout);
static DEVICE_ATTR(enable, 0644, show_enable, store_enable);

static irqreturn_t rx8731_irq_1_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct rx8731_data *rx8731 = i2c_get_clientdata(client);
	int flagreg;

	mutex_lock(&rx8731->rtc->ops_lock);

	flagreg = i2c_smbus_read_byte_data(client, RX8731_FLAG);

	if (flagreg <= 0) {
		mutex_unlock(&rx8731->rtc->ops_lock);
		return IRQ_NONE;
	}

	if (flagreg & RX8731_FLAG_VLF)
		dev_warn(&client->dev, "Frequency stop detected\n");

	if (flagreg & RX8731_FLAG_TF) {
		flagreg &= ~RX8731_FLAG_TF;
		rtc_update_irq(rx8731->rtc, 1, RTC_PF | RTC_IRQF);
	}

	if (flagreg & RX8731_FLAG_AF) {
		flagreg &= ~RX8731_FLAG_AF;
		rtc_update_irq(rx8731->rtc, 1, RTC_AF | RTC_IRQF);
	}

	if (flagreg & RX8731_FLAG_UF) {
		flagreg &= ~RX8731_FLAG_UF;
		rtc_update_irq(rx8731->rtc, 1, RTC_UF | RTC_IRQF);
	}

	i2c_smbus_write_byte_data(client, RX8731_FLAG, flagreg);

	mutex_unlock(&rx8731->rtc->ops_lock);
	return IRQ_HANDLED;
}

static int rx8731_get_time(struct device *dev, struct rtc_time *dt)
{
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	u8 date[7];
	int flagreg;
	int err;

	flagreg = i2c_smbus_read_byte_data(rx8731->client, RX8731_FLAG);
	if (flagreg < 0)
		return flagreg;

	if (flagreg & RX8731_FLAG_VLF) {
		dev_warn(dev, "Frequency stop detected\n");
		return -EINVAL;
	}

	err = i2c_smbus_read_i2c_block_data(rx8731->client, RX8731_SEC,
					    7, date);
	if (err != 7)
		return err < 0 ? err : -EIO;

	dt->tm_sec = bcd2bin(date[RX8731_SEC - RX8731_SEC] & 0x7f);
	dt->tm_min = bcd2bin(date[RX8731_MIN - RX8731_SEC] & 0x7f);
	dt->tm_hour = bcd2bin(date[RX8731_HOUR - RX8731_SEC] & 0x3f);
	dt->tm_mday = bcd2bin(date[RX8731_MDAY - RX8731_SEC] & 0x3f);
	dt->tm_mon = bcd2bin(date[RX8731_MONTH - RX8731_SEC] & 0x1f) - 1;
	dt->tm_year = bcd2bin(date[RX8731_YEAR - RX8731_SEC]) + 100;
	dt->tm_wday = ffs(date[RX8731_WDAY - RX8731_SEC] & 0x7f);

	return rtc_valid_tm(dt);
}

static int rx8731_set_time(struct device *dev, struct rtc_time *dt)
{
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	u8 date[7];
	int ctrl, flagreg;
	int ret;

	if ((dt->tm_year < 100) || (dt->tm_year > 199))
		return -EINVAL;

	/* set STOP bit before changing clock/calendar */
	ctrl = i2c_smbus_read_byte_data(rx8731->client, RX8731_CTRL);
	if (ctrl < 0)
		return ctrl;
	rx8731->ctrlreg = ctrl | RX8731_CTRL_STOP;
	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_CTRL,
					rx8731->ctrlreg);
	if (ret < 0)
		return ret;

	date[RX8731_SEC - RX8731_SEC] = bin2bcd(dt->tm_sec);
	date[RX8731_MIN - RX8731_SEC] = bin2bcd(dt->tm_min);
	date[RX8731_HOUR - RX8731_SEC] = bin2bcd(dt->tm_hour);
	date[RX8731_MDAY - RX8731_SEC] = bin2bcd(dt->tm_mday);
	date[RX8731_MONTH - RX8731_SEC] = bin2bcd(dt->tm_mon + 1);
	date[RX8731_YEAR - RX8731_SEC] = bin2bcd(dt->tm_year - 100);
	date[RX8731_WDAY - RX8731_SEC] = bin2bcd(1 << dt->tm_wday);

	ret = i2c_smbus_write_i2c_block_data(rx8731->client,
					     RX8731_SEC, 7, date);
	if (ret < 0)
		return ret;

	/* clear STOP bit after changing clock/calendar */
	ctrl = i2c_smbus_read_byte_data(rx8731->client, RX8731_CTRL);
	if (ctrl < 0)
		return ctrl;
	rx8731->ctrlreg = ctrl & ~RX8731_CTRL_STOP;
	ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_CTRL,
					rx8731->ctrlreg);
	if (ret < 0)
		return ret;

	flagreg = i2c_smbus_read_byte_data(rx8731->client, RX8731_FLAG);
	if (flagreg < 0) {
		return flagreg;
	}

	if (flagreg & RX8731_FLAG_VLF)
		ret = i2c_smbus_write_byte_data(rx8731->client, RX8731_FLAG,
						flagreg & ~RX8731_FLAG_VLF);

	return 0;
}

static int rx8731_init_client(struct i2c_client *client)
{
	struct rx8731_data *rx8731 = i2c_get_clientdata(client);
	u8 ctrl[2];
	int need_clear = 0, err = 0;
	int value = 0;
	unsigned char ext_reg = 0;

	value = i2c_smbus_read_byte_data(rx8731->client, RX8731_EXT);
        if(value < 0)
        {
		value=0;
        }
	ext_reg = value;
	/*stop timer counter*/
	ext_reg = ext_reg & (~RX8731_EXT_TE);

	err = i2c_smbus_write_byte_data(client, RX8731_EXT, ext_reg);
	if (err < 0)
		return err;
	

	err = i2c_smbus_read_i2c_block_data(rx8731->client, RX8731_FLAG,
					    2, ctrl);
	if (err != 2)
		return err < 0 ? err : -EIO;

	if (ctrl[0] & RX8731_FLAG_VLF)
		dev_warn(&client->dev, "Frequency stop was detected\n");

	if (ctrl[0] & RX8731_FLAG_AF) {
		dev_warn(&client->dev, "Alarm was detected\n");
		need_clear = 1;
	}

	if (ctrl[0] & RX8731_FLAG_TF)
		need_clear = 1;

	if (ctrl[0] & RX8731_FLAG_UF)
		need_clear = 1;

	if (need_clear) {
		ctrl[0] &= ~(RX8731_FLAG_AF | RX8731_FLAG_TF | RX8731_FLAG_UF);
		err = i2c_smbus_write_byte_data(client, RX8731_FLAG, ctrl[0]);
		if (err < 0)
			return err;
	}

	rx8731->ctrlreg = (ctrl[1] & ~RX8731_CTRL_TEST);

	return err;
}

static int rx8731_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	struct i2c_client *client = rx8731->client;
	u8 alarmvals[3];
	int flagreg;
	int err;

	err = i2c_smbus_read_i2c_block_data(client, RX8731_ALMIN, 3, alarmvals);
	if (err != 3)
		return err < 0 ? err : -EIO;

	flagreg = i2c_smbus_read_byte_data(client, RX8731_FLAG);
	if (flagreg < 0)
		return flagreg;

	t->time.tm_sec = 0;
	t->time.tm_min = bcd2bin(alarmvals[0] & 0x7f);
	t->time.tm_hour = bcd2bin(alarmvals[1] & 0x3f);

	if (!(alarmvals[2] & RX8731_ALARM_AE))
		t->time.tm_mday = bcd2bin(alarmvals[2] & 0x7f);

	t->enabled = !!(rx8731->ctrlreg & RX8731_CTRL_AIE);
	t->pending = (flagreg & RX8731_FLAG_AF) && t->enabled;

	return err;
}

static int rx8731_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	u8 alarmvals[3];
	int extreg, flagreg;
	int err;

	flagreg = i2c_smbus_read_byte_data(client, RX8731_FLAG);
	if (flagreg < 0) {
		return flagreg;
	}

	if (rx8731->ctrlreg & (RX8731_CTRL_AIE | RX8731_CTRL_UIE)) {
		rx8731->ctrlreg &= ~(RX8731_CTRL_AIE | RX8731_CTRL_UIE);
		err = i2c_smbus_write_byte_data(rx8731->client, RX8731_CTRL,
						rx8731->ctrlreg);
		if (err < 0) {
			return err;
		}
	}

	flagreg &= ~RX8731_FLAG_AF;
	err = i2c_smbus_write_byte_data(rx8731->client, RX8731_FLAG, flagreg);
	if (err < 0)
		return err;

	alarmvals[0] = bin2bcd(t->time.tm_min);
	alarmvals[1] = bin2bcd(t->time.tm_hour);
	alarmvals[2] = bin2bcd(t->time.tm_mday);

	err = i2c_smbus_write_i2c_block_data(rx8731->client, RX8731_ALMIN,
					     2, alarmvals);
	if (err < 0)
		return err;

	extreg = i2c_smbus_read_byte_data(client, RX8731_EXT);
	if (extreg < 0)
		return extreg;

	extreg |= RX8731_EXT_WADA;
	err = i2c_smbus_write_byte_data(rx8731->client, RX8731_EXT, extreg);
	if (err < 0)
		return err;

	if (alarmvals[2] == 0)
		alarmvals[2] |= RX8731_ALARM_AE;

	err = i2c_smbus_write_byte_data(rx8731->client, RX8731_ALWDAY,
					alarmvals[2]);
	if (err < 0)
		return err;

	if (t->enabled) {
		if (rx8731->rtc->uie_rtctimer.enabled)
			rx8731->ctrlreg |= RX8731_CTRL_UIE;
		if (rx8731->rtc->aie_timer.enabled)
			rx8731->ctrlreg |=
				(RX8731_CTRL_AIE | RX8731_CTRL_UIE);

		err = i2c_smbus_write_byte_data(rx8731->client, RX8731_CTRL,
						rx8731->ctrlreg);
		if (err < 0)
			return err;
	}

	return 0;
}

static int rx8731_alarm_irq_enable(struct device *dev,
				   unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	int flagreg;
	u8 ctrl;
	int err;

	ctrl = rx8731->ctrlreg;

	if (enabled) {
		if (rx8731->rtc->uie_rtctimer.enabled)
			ctrl |= RX8731_CTRL_UIE;
		if (rx8731->rtc->aie_timer.enabled)
			ctrl |= (RX8731_CTRL_AIE | RX8731_CTRL_UIE);
	} else {
		if (!rx8731->rtc->uie_rtctimer.enabled)
			ctrl &= ~RX8731_CTRL_UIE;
		if (!rx8731->rtc->aie_timer.enabled)
			ctrl &= ~RX8731_CTRL_AIE;
	}

	flagreg = i2c_smbus_read_byte_data(client, RX8731_FLAG);
	if (flagreg < 0)
		return flagreg;

	flagreg &= ~RX8731_FLAG_AF;
	err = i2c_smbus_write_byte_data(rx8731->client, RX8731_FLAG, flagreg);
	if (err < 0)
		return err;

	if (ctrl != rx8731->ctrlreg) {
		rx8731->ctrlreg = ctrl;
		err = i2c_smbus_write_byte_data(rx8731->client, RX8731_CTRL,
						rx8731->ctrlreg);
		if (err < 0)
			return err;
	}

	return 0;
}

static int rx8731_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx8731_data *rx8731 = dev_get_drvdata(dev);
	int ret, tmp;
	int flagreg;

	switch (cmd) {
	case RTC_VL_READ:
		flagreg = i2c_smbus_read_byte_data(rx8731->client, RX8731_FLAG);
		if (flagreg < 0)
			return flagreg;

		tmp = !!(flagreg & RX8731_FLAG_VLF);
		if (copy_to_user((void __user *)arg, &tmp, sizeof(int)))
			return -EFAULT;

		return 0;

	case RTC_VL_CLR:
		flagreg = i2c_smbus_read_byte_data(rx8731->client, RX8731_FLAG);
		if (flagreg < 0) {
			return flagreg;
		}

		flagreg &= ~RX8731_FLAG_VLF;
		ret = i2c_smbus_write_byte_data(client, RX8731_FLAG, flagreg);
		if (ret < 0)
			return ret;

		return 0;

	default:
		return -ENOIOCTLCMD;
	}
}

static struct rtc_class_ops rx8731_rtc_ops = {
	.read_time = rx8731_get_time,
	.set_time = rx8731_set_time,
	.ioctl = rx8731_ioctl,
};

static int rx8731_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rx8731_data *rx8731;
	int err = 0;
	int ret = 0;


	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA
		| I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&adapter->dev, "doesn't support required functionality\n");
		return -EIO;
	}

	rx8731 = devm_kzalloc(&client->dev, sizeof(struct rx8731_data),
			      GFP_KERNEL);
	if (!rx8731)
		return -ENOMEM;

	rx8731->client = client;
	i2c_set_clientdata(client, rx8731);

	err = rx8731_init_client(client);
	if (err)
		return err;

	if (client->irq > 0) {
		dev_info(&client->dev, "IRQ %d supplied\n", client->irq);
		err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						rx8731_irq_1_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"rx8731", client);

		if (err) {
			dev_err(&client->dev, "unable to request IRQ\n");
			client->irq = 0;
		} else {
			rx8731_rtc_ops.read_alarm = rx8731_read_alarm;
			rx8731_rtc_ops.set_alarm = rx8731_set_alarm;
			rx8731_rtc_ops.alarm_irq_enable = rx8731_alarm_irq_enable;
		}
	}

	rx8731->rtc = devm_rtc_device_register(&client->dev, client->name,
		&rx8731_rtc_ops, THIS_MODULE);

	if (IS_ERR(rx8731->rtc)) {
		dev_err(&client->dev, "unable to register the class device\n");
		return PTR_ERR(rx8731->rtc);
	}

	rx8731->rtc->max_user_freq = 1;

	ret = sysfs_create_file(&(client->dev.kobj), &dev_attr_uart_mode.attr);
	if(ret != 0)
	{
		pr_warn("sys create file failed.!\n");
		return ret;
	}

	ret = sysfs_create_file(&(client->dev.kobj), &dev_attr_timeout.attr);
	if(ret != 0)
	{
		pr_warn("sys create file failed.!\n");
		return ret;
	}

	ret = sysfs_create_file(&(client->dev.kobj), &dev_attr_enable.attr);
	if(ret != 0)
	{
		pr_warn("sys create file failed.!\n");
		return ret;
	}

	return err;
}





static int rx8731_remove(struct i2c_client *client)
{
	sysfs_remove_file(&(client->dev.kobj), &dev_attr_uart_mode.attr);
	sysfs_remove_file(&(client->dev.kobj), &dev_attr_timeout.attr);
	sysfs_remove_file(&(client->dev.kobj), &dev_attr_enable.attr);
	return 0;
}

static struct i2c_driver rx8731_driver = {
	.driver = {
		.name = "rtc-rx8731",
		.of_match_table = of_match_ptr(rx8731_of_match),
	},
	.probe		= rx8731_probe,
	.remove		= rx8731_remove,
	.id_table	= rx8731_id,
};

module_i2c_driver(rx8731_driver);

MODULE_AUTHOR("Akshay Bhat <akshay.bhat@timesys.com>");
MODULE_DESCRIPTION("Epson RX8731SJ RTC driver");
MODULE_LICENSE("GPL v2");

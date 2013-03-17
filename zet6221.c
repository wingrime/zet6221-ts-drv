/*
 * drivers/input/touchscreen/zet6221.c
 *
 * zet6221.c driver based on ft5x_ts source
 *
 * Copyright (c) 2013 Alexsey Shestacov
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include "zet6221.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/div64.h>
#include <linux/jiffies.h>

#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>
#include "ctp_platform_ops.h"

struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

struct zet_ts_data {
	struct input_dev	*input_dev;

	struct work_struct pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct timer_list pen_release_timer;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};

static struct class *i2c_dev_class;
static LIST_HEAD(i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);


static struct i2c_client *this_client;


/* specific tp related macro: need be configured for specific tp */
#define CTP_IRQ_NO			(gpio_int_info[0].port_num)

#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
#define CTP_NAME			"zet6221_ts"
#define TS_RESET_LOW_PERIOD		(1)
#define TS_INITIAL_HIGH_PERIOD		(30)
#define TS_WAKEUP_LOW_PERIOD	(20)
#define TS_WAKEUP_HIGH_PERIOD	(20)
#define SCREEN_MAX_X			(screen_max_x)
#define SCREEN_MAX_Y			(screen_max_y)
#define PRESS_MAX			(255)
#define TS_PRESSURE 200

#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS        (0x70>>1)

static void * __iomem gpio_addr;
static int gpio_int_hdle;
static int gpio_wakeup_hdle;
static int gpio_reset_hdle;
static int gpio_wakeup_enable = 1;
static int gpio_reset_enable = 1;
static int gpio_power_hdle;
static user_gpio_set_t gpio_int_info[1];

static int screen_max_x;
static int screen_max_y;
static int revert_x_flag;
static int revert_y_flag;
static int exchange_x_y_flag;
static int int_cfg_addr[] = {PIO_INT_CFG0_OFFSET, PIO_INT_CFG1_OFFSET,
			   PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
} u_i2c_addr = { { 0x00 } , };
static __u32 twi_id;

/*
 * ctp_get_pendown_state  : get the int_line data state,
 * return value:
 *             return PRESS_DOWN: if down
 *             return FREE_UP: if up,
 *             return 0: do not need process, equal free up.
 */
static int ctp_get_pendown_state(void)
{
	unsigned int reg_val;
	static int state = FREE_UP;

	reg_val = readl(gpio_addr + PIOH_DATA);
	if (!(reg_val&(1<<CTP_IRQ_NO)))
		state = PRESS_DOWN;
	else
		state = FREE_UP;
	return state;
}

/*
* ctp_clear_penirq - clear int pending
* it very stange for me that TS driver must
* touch IRQ controller at all
* more strange that system hungs without it
* that means IRQ controller not removes
* IRQ flag
*/
static void ctp_clear_penirq(void)
{	int reg_val;
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	reg_val = (reg_val&(1<<(CTP_IRQ_NO)));
	writel(reg_val, gpio_addr + PIO_INT_STAT_OFFSET);
	return;
}

/*
* ctp_set_irq_mode - according fex subkey "ctp_int_port" to config int port.
* return value:
*              0:      success;
*              others: fail;
*/
static int ctp_set_irq_mode(char *major_key, char *subkey,
			ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	/* config gpio to int mode */
	pr_info("%s: config gpio to int mode.\n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if (gpio_int_hdle)
		gpio_release(gpio_int_hdle, 2);

	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if (!gpio_int_hdle) {
		pr_info("request tp_int_port failed.\n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	pr_info("%s, %d: gpio_int_info, port = %d, port_num = %d.\n",
		__func__, __LINE__, gpio_int_info[0].port,
		gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	pr_info(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val, gpio_addr+int_cfg_addr[reg_addr]);
	ctp_clear_penirq();

	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET);
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val, gpio_addr+PIO_INT_CTRL_OFFSET);
#endif

request_tp_int_port_failed:
	return ret;
}

/*
* ctp_set_gpio_mode - according fex's subkey "ctp_io_port" to config io port.
* return value:
*              0:      success;
*              others: fail;
*/
static int ctp_set_gpio_mode(void)
{
	int ret = 0;
	pr_info("%s: config gpio to io mode.\n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if (gpio_int_hdle)
		gpio_release(gpio_int_hdle, 2);
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_io_port");
	if (!gpio_int_hdle) {
		pr_info("request ctp_io_port failed.\n");
		ret = -1;
		goto request_tp_io_port_failed;
	}
#endif
	return ret;

request_tp_io_port_failed:
	return ret;
}

/**
 * ctp_free_platform_resource - corresponding with ctp_init_platform_resource
 */
static void ctp_free_platform_resource(void)
{
	if (gpio_addr)
		iounmap(gpio_addr);
	if (gpio_int_hdle)
		gpio_release(gpio_int_hdle, 2);
	if (gpio_wakeup_hdle)
		gpio_release(gpio_wakeup_hdle, 2);
	if (gpio_reset_hdle)
		gpio_release(gpio_reset_hdle, 2);
	if (gpio_power_hdle)
		gpio_release(gpio_power_hdle, 2);
	return;
}


/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO :  i/o err.
 */
static int ctp_init_platform_resource(void)
{
	int ret = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	if (!gpio_addr) {
		ret = -EIO;
		goto exit_ioremap_failed;
	}
	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if (!gpio_wakeup_hdle) {
		pr_warning("%s: No valid wakeup wire defined in fex.\n",
			__func__);
		gpio_wakeup_enable = 0;
	}

	gpio_reset_hdle = gpio_request_ex("ctp_para", "ctp_reset");
	if (!gpio_reset_hdle) {
		pr_warning("%s: No valid reset wire defined in fex.\n",
			__func__);
		gpio_reset_enable = 0;
	}
	/* On some tables (for example: Explay informer 801) ts powered over*/
	/* MOSFET switch, with gate connected to GPIO pin. */
	gpio_power_hdle = gpio_request_ex("ctp_para", "ctp_power_port");
	if (!gpio_power_hdle)
		pr_info("%s: No valid power port wire defined in fex.\n",
			__func__);
	else {
		ret = gpio_write_one_pin_value(gpio_power_hdle,
					1, "ctp_power_port");
		if (ret != EGPIO_SUCCESS)
			pr_info("%s: ctp_power_port gpio set error.\n",
				__func__);
		else
			pr_info("%s: power port enabled\n",
				__func__);
	}
	return ret;

exit_ioremap_failed:
	ctp_free_platform_resource();
	return ret;
}


/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];
	__u32 twi_addr = 0;
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;

	pr_info("%s.\n", __func__);

	ret = script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1);
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", __func__);
		goto script_parser_fetch_err;
	}
	if (1 != ctp_used) {
		pr_err("%s: ctp_unused.\n",  __func__);
		return -1;
	}
	ret = script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name),
				&type, sizeof(name)/sizeof(int));
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", __func__);
		goto script_parser_fetch_err;
	}
	if (strcmp(CTP_NAME, name)) {
		pr_err("%s: name %s does not match CTP_NAME.\n",
			__func__, name);
		pr_err(CTP_NAME);
		return -1;
	}
	ret = script_parser_fetch("ctp_para", "ctp_twi_addr",
				&twi_addr, sizeof(twi_addr)/sizeof(__u32));
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", name);
		goto script_parser_fetch_err;
	}
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	ret =  script_parser_fetch("ctp_para",	"ctp_twi_id",
				&twi_id,	sizeof(twi_id)/sizeof(__u32));
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", name);
		goto script_parser_fetch_err;
	}
	pr_info("%s: ctp_twi_id is %d.\n", __func__, twi_id);

	ret = script_parser_fetch("ctp_para", "ctp_screen_max_x",
				&screen_max_x, 1);
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_x = %d.\n", __func__, screen_max_x);

	ret =  script_parser_fetch("ctp_para", "ctp_screen_max_y",
				&screen_max_y, 1);
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_y = %d.\n", __func__, screen_max_y);
	ret = script_parser_fetch("ctp_para", "ctp_revert_x_flag",
				&revert_x_flag, 1);
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_x_flag = %d.\n", __func__, revert_x_flag);

	ret =  script_parser_fetch("ctp_para", "ctp_revert_y_flag",
				&revert_y_flag, 1);
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("%s: script_parser_fetch err.\n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_y_flag = %d.\n", __func__, revert_y_flag);

	ret = script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag",
				&exchange_x_y_flag, 1);
	if (ret != SCRIPT_PARSER_OK) {
		pr_err("ft5x_ts: script_parser_fetch err.\n");
		goto script_parser_fetch_err;
	}
	pr_info("%s: exchange_x_y_flag = %d.\n", __func__,
		exchange_x_y_flag);

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}

static void ctp_reset(void)
{
	int ret = -1;
	if (gpio_reset_enable) {
		pr_info("%s.\n", __func__);
		ret = gpio_write_one_pin_value(gpio_reset_hdle, 0, "ctp_reset");
		if (ret != EGPIO_SUCCESS)
			pr_info("%s: err when operate gpio.\n", __func__);
		mdelay(TS_RESET_LOW_PERIOD);
		ret = gpio_write_one_pin_value(gpio_reset_hdle, 1, "ctp_reset");
		if (ret != EGPIO_SUCCESS)
			pr_info("%s: err when operate gpio.\n", __func__);
		mdelay(TS_INITIAL_HIGH_PERIOD);
	}
}

static void ctp_wakeup(void)
{
	int ret = -1;
	if (gpio_wakeup_enable) {
		pr_info("%s.\n", __func__);
		ret = gpio_write_one_pin_value(gpio_wakeup_hdle,
					0, "ctp_wakeup");
		if (ret != EGPIO_SUCCESS)
			pr_info("%s: err when operate gpio.\n", __func__);
		mdelay(TS_WAKEUP_LOW_PERIOD);
		ret = gpio_write_one_pin_value(gpio_wakeup_hdle,
					1, "ctp_wakeup");
		if (ret != EGPIO_SUCCESS)
			pr_info("%s: err when operate gpio.\n", __func__);
		mdelay(TS_WAKEUP_HIGH_PERIOD);
	}
}
/**
 * ctp_detect - Device detection callback for automatic device creation
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (twi_id == adapter->nr) {
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			__func__, CTP_NAME,
			i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	} else {
		return -ENODEV;
	}
}

static struct ctp_platform_ops ctp_ops = {
	.get_pendown_state = ctp_get_pendown_state,
	.clear_penirq	   = ctp_clear_penirq,
	.set_irq_mode      = ctp_set_irq_mode,
	.set_gpio_mode     = ctp_set_gpio_mode,
	.init_platform_resource = ctp_init_platform_resource,
	.free_platform_resource = ctp_free_platform_resource,
	.fetch_sysconfig_para = ctp_fetch_sysconfig_para,
	.ts_reset =          ctp_reset,
	.ts_wakeup =         ctp_wakeup,
	.ts_detect = ctp_detect,
};

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	spin_lock(&i2c_dev_list_lock);

	list_for_each_entry(i2c_dev, &i2c_dev_list, list) {
		pr_info("--line = %d ,i2c_dev->adapt->nr = %d,index = %d.\n",
			__LINE__, i2c_dev->adap->nr, index);
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
found:
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev ;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		pr_info("i2c-dev:out of device minors (%d).\n", adap->nr);
		return ERR_PTR(-ENODEV);
	}
	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);

	return i2c_dev;
}
static int zet_i2c_rxdata(char *rxdata, int length);


/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_read_interface(u8 bt_ctpm_addr, u8 *pbt_buf, u16 dw_lenth)
{
	int ret;

	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if (ret != dw_lenth) {
		pr_info("ret = %d.\n", ret);
		pr_info("i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}
/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_write_interface(u8 bt_ctpm_addr, u8 *pbt_buf, u16 dw_lenth)
{
	int ret;
	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);
	if (ret != dw_lenth) {
		pr_info("i2c_write_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}



u8 zet_register_read(u8 e_reg_name, u8 *pbt_buf, u8 bt_len)
{
	u8 read_cmd[3] = {0};
	u8 cmd_len = 0;

	read_cmd[0] = e_reg_name;
	cmd_len = 1;

	if (!i2c_write_interface(I2C_CTPM_ADDRESS, read_cmd, cmd_len))
		return FTS_FALSE;

	/*call the read callback function to get the register value*/
	if (!i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len))
		return FTS_FALSE;
	return FTS_TRUE;
}

int zet_register_write(u8 e_reg_name, u8 bt_value)
{
	unsigned char write_cmd[2] = {0};

	write_cmd[0] = e_reg_name;
	write_cmd[1] = bt_value;

	/*call the write callback function*/
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, 2);

}

/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in] :the valid input parameter numbers,
    if only command code needed and no parameters followed,then the num is 1;
[return]:
FTS_TRUE    :success;
FTS_FALSE    :io fail;
*/
int cmd_write(u8 btcmd, u8 btPara1, u8 btPara2, u8 btPara3, u8 num)
{
	unsigned char  write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}
int  byte_write(u8 *pbt_buf, u16 dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}


int byte_read(u8 *pbt_buf, u8 bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

static int zet_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if (ret < 0)
		pr_info("msg %s i2c read error: %d\n", __func__, ret);
	return ret;
}

static int zet_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);
	return ret;
}

static int zet_set_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = zet_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}
	udelay(50);
	return 0;
}


/*
 *      <ALW> <fing>
 * ts0x: 0x3c  0x80    0x0 0x10 0x8f 0xe9 0x1 0xff 0xff 0xff
 *   1-touch ts[1] 1000-0000 - 0x80
 *   2-touch ts[1] 1100-0000   0xc0
 *   3 touch ts[1] 1110-0000   0xe0
 *   4-touch ts[1] 1111-0000   0xf0
 *   5-touch ts[1] 1111-1000   0xf8
 *
 */
static void  zet_ts_pen_irq_work(struct work_struct *work)
{
	struct zet_ts_data *data = i2c_get_clientdata(this_client);
	unsigned char buf[71] = {0};
	int ret = -1;

	unsigned int x, y, p;
	ret = zet_i2c_rxdata(buf, 24);
	if (ret < 0) {
		pr_info("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return;
	}
	/*  pen release */
	if (!buf[1]) {
		/*
		pr_info("pen release\n");
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(data->input_dev, ABS_PRESSURE, 0);
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_sync(data->input_dev);
		*/
		mod_timer(&data->pen_release_timer,
			jiffies + msecs_to_jiffies(50));
		return;
	}
	if ((buf[1] & 0x80) &&  ((buf[6] & 0xf) == 1)) {
		if (buf[3] == 0xff)
			pr_info("buf3 err\n");
		if (buf[4] == 0xff)
			pr_info("buf4 err\n");
		if (buf[5] == 0xff)
			pr_info("buf5 err\n");
		x = ((unsigned int)((u8)buf[3]>>4)<<8) + (u8)buf[4];
		y = ((unsigned int)((u8)buf[3] & (u8)0xf)<<8) + (u8)buf[5];
		p = buf[6] & 0xf;
		pr_info("zet: x1 = %d, y1 = %d z = %d ,z2 = %d.\n", x, y, p,
			((unsigned int)((u8)buf[6]>>4)<<8));
		if (x > 950)
			x = 950;
		if (y > 630)
			y = 630;
		if (x < 4)
			x = 4;
		if (y < 4)
			y = 4;
		x = (x-4)*SCREEN_MAX_X;
		do_div(x, 950);
		y = (y-4)*SCREEN_MAX_Y;
		do_div(y, 630);

		if (revert_y_flag)
			y  = SCREEN_MAX_Y - y;
		if (exchange_x_y_flag)
			swap(x, y);
		if (!revert_x_flag)
			x  = SCREEN_MAX_X - x;
		pr_info("zet: x2 = %d, y1 = %d.\n", x, y);
		input_report_abs(data->input_dev,
				ABS_MT_TRACKING_ID, (buf[1] & 0x80));
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(data->input_dev, ABS_MT_PRESSURE, TS_PRESSURE);
		input_report_key(data->input_dev, BTN_TOUCH, 1);
		input_mt_sync(data->input_dev);
		}
	input_sync(data->input_dev);
	mod_timer(&data->pen_release_timer, jiffies + msecs_to_jiffies(50));
}
static irqreturn_t zet_ts_interrupt(int irq, void *dev_id)
{
	struct zet_ts_data *zet_ts = dev_id;

	queue_work(zet_ts->ts_workqueue, &zet_ts->pen_event_work);
	ctp_ops.clear_penirq();
	return IRQ_HANDLED;
}
#define ZET_REG_PMODE 0xb1
#define ZET_REG_CHARGE_DISABLE 0xb6
#define ZET_REG_CHARGE_ENABLE 0xb5
#define PMODE_HIBERNATE 0x1
#ifdef CONFIG_HAS_EARLYSUSPEND

static void zet_ts_suspend(struct early_suspend *handler)
{
/*if device support power port feature simply poweroff ts chip */
	if (gpio_power_hdle) {
		pr_info("zet_ts_suspend: poweroff ts.\n");
		gpio_write_one_pin_value(gpio_power_hdle, 0, "ctp_power_port");
	} else {
		pr_info("zet_ts_suspend: write ZET0X_REG_PMODE.\n");
		zet_set_reg(0x0, ZET_REG_PMODE);
	}
}

static void zet_ts_resume(struct early_suspend *handler)
{
	if (gpio_power_hdle) {
		gpio_write_one_pin_value(gpio_power_hdle, 1, "ctp_power_port");
		mdelay(48);
	} else
		ctp_ops.ts_wakeup();
}
#endif
/* release pen after time out*/
static void zet_pen_release_timer_func(unsigned long arg)
{
	struct zet_ts_data  *data = (struct zet_ts_data *)arg;

	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_sync(data->input_dev);
	pr_info("pen tim\n");

}
static int zet_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct zet_ts_data *zet_ts;
	struct input_dev *input_dev;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int err = 0;


	pr_info("====%s begin=====.\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}


	zet_ts = kzalloc(sizeof(*zet_ts), GFP_KERNEL);
	if (!zet_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	this_client->addr = client->addr;
	i2c_set_clientdata(client, zet_ts);
	/* when finger touches touchscreen, ic sends IRQ , we wait 100 ms,
	 * then report that finger release, and ignore ts release events
	 */
	setup_timer(&zet_ts->pen_release_timer,
		zet_pen_release_timer_func, (unsigned long)zet_ts);
	/* main touchevent work */
	INIT_WORK(&zet_ts->pen_event_work, zet_ts_pen_irq_work);
	zet_ts->ts_workqueue = create_singlethread_workqueue(
		dev_name(&client->dev));
	/*when finger not touches ts we must release pen*/

	if (!zet_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	zet_ts->input_dev = input_dev;

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);

	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev,
			ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
	input_set_abs_params(input_dev,
			ABS_MT_PRESSURE, 0, PRESS_MAX, 0 , 0);

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TRACKING_ID, 0, 4, 0, 0);

	input_dev->name = CTP_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"zet_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	pr_info("==register_early_suspend =\n");
	zet_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	zet_ts->early_suspend.suspend = zet_ts_suspend;
	zet_ts->early_suspend.resume	= zet_ts_resume;
	register_early_suspend(&zet_ts->early_suspend);
#endif

	err = ctp_ops.set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);
	if (0 != err) {
		pr_info("%s:ctp_ops.set_irq_mode err.\n", __func__);
		goto exit_set_irq_mode;
	}
	err = request_irq(SW_INT_IRQNO_PIO, zet_ts_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_SHARED, "zet_ts", zet_ts);

	if (err < 0) {
		dev_err(&client->dev, "zet_ts_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		err = PTR_ERR(i2c_dev);
		return err;
	}
	dev = device_create(i2c_dev_class, &client->adapter->dev,
			MKDEV(I2C_MAJOR, client->adapter->nr),
			NULL, "aw_i2c_ts%d", client->adapter->nr);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		return err;
	}

	pr_info("==%s over =\n", __func__);
	return 0;

exit_irq_request_failed:
exit_set_irq_mode:
	cancel_work_sync(&zet_ts->pen_event_work);
	destroy_workqueue(zet_ts->ts_workqueue);
	enable_irq(SW_INT_IRQNO_PIO);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(SW_INT_IRQNO_PIO, zet_ts);
exit_create_singlethread:
	pr_info("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(zet_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit zet_ts_remove(struct i2c_client *client)
{

	struct zet_ts_data *zet_ts = i2c_get_clientdata(client);
	/*TODO: Place to sleep*/

	pr_info("==zet_ts_remove=\n");
	free_irq(SW_INT_IRQNO_PIO, zet_ts);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&zet_ts->early_suspend);
#endif
	input_unregister_device(zet_ts->input_dev);
	input_free_device(zet_ts->input_dev);
	cancel_work_sync(&zet_ts->pen_event_work);
	del_timer_sync(&zet_ts->pen_release_timer);
	destroy_workqueue(zet_ts->ts_workqueue);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	class_destroy(i2c_dev_class);
	kfree(zet_ts);

	i2c_set_clientdata(client, NULL);
	ctp_ops.free_platform_resource();

	return 0;

}

static const struct i2c_device_id zet_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, zet_ts_id);

static struct i2c_driver zet_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= zet_ts_probe,
	.remove		= __devexit_p(zet_ts_remove),
	.id_table	= zet_ts_id,
	.driver	= {
		.name	= CTP_NAME,
		.owner	= THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
};


static int __init zet_ts_init(void)
{
	int ret = -1;
	int err = -1;

	pr_info("%s\n", __func__);

	gpio_power_hdle = 0;

	if (ctp_ops.fetch_sysconfig_para) {
		if (ctp_ops.fetch_sysconfig_para()) {
			pr_info("%s: err.\n", __func__);
			return -1;
		}
	}
	pr_info("%s:normal_i2c:0x%hx.normal_i2c[1]:0x%hx.\n",
		__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	err = ctp_ops.init_platform_resource();
	if (0 != err)
		pr_info("%s:ctp_ops.init_platform_resource err.\n", __func__);

	/* reset */
	ctp_ops.ts_reset();
	/* wakeup */
	ctp_ops.ts_wakeup();

	zet_ts_driver.detect = ctp_ops.ts_detect;

	i2c_dev_class = class_create(THIS_MODULE, "aw_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}

	ret = i2c_add_driver(&zet_ts_driver);

	return ret;
}

static void __exit zet_ts_exit(void)
{
	i2c_del_driver(&zet_ts_driver);
}

late_initcall(zet_ts_init);
module_exit(zet_ts_exit);

MODULE_AUTHOR("<wingrime@gmail.com");
MODULE_DESCRIPTION("Zet6221 Touchscreen driver");
MODULE_LICENSE("GPL");


#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
//#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
//#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
//#include <linux/sched.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include <net/sock.h>
//#include <net/netlink.h>

//#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
//#endif

#include <linux/miscdevice.h>
//#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
//#include <linux/completion.h>
#include <linux/version.h>
//#include "elan_fp_qcom.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/time.h>
#include <linux/list.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <asm/unaligned.h>

#define VERSION_LOG "0.2.3"

static unsigned int gPrint_point = 0;
static int elan_debug = 1;
#define ELAN_DEBUG(format, args ...) \
do { \
    if (elan_debug) \
        printk("[elan-spi] " format, ##args); \
    } while(0)

#define SPI_MAX_SPEED               10*1000*1000

//request fw
#define ELAN_FW_FILENAME  "elan_fw.ekt"
static uint8_t file_fw_data[] = {
    #include "fw_data.i"
};
#define PAGERETRY 				3
#define FLASH_MODE_RETRY 		1

#define PROTOCOL_B    /* Default: PROTOCOL B */
#define FINGER_NUM 				10
#define ELAN_PEN_OSR			260

#define MAX_FINGER_SIZE        	255
#define MAX_FINGER_PRESSURE		4096

#define BC_HELLO_LEN			10

#define DELAY_MS_IDLE_DUMMY		60

/* Command header definition */
#define CMD_HEADER_READ			0x53

/* FW read command, 0x53 0x?? 0x0, 0x01 */
#define E_ELAN_INFO_REK			0xD0

/* FW position data */
//#define PACKET_SIZE            	50      /* support SPI 10 fingers & USI Pen packet for 7315 */
#define PACKET_SIZE            	40      /* support SPI 10 fingers for eKTD6721 finger only */
#define FW_POS_WIDTH           	35

/* FW header data */
#define HEADER_SIZE				4
#define PEN_PKT        			0x07
#define BC_HELLO_PKT           	0x22
#define MC_HELLO_PKT           	0x55
#define TEN_FINGERS_PKT        	0x62
#define CALIB_PKT              	0x66

//#define IRQ_HANDLED				1
/* FW Mode Control */
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)
#define IDLE_MODE_DISABLE		0
#define IDLE_MODE_ENABLE		1
#define IDLE_MODE_MASK			BIT(7)


// For Firmware Update
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_FW_INFO  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)

#define ELAN_VTG_MIN_UV		3008000//2850000
#define ELAN_VTG_MAX_UV		3008000//2850000

#define ELAN_IO_VTG_MIN_UV	1800000
#define ELAN_IO_VTG_MAX_UV	1800000

#if 0 //Finger Printer Sample Code, remember to delete
static int read_all = 2; //0:one frame(int), 1:one row(int), 2:one frame(status cmd), 3:one row(status cmd)

static struct fasync_struct *fasync_queue = NULL;

static int elan_work_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);

static unsigned char IOIRQ_STATUS = 0;
#endif //Finger Printer Sample Code, remember to delete

static unsigned int spi_speed = 10*1000*1000; // 1MHz
static struct workqueue_struct *elan_wq;

static volatile int display_status = 0; // Screen On:0 Off:1

static int spi_read_len = PACKET_SIZE;
static int read_fw_flag = 0;
static int recovery_mode = 1; //1: recovery mode, 0: normal mode
static int wakeup_idle = 0; //1: need wakeup idle, 0: not need wakeup idle
static int enable_idle_mode = 0; //1: enable auto idle mode, 0: disable auto idle mode
static int enter_main_flag = 0; // 1: send enter main code cmd, 0: nothing  done

//int iap_times = 0;
bool bEnableIAP = false;
bool bFirstBoot = false;
bool bBuiltInEkt = false;
int gFlashMode = -1;
int gnFlashModeRetry = FLASH_MODE_RETRY;
int power_flag = 0;

//add by Minger
int tpd_flag;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
//end 

#if defined(CONFIG_FB)
static struct notifier_block fb_notif;
#endif

struct elan_spi_fw_info {
	int fw_ver;
	int fw_id;
	int bc_ver;
	int rx;
	int tx;
	int finger_osr;
	int x_res;
	int y_res;
	int testsolversion;
	int testversion;
	int solutionversion;
};

struct elants_data {
    struct spi_device   	*spi;
    struct input_dev    	*input_dev;
    struct input_dev    	*pen_input_dev;
	struct elan_spi_fw_info fw_info;
    int                 	int_gpio;
    int                 	irq;
    int                 	rst_gpio;
    int                 	resx_gpio;
    int                 	vddi_gpio;
    int                 	apqio1_gpio;
    int                 	apqio2_gpio;
    struct work_struct  	work;
    bool                	irq_status; // 1:enable, 0:disable
    wait_queue_head_t   	elan_wait;
    //struct wake_lock    	wake_lock;
    //struct wake_lock    	hal_wake_lock;
    struct wakeup_source		wake_lock;
    struct wakeup_source	hal_wake_lock;
    struct miscdevice   	elan_dev; /* char device for ioctl */
    struct regulator    	*reg;
	
	struct regulator *vdd;			/*tp vdd*/
	struct regulator *vcc;		/*tp vio*/
};

static struct elants_data *private_ts;

static void elan_reset(struct elants_data *ts);
static int elan_calibrate(struct spi_device *spi);
static int elan_spi_iap(struct spi_device *spi);
static int elan_fw_packet_handler(struct elants_data *ts);
static void elan_ktf_ts_set_auto_idle(struct spi_device *spi);
int EnterMianCode(struct spi_device *spi);

static void elan_irq_disable(struct elants_data *ts)
{
    ELAN_DEBUG("%s +\n", __func__);
    if (ts->irq_status) {
        ts->irq_status = false;
        disable_irq(ts->irq);
    }
}

static void elan_irq_enable(struct elants_data *ts)
{
    ELAN_DEBUG("%s +\n", __func__);
    if (!ts->irq_status) {
        ts->irq_status = true;
        enable_irq(ts->irq);
    }
}

static int elan_ts_poll(int ms, int times)
{
    int status = 0, retry = 0;

    do {
        status = gpio_get_value(private_ts->int_gpio);
        if(status==0)
			break;

        retry++;
        mdelay(ms);
    } while (status == 1 && retry < times);

    ELAN_DEBUG("%s: poll interrupt status %s (%d %d/%d)\n", 
			__func__, status == 1 ? "high" : "low", ms, retry, times);

    return (status == 0 ? 0 : -ETIMEDOUT);
}

static inline int elan_spi_transfer_auto_idle(struct spi_device *spi)
{
	const uint8_t wakeup_spi[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00 };
    struct spi_transfer t = {
        .tx_buf        = wakeup_spi,
        .rx_buf        = NULL,
        .len           = sizeof(wakeup_spi),
        .speed_hz      = spi_speed,
        .bits_per_word = 8,
    };
    struct spi_message m;
    int ret;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret = spi_sync(spi, &m);
    if (ret != 0)
        ELAN_DEBUG("spi_sync failed, ret = %d\n", ret);
    return ret;
}

static inline int elan_spi_transfer(struct spi_device *spi,
            const void *tx_buf, void *rx_buf, const size_t len)
{
    struct spi_transfer t = {
        .tx_buf        = tx_buf,
        .rx_buf        = rx_buf,
        .len           = len,
        .speed_hz      = spi_speed,
        .bits_per_word = 8,
    };
    struct spi_message m;
    int ret;

	if (wakeup_idle && tx_buf != NULL) {
		ret = elan_spi_transfer_auto_idle(spi);
		if (ret != 0)
			return ret;
		mdelay(DELAY_MS_IDLE_DUMMY);
	}

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret = spi_sync(spi, &m);
    if (ret != 0)
        ELAN_DEBUG("spi_sync failed, ret = %d\n", ret);
    return ret;
}

static int elan_spi_write_and_read(struct spi_device *spi, const uint8_t *txbuf,
						const int txlen, uint8_t *rxbuf, const int rxlen)
{
	int ret = 0;

	ret = elan_spi_transfer(spi, txbuf, NULL, txlen);
	if (ret) {
		ELAN_DEBUG("%s spi send command failed return ret = %d, txlen = %d\n",__func__,ret,txlen);
		return ret;
	}			

	ret = elan_ts_poll(5, 50);
	if (ret) {
		ELAN_DEBUG("%s int poll hight\n",__func__);
		return ret;
	}

	ret = elan_spi_transfer(spi,NULL, rxbuf, rxlen);
	if (ret) {
		ELAN_DEBUG("%s read back error return %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int elan_spi_read_mc_hello(struct elants_data *ts)
{
	unsigned char rxbuf[6] = {0};
	int ret = 0;
	
	elan_ts_poll(10, 50);
	
	ret = elan_spi_transfer(ts->spi, NULL, rxbuf, 6);
	if (ret) {
		ELAN_DEBUG("%s error\n", __func__);
		return ret;
	}

	ELAN_DEBUG("%s hello: 0x%2x:0x%2x:0x%2x:0x%2x\n", __func__,rxbuf[0],rxbuf[1],rxbuf[2],rxbuf[3]);

	return ret;
}


static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", VERSION_LOG);
}
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

static ssize_t elan_debug_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (elan_debug) {
        elan_debug=0;
    }
    else {
        elan_debug=1;
    }

    return sprintf(buf, "[elan-spi] elan debug %d\n", elan_debug);
}
static DEVICE_ATTR(elan_debug, S_IRUGO, elan_debug_value, NULL);

static ssize_t store_spi_calibrate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct spi_device *spi_dev = to_spi_device(dev);
    
    
    ELAN_DEBUG("%s: +\n", __func__);
    ret = elan_calibrate(spi_dev);
    if (ret)
        ELAN_DEBUG("%s: SPI Calibrate Fail, error=%d\n", __func__, ret);
    return count;
}
static DEVICE_ATTR(spi_calibrate, S_IWUSR | S_IWGRP, NULL, store_spi_calibrate);

static ssize_t show_calibration_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	const u8 cmd[] = { CMD_HEADER_READ, E_ELAN_INFO_REK, 0x00, 0x01, 0x00, 0x00 };
	u8 resp[HEADER_SIZE];
	u16 rek_count;
	int error;

    elan_irq_disable(private_ts);

	error = elan_spi_write_and_read(spi_dev, cmd, sizeof(cmd), resp, sizeof(resp));
	if (error) {
		ELAN_DEBUG("read ReK status error=%d, buf=%*phC\n",
			error, (int)sizeof(resp), resp);
		return sprintf(buf, "%d\n", error);
	}

	rek_count = get_unaligned_be16(&resp[2]);

	elan_irq_enable(private_ts);

	return sprintf(buf, "0x%04x\n", rek_count);
}
static DEVICE_ATTR(calibration_count, S_IRUGO, show_calibration_count, NULL);

static ssize_t store_spi_iap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //int ret = 0;
    //struct spi_device *spi_dev = to_spi_device(dev);
    
    
    ELAN_DEBUG("%s: +\n", __func__);
/*
    ret = elan_spi_iap(spi_dev);
    if (ret)
        ELAN_DEBUG("%s: SPI IAP Fail, error=%d\n", __func__, ret);
*/
    bEnableIAP = true;
    read_fw_flag = 1;
    elan_reset(private_ts);
    return count;
}
static DEVICE_ATTR(spi_iap, S_IWUSR | S_IWGRP, NULL, store_spi_iap);

static ssize_t store_spi_tp_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    ELAN_DEBUG("%s\n", __func__);
    elan_reset(private_ts);
    return count;
}
static DEVICE_ATTR(spi_tp_reset, S_IWUSR | S_IWGRP, NULL, store_spi_tp_reset);

static ssize_t store_spi_set_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int parameter[1];
    int irq_status = 0;

    sscanf(buf,"%d",&parameter[0]);
    irq_status = parameter[0];
    ELAN_DEBUG("irq_status[0] = %d\n", irq_status);
    if (irq_status != 0)
        elan_irq_enable(private_ts);
    else
        elan_irq_disable(private_ts);

    return count;
}
static DEVICE_ATTR(spi_set_irq, S_IWUSR | S_IWGRP, NULL, store_spi_set_irq);

static ssize_t store_spi_set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int parameter[1];
    int new_speed = 0; //MHz
    int old_speed = spi_speed;

    sscanf(buf,"%d",&parameter[0]);
    new_speed = parameter[0];
    spi_speed = new_speed * 1000 * 1000;
    old_speed /= 1000000;
    ELAN_DEBUG("%s: change spi speed from %dMHz to %dMHz\n", __func__, old_speed, new_speed);

    return count;
}
static DEVICE_ATTR(spi_set_speed, S_IWUSR | S_IWGRP, NULL, store_spi_set_speed);

static ssize_t store_idle_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int parameter[1];
    int new_idle = 0; //MHz
    int old_idle = enable_idle_mode;
    struct spi_device *spi_dev = to_spi_device(dev);

    sscanf(buf,"%d",&parameter[0]);
    new_idle = parameter[0];
    enable_idle_mode = new_idle;
    ELAN_DEBUG("%s: change idle mode from %d to %d\n", __func__, old_idle, new_idle);

	elan_ktf_ts_set_auto_idle(spi_dev);

    return count;
}
static ssize_t show_idle_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	const uint8_t r_idle_status[] = { 0x53, 0xB1, 0x00, 0x01, 0x00, 0x00 };
	char rbuf[4] = {0};
	uint8_t idle_state;
    struct spi_device *spi_dev = to_spi_device(dev);

	elan_irq_disable(private_ts);
	ret = elan_spi_write_and_read(spi_dev, r_idle_status, sizeof(r_idle_status), rbuf, sizeof(rbuf));
	ELAN_DEBUG("%s: response: %*ph\n", __func__, (int)sizeof(rbuf), rbuf);
	elan_irq_enable(private_ts);

    idle_state = (rbuf[3] & IDLE_MODE_MASK) >> 7; //;Bit[7]: IdleMode_B      (0: Off, 1: On)
    ELAN_DEBUG("idle mode = %s\n", 
            idle_state == IDLE_MODE_DISABLE ? "Idle Mode Disable" : "Idle Mode Enable");

	ELAN_DEBUG("EnableIdle=%d,WakeupIdle=%d,%s\n", enable_idle_mode, wakeup_idle, 
				idle_state == IDLE_MODE_DISABLE ? "Idle Mode Disable" : "Idle Mode Enable");
	return sprintf(buf, "EnableIdle=%d,WakeupIdle=%d,%s\n", enable_idle_mode, wakeup_idle,
				idle_state == IDLE_MODE_DISABLE ? "Idle Mode Disable" : "Idle Mode Enable");
}
static DEVICE_ATTR(idle_mode, S_IWUSR | S_IWGRP | S_IRUGO, show_idle_mode, store_idle_mode);

static struct attribute *elan_attributes[] = {
    &dev_attr_drv_version.attr,
    &dev_attr_elan_debug.attr,
    &dev_attr_spi_tp_reset.attr,
    &dev_attr_spi_calibrate.attr,
    &dev_attr_calibration_count.attr,
    &dev_attr_spi_iap.attr,
    &dev_attr_spi_set_irq.attr,
    &dev_attr_spi_set_speed.attr,
    &dev_attr_idle_mode.attr,
    NULL
};

static struct attribute_group elan_attr_group = {
    .name = "elan_ktf",
    .attrs = elan_attributes,
};


static void elan_reset(struct elants_data *ts)
{
    //[TOOD] check delay 50ms or less
    ELAN_DEBUG("%s\n", __func__);
    gpio_set_value(ts->rst_gpio, 0);
    mdelay(1);
    gpio_set_value(ts->rst_gpio, 1);
    mdelay(1);

    //read_fw_flag = 1;
}

static int elan_spi_check_ack(struct spi_device *spi, char *rbuf, const char *ackbuf, int read_len, int ack_len)
{
    int ret = 0;
    //int i = 0;
    unsigned char *rxbuf;

    rxbuf = kmalloc(sizeof(unsigned char) * read_len, GFP_KERNEL);
	if(rxbuf == NULL) {
		rxbuf = kmalloc(sizeof(unsigned char) * read_len, GFP_KERNEL);
		if(rxbuf == NULL)
			goto END;
	}

    ret = elan_spi_transfer(spi, NULL, rxbuf, read_len);
    if (ret != 0) {
        ELAN_DEBUG("spi_sync failed, ret = %d\n", ret);
        goto END;
    }

    //ret = memcmp(ackbuf, rxbuf, (size_t)sizeof(ackbuf));
    ret = memcmp(ackbuf, rxbuf, ack_len);
    if (ret != 0)
        ELAN_DEBUG("check ack failed, ret = %d\n", ret);
    memcpy(rbuf, rxbuf, read_len);
    //ELAN_DEBUG("ack=[%02x, %02x, %02x, %02x]\n", ackbuf[0], ackbuf[1], ackbuf[2], ackbuf[3]);
    //ELAN_DEBUG("rxbuf=[%02x, %02x, %02x, %02x]\n", rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);

    /*//Debug
    printk("[elan-spi] rxbuf(%d)=", read_len);
    for (i = 0; i < read_len; i++) {
        printk(" [%d]0x%02x,", i, rxbuf[i]);
    }
    printk("\n");

    printk("[elan-spi] ackbuf(%d)=", ack_len);
    for (i = 0; i < ack_len; i++) {
        printk(" [%d]0x%02x,", i, ackbuf[i]);
    }
    printk("\n");
    //*/
END:
	if(rxbuf != NULL)
		kfree(rxbuf);
    return ret;
}


static int elan_calibrate(struct spi_device *spi)
{
	int ret;
	const u8 w_flashkey[] = { 0x54, 0xC0, 0xE1, 0x5A, 0x00, 0x00 };
	const u8 rek[] = { 0x54, 0x29, 0x00, 0x01, 0x00, 0x00 };
	const u8 rek_resp[] = { 0x66, 0x66, 0x66, 0x66 };
	//char *rbuf;
	uint8_t rbuf[4] = {0x00};
	
	elan_irq_disable(private_ts);

	ret = elan_spi_transfer(spi, w_flashkey, NULL, sizeof(w_flashkey));
	if(ret)
		ELAN_DEBUG("send flah key failed\n");
	else
		ELAN_DEBUG("[elan] flash key cmd = [%2x, %2x, %2x, %2x]\n",
                 w_flashkey[0], w_flashkey[1], w_flashkey[2], w_flashkey[3]);
				
	msleep(5);  // for debug
	ret = elan_spi_transfer(spi, rek, NULL, sizeof(rek));
	if(ret)
		ELAN_DEBUG("send calibrate failed\n");
	else
		ELAN_DEBUG("[elan] calibration cmd = [%2x, %2x, %2x, %2x]\n",
                 rek[0], rek[1], rek[2], rek[3]);

	elan_ts_poll(5, 50);

    //rbuf = kmalloc(sizeof(unsigned char) * sizeof(rek_resp), GFP_KERNEL);
    ret = elan_spi_check_ack(spi, rbuf, rek_resp, sizeof(rek_resp), sizeof(rek_resp));
    if (ret != 0) {
        ELAN_DEBUG("unexpected calibration response: %*ph\n",
                (int)sizeof(rek_resp), rbuf);
    }

	elan_irq_enable(private_ts);

	return ret;
}

#if 0
static int elants_validate_remark_id(struct spi_device *spi,
					 const struct firmware *fw)
{
    int ret = 0;
	int i;
	const u8 query_remark_id_cmd[] = { 0x96, 0x80, 0x1F, 0x00, 0x00, 0x21 };
	u8 resp[6] = { 0 };
	u16 ts_remark_id = 0;
	u16 fw_remark_id = 0;

	/* Compare TS Remark ID and FW Remark ID */
    for (i = 0; i < (int)sizeof(query_remark_id_cmd); i++) {
		ret = elan_spi_transfer(spi, &query_remark_id_cmd[i], NULL, 1);
		usleep_range(100, 200);
		if (ret != 0) {
			ELAN_DEBUG("%s: query_remark_id_cmd[%d]=0x%02x fail\n", __func__, i, query_remark_id_cmd[i]);
			return ret;
		}
	}

    //elan_spi_poll();
	elan_ts_poll(5, 50);

    ret = elan_spi_transfer(spi, NULL, resp, sizeof(resp));
    if (ret != 0) {
        ELAN_DEBUG("read query_remark_id_cmd response failed, ret = %d\n", ret);
        return ret;
    }

    //*//Debug
    printk("[elan-spi] rbuf(%d)=", (int)sizeof(query_remark_id_cmd));
    for (i = 0; i < sizeof(query_remark_id_cmd); i++) {
        printk(" [%d]0x%02x,", i, resp[i]);
    }
    printk("\n");
    //*/

	ts_remark_id = get_unaligned_be16(&resp[3]);
	fw_remark_id = get_unaligned_le16(&fw->data[fw->size - 4]);
	if (fw_remark_id != ts_remark_id) {
		ELAN_DEBUG("Remark ID Mismatched: ts_remark_id=0x%04x, fw_remark_id=0x%04x.\n",
			ts_remark_id, fw_remark_id);
		//return -EINVAL;
	}
	
	return 0;
}
#endif

static int elan_spi_iap(struct spi_device *spi)
{
    int ret = 0;
    int iap_ack_len = 2;
    int flash_mode = -1; //1 : flash mode, 0 : non-flash mode
    int i = 0;
    char *rbuf;
    const char iap_page_ack[] = {0xAA, 0xAA};
    //unsigned char enter_isp_cmd[4] = {0x45, 0x49, 0x41, 0x50};
    //unsigned char enter_recovery_isp_cmd[4] = {0x54, 0x00, 0x12, 0x34};
    unsigned char dummy_cmd[1] = {0x10};
    const struct firmware *p_fw_entry;
    const uint8_t *szBuff = NULL;
    const u8 *fw_data;
    unsigned char *write_page_cmd;
    int fw_size = 0;
    int PageNum = 0;
    int PageSize = 132;
    int curIndex = 0;
    int rewriteCnt = 0;
	int FlashPageNum = 69; //1~69
	int RemainPage;
	int LastStartPage;
	//int spi_speed_ori = spi_speed;
	//int iap_version = private_ts->fw_info.bc_ver & 0x00FF;
	//int check_remark_id = 0;
    
    ELAN_DEBUG("%s\n", __func__);
	
    flash_mode = gFlashMode;

/*
    //Step 0: Setting
    elan_irq_disable(private_ts);
	if (iap_version >= 0x60)
		check_remark_id = 1;
	else
		check_remark_id = 0;
	ELAN_DEBUG("%s: check_remark_id=%d (0x%x)\n", __func__, check_remark_id, iap_version);
	
	if (wakeup_idle) //force wakeup_idle from 1 to 0 for iap flow
		wakeup_idle = !wakeup_idle;
	ELAN_DEBUG("%s: Force wakeup_idle = %d for IAP\n", __func__, wakeup_idle);
*/

    //Step 1: Open FW EKT file
	if (bBuiltInEkt == true) { //use fw_data.i
		ELAN_DEBUG("%s: Update Firmware by fw_data.i\n", __func__);
        PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);
        fw_data = file_fw_data;
		fw_size = sizeof(file_fw_data);
		ELAN_DEBUG("%s: PageSize=%d, PageNum=%d, EKT size=%d\n", __func__, PageSize, PageNum, fw_size);
	} else { //Standard path : /system/etc/firmware/elan_fw.ekt
		ELAN_DEBUG("%s: request_firmware name = %s\n", __func__, ELAN_FW_FILENAME);
		ret = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &spi->dev);
		if (ret) {
			ELAN_DEBUG("%s: request_firmware fail, ret=%d\n", __func__, ret);
			goto END;
		} else {
			ELAN_DEBUG("%s: Request Firmware Size=%zu\n", __func__, p_fw_entry->size);
		}

		fw_data = p_fw_entry->data;
		fw_size = p_fw_entry->size;
		PageNum = (fw_size/sizeof(uint8_t)/PageSize);
		ELAN_DEBUG("%s: PageSize=%d, PageNum=%d, EKT size=%d\n", __func__, PageSize, PageNum, fw_size);
	}

	RemainPage = PageNum % FlashPageNum;
	LastStartPage = PageNum - RemainPage;
	/*ELAN_DEBUG("%s: FlashPageNum=%d, RemainPage=%d, LastStartPage=%d\n", __func__, FlashPageNum, RemainPage, LastStartPage);*/

#if 0
    //Step 2: Check Remark ID and Enter ISP Mode
	spi_speed = 1 * 1000 * 1000; //1MHz

	if (recovery_mode == 1) {
		if (check_remark_id)
			ret = elants_validate_remark_id(spi, p_fw_entry);
		if (ret) {
			ELAN_DEBUG("%s: elants_validate_remark_id fail, ret=%d\n", __func__, ret);
			goto END;
		}

		/*
		ret = elan_spi_transfer(spi, enter_recovery_isp_cmd, NULL, sizeof(enter_recovery_isp_cmd));
		if (ret != 0) {
			ELAN_DEBUG("%s: enter_recovery_isp_cmd fail\n", __func__);
			goto END;
		}
		//*/
		for (i = 0; i < 4; i++) {
			ret = elan_spi_transfer(spi, &enter_recovery_isp_cmd[i], NULL, 1);
			usleep_range(100, 200);
			if (ret != 0) {
				ELAN_DEBUG("%s: enter_recovery_isp_cmd[%d]=0x%02x fail\n", __func__, i, enter_recovery_isp_cmd[i]);
				goto END;
			}
		}
	} else {		
		elan_reset(private_ts);
		mdelay(30);
		if (check_remark_id)
			ret = elants_validate_remark_id(spi, p_fw_entry);
		if (ret) {
			ELAN_DEBUG("%s: elants_validate_remark_id fail, ret=%d\n", __func__, ret);
			goto END;
		}

		/*
		ret = elan_spi_transfer(spi, enter_isp_cmd, NULL, sizeof(enter_isp_cmd));
		if (ret != 0) {
			ELAN_DEBUG("%s: enter_isp_cmd fail\n", __func__);
			goto END;
		}
		//*/
		for (i = 0; i < 4; i++) {
			ret = elan_spi_transfer(spi, &enter_isp_cmd[i], NULL, 1);
			usleep_range(100, 200);
			if (ret != 0) {
				ELAN_DEBUG("%s: enter_isp_cmd[%d]=0x%02x fail\n", __func__, i, enter_isp_cmd[i]);
				goto END;
			}
		}
	}
#endif

    //Step 3: Write Dummy byte
    udelay(10);
    ret = elan_spi_transfer(spi, dummy_cmd, NULL, sizeof(dummy_cmd));
    if (ret != 0) {
        ELAN_DEBUG("%s: dummy_cmd fail\n", __func__);
        goto END;
    }

    //Step 4: Start IAP Process
   // kfree(rbuf);
    rbuf = kmalloc(sizeof(unsigned char) * iap_ack_len, GFP_KERNEL);
    if(rbuf == NULL) {
		 rbuf = kmalloc(sizeof(unsigned char) * iap_ack_len, GFP_KERNEL);
		 if(rbuf == NULL)
		 	goto END;
	}
	
    write_page_cmd = kmalloc(sizeof(unsigned char) * PageSize * FlashPageNum, GFP_KERNEL);
    if(write_page_cmd == NULL) {
		write_page_cmd = kmalloc(sizeof(unsigned char) * PageSize * FlashPageNum, GFP_KERNEL);
		 if(write_page_cmd == NULL)
		 	goto END;
	}
    //write_page_cmd = (unsigned char *)malloc(sizeof(unsigned char)*PageSize);
	//spi_speed = 10 * 1000 * 1000; //10MHz
Retry:
    //lseek(nFile, 0, SEEK_SET);
    curIndex = 0;
    for (i = 0; i < PageNum; i+=FlashPageNum) {
		if (i == LastStartPage) {
			szBuff = fw_data + curIndex;
			memcpy(write_page_cmd, szBuff, PageSize * RemainPage);
			curIndex =  curIndex + PageSize * RemainPage;
			ret = elan_spi_transfer(spi, write_page_cmd, NULL, PageSize * RemainPage);
		} else {
			szBuff = fw_data + curIndex;
			memcpy(write_page_cmd, szBuff, PageSize * FlashPageNum);
			curIndex =  curIndex + PageSize * FlashPageNum;
			ret = elan_spi_transfer(spi, write_page_cmd, NULL, PageSize * FlashPageNum);
		}
        if (ret != 0) {
            ELAN_DEBUG("%s: IAP write page fail\n", __func__);
            goto Retry;
        }

		elan_ts_poll(1, 100);

        ret = elan_spi_check_ack(spi, rbuf, iap_page_ack, iap_ack_len, sizeof(iap_page_ack));
        if (ret != 0) {
            rewriteCnt++;
            ELAN_DEBUG("%s: [Page-%dth] page_ack compare fail %d times, ret = %d (0x%02x 0x%02x)\n", __func__, i, rewriteCnt, ret, rbuf[0], rbuf[1]);
            if (rewriteCnt == PAGERETRY)
                goto END;               
            else
                goto Retry;             
        } /* else {
            ELAN_DEBUG("%s: [Page-%dth] page_ack compare success (0x%02x 0x%02x)\n", __func__, i, rbuf[0], rbuf[1]);
        }*/
    }

    //Step 5: Finish IAP or Error Handle
END:
    if (p_fw_entry != NULL && bBuiltInEkt == false) {
        ELAN_DEBUG("%s: p_fw_entry release\n", __func__);
        release_firmware(p_fw_entry);
    } else {
        ELAN_DEBUG("%s: p_fw_entry NULL\n", __func__);
    }

    //spi_speed = spi_speed_ori;
    //elan_irq_enable(private_ts);
    elan_reset(private_ts);
    if(NULL != rbuf)
   		kfree(rbuf);
    if(NULL != write_page_cmd)
		kfree(write_page_cmd);
    return ret;
}

#if 0
static int elan_ktf_ts_wakeup_spi(struct spi_device *spi)
{
	int ret;
	const uint8_t wakeup_spi[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00 };

    ELAN_DEBUG("%s: enter\n", __func__);

	ret = elan_spi_transfer(spi, wakeup_spi, NULL, sizeof(wakeup_spi));
	if(ret)
		ELAN_DEBUG("send wakeup_spi failed\n");
	else
		ELAN_DEBUG("[elan] wakeup_spi = [%2x, %2x, %2x, %2x]\n",
                 wakeup_spi[0], wakeup_spi[1], wakeup_spi[2], wakeup_spi[3]);

	mdelay(60);

    return ret;
}
#endif

#if 0 //eKTD6721 Not need
static int elan_ktf_ts_get_power_state(struct spi_device *spi)
{
	int ret;
	const uint8_t r_power_status[] = { 0x53, 0x50, 0x00, 0x01, 0x00, 0x00 };
	char *rbuf;
	uint8_t power_state;

	elan_irq_disable(private_ts);
				
	ret = elan_spi_transfer(spi, r_power_status, NULL, sizeof(r_power_status));
	if(ret)
		ELAN_DEBUG("send r_power_status failed\n");
	else
		ELAN_DEBUG("[elan] r_power_status cmd = [%2x, %2x, %2x, %2x]\n",
                 r_power_status[0], r_power_status[1], r_power_status[2], r_power_status[3]);

	elan_ts_poll(5, 50);

    rbuf = kmalloc(sizeof(unsigned char) * 4, GFP_KERNEL);
    ret = elan_spi_transfer(spi, NULL, rbuf, 4);
    if (ret != 0) {
        ELAN_DEBUG("unexpected response: %*ph\n", 4, rbuf);
    }
	ELAN_DEBUG("%s: response: %*ph\n", __func__, 4, rbuf);

    power_state = rbuf[1];
    //ELAN_DEBUG("response: 0x%0x\n", power_state);
    power_state = (power_state & PWR_STATE_MASK) >> 3;
    ELAN_DEBUG("power state = %s\n", 
            power_state == PWR_STATE_DEEP_SLEEP ? "Deep Sleep" : "Normal/Idle");
	
	elan_irq_enable(private_ts);

	return ret;
}

static int elan_ktf_ts_set_power_state(struct spi_device *spi, uint8_t state)
{
	int ret;
	uint8_t power_cmd[] = { 0x54, 0x50, 0x00, 0x01, 0x00, 0x00 };

    ELAN_DEBUG("%s: enter\n", __func__);

	power_cmd[1] |= (state << 3); //0x50: sleep, 0x58: normal

	ret = elan_spi_transfer(spi, power_cmd, NULL, sizeof(power_cmd));
	if(ret)
		ELAN_DEBUG("send power_cmd failed\n");
	else
		ELAN_DEBUG("[elan] power_cmd = [%2x, %2x, %2x, %2x]\n",
                 power_cmd[0], power_cmd[1], power_cmd[2], power_cmd[3]);

    return ret;
}
#endif //eKTD6721 Not need

static void elan_release_finger(struct elants_data *ts)
{
	int i = 0;
	struct input_dev *idev = ts->input_dev; 
	 
	 
	if(idev != NULL) { 
		for(i=0; i<FINGER_NUM; i++){
			input_mt_slot(idev, i);
			input_mt_report_slot_state(idev, MT_TOOL_FINGER, 0);
		}
	
		input_sync(idev);
	}
	return;
}

static int elan_ktf_ts_suspend(struct spi_device *spi)
{
	
	ELAN_DEBUG("%s: enter\n", __func__);
	elan_release_finger(private_ts);
	elan_irq_disable(private_ts);
	power_flag = 1;
	
#if 0 //eKTD6721 Not need
	elan_ktf_ts_get_power_state(spi);
	elan_ktf_ts_set_power_state(spi, PWR_STATE_DEEP_SLEEP);
	elan_ktf_ts_get_power_state(spi);
#endif //eKTD6721 Not need

    return 0;
}
static int elan_check_status(struct elants_data *ts);
static int elan_ktf_ts_resume(struct spi_device *spi)
{
	//int wakeup_idle_ori = wakeup_idle;
	int ret = 0;
	
    ELAN_DEBUG("%s: enter\n", __func__);
	elan_release_finger(private_ts);
	ret = elan_check_status(private_ts);
	if(ret == 0) { //for fw in sram
		enter_main_flag = 0;
		EnterMianCode(private_ts->spi);
	}
	elan_irq_enable(private_ts);
	power_flag = 0;
#if 0 //eKTD6721 Not need
	if (!wakeup_idle) //force wakeup_idle from 0 to 1 for deep sleep
		wakeup_idle = !wakeup_idle;
	//elan_ktf_ts_wakeup_spi(spi);
	elan_ktf_ts_get_power_state(spi);
	//elan_ktf_ts_wakeup_spi(spi);
	elan_ktf_ts_set_power_state(spi, PWR_STATE_NORMAL);
	//elan_ktf_ts_wakeup_spi(spi);
	elan_ktf_ts_get_power_state(spi);
	wakeup_idle = wakeup_idle_ori;
#endif //eKTD6721 Not need

//    force_release_pos(input_dev);
      
    return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = NULL;
	static int first_in = 1;
    ELAN_DEBUG("%s fb notifier callback event = %lu\n",__func__, event);
    //return 0;
    if (!evdata) {
		ELAN_DEBUG("evdata is null");
		return 0;
	    }
     if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        ELAN_DEBUG("event(%lu) do not need process\n", event);
        return 0;
    }
       if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		ELAN_DEBUG("FB event:%lu,blank:%d\n", event, *blank);
        if (event == FB_EVENT_BLANK) {
            if (*blank == FB_BLANK_UNBLANK) {
                //display_status = 0;
                //elan_work_flag = 1;
                //wake_up(&elan_poll_wq);
				if (first_in)
					first_in = 0;
				else
					elan_ktf_ts_resume(private_ts->spi);
                ELAN_DEBUG("Display On\n");
            }
            else if (*blank == FB_BLANK_POWERDOWN) {
                //display_status = 1;
                //elan_work_flag = 1;
                //wake_up(&elan_poll_wq);
				elan_ktf_ts_suspend(private_ts->spi);
                ELAN_DEBUG("Display Off\n");
            }
        }
    }
    return 0;
}
#endif


#if 0 //Finger Printer Sample Code, remember to delete

static long elan_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    //struct elants_data *ts = filp->private_data;
#if 0
    struct thp_ioctl_spi_sync_data sync_data;
    void __user *argp = (void __user *)arg;
    unsigned char *tx;
    unsigned char *rx;
    //struct timeval tv;
    //ELAN_DEBUG("%s()\n", __func__);

    //[TODO] add ioctl for user space or HAL?
    switch(cmd) {
        case THP_IOCTL_CMD_RESET:
            //ret = thp_ioctl_reset(arg);
            ELAN_DEBUG("THP_IOCTL_CMD_RESET : %ld\n", arg);
            gpio_set_value(private_ts->rst_gpio, arg);
            break;

        case THP_IOCTL_CMD_SET_TIMEOUT:
            //ret = thp_ioctl_set_timeout(arg);
            ELAN_DEBUG("THP_IOCTL_CMD_SET_TIMEOUT : %ld\n", arg);
            break;

        case THP_IOCTL_CMD_SPI_SYNC:
            ///ret = thp_ioctl_spi_sync((void __user *)arg);
            //ELAN_DEBUG("THP_IOCTL_CMD_SPI_SYNC\n");
            if (copy_from_user(&sync_data, argp,
                    sizeof(struct thp_ioctl_spi_sync_data)))
                ELAN_DEBUG("Failed to copy_from_user\n");
            ELAN_DEBUG("sync_data.size=%d\n", sync_data.size);
            tx = kzalloc(sync_data.size, GFP_KERNEL);
            rx = kzalloc(sync_data.size, GFP_KERNEL);
            if (!tx || !rx)
                ELAN_DEBUG("%s:buf request memory fail,sync_data.size = %d\n", __func__,sync_data.size);

            if (copy_from_user(tx, sync_data.tx, sync_data.size))
                ELAN_DEBUG("Failed to copy in buff\n");

            if (elan_spi_transfer(private_ts->spi, tx, rx, sync_data.size))
                ELAN_DEBUG("Failed to spi transfer\n");

            if (sync_data.rx) {
                //ELAN_DEBUG("sync_data.rx exist\n");
                if (copy_to_user(sync_data.rx, rx, sync_data.size))
                    ELAN_DEBUG("Failed to copy out buff\n");
            }

            break;

        case THP_IOCTL_CMD_FINISH_NOTIFY:
            //ret = thp_ioctl_finish_notify(arg);
            ELAN_DEBUG("THP_IOCTL_CMD_FINISH_NOTIFY\n");
            break;
        case THP_IOCTL_CMD_SET_BLOCK:
            //ret = thp_ioctl_set_block(arg);
            ELAN_DEBUG("THP_IOCTL_CMD_SET_BLOCK : %ld\n", arg);
            break;

        case THP_IOCTL_CMD_SET_IRQ:
            //ret = thp_ioctl_set_irq(arg);
            ELAN_DEBUG("THP_IOCTL_CMD_SET_IRQ : %ld\n", arg);
            if (arg != 0)
                elan_irq_enable(private_ts);
            else
                elan_irq_disable(private_ts);
            break;

        default:
            ELAN_DEBUG("INVALID COMMAND\n");
            break;
    }
#endif
    return 0;
}

static unsigned int elan_poll(struct file *file, poll_table *wait)
{
    int mask=0;
    //ELAN_DEBUG("%s()\n",__func__);

    //wait_event_interruptible(elan_poll_wq, elan_work_flag > 0);
    poll_wait(file, &elan_poll_wq, wait);
    if (elan_work_flag > 0)
        mask = elan_work_flag;

    elan_work_flag = 0;

    return mask;
}

static int elan_fasync(int fd, struct file * filp, int on)
{
    ELAN_DEBUG("%s()\n",__func__);
    return fasync_helper(fd, filp, on, &fasync_queue);
}

static ssize_t elan_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
    ssize_t ret = 0;

    if (user_buf[0] == 10) {
        IOIRQ_STATUS = user_buf[1];
        elan_work_flag = 0;
        ELAN_DEBUG("set IOIRQ_STATUS = 0x%x, elan_work_flag = %d\n", IOIRQ_STATUS, elan_work_flag);
    }
    else if (user_buf[0] == 0X10) {
        read_all = user_buf[1];
        ELAN_DEBUG("read_all = %d\n", read_all);
    }

    return ret;
}

ssize_t elan_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
    ssize_t ret = 0;
    return ret;
}

static int elan_open(struct inode *inode, struct file *filp)
{
    struct elants_data *ts = container_of(filp->private_data, struct elants_data, elan_dev);
    filp->private_data = ts;
    //ELAN_DEBUG("%s()\n", __func__);
    return 0;
}

static int elan_close(struct inode *inode, struct file *filp)
{
    //ELAN_DEBUG("%s()\n", __func__);
    return 0;
}

//[TODO] check Herman for all operations
static const struct file_operations elan_fops = {
    .owner          = THIS_MODULE,
    .open           = elan_open,
    .read           = elan_read,
    .write          = elan_write,
    .unlocked_ioctl = elan_ioctl,
    .poll           = elan_poll,
    .release        = elan_close,
    .fasync         = elan_fasync,
};

#endif //Finger Printer Sample Code, remember to delete


// For Firmware Update
int elan_iap_open(struct inode *inode, struct file *filp) {
    ELAN_DEBUG("[elan]into elan_iap_open\n");
    if (private_ts == NULL)
		ELAN_DEBUG("private_ts is NULL\n");

    return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp) {
    return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp) {
	struct elants_data *ts = container_of(filp->private_data, struct elants_data, elan_dev);
    int ret;
    char *tmp;

    ELAN_DEBUG("[elan]%s: count=%d\n", __func__, (int)count);

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }

	ret = elan_spi_transfer(ts->spi, tmp, NULL, count);

	if(tmp != NULL)
		kfree(tmp);

    return (ret == 0) ? count : ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp) {
	struct elants_data *ts = container_of(filp->private_data, struct elants_data, elan_dev);
    char *tmp;
    int ret;
    long rc;

    ELAN_DEBUG("[elan]%s: count=%d\n", __func__, (int)count);

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

	ret = elan_spi_transfer(ts->spi, NULL, tmp, count);
    if (ret == 0)
        rc = copy_to_user(buff, tmp, count);

   if(tmp != NULL)
		kfree(tmp);

    return (ret == 0) ? count : ret;
}

static long elan_iap_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {

	struct elants_data *ts = container_of(filp->private_data, struct elants_data, elan_dev);
    int __user *ip = (int __user *)arg;

    ELAN_DEBUG("[elan]%s: cmd value %x\n", __func__, cmd);

    switch (cmd) {
    case IOCTL_I2C_INT:
        put_user(gpio_get_value(ts->int_gpio), ip);
        break;
    case IOCTL_RESET:
        // modify
        elan_reset(ts);
        break;
    case IOCTL_IAP_MODE_LOCK:
            elan_irq_disable(ts);
        break;
    case IOCTL_IAP_MODE_UNLOCK:
            elan_irq_enable(ts);
        break;
    case IOCTL_ROUGH_CALIBRATE:
        return elan_calibrate(ts->spi);
    case IOCTL_FW_UPDATE:
        elan_spi_iap(ts->spi);
        break;
    case IOCTL_CHECK_RECOVERY_MODE:
        return recovery_mode;
        break;
    case IOCTL_FW_INFO:
		elan_fw_packet_handler(ts);
        break;
    case IOCTL_FW_ID:
        return ts->fw_info.fw_id;
        break;
    case IOCTL_FW_VER:
        return ts->fw_info.fw_ver;
        break;
	case IOCTL_BC_VER:
        return ts->fw_info.bc_ver;
		break;
    case IOCTL_X_RESOLUTION:
        return ts->fw_info.x_res;
        break;
    case IOCTL_Y_RESOLUTION:
        return ts->fw_info.y_res;
        break;
    default:
        ELAN_DEBUG("[elan] Un-known IOCTL Command %d\n", cmd);
        break;
    }
    return 0;
}

struct file_operations elan_touch_fops = {
    .open =         elan_iap_open,
    .write =        elan_iap_write,
    .read = 	elan_iap_read,
    .release =	elan_iap_release,
    .unlocked_ioctl=elan_iap_ioctl,
    .compat_ioctl=elan_iap_ioctl,
};

static int elan_setup_cdev(struct elants_data *ts)
{
    //[TODO] check MISC_DYNAMIC_MINOR
    ts->elan_dev.minor = MISC_DYNAMIC_MINOR;
    ts->elan_dev.name = "elan-iap";
    ts->elan_dev.fops = &elan_touch_fops;
    //[TODO] check S_IFREG|S_IRWXUGO
    ts->elan_dev.mode = S_IFREG|S_IRWXUGO;
    if (misc_register(&ts->elan_dev) < 0) {
        ELAN_DEBUG("misc_register failed\n");
        return -1;
    }
    else {
        ELAN_DEBUG("misc_register success\n");
    }

    return 0;
}

/*
int EnterMainMode(struct spi_device *spi)
{
    int len = 0;
    int i = 0;
    uint8_t main_cmd[4] = {0x4D, 0x61, 0x69, 0x6E}; //{0x45, 0x49, 0x41, 0x50};

    for (i = 0; i < sizeof(main_cmd); i++)
    {
        len = elan_spi_transfer(spi, &main_cmd[i], NULL, 1);
    if (len != 0) {
        ELAN_DEBUG("[elan] ERROR: EnterMainMode fail! len=%d\n", len);
        return -1;
    } else {
        ELAN_DEBUG("[elan] MainMode write data successfully! cmd = [%2x]\n", main_cmd[i]);
            usleep_range(100, 200);
    }
    } 
    return 0;
}
//*/

int EnterISPMode(struct spi_device *spi)
{
    int len = 0;
    int i = 0;
    uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50};

    for (i = 0; i < sizeof(isp_cmd); i++)
    {
        len = elan_spi_transfer(spi, &isp_cmd[i], NULL, 1);
    if (len != 0) {
        ELAN_DEBUG("[elan] ERROR: EnterISPMode fail! len=%d\n", len);
        return -1;
    } else {
        ELAN_DEBUG("[elan] ISPMode write data successfully! cmd = [%2x]\n", isp_cmd[i]);
            usleep_range(100, 200);
    }
    } 
    return 0;
}


int EnterMianCode(struct spi_device *spi)
{
    int len = 0;
    uint8_t enter_main[] = {0x4D, 0x61, 0x69, 0x6e};
	int i = 0;

	
    for (i = 0; i < sizeof(enter_main); i++)
    {
        len = elan_spi_transfer(spi, &enter_main[i], NULL, 1);
		if (len != 0) {
			ELAN_DEBUG("[elan] ERROR: enter_main fail! len=%d\n", len);
			return -1;
		} else {
			ELAN_DEBUG("[elan] enter_main write data successfully! cmd = [%2x]\n", enter_main[i]);
			usleep_range(100, 200);
		}
    } 
 
    return 0;
}

static inline int elan_ktf_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
    *x = *y = 0;

    *x = (data[0] & 0xf0);
    *x <<= 4;
    *x |= data[1];

    *y = (data[0] & 0x0f);
    *y <<= 8;
    *y |= data[2];

    return 0;
}

static inline int elan_ktf_pen_parse_xy(uint8_t *data,
                                        uint16_t *x, uint16_t *y, uint16_t *p)
{
    *x = *y = *p = 0;

    *x = data[3];
    *x <<= 8;
    *x |= data[2];

    *y = data[5];
    *y <<= 8;
    *y |= data[4];

    *p = data[7];
    *p <<= 8;
    *p |= data[6];

    return 0;
}

#ifdef PROTOCOL_B
/* Protocol B  */
static int mTouchStatus[FINGER_NUM] = {0};  /* finger_num=10 */
void force_release_pos(struct input_dev *idev)
{
    //struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
    int i;
    for (i=0; i < FINGER_NUM; i++) {
        if (mTouchStatus[i] == 0)
            continue;
        input_mt_slot(idev, i);
        input_mt_report_slot_state(idev, MT_TOOL_FINGER, 0);
        mTouchStatus[i] = 0;
    }

    input_sync(idev);
}

static void elan_ktf_ts_report_data(struct elants_data *ts, uint8_t *buf)
{
    struct input_dev *idev = ts->input_dev;
    uint16_t x =0, y =0,touch_size/*, pressure_size*/;
    uint16_t fbits=0;
    uint8_t i, num;
    uint16_t active = 0;
    uint8_t idx, btn_idx;
    int finger_num;
    struct input_dev *pen_idev = ts->pen_input_dev;
    int pen_hover = 0;
    int pen_down = 0;
    uint16_t p = 0;
	//int pen_res_x = (ts->fw_info.rx - 1) * ELAN_PEN_OSR;
	//int pen_res_y = (ts->fw_info.tx - 1) * ELAN_PEN_OSR;
    uint16_t fw_status = 0;
    uint16_t fw_checksum = 0;
	int iap_times = 0;
	
	//bool bBuiltInEkt = false;

    if (buf[0] == TEN_FINGERS_PKT) {
        /* for 10 fingers */
        finger_num = 10;
        num = buf[2] & 0x0f;
        fbits = buf[2] & 0x30;
        fbits = (fbits << 4) | buf[1];
        idx=3;
        btn_idx=33;
	}

    switch (buf[0]) {
	case TEN_FINGERS_PKT:
        for (i = 0; i < finger_num; i++) {
            active = fbits & 0x1;
            if(active || mTouchStatus[i]) {
                //input_mt_slot(idev, i);
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(idev, MT_TOOL_FINGER, active);
                if(active) {
                    elan_ktf_ts_parse_xy(&buf[idx], &x, &y);
                    if (i % 2 == 0)
                        touch_size = (buf[FW_POS_WIDTH + (i / 2)] & 0xf0) >> 4;
                    else
                        touch_size = buf[FW_POS_WIDTH + (i / 2)] & 0x0f;
                    //pressure_size = buf[FW_POS_CHECKSUM];
                    x = x * 720 / 1224;
					y = y * 1600/ 2520;
			
                    input_report_abs(idev, ABS_MT_TOUCH_MAJOR, touch_size);
                    //input_report_abs(idev, ABS_MT_PRESSURE, pressure_size);
                    input_report_abs(idev, ABS_MT_POSITION_X, x);
                    input_report_abs(idev, ABS_MT_POSITION_Y, y);
					ELAN_DEBUG("finger id=%d x=%d y=%d size=%d\n", i, x, y, touch_size);
                    if(unlikely(gPrint_point))
                        //ELAN_DEBUG("finger id=%d x=%d y=%d size=%d pressure=%d\n", i, x, y, touch_size, pressure_size);
                        ELAN_DEBUG("finger id=%d x=%d y=%d size=%d\n", i, x, y, touch_size);
                }
            }
            mTouchStatus[i] = active;
            fbits = fbits >> 1;
            idx += 3;
        }
        if (num == 0) {
            ELAN_DEBUG("ALL Finger Up\n");
            input_report_key(idev, BTN_TOUCH, 0); //for all finger up
            force_release_pos(idev);
        }
        input_sync(idev);
        break;
    case PEN_PKT:
        pen_hover = buf[1] & 0x1;
        pen_down = buf[1] & 0x03;
        input_mt_slot(pen_idev, 0);
        input_mt_report_slot_state(pen_idev, MT_TOOL_PEN, pen_hover);
        if (pen_hover) {
            elan_ktf_pen_parse_xy(&buf[0], &x, &y, &p);
            //x = x * ts->fw_info.x_res / pen_res_x;
            //y = y * ts->fw_info.y_res / pen_res_y;
            if (pen_down == 0x01) {  /* report hover function  */
                input_report_abs(pen_idev, ABS_MT_PRESSURE, 0);
                input_report_abs(pen_idev, ABS_MT_DISTANCE, 15);
                //ELAN_DEBUG("[pen] Hover DISTANCE=15 \n");
            } else {
                input_report_abs(pen_idev, ABS_MT_TOUCH_MAJOR, 20);
                input_report_abs(pen_idev, ABS_MT_PRESSURE, p);
                //ELAN_DEBUG("[pen] PRESSURE=%d \n", p);
            }
            input_report_abs(pen_idev, ABS_MT_POSITION_X, x);
            input_report_abs(pen_idev, ABS_MT_POSITION_Y, y);
        }
        if(unlikely(gPrint_point)) {
            ELAN_DEBUG("[pen] %x %x %x %x %x %x %x %x \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
            ELAN_DEBUG("[pen] x=%d y=%d p=%d \n", x, y, p);
        }
        if (pen_down == 0) {
            ELAN_DEBUG("Pen Up\n");
            input_mt_slot(pen_idev, 0);
            input_mt_report_slot_state(pen_idev, MT_TOOL_PEN, pen_hover);
        }
        input_sync(pen_idev);
        break;
    case MC_HELLO_PKT:
        ELAN_DEBUG("%s: Report Main Code Hello packet: %x %x %x %x\n",__func__, buf[0],buf[1],buf[2],buf[3]);
#if 0 //eKTD6721 Not need
		if (wakeup_idle) //force wakeup_idle from 1 to 0 after fw reboot
			wakeup_idle = !wakeup_idle;

		elan_ktf_ts_set_auto_idle(ts->spi);
#endif //eKTD6721 Not need

        if (read_fw_flag) {
            msleep(1000);
            elan_fw_packet_handler(private_ts);
			//read_fw_flag = 0;
        }

        //solution request to send finger up event
        //force_release_pos(client);
        break;
    case BC_HELLO_PKT:
        ELAN_DEBUG("%s: Report Boot Code Hello packet: %x %x %x %x\n",__func__, buf[0],buf[1],buf[2],buf[3]);
        fw_status = buf[5]<<8 | buf[4];
        fw_checksum = buf[7]<<8 | buf[6];
        ELAN_DEBUG("%s: Report MainCode Status packet: 0x%x\n",__func__, fw_status);
        ELAN_DEBUG("%s: Report MainCode CheckSum packet: 0x%x\n",__func__, fw_checksum);

		enter_main_flag = 1;
        gFlashMode = buf[8];
        ELAN_DEBUG("%s: Report external flash packet: %d, first boot\n",__func__, gFlashMode,bFirstBoot);
        // [TODO]: If enable LK Non-Flash IAP, Kernel could Compare Checksum directly
        if (bFirstBoot == true) { //Need IAP when TDDI Non-Flash Mode and First Boot
            if (gFlashMode == 0)
				bEnableIAP = true;
			bBuiltInEkt = true;
		} else {
			bBuiltInEkt = false;
		}

        ELAN_DEBUG("%s: Enable IAP = %s, \n",__func__, (bEnableIAP ? "true" : "false"));
        if (bEnableIAP == false) {
			if (fw_status == 0x5053 || fw_status == 0x5073) {
                ELAN_DEBUG("%s: Check FW Main Status Pass\n",__func__);
                //EnterMainMode(private_ts->client);
                bFirstBoot = false;
            } else {
                ELAN_DEBUG("%s: Check FW Main Status Fail\n",__func__);
                if (gFlashMode != 0 && gnFlashModeRetry > 0) {
                    gnFlashModeRetry--;
                    elan_reset(ts);
                } else {
                    //bEnableIAP = true;                  
                }
            }		
        }

#if 1
        if(bEnableIAP == false)
            break;

        bEnableIAP = false;
		gnFlashModeRetry = FLASH_MODE_RETRY;
        iap_times++;
        ELAN_DEBUG( "%s: EnterISPMode times = %d\n",__func__, iap_times);
        EnterISPMode(ts->spi);    //enter ISP mode

        //power_lock = 1;
#if defined( ESD_CHECK )
        work_lock=1;
        cancel_delayed_work_sync( &esd_work );
#endif
        if (bBuiltInEkt == false) {
            ELAN_DEBUG( "%s: Start IAP at /system/etc/firmware/elan_fw.ekt\n",__func__);
            //Update_FW_One(1);
            elan_spi_iap(ts->spi);
			read_fw_flag = 1;
        } else {
            ELAN_DEBUG( "%s: Start IAP at build in ekt file\n",__func__);
            //Update_FW_One(0);
            //elan_spi_iap(fp->spi);
        }
#if defined( ESD_CHECK )
        work_lock=0;
        queue_delayed_work(esd_wq, &esd_work, delay);   
#endif
        //power_lock = 0;

#endif

        break;
    case CALIB_PKT:
        ELAN_DEBUG("%s: Report Calibration packet: %x %x %x %x\n",__func__, buf[0],buf[1],buf[2],buf[3]);
        break;
	case 0xFF:
	case 0x00:
		if(enter_main_flag == 1) {
			enter_main_flag = 0;
			EnterMianCode(ts->spi);
		}
		break;
    default:
        ELAN_DEBUG("%s: unknown packet type: %x %x %x %x\n",__func__, buf[0],buf[1],buf[2],buf[3]);
        break;
	} // end switch
}
#endif
//end #ifdef PROTOCOL_B

//add by Minger
#if 1
static int touch_event_handler(void *unused)
{
   struct elants_data *ts = private_ts;
   int ret = 0;
   int i = 0;
   int print_len = 4;
   unsigned char rxbuf[PACKET_SIZE] = {0};
  
	

    struct sched_param param = { .sched_priority = 6 };
    sched_setscheduler(current, SCHED_RR, &param);
    
    do{
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
		
        set_current_state(TASK_RUNNING);
       
	   if(gpio_get_value(ts->int_gpio) == 0) {
			spi_read_len = (bEnableIAP == true) ? BC_HELLO_LEN : PACKET_SIZE;
			print_len = (spi_read_len == PACKET_SIZE) ? 6 : spi_read_len;
			ret = elan_spi_transfer(ts->spi, NULL, rxbuf, spi_read_len);
			if (ret != 0)
				ELAN_DEBUG("spi_sync failed, ret = %d\n", ret);
			else {
				printk("[elan-spi][INT-R%d] rxbuf(%d) :", spi_read_len, print_len);
				for(i = 0; i < print_len; i++)
					printk(" %02x", rxbuf[i]);
			}
			printk("\n");

			elan_ktf_ts_report_data(ts, rxbuf);
	   }
	}while(!kthread_should_stop());
    
    return 0;
}
//end
#else

static void elan_work_func(struct work_struct *work)
{

    struct elants_data *ts;
    int ret = 0;
    int i = 0;
    int print_len = 4;
    unsigned char rxbuf[PACKET_SIZE] = {0};

    //ELAN_DEBUG("%s: +\n", __func__);

    ts = container_of(work, struct elants_data, work);
    //spi_read_len = 50;
    spi_read_len = (bEnableIAP == true) ? BC_HELLO_LEN : PACKET_SIZE;

    if (gpio_get_value(ts->int_gpio)) {
        ELAN_DEBUG("Detected the jitter on INT pin\n");
        return;
    }

    print_len = (spi_read_len == PACKET_SIZE) ? 6 : spi_read_len;
    ret = elan_spi_transfer(ts->spi, NULL, rxbuf, spi_read_len);
    if (ret != 0)
        ELAN_DEBUG("spi_sync failed, ret = %d\n", ret);
    else {
        printk("[elan-spi][INT-R%d] rxbuf(%d) :", spi_read_len, print_len);
        for(i = 0; i < print_len; i++)
            printk(" %02x", rxbuf[i]);
    }
    printk("\n");

    elan_ktf_ts_report_data(ts, rxbuf);
	
}

#endif

static irqreturn_t elan_irq_handler(int irq, void *dev_id)
{
//add by Minger
#if 1
    ELAN_DEBUG("%s()\n", __func__);
	tpd_flag = 1;
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
//endif
#else
    struct elants_data *ts = (struct elants_data *)dev_id;

    ELAN_DEBUG("%s()\n", __func__);

   // if (ts == NULL)
   //     return IRQ_NONE;
    //[TODO] check usage
    //wake_lock_timeout(&ts->wake_lock, msecs_to_jiffies(1000));
    //__pm_wakeup_event(&ts->wake_lock, msecs_to_jiffies(1000));

    queue_work(elan_wq, &ts->work);
    return IRQ_HANDLED;
#endif

    
}

static int elan_finger_input_create(struct elants_data *ts)
{
    struct input_dev *input_dev = NULL;
	int ret;
    //[TODO] modify input parameter for ts
    /* Init Input Device */
    input_dev = input_allocate_device();
    if (input_dev == NULL) {
		ret = -ENOMEM;
        ELAN_DEBUG("alloc input_dev failed\n");
		goto err_input_dev_alloc_failed;
	}

    input_dev->name = "elan-touchscreen";
    input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = &ts->spi->dev;
    input_set_drvdata(input_dev, ts);

    input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
    //input_set_capability(input_dev, EV_KEY, KEY_FP_INT); // change by customer, send key event to framework. KEY_xxx could be changed.

    ts->input_dev = input_dev;

#ifdef PROTOCOL_B
    input_mt_init_slots(ts->input_dev, FINGER_NUM, 0);
#endif
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1600/*2520;ts->fw_info.y_res*/, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 720/*1224;ts->fw_info.x_res*/, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_SIZE, 0, 0);
    //input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, MAX_FINGER_PRESSURE, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_DISTANCE, 0, MAX_FINGER_SIZE, 0, 0);

    __set_bit(EV_ABS, ts->input_dev->evbit);
    __set_bit(EV_SYN, ts->input_dev->evbit);
    __set_bit(EV_KEY, ts->input_dev->evbit);
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
    //input_set_abs_params(ts->input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);

    /* Register Input Device */
    ret = input_register_device(ts->input_dev);
    if(ret)
        ELAN_DEBUG("register input device failed, ret = %d\n", ret);
    if (ret) {
        ELAN_DEBUG("%s: unable to register %s input device\n",
                __func__, ts->input_dev->name);
        goto err_input_register_device_failed;
    }

    return ret;

err_input_register_device_failed:
    if (ts->input_dev)
        input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
    return ret;

}

static int elan_pen_input_create(struct elants_data *ts)
{
    struct input_dev *input_dev = NULL;
	int ret;
	int pen_res_x = (ts->fw_info.rx - 1) * ELAN_PEN_OSR;
	int pen_res_y = (ts->fw_info.tx - 1) * ELAN_PEN_OSR;
    //[TODO] modify input parameter for ts
    /* Init Input Device */
    input_dev = input_allocate_device();
    if (input_dev == NULL) {
		ret = -ENOMEM;
        ELAN_DEBUG("alloc input_dev failed\n");
		goto err_input_dev_alloc_failed;
	}

    input_dev->name = "elan-touchscreen-pen";
    input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = &ts->spi->dev;
    input_set_drvdata(input_dev, ts);

    input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
    //input_set_capability(input_dev, EV_KEY, KEY_FP_INT); // change by customer, send key event to framework. KEY_xxx could be changed.

    ts->pen_input_dev = input_dev;

#ifdef PROTOCOL_B
    input_mt_init_slots(ts->pen_input_dev, FINGER_NUM, 0);
#endif
    input_set_abs_params(ts->pen_input_dev, ABS_MT_POSITION_Y, 0, pen_res_y, 0, 0);
    input_set_abs_params(ts->pen_input_dev, ABS_MT_POSITION_X, 0, pen_res_x, 0, 0);
    input_set_abs_params(ts->pen_input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_SIZE, 0, 0);
    input_set_abs_params(ts->pen_input_dev, ABS_MT_PRESSURE, 0, MAX_FINGER_PRESSURE, 0, 0);
    input_set_abs_params(ts->pen_input_dev, ABS_MT_DISTANCE, 0, MAX_FINGER_SIZE, 0, 0);

    __set_bit(EV_ABS, ts->pen_input_dev->evbit);
    __set_bit(EV_SYN, ts->pen_input_dev->evbit);
    __set_bit(EV_KEY, ts->pen_input_dev->evbit);
    __set_bit(INPUT_PROP_DIRECT, ts->pen_input_dev->propbit);
    input_set_capability(ts->pen_input_dev, EV_KEY, KEY_POWER);
    input_set_abs_params(ts->pen_input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);

    /* Register Input Device */
    ret = input_register_device(ts->pen_input_dev);
    if(ret)
        ELAN_DEBUG("register input device failed, ret = %d\n", ret);
    if (ret) {
        ELAN_DEBUG("%s: unable to register %s input device\n",
                __func__, ts->pen_input_dev->name);
        goto err_input_register_device_failed;
    }

    return ret;

err_input_register_device_failed:
    if (ts->pen_input_dev)
        input_free_device(ts->pen_input_dev);

err_input_dev_alloc_failed:
    return ret;

}

static void elan_ktf_ts_set_auto_idle(struct spi_device *spi)
{
	int ret;
	const uint8_t r_idle_status[] = { 0x53, 0xB1, 0x00, 0x01, 0x00, 0x00 };
	uint8_t w_idle_cmd[] = { 0x54, 0xB1, 0x00, 0x00, 0x00, 0x00 };
	char rbuf[4] = {0};
	uint8_t idle_state;

	elan_irq_disable(private_ts);

	ret = elan_spi_write_and_read(spi, r_idle_status, sizeof(r_idle_status), rbuf, sizeof(rbuf));
	ELAN_DEBUG("%s: response: %*ph\n", __func__, (int)sizeof(rbuf), rbuf);

    idle_state = (rbuf[3] & IDLE_MODE_MASK) >> 7; //;Bit[7]: IdleMode_B      (0: Off, 1: On)
    ELAN_DEBUG("idle mode = %s\n", 
            idle_state == IDLE_MODE_DISABLE ? "Idle Mode Disable" : "Idle Mode Enable");

	ELAN_DEBUG("%s wakeup_idle = %d\n", __func__, wakeup_idle);
	if (enable_idle_mode) { //Need Enable Auto Idle Mode
		if (!idle_state) { //Current Disable Idle, need enable
			w_idle_cmd[2] = rbuf[2];
			w_idle_cmd[3] = rbuf[3];
			w_idle_cmd[3] |= (!idle_state << 7); //;Bit[7]: IdleMode_B      (0: Off, 1: On)

			ret = elan_spi_transfer(spi, w_idle_cmd, NULL, sizeof(w_idle_cmd));
			if(ret) {
				ELAN_DEBUG("%s: w_idle_cmd failed: %*ph\n", __func__, (int)sizeof(w_idle_cmd), w_idle_cmd);
			} else {
				wakeup_idle = !idle_state;
				ELAN_DEBUG("%s: w_idle_cmd success: %*ph\n", __func__, (int)sizeof(w_idle_cmd), w_idle_cmd);
			}			
		} else { //Already Enable Idle
				wakeup_idle = idle_state;
				ELAN_DEBUG("%s: already in enable idle mode\n", __func__);
		}
	} else { //Need Disable Auto Idle Mode
		if (!idle_state) { //Already Disable Idle
				wakeup_idle = idle_state;
				ELAN_DEBUG("%s: already in disable idle mode\n", __func__);
		} else { //Current Enable Idle, need disable
			w_idle_cmd[2] = rbuf[2];
			w_idle_cmd[3] = rbuf[3];
			w_idle_cmd[3] &= (!idle_state << 7); //;Bit[7]: IdleMode_B      (0: Off, 1: On)

			ret = elan_spi_transfer(spi, w_idle_cmd, NULL, sizeof(w_idle_cmd));
			if(ret) {
				ELAN_DEBUG("%s: w_idle_cmd failed: %*ph\n", __func__, (int)sizeof(w_idle_cmd), w_idle_cmd);
			} else {
				wakeup_idle = !idle_state;
				ELAN_DEBUG("%s: w_idle_cmd success: %*ph\n", __func__, (int)sizeof(w_idle_cmd), w_idle_cmd);
			}			
		}
	}
	ELAN_DEBUG("%s wakeup_idle new = %d\n", __func__, wakeup_idle);

	elan_irq_enable(private_ts);
}

static int elan_check_status(struct elants_data *ts)
{
	int ret = 0;
	unsigned char rxbuf[BC_HELLO_LEN] = {0};
    uint16_t fw_status = 0;
    uint16_t fw_checksum = 0;
	int flash_mode = -1;
	int need_update = 0;
	int retry = 3;

	do {
		elan_reset(ts);

		ret = elan_ts_poll(5, 50);
		if (ret) {
			ELAN_DEBUG("%s int poll hight\n",__func__);
			return ret;
		}

		ret = elan_spi_transfer(ts->spi, NULL, rxbuf, BC_HELLO_LEN);
		if (ret) {
			ELAN_DEBUG("%s error\n", __func__);
			return ret;
		}

		//ELAN_DEBUG("%s hello: 0x%2x:0x%2x:0x%2x:0x%2x\n", __func__,rxbuf[0],rxbuf[1],rxbuf[2],rxbuf[3]);
		ELAN_DEBUG("%s Boot Code Hello: %*ph\n", __func__, BC_HELLO_LEN, rxbuf);

		fw_status = rxbuf[5]<<8 | rxbuf[4];
		fw_checksum = rxbuf[7]<<8 | rxbuf[6];
		ELAN_DEBUG("%s: MainCode Status packet: 0x%x\n",__func__, fw_status);
		ELAN_DEBUG("%s: MainCode CheckSum packet: 0x%x\n",__func__, fw_checksum);
		
		flash_mode = rxbuf[8];
		ELAN_DEBUG("%s: external flash packet: %d\n",__func__, flash_mode);

		if (fw_status == 0x5053 || fw_status == 0x5073) {
			need_update = 0;
			break;
		} else {
			need_update = 1;
		}
		
		if (need_update == 1 && flash_mode == 0) {
			ELAN_DEBUG("%s: Start Update Built In Firmware for Non-Flash\n",__func__);
			bBuiltInEkt = true;
			EnterISPMode(ts->spi);    //enter ISP mode
			elan_spi_iap(ts->spi);
			if(power_flag != 1)
				read_fw_flag = 1;
			bBuiltInEkt = false;
		}
		
		retry--;
	} while (need_update == 1 && retry > 0);

    return ret = (need_update ? -need_update : need_update);
}
#ifdef CONFIG_DEV_INFO
extern int store_tp_info(const char *const str);
#endif

static int elan_fw_packet_handler(struct elants_data *ts)
{
	int ret = 0;
	const uint8_t cmd_ver[6]	= {0x53, 0x00, 0x00, 0x01, 0x00, 0x00};
	const uint8_t cmd_id[6]		= {0x53, 0xf0, 0x00, 0x01, 0x00, 0x00}; 
	const uint8_t cmd_bc[6]		= {0x53, 0x10, 0x00, 0x01, 0x00, 0x00};
	const uint8_t cmd_res[6]	= {0x5B, 0x00, 0x00, 0x00, 0x00, 0x00};
	const uint8_t cmd_osr[6]	= {0x53, 0xD6, 0x00, 0x01, 0x00, 0x00};
	const uint8_t cmd_test_ver[6] = {0x53, 0xe0, 0x00, 0x01, 0x00, 0x00};
	uint8_t rbuf[17] = {0x00};
	char buf[80];
	int major, minor;
	struct elan_spi_fw_info *fw_info = &ts->fw_info;
	int i = 2;
	int len_ack_53 = 4;
	int len_ack_5b = 17;

	/*get fw id*/
	ret = elan_spi_write_and_read(ts->spi, cmd_id, sizeof(cmd_id), rbuf, len_ack_53);
	if (ret) {
		dev_info(&ts->spi->dev, "get fw id faile\n");
		goto out;//return ret;
	}
	major = ((rbuf[1] & 0x0f) << 4) | ((rbuf[2] & 0xf0) >> 4);
	minor = ((rbuf[2] & 0x0f) << 4) | ((rbuf[3] & 0xf0) >> 4);
	fw_info->fw_id = major << 8 | minor;

	/*get bootcode version*/
	ret = elan_spi_write_and_read(ts->spi, cmd_bc, sizeof(cmd_bc), rbuf, len_ack_53);
	if (ret) {
		dev_info(&ts->spi->dev, "get bootcode version faile\n");
		goto out;//return ret;
	}
	major = ((rbuf[1] & 0x0f) << 4) | ((rbuf[2] & 0xf0) >> 4);
	minor = ((rbuf[2] & 0x0f) << 4) | ((rbuf[3] & 0xf0) >> 4);
	fw_info->bc_ver = major << 8 | minor;

	/*get fw version*/
	ret = elan_spi_write_and_read(ts->spi, cmd_ver, sizeof(cmd_ver), rbuf, len_ack_53);
	if (ret) {
		dev_info(&ts->spi->dev, "get fw version faile\n");
		goto out;//return ret;
	}
	major = ((rbuf[1] & 0x0f) << 4) | ((rbuf[2] & 0xf0) >> 4);
	minor = ((rbuf[2] & 0x0f) << 4) | ((rbuf[3] & 0xf0) >> 4);

	fw_info->fw_ver =  major << 8 | minor;

	/*get tx/rx trace*/
	ret = elan_spi_write_and_read(ts->spi, cmd_res, sizeof(cmd_res), rbuf, len_ack_5b);
	if (ret) {
		dev_info(&ts->spi->dev, "get tp tx/rx trace faile\n");
		goto out;//return ret;
	}
	fw_info->rx = rbuf[2]; 
	fw_info->tx = rbuf[3];

	/*get tp finger osr*/
retry:
	ret = elan_spi_write_and_read(ts->spi, cmd_osr, sizeof(cmd_osr), rbuf, len_ack_53);
	if (ret) {
		dev_info(&ts->spi->dev, "get tp finger osr faile\n");
		goto out;//return ret;
	}
	dev_info(&ts->spi->dev, "%x:%x:%x:%x\n",rbuf[0],rbuf[1],rbuf[2],rbuf[3]);
	fw_info->finger_osr = rbuf[3];
	if((rbuf[0] != 0x52) && ((i--) > 0)) {
		goto retry;
	}



	/*get finger x/y coordinate resolution*/
	fw_info->x_res = (fw_info->rx - 1) * fw_info->finger_osr;
	fw_info->y_res = (fw_info->tx - 1) * fw_info->finger_osr;


	/*get test version*/
	ret = elan_spi_write_and_read(ts->spi, cmd_test_ver, sizeof(cmd_test_ver), rbuf, len_ack_53);
	if (ret) {
		dev_info(&ts->spi->dev, "get tp test version faile\n");
		goto out;//return ret;
	}
	major = ((rbuf[1] & 0x0f) << 4) | ((rbuf[2] & 0xf0) >> 4);
	minor = ((rbuf[2] & 0x0f) << 4) | ((rbuf[3] & 0xf0) >> 4);
	fw_info->testsolversion = major << 8 | minor;
	fw_info->testversion = major;
	fw_info->solutionversion = minor;

       sprintf(buf, "hlt-k580-ektd6721-v0x%02x",fw_info->fw_ver);
	
	store_tp_info(buf);

	dev_info(&ts->spi->dev,
			"[elan] %s fw version:0x%4.4x\n",
			__func__,fw_info->fw_ver);
	dev_info(&ts->spi->dev,
			"[elan] %s fw id:0x%4.4x\n",
			__func__,fw_info->fw_id);
	dev_info(&ts->spi->dev,
			"[elan] %s bootcode version:0x%4.4x\n",
			__func__,fw_info->bc_ver);
	dev_info(&ts->spi->dev,
			"[elan] %s rx/tx: %d:%d\n",
			__func__,fw_info->rx, fw_info->tx);
	dev_info(&ts->spi->dev,
			"[elan] %s finger osr: %d\n",
			__func__, fw_info->finger_osr);
	dev_info(&ts->spi->dev,
			"[elan] %s finger x/y resolution: %d:%d\n",
			__func__, fw_info->x_res, fw_info->y_res);
	dev_info(&ts->spi->dev,
			  "[elan] %s  testsolversion:testversion :0x%4.4x/0x%4.4x\n",
			   __func__,fw_info->testsolversion,fw_info->testversion);
out:
	read_fw_flag = 0;
	return ret;
}

static int elan_sysfs_create(struct elants_data *sysfs)
{
    struct elants_data *ts = spi_get_drvdata(sysfs->spi);
    int ret = 0;

    /* Register sysfs */
    ret = sysfs_create_group(&ts->spi->dev.kobj, &elan_attr_group);
    if (ret) {
        ELAN_DEBUG("create sysfs attributes failed, ret = %d\n", ret);
        goto fail_un;
    }
    return 0;

fail_un:
    /* Remove sysfs */
    sysfs_remove_group(&ts->spi->dev.kobj, &elan_attr_group);

    return ret;
}

static int elan_gpio_config(struct elants_data *ts)
{
    int ret = 0;
/*
    //For OpenQ820
    // Configure vddi GPIO (Output)
    ret = gpio_request(ts->vddi_gpio, "elan-vddi");
    if (ret < 0)
        ELAN_DEBUG("vddi pin request gpio failed, ret = %d\n", ret);
    else
        gpio_direction_output(ts->vddi_gpio, 1);

    ELAN_DEBUG("vddi msleep(100)\n");
    msleep(100);

    // Configure apqio1 GPIO (Output)
    ret = gpio_request(ts->apqio1_gpio, "elan-apqio1");
    if (ret < 0) {
        gpio_free(ts->vddi_gpio);
        ELAN_DEBUG("apqio1 pin request gpio failed, ret = %d\n", ret);
    } else {
        gpio_direction_output(ts->apqio1_gpio, 1);
	}

    ELAN_DEBUG("apqio1 msleep(100)\n");
    msleep(100);

    // Configure apqio2 GPIO (Output)
    ret = gpio_request(ts->apqio2_gpio, "elan-apqio2");
    if (ret < 0) {
        gpio_free(ts->vddi_gpio);
        gpio_free(ts->apqio1_gpio);
        ELAN_DEBUG("apqio2 pin request gpio failed, ret = %d\n", ret);
    } else {
        gpio_direction_output(ts->apqio2_gpio, 1);
	}

    // Configure RESX GPIO (Output)
    ret = gpio_request(ts->resx_gpio, "elan-resx");
    if (ret < 0) {
        gpio_free(ts->vddi_gpio);
        gpio_free(ts->apqio1_gpio);
        gpio_free(ts->apqio2_gpio);
        ELAN_DEBUG("resx pin request gpio failed, ret = %d\n", ret);
    } else {
        gpio_direction_output(ts->resx_gpio, 1);
	}

    ELAN_DEBUG("apqio2 msleep(200)\n");
    msleep(200);

    gpio_set_value(ts->resx_gpio, 0);
    msleep(20);
    ELAN_DEBUG("RESX = %d\n", gpio_get_value(ts->resx_gpio));
    msleep(20);
    gpio_set_value(ts->resx_gpio, 1);
    msleep(20);
    ELAN_DEBUG("RESX = %d\n", gpio_get_value(ts->resx_gpio));
    msleep(20);
    //For OpenQ820
*/
    // Configure INT GPIO (Input)
    ret = gpio_request(ts->int_gpio, "elan-irq");
    if (ret < 0) {
        ELAN_DEBUG("interrupt pin request gpio failed, ret = %d\n", ret);
	} else {
        gpio_direction_input(ts->int_gpio);
        ts->irq = gpio_to_irq(ts->int_gpio);
        if (ts->irq < 0) {
            ELAN_DEBUG("gpio to irq failed, irq = %d", ts->irq);
            ret = -1;
        } else {
            ELAN_DEBUG("gpio to irq success, irq = %d\n",ts->irq);
		}
    }

    // Configure RST GPIO (Output)
    ret =  gpio_request(ts->rst_gpio, "elan-rst");
    if (ret < 0) {
        gpio_free(ts->int_gpio);
        free_irq(ts->irq, ts);
        ELAN_DEBUG("reset pin request gpio failed, ret = %d\n", ret);
    } else {
        gpio_direction_output(ts->rst_gpio, 1);
	}

    return ret;
}

static int elan_dts_init(struct elants_data *ts, struct device_node *np)
{
    ts->rst_gpio = of_get_named_gpio(np, "touch,reset-gpio", 0);
    ELAN_DEBUG("rst_gpio = %d\n", ts->rst_gpio);
    if (ts->rst_gpio < 0)
        return ts->rst_gpio;

    ts->int_gpio = of_get_named_gpio(np, "touch,irq-gpio", 0);
    ELAN_DEBUG("int_gpio = %d\n", ts->int_gpio);
    if (ts->int_gpio < 0)
        return ts->int_gpio;
/*
    //For OpenQ820
    ts->resx_gpio = of_get_named_gpio(np, "elan,resx-gpio", 0);
    ELAN_DEBUG("resx_gpio = %d\n", ts->resx_gpio);
    if (ts->resx_gpio < 0)
        return ts->resx_gpio;

    ts->apqio1_gpio = of_get_named_gpio(np, "elan,apqio1-gpio", 0);
    ELAN_DEBUG("apqio1_gpio = %d\n", ts->apqio1_gpio);
    if (ts->apqio1_gpio < 0)
        return ts->apqio1_gpio;

    ts->apqio2_gpio = of_get_named_gpio(np, "elan,apqio2-gpio", 0);
    ELAN_DEBUG("apqio2_gpio = %d\n", ts->apqio2_gpio);
    if (ts->apqio2_gpio < 0)
        return ts->apqio2_gpio;

    ts->vddi_gpio = of_get_named_gpio(np, "elan,vddi-gpio", 0);
    ELAN_DEBUG("vddi_gpio = %d\n", ts->vddi_gpio);
    if (ts->vddi_gpio < 0)
        return ts->vddi_gpio;
    //For OpenQ820
*/
    return 0;
}


/*******************************************************
Function:
   	Power on  Funtion.
Input:
    ts: elan_ts_data struct.
    on: bool, true:on, flase:off
Output:
    Executive outcomes.
    0: succeed. otherwise: failed 
*******************************************************/

static int elan_ts_power_on(struct elants_data *ts, bool on)
{
	int ret = 0;
	
	if (!on)
		goto power_off;

	ret = regulator_enable(ts->vdd);
	if (ret) {
		dev_err(&ts->spi->dev,
				"Regulator vdd enable failed ret = %d\n",ret);
		return ret;
	}

	ret = regulator_enable(ts->vcc);
	if (ret) {
		dev_err(&ts->spi->dev,
				"Regulator vcc enable failed ret = %d\n",ret);
		regulator_disable(ts->vdd);
	}
	return ret;

power_off:
	ret = regulator_disable(ts->vdd);
	if (ret) {
		dev_err(&ts->spi->dev,
				"Regulator vdd disable failed ret = %d\n",ret);
		return ret;
	}

	ret = regulator_disable(ts->vcc);
	if (ret) {
		dev_err(&ts->spi->dev,
				"Regulator vcc disable failed ret = %d\n", ret);
		ret = regulator_enable(ts->vdd);
		if (ret)
			dev_err(&ts->spi->dev,
					"Regulator vdd enable failed ret = %d\n", ret);
	}

	return ret;
}

static int elan_power_initial(struct elants_data *ts)
{
	int ret = 0;
	
	ts->vdd = regulator_get(&ts->spi->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		dev_err(&ts->spi->dev,
			"Regulator get failed vdd rc=%d\n", ret);
		return ret;
	}


	if (regulator_count_voltages(ts->vdd) > 0) {
		ret = regulator_set_voltage(ts->vdd,ELAN_VTG_MIN_UV,
				ELAN_VTG_MAX_UV);
		if (ret) {
			dev_err(&ts->spi->dev,
				"Regulator set_vtg failed vdd rc=%d\n", ret);
			goto reg_vdd_put;
		}
	}

	ts->vcc = regulator_get(&ts->spi->dev, "vcc");
	if (IS_ERR(ts->vcc)) {
		ret = PTR_ERR(ts->vcc);
		dev_err(&ts->spi->dev,
			"Regulator get failed vcc rc=%d\n", ret);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(ts->vcc) > 0) {
		ret = regulator_set_voltage(ts->vcc, ELAN_IO_VTG_MIN_UV,
				ELAN_IO_VTG_MAX_UV);
		if (ret) {
			dev_err(&ts->spi->dev,
				"Regulator set_vtg failed vcc rc=%d\n", ret);
			goto reg_vcc_put;
		}
	}

	return ret;

	
reg_vcc_put:
	regulator_put(ts->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(ts->vdd) > 0)
		regulator_set_voltage(ts->vdd, 0, ELAN_VTG_MAX_UV);


reg_vdd_put:
	regulator_put(ts->vdd);	
	return ret;
}

static int elan_ts_set_power(struct elants_data *ts, bool on)
{
	int ret = 0;
		
	if(!on) {
		ret = on;
		goto pwr_deinit;
	}
		
	/*initial power*/
	ret = elan_power_initial(ts);
	if(ret)
		goto elan_power_init_failed;
		
	/*power on*/
	ret = elan_ts_power_on(ts,on);
	if(ret)
		goto elan_power_on_failed;
		
	return ret;

elan_power_on_failed:
	regulator_put(ts->vdd);
	regulator_put(ts->vcc);
elan_power_init_failed:
pwr_deinit:
	return ret;
}


static int elan_probe(struct spi_device *spi)
{
    struct elants_data *ts = NULL;
    //struct input_dev *input_dev = NULL;
    int ret = 0;
	int retry_cnt = 3;
	
//add by Minger	
#if 1
	  struct task_struct *work_thread;
#endif
//endif

    ELAN_DEBUG("%s(), version = %s\n", __func__, VERSION_LOG);
    printk(KERN_ERR "[ELAN/E]%s: line = %d\n", __func__,__LINE__);
    //[TODO] check Herman for usage.
    //init_completion(&cmd_done);

    /* Setup SPI */
    spi->mode = SPI_MODE_0;             // set at spi_board_info
    spi->max_speed_hz = SPI_MAX_SPEED;  // set at spi_board_info
    spi->chip_select = 0;               // set at spi_board_info
    spi->bits_per_word = 8;             // do not change

    ret = spi_setup(spi);
    if (ret < 0)
        ELAN_DEBUG("spi_setup failed, ret = %d\n", ret);

    /* Allocate Device Data */
    ts = kzalloc(sizeof(struct elants_data), GFP_KERNEL);
    if (!ts) {
        ELAN_DEBUG("kzmalloc elan data failed\n");
        return -ENOMEM;
    }

    ts->spi = spi;

    spi_set_drvdata(spi, ts);

    //[TODO] temp for sysfs, check how to convert data structure by sysfs
    private_ts = ts;

    //[TODO] check reserve or not
    init_waitqueue_head(&ts->elan_wait);

    /* Init Sysfs */
    ret = elan_sysfs_create(ts);
    if (ret < 0)
        ELAN_DEBUG("sysfs create failed, ret = %d\n", ret);
    
    //[TODO] check wake_lock_init meaning
    //wake_lock_init(&ts->wake_lock, WAKE_LOCK_SUSPEND, "ts_wake_lock");
    wakeup_source_init(&ts->wake_lock,"ts_wake_lock");
    //wake_lock_init(&ts->hal_wake_lock, WAKE_LOCK_SUSPEND, "hal_ts_wake_lock");
    wakeup_source_init(&ts->hal_wake_lock,"hal_ts_wake_lock");
    /* Init Char Device */
    ret = elan_setup_cdev(ts);
    if (ret < 0)
        ELAN_DEBUG("setup device failed, ret = %d\n", ret);

    /* Init device tree setting */
    spi->dev.of_node=of_find_compatible_node(NULL, NULL, "elan,elan_ktd_spi");

	
	ret = elan_ts_set_power(ts,1);
	if (ret) {
		  ELAN_DEBUG("%s power seting  failed\n",__func__);
		//goto free_io_port;
	}
	mdelay(100);

    ret = elan_dts_init(ts, spi->dev.of_node);
    if (ret < 0)
        ELAN_DEBUG("device tree initial failed, ret = %d\n", ret);

    ret = elan_gpio_config(ts);
    if (ret < 0)
        ELAN_DEBUG("gpio config failed, ret = %d\n", ret);

    /* Checking IC Status */
retry:	
    ret = elan_check_status(ts);
    ELAN_DEBUG("zx elan_probexxxxx %d\n", ret);
    if (ret < 0) {
		if(retry_cnt--) {
			elan_reset(ts);
			mdelay(20);
			ELAN_DEBUG("checking status retry count %d\n", retry_cnt);
			goto retry;
		} else {
			ELAN_DEBUG("checking status retry failed %d\n", retry_cnt);
			goto free_io;
		}
	} else {
		//mdelay(300);
		/*get maincode hello*/
		 elan_spi_read_mc_hello(ts);
		 if (ret < 0)
			ELAN_DEBUG("Read MainCode hello failed, ret = %d\n", ret);
		
		mdelay(5);			
		/* Read FW Information */
		ret = elan_fw_packet_handler(ts);
		if (ret < 0)
			ELAN_DEBUG("Read FW Information failed, ret = %d\n", ret);		
	}


    /* Init Finger Input Device */
    ret = elan_finger_input_create(ts);
    if (ret < 0)
        ELAN_DEBUG("finger input device create failed, ret = %d\n", ret);

    /* Init Pen Input Device */
    ret = elan_pen_input_create(ts);
    if (ret < 0)
        ELAN_DEBUG("pen input device create failed, ret = %d\n", ret);
//add by Minger
#if 1
	work_thread = kthread_run(touch_event_handler, 0, "elan_ts");
	if(IS_ERR(work_thread)) {
      ret = PTR_ERR(work_thread);
      printk("[elan error] failed to create kernel thread: %d\n", ret);
      return -EINVAL;
	}
//endif
#else	
	  /* Init interrupt */
    elan_wq = create_singlethread_workqueue("elan_wq");
    if (!elan_wq)
        ELAN_DEBUG("create workqueue failed\n");

    INIT_WORK(&ts->work, elan_work_func);	
#endif	

  

    ret = request_irq(ts->irq, elan_irq_handler,
                    /*IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_ONESHOT, */
                    IRQF_TRIGGER_FALLING |/* IRQF_TRIGGER_LOW |*/ IRQF_ONESHOT,
                    spi->dev.driver->name, ts);
    if (ret)
        ELAN_DEBUG("request irq failed, ret = %d\n", ret);

    //[TODO] check usage
    irq_set_irq_wake(ts->irq, 1);
    ts->irq_status = true;

#if defined(CONFIG_FB)
    ELAN_DEBUG("Register fb_notify\n");
    fb_notif.notifier_call = fb_notifier_callback;
    fb_register_client(&fb_notif);
#endif

    ELAN_DEBUG("%s() End\n", __func__);
	return 0;
	
free_io:
	ELAN_DEBUG("%s() End failed\n", __func__);
	if (gpio_is_valid(ts->rst_gpio)) 
		gpio_free(ts->rst_gpio);
	
	if (gpio_is_valid(ts->int_gpio))
		gpio_free(ts->int_gpio);
		
    return ret;
}

//MODULE_DEVICE_TABLE(spi, efp_id);
//#ifdef CONFIG_OF
static struct of_device_id elan_of_match[] = {
    { .compatible = "elan,elan_ktd_spi",},
    {},
};

MODULE_DEVICE_TABLE(of, elan_of_match);
//#endif

static struct spi_driver elan_driver = {
    .driver = {
        .name           = "elan_ktd_spi",
        .bus            = &spi_bus_type,
        .owner          = THIS_MODULE,
//#ifdef CONFIG_OF
        .of_match_table = elan_of_match,
//#endif
    },
    .probe      = elan_probe,
};

static int __init elan_init(void)
{
    //ELAN_DEBUG("%s() Start\n", __func__);
    printk(KERN_ERR "[ELAN/E]%s: line = %d\n", __func__,__LINE__);
    if (spi_register_driver(&elan_driver))
        return -EINVAL;
    ELAN_DEBUG("%s() End\n", __func__);
    return 0;
}

static void __exit elan_exit(void)
{
    spi_unregister_driver(&elan_driver);

    if (elan_wq)
        destroy_workqueue(elan_wq);
}
module_init(elan_init);
module_exit(elan_exit);

MODULE_AUTHOR("ELAN");
MODULE_DESCRIPTION("spi touchscreen driver for ree");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");

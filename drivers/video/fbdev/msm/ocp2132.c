#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/rtc.h>
#include <linux/of_gpio.h>
#include "ocp2132.h"
int ocp_p_gpio;
int ocp_n_gpio;

struct i2c_client* ocp2132wpad_i2c_client  = NULL;;
static int Ocp2132wpad_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data);

void ocp2132_lcd_resume(uint8_t voltage)
{
	int rc = 0;

	if (gpio_is_valid(ocp_n_gpio)) {			

		rc = gpio_direction_output(ocp_n_gpio, 2);
		if (rc) {
			pr_err("%s: unable to set dir for mode gpio\n",
				__func__);
			return;
		}
		gpio_set_value(ocp_n_gpio, 1);
		usleep_range( 5000, 5010);
	}
	Ocp2132wpad_I2C_Write(ocp2132wpad_i2c_client, OCP2131_AVEE_ADDR, voltage);
	usleep_range( 3000, 3010);
	Ocp2132wpad_I2C_Write(ocp2132wpad_i2c_client, OCP2131_AVDD_ADDR, voltage);
	usleep_range( 3000, 3010);

	if (gpio_is_valid(ocp_p_gpio)) {			

		rc = gpio_direction_output(ocp_p_gpio, 2);
		if (rc) {
			pr_err("%s: unable to set dir for mode gpio\n",
				__func__);
			return;
		}
		gpio_set_value(ocp_p_gpio, 1);
		usleep_range( 5000, 5010);
	}
}

void ocp2132_lcd_suspend(void)
{
   
	usleep_range( 20000, 20010);
	gpio_set_value(ocp_n_gpio,0);
	gpio_free(ocp_n_gpio);
	usleep_range( 300, 310);	
	gpio_set_value(ocp_p_gpio,0);
	gpio_free(ocp_p_gpio);

	usleep_range( 10000, 10010);
}



static int ocp2132wpad_i2c_write(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

	if(!client)  return -1;
    buffer[0] = regaddr ;
    buffer[1] = data;

    for(retry = 0; retry < OCP_2132_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);
        if (ret == txbyte)
        {
            break;
        }

        //printk("i2c write error,TXBYTES %d\n",ret);
        mdelay(10);
    }

    if(retry>=OCP_2132_RETRY_COUNT)
    {
        //printk("i2c write retry over %d\n", OCP_2132_RETRY_COUNT);
        return -EINVAL;
    }
    return ret;
}

static int Ocp2132wpad_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = ocp2132wpad_i2c_write(client, regaddr, data, 0x02);
    return ret;
}


static int ocp2132wpad_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	ocp2132wpad_i2c_client = client;

		ocp_n_gpio = of_get_named_gpio(
					client->dev.of_node,
					"ocp,n-gpio", 0);
		if (!gpio_is_valid(ocp_n_gpio))
			pr_info("%s:%d, lcd vsn gpio not specified\n",
							__func__, __LINE__);

		ocp_p_gpio = of_get_named_gpio(
					client->dev.of_node,
					"ocp,p-gpio", 0);
		if (!gpio_is_valid(ocp_p_gpio))
			pr_info("%s:%d, lcd vsp gpio not specified\n",
							__func__, __LINE__);	

	return 0;
}
static int ocp2132wpad_remove(struct i2c_client *client)
{	

	return 0;
}
static const struct i2c_device_id ocp2132wpad_id[] = {
	{OCP2132PWAD_NAME, 0},
	{}
};
static const struct of_device_id ocp2132wpad_of_match[] = {
	{.compatible = "ocp,ocp_wpad",},
	{}
};
MODULE_DEVICE_TABLE(of,ocp2132wpad_of_match);
static struct i2c_driver ocp2132wpad_i2c_driver = {
	.probe = ocp2132wpad_probe,
	.remove = ocp2132wpad_remove,
	.id_table = ocp2132wpad_id,
	.driver = {
		   .name = "ocp,ocp_wpad",
		   .owner = THIS_MODULE,
		   .of_match_table = ocp2132wpad_of_match,
		   },
};

static int __init ocp2132wpad_i2c_init(void)
{
	int rc = 0;
	rc = i2c_add_driver(&ocp2132wpad_i2c_driver);
	return rc;
}

module_init(ocp2132wpad_i2c_init);

static void __exit  ocp2132wpad_i2c_exit(void)
{
	i2c_del_driver(&ocp2132wpad_i2c_driver);
}

module_exit(ocp2132wpad_i2c_exit);

MODULE_DESCRIPTION("ocp2132wpad Driver");
MODULE_LICENSE("GPL v2");

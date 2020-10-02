/**
 * drivers/extcon/extcon-dp-gpio.c - DP GPIO extcon driver
 *
 * Copyright (C) 2020 Engicam srl - http://www.engicam.com
 * Author: Massimo Manetti <support@engicam.com>
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
 * Author: Roger Quadros <rogerq@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define DP_GPIO_DEBOUNCE_MS	20	/* ms */

struct dp_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	struct gpio_desc *id_gpiod;
	int id_irq;

	unsigned long debounce_jiffies;
	struct delayed_work wq_detcable;
};

static const unsigned int dp_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

static void dp_extcon_detect_cable(struct work_struct *work)
{
	int id;
	union extcon_property_value property;
	struct dp_extcon_info *info = container_of(to_delayed_work(work),
						    struct dp_extcon_info,
						    wq_detcable);

	printk("***************** %s\n", __func__);
	property.intval = false;	
	extcon_set_property(info->edev, EXTCON_USB,
			    EXTCON_PROP_USB_TYPEC_POLARITY, property);
	extcon_set_property(info->edev, EXTCON_USB_HOST,
			    EXTCON_PROP_USB_TYPEC_POLARITY, property);
	extcon_set_property(info->edev, EXTCON_DISP_DP,
			    EXTCON_PROP_USB_TYPEC_POLARITY, property);

	extcon_set_state(info->edev, EXTCON_USB, 0);
	extcon_set_state(info->edev, EXTCON_USB_HOST, 0);

	property.intval = false;	
	extcon_set_property(info->edev, EXTCON_USB,
			    EXTCON_PROP_USB_SS, property);
	extcon_set_property(info->edev, EXTCON_USB_HOST,
			    EXTCON_PROP_USB_SS, property);
	extcon_set_property(info->edev, EXTCON_DISP_DP,
			    EXTCON_PROP_USB_SS, property);


	/* check ID and update cable state */
	id = gpiod_get_value_cansleep(info->id_gpiod);
	if (id) {
		/*
		 * ID = 1 means DP HOST cable detached.
		 * As we don't have event for DP peripheral cable attached,
		 * we simulate DP peripheral attach here.
		 */
		extcon_set_cable_state_(info->edev, EXTCON_DISP_DP, 1);
		extcon_set_state(info->edev, EXTCON_DISP_DP, 1);
	} else {
		/*
		 * ID = 0 means DP HOST cable attached.
		 * As we don't have event for DP peripheral cable detached,
		 * we simulate DP peripheral detach here.
		 */
		extcon_set_cable_state_(info->edev, EXTCON_DISP_DP, 0);
		extcon_set_state(info->edev, EXTCON_DISP_DP, 0);
	}
	extcon_sync(info->edev, EXTCON_DISP_DP);
	extcon_sync(info->edev, EXTCON_USB);
	extcon_sync(info->edev, EXTCON_USB_HOST);

}

static irqreturn_t dp_irq_handler(int irq, void *dev_id)
{
	struct dp_extcon_info *info = dev_id;

	queue_delayed_work(system_power_efficient_wq, &info->wq_detcable,
			   info->debounce_jiffies);

	return IRQ_HANDLED;
}

static int dp_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dp_extcon_info *info;
	int ret;

	if (!np)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->id_gpiod = devm_gpiod_get(&pdev->dev, "id", GPIOD_IN);
	if (IS_ERR(info->id_gpiod)) {
		dev_err(dev, "failed to get ID GPIO\n");
		return PTR_ERR(info->id_gpiod);
	}

	info->edev = devm_extcon_dev_allocate(dev, dp_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

	ret = gpiod_set_debounce(info->id_gpiod,
				 DP_GPIO_DEBOUNCE_MS * 1000);
	if (ret < 0)
		info->debounce_jiffies = msecs_to_jiffies(DP_GPIO_DEBOUNCE_MS);

	INIT_DELAYED_WORK(&info->wq_detcable, dp_extcon_detect_cable);

	info->id_irq = gpiod_to_irq(info->id_gpiod);
	if (info->id_irq < 0) {
		dev_err(dev, "failed to get ID IRQ\n");
		return info->id_irq;
	}

	ret = devm_request_threaded_irq(dev, info->id_irq, NULL,
					dp_irq_handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					pdev->name, info);
	if (ret < 0) {
		dev_err(dev, "failed to request handler for ID IRQ\n");
		return ret;
	}

	platform_set_drvdata(pdev, info);
	device_init_wakeup(dev, 1);

	ret = extcon_set_property_capability(info->edev, EXTCON_DISP_DP,
					     EXTCON_PROP_USB_TYPEC_POLARITY);
	printk("ret = %d\n", ret);

	ret |= extcon_set_property_capability(info->edev, EXTCON_USB,
					     EXTCON_PROP_USB_TYPEC_POLARITY);
	printk("ret = %d\n", ret);
	ret |= extcon_set_property_capability(info->edev, EXTCON_USB_HOST,
					     EXTCON_PROP_USB_TYPEC_POLARITY);
	printk("ret = %d\n", ret);

	if (ret) {
		dev_err(dev,
			"failed to set EXTCON_PROP_USB_TYPEC_POLARITY property capability: %d\n",
			ret);
		return ret;
	}

	ret = extcon_set_property_capability(info->edev, EXTCON_DISP_DP,
					     EXTCON_PROP_USB_SS);
	ret |= extcon_set_property_capability(info->edev, EXTCON_USB,
					     EXTCON_PROP_USB_SS);
	ret |= extcon_set_property_capability(info->edev, EXTCON_USB_HOST,
					     EXTCON_PROP_USB_SS);
	if (ret) {
		dev_err(dev,
			"failed to set EXTCON_PROP_USB_SS property capability: %d\n",
			ret);
		return ret;
	}


	/* Perform initial detection */
	dp_extcon_detect_cable(&info->wq_detcable.work);

	return 0;
}

static int dp_extcon_remove(struct platform_device *pdev)
{
	struct dp_extcon_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->wq_detcable);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dp_extcon_suspend(struct device *dev)
{
	struct dp_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev)) {
		ret = enable_irq_wake(info->id_irq);
		if (ret)
			return ret;
	}

	/*
	 * We don't want to process any IRQs after this point
	 * as GPIOs used behind I2C subsystem might not be
	 * accessible until resume completes. So disable IRQ.
	 */
	disable_irq(info->id_irq);

	return ret;
}

static int dp_extcon_resume(struct device *dev)
{
	struct dp_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev)) {
		ret = disable_irq_wake(info->id_irq);
		if (ret)
			return ret;
	}

	enable_irq(info->id_irq);
	if (!device_may_wakeup(dev))
		queue_delayed_work(system_power_efficient_wq,
				   &info->wq_detcable, 0);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(dp_extcon_pm_ops,
			 dp_extcon_suspend, dp_extcon_resume);

static const struct of_device_id dp_extcon_dt_match[] = {
	{ .compatible = "linux,extcon-dp-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dp_extcon_dt_match);

static struct platform_driver dp_extcon_driver = {
	.probe		= dp_extcon_probe,
	.remove		= dp_extcon_remove,
	.driver		= {
		.name	= "extcon-dp-gpio",
		.pm	= &dp_extcon_pm_ops,
		.of_match_table = dp_extcon_dt_match,
	},
};

module_platform_driver(dp_extcon_driver);

MODULE_AUTHOR("Massimo Manetti <support@engicam.com>");
MODULE_DESCRIPTION("DP GPIO extcon driver");
MODULE_LICENSE("GPL v2");

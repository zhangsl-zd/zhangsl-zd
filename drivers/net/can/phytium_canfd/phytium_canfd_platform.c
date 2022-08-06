// SPDX-License-Identifier: GPL-2.0
/* CAN bus driver for Phytium CAN controller
 *
 * Copyright (C) 2021 Phytium Technology Co., Ltd.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/acpi.h>
#include "phytium_canfd.h"

/* CAN Bittiming constants as per Phytium CAN specs */
static const struct can_bittiming_const phytium_bittiming_const_8192 = {
	.name = DRIVER_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 8192,
	.brp_inc = 2,
};

static const struct phytium_can_devtype phytium_canfd_data = {
	.cantype = PHYTIUM_CANFD,
	.bittiming_const = &phytium_bittiming_const_8192,
};

static const struct of_device_id phytium_can_of_ids[] = {
	{
		.compatible = "phytium,canfd", .data = &phytium_canfd_data
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, phytium_can_of_ids);

static int phytium_can_probe(struct platform_device *pdev)
{
	struct resource *res; /* IO mem resources */
	struct net_device *ndev;
	struct phytium_can_priv *priv;
	void __iomem *addr;
	int ret, rx_max, tx_max;
	const struct of_device_id *of_id;
	const struct phytium_can_devtype *devtype = &phytium_canfd_data;

	/* Get the virtual base address for the device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr)) {
		ret = PTR_ERR(addr);
		goto err;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth", &tx_max);
	if (ret < 0)
		goto err;

	ret = of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth", &rx_max);
	if (ret < 0)
		goto err;

	/* Create a CAN device instance */
	ndev = alloc_candev(sizeof(struct phytium_can_priv), tx_max);
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->reg_base = addr;
	priv->dev = &pdev->dev;
	priv->tx_max = tx_max;

	/* Get IRQ for the device */
	ndev->irq = platform_get_irq(pdev, 0);
	ndev->flags |= IFF_ECHO;	/* We support local echo */
	priv->irq_flags = IRQF_SHARED;

	/* Getting the CAN can_clk info */
	if (pdev->dev.of_node) {
		priv->can_clk = devm_clk_get(priv->dev, "can_clk");
		if (IS_ERR(priv->can_clk)) {
			dev_err(priv->dev, "Device clock not found.\n");
			ret = PTR_ERR(priv->can_clk);
			goto err;
		}
		priv->can.clock.freq = clk_get_rate(priv->can_clk);

		of_id = of_match_device(phytium_can_of_ids, &pdev->dev);
		if (of_id && of_id->data)
			devtype = of_id->data;

	} else if (has_acpi_companion(&pdev->dev)) {
		ret = fwnode_property_read_u32(dev_fwnode(&pdev->dev),
					       "clock-frequency",
					       &priv->can.clock.freq);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to get clock frequency.\n");
			goto err;
		}
	}

	ret = clk_prepare_enable(priv->can_clk);
	if (ret)
		return ret;

	if (devtype->cantype == PHYTIUM_CANFD)
		priv->fdmode = PHYTIUM_CANFD;
	else
		priv->fdmode = PHYTIUM_CAN;

	priv->can.bittiming_const = devtype->bittiming_const;
	dev_info(priv->dev, "CAN register address :%p, phy address:%llx\n",
		 addr, res->start);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = phytium_canfd_register(ndev);

err:
	return ret;
}

static int phytium_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	int ret;

	ret = phytium_canfd_unregister(ndev);
	if (ret)
		dev_warn(&pdev->dev, "can't remove device properly: %d\n", ret);

	return ret;
}

static int __maybe_unused phytium_can_suspend(struct device *dev)
{
	/* ??? */
	return 0;
}

static int __maybe_unused phytium_can_resume(struct device *dev)
{
	/* ??? */
	return 0;
}

static const struct dev_pm_ops phytium_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(phytium_can_suspend, phytium_can_resume)
};

static struct platform_driver phytium_can_driver = {
	.driver	= {
		.name		= "phytium-canfd",
		.of_match_table = of_match_ptr(phytium_can_of_ids),
		.pm		= &phytium_dev_pm_ops,
	},
	.probe = phytium_can_probe,
	.remove	= phytium_can_remove,
};
module_platform_driver(phytium_can_driver)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Phytium canfd controller driver");
MODULE_AUTHOR("Cheng Quan <chengquan@phytium.com.cn>");

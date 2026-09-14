/*
 * Spreadtrum pmic watchdog driver
 * Copyright (C) 2020 Spreadtrum - http://www.spreadtrum.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/alarmtimer.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/sipc.h>
#include <linux/workqueue.h>
#include <uapi/linux/sched/types.h>


#define SPRD_PMIC_WDT_LOAD_LOW		0x0
#define SPRD_PMIC_WDT_LOAD_HIGH		0x4
#define SPRD_PMIC_WDT_CTRL		0x8
#define SPRD_PMIC_WDT_INT_CLR		0xc
#define SPRD_PMIC_WDT_INT_RAW		0x10
#define SPRD_PMIC_WDT_INT_MSK		0x14
#define SPRD_PMIC_WDT_CNT_LOW		0x18
#define SPRD_PMIC_WDT_CNT_HIGH		0x1c
#define SPRD_PMIC_WDT_LOCK			0x20
#define SPRD_PMIC_WDT_IRQ_LOAD_LOW		0x2c
#define SPRD_PMIC_WDT_IRQ_LOAD_HIGH		0x30

/* WDT_CTRL */
#define SPRD_PMIC_WDT_INT_EN_BIT		BIT(0)
#define SPRD_PMIC_WDT_CNT_EN_BIT		BIT(1)
#define SPRD_PMIC_WDT_NEW_VER_EN		BIT(2)
#define SPRD_PMIC_WDT_RST_EN_BIT		BIT(3)

/* WDT_INT_CLR */
#define SPRD_PMIC_WDT_INT_CLEAR_BIT		BIT(0)
#define SPRD_PMIC_WDT_RST_CLEAR_BIT		BIT(3)

/* WDT_INT_RAW */
#define SPRD_PMIC_WDT_INT_RAW_BIT		BIT(0)
#define SPRD_PMIC_WDT_RST_RAW_BIT		BIT(3)
#define SPRD_PMIC_WDT_LD_BUSY_BIT		BIT(4)

/* 1s equal to 32768 counter steps */
#define SPRD_PMIC_WDT_CNT_STEP		32768

#define SPRD_PMIC_WDT_UNLOCK_KEY		0xe551
#define SPRD_PMIC_WDT_MIN_TIMEOUT		3
#define SPRD_PMIC_WDT_MAX_TIMEOUT		60

#define SPRD_PMIC_WDT_CNT_HIGH_SHIFT		16
#define SPRD_PMIC_WDT_LOW_VALUE_MASK		GENMASK(15, 0)
#define SPRD_PMIC_WDT_LOAD_TIMEOUT		1000

#define SPRD_PMIC_WDT_TIMEOUT		60
#define SPRD_PMIC_WDT_PRETIMEOUT	0
#define SPRD_PMIC_WDT_FEEDTIME		45

#define SPRD_PMIC_WDTEN_MAGIC "e551"
#define SPRD_PMIC_WDTEN_MAGIC_LEN_MAX  10

#define PMIC_WDT_WAKE_UP_MS 2000
#define PMIC_WDT_DIAG_INTERVAL_MS 30000
#define PMIC_WDT_DIAG_TAG "[A3DBG][PM_SYS_WDT]"

struct sprd_pmic_wdt {
	struct regmap		*regmap;
	struct device		*dev;
	u32			base;
	bool wdten;
	struct alarm wdt_timer;
	struct kthread_worker wdt_kworker;
	struct kthread_work wdt_kwork;
	struct task_struct *wdt_thread;
	struct delayed_work diag_work;
	u32 wdt_flag;
	int last_cmd_ret;

};

static int sprd_pmic_wdt_enable(struct sprd_pmic_wdt *wdt, bool en)
{
	int nwrite;
	int timeout = 100;
	char *p_cmd;
	int len;

	if (en)
		p_cmd = "watchdog on";
	else
		p_cmd = "watchdog rstoff";

	len = strlen(p_cmd) + 1;
	dev_notice(wdt->dev,
		   PMIC_WDT_DIAG_TAG " tx cmd='%s' len=%d wdten=%d flag=%u\n",
		   p_cmd, len, en, wdt->wdt_flag);

	nwrite = sbuf_write(SIPC_ID_PM_SYS, SMSG_CH_TTY, 0,
			    p_cmd, len, msecs_to_jiffies(timeout));
	wdt->last_cmd_ret = nwrite;

	if (nwrite != len) {
		dev_err(wdt->dev,
			PMIC_WDT_DIAG_TAG " sbuf_write failed ret=%d expected=%d cmd='%s'\n",
			nwrite, len, p_cmd);
		return nwrite < 0 ? nwrite : -EIO;
	}

	dev_notice(wdt->dev,
		   PMIC_WDT_DIAG_TAG " command accepted ret=%d cmd='%s'\n",
		   nwrite, p_cmd);
	return 0;
}

static void sprd_pimc_wdt_init(int event, void *data)
{
	struct sprd_pmic_wdt *pmic_wdt = data;

	dev_notice(pmic_wdt->dev,
		   PMIC_WDT_DIAG_TAG " notifier event=%d flag=%u wdten=%d\n",
		   event, pmic_wdt->wdt_flag, pmic_wdt->wdten);

	switch (event) {
	case SBUF_NOTIFY_READY:
		dev_notice(pmic_wdt->dev,
			   PMIC_WDT_DIAG_TAG " PM_SYS SBUF is READY; queue watchdog command\n");
		pm_wakeup_event(pmic_wdt->dev, PMIC_WDT_WAKE_UP_MS);
		kthread_queue_work(&pmic_wdt->wdt_kworker, &pmic_wdt->wdt_kwork);
		pmic_wdt->wdt_flag = 1;
		break;
	case SBUF_NOTIFY_READ:
		if (!pmic_wdt->wdt_flag) {
			dev_notice(pmic_wdt->dev,
				   PMIC_WDT_DIAG_TAG " first PM_SYS SBUF READ; queue watchdog command\n");
			pm_wakeup_event(pmic_wdt->dev, PMIC_WDT_WAKE_UP_MS);
			kthread_queue_work(&pmic_wdt->wdt_kworker, &pmic_wdt->wdt_kwork);
			pmic_wdt->wdt_flag = 1;
		}
		break;
	default:
		dev_notice(pmic_wdt->dev,
			   PMIC_WDT_DIAG_TAG " ignored notifier event=%d\n", event);
		return;
	}
}

static void sprd_pimc_wdt_work(struct kthread_work *work)
{
	struct sprd_pmic_wdt *pmic_wdt = container_of(work,
						 struct sprd_pmic_wdt,
						 wdt_kwork);
	int ret;

	dev_notice(pmic_wdt->dev,
		   PMIC_WDT_DIAG_TAG " worker enter wdten=%d flag=%u\n",
		   pmic_wdt->wdten, pmic_wdt->wdt_flag);

	ret = sprd_pmic_wdt_enable(pmic_wdt, pmic_wdt->wdten);
	if (ret)
		dev_err(pmic_wdt->dev,
			PMIC_WDT_DIAG_TAG " worker command failed ret=%d wdten=%d\n",
			ret, pmic_wdt->wdten);
	else
		dev_notice(pmic_wdt->dev,
			   PMIC_WDT_DIAG_TAG " worker command completed wdten=%d\n",
			   pmic_wdt->wdten);
}

static void sprd_pmic_wdt_diag_work(struct work_struct *work)
{
	struct sprd_pmic_wdt *pmic_wdt = container_of(to_delayed_work(work),
							 struct sprd_pmic_wdt,
							 diag_work);
	int ret = pmic_wdt->last_cmd_ret;

	dev_notice(pmic_wdt->dev,
		   PMIC_WDT_DIAG_TAG " heartbeat jiffies=%lu wdten=%d flag=%u last_cmd_ret=%d\n",
		   jiffies, pmic_wdt->wdten, pmic_wdt->wdt_flag,
		   pmic_wdt->last_cmd_ret);

	/*
	 * Diagnostic builds set wdten=false.  Retry the idempotent rstoff
	 * command periodically as PM_SYS/SBUF may become usable after probe
	 * without delivering the notifier event expected by this vendor tree.
	 */
	if (!pmic_wdt->wdten) {
		ret = sprd_pmic_wdt_enable(pmic_wdt, false);
		dev_notice(pmic_wdt->dev,
			   PMIC_WDT_DIAG_TAG " periodic rstoff retry ret=%d\n", ret);
	}

	schedule_delayed_work(&pmic_wdt->diag_work,
			      msecs_to_jiffies(PMIC_WDT_DIAG_INTERVAL_MS));
}

static bool sprd_pimc_wdt_en(void)
{
	struct device_node *cmdline_node;
	const char *cmd_line, *wdten_name_p;
	char wdten_value[SPRD_PMIC_WDTEN_MAGIC_LEN_MAX] = "NULL";
	int ret;

	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmd_line);

	if (ret) {
		pr_err(PMIC_WDT_DIAG_TAG " can't parse /chosen bootargs ret=%d\n", ret);
		return false;
	}

	wdten_name_p = strstr(cmd_line, "androidboot.wdten=");
	if (!wdten_name_p) {
		pr_err(PMIC_WDT_DIAG_TAG " androidboot.wdten is absent\n");
		return false;
	}

	sscanf(wdten_name_p, "androidboot.wdten=%8s", wdten_value);
	pr_notice(PMIC_WDT_DIAG_TAG " parsed androidboot.wdten='%s'\n",
		  wdten_value);

	if (strncmp(wdten_value, SPRD_PMIC_WDTEN_MAGIC,
		    strlen(SPRD_PMIC_WDTEN_MAGIC))) {
		pr_notice(PMIC_WDT_DIAG_TAG " reset request disabled by bootarg\n");
		return false;
	}

	pr_notice(PMIC_WDT_DIAG_TAG " reset request enabled by bootarg\n");
	return true;
}


static const struct of_device_id sprd_pmic_wdt_of_match[] = {
	{.compatible = "sprd,sc2723t-wdt",},
	{.compatible = "sprd,sc2731-wdt",},
	{.compatible = "sprd,sc2730-wdt",},
	{.compatible = "sprd,sc2721-wdt",},
	{.compatible = "sprd,sc2720-wdt",},
	{.compatible = "sprd,ump9620-wdt",},
	{}
};

static int sprd_pmic_wdt_probe(struct platform_device *pdev)
{
	int ret, rval;
	struct device_node *node = pdev->dev.of_node;
	struct sprd_pmic_wdt *pmic_wdt;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	pmic_wdt = devm_kzalloc(&pdev->dev, sizeof(*pmic_wdt), GFP_KERNEL);
	if (!pmic_wdt)
		return -ENOMEM;

	pmic_wdt->dev = &pdev->dev;
	pmic_wdt->last_cmd_ret = -EAGAIN;
	dev_notice(&pdev->dev,
		   PMIC_WDT_DIAG_TAG " probe begin node=%s\n",
		   node ? node->full_name : "<none>");

	pmic_wdt->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pmic_wdt->regmap) {
		dev_err(&pdev->dev,
			PMIC_WDT_DIAG_TAG " probe failed: no parent regmap\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "reg", &pmic_wdt->base);
	if (ret) {
		dev_err(&pdev->dev,
			PMIC_WDT_DIAG_TAG " failed to get base address ret=%d\n", ret);
		return ret;
	}
	dev_notice(&pdev->dev,
		   PMIC_WDT_DIAG_TAG " regmap=%p base=0x%x\n",
		   pmic_wdt->regmap, pmic_wdt->base);

	rval = device_init_wakeup(pmic_wdt->dev, true);
	if (rval)
		dev_warn(&pdev->dev,
			 PMIC_WDT_DIAG_TAG " device_init_wakeup ret=%d\n", rval);

	kthread_init_worker(&pmic_wdt->wdt_kworker);
	kthread_init_work(&pmic_wdt->wdt_kwork, sprd_pimc_wdt_work);
	pmic_wdt->wdt_thread = kthread_run(kthread_worker_fn,
					   &pmic_wdt->wdt_kworker,
					   "pmic_wdt_worker");
	if (IS_ERR(pmic_wdt->wdt_thread)) {
		ret = PTR_ERR(pmic_wdt->wdt_thread);
		pmic_wdt->wdt_thread = NULL;
		dev_err(&pdev->dev,
			PMIC_WDT_DIAG_TAG " failed to run worker thread ret=%d\n", ret);
		return ret;
	}

	rval = sched_setscheduler(pmic_wdt->wdt_thread, SCHED_FIFO, &param);
	dev_notice(&pdev->dev,
		   PMIC_WDT_DIAG_TAG " worker pid=%d sched_setscheduler ret=%d\n",
		   pmic_wdt->wdt_thread->pid, rval);

	pmic_wdt->wdten = sprd_pimc_wdt_en();
	dev_notice(&pdev->dev,
		   PMIC_WDT_DIAG_TAG " effective wdten=%d before notifier registration\n",
		   pmic_wdt->wdten);

	INIT_DELAYED_WORK(&pmic_wdt->diag_work, sprd_pmic_wdt_diag_work);

	rval = sbuf_register_notifier(SIPC_ID_PM_SYS, SMSG_CH_TTY, 0,
				      sprd_pimc_wdt_init, pmic_wdt);
	dev_notice(&pdev->dev,
		   PMIC_WDT_DIAG_TAG " sbuf_register_notifier dst=%d ch=%d ret=%d\n",
		   SIPC_ID_PM_SYS, SMSG_CH_TTY, rval);
	if (rval) {
		kthread_stop(pmic_wdt->wdt_thread);
		pmic_wdt->wdt_thread = NULL;
		dev_err(&pdev->dev,
			PMIC_WDT_DIAG_TAG " PM_SYS SBUF unavailable; defer probe ret=%d\n",
			rval);
		return -EPROBE_DEFER; /* depends on SPRD_SIPC_SPIPE for SP9863-GO */
	}

	platform_set_drvdata(pdev, pmic_wdt);
	schedule_delayed_work(&pmic_wdt->diag_work,
			      msecs_to_jiffies(5000));
	dev_notice(&pdev->dev,
		   PMIC_WDT_DIAG_TAG " probe complete; diagnostics armed\n");

	return 0;
}

static int sprd_pmic_wdt_remove(struct platform_device *pdev)
{
	struct sprd_pmic_wdt *pmic_wdt = dev_get_drvdata(&pdev->dev);
	int rval;

	dev_notice(&pdev->dev, PMIC_WDT_DIAG_TAG " remove begin\n");
	cancel_delayed_work_sync(&pmic_wdt->diag_work);

	rval = sbuf_register_notifier(SIPC_ID_PM_SYS, SMSG_CH_TTY,
				      0, NULL, NULL);
	if (rval) {
		dev_err(&pdev->dev,
			PMIC_WDT_DIAG_TAG " notifier unregister failed ret=%d\n", rval);
		return rval;
	}

	kthread_flush_worker(&pmic_wdt->wdt_kworker);
	kthread_stop(pmic_wdt->wdt_thread);
	dev_notice(&pdev->dev, PMIC_WDT_DIAG_TAG " remove complete\n");

	return 0;
}

static struct platform_driver sprd_pmic_wdt_driver = {
	.probe = sprd_pmic_wdt_probe,
	.remove = sprd_pmic_wdt_remove,
	.driver = {
		.name = "sprd-pmic-wdt",
		.of_match_table = sprd_pmic_wdt_of_match,
	},
};
module_platform_driver(sprd_pmic_wdt_driver);

MODULE_AUTHOR("Ling Xu <ling_ling.xu@unisoc.com>");
MODULE_DESCRIPTION("Spreadtrum PMIC Watchdog Timer Controller Driver");
MODULE_LICENSE("GPL v2");

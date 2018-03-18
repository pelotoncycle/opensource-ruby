#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
	static struct early_suspend uartirq_early_suspend;
#endif

enum Irq_wake_state {
	UARTIRQ_SUSPEND = 0,
	UARTIRQ_RESUME = 1,
};

enum Irq_evnet_state {
	UARTIRQ_EVENT_UNLOCK = 0,
	UARTIRQ_EVENT_LOCK = 1,
};

enum Irq_disableNoSync_state {
	UARTIRQ_NOSYNC_NONE = 0,
	UARTIRQ_NOSYNC_EN = 1,
};

static struct platform_driver uartirq_driver;
struct input_dev *uartirq_input_dev;
static int gEventLock = UARTIRQ_EVENT_UNLOCK;
static int gWakeLock = 0;
static int gIrqWake = UARTIRQ_SUSPEND;
static int gIrqDisableNoSync = UARTIRQ_NOSYNC_NONE;
static unsigned int uartirq_num;
static bool gTriggerFromRS232 = FALSE;
static bool gIrqLock = FALSE;

static void uartirq_handler(void);
static int uartirq_probe(struct platform_device *pdev);
static int uartirq_remove(struct platform_device *pdev);
static int uartirq_suspend(struct platform_device *pdev, pm_message_t state);
static int uartirq_resume(struct platform_device *pdev);

static struct platform_device uartirq_device = {
	.name = "uartirq_driver",
	.id     = -1,
};

static struct platform_driver uartirq_driver = {
	.probe	 = uartirq_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND 
	.resume  = uartirq_resume,
	.suspend = uartirq_suspend,
#endif
	.remove	 = uartirq_remove,	
	.driver	 = {
		.name  = "uartirq_driver",
	}
};


static void uartirq_handler(void)
{
	/* Wakelock use to block unuse interrupt whan we re-active enanble_irq, we set two times to avoid problem */
	/* Trigger irq wake up device condition: suspend mode and gWakelock unlock*/
	if (gIrqWake == UARTIRQ_SUSPEND && gWakeLock >= 10) {	
		/* Send PRESS POWER KEY evnet to framework */
		input_report_key(uartirq_input_dev, KEY_POWER, 1);
		input_sync(uartirq_input_dev);

		/* Send RELEASE POWER KEY evnet to framework */
		input_report_key(uartirq_input_dev, KEY_POWER, 0);
		input_sync(uartirq_input_dev);

		/* Set gEventLock to LOCK state to avoid unnessary interrupt to trigger input report */
		/* Set gWakeLock count to zero */
		/* Set gIrqWake to RESUME state that means device will enter resume stage. */
		gEventLock = UARTIRQ_EVENT_LOCK;
		gWakeLock = 0;
		gIrqWake = UARTIRQ_RESUME;
		gTriggerFromRS232 = TRUE;
	} else {
		/* If gIrqWake is UARTIRQ_SUSPEND, we set count to make sure that user use bikecycle to wake device  */
		gWakeLock++;
	}
	
	/* IRQ only setup enable /disable once, we  should add flag for avoiding 'irq count lock' problem. */
	if (gIrqLock == TRUE){ 
		/*Avoid unbalance irq problem */
		enable_irq(uartirq_num);
		gIrqLock = FALSE;
	}
}

irqreturn_t uartirq_irq_handler(int irq, void *dev_id)
{
	/* Set condition to avoid no disable irq when interrupt occur after uartirq_resume. */
	if (gIrqWake == UARTIRQ_RESUME) {
		printk("[UARTIRQ] Devices wakeup, irq still active, close it!!\n");

		/* IRQ only setup enable /disable once, we  should add flag for avoiding 'irq count lock' problem. */
		if (gIrqLock != TRUE) {
			disable_irq_nosync(uartirq_num);
			gIrqLock = TRUE;
		}

		/* We must use flag to check irq active state that avoid double disable irq problem */
		gIrqDisableNoSync = UARTIRQ_NOSYNC_EN;
		return IRQ_HANDLED;
	}

	/* Set evnetLock for avoid unnessary evnet report to framework */
	if (gEventLock == UARTIRQ_EVENT_UNLOCK) {
		/* IRQ only setup enable /disable once, we  should add flag for avoiding 'irq count lock' problem. */
		if (gIrqLock != TRUE) {
			/* Don't use disable_irq at here, it could be generate daed lock */
			disable_irq_nosync(uartirq_num);
			gIrqLock = TRUE;
		}

        uartirq_handler();
    }
	
    return IRQ_HANDLED;
}

static int uartirq_remove(struct platform_device *pdev)	
{
	free_irq(uartirq_num, NULL);
	input_unregister_device(uartirq_input_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&uartirq_early_suspend);
#endif	
	gEventLock = UARTIRQ_EVENT_UNLOCK;
	gWakeLock = 0;
	gIrqWake = UARTIRQ_SUSPEND;
	gIrqDisableNoSync = UARTIRQ_NOSYNC_NONE;
	gTriggerFromRS232 = FALSE;
	gIrqLock = FALSE;
	
	return 0;
}

static int uartirq_resume(struct platform_device *pdev)	
{
	printk("[UARTIRQ] uartirq_resume\n");

	/* Dependent with different scanarios, resume function should be handle different procedure. */
	if (gTriggerFromRS232==TRUE) {
		if (gIrqWake == UARTIRQ_RESUME) {
			if (gIrqDisableNoSync != UARTIRQ_NOSYNC_EN) {
				/* IRQ only setup enable /disable once, we  should add flag for avoiding 'irq count lock' problem. */
				if (gIrqLock != TRUE) {
					printk("[UARTIRQ] uartirq_resume(RS232), disable irq\n");
					disable_irq(uartirq_num);
					gIrqLock = TRUE;
				}
			} else {
				printk("[UARTIRQ] uartirq_resume: UARTIRQ_NOSYNC_EN lock, igrone this resume task. \n");
			}
		} else {
			printk("[UARTIRQ] uartirq_resume: devices doesn't stay in resume stage, igrone this resume task. \n");
		}
	} else {
		/* IRQ only setup enable /disable once, we  should add flag for avoiding 'irq count lock' problem. */
		if (gIrqLock != TRUE) {
			printk("[UARTIRQ] uartirq_resume(HW Key), disable irq\n");
			disable_irq(uartirq_num);
			gIrqLock = TRUE;
		}

		gEventLock = UARTIRQ_EVENT_LOCK;
		gWakeLock = 0;
		gIrqWake = UARTIRQ_RESUME;
	}
	
    return 0;
}

static int uartirq_suspend(struct platform_device *pdev, pm_message_t state)	
{
	printk("[UARTIRQ] uartirq_suspend\n");

	/* Cancel irq EventLock */
	if (gEventLock == UARTIRQ_EVENT_LOCK) {
		gEventLock = UARTIRQ_EVENT_UNLOCK;
	}

	/* If gIrqWake is UARTIRQ_RESUME, it means devices enter suspend state at the first time,  we change gIrqWake state and set nosnyc to none if need. */
	if (gIrqWake == UARTIRQ_RESUME) {
		gIrqWake = UARTIRQ_SUSPEND;

		if (gIrqDisableNoSync == UARTIRQ_NOSYNC_EN) {
			gIrqDisableNoSync = UARTIRQ_NOSYNC_NONE;
		}
		
		/* IRQ only setup enable /disable once, we  should add flag for avoiding 'irq count lock' problem. */
		if (gIrqLock == TRUE) {
			printk("[UARTIRQ] uartirq_suspend, enable irq\n");
			enable_irq(uartirq_num);
			gIrqLock = FALSE;
		}

		gTriggerFromRS232 = FALSE;
	}
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void uartirq_irq_early_suspend(struct early_suspend *handler)
{
	pm_message_t state;
	state.event = PM_EVENT_SUSPEND;

	printk("[UARTIRQ] Uartirq_irq_early_suspend\n");
 	uartirq_suspend(&uartirq_device, state);
}

static void uartirq_irq_early_resume(struct early_suspend *handler)
{
	printk("[UARTIRQ] Uartirq_irq_early_resume\n");
	uartirq_resume(&uartirq_device);
}
#endif

static int uartirq_probe(struct platform_device *pdev)	
{
	struct device_node *node = NULL;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "innoc,uartirq");
	if (node) {
		uartirq_num = irq_of_parse_and_map(node, 0);
		printk("[UARTIRQ] uartirq_probe uartirq_num:(%d)!!\n", uartirq_num);

		uartirq_input_dev = input_allocate_device();
		if (!uartirq_input_dev) {
			return -ENOMEM;
		}
		
		/* Set input dev bit, KEY evnet, POWER KEY */
		__set_bit(EV_KEY, uartirq_input_dev->evbit);
		__set_bit(KEY_POWER, uartirq_input_dev->keybit);

		uartirq_input_dev->dev.parent = &pdev->dev;

		ret = input_register_device(uartirq_input_dev);
		if (ret) {
			printk("[UARTIRQ] device failed (%d)\n", ret);
			input_unregister_device(uartirq_input_dev);
			return ret;
		}		
		
		/* Should be disable irq in system init, because system not enter suspend stage yet. */
		irq_set_status_flags(uartirq_num, IRQ_NOAUTOEN);
		gIrqLock = TRUE;

		ret = request_irq(uartirq_num, uartirq_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING, "uartirq", NULL);
		if (ret) {
			printk("[UARTIRQ] Request irq fail\n");
			input_unregister_device(uartirq_input_dev);
			return -EINVAL;
		}

		/* Set irq to wake irq */
		ret = enable_irq_wake(uartirq_num);
		if (ret) {
			printk("[UARTIRQ] Enable_irq_wake fail\n");
			input_unregister_device(uartirq_input_dev);
			return -EINVAL;
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		uartirq_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
		uartirq_early_suspend.suspend = uartirq_irq_early_suspend;
		uartirq_early_suspend.resume = uartirq_irq_early_resume;
		register_early_suspend(&uartirq_early_suspend);
		printk("[UARTIRQ] Register early_suspend done\n");
#endif

		gIrqWake = UARTIRQ_RESUME;
	}

	return 0;
}

static int __init uartirq_init(void)
{
	int ret = 0;

	ret = platform_device_register(&uartirq_device);
	if (ret) {
		printk("[UARTIRQ] Platform_device_register error:(%d)\n", ret);
		return ret;
	} else {
		ret = platform_driver_register(&uartirq_driver);
		if (ret) {
			printk("[UARTIRQ] Platform_driver_register error:(%d)\n", ret);
			return ret;
		} else {
			printk("[UARTIRQ] Platform_driver_register done!\n");
		}
	}
	
	return 0;
}

static void  __exit uartirq_exit(void)
{
	platform_driver_unregister(&uartirq_driver);
}


module_init(uartirq_init);
module_exit(uartirq_exit);

MODULE_AUTHOR("Bob.Tau <bob.tau@innocomm.com>");
MODULE_DESCRIPTION("UART1 to irq driver");
MODULE_LICENSE("GPL");

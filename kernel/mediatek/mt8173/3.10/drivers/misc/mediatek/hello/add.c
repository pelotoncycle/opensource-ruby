#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>	
#include <linux/dma-mapping.h>
#include <linux/xlog.h>
#include <mach/sync_write.h>
#include <mt_typedefs.h>

#define PFX "[mtk_add]"

#define PK_DBG_FUNC(fmt, arg...)    pr_err(PFX "[%s]" fmt, __func__, ##arg)

int mtk_add_for_test(int a, int b)
{
	return a + b;
}

EXPORT_SYMBOL(mtk_add_for_test);



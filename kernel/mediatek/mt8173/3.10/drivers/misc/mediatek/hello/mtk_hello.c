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

extern int mtk_add_for_test(int a, int b);

#define PFX "[mtk_hello]"

#define PK_DBG_FUNC(fmt, arg...)    pr_err(PFX "[%s]" fmt, __func__, ##arg)



static int __init mtk_hello_init(void)
{
    int a = 1, b = 2;
	PK_DBG_FUNC("enter mtk hello init, a + b: %d\n", mtk_add_for_test(a , b));
	return 0;
}

static void __exit mtk_hello_uninit(void)
{
    PK_DBG_FUNC("enter mtk hello uninit\n");
}



module_init(mtk_hello_init);
module_exit(mtk_hello_uninit);

MODULE_DESCRIPTION("MtkHello");
MODULE_AUTHOR("Xixi Chen <xixi.chen@mediatek.com>");
MODULE_LICENSE("GPL");


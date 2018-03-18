#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <sound/mt_soc_audio.h>

#include "mt_spm_internal.h"

/**************************************
 * Config and Parameter
 **************************************/
#define SPM_USE_TWAM_DEBUG	0
#define SPM_USE_GPIO_DEBUG	0


/**************************************
 * Define and Declare
 **************************************/
static bool check_talking;	/* FIXME */

#if SPM_CTRL_BIG_CPU
static const u32 talking_binary[] = {
	0xf0000000, 0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x803d8400, 0x1380081f,
	0x80340400, 0x17c07c1f, 0x17c07c1f, 0x80310400, 0x11c00c1f, 0x1b80001f,
	0x200000b6, 0xa2110408, 0xf0000000, 0x1300101f, 0x1b00001f, 0x3fffe7ff,
	0x1b80001f, 0x20000004, 0xd820034c, 0x17c07c1f, 0x1980001f, 0x3fffe7ff,
	0xd0000440, 0x17c07c1f, 0x11c0141f, 0x1380081f, 0x82863401, 0xd800014a,
	0x1300181f, 0xa0110400, 0xa0140400, 0xa01d8400, 0xf0000000, 0xa2118408,
	0x1900001f, 0x10006830, 0xe1000003, 0xe8208000, 0x10006834, 0x00000001,
	0x18d0001f, 0x10006830, 0x68e00003, 0x0000beef, 0xd8200543, 0x17c07c1f,
	0xf0000000, 0x17c07c1f, 0xe0f07f16, 0x1380201f, 0xe0f07f1e, 0x1380201f,
	0xe0f07f0e, 0x1b80001f, 0x20000104, 0xe0f07f0c, 0xe0f07f0d, 0xe0f07e0d,
	0xe0f07c0d, 0xe0f0780d, 0xe0f0700d, 0xe0f0600d, 0xe0f0400d, 0xe0f0000d,
	0xe0e0000d, 0xf0000000, 0x17c07c1f, 0xe0f07f0d, 0xe0f07f0f, 0xe0f07f1e,
	0xe0f07f12, 0xf0000000, 0x17c07c1f, 0xe0e001fe, 0xe0e003fc, 0xe0e007f8,
	0xe0e00ff0, 0x1b80001f, 0x20000020, 0xe0f07ff0, 0xe0f07f00, 0xf0000000,
	0x17c07c1f, 0xa1d08407, 0xa1d18407, 0x1b80001f, 0x20000080, 0x812ab401,
	0x80ebb401, 0xa0c00c04, 0x1a00001f, 0x10006814, 0xe2000003, 0xf0000000,
	0x17c07c1f, 0xa1d10407, 0x1b80001f, 0x20000020, 0xf0000000, 0x17c07c1f,
	0xd8000d4a, 0x17c07c1f, 0xe2e0006d, 0xe2e0002d, 0xd8200dea, 0x17c07c1f,
	0xe2e0002f, 0xe2e0003e, 0xe2e00032, 0xf0000000, 0x17c07c1f, 0xd8000f2a,
	0x17c07c1f, 0xe2e00036, 0x1380201f, 0xe2e0003e, 0x1380201f, 0xe2e0002e,
	0x1380201f, 0xd820102a, 0x17c07c1f, 0xe2e0006e, 0xe2e0004e, 0xe2e0004c,
	0x1b80001f, 0x20000020, 0xe2e0004d, 0xf0000000, 0x17c07c1f, 0xa1d00407,
	0x1b80001f, 0x20000100, 0x80ea3401, 0x1a00001f, 0x10006814, 0xe2000003,
	0xf0000000, 0x17c07c1f, 0xe0e03101, 0xe0e03201, 0xe0e03603, 0xe0e03602,
	0xe0e03606, 0xf0000000, 0x17c07c1f, 0xd800134a, 0x17c07c1f, 0xe0e03602,
	0xe0e03603, 0xe0e03601, 0xd0001380, 0x17c07c1f, 0xe0e03201, 0xe0e03001,
	0xf0000000, 0x17c07c1f, 0x18c0001f, 0x10006b6c, 0x1910001f, 0x10006b6c,
	0xa1002804, 0xe0c00004, 0xf0000000, 0x17c07c1f, 0xd82015c9, 0x17c07c1f,
	0xe2e0000d, 0xe2e0000c, 0xe2e0001c, 0xe2e0001e, 0xe2e00016, 0xe2e00012,
	0xf0000000, 0x17c07c1f, 0xd8201749, 0x17c07c1f, 0xe2e00016, 0x1380201f,
	0xe2e0001e, 0x1380201f, 0xe2e0001c, 0x1380201f, 0xe2e0000c, 0xe2e0000d,
	0xf0000000, 0x17c07c1f, 0xa1d40407, 0x1391841f, 0xa1d90407, 0xf0000000,
	0x17c07c1f, 0x18d0001f, 0x10006604, 0x10cf8c1f, 0xd8201823, 0x17c07c1f,
	0xf0000000, 0x17c07c1f, 0xe8208000, 0x11008014, 0x00000002, 0xe8208000,
	0x11008020, 0x00000101, 0xe8208000, 0x11008004, 0x000000d0, 0x1a00001f,
	0x11008000, 0xd8001aea, 0xe220005d, 0xd8201b0a, 0xe2200000, 0xe2200001,
	0xe8208000, 0x11008024, 0x00000001, 0x1b80001f, 0x20000424, 0xf0000000,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001, 0xa1d48407, 0x1990001f,
	0x10006b08, 0x1a50001f, 0x10006610, 0x8246a401, 0xe8208000, 0x10006b6c,
	0x00000000, 0x1b00001f, 0x2f7be75f, 0x1b80001f, 0xd00f0000, 0x8880000c,
	0x2f7be75f, 0xd80068c2, 0x17c07c1f, 0xe8208000, 0x10006354, 0xfffe7b47,
	0xc0c00c20, 0x81401801, 0xd8004a25, 0x17c07c1f, 0x81f60407, 0x18c0001f,
	0x100062a0, 0xc0c00cc0, 0x12807c1f, 0x18c0001f, 0x100062b4, 0x1910001f,
	0x100062b4, 0xa9000004, 0x00000001, 0xe0c00004, 0xa9000004, 0x00000011,
	0xe0c00004, 0x18c0001f, 0x100062a0, 0xc0c00cc0, 0x1280041f, 0x18c0001f,
	0x100062b0, 0xc0c00cc0, 0x12807c1f, 0x81451801, 0xd80047c5, 0x17c07c1f,
	0xe8208000, 0x100062b8, 0x00000011, 0x1b80001f, 0x20000080, 0xe8208000,
	0x100062b8, 0x00000015, 0xd00048c0, 0x17c07c1f, 0xe8208000, 0x100062b8,
	0x00000000, 0x1b80001f, 0x20000080, 0xe8208000, 0x100062b8, 0x00000040,
	0xc0c00cc0, 0x1280041f, 0x18c0001f, 0x10006290, 0xc0c00cc0, 0x1280041f,
	0xe8208000, 0x10006404, 0x00003101, 0xc28013c0, 0x1292041f, 0xc0c01780,
	0x17c07c1f, 0x81449801, 0xd8004d85, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0x81451801, 0xd8004c45, 0x17c07c1f, 0xe2200001, 0xc0c01900, 0x12807c1f,
	0xc0c01820, 0x17c07c1f, 0xd0004d80, 0x17c07c1f, 0xe2200001, 0x18c0001f,
	0x10005460, 0x1910001f, 0x10005460, 0x89000004, 0xfffffffb, 0xe0c00004,
	0xc0c01820, 0x17c07c1f, 0x81441801, 0xd8004f65, 0x17c07c1f, 0xa1d38407,
	0xa1d98407, 0x1800001f, 0x00000012, 0x1800001f, 0x00000e12, 0x1800001f,
	0x03800e12, 0x1800001f, 0x038e0e12, 0xd0005080, 0x17c07c1f, 0xa1d38407,
	0x1800001f, 0x00000012, 0x1800001f, 0x00000a12, 0x1800001f, 0x02800a12,
	0x1800001f, 0x028a0a12, 0x18c0001f, 0x10209200, 0x1910001f, 0x10209200,
	0x81200404, 0xe0c00004, 0x18c0001f, 0x1020920c, 0x1910001f, 0x1020920c,
	0xa1108404, 0xe0c00004, 0x81200404, 0xe0c00004, 0xe8208000, 0x10006310,
	0x0b1600c8, 0x1880001f, 0x20000015, 0x18c0001f, 0x010d0ba4, 0x1900001f,
	0xbfffe7ff, 0x1940001f, 0x001d0bae, 0x1a10001f, 0x10006b6c, 0x1980001f,
	0x7ffff7ff, 0x12807c1f, 0x1b00001f, 0xbfffe7ff, 0x1b80001f, 0x90100000,
	0x82810001, 0xca80008a, 0x17c07c1f, 0x18c0001f, 0x10006b6c, 0xe0c00008,
	0x1990001f, 0x10006b08, 0x81449801, 0xd8005925, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0x81451801, 0xd80057a5, 0x17c07c1f, 0xe2200000, 0xc0c01900,
	0x1280041f, 0xc0c01820, 0x17c07c1f, 0xd00058e0, 0x17c07c1f, 0xe2200000,
	0x18c0001f, 0x10005460, 0x1910001f, 0x10005460, 0xa9000004, 0x00000004,
	0xe0c00004, 0xc0c01820, 0x17c07c1f, 0x1b80001f, 0x200016a8, 0x81441801,
	0xd8005cc5, 0x17c07c1f, 0x1800001f, 0x03800e12, 0x18c0001f, 0x1020920c,
	0x1910001f, 0x1020920c, 0xa1000404, 0xe0c00004, 0x1b80001f, 0x20000300,
	0x1800001f, 0x00000e12, 0x81308404, 0xe0c00004, 0x18c0001f, 0x10209200,
	0x1910001f, 0x10209200, 0xa1000404, 0xe0c00004, 0x1b80001f, 0x20000300,
	0x1800001f, 0x00000012, 0xd0005fc0, 0x17c07c1f, 0x1800001f, 0x02800a12,
	0x18c0001f, 0x1020920c, 0x1910001f, 0x1020920c, 0xa1000404, 0xe0c00004,
	0x1b80001f, 0x20000300, 0x1800001f, 0x00000a12, 0x81308404, 0xe0c00004,
	0x18c0001f, 0x10209200, 0x1910001f, 0x10209200, 0xa1000404, 0xe0c00004,
	0x1b80001f, 0x20000300, 0x1800001f, 0x00000012, 0x1b80001f, 0x20000104,
	0x10007c1f, 0x81f38407, 0x81f98407, 0x81f90407, 0x81f40407, 0x81401801,
	0xd80068c5, 0x17c07c1f, 0xe8208000, 0x10006404, 0x00001101, 0x18c0001f,
	0x10006290, 0x1212841f, 0xc0c00e20, 0x12807c1f, 0xc0c00e20, 0x1280041f,
	0x18c0001f, 0x100062b0, 0x1212841f, 0xc0c00e20, 0x12807c1f, 0x81451801,
	0xd8006485, 0x17c07c1f, 0xe8208000, 0x100062b8, 0x00000011, 0xe8208000,
	0x100062b8, 0x00000010, 0x1b80001f, 0x20000080, 0xd0006580, 0x17c07c1f,
	0xe8208000, 0x100062b8, 0x00000000, 0xe8208000, 0x100062b8, 0x00000010,
	0x1b80001f, 0x20000080, 0xc0c00e20, 0x1280041f, 0xe8208000, 0x10200268,
	0x000ffffe, 0x18c0001f, 0x100062a0, 0x1212841f, 0xc0c00e20, 0x12807c1f,
	0x18c0001f, 0x100062b4, 0x1910001f, 0x100062b4, 0x89000004, 0xffffffef,
	0xe0c00004, 0x89000004, 0xffffffee, 0xe0c00004, 0x1b80001f, 0x20000a50,
	0x18c0001f, 0x100062a0, 0xc0c00e20, 0x1280041f, 0x19c0001f, 0x01411820,
	0x1ac0001f, 0x55aa55aa, 0x10007c1f, 0xf0000000
};

static struct pcm_desc talking_pcm = {
	.version = "pcm_talk_idle_v5.5_2014_0207_big",
	.base = talking_binary,
	.size = 844,
	.sess = 2,
	.replace = 0,
	.vec0 = EVENT_VEC(11, 1, 0, 0),	/* FUNC_26M_WAKEUP */
	.vec1 = EVENT_VEC(12, 1, 0, 2),	/* FUNC_26M_SLEEP */
	.vec2 = EVENT_VEC(30, 1, 0, 4),	/* FUNC_APSRC_WAKEUP */
	.vec3 = EVENT_VEC(31, 1, 0, 16),	/* FUNC_APSRC_SLEEP */
};

#else				/* !SPM_CTRL_BIG_CPU */
static const u32 talking_binary[] = {
	0xf0000000, 0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x803d8400, 0x1380081f,
	0x80340400, 0x17c07c1f, 0x17c07c1f, 0x80310400, 0x11c00c1f, 0x1b80001f,
	0x200000b6, 0xa2110408, 0xf0000000, 0x1300101f, 0x1b00001f, 0x3fffe7ff,
	0x1b80001f, 0x20000004, 0xd820034c, 0x17c07c1f, 0x1980001f, 0x3fffe7ff,
	0xd0000440, 0x17c07c1f, 0x11c0141f, 0x1380081f, 0x82863401, 0xd800014a,
	0x1300181f, 0xa0110400, 0xa0140400, 0xa01d8400, 0xf0000000, 0xa2118408,
	0x1900001f, 0x10006830, 0xe1000003, 0xe8208000, 0x10006834, 0x00000001,
	0x18d0001f, 0x10006830, 0x68e00003, 0x0000beef, 0xd8200543, 0x17c07c1f,
	0xf0000000, 0x17c07c1f, 0xe0f07f16, 0x1380201f, 0xe0f07f1e, 0x1380201f,
	0xe0f07f0e, 0x1b80001f, 0x20000104, 0xe0f07f0c, 0xe0f07f0d, 0xe0f07e0d,
	0xe0f07c0d, 0xe0f0780d, 0xe0f0700d, 0xe0f0600d, 0xe0f0400d, 0xe0f0000d,
	0xe0e0000d, 0xf0000000, 0x17c07c1f, 0xe0f07f0d, 0xe0f07f0f, 0xe0f07f1e,
	0xe0f07f12, 0xf0000000, 0x17c07c1f, 0xe0e001fe, 0xe0e003fc, 0xe0e007f8,
	0xe0e00ff0, 0x1b80001f, 0x20000020, 0xe0f07ff0, 0xe0f07f00, 0xf0000000,
	0x17c07c1f, 0xa1d08407, 0xa1d18407, 0x1b80001f, 0x20000080, 0x812ab401,
	0x80ebb401, 0xa0c00c04, 0x1a00001f, 0x10006814, 0xe2000003, 0xf0000000,
	0x17c07c1f, 0xa1d10407, 0x1b80001f, 0x20000020, 0xf0000000, 0x17c07c1f,
	0xd8000d4a, 0x17c07c1f, 0xe2e0006d, 0xe2e0002d, 0xd8200dea, 0x17c07c1f,
	0xe2e0002f, 0xe2e0003e, 0xe2e00032, 0xf0000000, 0x17c07c1f, 0xd8000f2a,
	0x17c07c1f, 0xe2e00036, 0x1380201f, 0xe2e0003e, 0x1380201f, 0xe2e0002e,
	0x1380201f, 0xd820102a, 0x17c07c1f, 0xe2e0006e, 0xe2e0004e, 0xe2e0004c,
	0x1b80001f, 0x20000020, 0xe2e0004d, 0xf0000000, 0x17c07c1f, 0xa1d00407,
	0x1b80001f, 0x20000100, 0x80ea3401, 0x1a00001f, 0x10006814, 0xe2000003,
	0xf0000000, 0x17c07c1f, 0xe0e03101, 0xe0e03201, 0xe0e03603, 0xe0e03602,
	0xe0e03606, 0xf0000000, 0x17c07c1f, 0xd800134a, 0x17c07c1f, 0xe0e03602,
	0xe0e03603, 0xe0e03601, 0xd0001380, 0x17c07c1f, 0xe0e03201, 0xe0e03001,
	0xf0000000, 0x17c07c1f, 0x18c0001f, 0x10006b6c, 0x1910001f, 0x10006b6c,
	0xa1002804, 0xe0c00004, 0xf0000000, 0x17c07c1f, 0xd82015c9, 0x17c07c1f,
	0xe2e0000d, 0xe2e0000c, 0xe2e0001c, 0xe2e0001e, 0xe2e00016, 0xe2e00012,
	0xf0000000, 0x17c07c1f, 0xd8201749, 0x17c07c1f, 0xe2e00016, 0x1380201f,
	0xe2e0001e, 0x1380201f, 0xe2e0001c, 0x1380201f, 0xe2e0000c, 0xe2e0000d,
	0xf0000000, 0x17c07c1f, 0xa1d40407, 0x1391841f, 0xa1d90407, 0xf0000000,
	0x17c07c1f, 0x18d0001f, 0x10006604, 0x10cf8c1f, 0xd8201823, 0x17c07c1f,
	0xf0000000, 0x17c07c1f, 0xe8208000, 0x11008014, 0x00000002, 0xe8208000,
	0x11008020, 0x00000101, 0xe8208000, 0x11008004, 0x000000d0, 0x1a00001f,
	0x11008000, 0xd8001aea, 0xe220005d, 0xd8201b0a, 0xe2200000, 0xe2200001,
	0xe8208000, 0x11008024, 0x00000001, 0x1b80001f, 0x20000424, 0xf0000000,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001, 0xa1d48407, 0x1990001f,
	0x10006b08, 0x1a50001f, 0x10006610, 0x8246a401, 0xe8208000, 0x10006b6c,
	0x00000000, 0x1b00001f, 0x2f7be75f, 0x1b80001f, 0xd00f0000, 0x8880000c,
	0x2f7be75f, 0xd80061c2, 0x17c07c1f, 0xe8208000, 0x10006354, 0xfffe7b47,
	0xc0c00c20, 0x81401801, 0xd8004a25, 0x17c07c1f, 0x81f60407, 0x18c0001f,
	0x10006200, 0xc0c00cc0, 0x12807c1f, 0xe8208000, 0x1000625c, 0x00000001,
	0x1890001f, 0x1000625c, 0x81040801, 0xd8204444, 0x17c07c1f, 0xc0c00cc0,
	0x1280041f, 0x18c0001f, 0x10006204, 0xc0c014c0, 0x1280041f, 0x18c0001f,
	0x10006208, 0xc0c00cc0, 0x12807c1f, 0x81451801, 0xd80047c5, 0x17c07c1f,
	0xe8208000, 0x10006244, 0x00000001, 0x1890001f, 0x10006244, 0x81040801,
	0xd82046e4, 0x17c07c1f, 0xd00048c0, 0x17c07c1f, 0xe8208000, 0x10006248,
	0x00000000, 0x1890001f, 0x10006248, 0x81040801, 0xd8004824, 0x17c07c1f,
	0xc0c00cc0, 0x1280041f, 0x18c0001f, 0x10006290, 0xc0c00cc0, 0x1280041f,
	0xe8208000, 0x10006404, 0x00003101, 0xc28013c0, 0x1292041f, 0xc0c01780,
	0x17c07c1f, 0x81449801, 0xd8004cc5, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0x81451801, 0xd8004c65, 0x17c07c1f, 0xe2200003, 0xc0c01820, 0x17c07c1f,
	0xe2200005, 0xc0c01820, 0x17c07c1f, 0xd0004cc0, 0x17c07c1f, 0xe2200003,
	0xc0c01820, 0x17c07c1f, 0x81441801, 0xd8004ea5, 0x17c07c1f, 0xa1d38407,
	0xa1d98407, 0x1800001f, 0x00000012, 0x1800001f, 0x00000e12, 0x1800001f,
	0x03800e12, 0x1800001f, 0x038e0e12, 0xd0004fc0, 0x17c07c1f, 0xa1d38407,
	0x1800001f, 0x00000012, 0x1800001f, 0x00000a12, 0x1800001f, 0x02800a12,
	0x1800001f, 0x028a0a12, 0xe8208000, 0x10006310, 0x0b1600c8, 0x1880001f,
	0x20000015, 0x18c0001f, 0x010d0ba4, 0x1900001f, 0xbfffe7ff, 0x1940001f,
	0x001d0bae, 0x1a10001f, 0x10006b6c, 0x1980001f, 0x7ffff7ff, 0x12807c1f,
	0x1b00001f, 0xbfffe7ff, 0x1b80001f, 0x90100000, 0x82810001, 0xca80008a,
	0x17c07c1f, 0x18c0001f, 0x10006b6c, 0xe0c00008, 0x1990001f, 0x10006b08,
	0x81449801, 0xd80055e5, 0x17c07c1f, 0x1a00001f, 0x10006604, 0x81451801,
	0xd8005545, 0x17c07c1f, 0xe2200002, 0xc0c01820, 0x17c07c1f, 0xe2200004,
	0xc0c01820, 0x17c07c1f, 0xd00055a0, 0x17c07c1f, 0xe2200002, 0xc0c01820,
	0x17c07c1f, 0x1b80001f, 0x200016a8, 0x81441801, 0xd80057c5, 0x17c07c1f,
	0x1800001f, 0x03800e12, 0x1b80001f, 0x20000300, 0x1800001f, 0x00000e12,
	0x1b80001f, 0x20000300, 0x1800001f, 0x00000012, 0xd0005900, 0x17c07c1f,
	0x1800001f, 0x02800a12, 0x1b80001f, 0x20000300, 0x1800001f, 0x00000a12,
	0x1b80001f, 0x20000300, 0x1800001f, 0x00000012, 0x1b80001f, 0x20000104,
	0x10007c1f, 0x81f38407, 0x81f98407, 0x81f90407, 0x81f40407, 0x81401801,
	0xd80061c5, 0x17c07c1f, 0xe8208000, 0x10006404, 0x00002101, 0x18c0001f,
	0x10006290, 0x1212841f, 0xc0c00e20, 0x12807c1f, 0xc0c00e20, 0x1280041f,
	0x18c0001f, 0x10006208, 0x1212841f, 0xc0c00e20, 0x12807c1f, 0x81451801,
	0xd8005dc5, 0x17c07c1f, 0xe8208000, 0x10006244, 0x00000000, 0x1890001f,
	0x10006244, 0x81040801, 0xd8005ce4, 0x17c07c1f, 0xd0005f00, 0x17c07c1f,
	0xe8208000, 0x10006248, 0x00000001, 0x1890001f, 0x10006248, 0x81040801,
	0xd8205e24, 0x17c07c1f, 0x1b80001f, 0x20000020, 0xc0c00e20, 0x1280041f,
	0x18c0001f, 0x10006204, 0x1212841f, 0xc0c01600, 0x1280041f, 0x18c0001f,
	0x10006200, 0x1212841f, 0xc0c00e20, 0x12807c1f, 0xe8208000, 0x1000625c,
	0x00000000, 0x1890001f, 0x1000625c, 0x81040801, 0xd80060e4, 0x17c07c1f,
	0xc0c00e20, 0x1280041f, 0x19c0001f, 0x01411820, 0x1ac0001f, 0x55aa55aa,
	0x10007c1f, 0xf0000000
};

static struct pcm_desc talking_pcm = {
	.version = "pcm_talk_idle_v5.5_2014_0207",
	.base = talking_binary,
	.size = 788,
	.sess = 2,
	.replace = 0,
	.vec0 = EVENT_VEC(11, 1, 0, 0),	/* FUNC_26M_WAKEUP */
	.vec1 = EVENT_VEC(12, 1, 0, 2),	/* FUNC_26M_SLEEP */
	.vec2 = EVENT_VEC(30, 1, 0, 4),	/* FUNC_APSRC_WAKEUP */
	.vec3 = EVENT_VEC(31, 1, 0, 16),	/* FUNC_APSRC_SLEEP */
};
#endif

static struct pwr_ctrl talking_ctrl = {
	/* wake_src             = suspend/dpidle_ctrl.wake_src */
	/* wake_src_md32        = suspend/dpidle_ctrl.wake_src_md32 */
	.r0_ctrl_en = 1,
	.r7_ctrl_en = 1,
	.infra_dcm_lock = 1,
	.wfi_op = WFI_OP_AND,
	.ca15_wfi0_en = 1,
	.ca15_wfi1_en = 1,
	.ca15_wfi2_en = 1,
	.ca15_wfi3_en = 1,
	.ca7_wfi0_en = 1,
	.ca7_wfi1_en = 1,
	.ca7_wfi2_en = 1,
	.ca7_wfi3_en = 1,
	.md2_req_mask = 1,
	.md_apsrc_sel = SEL_MD_DDR_EN,
	.disp_req_mask = 1,
	.mfg_req_mask = 1,
};

struct spm_lp_scen __spm_talking = {
	.pcmdesc = &talking_pcm,
	.pwrctrl = &talking_ctrl,
};


/**************************************
 * Function and API
 **************************************/
static bool is_in_talking(void)
{
	return false;		/*get_voice_status(); */
}

/*
 * if in talking, modify @spm_flags based on @lpscen and return __spm_talking,
 * otherwise, do nothing and return @lpscen
 */
struct spm_lp_scen *spm_check_talking_get_lpscen(struct spm_lp_scen *lpscen, u32 *spm_flags)
{
	if (check_talking && is_in_talking()) {
		if (lpscen == &__spm_suspend) {
			*spm_flags &= ~SPM_CPU_DORMANT;
			spm_crit2("in talking\n");
		} else {
			*spm_flags |= SPM_CPU_DORMANT;
			spm_info("in talking\n");
		}

		set_flags_for_mainpll(spm_flags);	/* ANC needs MAINPLL on */

		__spm_talking.pwrctrl->wake_src = lpscen->pwrctrl->wake_src;
		__spm_talking.pwrctrl->wake_src_md32 = lpscen->pwrctrl->wake_src_md32;

		lpscen = &__spm_talking;
	}

	return lpscen;
}

#if SPM_USE_TWAM_DEBUG
static void twam_handler(struct twam_sig *twamsig)
{
	spm_crit("sig_high = %u%%  %u%%  %u%%  %u%%, r13 = 0x%x\n",
		 get_high_percent(twamsig->sig0),
		 get_high_percent(twamsig->sig1),
		 get_high_percent(twamsig->sig2),
		 get_high_percent(twamsig->sig3), spm_read(SPM_PCM_REG13_DATA));
}
#endif

static int spm_talking_init(void)
{
#if SPM_USE_TWAM_DEBUG
	unsigned long flags;
	struct twam_sig twamsig = {
		.sig0 = 28,	/* md1_apsrc_req or md_ddr_en */
		.sig1 = 27,	/* md1_srclkena */
		.sig2 = 29,	/* md2_srclkena */
		.sig3 = 26,	/* md32_apsrc_req */
	};

	spin_lock_irqsave(&__spm_lock, flags);
	spm_write(SPM_AP_STANBY_CON, spm_read(SPM_AP_STANBY_CON) | ASC_MD_DDR_EN_SEL);
	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_twam_register_handler(twam_handler);
	spm_twam_enable_monitor(&twamsig, false);
#endif

#if SPM_USE_GPIO_DEBUG
	spm_write(0xf0006b20, 0x1);
	spm_write(0xf0000230, (spm_read(0xf0000230) & ~(0x7fff << 16)) |
		  (0x3 << 26) | (0x3 << 21) | (0x3 << 16));

	/* output md_ddr_en */
	spm_write(0xf00057e0, spm_read(0xf00057e0) | (0x7 << 0));
	spm_write(0xf0001500, 0x70e);

	/* output emi_clk_off_req */
	spm_write(0xf00057c0, spm_read(0xf00057c0) | (0x7 << 0));
	spm_write(0xf000150c, 0x3fe);
#endif

	return 0;
}
late_initcall(spm_talking_init);

module_param(check_talking, bool, 0644);

MODULE_DESCRIPTION("SPM-Talking Driver v0.1");

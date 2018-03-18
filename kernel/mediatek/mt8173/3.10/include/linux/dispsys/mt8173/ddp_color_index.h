
#include "ddp_drv.h"

static DISPLAY_PQ_T pqindex = {
 GLOBAL_SAT:
	{0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c, 0xa0, 0xa5},	/* 0~9 */

 PARTIAL_Y :
	{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	 0x80},

 PURP_TONE_S :
	{			/* hue 0~10 */
	 {			/* 0 disable */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 1 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 2 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 {			/* 3 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 4 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 5 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 6 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 7 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 8 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 9 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 10 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 11 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 12 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 13 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 14 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 15 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 16 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 17 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 /* 18 */
	 {
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  }
	 },
 SKIN_TONE_S :
	{
	 {			/* 0 disable */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 1 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 2 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 3 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 4 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 5 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },
	 {			/* 6 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 7 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 8 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 9 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 10 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 11 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 12 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 13 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 14 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 15 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 16 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 17 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 18 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  }
	 },
 GRASS_TONE_S :
	{
	 {			/* 0 disable */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 1 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 2 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

	  },

	 {			/* 3 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

	  },

	 {			/* 4 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

	  },

	 {			/* 5 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

	  },

	 {			/* 6 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

	  },

	 {			/* 7 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 8 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 9 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 10 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 11 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 12 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 13 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 14 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 15 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 16 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 17 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  },

	 {			/* 18 */
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	  }
	 },
 SKY_TONE_S :
	{
	 {			/* 0 disable */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },
	 {			/* 1 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}

	  },
	 {			/* 2 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}

	  },

	 {			/* 3 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}

	  },
	 {			/* 4 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}

	  },

	 {			/* 5 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 6 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 7 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 8 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 9 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 10 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 11 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 12 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 13 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 14 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 15 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 16 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 17 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  },

	 {			/* 18 */
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80},
	  {0x80, 0x80, 0x80}
	  }
	 },

 PURP_TONE_H :
	{
/* hue 0~2 */
	 {0x80, 0x80, 0x80},	/* 0 */
	 {0x80, 0x80, 0x80},	/* 1 */
	 {0x80, 0x80, 0x80},	/* 2 */
	 {0x80, 0x80, 0x80},	/* 3 */
	 {0x80, 0x80, 0x80},	/* 4 */
	 {0x80, 0x80, 0x80},	/* 5 */
	 {0x80, 0x80, 0x80},	/* 6 */
	 {0x80, 0x80, 0x80},	/* 7 */
	 {0x80, 0x80, 0x80},	/* 8 */
	 {0x80, 0x80, 0x80},	/* 9 */
	 {0x80, 0x80, 0x80},	/* 10 */
	 {0x80, 0x80, 0x80},	/* 11 */
	 {0x80, 0x80, 0x80},	/* 12 */
	 {0x80, 0x80, 0x80},	/* 13 */
	 {0x80, 0x80, 0x80},	/* 14 */
	 {0x80, 0x80, 0x80},	/* 15 */
	 {0x80, 0x80, 0x80},	/* 16 */
	 {0x80, 0x80, 0x80},	/* 17 */
	 {0x80, 0x80, 0x80}	/* 18 */
	 },

 SKIN_TONE_H :
	{
/* hue 3~10 */
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	 },

 GRASS_TONE_H :
	{
/* hue 11~16 */
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	 },

 SKY_TONE_H :
	{
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80},
	 {0x80, 0x80, 0x80}
	 }

};
#ifndef MT_SPOWER_CPU_H
#define MT_SPOWER_CPU_H



#define VSIZE 9
#define TSIZE 20
#define MAX_TABLE_SIZE 3

/***************************/
/* "(WAT 14.35%)	   */
/* Leakage Power"	   */
/***************************/
#define CA7_TABLE_0							\
	/*      */    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     36,	46,	59,	76,	98,	127,	162,	178,	194, \
		30,     41,	52,	67,	86,	112,	146,	187,	206,	227, \
		35,     46,	59,	76,	98,	128,	166,	213,	239,	266, \
		40,     52,	66,	86,	112,	146,	188,	243,	275,	309, \
		45,     59,	75,	97,	127,	165,	213,	278,	316,	361, \
		50,     67,	85,	110,	143,	187,	241,	316,	363,	420, \
		55,     75,	96,	125,	162,	210,	272,	359,	417,	487, \
		60,     85,	110,	141,	182,	237,	309,	408,	479,	567, \
		65,     97,	124,	160,	207,	268,	352,	466,	553,	667, \
		70,     110,	141,	181,	235,	304,	400,	532,	638,	778, \
		75,     125,	160,	206,	266,	345,	455,	611,	737,	912, \
		80,     141,	181,	232,	301,	392,	520,	699,	848,	1067, \
		85,     160,	204,	261,	341,	447,	591,	798,	979,	1247, \
		90,     181,	232,	298,	388,	507,	671,	911,	1129,	1458, \
		95,     206,	264,	339,	440,	575,	767,	1046,	1305,	1725, \
		100,    234,	298,	384,	497,	651,	871,	1195,	1507,	2020, \
		105,    266,	338,	435,	564,	739,	989,	1370,	1743,	2363, \
		110,    301,	382,	495,	641,	841,	1123,	1559,	2016,	2744, \
		115,    342,	431,	557,	727,	955,	1288,	1787,	2318,	3202, \
		120,    386,	490,	630,	820,	1080,	1463,	2043,	2681,	3732,

	/******************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,   1050,   1100,   1150, \           */
	/*      25,     20,     25,     31,     39,     48,     60,     74,     90, \             */
	/*      30,     29,     37,     46,     58,     71,     88,     109,    133, \            */
	/*      35,     43,     54,     68,     85,     104,    130,    160,    196, \            */
	/*      40,     63,     80,     101,    125,    154,    192,    236,    289, \            */
	/*      45,     93,     118,    149,    185,    227,    283,    349,    426, \            */
	/*      50,     101,    129,    162,    201,    247,    308,    380,    464, \            */
	/*      55,     110,    140,    177,    219,    269,    335,    413,    505, \            */
	/*      60,     120,    153,    192,    239,    293,    365,    450,    550, \            */
	/*      65,     131,    167,    209,    260,    319,    397,    490,    598, \            */
	/*      70,     144,    184,    231,    286,    352,    438,    540,    660, \            */
	/*      75,     159,    202,    254,    316,    388,    483,    595,    727, \            */
	/*      80,     175,    223,    280,    348,    427,    532,    656,    801, \            */
	/*      85,     193,    246,    309,    384,    471,    587,    723,    883, \            */
	/*      90,     203,    259,    326,    404,    497,    618,    762,    931, \            */
	/*      95,     214,    273,    343,    426,    523,    652,    804,    982, \            */
	/*      100,    226,    288,    362,    450,    552,    687,    847,    1035, \           */
	/*      105,    238,    304,    382,    474,    582,    725,    893,    1091,             */
	/******************************************************************************************/

/******************/
/* "(WAT 2.65%)	  */
/* Leakage Power" */
/******************/
#define CA7_TABLE_1							\
	/*      */    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     11,	13,	16,	20,	25,	30,	39,	43,	48 , \
		30,     12,	15,	19,	23,	28,	35,	44,	49,	55 , \
		35,     14,	17,	21,	26,	32,	39,	49,	56,	62 , \
		40,     16,	20,	24,	30,	36,	45,	56,	63,	70 , \
		45,     18,	22,	27,	34,	41,	51,	64,	71,	80 , \
		50,     20,	25,	31,	38,	46,	58,	72,	81,	92 , \
		55,     23,	29,	35,	43,	53,	66,	82,	93,	104 , \
		60,     26,	32,	40,	49,	60,	75,	94,	105,	118 , \
		65,     30,	37,	45,	55,	68,	85,	106,	118,	133 , \
		70,     33,	41,	51,	63,	77,	95,	120,	134,	151 , \
		75,     38,	47,	58,	71,	88,	108,	136,	153,	171 , \
		80,     43,	54,	65,	81,	99,	122,	156,	173,	194 , \
		85,     48,	61,	74,	91,	112,	139,	176,	197,	220 , \
		90,     54,	68,	84,	103,	127,	158,	201,	224,	253 , \
		95,     62,	77,	95,	116,	146,	184,	228,	255,	288 , \
		100,    69,	87,	106,	132,	166,	210,	257,	286,	327 , \
		105,    77,	98,	119,	149,	189,	236,	290,	326,	371 , \
		110,    87,	111,	135,	168,	213,	266,	326,	372,	421 , \
		115,    99,	125,	154,	190,	240,	301,	373,	424,	478 , \
		120,    112,	140,	173,	213,	271,	337,	422,	480,	542 ,

	/*****************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,   1050,   1100,   1150, \          */
	/*      25,     8,      10,     13,     16,     20,     25,     30,     37, \            */
	/*      30,     10,     12,     16,     19,     24,     29,     36,     44, \            */
	/*      35,     12,     15,     19,     23,     28,     35,     44,     53, \            */
	/*      40,     14,     18,     22,     28,     34,     43,     52,     64, \            */
	/*      45,     17,     21,     27,     33,     41,     51,     63,     77, \            */
	/*      50,     20,     25,     32,     40,     49,     61,     75,     92, \            */
	/*      55,     24,     30,     38,     47,     58,     72,     89,     109, \           */
	/*      60,     28,     36,     45,     56,     69,     86,     106,    130, \           */
	/*      65,     34,     43,     54,     67,     82,     103,    127,    155, \           */
	/*      70,     39,     50,     63,     79,     96,     120,    148,    181, \           */
	/*      75,     46,     59,     74,     92,     113,    141,    173,    212, \           */
	/*      80,     54,     69,     87,     108,    132,    164,    203,    248, \           */
	/*      85,     63,     81,     101,    126,    155,    192,    237,    290, \           */
	/*      90,     73,     93,     117,    146,    179,    223,    275,    336, \           */
	/*      95,     85,     108,    136,    169,    207,    258,    318,    389, \           */
	/*      100,    98,     125,    158,    196,    240,    299,    369,    451, \           */
	/*      105,    114,    145,    183,    227,    278,    347,    428,    522,             */
	/*****************************************************************************************/

#define CA7_TABLE_2							\
	/**/    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     10,	12,	14,	17,	20,	25,	30,	33,	38 , \
		30,     11,	13,	16,	19,	22,	27,	33,	37,	42 , \
		35,     12,	14,	17,	21,	24,	30,	37,	40,	46 , \
		40,     13,	16,	19,	22,	27,	33,	40,	45,	50 , \
		45,     14,	17,	21,	25,	29,	36,	45,	49,	56 , \
		50,     16,	19,	23,	27,	32,	40,	49,	54,	61 , \
		55,     17,	21,	25,	30,	36,	44,	54,	60,	67 , \
		60,     19,	23,	27,	33,	40,	48,	60,	66,	74 , \
		65,     21,	25,	30,	36,	44,	53,	66,	74,	82 , \
		70,     23,	27,	33,	40,	49,	59,	73,	82,	90 , \
		75,     25,	30,	36,	44,	54,	65,	80,	90,	100 , \
		80,     28,	33,	40,	49,	59,	72,	89,	98,	110 , \
		85,     30,	37,	44,	54,	65,	80,	98,	109,	121 , \
		90,     33,	41,	48,	59,	72,	88,	109,	120,	134 , \
		95,     37,	44,	53,	64,	80,	96,	120,	132,	147 , \
		100,    40,	48,	57,	71,	88,	106,	132,	146,	163 , \
		105,    43,	53,	63,	78,	97,	116,	145,	161,	179 , \
		110,    47,	58,	69,	86,	106,	129,	159,	177,	196 , \
		115,    51,	63,	75,	94,	117,	142,	175,	195,	215 , \
		120,    56,	69,	82,	103,	130,	156,	193,	214,	236 ,

	/*****************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,   1050,   1100,   1150, \          */
	/*      25,     8,      10,     13,     16,     20,     25,     30,     37, \            */
	/*      30,     10,     12,     16,     19,     24,     29,     36,     44, \            */
	/*      35,     12,     15,     19,     23,     28,     35,     44,     53, \            */
	/*      40,     14,     18,     22,     28,     34,     43,     52,     64, \            */
	/*      45,     17,     21,     27,     33,     41,     51,     63,     77, \            */
	/*      50,     20,     25,     32,     40,     49,     61,     75,     92, \            */
	/*      55,     24,     30,     38,     47,     58,     72,     89,     109, \           */
	/*      60,     28,     36,     45,     56,     69,     86,     106,    130, \           */
	/*      65,     34,     43,     54,     67,     82,     103,    127,    155, \           */
	/*      70,     39,     50,     63,     79,     96,     120,    148,    181, \           */
	/*      75,     46,     59,     74,     92,     113,    141,    173,    212, \           */
	/*      80,     54,     69,     87,     108,    132,    164,    203,    248, \           */
	/*      85,     63,     81,     101,    126,    155,    192,    237,    290, \           */
	/*      90,     73,     93,     117,    146,    179,    223,    275,    336, \           */
	/*      95,     85,     108,    136,    169,    207,    258,    318,    389, \           */
	/*      100,    98,     125,    158,    196,    240,    299,    369,    451, \           */
	/*      105,    114,    145,    183,    227,    278,    347,    428,    522,             */
	/*****************************************************************************************/


/***************************/
/* "(WAT 14.35%)	   */
/* Leakage Power"	   */
/***************************/
#define CA15L_TABLE_0							\
	/**/    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     129,	168,	220,	282,	357,	444,	533,	553,	559 , \
		30,     149,	193,	253,	325,	415,	525,	634,	673,	705 , \
		35,     173,	222,	291,	375,	483,	610,	762,	823,	885 , \
		40,     198,	257,	335,	432,	560,	725,	913,	1002,	1110 , \
		45,     229,	295,	385,	498,	649,	844,	1092,	1234,	1408 , \
		50,     263,	339,	443,	576,	750,	988,	1301,	1514,	1773 , \
		55,     302,	390,	509,	662,	868,	1151,	1541,	1845,	2232 , \
		60,     348,	449,	585,	761,	1005,	1343,	1837,	2264,	2841 , \
		65,     399,	516,	670,	877,	1160,	1577,	2192,	2756,	3564 , \
		70,     459,	592,	769,	1008,	1349,	1845,	2605,	3376,	4498 , \
		75,     526,	680,	886,	1168,	1567,	2157,	3099,	4150,	5505 , \
		80,     606,	780,	1020,	1352,	1816,	2519,	3687,	5083,	6710 , \
		85,     701,	902,	1174,	1553,	2104,	2942,	4352,	6156,	8036 , \
		90,     803,	1035,	1352,	1793,	2440,	3429,	5157,	7473,	9863 , \
		95,     927,	1193,	1562,	2071,	2826,	3995,	6133,	9084,	11582 , \
		100,    1067,	1372,	1795,	2385,	3266,	4689,	7255,	11014,	13831 , \
		105,    1233,	1591,	2069,	2751,	3777,	5489,	8591,	12942,	16334 , \
		110,    1431,	1835,	2398,	3194,	4396,	6387,	10173,	15558,	19494 , \
		115,    1642,	2110,	2763,	3695,	5107,	7462,	12018,	18354,	23611 , \
		120,    1881,	2425,	3176,	4241,	5909,	8736,	14267,	21928,	28407 ,

	/***************************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,           1050,   1100,   1150, \            */
	/*      25,     65,     83,     104,    129,            172,    195,    241,    294 , \            */
	/*      30,     76,     97,     122,    151,            201,    229,    282,    344 , \            */
	/*      35,     89,     113,    142,    177,            235,    267,    330,    403 , \            */
	/*      40,     104,    132,    166,    207,            275,    313,    386,    471 , \            */
	/*      45,     122,    155,    195,    242,            321,    366,    451,    551 , \            */
	/*      50,     142,    181,    228,    283,            376,    428,    528,    645 , \            */
	/*      55,     166,    212,    266,    331,            440,    501,    617,    754 , \            */
	/*      60,     195,    248,    312,    387,            514,    586,    722,    882 , \            */
	/*      65,     228,    290,    365,    453,            602,    685,    845,    1032 , \           */
	/*      70,     266,    339,    427,    530,            704,    802,    989,    1207 , \           */
	/*      75,     312,    397,    499,    619,            823,    938,    1156,   1412 , \           */
	/*      80,     364,    464,    584,    725,            963,    1097,   1353,   1652 , \           */
	/*      85,     426,    543,    683,    848,            1127,   1284,   1582,   1933 , \           */
	/*      90,     499,    635,    799,    992,            1318,   1501,   1851,   2261 , \           */
	/*      95,     583,    743,    934,    1160,           1541,   1756,   2165,   2644 , \           */
	/*      100,    682,    869,    1093,   1357,           1803,   2054,   2532,   3093 , \           */
	/*      105,    798,    1017,   1278,   1587,           2109,   2402,   2962,   3618 ,             */
	/***************************************************************************************************/

#define CA15L_TABLE_1							\
	/**/    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     38,	45,	57,	70,	86,	107,	136,	154,	173 , \
		30,     43,	52,	64,	79,	99,	122,	156,	176,	199 , \
		35,     49,	59,	73,	91,	114,	141,	178,	200,	226 , \
		40,     56,	68,	83,	104,	130,	162,	206,	228,	259 , \
		45,     64,	77,	96,	121,	149,	187,	233,	263,	296 , \
		50,     73,	89,	110,	137,	169,	214,	267,	300,	338 , \
		55,     84,	103,	127,	158,	195,	244,	308,	346,	391 , \
		60,     96,	118,	145,	180,	223,	282,	354,	399,	454 , \
		65,     110,	134,	168,	208,	258,	322,	404,	457,	519 , \
		70,     125,	155,	192,	238,	296,	368,	461,	521,	592 , \
		75,     143,	179,	220,	272,	337,	420,	524,	593,	673 , \
		80,     164,	202,	250,	310,	383,	475,	595,	676,	765 , \
		85,     186,	230,	288,	351,	439,	544,	678,	771,	880 , \
		90,     213,	263,	327,	401,	498,	624,	776,	881,	1007 , \
		95,     242,	304,	376,	463,	573,	717,	894,	1012,	1157 , \
		100,    276,	350,	429,	533,	661,	818,	1026,	1164,	1326 , \
		105,    313,	401,	491,	614,	756,	942,	1178,	1338,	1527 , \
		110,    358,	458,	564,	703,	870,	1074,	1352,	1534,	1751 , \
		115,    409,	523,	650,	808,	1003,	1228,	1548,	1740,	2022 , \
		120,    467,	597,	748,	911,	1150,	1407,	1776,	1978,	2300 ,

	/***************************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,           1050,   1100,   1150, \            */
	/*      25,     27,     34,     43,     53,             71,     80,     99,     121 , \            */
	/*      30,     31,     40,     50,     63,             83,     95,     117,    143 , \            */
	/*      35,     37,     47,     59,     74,             98,     112,    138,    168 , \            */
	/*      40,     44,     56,     70,     87,             116,    132,    162,    198 , \            */
	/*      45,     52,     66,     83,     102,            136,    155,    191,    234 , \            */
	/*      50,     61,     77,     97,     121,            161,    183,    226,    275 , \            */
	/*      55,     72,     91,     115,    142,            189,    216,    266,    325 , \            */
	/*      60,     84,     108,    135,    168,            223,    254,    313,    383 , \            */
	/*      65,     99,     127,    159,    198,            263,    300,    369,    451 , \            */
	/*      70,     117,    149,    188,    233,            310,    353,    435,    532 , \            */
	/*      75,     138,    176,    221,    275,            365,    416,    513,    627 , \            */
	/*      80,     163,    208,    261,    324,            430,    490,    605,    738 , \            */
	/*      85,     192,    245,    307,    382,            507,    578,    712,    870 , \            */
	/*      90,     226,    288,    362,    450,            598,    681,    840,    1026 , \           */
	/*      95,     267,    340,    427,    530,            705,    803,    990,    1209 , \           */
	/*      100,    314,    400,    503,    625,            830,    946,    1166,   1425 , \           */
	/*      105,    370,    472,    593,    736,            979,    1115,   1375,   1679 ,             */
	/***************************************************************************************************/

#define CA15L_TABLE_2							\
	/**/    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     32,	37,	45,	54,	66,	79,	100,	112,	126 , \
		30,     34,	41,	50,	60,	73,	88,	111,	124,	140 , \
		35,     38,	46,	56,	67,	82,	99,	124,	138,	156 , \
		40,     43,	51,	62,	75,	91,	110,	138,	154,	174 , \
		45,     47,	57,	68,	83,	101,	123,	153,	171,	194 , \
		50,     52,	63,	76,	92,	113,	139,	171,	192,	216 , \
		55,     58,	70,	84,	102,	125,	155,	192,	214,	241 , \
		60,     64,	78,	93,	115,	141,	172,	214,	240,	270 , \
		65,     72,	87,	104,	128,	156,	193,	239,	268,	301 , \
		70,     80,	97,	118,	145,	175,	216,	268,	300,	336 , \
		75,     88,	109,	131,	161,	195,	241,	299,	333,	374 , \
		80,     99,	122,	146,	179,	218,	270,	333,	370,	415 , \
		85,     110,	136,	163,	200,	245,	303,	373,	413,	464 , \
		90,     123,	151,	182,	223,	275,	337,	415,	460,	514 , \
		95,     136,	167,	202,	248,	305,	376,	461,	513,	573 , \
		100,    151,	185,	224,	276,	338,	418,	514,	570,	639 , \
		105,    166,	205,	250,	305,	378,	466,	571,	635,	710 , \
		110,    183,	228,	278,	338,	417,	516,	632,	707,	793 , \
		115,    203,	251,	308,	374,	463,	574,	700,	785,	878 , \
		120,    226,	277,	342,	416,	514,	638,	775,	869,	980 ,

	/***************************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,           1050,   1100,   1150, \            */
	/*      25,     27,     34,     43,     53,             71,     80,     99,     121 , \            */
	/*      30,     31,     40,     50,     63,             83,     95,     117,    143 , \            */
	/*      35,     37,     47,     59,     74,             98,     112,    138,    168 , \            */
	/*      40,     44,     56,     70,     87,             116,    132,    162,    198 , \            */
	/*      45,     52,     66,     83,     102,            136,    155,    191,    234 , \            */
	/*      50,     61,     77,     97,     121,            161,    183,    226,    275 , \            */
	/*      55,     72,     91,     115,    142,            189,    216,    266,    325 , \            */
	/*      60,     84,     108,    135,    168,            223,    254,    313,    383 , \            */
	/*      65,     99,     127,    159,    198,            263,    300,    369,    451 , \            */
	/*      70,     117,    149,    188,    233,            310,    353,    435,    532 , \            */
	/*      75,     138,    176,    221,    275,            365,    416,    513,    627 , \            */
	/*      80,     163,    208,    261,    324,            430,    490,    605,    738 , \            */
	/*      85,     192,    245,    307,    382,            507,    578,    712,    870 , \            */
	/*      90,     226,    288,    362,    450,            598,    681,    840,    1026 , \           */
	/*      95,     267,    340,    427,    530,            705,    803,    990,    1209 , \           */
	/*      100,    314,    400,    503,    625,            830,    946,    1166,   1425 , \           */
	/*      105,    370,    472,    593,    736,            979,    1115,   1375,   1679 ,             */
	/***************************************************************************************************/

#define GPU_TABLE_0							\
	/**/    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     23,	29,	37,	47,	60,	77,	97,	107,	119 , \
		30,     26,	33,	42,	54,	69,	88,	111,	124,	138 , \
		35,     30,	38,	48,	62,	79,	101,	127,	143,	160 , \
		40,     34,	43,	56,	71,	90,	115,	146,	165,	186 , \
		45,     39,	50,	63,	81,	103,	131,	167,	191,	216 , \
		50,     45,	57,	72,	92,	117,	149,	192,	219,	251 , \
		55,     51,	65,	83,	105,	133,	170,	219,	252,	290 , \
		60,     59,	74,	94,	120,	152,	195,	251,	289,	335 , \
		65,     68,	85,	107,	137,	173,	222,	286,	331,	387 , \
		70,     78,	98,	123,	155,	198,	253,	326,	379,	449 , \
		75,     89,	112,	140,	177,	225,	288,	370,	437,	519 , \
		80,     103,	128,	161,	202,	256,	328,	424,	500,	603 , \
		85,     117,	147,	184,	232,	292,	374,	485,	574,	702 , \
		90,     135,	168,	210,	265,	335,	428,	557,	666,	817 , \
		95,     155,	193,	240,	303,	383,	488,	636,	770,	951 , \
		100,    179,	221,	275,	345,	437,	559,	728,	886,	1106 , \
		105,    205,	253,	316,	395,	499,	642,	836,	1020,	1280 , \
		110,    236,	291,	362,	452,	571,	733,	960,	1176,	1478 , \
		115,    270,	334,	414,	518,	651,	834,	1097,	1350,	1719 , \
		120,    308,	380,	472,	590,	744,	955,	1255,	1558,	2004 ,

	/******************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,   1050,   1100,   1150, \           */
	/*      25,     24,     31,     39,     48,     59,     73,     90,     110 , \           */
	/*      30,     28,     36,     45,     55,     68,     85,     105,    128 , \           */
	/*      35,     32,     41,     52,     64,     79,     98,     121,    148 , \           */
	/*      40,     37,     47,     60,     74,     91,     113,    140,    171 , \           */
	/*      45,     43,     55,     69,     86,     105,    131,    161,    197 , \           */
	/*      50,     50,     63,     80,     99,     121,    151,    187,    228 , \           */
	/*      55,     57,     73,     92,     114,    140,    175,    216,    263 , \           */
	/*      60,     66,     85,     106,    132,    162,    202,    249,    304 , \           */
	/*      65,     77,     98,     123,    153,    188,    234,    288,    352 , \           */
	/*      70,     89,     113,    142,    177,    217,    270,    333,    406 , \           */
	/*      75,     103,    131,    164,    204,    250,    312,    385,    470 , \           */
	/*      80,     119,    151,    190,    236,    289,    361,    444,    543 , \           */
	/*      85,     137,    175,    219,    272,    335,    417,    514,    627 , \           */
	/*      90,     158,    202,    254,    315,    387,    482,    594,    725 , \           */
	/*      95,     183,    233,    293,    364,    447,    557,    686,    838 , \           */
	/*      100,    212,    270,    339,    421,    516,    643,    793,    969 , \           */
	/*      105,    244,    312,    392,    486,    597,    743,    917,    1120 ,            */
	/******************************************************************************************/

#define GPU_TABLE_1							\
	/**/    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     8,	10,	13,	16,	21,	26,	34,	38,	44 , \
		30,     9,	12,	15,	19,	23,	30,	38,	43,	49 , \
		35,     11,	14,	17,	21,	27,	34,	43,	49,	55 , \
		40,     12,	16,	19,	24,	30,	38,	49,	55,	63 , \
		45,     14,	18,	22,	28,	34,	43,	55,	62,	70 , \
		50,     16,	20,	25,	31,	39,	50,	62,	70,	79 , \
		55,     19,	23,	29,	36,	45,	56,	70,	79,	89 , \
		60,     22,	27,	33,	41,	51,	63,	79,	89,	100 , \
		65,     25,	31,	38,	46,	57,	71,	89,	101,	113 , \
		70,     28,	35,	43,	53,	65,	81,	101,	114,	128 , \
		75,     33,	40,	49,	61,	75,	92,	114,	128,	144 , \
		80,     37,	46,	56,	69,	85,	104,	128,	144,	161 , \
		85,     43,	53,	64,	79,	96,	118,	145,	161,	181 , \
		90,     49,	60,	73,	90,	109,	133,	163,	182,	204 , \
		95,     56,	69,	83,	101,	123,	151,	184,	206,	230 , \
		100,    64,	78,	94,	114,	138,	168,	207,	232,	258 , \
		105,    73,	88,	107,	130,	157,	190,	233,	260,	287 , \
		110,    82,	101,	122,	147,	178,	215,	261,	290,	322 , \
		115,    93,	114,	138,	166,	202,	242,	295,	326,	362 , \
		120,    106,	130,	156,	186,	229,	273,	333,	367,	409 ,

	/******************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,   1050,   1100,   1150, \           */
	/*      25,     9,      11,     14,     18,     22,     27,     34,     41 , \            */
	/*      30,     11,     13,     17,     21,     26,     32,     40,     48 , \            */
	/*      35,     12,     16,     20,     25,     30,     38,     47,     57 , \            */
	/*      40,     15,     19,     23,     29,     36,     44,     55,     67 , \            */
	/*      45,     17,     22,     28,     34,     42,     52,     65,     79 , \            */
	/*      50,     20,     26,     32,     40,     50,     62,     76,     93 , \            */
	/*      55,     24,     30,     38,     47,     58,     73,     89,     109 , \           */
	/*      60,     28,     36,     45,     56,     69,     85,     105,    129 , \           */
	/*      65,     33,     42,     53,     66,     81,     101,    124,    151 , \           */
	/*      70,     39,     50,     62,     77,     95,     118,    146,    178 , \           */
	/*      75,     46,     58,     73,     91,     112,    139,    172,    210 , \           */
	/*      80,     54,     69,     86,     107,    132,    164,    202,    247 , \           */
	/*      85,     64,     81,     102,    126,    155,    193,    238,    291 , \           */
	/*      90,     75,     95,     120,    149,    183,    227,    280,    342 , \           */
	/*      95,     88,     112,    141,    175,    215,    268,    330,    403 , \           */
	/*      100,    104,    132,    166,    206,    253,    315,    389,    475 , \           */
	/*      105,    122,    156,    195,    243,    298,    371,    457,    559 ,             */
	/******************************************************************************************/

#define GPU_TABLE_2							\
	/**/    800,	850,	900,	950,	1000,	1050,	1100,	1125,	1150, \
		25,     11,	13,	16,	21,	26,	33,	42,	48,	54 , \
		30,     12,	14,	18,	22,	28,	35,	45,	51,	58 , \
		35,     13,	16,	20,	25,	31,	38,	49,	55,	63 , \
		40,     14,	17,	21,	27,	33,	42,	53,	60,	67 , \
		45,     15,	19,	23,	29,	36,	45,	57,	64,	73 , \
		50,     17,	21,	26,	32,	39,	49,	62,	70,	79 , \
		55,     18,	23,	28,	35,	43,	53,	67,	76,	85 , \
		60,     20,	25,	30,	38,	47,	58,	73,	82,	92 , \
		65,     22,	27,	33,	41,	51,	63,	79,	88,	100 , \
		70,     24,	30,	36,	45,	55,	69,	86,	96,	108 , \
		75,     27,	33,	40,	49,	60,	74,	93,	104,	116 , \
		80,     29,	36,	44,	54,	66,	81,	100,	112,	126 , \
		85,     32,	39,	48,	58,	72,	88,	109,	121,	136 , \
		90,     35,	43,	53,	64,	78,	96,	118,	131,	147 , \
		95,     39,	47,	57,	70,	85,	104,	128,	142,	159 , \
		100,    43,	52,	63,	76,	93,	113,	139,	155,	172 , \
		105,    47,	57,	69,	83,	101,	123,	150,	167,	186 , \
		110,    51,	63,	75,	91,	111,	134,	163,	181,	202 , \
		115,    56,	69,	82,	99,	121,	146,	178,	196,	219 , \
		120,    62,	75,	90,	108,	131,	159,	193,	213,	237 ,

	/******************************************************************************************/
	/* /\**\/       800,    850,    900,    950,    1000,   1050,   1100,   1150, \           */
	/*      25,     9,      11,     14,     18,     22,     27,     34,     41 , \            */
	/*      30,     11,     13,     17,     21,     26,     32,     40,     48 , \            */
	/*      35,     12,     16,     20,     25,     30,     38,     47,     57 , \            */
	/*      40,     15,     19,     23,     29,     36,     44,     55,     67 , \            */
	/*      45,     17,     22,     28,     34,     42,     52,     65,     79 , \            */
	/*      50,     20,     26,     32,     40,     50,     62,     76,     93 , \            */
	/*      55,     24,     30,     38,     47,     58,     73,     89,     109 , \           */
	/*      60,     28,     36,     45,     56,     69,     85,     105,    129 , \           */
	/*      65,     33,     42,     53,     66,     81,     101,    124,    151 , \           */
	/*      70,     39,     50,     62,     77,     95,     118,    146,    178 , \           */
	/*      75,     46,     58,     73,     91,     112,    139,    172,    210 , \           */
	/*      80,     54,     69,     86,     107,    132,    164,    202,    247 , \           */
	/*      85,     64,     81,     102,    126,    155,    193,    238,    291 , \           */
	/*      90,     75,     95,     120,    149,    183,    227,    280,    342 , \           */
	/*      95,     88,     112,    141,    175,    215,    268,    330,    403 , \           */
	/*      100,    104,    132,    166,    206,    253,    315,    389,    475 , \           */
	/*      105,    122,    156,    195,    243,    298,    371,    457,    559 ,             */
	/******************************************************************************************/

typedef struct spower_raw_s {
	int vsize;
	int tsize;
	int table_size;
	int *table[];
} spower_raw_t;


/** table order: ff, tt, ss **/
int ca7_data[][VSIZE * TSIZE + VSIZE + TSIZE] = {
	{CA7_TABLE_0},
	{CA7_TABLE_1},
	{CA7_TABLE_2},
};

int ca15l_data[][VSIZE * TSIZE + VSIZE + TSIZE] = {
	{CA15L_TABLE_0},
	{CA15L_TABLE_1},
	{CA15L_TABLE_2},
};

int gpu_data[][VSIZE * TSIZE + VSIZE + TSIZE] = {
	{GPU_TABLE_0},
	{GPU_TABLE_1},
	{GPU_TABLE_2},
};


spower_raw_t ca7_spower_raw = {
	.vsize = VSIZE,
	.tsize = TSIZE,
	.table_size = 3,
	.table = {(int *)&ca7_data[0], (int *)&ca7_data[1], (int *)&ca7_data[2]},
};


spower_raw_t ca15l_spower_raw = {
	.vsize = VSIZE,
	.tsize = TSIZE,
	.table_size = 3,
	.table = {(int *)&ca15l_data[0], (int *)&ca15l_data[1], (int *)&ca15l_data[2]},
};

spower_raw_t gpu_spower_raw = {
	.vsize = VSIZE,
	.tsize = TSIZE,
	.table_size = 3,
	.table = {(int *)&gpu_data[0], (int *)&gpu_data[1], (int *)&gpu_data[2]},
};



typedef struct voltage_row_s {
	int mV[VSIZE];
} vrow_t;

typedef struct temperature_row_s {
	int deg;
	int mA[VSIZE];
} trow_t;


typedef struct sptab_s {
	int vsize;
	int tsize;
	int *data;		/* array[VSIZE + TSIZE + (VSIZE*TSIZE)]; */
	vrow_t *vrow;		/* pointer to voltage row of data */
	trow_t *trow;		/* pointer to temperature row of data */
} sptbl_t;

#define trow(tab, ti)		((tab)->trow[ti])
#define mA(tab, vi, ti)	((tab)->trow[ti].mA[vi])
#define mV(tab, vi)		((tab)->vrow[0].mV[vi])
#define deg(tab, ti)		((tab)->trow[ti].deg)
#define vsize(tab)		((tab)->vsize)
#define tsize(tab)		((tab)->tsize)
#define tab_validate(tab)	(!!(tab) && (tab)->data != NULL)

static inline void spower_tab_construct(sptbl_t(*tab)[], spower_raw_t *raw)
{
	int i;
	sptbl_t *ptab = (sptbl_t *) tab;

	for (i = 0; i < raw->table_size; i++) {
		ptab->vsize = raw->vsize;
		ptab->tsize = raw->tsize;
		ptab->data = raw->table[i];
		ptab->vrow = (vrow_t *) ptab->data;
		ptab->trow = (trow_t *) (ptab->data + ptab->vsize);
		ptab++;
	}
}



#endif
#if 0
/*
 * soc-ocv.h
 *
 *  Created on: Apr 29, 2019
 *      Author: LeBaCuong
 */

#ifndef SERVICE_SOC_OCV_SOC_OCV_H_
#define SERVICE_SOC_OCV_SOC_OCV_H_

#include "stdint.h"

#define SOC_LOOKUPS_TABLE_SIZE 150
#define OCV_LOOKUPS_TABLE_SIZE 150

#define SOC_LUT_SIZE 101
#define OCV_LUT_SIZE 101
#define INTERNAL_RESISTANCE_mOhm  100
//data 1 cell by LBC
static const float soc_values[SOC_LOOKUPS_TABLE_SIZE] ={100.0,
		99.25,
		98.5,
		97.75,
		97,
		96.25,
		95.525,
		94.775,
		94.05,
		93.3,
		92.55,
		91.8,
		91.075,
		90.35,
		89.625,
		88.875,
		88.15,
		87.425,
		86.675,
		85.975,
		85.25,
		84.525,
		83.8,
		83.075,
		82.35,
		81.65,
		80.925,
		80.175,
		79.475,
		78.775,
		78.05,
		77.325,
		76.625,
		75.9,
		75.2,
		74.475,
		73.775,
		73.075,
		72.35,
		71.625,
		70.875,
		70.25,
		69.55,
		68.875,
		68.15,
		67.45,
		66.75,
		66.075,
		65.35,
		64.65,
		63.975,
		63.275,
		62.575,
		61.9,
		61.175,
		60.525,
		59.825,
		59.1,
		58.45,
		57.775,
		57.1,
		56.425,
		55.75,
		55.05,
		54.4,
		53.725,
		53.05,
		52.375,
		52.0,
		51.35,
		50.7,
		50.025,
		49.375,
		48.7,
		48.05,
		47.375,
		46.725,
		46.05,
		45.4,
		44.725,
		44.075,
		43.425,
		42.775,
		42.1,
		41.45,
		40.8,
		40.15,
		39.5,
		38.875,
		38.225,
		37.525,
		36.9,
		36.275,
		35.55,
		34.925,
		34.275,
		33.625,
		32.975,
		32.325,
		31.65,
		31.0,
		30.35,
		29.7,
		29.05,
		28.4,
		27.75,
		27.1,
		26.45,
		25.8,
		25.175,
		24.525,
		23.875,
		23.225,
		22.575,
		21.95,
		21.3,
		20.65,
		20.0,
		19.35,
		18.725,
		18.075,
		17.425,
		16.775,
		16.15,
		15.5,
		14.87,5,
		14.225,
		13.6,
		12.975,
		12.325,
		11.7,
		11.075,
		10.45,
		9.825,
		9.175,
		8.55,
		7.925,
		7.3,
		6.675,
		6.05,
		5.4,
		4.8,
		4.175,
		3.55,
		2.95,
		2.325,
		1.725,
		1.15,
		0.575
};

static const float ocv_values[OCV_LOOKUPS_TABLE_SIZE] = {4.2025,
		4.1887,
		4.177,
		4.1657,
		4.1555,
		4.1452,
		4.1351,
		4.125,
		4.1152,
		4.1054,
		4.0957,
		4.0861,
		4.0764,
		4.0671,
		4.0576,
		4.0484,
		4.0391,
		4.0298,
		4.0207,
		4.0116,
		4.0028,
		3.9939,
		3.9852,
		3.9764,
		3.9681,
		3.9595,
		3.9511,
		3.9421,
		3.9341,
		3.9263,
		3.918,
		3.9098,
		3.9015,
		3.8932,
		3.8854,
		3.8771,
		3.8694,
		3.8615,
		3.8532,
		3.8454,
		3.8375,
		3.8296,
		3.8217,
		3.8138,
		3.8061,
		3.7984,
		3.7907,
		3.7831,
		3.7756,
		3.7672,
		3.7607,
		3.7538,
		3.7464,
		3.7398,
		3.7334,
		3.727,
		3.7209,
		3.715,
		3.7096,
		3.7042,
		3.6989,
		3.6941,
		3.6892,
		3.6845,
		3.6803,
		3.6762,
		3.6722,
		3.6683,
		3.6665,
		3.663,
		3.6594,
		3.6562,
		3.6531,
		3.6502,
		3.6472,
		3.6444,
		3.6415,
		3.6389,
		3.6361,
		3.6335,
		3.631,
		3.6288,
		3.6265,
		3.6239,
		3.6217,
		3.6195,
		3.6172,
		3.6149,
		3.6128,
		3.6104,
		3.6083,
		3.6062,
		3.604,
		3.6018,
		3.5997,
		3.5974,
		3.5951,
		3.593,
		3.5908,
		3.5881,
		3.5861,
		3.5836,
		3.5812,
		3.5788,
		3.5762,
		3.57736,
		3.5711,
		3.5685,
		3.5658,
		3.563,
		3.5601,
		3.5572,
		3.5541,
		3.551,
		3.5476,
		3.544,
		3.5405,
		3.5366,
		3.5326,
		3.5283,
		3.5237,
		3.5187,
		3.5133,
		3.5079,
		3.5019,
		3.4952,
		3.4884,
		3.4811,
		3.4738,
		3.4667,
		3.4608,
		3.456,
		3.4517,
		3.4476,
		3.4436,
		3.4394,
		3.4348,
		3.4298,
		3.4241,
		3.4179,
		3.4102,
		3.4009,
		3.3887,
		3.3692,
		3.3469,
		3.3103,
		3.2592,
		3.1875,
		3.0849
};
//data pack 24Ah by NTL
static const float soc_values_24ah[90] ={
		98.833,
		97.674,
		96.519,
		95.37,
		94.224,
		93.082,
		91.944,
		90.819,
		89.697,
		88.577,
		87.461,
		86.348,
		85.237,
		84.119,
		83.013,
		81.91,
		80.801,
		79.703,
		78.606,
		77.502,
		76.412,
		75.313,
		74.217,
		73.132,
		72.047,
		70.963,
		69.882,
		68.804,
		67.727,
		66.652,
		65.578,
		64.506,
		63.434,
		62.366,
		61.29,
		60.217,
		59.146,
		58.076,
		57.012,
		55.951,
		54.892,
		53.833,
		52.777,
		51.722,
		50.667,
		49.624,
		48.581,
		47.55,
		46.513,
		45.475,
		44.437,
		43.401,
		42.366,
		41.347,
		40.33,
		39.314,
		38.29,
		37.268,
		36.246,
		35.225,
		34.216,
		33.2,
		32.184,
		31.168,
		30.152,
		29.138,
		28.138,
		27.132,
		26.127,
		25.123,
		24.12,
		23.118,
		22.118,
		21.119,
		20.121,
		19.125,
		18.13,
		17.127,
		16.126,
		15.127,
		14.128,
		13.13,
		12.134,
		11.14,
		10.149,
		9.16,
		8.175,
		7.205,
		6.241,
		5.106,
};
static const float ocv_values_24ah[90] = {
		60497,
		60207,
		60022,
		59857,
		59699,
		59542,
		59393,
		59244,
		59096,
		58953,
		58809,
		58669,
		58526,
		58384,
		58242,
		58105,
		57968,
		57831,
		57700,
		57571,
		57445,
		57326,
		57208,
		57093,
		56988,
		56890,
		56783,
		56679,
		56573,
		56471,
		56369,
		56268,
		56173,
		56074,
		55979,
		55875,
		55778,
		55685,
		55615,
		55525,
		55441,
		55358,
		55275,
		55195,
		55115,
		55030,
		54955,
		54886,
		54831,
		54759,
		54693,
		54628,
		54564,
		54503,
		54433,
		54376,
		54323,
		54273,
		54215,
		54157,
		54104,
		54049,
		53999,
		53943,
		53887,
		53821,
		53765,
		53709,
		53657,
		53604,
		53542,
		53483,
		53423,
		53363,
		53295,
		53239,
		53180,
		53126,
		53065,
		53004,
		52943,
		52881,
		52821,
		52749,
		52655,
		52556,
		52417,
		52248,
		52017,
		51710
};
// data pack 16Ah by NTL
static const float soc_values_16ah[64] ={
		100,
		98.888,
		97.791,
		96.702,
		95.617,
		94.529,
		93.445,
		92.346,
		91.249,
		90.158,
		89.071,
		87.976,
		86.896,
		85.82,
		84.737,
		83.658,
		82.592,
		81.529,
		80.46,
		79.394,
		78.341,
		77.291,
		76.232,
		75.187,
		74.145,
		73.105,
		72.058,
		71.021,
		69.986,
		68.945,
		67.906,
		66.878,
		65.852,
		64.818,
		63.785,
		62.761,
		61.74,
		60.712,
		59.686,
		58.669,
		57.654,
		56.631,
		55.617,
		54.61,
		53.601,
		52.583,
		51.575,
		50.569,
		49.565,
		48.553,
		47.551,
		46.551,
		45.544,
		44.539,
		43.544,
		42.551,
		41.551,
		40.554,
		39.568,
		38.585,
		37.596,
		36.61,
		35.638,
		34.672
};

static const float ocv_values_16ah[64] = {
		60750,
		60272,
		59869,
		59615,
		59381,
		59153,
		58927,
		58708,
		58476,
		58254,
		58040,
		57826,
		57659,
		57481,
		57295,
		57098,
		56905,
		56721,
		56513,
		56321,
		56190,
		56030,
		55873,
		55685,
		55546,
		55376,
		55267,
		55161,
		55053,
		54944,
		54840,
		54745,
		54632,
		54536,
		54448,
		54371,
		54297,
		54219,
		54143,
		54076,
		54006,
		53937,
		53869,
		53804,
		53742,
		53667,
		53599,
		53534,
		53463,
		53389,
		53322,
		53246,
		53163,
		53077,
		52991,
		52902,
		52806,
		52702,
		52590,
		52479,
		52350,
		52204,
		52028,
		51821
};
//data voltage_pack OCV based on polynomial of degree 5 of SoC
static const uint16_t ocv_values_pack[OCV_LUT_SIZE] ={
		65500, //64800, //because change OV_threshold
		64411,
		64045,
		63700,
		63377,
		63072,
		62786,
		62516,
		62263,
		62024,
		61799,
		61587,
		61387,
		61198,
		61020,
		60851,
		60691,
		60539,
		60395,
		60258,
		60128,
		60004,
		59885,
		59771,
		59662,
		59557,
		59456,
		59359,
		59265,
		59174,
		59086,
		59000,
		58917,
		58836,
		58757,
		58680,
		58605,
		58531,
		58459,
		58389,
		58320,
		58252,
		58186,
		58121,
		58057,
		57994,
		57933,
		57872,
		57813,
		57755,
		57697,
		57641,
		57586,
		57531,
		57478,
		57425,
		57373,
		57322,
		57271,
		57220,
		57170,
		57120,
		57070,
		57020,
		56969,
		56918,
		56866,
		56813,
		56759,
		56703,
		56645,
		56584,
		56520,
		56454,
		56383,
		56308,
		56229,
		56144,
		56053,
		55955,
		55850,
		55737,
		55615,
		55483,
		55340,
		55186,
		55020,
		54839,
		54645,
		54434,
		54206,
		53960,
		53695,
		53409,
		53100,
		52768,
		52410,
		52025,
		51612,
		51168,
		50692
};
static const uint16_t soc_values_pack[SOC_LUT_SIZE] ={
		100,
		99,
		98,
		97,
		96,
		95,
		94,
		93,
		92,
		91,
		90,
		89,
		88,
		87,
		86,
		85,
		84,
		83,
		82,
		81,
		80,
		79,
		78,
		77,
		76,
		75,
		74,
		73,
		72,
		71,
		70,
		69,
		68,
		67,
		66,
		65,
		64,
		63,
		62,
		61,
		60,
		59,
		58,
		57,
		56,
		55,
		54,
		53,
		52,
		51,
		50,
		49,
		48,
		47,
		46,
		45,
		44,
		43,
		42,
		41,
		40,
		39,
		38,
		37,
		36,
		35,
		34,
		33,
		32,
		31,
		30,
		29,
		28,
		27,
		26,
		25,
		24,
		23,
		22,
		21,
		20,
		19,
		18,
		17,
		16,
		15,
		14,
		13,
		12,
		11,
		10,
		9,
		8,
		7,
		6,
		5,
		4,
		3,
		2,
		1,
		0
};
typedef struct SOC_OCV_t SOC_OCV;
struct SOC_OCV_t{
	uint32_t soc;
	uint32_t vol;
};

void soc_ocv_interpoltae(SOC_OCV *soc_ocv);
void ocv_soc_interpoltae(SOC_OCV *soc_ocv);
int32_t get_soc_from_ocv(uint32_t voltage, int32_t current);

#endif /* SERVICE_SOC_OCV_SOC_OCV_H_ */
#endif

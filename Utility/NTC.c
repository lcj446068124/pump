#include "ntc.h"

// extern volatile int16_t NTCTemperature;

int StartNTC(uint16_t ad_res)
{
	int R;
	R = 100000 * (int)ad_res / (4096 - ad_res);
	return((int)(10*ConverseToTemperature(R)));
	
}

float ConverseToTemperature(int R)
{
	uint8_t low,high,i;
	int integer;
	float res,fraction;
	low = 70; high = 75;
    while(low < high)
    {
       if(NTC_103_INTERGER[low] < R)
       {
           high = low;
           low = low - 5;
       }
       else if(NTC_103_INTERGER[high] > R)
       {
           low = high;
           high = high + 5;
       }
       else
       {
           for(i = low; i <= high; i++)
           {
               if(NTC_103_INTERGER[i+1] < R)
		       {
                   /* calculate the current temperature interger section and decimal section digital seperately*/
            	   fraction = (NTC_103_INTERGER[i] - R)/NTC_103_FRACTION[i];
                   integer = i-50;
                   res = integer+0.1*fraction;
                   break;
		        }
						   
           }
           break;
       }
    }
    return res;
}

uint32_t const NTC_103_INTERGER[161] = {
		3295000, 3109000, 2935000, 2772000, 2620000, 2477000, 2343000, 2217000, 2099000, 1989000,
		1885000, 1785000, 1690000, 1602000, 1519000, 1441000, 1367000, 1298000, 1233000, 1171000,
		1113000, 1057000, 1005000,  955200,  908400,  864300,  822600,  783300,  746100,  711000,
		 677700,  645699,  615400,  586800,  559700,  534100,  509799,  486800,  465000,  444300,
		 424700,  405700,  387700,  370600,  354400,  339000,  324400,  310500,  297300,  284800,
		 272800,  261300,  250300,  239899,  230000,  220500,  211500,  203000,  194800,  187000,
		 179600,  172399,  165600,  159000,  152800,  146900,  141200,  135800,  130600,  125600,
		 120900,  116300,  112000,  107800,  103800,  100000,   96320,   92810,   89440,   86220,
		  83130,   80140,   77280,   74540,   71920,   69400,   66990,   64670,   62450,   60320,
		  58270,   56289,   54380,   52550,   50800,   49109,   47490,   45930,   44429,   42990,
		  41600,   40260,   38960,   37710,   36510,   35360,   34250,   33180,   32150,   31160,
		  30200,   29270,   28380,   27510,   26680,   25880,   25110,   24360,   23640,   22950,
		  22280,   21629,   21000,   20390,   19800,   19240,   18690,   18160,   17650,   17160,
		  16680,   16220,   15770,   15330,   14920,   14510,   14120,   13730,   13360,   13010,
		  12660,   12320,   12000,   11680,   11370,   11080,   10790,   10510,   10240,    9984,
		   9731,    9484,    9246,    9014,    8789,    8572,    8360,    8155,    7956,    7763,
		   7576
};

uint32_t const NTC_103_FRACTION[160] = {
		18600,   17400,   16300,   15200,   14300,   13400,   12600,   11800,   11000,   10400,
		  10000,    9500,    8800,    8300,    7800,    7400,    6900,    6500,    6200,    5800,
		   5600,    5200,    4980,    4680,    4410,    4170,    3930,    3720,    3510,    3330,
		   3200,    3029,    2860,    2710,    2560,    2430,    2299,    2180,    2070,    1960,
		   1900,    1800,    1710,    1620,    1540,    1460,    1390,    1320,    1250,    1200,
		   1150,    1100,    1040,     989,     950,     900,     850,     820,     780,     740,
		    720,     679,     660,     620,     590,     570,     540,     520,     500,     470,
		    460,     430,     420,     400,     380,     368,     351,     337,     322,     309,
		    299,     286,     274,     262,     252,     241,     232,     222,     213,     205,
		    198,     190,     183,     175,     169,     161,     156,     150,     143,     139,
		    134,     130,     125,     120,     115,     111,     107,     103,      99,      96,
		     93,      89,      87,      83,      80,      77,      75,      72,      69,      67,
		     65,      62,      61,      59,      56,      55,      53,      51,      49,      48,
		     46,      45,      44,      41,      41,      39,      39,      37,      35,      35,
		     34,      32,      32,      31,      29,      29,      28,      27,      25,      25,
		     24,      23,      23,      22,      21,      21,      20,      19,      19,      18
};
/*****************************************************************************
 * File : ThermP_K.c
 *
 * Source       : Automatically generated using 'TempGen.exe'
 *
 * Compiler     : Intended for most C compilers
 * Description  : Subroutines for thermocouple linearization
 *                (for type K thermocouples)
 *                using piecewise linear approximation method.
 * More Info    : Details in application note available at....
 *                http://www.ti.com/msc
 ******************************************************************************/

/*  Defines section */
#define TMIN (-200.000000)  // = minimum temperature in degC
#define TMAX (1372.000000)  // = maximum temperature in degC
#define NSEG 255  // = number of sections in table
#define TSEG 6.164706  // = (TMAX-TMIN)/NSEG = temperature span in degC of each segment

/********************** Lookup Table ****************************************
 * lookup table size:
 *   = 255 linear sections
 *   = 256 coefficients
 *   = 1024 bytes (4 bytes per floating point coefficient)  *******************/
const float C_volt[] = \
 { -5.891836,-5.794294,-5.689911,-5.578872,-5.461338,-5.337458,-5.207376,-5.071231, \
	 -4.929158,-4.781291,-4.627765,-4.468716,-4.304279,-4.134592,-3.959794,-3.780025, \
	 -3.595429,-3.406151,-3.212337,-3.014137,-2.811703,-2.605189,-2.394749,-2.180538, \
	 -1.962708,-1.741411,-1.516793,-1.288996,-1.058162,-0.824434,-0.587970,-0.348955, \
	 -0.107628,0.135692,0.380791,0.627569,0.875909,1.125694,1.376799,1.629089, \
	 1.882425,2.136652,2.391608,2.647121,2.903015,3.159111,3.415223,3.671166, \
	 3.926776,4.181897,4.436391,4.690146,4.943071,5.195108,5.446231,5.696443, \
	 5.945775,6.194290,6.442076,6.689241,6.935911,7.182220,7.428312,7.674319, \
	 7.920396,8.166663,8.413239,8.660222,8.907701,9.155741,9.404392,9.653690, \
	 9.903653,10.154284,10.405580,10.657521,10.910089,11.163251,11.416980,11.671241, \
	 11.926003,12.181234,12.436905,12.692987,12.949458,13.206293,13.463474,13.720984, \
	 13.978809,14.236935,14.495351,14.754048,15.013017,15.272249,15.531737,15.791474, \
	 16.051453,16.311665,16.572104,16.832762,17.093632,17.354704,17.615969,17.877420, \
	 18.139046,18.400839,18.662786,18.924881,19.187107,19.449457,19.711920,19.974482, \
	 20.237131,20.499857,20.762644,21.025484,21.288363,21.551264,21.814180,22.077097, \
	 22.339998,22.602875,22.865711,23.128496,23.391216,23.653858,23.916412,24.178864, \
	 24.441198,24.703409,24.965479,25.227400,25.489159,25.750744,26.012144,26.273352, \
	 26.534351,26.795135,27.055696,27.316019,27.576099,27.835926,28.095490,28.354784, \
	 28.613800,28.872528,29.130964,29.389099,29.646929,29.904444,30.161642,30.418512, \
	 30.675055,30.931259,31.187126,31.442650,31.697824,31.952646,32.207115,32.461224, \
	 32.714970,32.968353,33.221371,33.474018,33.726295,33.978203,34.229733,34.480888, \
	 34.731670,34.982071,35.232098,35.481739,35.731007,35.979893,36.228394,36.476517, \
	 36.724258,36.971619,37.218594,37.465187,37.711399,37.957226,38.202671,38.447731, \
	 38.692406,38.936695,39.180599,39.424114,39.667240,39.909981,40.152328,40.394283, \
	 40.635845,40.877014,41.117786,41.358158,41.598129,41.837692,42.076851,42.315601, \
	 42.553936,42.791859,43.029358,43.266434,43.503082,43.739300,43.975079,44.210419, \
	 44.445309,44.679752,44.913738,45.147263,45.380318,45.612904,45.845005,46.076626, \
	 46.307751,46.538380,46.768509,46.998123,47.227226,47.455807,47.683853,47.911366, \
	 48.138340,48.364761,48.590633,48.815945,49.040691,49.264866,49.488464,49.711487, \
	 49.933918,50.155766,50.377018,50.597672,50.817730,51.037182,51.256035,51.474285, \
   51.691929,51.908966,52.125404,52.341240,52.556477,52.771122,52.985172,53.198643, \
   53.411530,53.623852,53.835613,54.046822,54.257488,54.467632,54.677261,54.886395};

/* linearization routine error = +/-0.026985 degC
 * specified over measurement range -200.000000 degC to 1372.000000 degC  */

/*****************************************************************************
 * Function name     : Ttype
 *   Returns         : Thermocouple Type
 *   Arguments       : None
 * Description       : Returns the letter designation for the thermocouple type
 *****************************************************************************/
char Ttype () {
  return ('K');
}

/*****************************************************************************
 * Function name     : MinTemp
 *   Returns         : Minimum Temperature of Temperature Range
 *   Arguments       : None
 * Description       : Returns minimum temperature specified by lookup table.
 *****************************************************************************/
float MinTemp () {
  return (TMIN);
}

/****************************************************************************
 * Function Name     : MaxTemp
 *   Returns         : Maximum Temperature of Temperature Range
 *   Arguments       : None
 * Description       : Returns maximum temperature specified by lookup table.
 ****************************************************************************/
float MaxTemp () {
  return (TMAX);
}

/*****************************************************************************
 * Function Name     : T_volt(mV)
 *   Returns         : Temperature in degC (for type K thermocouples)
 *   Argument        : Voltage in mV
 * Description       : Calculates temperature of thermocouple as a function of
 *                     voltage via a piecewise linear approximation method.
 ******************************************************************************/
  float T_volt (float mV) {
  float t;
  int i, Add;

                                // set up initial values
  i = NSEG/2;                   // starting value for 'i' index
  Add = (i+1)/2;                // Add value used in do loop

  // determine if input v is within range
  if (mV<C_volt[0])             // if input is under-range..
    i=0;                        // ..then use lowest coefficients
  else if (mV>C_volt[NSEG])     // if input is over-range..
    i=NSEG-1;                   // ..then use highest coefficients

  // if input within range, determine which coefficients to use
  else do {
    if (C_volt[i]>mV)   i-=Add; // either decrease i by Add..
    if (C_volt[i+1]<mV) i+=Add; // ..or increase i by Add
    if (i<0)       i=0;         // make sure i is >=0..
    if (i>NSEG-1)  i=NSEG-1;    // ..and <=NSEG-1
    Add = (Add+1)/2;            // divide Add by two (rounded)
  } while ((C_volt[i]>mV)||(C_volt[i+1]<mV));  // repeat 'til done

  // compute final result
  t = TMIN+TSEG*i + (mV-C_volt[i])*TSEG/(C_volt[i+1]-C_volt[i]);

  return (t);
}

/***************************************************************************
 * Function Name    : V_temp(t)
 *   Returns        : Voltage (mV) as a function of temperature
 *                  : (for type K thermocouples)
 *   Argument       : Temperature of thermocouple in degC
 * Description      : Calculates voltage of thermocouple as a function of
 *                    temperature via a piecewise linear approximation method.
 ****************************************************************************/
float V_temp (float t) {
  float v;
  int i;

  i=(t-TMIN)/TSEG;      // determine which coefficients to use

  if (i<0)              // if input is under-range..
    i=0;                // ..then use lowest coefficients
  else if (i>NSEG-1)    // if input is over-range..
    i=NSEG-1;           // ..then use highest coefficients

  // compute final result
  v = C_volt[i]+(t-(TMIN+TSEG*i))*(C_volt[i+1]-C_volt[i])/TSEG;

  return (v);
}


/* AM32- multi-purpose brushless controller firmware for the stm32f051 */

//===========================================================================
//=============================== Changelog =================================
//===========================================================================
/*
* 1.54 Changelog;
* --Added firmware name to targets and firmware version to main
* --added two more dshot to beacons 1-3 currently working
* --added KV option to firmware, low rpm power protection is based on KV
* --start power now controls minimum idle power as well as startup strength.
* --change default timing to 22.5
* --Lowered default minimum idle setting to 1.5 percent duty cycle, slider range from 1-2.
* --Added dshot commands to save settings and reset ESC.
*
*1.56 Changelog.
* -- added check to stall protection to wait until after 40 zero crosses to fix high startup throttle hiccup.
* -- added TIMER 1 update interrupt and PWM changes are done once per pwm period
* -- reduce commutation interval averaging length
* -- reduce false positive filter level to 2 and eliminate threshold where filter is stopped.
* -- disable interrupt before sounds
* -- disable TIM1 interrupt during stepper sinusoidal mode
* -- add 28us delay for dshot300
* -- report 0 rpm until the first 10 successful steps.
* -- move serial ADC telemetry calculations and desync check to 10Khz interrupt.
*
* 1.57
* -- remove spurious commutations and rpm data at startup by polling for longer interval on startup
*
* 1.58
* -- move signal timeout to 10khz routine and set armed timeout to one quarter second 2500 / 10000
* 1.59
* -- moved comp order definitions to target.h
* -- fixed update version number if older than new version
* -- cleanup, moved all input and output to IO.c
* -- moved comparator functions to comparator.c
* -- removed ALOT of useless variables
* -- added siskin target
* -- moved pwm changes to 10khz routine
* -- moved basic functions to functions.c
* -- moved peripherals setup to periherals.c
* -- added crawler mode settings
*
* 1.60
* -- added sine mode hysteresis
* -- increased power in stall protection and lowered start rpm for crawlers
* -- removed onehot125 from crawler mode
* -- reduced maximum startup power from 400 to 350
* -- change minimum duty cycle to DEAD_TIME
* -- version and name moved to permanent spot in FLASH memory, thanks mikeller
*
* 1.61
* -- moved duty cycle calculation to 10khz and added max change option.
* -- decreased maximum interval change to 25%
* -- reduce wait time on fast acceleration (fast_accel)
* -- added check in interrupt for early zero cross
*
* 1.62
* --moved control to 10khz loop
* --changed condition for low rpm filter for duty cycle from || to &&
* --introduced max deceleration and set it to 20ms to go from 100 to 0
* --added configurable servo throttle ranges
*
*
*1.63
*-- increase time for zero cross error detection below 250us commutation interval
*-- increase max change a low rpm x10
*-- set low limit of throttle ramp to a lower point and increase upper range
*-- change desync event from full restart to just lower throttle.

*1.64
* --added startup check for continuous high signal, reboot to enter bootloader.
*-- added brake on stop from eeprom
*-- added stall protection from eeprom
*-- added motor pole divider for sinusoidal and low rpm power protection
*-- fixed dshot commands, added confirmation beeps and removed blocking behavior
*--
*1.65
*-- Added 32 millisecond telemetry output
*-- added low voltage cutoff , divider value and cutoff voltage needs to be added to eeprom
*-- added beep to indicate cell count if low voltage active
*-- added current reading on pa3 , conversion factor needs to be added to eeprom
*-- fixed servo input capture to only read positive pulse to handle higher refresh rates.
*-- disabled oneshot 125.
*-- extended servo range to match full output range of receivers
*-- added RC CAR style reverse, proportional brake on first reverse , double tap to change direction
*-- added brushed motor control mode
*-- added settings to EEPROM version 1
*-- add gimbal control option.
*--
*1.66
*-- move idwg init to after input tune
*-- remove reset after save command -- dshot
*-- added wraith32 target
*-- added average pulse check for signal detection
*--
*1.67
*-- Rework file structure for multiple MCU support
*-- Add g071 mcu
*--
*1.68
*--increased allowed average pulse length to avoid double startup
*1.69
*--removed line re-enabling comparator after disabling.
*1.70 fix dshot for Kiss FC
*1.71 fix dshot for Ardupilot / Px4 FC
*1.72 Fix telemetry output and add 1 second arming.
*1.73 Fix false arming if no signal. Remove low rpm throttle protection below 300kv
*1.74 Add Sine Mode range and drake brake strength adjustment
*1.75 Disable brake on stop for PWM_ENABLE_BRIDGE 
Removed automatic brake on stop on neutral for RC car proportional brake.
Adjust sine speed and stall protection speed to more closely match
makefile fixes from Cruwaller 
Removed gd32 build, until firmware is functional
*/
#include <stdint.h>
#include "main.h"
#include "targets.h"
#include "signal.h"
#include "dshot.h"
#include "phaseouts.h"
#include "eeprom.h"
#include "sounds.h"
#include "ADC.h"
#include "serial_telemetry.h"
#include "IO.h"
#include "comparator.h"
#include "functions.h"
#include "peripherals.h"

//===========================================================================
//============================= EEPROM Defaults =============================
//===========================================================================

#define VERSION_MAJOR 1
#define VERSION_MINOR 75
char dir_reversed = 0;
char VARIABLE_PWM = 1;
char brake_on_stop = 0;
char stall_protection = 0;
char THIRTY_TWO_MS_TLM = 0;

char advance_level = 2;			// 7.5 degree increments 0 , 7.5, 15, 22.5)
uint16_t motor_kv = 2000;
char motor_poles = 14;
//add Startup Power
//Add PWM Frequency
//Add Beep Volume
char drag_brake_strength = 10;		// Drag Brake Power
char sine_mode_changeover_thottle_level = 5;	// Sine Startup Range
char sine_mode_changeover_mutliplier = 20;
short sine_mode_changeover = 5 * 20;
int last_zero_crosses = 0;

char USE_HALL_SENSOR = 0;

//============================= Servo Settings ==============================
uint16_t servo_low_threshold = 1100;	// anything below this point considered 0
uint16_t servo_high_threshold = 1900;	// anything above this point considered 2000 (max)
uint16_t servo_neutral = 1500;
uint8_t servo_dead_band = 100;

//========================= Battery Cuttoff Settings ========================
char LOW_VOLTAGE_CUTOFF = 0;		// Turn Low Voltage CUTOFF on or off
uint16_t low_cell_volt_cutoff = 330;	// 3.3volts per cell

//Add Car/basher mode

//=========================== END EEPROM Defaults ===========================

typedef struct __attribute__((packed)) {
	uint8_t version_major;
	uint8_t version_minor;
	char device_name[12];
} firmware_info_s;

firmware_info_s __attribute__ ((section(".firmware_info"))) firmware_info = {
	version_major: VERSION_MAJOR,
	version_minor: VERSION_MINOR,
	device_name: FIRMWARE_NAME
};

uint8_t EEPROM_VERSION;

uint32_t MCU_Id = 0;
uint32_t REV_Id = 0;

uint16_t armed_timeout_count;

uint8_t desync_happened = 0;
char maximum_throttle_change_ramp = 1;
  
uint16_t velocity_count = 0;
uint16_t velocity_count_threshold = 100;

char low_rpm_throttle_limit = 0;

uint16_t low_voltage_count = 0;


uint16_t thirty_two_ms_count;

char VOLTAGE_DIVIDER = TARGET_VOLTAGE_DIVIDER;     // 100k upper and 10k lower resistor in divider

uint16_t battery_voltage;  // scale in volts * 10.  1260 is a battery voltage of 12.60

char cell_count = 0;

uint16_t consumption_timer = 0;

float consumed_current = 0;
uint16_t smoothed_raw_current = 0;
uint16_t actual_current = 0;

char lowkv = 0;
char bemf_timeout = 10;

char startup_boost = 35;
char reversing_dead_band = 1;

int checkcount = 0;
uint16_t low_pin_count = 0;

uint8_t max_duty_cycle_change = 2;
char fast_accel = 1;
uint16_t last_duty_cycle = 0;
char play_tone_flag = 0;

typedef enum
{
	GPIO_PIN_RESET = 0U,
	GPIO_PIN_SET
} GPIO_PinState;

uint16_t minimum_duty_cycle = DEAD_TIME;
char desync_check = 0;
char low_kv_filter_level = 20;

uint16_t tim1_arr = TIM1_AUTORELOAD;         // current auto reset value
uint16_t TIMER1_MAX_ARR = TIM1_AUTORELOAD;      // maximum auto reset register value
int duty_cycle_maximum = TIM1_AUTORELOAD;     //restricted by temperature or low rpm throttle protect
int low_rpm_level  = 20;        // thousand erpm used to set range for throttle resrictions
int high_rpm_level = 70;      //
int throttle_max_at_low_rpm  = 400;
int throttle_max_at_high_rpm = TIM1_AUTORELOAD;

uint16_t commutation_intervals[6] = {0};
uint32_t average_interval = 0;
uint32_t last_average_interval;
int e_com_time;

uint16_t ADC_smoothed_input = 0;
uint8_t degrees_celsius;
uint16_t converted_degrees;
uint8_t temperature_offset;
uint16_t ADC_raw_temp;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;
uint16_t ADC_raw_input;
int adc_counter = 0;
char send_telemetry = 0;
char telemetry_done = 0;
char prop_brake_active = 0;

uint8_t eepromBuffer[48] ={0};
uint32_t gcr[30] =  {0,0,0,0,0,0,0,0,0,0,0,64,0,0,0,0,64,0,0,0,0,64,0,0,0,64,64,0,64,0};
uint8_t gcr_size;
uint16_t process_time = 0;

char dshot_telemetry = 0;
char output = 0;
int dshot_frametime = 0;

uint16_t phase_a_interval = 0;
uint16_t phase_b_interval = 0;
uint16_t phase_c_interval = 0;
uint32_t current_EXTI_LINE;

int dshot_goodcounts = 0;
int dshot_badcounts = 0;
uint8_t last_dshot_command = 0;
char old_routine = 0;
int adjusted_input;

#define TEMP30_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7B8))
#define TEMP110_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7C2))
//
//uint32_t temp110cal = 1;
//uint32_t temp30cal = 1;

int smoothedinput = 0;
int voltageraw;

const int numReadings = 30;     // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;
int readings[30];
int tempraw = 0;
int temp_degrees = 0;

int bemf_timout_happened = 0;
int timeout_count = 0;
int bemf_timeout_threshold = 10;

int changeover_step = 5;
int filter_level = 5;
int running = 0;
int advance = 0;
int advancedivisor = 6;
char rising = 1;
char amplitude = 150;
char sin_cycle_complete = 0;

////Space Vector PWM ////////////////
//const int pwmSin[] ={128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};


////Sine Wave PWM ///////////////////
/*const int pwmSin[] = {180,183,186,189,193,196,199,202,
205,208,211,214,217,220,224,227,
230,233,236,239,242,245,247,250,
253,256,259,262,265,267,270,273,
275,278,281,283,286,288,291,293,
296,298,300,303,305,307,309,312,
314,316,318,320,322,324,326,327,
329,331,333,334,336,337,339,340,
342,343,344,346,347,348,349,350,
351,352,353,354,355,355,356,357,
357,358,358,359,359,359,360,360,
360,360,360,360,360,360,360,359,
359,359,358,358,357,357,356,355,
355,354,353,352,351,350,349,348,
347,346,344,343,342,340,339,337,
336,334,333,331,329,327,326,324,
322,320,318,316,314,312,309,307,
305,303,300,298,296,293,291,288,
286,283,281,278,275,273,270,267,
265,262,259,256,253,250,247,245,
242,239,236,233,230,227,224,220,
217,214,211,208,205,202,199,196,
193,189,186,183,180,177,174,171,
167,164,161,158,155,152,149,146,
143,140,136,133,130,127,124,121,
118,115,113,110,107,104,101,98,
95,93,90,87,85,82,79,77,
74,72,69,67,64,62,60,57,
55,53,51,48,46,44,42,40,
38,36,34,33,31,29,27,26,
24,23,21,20,18,17,16,14,
13,12,11,10,9,8,7,6,
5,5,4,3,3,2,2,1,
1,1,0,0,0,0,0,0,
0,0,0,1,1,1,2,2,
3,3,4,5,5,6,7,8,
9,10,11,12,13,14,16,17,
18,20,21,23,24,26,27,29,
31,33,34,36,38,40,42,44,
46,48,51,53,55,57,60,62,
64,67,69,72,74,77,79,82,
85,87,90,93,95,98,101,104,
107,110,113,115,118,121,124,127,
130,133,136,140,143,146,149,152,
155,158,161,164,167,171,174,177};*/

const float pwmSin[3][180] = {
{0.866025403784439,
0.848048088228774,0.829037555823863,0.809016967994945,0.788010716765259,0.766044395038099,0.743144765415773,0.719339727593471,0.694658284367752,
0.669130506301075,0.642787495085415,0.615661345649618,0.587785107056632,0.559192742238287,0.529919086616654,0.499999805662414,0.469471351441939,
0.438370918206029,0.4067363970744,0.37460632987116,0.34201986216749,0.309016695588752,0.275637039444137,0.241921561737769,0.207911339620969,
0.173647809346027,0.139172715782467,0.104528061557316,0.0697560558813225,0.0348990631234946,-4.48803402319435E-07,-0.0348999601835005,-0.0697569513015993,
-0.104528954246932,-0.139173604653816,-0.173648693316155,-0.207912217612894,-0.241922432681792,-0.275637902279146,-0.309017549263515,-0.342020705641934,
-0.374607162117642,-0.406737217078953,-0.438371724969604,-0.469472143981618,-0.500000583012608,-0.529919847830282,-0.559193486387926,-0.587785833235649,
-0.615662052973275,-0.642788182691947,-0.669131173352737,-0.694658930051847,-0.719340351123329,-0.743145366031718,-0.766044972008373,-0.788011269386913,
-0.809017495594693,-0.829038057758906,-0.848048563887581,-0.866025852587492,-0.882948021282777,-0.898794452899869,-0.913545840986253,-0.927184213232418,
-0.939692953368372,-0.951056821408023,-0.961261972216765,-0.970295972379666,-0.978147815349675,-0.984807934857418,-0.990268216566233,-0.994522007958248,
-0.997564126439457,-0.999390865653915,-0.999999999999371,-0.999390787338817,-0.997563969904676,-0.994521773394498,-0.990267904259294,-0.984807545187788,
-0.978147348792107,-0.970295429502589,-0.961261353681591,-0.951056127968343,-0.939692185869037,-0.927183372608508,-0.913544928261938,-0.898793469187166,
-0.882946967780188,-0.86602473057855,-0.848047374739282,-0.829036802920045,-0.809016176594099,-0.788009887831587,-0.766043529581528,-0.743143864490731,
-0.719338792297598,-0.694657315840562,-0.669129505722568,-0.642786463674646,-0.615660284663201,-0.587784017787218,-0.559191626012984,-0.529917944795411,
-0.499998639636367,-0.469470162631712,-0.438369708060003,-0.406735167066956,-0.374605081500871,-0.342018596955306,-0.30901541507614,-0.275635745191205,
-0.241920255321368,-0.207910022632767,-0.173646483390571,-0.139171382475233,-0.104526722522733,-0.0697547127508003,-0.034897717533433,1.79521360952176E-06,
0.0349013057734571,0.0697582944319116,0.104530293281199,0.13917493796063,0.173650019271086,0.207913534600469,0.241923739097462,0.275639196531245,
0.309018829775193,0.342021970853084,0.374608410486799,0.406738447085169,0.438372935114305,0.469473332790428,0.500001749037144,0.529920989649923,
0.559194602611539,0.587786922503286,0.615663113957831,0.642789214100774,0.669132173929223,0.694659898576939,0.719341286417029,0.743146266954515,
0.766045837462629,0.788012098318204,0.809018286993095,0.829038810660219,0.848049277374511,0.866026525790764,0.882948653382195,0.898795043125318,
0.913546388618633,0.927184717604523,0.939693413865703,0.951057237469531,0.961262343335546,0.970296298103567,0.978148095281852,0.984808168656816,
0.990268403948003,0.994522148694094,0.997564220357914,0.999390912640558,0.999999999996953,0.999390740347343,0.997563875981396,0.994521632653845,
0.990267716872737,0.98480731138363,0.978147068855202,0.970295103773997,0.961260982558164,0.951055711902236,0.939691725367165,0.92718286823192,
0.913544380625142,0.898792878957372,0.882946335676501},
{-0.866025403784438,
-0.882947599882275,-0.89879405941533,-0.913545475897079,-0.927183876983414,-0.939692646369206,-0.951056544032725,-0.961261724803276,-0.970295755229421,
-0.978147628727239,-0.984807778990161,-0.990268091644056,-0.994521914133349,-0.997564063826147,-0.99939083432848,-0.999999999999975,-0.99939081866546,
-0.997564032519191,-0.994521867220599,-0.990268029182668,-0.984807701056235,-0.978147535415725,-0.970295646654006,-0.961261601096242,-0.95105640534479,
-0.939692492869339,-0.927183708858632,-0.913545293352216,-0.89879386267279,-0.882947389181757,-0.866025179382651,-0.848047850399114,-0.829037304856091,
-0.809016704194826,-0.788010440454194,-0.766044106552729,-0.743144465107575,-0.719339415828325,-0.694657961525496,-0.669130172775041,-0.642787151281955,
-0.615660991987602,-0.587784743966946,-0.559192370163299,-0.52991870600968,-0.499999416987166,-0.469470955171958,-0.438370514824109,-0.406735987072001,
-0.374605913747807,-0.342019440430165,-0.309016268751277,-0.275636608026548,-0.241921126265685,-0.207910900624944,-0.17364736736091,-0.139172271346751,
-0.104527615212477,-0.0697556081711626,-0.0348986145934809,8.9760680463878E-07,0.0349004087134927,0.0697573990117171,0.104529400591709,0.139174049089448,
0.173649135301166,0.207912656608794,0.24192286815373,0.275638333696568,0.309017976100803,0.342021127379053,0.374607578240769,0.406737627081107,
0.438372128351259,0.469472540251316,0.500000971687554,0.529920228436935,0.559193858462576,0.58778619632498,0.615662406634917,0.642788526495018,
0.669131506878367,0.694659252893684,0.71934066288804,0.743145666339467,0.766045260493279,0.788011545697502,0.809017759394323,0.829038308726177,
0.848048801716728,0.866026076988757,0.88294823198276,0.898794649641866,0.913546023530563,0.92718438135664,0.939693106867672,0.951056960095384,
0.961262095923219,0.970296080954495,0.978147908660598,0.984808012790749,0.990268279027023,0.994522054870397,0.99756415774581,0.999390881316331,
0.999999999998766,0.999390771675194,0.997563938597117,0.994521726481147,0.990267841797308,0.984807467253267,0.978147255480002,0.970295320926587,
0.961261229973976,0.951055989279832,0.939692032368603,0.927183204483165,0.913544745716523,0.898793272444082,0.882946757079138,0.866024506176239,
0.848047136909109,0.829036551951772,0.809015912793492,0.788009611520046,0.766043241095696,0.743143564182084,0.719338480532016,0.694656992997885,
0.669129172196131,0.642786119870798,0.615659931000815,0.587783654697176,0.559191253937658,0.529917564188118,0.499998250960817,0.469469766361446,
0.438369304677817,0.406734757064309,0.374604665377292,0.342018175217775,0.309014988238479,0.275635313773452,0.241919819849136,0.207909583636615,
0.17364604140535,0.139170938039433,0.104526276177831,0.0697542650405974,0.0348972690033973,-2.24401701095211E-06,-0.0349017543034278,-0.0697587421419864,
-0.104530739625913,-0.139175382396179,-0.173650461255992,-0.207913973596242,-0.241924174569255,-0.275639627948499,-0.309019256612295,-0.342022392589997,
-0.374608826609699,-0.406738857087076,-0.438373338495695,-0.469473729059843,-0.500002137711789,-0.529921370256256,-0.559194974685851,-0.587787285592262,
-0.615663467619101,-0.642789557903458,-0.669132507454447,-0.694660221418356,-0.719341598181306,-0.743146567261814,-0.766046125947073,-0.788012374628317,
-0.809018550792235,-0.829039061626989,-0.848049515203145},
{-2.44921270764475E-16,
0.0348995116535011,0.0697565035914672,0.104528507902134,0.139173160218155,0.173648251331108,0.207911778616952,0.241921997209805,0.275637470861669,
0.309017122426165,0.342020283904746,0.374606745994438,0.406736807076717,0.43837132158786,0.469471747711826,0.500000194337561,0.529919467223521,
0.559193114313163,0.587785470146199,0.615661699311508,0.642787838888745,0.669130839826974,0.694658607209869,0.719340039358472,0.74314506572382,
0.766044683523312,0.788010993076165,0.8090172317949,0.829037806791468,0.848048326058263,0.866025628186052,0.882947810582614,0.89879425615769,
0.913545658441758,0.927184045108009,0.939692799868884,0.95105668272047,0.961261848510118,0.970295863804641,0.978147722038556,0.984807856923889,
0.990268154105244,0.994521961045899,0.997564095132902,0.999390849991298,0.999999999999773,0.999390803002239,0.997564001212034,0.994521820307649,
0.990267966721081,0.984807623122111,0.978147442104015,0.970295538078395,0.961261477389013,0.951056266656662,0.939692339369283,0.927183540733663,
0.913545110807169,0.898793665930068,0.882947178481062,0.866024954980688,0.848047612569283,0.829037053888151,0.809016440394543,0.788010164142969,
0.766043818067206,0.743144164799229,0.719339104063033,0.694657638683099,0.669129839248871,0.642786807478365,0.615660638325464,0.587784380877141,
0.559191998088198,0.529918325402598,0.499999028311816,0.469470558901882,0.438370111442101,0.406735577069519,0.374605497624377,0.342019018692769,
0.30901584191374,0.275636176608906,0.24192069079355,0.207910461628876,0.173646925375757,0.139171826911005,0.104527168867616,0.0697551604609877,
0.0348981660634601,-1.34641020784612E-06,-0.0349008572434792,-0.0697578467218208,-0.104529846936463,-0.139174493525054,-0.173649577286143,-0.207913095604653,
-0.241923303625621,-0.275638765113933,-0.30901840293803,-0.342021549116104,-0.374607994363822,-0.406738037083179,-0.438372531732826,-0.469472936520918,
-0.5000013603624,-0.529920609043482,-0.559194230537114,-0.587786559414192,-0.615662760296435,-0.642788870297961,-0.669131840403862,-0.694659575735382,
-0.719340974652608,-0.743145966647065,-0.766045548978031,-0.788011822007933,-0.809018023193791,-0.829038559693282,-0.848049039545705,-0.866026301389847,
-0.882948442682567,-0.898794846383683,-0.913546206074691,-0.927184549480675,-0.939693260366782,-0.951057098782554,-0.961262219629479,-0.970296189529129,
-0.978148001971323,-0.984808090723882,-0.990268341487612,-0.994522101782346,-0.997564189051963,-0.999390896978545,-0.999999999997961,-0.999390756011369,
-0.997563907289357,-0.994521679567596,-0.990267779335122,-0.984807389318548,-0.978147162167701,-0.97029521235039,-0.961261106266167,-0.95105585059113,
-0.939691878867978,-0.927183036357636,-0.913544563170925,-0.898793075700817,-0.882946546377907,-0.866024281773752,-0.848046899078766,-0.829036300983332,
-0.809015648992721,-0.788009335208345,-0.766042952609709,-0.743143263873288,-0.719338168766291,-0.694656670155069,-0.669128838669557,-0.642785776066818,
-0.615659577338303,-0.587783291607017,-0.55919088186222,-0.529917183580716,-0.499997862285165,-0.469469370091086,-0.438368901295545,-0.406734347061583,
-0.374604249253637,-0.342017753480173,-0.309014561400754,-0.27563488235564,-0.241919384376857,-0.207909144640423,-0.173645599420093,-0.139170493603602,
-0.104525829832906,-0.0697538173303821,-0.0348968204733563}
};



//int sin_divider = 2;
int phase_A_position = 0;
int phase_B_position = 119;
int phase_C_position = 239;
int step_delay  = 100;
char stepper_sine = 0;
long max_sin_inc = 5;
int forward = 1;
int gate_drive_offset = 60;

int stuckcounter = 0;
int k_erpm;
uint16_t e_rpm;      // electrical revolution /100 so,  123 is 12300 erpm

uint16_t adjusted_duty_cycle;

int bad_count = 0;
int dshotcommand;
int armed_count_threshold = 1000;

char armed = 0;
int zero_input_count = 0;

int input = 0;
int newinput =0;
char inputSet = 0;
char dshot = 0;
char servoPwm = 0;
int zero_crosses;

int zcfound = 0;

int bemfcounter;
int min_bemf_counts_up = 7;
int min_bemf_counts_down = 7;
int adc_timer = 600;
int lastzctime;
uint16_t thiszctime;
int phase = 1;
int duty_cycle = 0;
char step = 1;
uint16_t commutation_interval = 12500;
int pwm = 1;
int floating =2;
int lowside = 3;
int sensorless = 1;
uint32_t waitTime = 0;
int signaltimeout = 0;

uint8_t ubAnalogWatchdogStatus = RESET;


void checkForHighSignal(){
	changeToInput();
	LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_DOWN);
	delayMicros(1000);
	for(int i = 0 ; i < 1000; i ++){
		if( !(INPUT_PIN_PORT->IDR & INPUT_PIN)){  // if the pin is low for 5 checks out of 100 in  100ms or more its either no signal or signal. jump to application
			low_pin_count++;
		}
		delayMicros(10);
	}
	LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_NO);
	if(low_pin_count > 5){
		return;      // its either a signal or a disconnected pin
	}
	else{
		allOff();
		NVIC_SystemReset();
	}
}

void loadEEpromSettings(){
	read_flash_bin( eepromBuffer , EEPROM_START_ADD , 48);

	if(eepromBuffer[17] == 0x01){
		dir_reversed =  1;
	}
	else{
		dir_reversed = 0;
	}

	if(eepromBuffer[21] == 0x01){
		VARIABLE_PWM = 1;
	}
	else{
		VARIABLE_PWM = 0;
	}

	if(eepromBuffer[23] < 4){
		advance_level = eepromBuffer[23];
	}
	else{
		advance_level = 2;  // * 7.5 increments
	}

	if(eepromBuffer[24] < 49 && eepromBuffer[24] > 23){
		TIMER1_MAX_ARR = map (eepromBuffer[24], 24, 48, TIM1_AUTORELOAD ,TIM1_AUTORELOAD/2);
		TIM1->ARR = TIMER1_MAX_ARR;	   
	}
	else{
		tim1_arr = TIM1_AUTORELOAD;
		TIM1->ARR = tim1_arr;
	}

	if(eepromBuffer[25] < 151 && eepromBuffer[25] > 49){
		minimum_duty_cycle = (eepromBuffer[25] / 2) + (eepromBuffer[26] / 3);
	}
	else{
		minimum_duty_cycle = 150;
	}

	motor_kv = (eepromBuffer[26] * 40) + 20;
	motor_poles = eepromBuffer[27];

	if(eepromBuffer[28] == 0x01){
		brake_on_stop = 1;
	}
	else{
		brake_on_stop = 0;
	}

	if(eepromBuffer[29] == 0x01){
		stall_protection = 1;
	}
	else{
		stall_protection = 0;
	}

	setVolume(5);

	if(eepromBuffer[1] > 0){             // these commands weren't introduced until eeprom version 1.
		if(eepromBuffer[30] > 11){
			setVolume(5);
		}
		else{
			setVolume(eepromBuffer[30]);
		}

		if(eepromBuffer[31] == 0x01){
			THIRTY_TWO_MS_TLM = 1;
		}
		else{
			THIRTY_TWO_MS_TLM = 0;
		}

		servo_low_threshold = (eepromBuffer[32]*2) + 750; // anything below this point considered 0
		servo_high_threshold = (eepromBuffer[33]*2) + 1750;;  // anything above this point considered 2000 (max)
		servo_neutral = (eepromBuffer[34]) + 1374;
		servo_dead_band = eepromBuffer[35];

		if(eepromBuffer[36] == 0x01){
			LOW_VOLTAGE_CUTOFF = 1;
		}
		else{
			LOW_VOLTAGE_CUTOFF = 0;
		}

		low_cell_volt_cutoff = eepromBuffer[37] + 250; // 2.5 to 3.5 volts per cell range
		   
		if(eepromBuffer[39] == 0x01){
			#ifdef HAS_HALL_SENSORS
			USE_HALL_SENSOR = 1;
			#else
			USE_HALL_SENSOR = 0;
			#endif
		}
		else{
			USE_HALL_SENSOR = 0;
		}

		if(eepromBuffer[40] > 4 && eepromBuffer[40] < 26){            // sine mode changeover 5-25 percent throttle
			sine_mode_changeover_thottle_level = eepromBuffer[40];
			sine_mode_changeover = map(sine_mode_changeover_thottle_level, 5, 25, ((TIM1_AUTORELOAD + 1) / 100) * 5, ((TIM1_AUTORELOAD + 1) / 100) * 25);
		}

		if(eepromBuffer[41] > 0 && eepromBuffer[41] < 11){        // drag brake 0-10
			drag_brake_strength = eepromBuffer[41];
		}
	}


	if(motor_kv < 300){
		low_rpm_throttle_limit = 0;
	}
	low_rpm_level  = motor_kv / 200 / (16 / motor_poles);
	high_rpm_level = (40 + (motor_kv / 100)) / (16/motor_poles);
}

void saveEEpromSettings(){
	if(dir_reversed == 1){
		eepromBuffer[17] = 0x01;
	}
	else{
		eepromBuffer[17] = 0x00;
	}

	eepromBuffer[19] = 0x01;

	if(VARIABLE_PWM == 1){
		eepromBuffer[21] = 0x01;
	}
	else{
		eepromBuffer[21] = 0x00;
	}

	eepromBuffer[23] = advance_level;

	save_flash_nolib(eepromBuffer, 48, EEPROM_START_ADD);
}





void getSmoothedInput() {
	total = total - readings[readIndex];
	readings[readIndex] = commutation_interval;
	total = total + readings[readIndex];
	readIndex = readIndex + 1;
	
	if (readIndex >= numReadings) {
		readIndex = 0;
	}
	smoothedinput = total / numReadings;
}

void getBemfState(){
	if (rising){
		if (LL_COMP_ReadOutputLevel(MAIN_COMP) == LL_COMP_OUTPUT_LEVEL_LOW){
			bemfcounter++;
		}
		else{
			bad_count++;
			if(bad_count > 2){
				bemfcounter = 0;
			}
		}
	}
	else{
		if(LL_COMP_ReadOutputLevel(MAIN_COMP) == LL_COMP_OUTPUT_LEVEL_HIGH){
			bemfcounter++;
		}
		else{
			bad_count++;
			if(bad_count > 2){
				bemfcounter = 0;
			}
		}
	}
}

void commutate(){
	commutation_intervals[step-1] = commutation_interval;
	e_com_time = (commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] +commutation_intervals[5]) >> 1;  // COMMUTATION INTERVAL IS 0.5US INCREMENTS

	//	COM_TIMER->CNT = 0;
	if (forward == 1){
		step++;
		if (step > 6) {
			step = 1;
			desync_check = 1;
		}
		rising = step % 2;
	}
	else{
		step--;
		if (step < 1) {
			step = 6;
			desync_check = 1;
		}
		rising = !(step % 2);
	}

	if(!prop_brake_active){
		comStep(step);
	}

	changeCompInput();

	if(average_interval > 2000 && stall_protection){
		old_routine = 1;
	}

	bemfcounter = 0;
	zcfound = 0;
	timeout_count = 0;
}

void PeriodElapsedCallback(){
	COM_TIMER->DIER &= ~((0x1UL << (0U)));             // disable interrupt
	commutation_interval = (( 3*commutation_interval) + thiszctime)>>2;
	commutate();
	advance = (commutation_interval>>3) * advance_level;   // 60 divde 8 7.5 degree increments
	waitTime = (commutation_interval >>1)  - advance;

	if(!old_routine){
		enableCompInterrupts();     // enable comp interrupt
	}

	if(zero_crosses<10000){
		zero_crosses++;
	}
	//	UTILITY_TIMER->CNT = 0;
}


void interruptRoutine(){
	if (average_interval > 125){
		if ((INTERVAL_TIMER->CNT < 125) && (duty_cycle < 600) && (zero_crosses < 500)){    //should be impossible, desync?exit anyway
			return;
		}

		if (INTERVAL_TIMER->CNT < (commutation_interval >> 1)){
			return;
		}

		stuckcounter++;             // stuck at 100 interrupts before the main loop happens again.
		if (stuckcounter > 100){
			maskPhaseInterrupts();
			zero_crosses = 0;
			return;
		}
	}

	thiszctime = INTERVAL_TIMER->CNT;

	if (rising){
		for (int i = 0; i < filter_level; i++){
			if(LL_COMP_ReadOutputLevel(MAIN_COMP) == LL_COMP_OUTPUT_LEVEL_HIGH){
			return;
			}
		}
	}
	else{
		for (int i = 0; i < filter_level; i++){
			if(LL_COMP_ReadOutputLevel(MAIN_COMP) == LL_COMP_OUTPUT_LEVEL_LOW){
				return;
			}
		}
	}
	maskPhaseInterrupts();
	INTERVAL_TIMER->CNT = 0 ;

	waitTime = waitTime >> fast_accel;

	COM_TIMER->CNT = 0;
	COM_TIMER->ARR = waitTime;
	COM_TIMER->SR = 0x00;
	COM_TIMER->DIER |= (0x1UL << (0U));             // enable COM_TIMER interrupt
}

void startMotor() {
	if (running == 0){
		commutate();
		commutation_interval = 10000;
		INTERVAL_TIMER->CNT = 5000;
		running = 1;
	}
	enableCompInterrupts();
	sensorless = 1;
}




void tenKhzRoutine(){
	consumption_timer++;

	if(consumption_timer > 10000){      // 1s sample interval
		consumed_current = (float)actual_current/3600 + consumed_current;
		consumption_timer = 0;
	}


	if(!armed && inputSet){
		if(adjusted_input == 0){
			armed_timeout_count++;
			if(armed_timeout_count > 10000){    // one second
				if(zero_input_count > 30){
					armed = 1;
					#ifdef tmotor55
					GPIOB->BRR = LL_GPIO_PIN_3;    // turn off red
					GPIOA->BSRR = LL_GPIO_PIN_15;   // turn on green
					#endif
					if(cell_count == 0 && LOW_VOLTAGE_CUTOFF){
						cell_count = battery_voltage / 370;
						for (int i = 0 ; i < cell_count; i++){
							playInputTune();
							delayMillis(100);
							LL_IWDG_ReloadCounter(IWDG);
						}
					}
					else{
						playInputTune();
					}
				}
				else{
					inputSet = 0;
					armed_timeout_count =0;
				}
			}
		}
		else{
			armed_timeout_count =0;
		}
	}

	if(THIRTY_TWO_MS_TLM){
		thirty_two_ms_count++;
		if(thirty_two_ms_count>320){
			send_telemetry = 1;
			thirty_two_ms_count = 0;
		}
	}

	if(!stepper_sine){
		if (input >= 127 && armed){
			if (running == 0){
				allOff();
				if(!old_routine){
					startMotor();
				}
				running = 1;
				last_duty_cycle = minimum_duty_cycle;
				#ifdef tmotor55
				GPIOB->BRR = LL_GPIO_PIN_3;  // off red
				GPIOA->BRR = LL_GPIO_PIN_15; // off green
				GPIOB->BSRR = LL_GPIO_PIN_5;  // on blue
				#endif
			}
	  
			duty_cycle = map(input, sine_mode_changeover, 2047, minimum_duty_cycle, TIMER1_MAX_ARR);
			prop_brake_active = 0;
		}

		if (input < 47){
			if(play_tone_flag != 0){
				if(play_tone_flag == 1){
					playDefaultTone();
				}
				if(play_tone_flag == 2){
					playChangedTone();
				}

				play_tone_flag = 0;
			}

			if (!running){
				duty_cycle = 0;
				old_routine = 1;
				zero_crosses = 0;
				bad_count = 0;
				if(!brake_on_stop){		  
					allOff();
					duty_cycle = 0;
				}
			}
			phase_A_position = 0;
			phase_B_position = 119;
			phase_C_position = 239;
			stepper_sine = 1;		  
		}
		else if (input < ((sine_mode_changeover / 10) * 9)) {
			phase_A_position = 0;
			phase_B_position = 119;
			phase_C_position = 239;
			zero_crosses = 0;
			stepper_sine = 1;
		}

		if(!prop_brake_active){
			if (zero_crosses < (20 >> stall_protection)){
				if (duty_cycle < minimum_duty_cycle){
					duty_cycle = minimum_duty_cycle;
				}
				//	   if (duty_cycle > 200<<stall_protection){
				//		   duty_cycle = 200<<stall_protection;
				//	   }
			}

			if (running){
				if(stall_protection){  // this boosts throttle as the rpm gets lower, for crawlers and rc cars only, do not use for multirotors.
					//minimum_duty_cycle = eepromBuffer[25];
					velocity_count++;
					if (velocity_count > velocity_count_threshold){
						if(commutation_interval > 9000){
						// duty_cycle = duty_cycle + map(commutation_interval, 10000, 12000, 1, 100);
							minimum_duty_cycle ++;
						}
						else{
						//minimum_duty_cycle--;
						}

						if(minimum_duty_cycle > (minimum_duty_cycle + (minimum_duty_cycle / 3))){
							minimum_duty_cycle = minimum_duty_cycle + (minimum_duty_cycle / 3);
						}

						velocity_count = 0;
					}
				}

				if (input < sine_mode_changeover) {
					duty_cycle = map(input, sine_mode_changeover, (sine_mode_changeover / 10) * 9, minimum_duty_cycle, (minimum_duty_cycle / 10) * 8);
				}

			}

			if (duty_cycle > duty_cycle_maximum){
				duty_cycle = duty_cycle_maximum;
			}

			if(maximum_throttle_change_ramp){
				//	max_duty_cycle_change = map(k_erpm, low_rpm_level, high_rpm_level, 1, 40);
				if(average_interval > 500){
					max_duty_cycle_change = 10;
				}
				else{
					max_duty_cycle_change = 30;
				}

				if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change){
					duty_cycle = last_duty_cycle + max_duty_cycle_change;

					if(commutation_interval > 500){
						fast_accel = 1;
					}
					else{
						fast_accel = 0;
					}
				}
				else if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change){
					duty_cycle = last_duty_cycle - max_duty_cycle_change;
					fast_accel = 0;
				}
				else{
					fast_accel = 0;
				}
			}
		}

		if (armed && running && (input > 47)){
			if(VARIABLE_PWM){
				tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR/2, TIMER1_MAX_ARR);
				advance_level = eepromBuffer[23];
			}
			adjusted_duty_cycle = ((duty_cycle * tim1_arr)/TIMER1_MAX_ARR)+1;
		}
		else{
			if(prop_brake_active){
				adjusted_duty_cycle = TIMER1_MAX_ARR - ((duty_cycle * tim1_arr)/TIMER1_MAX_ARR)+1;
			}
			else{
				adjusted_duty_cycle = 0;
			}
		}

		last_duty_cycle = duty_cycle;

		TIM1->ARR = tim1_arr;
		TIM1->CCR1 = adjusted_duty_cycle;
		TIM1->CCR2 = adjusted_duty_cycle;
		TIM1->CCR3 = adjusted_duty_cycle;
	}

	average_interval = e_com_time / 3;

	if(desync_check && zero_crosses > 10){
		//	if(average_interval < last_average_interval){
		//
		//	}
		if((getAbsDif(last_average_interval,average_interval) > average_interval>>1) && (average_interval < 1000)){ //throttle resitricted before zc 20.
			zero_crosses = 10;
			desync_happened ++;
			//running = 0;
			//old_routine = 1;
			//last_duty_cycle = minimum_duty_cycle/2;
		}

		desync_check = 0;
		//	}
		last_average_interval = average_interval;
	}

	if(send_telemetry){
		#ifdef	USE_SERIAL_TELEMETRY
		makeTelemPackage(degrees_celsius,
		battery_voltage,
		actual_current,
		(uint16_t)consumed_current/10,
		e_rpm);
		send_telem_DMA();
		send_telemetry = 0;
		#endif
	}

	if(commutation_interval > 400){
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
		NVIC_SetPriority(ADC1_COMP_IRQn, 1);
	}
	else{
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
		NVIC_SetPriority(ADC1_COMP_IRQn, 0);
	}

	signaltimeout++;
	if(signaltimeout > 2500 * (servoPwm+1)) { // quarter second timeout when armed half second for servo;
		if(armed){
			allOff();
			armed = 0;
			input = 0;
			inputSet = 0;
			zero_input_count = 0;
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			IC_TIMER_REGISTER->PSC = 0;
			IC_TIMER_REGISTER->CNT = 0;
			for(int i = 0; i < 64; i++){
			dma_buffer[i] = 0;
			}
			NVIC_SystemReset();
		}

		if (signaltimeout > 25000){     // 2.5 second
			allOff();
			armed = 0;
			input = 0;
			inputSet = 0;
			zero_input_count = 0;
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			IC_TIMER_REGISTER->PSC = 0;
			IC_TIMER_REGISTER->CNT = 0;
			for(int i = 0; i < 64; i++){
			dma_buffer[i] = 0;
			}
			NVIC_SystemReset();
		}
	}
}

void advanceincrement(int input){

	char inc = map(input, 47, sine_mode_changeover, 1, 2);

	if (forward){
		phase_A_position += inc;
		if (phase_A_position > 179){
			phase_A_position -= 180;
			sin_cycle_complete = 1;
		}

		phase_B_position += inc;
		if (phase_B_position > 179){
			phase_B_position -= 180;
		}

		phase_C_position += inc;
		if (phase_C_position > 179){
			phase_C_position -= 180;
		}
	}
	else{
		phase_A_position -= inc;
		if (phase_A_position < 0){
			phase_A_position += 180;
			sin_cycle_complete = 1;
		}

		phase_B_position -= inc;
		if (phase_B_position < 0){
			phase_B_position += 180;
		}

		phase_C_position -= inc;
		if (phase_C_position < 0){
			phase_C_position += 180;
		}
	}

	TIM1->CCR1 = (amplitude * pwmSin[0][phase_A_position]) + (amplitude + 2);
	TIM1->CCR2 = (amplitude * pwmSin[1][phase_B_position]) + (amplitude + 2);
	TIM1->CCR3 = (amplitude * pwmSin[2][phase_C_position]) + (amplitude + 2);

	//TIM1->CCR1 = ((2*pwmSin[phase_A_position]/SINE_DIVIDER)+ gate_drive_offset)*TIM1_AUTORELOAD/2000;
	//TIM1->CCR2 = ((2*pwmSin[phase_B_position]/SINE_DIVIDER)+ gate_drive_offset)*TIM1_AUTORELOAD/2000;
	//TIM1->CCR3 = ((2*pwmSin[phase_C_position]/SINE_DIVIDER)+ gate_drive_offset)*TIM1_AUTORELOAD/2000;
    
}

void zcfoundroutine(){   // only used in polling mode, blocking routine.
	thiszctime = INTERVAL_TIMER->CNT;
	INTERVAL_TIMER->CNT = 0;
	commutation_interval = (thiszctime + (3*commutation_interval)) / 4;
	advance = commutation_interval / advancedivisor;
	waitTime = commutation_interval /2  - advance;
	//	blanktime = commutation_interval / 4;
	while (INTERVAL_TIMER->CNT - thiszctime < waitTime - advance){

	}

	commutate();
	bemfcounter = 0;
	bad_count = 0;

	zero_crosses++;
	if(stall_protection){
		if (zero_crosses >= 100 && commutation_interval <= 2000) {
			old_routine = 0;
			enableCompInterrupts();          // enable interrupt
		}
	}
	else{
		if(zero_crosses > 30){
			old_routine = 0;
			enableCompInterrupts();          // enable interrupt
		}
	}
}

void SwitchOver() {
	sin_cycle_complete = 0;
	stepper_sine = 0;
	running = 1;
	old_routine = 1;
	prop_brake_active = 0;
	commutation_interval = 9000;
	average_interval = 9000;
	last_average_interval = average_interval;
	//  minimum_duty_cycle = ;
	INTERVAL_TIMER->CNT = 9000;
	zero_crosses = 0;
	prop_brake_active = 0;

	adjusted_duty_cycle = ((duty_cycle * tim1_arr) / TIMER1_MAX_ARR) + 1;
	TIM1->ARR = tim1_arr;
	TIM1->CCR1 = adjusted_duty_cycle;
	TIM1->CCR2 = adjusted_duty_cycle;
	TIM1->CCR3 = adjusted_duty_cycle;

	step = changeover_step;              // rising bemf on a same as position 0.	
	comStep(step);
	changeCompInput();
	enableCompInterrupts();
	// rising bemf on a same as position 0.
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	zcfoundroutine();
}


int main(void)
{
	initAfterJump();

	initCorePeripherals();

	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

	/* Enable counter */
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableAllOutputs(TIM1);
	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	// LL_TIM_EnableIT_UPDATE(TIM1);
	#ifdef USE_ADC_INPUT

	#else
	//
	LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL);  // input capture and output compare
	LL_TIM_EnableCounter(IC_TIMER_REGISTER);
	#endif

	#ifdef tmotor55
	LED_GPIO_init();
	GPIOB->BSRR = LL_GPIO_PIN_3; // turn on red
	#endif

	LL_TIM_EnableCounter(COM_TIMER);               // commutation_timer priority 0
	LL_TIM_GenerateEvent_UPDATE(COM_TIMER);
	LL_TIM_EnableIT_UPDATE(COM_TIMER);
	COM_TIMER->DIER &= ~((0x1UL << (0U)));         // disable for now.
	//
	LL_TIM_EnableCounter(UTILITY_TIMER);
	LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
	//
	LL_TIM_EnableCounter(INTERVAL_TIMER);
	LL_TIM_GenerateEvent_UPDATE(INTERVAL_TIMER);

	LL_TIM_EnableCounter(TEN_KHZ_TIMER);                 // 10khz timer
	LL_TIM_GenerateEvent_UPDATE(TEN_KHZ_TIMER);
	TEN_KHZ_TIMER->DIER |= (0x1UL << (0U));  // enable interrupt

	//RCC->APB2ENR  &= ~(1 << 22);  // turn debug off
	#ifdef USE_ADC
	ADC_Init();
	enableADC_DMA();
	activateADC();
	#endif

	__IO uint32_t wait_loop_index = 0;
	/* Enable comparator */

	LL_COMP_Enable(MAIN_COMP);

	wait_loop_index = ((LL_COMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
	while(wait_loop_index != 0){
		wait_loop_index--;
	}

	loadEEpromSettings();
	//  EEPROM_VERSION = *(uint8_t*)(0x08000FFC);
	if(firmware_info.version_major != eepromBuffer[3] || firmware_info.version_minor != eepromBuffer[4]){
		eepromBuffer[3] = firmware_info.version_major;
		eepromBuffer[4] = firmware_info.version_minor;
		for(int i = 0; i < 12 ; i ++){
			eepromBuffer[5+i] = firmware_info.device_name[i];
		}
		saveEEpromSettings();
	}
	//  if(EEPROM_VERSION != eepromBuffer[2]){
	//	  eepromBuffer[2] = EEPROM_VERSION;
	//	  saveEEpromSettings();
	//  }

	if (dir_reversed == 1){
		forward = 0;
	}
	else{
		forward = 1;
	}

	tim1_arr = TIMER1_MAX_ARR;
	
	playStartupTune();
	zero_input_count = 0;
	MX_IWDG_Init();
	LL_IWDG_ReloadCounter(IWDG);

	#ifdef USE_ADC_INPUT
	armed_count_threshold = 5000;
	inputSet = 1;

	#else
	checkForHighSignal();     // will reboot if signal line is high for 10ms
	receiveDshotDma();
	#endif

	#ifdef MCU_F051
	MCU_Id = DBGMCU->IDCODE &= 0xFFF;
	REV_Id = DBGMCU->IDCODE >> 16;

	if(REV_Id >= 4096){
		temperature_offset = 0;
	}
	else{
		temperature_offset = 230;
	}
	#endif
	while (1){

		LL_IWDG_ReloadCounter(IWDG);

		adc_counter++;
		if(adc_counter>100){   // for testing adc and telemetry
			ADC_raw_temp = ADC_raw_temp - (temperature_offset);
			converted_degrees =__LL_ADC_CALC_TEMPERATURE(3300,  ADC_raw_temp, LL_ADC_RESOLUTION_12B);
			degrees_celsius =((7 * degrees_celsius) + converted_degrees) >> 3;

			battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER)/100)) >> 3;
			smoothed_raw_current = ((7*smoothed_raw_current + (ADC_raw_current) )>> 3);
			actual_current = ((smoothed_raw_current * 3300/4095) * MILLIVOLT_PER_AMP )/10  + CURRENT_OFFSET;

			LL_ADC_REG_StartConversion(ADC1);
			if(LOW_VOLTAGE_CUTOFF){
				if(battery_voltage < (cell_count * low_cell_volt_cutoff)){
					low_voltage_count++;
					if(low_voltage_count > 1000){
						input = 0;
						allOff();
						maskPhaseInterrupts();
						running = 0;
						zero_input_count = 0;
						armed = 0;
					}
				}
				else{
					low_voltage_count = 0;
				}
			}
			adc_counter = 0;

			#ifdef USE_ADC_INPUT
			if(ADC_raw_input < 10){
				zero_input_count++;
			}
			else{
				zero_input_count=0;
			}
			#endif
		}


		#ifdef USE_ADC_INPUT

		signaltimeout = 0;
		ADC_smoothed_input = (((10*ADC_smoothed_input) + ADC_raw_input)/11);
		newinput = ADC_smoothed_input / 2;
		if(newinput > 2000){
			newinput = 2000;
		}
		#endif
		stuckcounter = 0;

		if (dshot == 0){
			if (newinput > (1000 + (servo_dead_band<<1))) {
				if (forward == dir_reversed) {
					if(commutation_interval > 1500 || stepper_sine){
						forward = 1 - dir_reversed;
						zero_crosses = 0;
						old_routine = 1;
						maskPhaseInterrupts();
					}
					else{
						newinput = 1000;
					}
				}
				adjusted_input = map(newinput, 1000 + (servo_dead_band<<1), 2000, 47, 2047);
			}
			else if (newinput < (1000 -(servo_dead_band<<1))) {
				if (forward == (1 - dir_reversed)) {
					if(commutation_interval > 1500 || stepper_sine){
						zero_crosses = 0;
						old_routine = 1;
						forward = dir_reversed;
						maskPhaseInterrupts();
					}
					else{
						newinput = 1000;
					}
				}
				adjusted_input = map(newinput, 0, 1000-(servo_dead_band<<1), 2047, 47);
			}
			else if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band <<1))) {
				adjusted_input = 0;
			}  			  
		}
		else if (dshot) {
			if (newinput > 1047) {
				if (forward == dir_reversed) {
					if(commutation_interval > 1500 || stepper_sine){
						forward = 1 - dir_reversed;
						zero_crosses = 0;
						old_routine = 1;
						maskPhaseInterrupts();
					}
					else{
						newinput = 0;
					}
				}

				adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
			}
			else if (newinput <= 1047  && newinput > 47) {
			//	startcount++;
				if (forward == (1 - dir_reversed)) {
					if(commutation_interval > 1500 || stepper_sine){
						zero_crosses = 0;
						old_routine = 1;
						forward = dir_reversed;
						maskPhaseInterrupts();
					}
					else{
						newinput = 0;
					}
				}
				adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
			}
			else if ( newinput < 48) {
				adjusted_input = 0;
			}
		}
		else{
			adjusted_input = newinput;
		}

		if ((zero_crosses > 1000) || (adjusted_input == 0)){
			bemf_timout_happened = 0;
			#ifdef tmotor55
			if(adjusted_input == 0 && armed){
			GPIOA->BSRR = LL_GPIO_PIN_15; // on green
			GPIOB->BRR = LL_GPIO_PIN_5;  // off blue
			GPIOB->BRR = LL_GPIO_PIN_3;  //off red
			}
			#endif
		}

		if(zero_crosses > 100 && adjusted_input < 200){
			bemf_timout_happened = 0;
		}

		if(adjusted_input < 160){
			bemf_timout_happened = 0;
		}
 	 	 
		if (adjusted_input < 150){              // startup duty cycle should be low enough to not burn motor
			bemf_timeout = 100;
		}
		else{
			bemf_timeout = 10;
		}
	  	  	
		if(adjusted_input < 30){           // dead band ?
			input= 0;
		}
		else {
			input = map(adjusted_input, 30, 2000, 47, 2000);
		}
	 	  
		if ( stepper_sine == 0){
			e_rpm = running * (100000/ e_com_time) * 6;
			k_erpm =  e_rpm / 10; // ecom time is time for one electrical revolution in microseconds
			if(low_rpm_throttle_limit){     // some hardware doesn't need this, its on by default to keep hardware / motors protected but can slow down the response in the very low end a little.
				duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, throttle_max_at_low_rpm, throttle_max_at_high_rpm);   // for more performance lower the high_rpm_level, set to a consvervative number in source.
			}

			if (zero_crosses < 100 || commutation_interval > 500) {
				filter_level = 12;
			} 
			else {
				filter_level = map(average_interval, 100 , 500, 3 , 8);
			}

			if (commutation_interval < 100){
				filter_level = 2;
			}

			if(lowkv){
				filter_level = low_kv_filter_level;
			}

			/**************** old routine*********************/
			if (old_routine && running){
				maskPhaseInterrupts();
				getBemfState();
				if (!zcfound){
					if (rising){
						if (bemfcounter > min_bemf_counts_up){
							zcfound = 1;
							zcfoundroutine();
						}
					}
					else{
						if (bemfcounter > min_bemf_counts_down){
							zcfound = 1;
							zcfoundroutine();
						}
					}
				}
			}
			if (INTERVAL_TIMER->CNT > 45000 && running == 1){
				bemf_timout_happened ++;
				zcfoundroutine();
				maskPhaseInterrupts();
				old_routine = 1;
				running = 0;
				zero_crosses = 0;
			}
		}
		else{            // stepper sine
			if(input >= 47 && armed){
				maskPhaseInterrupts();
				allpwm();
				advanceincrement(input);
				step_delay = map (input, 48, sine_mode_changeover, 350, 20);
				
				if (input > sine_mode_changeover && sin_cycle_complete == 1){
					duty_cycle = map(input, sine_mode_changeover, 2047, minimum_duty_cycle, TIMER1_MAX_ARR);
					SwitchOver();
				}
				else {
					delayMicros(step_delay);
				}
			}
			else{
				if(brake_on_stop){
					#ifndef PWM_ENABLE_BRIDGE
					duty_cycle = (TIMER1_MAX_ARR-19) + drag_brake_strength*2;
					adjusted_duty_cycle = TIMER1_MAX_ARR - ((duty_cycle * tim1_arr)/TIMER1_MAX_ARR)+1;
					TIM1->CCR1 = adjusted_duty_cycle;
					TIM1->CCR2 = adjusted_duty_cycle;
					TIM1->CCR3 = adjusted_duty_cycle;
					proportionalBrake();
					prop_brake_active = 1;
					#else
					// todo add braking for PWM /enable style bridges.
					#endif 
				}
				else{
					TIM1->CCR1 = 0;
					TIM1->CCR2 = 0;
					TIM1->CCR3 = 0;
					allOff();
				}
			}	 			
		}
	}
}



void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
/* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

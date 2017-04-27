//Input Defines
#define FSW_DETECT		GpioDataRegs.GPBDAT.bit.GPIO40
#define HSW_LARGE		GpioDataRegs.GPADAT.bit.GPIO26
#define HSW_SMALL		GpioDataRegs.GPADAT.bit.GPIO27
#define FSW_LARGE		GpioDataRegs.GPBDAT.bit.GPIO32
#define FSW_SMALL		GpioDataRegs.GPBDAT.bit.GPIO33
#define XDUCER_DETECT	GpioDataRegs.GPCDAT.bit.GPIO81
#define TWENTY_AMP_OFF	GpioDataRegs.GPCDAT.bit.GPIO75
#define FIVE_AMP_OFF	GpioDataRegs.GPCDAT.bit.GPIO76
#define OVERVOLTS_OFF	GpioDataRegs.GPCDAT.bit.GPIO77
#define AGC_AMP_OFF		GpioDataRegs.GPCDAT.bit.GPIO78

//Output Defines
#define WD_BONE_ON		GpioDataRegs.GPASET.bit.GPIO12 		= 1										// Toggle this to reset hardware watchdog (every 3.4ms)
#define WD_BONE_OFF		GpioDataRegs.GPACLEAR.bit.GPIO12 	= 1

#define ULTRASOUND_OFF	GpioDataRegs.GPCSET.bit.GPIO68 		= 1										// Turns the system off
#define ULTRASOUND_ON	GpioDataRegs.GPCCLEAR.bit.GPIO68 	= 1										// Turns the system on
#define ULTRA_ON		1
#define ULTRA_OFF		0

#define DIG_FREQ_OFF	GpioDataRegs.GPBSET.bit.GPIO36 		= 1										// Turn off Digital freq and allow use of analog freq (4046 PLL chip)
#define DIG_FREQ_ON		GpioDataRegs.GPBCLEAR.bit.GPIO36 	= 1										// Turn on Digital AGC

#define ANA_FREQ_ON		GpioDataRegs.GPBSET.bit.GPIO37 		= 1										// Turn on Analog freq (4046 PLL chip)
#define ANA_FREQ_OFF	GpioDataRegs.GPBCLEAR.bit.GPIO37 	= 1										// Turn off analog freq and allow use of digital freq

#define DIG_AGC_OFF		GpioDataRegs.GPASET.bit.GPIO13 		= 1										// Turn off Digital AGC and allow use of analog AGC
#define DIG_AGC_ON		GpioDataRegs.GPACLEAR.bit.GPIO13 	= 1										// Turn on Digital AGC

#define ANA_AGC_ON		GpioDataRegs.GPBSET.bit.GPIO35 		= 1										// Turn on Analog AGC
#define ANA_AGC_OFF		GpioDataRegs.GPBCLEAR.bit.GPIO35 	= 1										// Turn off analog AGC and allow use of digital AGC

#define DSP_RESET_PA_H	GpioDataRegs.GPBSET.bit.GPIO52 		= 1										// Reset SR Latch on Power Board (Bridge and AGC Errors)
#define DSP_RESET_PA_L	GpioDataRegs.GPBCLEAR.bit.GPIO52	= 1										// Use to clear reset on SR Latch on Power Board

#define SYS_ON_ON_2		GpioDataRegs.GPBSET.bit.GPIO59		= 1										// Use to turn on/open LBB127 on power board
#define SYS_ON_OFF_2	GpioDataRegs.GPBCLEAR.bit.GPIO59	= 1										// Use to turn off/close LBB127 on power board for proper shutdown of system


//Front Panel LEDs (Active Low) - Defines
#define CHKPRB_WHT_OFF	GpioDataRegs.GPBSET.bit.GPIO42 		= 1										// Check Probe White LED, on when no error present
#define CHKPRB_WHT_ON	GpioDataRegs.GPBCLEAR.bit.GPIO42 	= 1

#define CHKPRB_RED_OFF	GpioDataRegs.GPBSET.bit.GPIO43 		= 1										// Check Probe Red LED, on only when there is a probe error
#define CHKPRB_RED_ON	GpioDataRegs.GPBCLEAR.bit.GPIO43 	= 1

#define FSW_WHT_OFF		GpioDataRegs.GPBSET.bit.GPIO44 		= 1										// Footswitch White LED, on only when footswitch is not connected
#define FSW_WHT_ON		GpioDataRegs.GPBCLEAR.bit.GPIO44 	= 1

#define FSW_GRN_OFF		GpioDataRegs.GPBSET.bit.GPIO45 		= 1										// Footswitch Green LED, on only when footswitch is connected
#define FSW_GRN_ON		GpioDataRegs.GPBCLEAR.bit.GPIO45 	= 1

#define SMSTN_WHT_OFF	GpioDataRegs.GPBSET.bit.GPIO46 		= 1										// Large Stone White LED, on only when large stone is not activated
#define SMSTN_WHT_ON	GpioDataRegs.GPBCLEAR.bit.GPIO46 	= 1

#define SMSTN_GRN_OFF	GpioDataRegs.GPBSET.bit.GPIO47 		= 1										// Large Stone Green LED, on only when large stone is activiated and ultrasonics is on
#define SMSTN_GRN_ON	GpioDataRegs.GPBCLEAR.bit.GPIO47 	= 1

#define LGSTN_WHT_OFF	GpioDataRegs.GPCSET.bit.GPIO80 		= 1										// Small Stone White LED, on only when small stone is not activated
#define LGSTN_WHT_ON	GpioDataRegs.GPCCLEAR.bit.GPIO80 	= 1

#define LGSTN_GRN_OFF	GpioDataRegs.GPCSET.bit.GPIO64 		= 1										// Small Stone Green LED, on only when small stone is activiated and ultrasonics is on
#define LGSTN_GRN_ON	GpioDataRegs.GPCCLEAR.bit.GPIO64 	= 1

#define ERROR_WHT_OFF	GpioDataRegs.GPCSET.bit.GPIO65 		= 1										// Error White LED, on only when no errors are present
#define ERROR_WHT_ON	GpioDataRegs.GPCCLEAR.bit.GPIO65 	= 1

#define ERROR_RED_OFF	GpioDataRegs.GPCSET.bit.GPIO66 		= 1										// Error Red LED, on only where there is a need for user interaction
#define ERROR_RED_ON	GpioDataRegs.GPCCLEAR.bit.GPIO66 	= 1

#define LBLS_WHT_OFF	GpioDataRegs.GPCSET.bit.GPIO67 		= 1										// Labels White LED, on all the time, light for text labels
#define LBLS_WHT_ON		GpioDataRegs.GPCCLEAR.bit.GPIO67 	= 1


//ePWM Defines
#define R1_ePWM 		EPwm1Regs.CMPA.half.CMPA
#define R2_ePWM 		EPwm2Regs.CMPA.half.CMPA
#define AGC_Min_ePWM 	EPwm2Regs.CMPB
#define PHASE_CMD_ePWM 	EPwm3Regs.CMPA.half.CMPA

//ePWM Defines for LED Ring (LED's are active low hence the (100-x) in the equation)
#define GRN_LED(x)		EPwm1Regs.CMPB = (float)(100-x)/100*3000									// x equals the percentage of brightness the LED is on 0 being off, and 100 full on
#define RED_LED(x)		EPwm3Regs.CMPB = (float)(100-x)/100*3000
#define BLU_LED(x)		EPwm5Regs.CMPB = (float)(100-x)/100*3000


#define DIG_FREQ_ePWM 	EPwm4Regs.CMPA.half.CMPA													// Sets VCO_IN level which corresponds to the upper and lower window range of 4046
#define AGC_CMD_ePWM 	EPwm5Regs.CMPA.half.CMPA
#define DIG_AGC_ePWM 	EPwm6Regs.CMPA.half.CMPA

//Constant Variable Defines
#define ANALOG 	0
#define DIGITAL 1
#define PLL_CTR_MAX			100000
#define AGC_MIN_VAL 		800
#define AGC_START_VAL 		1200
#define AGC_COMMAND_VAL 	50
#define START_PHASE_INIT 	1500
#define MIN_PHASE			1300
#define SYS_ANA_RUN_OK  	0
#define SYS_RESTART 		1
#define FREQ_ERROR			2
#define AGC_FAULT 			3
#define OVERLOAD_FAULT		4
#define SYS_OFF				10
#define SAFETY_CTR_MAX		5000
#define SYS_DOG_OUT 		0
#define SYS_DOG_OK	 		1
#define XMT_OK 				1
#define XMT_FAIL 			0
#define HIGH_VCO 			2790																		// Set Point for setting VCO to ~5V
#define LOW_VCO  			10																			// Set Point for setting VCO to ~0V
#define HIGH_PWM			2800
#define LOW_PWM				40
#define SAMPLE_WINDOW_VALS	10
#define LARGE_AGC			500
#define SMALL_AGC			350
#define R1_INIT_VAL			1500
#define R2_INIT_VAL 		1500
#define I_MOT_OK			1
#define I_MOT_NG			0
#define SMALL 				0
#define LARGE 				1
#define FSW 				2
#define HSW_DELAY			20000
#define START_POT_VAL		0x0401
#define HSW_DEBOUNCE_CTR	500
#define MAX_RESTARTS		4
#define HIGH_FREQ			21775
#define LOW_FREQ			19500
#define WINDOW_ADJ_PERIOD	500

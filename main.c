//###########################################################################
//
// FILE:	main.c
// 
// TITLE:	Software, Final Checkout Generator
// Copyright © Cybersonics Inc., 2016
//################################################################################################################################################################################################
//  Ver | mm dd yyyy | Who  | Description of changes
// =====|============|======|===============================================
//  01  | 07 31 2013 | S.R. | 1st Software release based on C1S01710 - Rev 05
//							  Added delay after PLL is turned on to allow more time for system to lock
//							  Added a counter to ensure that the over voltage / over current is a “real” threat
//							  Made it so that if a restart occurs it does not try to relock at full power
//	02	| 04 27 2017 | S.R. | Updated the frequency window to be 19500 - 21775, because some transducer were having higher Fr's
//################################################################################################################################################################################################
#include "DSP2833x_EPwm_defines.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "defines.h"
#include "DSP2833x_eCAP_defines.h"

// Code to insert application version into flash of DSP for bootloader purposes
#ifndef DEBUG
	#pragma DATA_SECTION(appVersion,"version");
	const Uint32 appVersion = 0x00000002;
#endif
//################################################################################################################################################################################################
//						Prototype statements for functions found within this file.
//################################################################################################################################################################################################

void Gpio_select(void);

#ifndef DEBUG
// Do not remove code below used to put program on flash of DSP
// (must include 28335.cmd and exclude 28335_RAM_lnk.cmd)
void InitFlash(void);
#endif

void ADCconfig(void);
void Setup_ePWM(void);
void Setup_eCAP(void);
unsigned int spi_xmit(Uint16 a);
void spi_init(void);
void spi_fifo_init(void);
void Error_lights(void);
void ultrasound_on_off(int a);
void DETECT_AGC_FAULT(void);
void AGC_SOFT_D_TO_A(void);
void ana_or_dig_freq(int a);
void ana_or_dig_AGC(int a);																				//Goes to R83 / D16 on schem and see above for op.
unsigned int Cal_I_mot(void);
void ANA_RUNNING_AND_LOCKED(void);
unsigned int Sys_Watchdog(void);
interrupt void cpu_timer0_isr(void);
void Unrecoverable_Error(void);
void Recoverable_Error(void);

extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitAdc(void);


//################################################################################################################################################################################################
//						Global Variables
//################################################################################################################################################################################################
Uint16 rdata;																							// SPI receive data
unsigned int volts_val;																					// ADC Volts Value (0-4095 - min - max)
unsigned int current_val;																				// ADC Current Value (0-4095 - min - max)
unsigned int phase_val;																					// ADC Phase Value (0-4095 - min - max, 90° is midpoint 2048)
unsigned int i_mot_val;																					// ADC I Motional Value (0-4095 - min - max)
unsigned int phase_error_val;																			// ADC Phase Error Value (0-2048-4095 - 0 is at mid point 2048)
unsigned int vco_in_val;																				// ADC VCO IN Value (0-4095 - min - max)
unsigned int agc_error_val;																				// ADC AGC Error Value (0-2048-4095 - 0 is at mid point 2048)
unsigned int agc_pwm_val;																				// ADC AGC PWM Value (0-4095 - min - max)
unsigned int sys_fail;		 																			// Used to force a pedal / switch release after a recoverable fault
unsigned int pll_on;																					// Tells system that PLL is engaged (analog control)
unsigned int lock;																						// Used to inform system that analog PLL control is locked
unsigned int ana_agc_on;																				// Tells system that analog AGC is engaged.
unsigned int sys_on;																					// From main loop to timer0 tells timer0 that user is commanding ultrasound
long unsigned int overload_ctr;																			// Determines that system is loaded beyond system power capability or feedback on AGC is faulty
unsigned int start_ctr;																					// Used for AGC error and overload not to start during start transient
unsigned int capture_ctr;																				// Used to detect initial capture - need N OK events
unsigned int restart_timer;																				// Controls program flow during re-start
unsigned int restart_ctr;																				// Number of restarts before foot off pedal required
unsigned int new_ad_vals;																				// A semaphore to coordinate with routines that use A/D vals to make sure each iteration uses new vals
unsigned int sys_watchword;																				// Bits are set by each critical routine that must execute under system state for watchdog to determine proper function
unsigned int sys_watchdog_ctr;																			// Counts before above mentioned bits not being set indicate system malfunction
unsigned int r1_val;																					// PLL variable R control param
unsigned int r2_val;																					// PLL variable R control param
long unsigned int sys_safety_ctr;																		// If system re-start takes too long, issues a fault
unsigned int sample_window[SAMPLE_WINDOW_VALS];															// Window used in ANA_RUNNING_AND_LOCKED function
unsigned int sample_window_2[SAMPLE_WINDOW_VALS];														// Window used in AGC_SOFT_D_TO_A function
unsigned int ana_run_status;																			// Variable used by ANA_RUNNING_AND_LOCKED function to tell Timer0 status of PLL control
unsigned int pot_val;																					// Co function variable for adjusting I motional current
unsigned int co_on, co_done;																			// Semaphore used to indicate I motional adjust active so other routines do not interfere
unsigned int hsw_enabled;																				// Semaphore used to indicated is hand switches are enabled
unsigned int button_timer, hsw_sys_on, mode, indicator_light, button_delay, hsw_large_or_small;			// Used for controlling different button modes
unsigned int hsw_debounce, xducer_detect_debounce, debounce_ctr;										// Used for debounce
unsigned int xmt_test_val;
unsigned int restarted, no_restart_ctr, lost_lock;
unsigned int vco_in_last, vco_adj_done, vco_ctr, vco_freq_ctr, array_location_oldest;					// Used for bringing system into digital mode
unsigned int vco_in_last_array[1000], vco_in_last_ctr, vco_in_last_position, array_location_newest;
unsigned int agc_delay_ctr, agc_fault_ctr;																// Used for when turning on AGC before locking
unsigned int agc_ramp_down_enabled, agc_ramp_down_delay;												// Used for AGC ramp down
unsigned int window_val;
long unsigned int high_limit, low_limit;
unsigned int high_window_adj, low_window_adj, low_window_init, high_window_init, window_start_ctr;		// Used for setting the PLL window
unsigned int window_delay;
unsigned int hsw_pressed, hsw_pressed_detect;
unsigned int START_PHASE;
unsigned int sys_stable, Stable_Ctr;
long unsigned int pll_ctr;
long unsigned int Sys_Monitor_Ctr;
unsigned int overCtr=0, max=0;

#ifndef DEBUG
// Do not remove code below used to put program on flash of DSP
// (must include 28335.cmd and exclude 28335_RAM_lnk.cmd)

extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;
#endif

//################################################################################################################################################################################################
//						main code									
//################################################################################################################################################################################################
void main(void)
{
	unsigned int i;
	InitSysCtrl();	 																					// Basic Core Initialization
	EALLOW;
	SysCtrlRegs.WDCR = 0x0068;				 															// 0x00AF = Enable Watchdog 0x0068 = Disable Watchdog
	EDIS;

	DINT;																								// Disable all interrupts

#ifndef DEBUG
	// Do not remove code below used to put program on flash of DSP
	// (must include 28335.cmd and exclude 28335_RAM_lnk.cmd)
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, &RamfuncsLoadEnd - &RamfuncsLoadStart);				// Setup program to run in flash
	InitFlash();
#endif

	ana_run_status=SYS_OFF;																				// Set initial values
	sys_fail=0;
	sys_watchdog_ctr=0;
	sys_watchword=0;
	volts_val=0;
	hsw_enabled = 0;
	button_timer = 0;
	hsw_sys_on = 0;
	mode = 0;
	indicator_light = 0;
	button_delay = 0;
	hsw_large_or_small = SMALL;
	hsw_debounce = 0;
	xducer_detect_debounce = 0;
	agc_ramp_down_delay = 0;
	hsw_pressed = 0, hsw_pressed_detect = 0;
	sys_on=0;
	sys_safety_ctr=0;
	co_done=0;
	co_on=0;
	pll_on=0;
	vco_adj_done = 0, vco_freq_ctr = 0,	vco_ctr = 0, vco_in_last=0, vco_in_last_ctr=0, vco_in_last_position=0;
	array_location_newest=0, array_location_oldest=0;
	for(i=0;i<1000;i++)
		vco_in_last_array[i]=0;
	agc_fault_ctr = 0, window_delay = 0, no_restart_ctr = 0;
	lost_lock = 0;
	r1_val = R1_INIT_VAL;																				// Setup the frequency window of the PLL (4046) chip
	r2_val = R2_INIT_VAL;
	START_PHASE=START_PHASE_INIT;
	low_window_init = 0;
	high_window_init = 0;
	ultrasound_on_off(ULTRA_OFF);																		// Make sure ultra sound is off
	low_window_adj = 1;
	high_window_adj = 0;
	pll_ctr = 0;

	Gpio_select();																						// Setup GPIO's
	Setup_ePWM();																						// Setup ePWM signals
	InitPieCtrl();																						// Basic setup of PIE table; from DSP2833x_PieCtrl.c
	InitPieVectTable();																					// Default ISR's in PIE
	InitAdc();																							// Initialize Analog to Digital converter
	ADCconfig();																						// Configure the ADC
	spi_init();																							// Initialize SPI interface
	spi_fifo_init();																					// Initialize SPI FIFO
	Setup_eCAP();																						// Setup the eCAP pins

	EALLOW;																								// Enable access to protected register
	PieVectTable.TINT0 = &cpu_timer0_isr;																// Re-map entry for Timer0 from ESTOP0 to real interrupt service
	EDIS;																								// Disable access to protected register
	InitCpuTimers();																					// Basic setup CPU Timer0, 1 and 2
	ConfigCpuTimer(&CpuTimer0, 150, 100);																// Initialize Timer0 to 100us (150 = speed of DSP, 100000 = 100ms, 100 = 100us)
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;																	// Enable interrupt masks, Timer0 connected to group INT1, Bit7

	if(sys_watchword!=0x03FF)																			// All setups done?
	{
		ultrasound_on_off(ULTRA_OFF);
		Unrecoverable_Error();
	}

	IER |= 1;																							// Enable interrupt core line 1 (INT 1), modify register IER acc
	EINT;																								// Enable control interrupts
	ERTM;																								// Enable debug interrupts

	CpuTimer0Regs.TCR.bit.TSS = 0;																		// Start Timer0 Interrupt

	while(1)																							// Main program (keep looping)
	{    
		if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)														// Detects missing clock and issues failure if sys goes into limp mode.
		{
			ultrasound_on_off(ULTRA_OFF);
			Unrecoverable_Error();
		}
		sys_watchword|=0x00F0;																			// Update sys_watchword to verify loop is running
		//*********************************************************Transducer Connected**********************************************************************************
		if (!XDUCER_DETECT)																				// Handswitch detect debounce
			xducer_detect_debounce = 0;
		if(xducer_detect_debounce<500&&(low_window_init&&high_window_init))								// Only allow system to run it transducer is connected and debounce count is less than 1000 and setup of the window is close.
		{
			GRN_LED(40);																				// Set transducer light ring to match color of power ring
			//***********************************************************Button Pressed***********************************************************************************
			if((hsw_sys_on&&hsw_enabled)||((!FSW_LARGE||!FSW_SMALL)&&hsw_large_or_small == FSW))		// Test if user is commanding US
			{
				if (((hsw_large_or_small == SMALL)||!FSW_SMALL)&&indicator_light&&!sys_fail)			// Small Stone Mode (Standard Power)
				{
					AGC_CMD_ePWM = SMALL_AGC;
					SMSTN_WHT_OFF;
					SMSTN_GRN_ON;
					LGSTN_GRN_OFF;
					LGSTN_WHT_ON;
					ERROR_RED_OFF;
					ERROR_WHT_ON;
				}
				else if (((hsw_large_or_small == LARGE)||!FSW_LARGE)&&indicator_light&&!sys_fail)		// Large Stone Mode (High Power)
				{
					AGC_CMD_ePWM = LARGE_AGC;
					LGSTN_WHT_OFF;
					LGSTN_GRN_ON;
					SMSTN_GRN_OFF;
					SMSTN_WHT_ON;
					ERROR_RED_OFF;
					ERROR_WHT_ON;
				}
				if(debounce_ctr>1000)																	// Co Adjust Debounce to only allow Co Adj once
				{
					if((!co_done)&&(ana_run_status!=SYS_RESTART)&&!sys_fail)							// Run Co Adjust
					{
						co_on=1;																		// Do not do anything other than Co
						if(Cal_I_mot())																	// Co turns on US and nobody else should touch it
						{
							sys_on=1;																	// Tell Timer0 to run US normally
							co_on=0;
						}
						else
						{
							ultrasound_on_off(ULTRA_OFF);
							Recoverable_Error();														// If Co did not succeed - do not run.
						}
					}
				}
			}
			//***********************************************************End Button Pressed******************************************************************************

			//************************************************Hand switch pressed while footswitch plugged in************************************************************
			else if((hsw_large_or_small == FSW)&&(!HSW_LARGE||!HSW_SMALL))								// If user presses hand switch while footswitch is plugged in show error
			{
				hsw_pressed_detect = 1;
				if (hsw_pressed>200)																	// Handswitch detect debounce
				{
					ana_run_status=SYS_OFF;
					ultrasound_on_off(ULTRA_OFF);
					SMSTN_GRN_OFF;
					SMSTN_WHT_ON;
					LGSTN_GRN_OFF;
					LGSTN_WHT_ON;
					ERROR_RED_ON;
					ERROR_WHT_OFF;
					if (CpuTimer0.InterruptCount < 5000)
					{
						FSW_WHT_ON;
						FSW_GRN_OFF;
					}
					else if (CpuTimer0.InterruptCount > 4999 && CpuTimer0.InterruptCount < 10000)
					{
						FSW_WHT_OFF;
						FSW_GRN_ON;
					}
					else
						CpuTimer0.InterruptCount = 0;
				}
			}
			//************************************************End Hand switch pressed while footswitch plugged in********************************************************

			//******************************************************************* No Button Pressed**********************************************************************
			else
			{
				hsw_pressed_detect = 0;
				debounce_ctr=0;
				ultrasound_on_off(ULTRA_OFF);
				sys_on=0;
				co_done=0;
				co_on=0;
				ana_run_status=SYS_OFF;
				restart_ctr=0;
				restart_timer=0;
				if(sys_fail==1)
					sys_fail=0;
				restarted=0;
				lost_lock=0;
				Sys_Monitor_Ctr=0;
				SMSTN_WHT_ON;
				SMSTN_GRN_OFF;
				LGSTN_WHT_ON;
				LGSTN_GRN_OFF;
				no_restart_ctr=0;
				if (pot_val != START_POT_VAL)
				{
					pot_val = START_POT_VAL;
					xmt_test_val=spi_xmit(0x1802);
					if(xmt_test_val!=XMT_OK)
						Recoverable_Error();
					xmt_test_val=spi_xmit(pot_val);
					if(xmt_test_val!=XMT_OK)
						Recoverable_Error();
				}
				if(mode==0||FSW_LARGE||FSW_SMALL)
				{
					if(sys_fail==1)
						sys_fail=0;
					ERROR_RED_OFF;
					ERROR_WHT_ON;
					CHKPRB_WHT_ON;
					CHKPRB_RED_OFF;
				}
			}
		}
		//******************************************************************* End No Button Pressed**********************************************************************
		//******************************************************************* End Transducer Connected*******************************************************************

		//******************************************************No Transducer Connected**********************************************************************************
		else																							// If hand piece is not connected
		{
			ultrasound_on_off(ULTRA_OFF);																// Make sure US is off
			lost_lock=0;
			sys_on = 0;																					// Make sure US flag is reset
			SMSTN_WHT_ON;
			SMSTN_GRN_OFF;
			LGSTN_WHT_ON;
			LGSTN_GRN_OFF;
			if (low_window_init&&high_window_init)
			{
				if (CpuTimer0.InterruptCount < 5000)
				{
					GRN_LED(0);																			// Blink transducer LED ring
				}
				else if (CpuTimer0.InterruptCount > 4999 && CpuTimer0.InterruptCount < 10000)
				{
					GRN_LED(40);																		// Blink transducer LED ring
				}
				else
					CpuTimer0.InterruptCount = 0;
			}
		}
		//******************************************************End No Transducer Connected**********************************************************************************
	}

} 

//################################################################################################################################################################################################
//						End of main code
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Timer 0 (100us)
//################################################################################################################################################################################################

void cpu_timer0_isr()
{
	#define VOLTS 		AdcMirror.ADCRESULT0
	#define CURRENT 	AdcMirror.ADCRESULT1
	#define I_MOT		AdcMirror.ADCRESULT2
	#define PHASE		AdcMirror.ADCRESULT3
	#define PHASE_ERROR	AdcMirror.ADCRESULT4
	#define VCO_IN		AdcMirror.ADCRESULT5
	#define AGC_ERROR	AdcMirror.ADCRESULT6
	#define AGC_PWM		AdcMirror.ADCRESULT7

	static unsigned long int master_restart_ctr;
	static unsigned int volts_while_off_ctr;
	unsigned int sys_integrity;

	CpuTimer0.InterruptCount++;																			// Increment global interrupt counter

	//******************************************************Debounce for functions in main while(1) loop*****************************************************************
	if (xducer_detect_debounce<501)
		xducer_detect_debounce++;																		// Xducer detect debounce
	if (hsw_pressed_detect&&hsw_pressed<205)
		hsw_pressed++;
	else if (!hsw_pressed_detect)
		hsw_pressed = 0;
	if((!FSW_LARGE||!FSW_SMALL||hsw_sys_on)&&debounce_ctr<=1000&&!co_done)
		debounce_ctr++;
	else
		debounce_ctr=0;
	//******************************************************End Debounce for functions in main while(1) loop*************************************************************


	//******************************************************Detect if footswitch is plugged in***************************************************************************
	if (!FSW_DETECT)																					// Detect footswitch connection, Footswitch detected
	{
		FSW_WHT_OFF;																					// Turn on footswitch light
		FSW_GRN_ON;
		hsw_enabled = 0;																				// Disable hand switches
		indicator_light = 1;																			// Set indicator light flag
		hsw_large_or_small = FSW;																		// Set hand switch large or small indicator flag to footswitch operation
	}
	else																								// No footswitch detected
	{
		FSW_WHT_ON;																						// Turn footswitch light off
		FSW_GRN_OFF;
		hsw_enabled = 1;																				// Enable hand switches
		if (hsw_large_or_small == FSW)
			hsw_large_or_small = SMALL;																	// Set to default state
	}
	//******************************************************End Detect if footswitch is plugged in***********************************************************************

	//*********************************************************Hardware Safety Circuit Fault Detection*******************************************************************
	if ((TWENTY_AMP_OFF || FIVE_AMP_OFF ||OVERVOLTS_OFF || AGC_AMP_OFF) && low_window_init && high_window_init)
	{
		ultrasound_on_off(ULTRA_OFF);
		Unrecoverable_Error();
	}
	//*********************************************************End Hardware Safety Circuit Fault Detection*******************************************************************

	//*************************************************************Hand Switch Detection*********************************************************************************
	// The code below allows the user to double click either the HSW_SMALL or HSW_LARGE button and latch the system on
	// To turn system off press button a 3rd time.
	if (hsw_enabled)																					// Check if hand switches are enabled
	{
		if (button_timer<10002)																			// Only increment button_timer to 10000 don't let it loop around back to 0
		{
			if (mode >= 1 && mode <=3)
				button_timer++;
		}
		switch( mode )
		{
			case 0:																						// Detect 1st button down
				if (!HSW_LARGE||!HSW_SMALL)
				{
					if (hsw_debounce<hsw_debounce+2)													// Debounce handswitch
						hsw_debounce++;
					if (hsw_debounce>HSW_DEBOUNCE_CTR)
					{
						button_timer=0;																	// Initialize button timer to 0
						mode = 1;																		// First button press detected move to the next detection state (1st release)
						if (!HSW_LARGE)
							hsw_large_or_small = LARGE;													// Set to large stone mode
						else if (!HSW_SMALL)
							hsw_large_or_small = SMALL;													// Set to small stone mode
						hsw_debounce = 0;																// Reset debounce
					}

				}
				else 																					// If neither switches are pressed reset debounce
					hsw_debounce = 0;
				break;
			case 1:																						// Detect 1st button release
				if ((!HSW_LARGE||!HSW_SMALL)&&(button_timer>3000))										// Button still depressed and button_timer has waited long enough (momentary action, turn on)
				{
					hsw_sys_on = 1;																		// Allow system to run with hand switches
					indicator_light = 1;
				}
				if ((HSW_LARGE&&HSW_SMALL)&&button_timer>3000)											// Button Release and button_timer has exceeded 3000, turn off system (momentary action, turn off)
				{
					if (hsw_debounce<HSW_DEBOUNCE_CTR+2)
						hsw_debounce++;
					if (hsw_debounce>HSW_DEBOUNCE_CTR)
					{
						hsw_sys_on = 0;																	// Make sure system is off
						indicator_light = 0;
						mode = 0;																		// Reset hand switch states
						hsw_debounce = 0;
					}
				}
				if ((HSW_LARGE&&HSW_SMALL)&&button_timer<=3000)											// Button Release and button_timer has not exceeded 3000 (user is engaging double click latch action)
					mode = 2;																			// Button release detected, move on to next detection
				break;
			case 2:																						// Detect 2nd button down
				if (button_timer>10000)																	// If 2nd button button down is not received soon enough reset double click
				{
					mode = 0;
					hsw_sys_on = 0;																		// Make sure system is off
					indicator_light = 0;
				}
				if (!HSW_LARGE||!HSW_SMALL)																// 2nd button down detected
				{
					mode = 3;																			// 2nd button down detected, move on to the next detection
					button_timer = 0;
				}
				break;
			case 3:																						// Detect 2nd button release
				if ((!HSW_LARGE||!HSW_SMALL)&&(button_timer>5000))										// Button still depressed and button_timer has waited long enough (momentary action, turn on, after the double click)
				{																						// User held button down too long and button will now be used as a momentary button
					hsw_sys_on = 1;																		// Allow system to run
					indicator_light = 1;
				}
				if ((HSW_LARGE&&HSW_SMALL)&&button_timer>5000)											// Button Release and button_timer has exceeded 10000, turn off system (momentary action, turn off, after the double click)
				{
					if (hsw_debounce<HSW_DEBOUNCE_CTR+2)
						hsw_debounce++;
					if (hsw_debounce>HSW_DEBOUNCE_CTR)
					{
						hsw_sys_on = 0;																	// Make sure system is off
						indicator_light = 0;
						mode = 0;
						hsw_debounce = 0;
					}
				}
				if (HSW_LARGE&&HSW_SMALL&&button_timer<=5000)											// 2nd button release detected and time is less than 10000 so user intended to double click latch the system
				{
					mode = 4;																			// Move on to the next detection
					hsw_sys_on = 1;																		// Allow system to run
					indicator_light = 1;
				}
				break;
			case 4:																						// Detect 3rd button down, to shut off system
				if (!HSW_LARGE||!HSW_SMALL)
				{
					if (hsw_debounce<HSW_DEBOUNCE_CTR+2)
						hsw_debounce++;
					if (hsw_debounce>HSW_DEBOUNCE_CTR)
					{
						hsw_sys_on = 0;
						indicator_light = 0;
						mode = 5;
						hsw_debounce = 0;
					}
				}
				else if (HSW_LARGE&&HSW_SMALL)
					hsw_debounce = 0;																	// Reset debounce if no switch is pressed
				break;
			case 5:																						// Detect 3rd button release, reset double click
				if (HSW_LARGE&&HSW_SMALL)
					mode = 0;
				break;
		}
	}
	//*************************************************************End Hand Switch Detection*****************************************************************************

	//*************************************************************Recoverable Error Check*******************************************************************************
	// If system is in recoverable error continue lighting the correct lights
	if(sys_fail==1)
		Error_lights();
	//*************************************************************End Recoverable Error Check***************************************************************************

	//***********************************************************************ADC Update**********************************************************************************
	if(AdcRegs.ADCST.bit.INT_SEQ1)																		// If an ADC conversion is taking place do the following
	{
		sys_watchword|=0x000F;																			// A/D vals must be updated
		if((VOLTS>3500) || (CURRENT>3500))																// Check to make sure voltage and current are within a safe range
		{
			overCtr++;
			if(overCtr>max)
				max=overCtr;
			if(overCtr>1000)
			{
				ultrasound_on_off(ULTRA_OFF);
				Unrecoverable_Error();
			}
		}
		else
			overCtr=0;
		if(AGC_PWM>3200)																				// Make sure AGC did not go to high
		{
			ultrasound_on_off(ULTRA_OFF);
			Recoverable_Error();
		}

		phase_val=PHASE;																				// Store current ADC values in global variables
		i_mot_val=I_MOT;
		phase_error_val=PHASE_ERROR;
		vco_in_val=VCO_IN;
		agc_error_val=AGC_ERROR;
		agc_pwm_val=AGC_PWM;
		volts_val=VOLTS;
		current_val=CURRENT;
		new_ad_vals=1;																					// Tell other routines of new A/D vals
	}
	//*********************************************************************End ADC Update********************************************************************************

	//***********************************************************************START US MODULE*****************************************************************************
	if(sys_on && co_done)																				// Sys on and doing Co should not go in here until Co done
	{
		if(start_ctr<200)																				// Let sys settle before using error checks
			start_ctr++;
		if((!pll_on)&&(!sys_fail))																		// Start up US
		{
			no_restart_ctr++;
			ultrasound_on_off(ULTRA_ON);																// Turn on PA fist with fixed freq at low end of window (set up when US was turned off)
			if((vco_in_val<400)&&(restart_ctr<MAX_RESTARTS))											// Let analog PLL capture and control
			{
				ana_or_dig_freq(ANALOG);
				pll_on=1;
				DELAY_US(400000);																	// Delay to allow lock
			}
			else if((restart_ctr>=MAX_RESTARTS)&&vco_in_last&&(!(no_restart_ctr>10000)))				// If MAX_RESTARTS occurred and there is a value stored for vco_in_last, run system in digital freq mode
			{
				if(new_ad_vals)																			// Make sure A/D values were updated before running again
				{
					ana_or_dig_freq(DIGITAL);
					//ana_or_dig_AGC(ANALOG);
					//ana_agc_on=1;																		// Switch to digital frequency
					pll_on=0;
					if (vco_freq_ctr < 1000)															// Delay to allow frequency to settle after adjustment
						vco_freq_ctr++;
					else if(!vco_adj_done&&vco_ctr<1000)												// Algorithm to adjust DIG_FREQ_ePWM until vco_in_val = vco_in_last = last known resonance
					{
						if ((((int)vco_in_val-(int)vco_in_last)<-5)&&(DIG_FREQ_ePWM<HIGH_PWM))
							DIG_FREQ_ePWM++;
						else if ((((int)vco_in_val-(int)vco_in_last)>5)&&(DIG_FREQ_ePWM>LOW_PWM))
							DIG_FREQ_ePWM--;
						else
							vco_ctr++;																	// Counter to make sure vco_in_val has settled to correct spot
					}
					else
						vco_adj_done = 1;																// Indication that VCO adjustment is done
					if(!ana_agc_on&&vco_adj_done)														// Once VCO adjustment is done turn on analog AGC with AGC_SOFT_D_TO_A();
						AGC_SOFT_D_TO_A();
					else if(ana_agc_on)
					{
						no_restart_ctr=0;
					}

				}
			}
			else if(((restart_ctr>=MAX_RESTARTS)&&(vco_in_last==0))||(no_restart_ctr>10000))			// System restarted without first capturing a resonance
			{
				ultrasound_on_off(ULTRA_OFF);
				Recoverable_Error();
			}
			sys_watchword|=0x0F00;
		}
		//***********************************************************************END START US MODULE*********************************************************************

		//***********************************************************************MONITOR US MODULE***********************************************************************
		if(restart_ctr<MAX_RESTARTS)																	// Continue to run this as long as system has not restarted too many times
		{
			ANA_RUNNING_AND_LOCKED();																	// Call capture / lock detect routine
			if(ana_run_status==SYS_ANA_RUN_OK)															// If sys locked and OK
			{
				if((!ana_agc_on)&&new_ad_vals&&!restarted&&lock)
					AGC_SOFT_D_TO_A();																	// Do AGC soft start and turn sys over to closed loop AGC
				else if(ana_agc_on&&(PHASE_CMD_ePWM<(START_PHASE+150)))									// After system has locked increase phase command
					PHASE_CMD_ePWM++;
				else if((Stable_Ctr<10000)&&ana_agc_on&&(PHASE_CMD_ePWM>=(START_PHASE+150)))
					Stable_Ctr++, vco_in_last_ctr=0;
				else if(Stable_Ctr>=10000)
				{
					sys_stable=1;
					Sys_Monitor_Ctr=0;
				}
				else
					Sys_Monitor_Ctr++;
				if(Sys_Monitor_Ctr>10000)																// If phase command is too low, the system will sit at the low end of the window
				{
					lost_lock=1;																		// Force system to restart
					if (sys_fail!=1&&vco_in_val<200)													// Don't allow increment when system is sitting in a Recoverable Error and only allow when system is at low end of window
						START_PHASE +=80;																// Increase phase by 40, 80 in code because later in code it reduces it by 40
					Sys_Monitor_Ctr=0;																	// Reset counter
					if(START_PHASE>2800)																// If phase command gets too high issue a recoverable error
					{
						ultrasound_on_off(ULTRA_OFF);
						Recoverable_Error();
						START_PHASE = START_PHASE_INIT;													// If max limit reached reset
					}
				}
			}
			else if(ana_run_status==SYS_RESTART)														// If sys in re-start, and monitor time out
			{
				master_restart_ctr++;
				sys_watchword|=0x0F00;
				if(master_restart_ctr>200000)
				{
					ultrasound_on_off(ULTRA_OFF);
					Recoverable_Error();
				}
			}
			else if(ana_run_status==FREQ_ERROR)															// If sys could not capture after N re-starts, user must remove US command and try again.
			{
				ultrasound_on_off(ULTRA_OFF);
				Recoverable_Error();
			}
			if(ana_run_status!=SYS_RESTART)
			{
				DETECT_AGC_FAULT();
				if(ana_run_status==OVERLOAD_FAULT)														// If sys is overloaded where the feedbacks are maxed out and commands are not met
				{																						// User needs to back off on pressure
					ultrasound_on_off(ULTRA_OFF);
					Recoverable_Error();
				}
				else if(ana_run_status==AGC_FAULT)														// If sys errors are high and there is small feedback, there may be a fault in feedback.
				{
					ultrasound_on_off(ULTRA_OFF);
					Recoverable_Error();
				}
			}
			sys_watchword|=0x0800;
		}
		//***********************************************************************END MONITOR US MODULE*******************************************************************
	}
		//***********************************************************************MONITOR UNEXPECTED RUNNING MODULE*******************************************************
	else if((!co_on)&&(ana_run_status!=SYS_RESTART))													// Should happen only with sys off.
	{
		ana_run_status=SYS_OFF;
		ultrasound_on_off(ULTRA_OFF);
		co_done=0;
		sys_safety_ctr=0;
		master_restart_ctr=0;
		if(volts_val>200)																				// Sys should have no volts if not on
		{
			if(volts_while_off_ctr<10000)
				volts_while_off_ctr++;
			ultrasound_on_off(ULTRA_OFF);
			if(volts_while_off_ctr>=10000)
			{
				ultrasound_on_off(ULTRA_OFF);
				Unrecoverable_Error();																	// Turn off and let watchdog reset if sys running when not intended.
			}
		}
		else
			volts_while_off_ctr=0;
		//***********************************************************************END MONITOR UNEXPECTED RUNNING MODULE***************************************************

		//***********************************************************************FREQUENCY WINDOW ADJ MODULE*************************************************************
		if(ECap5Regs.ECFLG.bit.CEVT4)																	// Setup window function, Perform the following if the 4th capture event occurred
		{
			if(((low_window_adj<WINDOW_ADJ_PERIOD)||!low_window_init)&&!window_delay)					// Continue adjusting low end of window until WINDOW_ADJ_PERIOD or window_delay has timed out
			{
				low_limit  = 150000000/LOW_FREQ;														// 150e6 comes from processor clock speed (150MHz)
				DIG_FREQ_ePWM=LOW_VCO;																	// Set frequency low to adjust low end of PLL window
				window_start_ctr++;
				window_val=ECap5Regs.CAP4-ECap5Regs.CAP1;												// Subtract the 1st rising edge with the 4th rising edge (allows for some averaging)
				window_val/=3;																			// Divide by 3 because 3 periods were captured, Calculate period of wave = 1/f
				if(!low_window_init)
				{
					if((window_val < low_limit-200)&&(r2_val>LOW_PWM))									// Course adjust to make loop faster.If window_val is small, freq is too high (window_val is a period)
						r2_val -= 10;																	// Decreasing pulse width lowers FET gate volts, causes higher R and lower freq / greater period
					else if((window_val > low_limit+200)&&(r2_val<HIGH_PWM))
						r2_val += 10;
					R2_ePWM = r2_val;																	// Store new r2_val into the ePWM unit
				}
				if(window_start_ctr>(WINDOW_ADJ_PERIOD/4))												// After set period of time start fine adjustment
				{
					if((window_val<(low_limit-5))&&(r2_val>LOW_PWM))
						r2_val--;																		// Decreasing pulse width lowers FET gate volts, causes higher R and lower freq / greater period
					else if((window_val>(low_limit+5))&&(r2_val<HIGH_PWM))                              // Increase R2_ePWM if window_val is greater than low_limit and R2_ePWM is less than HIGH_VCO
						r2_val++;
					else
						low_window_init=1;																// LOW WINDOW adjustment is done enable HIGH WINDOW adjustment
					R2_ePWM=r2_val;
					low_window_adj++;
				}
				if(low_window_adj>=WINDOW_ADJ_PERIOD)													// Reset counters once low_window_adj reaches WINDOW_ADJ_PERIOD
				{
					high_window_adj=0;
					window_start_ctr=0;
				}
			}
			else if(low_window_init&&!window_delay)
			{
				high_limit = 150000000/HIGH_FREQ;														// Calculate high limit window value
				DIG_FREQ_ePWM=HIGH_VCO;
				window_start_ctr++;
				window_val=ECap5Regs.CAP4-ECap5Regs.CAP1;
				window_val/=3;
				if(!high_window_init)
				{
					if ((window_val < high_limit-200)&&(r1_val>LOW_PWM))								// Course adjustment to speed up loop
						r1_val -= 10;
					else if( (window_val > high_limit+200)&&(r1_val<HIGH_PWM))
						r1_val += 10;
					R1_ePWM = r1_val;
				}
				if(window_start_ctr>(WINDOW_ADJ_PERIOD/4))
				{
					if((window_val<(high_limit-5))&&(r1_val>LOW_PWM))
						r1_val--;																		// Decreasing pulse width lowers FET gate volts, causes higher R and lower freq / greater period
					else if((window_val>(high_limit+5))&&(r1_val<HIGH_PWM))
						r1_val++;																		// Decreasing pulse width lowers FET gate volts, causes higher R and lower freq / greater period
					else
						high_window_init=1;
					R1_ePWM=r1_val;
					high_window_adj++;
				}
				if(high_window_adj>=WINDOW_ADJ_PERIOD)													// Reset counters once high_window_adj reaches WINDOW_ADJ_PERIOD
				{
					low_window_adj=0;
					window_start_ctr=0;
				}
			}
			if (window_delay<20)																		// Add delay for frequencies to settle
				window_delay++;
			else
			{
				ECap5Regs.ECCLR.bit.CEVT4=1;															// Clear the 4th capture event flag
				ECap5Regs.ECCTL2.bit.REARM=1;															// Re-arm the eCAP5
				window_delay = 0;																		// Reset window_delay
			}

		}
		sys_watchword|=0x0F00;																			// All done check
	}
	//***********************************************************************END FREQUENCY WINDOW ADJ MODULE*************************************************************

	else																								// Should happen during re-start and during initial start up / Cal_I_mot().
	{
		if(!co_on)
			ANA_RUNNING_AND_LOCKED();																	// Keep on calling routine that is controlling the re-start.
		sys_safety_ctr++;
		sys_watchword|=0x0F00;																			// Update sys_watchword
		if (co_on)
			sys_watchword|=0x00F0;
		if(sys_safety_ctr>1e6)																			// If restart takes too long issue fault
		{
			ultrasound_on_off(ULTRA_OFF);
			Recoverable_Error();
		}
	}

	//*****************************************************************************WATCHDOG STATUS***********************************************************************
	sys_integrity=Sys_Watchdog();																		// Check status of watchdog
	if(sys_integrity==SYS_DOG_OUT)
	{
		ultrasound_on_off(ULTRA_OFF);
		Unrecoverable_Error();
	}
	//***************************************************************************END WATCHDOG STATUS*********************************************************************
	sys_watchword|=0xF000;																				// Update sys_watchword
	if(!co_on)
		new_ad_vals=0;
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;																	// Reset ADC Seq 1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;																	// Clears the SEQ1 interrupt flag bit, INT_SEQ1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;																	// Start an ADC conversion
}
//################################################################################################################################################################################################
//						End Timer 0 (100us)
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						GPIO Setup
//################################################################################################################################################################################################

void Gpio_select(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all = 0;																		// GPIO0  ... GPIO15 = General Purpose I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;																// Set GPIO 0  as EPWM1A (PWM for R1 of 4046 Chip)
	GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 1;																// Set GPIO 2  as EPWM2A (PWM for R2 of 4046 Chip)
	GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 1;																// Set GPIO 3  as EPWM2B (PWM for AGC_MIN_LEVEL_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 1;																// Set GPIO 4  as EPWM3A (PWM for PHASE_COMND_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 1;																// Set GPIO 6  as EPWM4A (PWM for DIG_FREQ_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 1;																// Set GPIO 8  as EPWM5A (PWM for AGC_COMMAND_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;																// Set GPIO 10 as EPWM6A (PWM for DIG_AGC_PWM)

	GpioCtrlRegs.GPAMUX2.all = 0;																		// GPIO16 ... GPIO31 = General Purpose I/O
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;																// GPIO24 as eCAP1 for detecting voltage freq/phase
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;																// GPIO25 as eCAP2 for detecting current freq/phase
	GpioCtrlRegs.GPBMUX1.all = 0;																		// GPIO32 ... GPIO47 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0;																		// GPIO48 ... GPIO63 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 1;																// GPIO48 as eCAP5 for detecting VCO frequency
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;																// Set GPIO54 as SPISIMOA
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1;																// Set GPIO55 as SPISOMIA
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1;																// Set GPIO56 as SPICLKA
	GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1;																// Set GPIO57 as SPISTEA
	GpioCtrlRegs.GPCMUX1.all = 0;																		// GPIO64 ... GPIO79 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0;																		// GPIO80 ... GPIO87 = General Purpose I/O

	GpioCtrlRegs.GPADIR.all = 0;																		// GPIO0  ... GPIO31 as inputs

	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;																	// Set GPIO 12 as output (WD_BONE)
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;																	// Set GPIO 13 as output (ANA_OR_DIG_AGC)

	//Hand Piece High and Low Command Input Qualification
	GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 0;																// Sample Period = System Clock (change if need more time to sample)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 2;																// Qual to 6 samples
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 2;																// Qual to 6 samples


	GpioCtrlRegs.GPBDIR.all = 0;																		// GPIO32 ... GPIO63 as inputs

	//Footswitch High and Low Command Input Qualification
	GpioCtrlRegs.GPBCTRL.bit.QUALPRD0 = 0;																// Sample Period = System Clock (change if need more time to sample)
	GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 2;																// Qual to 6 samples
	GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 2;																// Qual to 6 samples

	//Footswitch In Detect Input Qualification
	GpioCtrlRegs.GPBCTRL.bit.QUALPRD1 = 0;																// Sample Period = System Clock (change if need more time to sample)
	GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 2;																// Qual to 6 samples

	//LED Indicator Lights
	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;																	// Set GPIO 42 as output (LED3) (CHKPRB WHT) Check Probe White Light
	GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;																	// Set GPIO 43 as output (LED4) (CHKPRB RED) Check Probe Error (Red) Light
	GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;																	// Set GPIO 44 as output (LED5) (FTSW WHT) Footswitch Not Connected White Light
	GpioCtrlRegs.GPBDIR.bit.GPIO45 = 1;																	// Set GPIO 45 as output (LED6) (FTSW GRN) Footswitch Connected Green Light
	GpioCtrlRegs.GPBDIR.bit.GPIO46 = 1;																	// Set GPIO 46 as output (LED7) (LGSTN WHT) Large Stone Not Activated White Light
	GpioCtrlRegs.GPBDIR.bit.GPIO47 = 1;																	// Set GPIO 47 as output (LED8) (LGSTN GRN) Large Stone Activated Green Light

	GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;																	// Set GPIO 35 as output (ANA_OR_DIG_AGC2)
	GpioCtrlRegs.GPBDIR.bit.GPIO36 = 1;																	// Set GPIO 35 as output (ANA_OR_DIG_FREQ)
	GpioCtrlRegs.GPBDIR.bit.GPIO37 = 1;																	// Set GPIO 35 as output (ANA_OR_DIG_FREQ2)

	GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;																	// Set GPIO 52 as output (DSP_RESET_PA)
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;																	// Set GPIO 58 as output (DSP_40KHz_ON)(not used on power board C1E01608)
	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;																	// Set GPIO 59 as output (SYS_ON_ON_2)(used for proper shutdown of system)

	GpioCtrlRegs.GPCDIR.all = 0;																		// GPIO64 ... GPIO87 as inputs
	GpioCtrlRegs.GPCDIR.bit.GPIO64 = 1;																	// Set GPIO 64 as output (LED 10) (SMSTN GRN) Small Stone Activated Green Light
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;																	// Set GPIO 65 as output (LED 11) (ERROR WHT) Error Not Active White Light
	GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;																	// Set GPIO 66 as output (LED 12) (ERROR RED) Error Active Red Light
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;																	// Set GPIO 67 as output (LED 13) (LBLS WHT) White Light for Labels (Always On)
	GpioCtrlRegs.GPCDIR.bit.GPIO80 = 1;																	// Set GPIO 80 as output (LED9) (SMSTN WHT) Small Stone Not Activated White Light

	GpioCtrlRegs.GPCDIR.bit.GPIO68 = 1;																	// Set GPIO 68 as output (sys_on_3.3)
	ULTRASOUND_OFF;																						// PREVENT SYS FROM TURNING ON @ INITIALIZATION
	SYS_ON_OFF_2;

	sys_watchword|=0x0001;																				// Update sys_watchword
	EDIS;

	//Set initial states of LEDs
	CHKPRB_WHT_ON;
	CHKPRB_RED_OFF;
	FSW_WHT_ON;
	FSW_GRN_OFF;
	LGSTN_WHT_ON;
	LGSTN_GRN_OFF;
	SMSTN_WHT_ON;
	SMSTN_GRN_OFF;
	ERROR_WHT_ON;
	ERROR_RED_OFF;
	LBLS_WHT_ON;

	DSP_RESET_PA_L;																						// Make sure SR latch is reset and ready to operate
	DELAY_US(1000);
	DSP_RESET_PA_H;
	DELAY_US(1000);
	DSP_RESET_PA_L;
	DELAY_US(1000);
}
//################################################################################################################################################################################################
//						End of GPIO Setup
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Setup ePWM
//################################################################################################################################################################################################

void Setup_ePWM(void)
{

	//ePWM1A Setup - Sets PWM for R1 on 4046 Chip
	//ePWM1B Setup - Sets PWM for Green Transducer LED
	//ePWM2A Setup - Sets PWM for R2 on 4046 Chip
	//ePWM2B Setup - Sets PWM for AGC Min Level
	//ePWM3A Setup - Sets PWM for Phase Command
	//ePWM3B Setup - Sets PWM for Red Transducer LED
	//ePWM4A Setup - Sets PWM for Digital Frequency
	//ePWM5A Setup - Sets PWM for AGC Command
	//ePWM5B Setup - Sets PWM for Blue Transducer LED
	//ePWM6A Setup - Sets PWM for Digital AGC
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default

	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;															// HSPCLKDIV = /1
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;															// Count up mode
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

	EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)
	EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm3Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm4Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm5Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm6Regs.TBCTL.bit.FREE_SOFT = 2;

	// Set Up AQCTLA registers
	// When CTR = CMPA on Up Count - Clear
	// When CTR = CMPA on Up Count - Set
	// When CTR = 0 - Set
	// Modifying these can alter the direction of the duty cycle
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;

	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;

	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;

	EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;

	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm1Regs.CMPB 	= 3000;																				// Initialize green led here so there is no flash
	EPwm1Regs.TBPRD = 3000;																				// 50 kHz - PWM Signal  = TBPRD = fsysclockout/(fpwm*CLKDIV*HSPCLKDIV)

	EPwm2Regs.TBPRD = 3000;																				// TBPRD = 150e6/(50e3*1*1), CLKDIV and HSPCLKDIV are the divisor numbers both 1.

	EPwm3Regs.CMPB	= 3000;																				// Initialize red led here so there is no flash
	EPwm3Regs.TBPRD = 3000;																				// CTRMODE has a scaling factor of 0.5 if up/down mode is used

	EPwm4Regs.TBPRD = 3000;

	EPwm5Regs.CMPB	= 3000;																				// Initialize blue led here so there is no flash
	EPwm5Regs.TBPRD = 3000;

	EPwm6Regs.TBPRD = 3000;

	EALLOW;																								// Update the MUX here for these GPIO's so ring LED does not flash during startup
	GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 1;																// Set GPIO 1  as EPWM1B (PWM for TRCON_GRN)
	GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 1;																// Set GPIO 5  as EPWM1B (PWM for TRCON_RED)
	GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 1;																// Set GPIO 9  as EPWM1B (PWM for TRCON_BLU)
	EDIS;

	// Set initial values for ePWM modules
	R1_ePWM 		= R1_INIT_VAL;
	R2_ePWM 		= R2_INIT_VAL;
	AGC_Min_ePWM	= AGC_MIN_VAL;
	PHASE_CMD_ePWM 	= START_PHASE;
	DIG_FREQ_ePWM 	= LOW_VCO;
	AGC_CMD_ePWM 	= AGC_COMMAND_VAL;
	DIG_AGC_ePWM 	= AGC_START_VAL;
	RED_LED(0);
	GRN_LED(0);
	BLU_LED(0);
	sys_watchword|=0x0002;
}

//################################################################################################################################################################################################
//						End Setup ePWM1A
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Setup eCAP
//################################################################################################################################################################################################
void Setup_eCAP(void)
{
	ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;															// Capture Event 1 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CAP2POL = EC_RISING;															// Capture Event 2 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;															// Capture Event 3 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CAP4POL = EC_RISING;															// Capture Event 4 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;															// Do not reset counter on Capture Event 1 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;															// Do not reset counter on Capture Event 2 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;															// Do not reset counter on Capture Event 3 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;															// Do not reset counter on Capture Event 4 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;															// Enable Loading of CAP1-4 registers on a capture event
	ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap1Regs.ECCTL1.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)

	ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_ONESHT;														// Operate in one shot mode
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;														// Disable sync out signal
	ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;															// Disable sync in signal
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;															// Counter stop control (free running)

	ECap2Regs.ECCTL1.bit.CAP1POL = EC_RISING;															// Capture Event 1 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CAP2POL = EC_RISING;															// Capture Event 2 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CAP3POL = EC_RISING;															// Capture Event 3 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CAP4POL = EC_RISING;															// Capture Event 4 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;															// Do not reset counter on Capture Event 1 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;															// Do not reset counter on Capture Event 2 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;															// Do not reset counter on Capture Event 3 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;															// Do not reset counter on Capture Event 4 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;															// Enable Loading of CAP1-4 registers on a capture event
	ECap2Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap2Regs.ECCTL1.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)

	ECap2Regs.ECCTL2.bit.CONT_ONESHT = EC_ONESHT;														// Operate in one shot mode
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;														// Disable sync out signal
	ECap2Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;															// Disable sync in signal
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;															// Counter stop control (free running)

	ECap5Regs.ECCTL1.bit.CAP1POL = EC_RISING;															// Capture Event 1 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CAP2POL = EC_RISING;															// Capture Event 2 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CAP3POL = EC_RISING;															// Capture Event 3 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CAP4POL = EC_RISING;															// Capture Event 4 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;															// Do not reset counter on Capture Event 1 (absolute time stamp)
	ECap5Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;															// Do not reset counter on Capture Event 2 (absolute time stamp)
	ECap5Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;															// Do not reset counter on Capture Event 3 (absolute time stamp)
	ECap5Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;															// Do not reset counter on Capture Event 4 (absolute time stamp)
	ECap5Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;															// Enable Loading of CAP1-4 registers on a capture event
	ECap5Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap5Regs.ECCTL1.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)

	ECap5Regs.ECCTL2.bit.CONT_ONESHT = EC_ONESHT;														// Operate in one shot mode
	ECap5Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;														// Disable sync out signal
	ECap5Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;															// Disable sync in signal
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;															// Counter stop control (free running)

	sys_watchword|=0x0100;																				// Update sys_watchword
}

//################################################################################################################################################################################################
//						End of Setup eCAP
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						ADC Config
//################################################################################################################################################################################################
void ADCconfig()
{
	AdcRegs.ADCTRL1.all = 0;
	AdcRegs.ADCTRL1.bit.ACQ_PS = 7;																		// Sampling sw close time = (2 / HSPCLK) * 8 - ends up = .1us
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 0; 																	// 0 = Dual Sequencer Mode, 1 = cascaded sequencer
	AdcRegs.ADCTRL1.bit.CPS = 0;																		// divide by 1
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0; 																	// Single Run Mode
	AdcRegs.ADCTRL2.all = 0;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0;																// Disable SEQ1 interrupt
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 6;																	// ADC clock: FCLK = HSPCLK / (2 * ADCCLKPS)
																										// HSPCLK = 150MHz (see DSP2833x_SysCtrl.c)
																										// FCLK = 12.5 MHz
	AdcRegs.ADCTRL3.bit.SMODE_SEL = 1;																	// Simultaneous Sampling Mode
	AdcRegs.ADCMAXCONV.all = 0x0003;																	// 4 conversions from Sequencer 1 (Number of conversions minus 1)
																										// Really 8 b/c ADC is in Simultaneous Sampling mode
																										// When in Dual Sequencer Mode ADCMAXCONV.all is bits 0-2 are for SEQ1
																										// bits 4-6 are for SEQ2
	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;																// RESULT0 = ADCINA0 = VOLTS, 			RESULT1 = ADCINB0 = CURRENT
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;																// RESULT2 = ADCINA1 = I_MOT, 			RESULT3 = ADCINB1 = PHASE
	AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 7;																// RESULT4 = ADCINA7 = PHASE_ERROR, 	RESULT5 = ADCINB7 = VCO_IN
	AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 6;																// RESULT6 = ADCINA6 = AGC_ERROR, 		RESULT7 = ADCINB6 = AGC_PWM
																										// This is because ADC is in Simultaneous Sampling Mode
	sys_watchword|=0x0020;																				// Update sys_watchword
}
//################################################################################################################################################################################################
//						End ADC Config
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						SPI Init
//################################################################################################################################################################################################
void spi_init()
{
	SpiaRegs.SPICCR.all =0x000F;	             														// Reset on, rising edge, 16-bit char bits
	SpiaRegs.SPICTL.all =0x0006;    		    														// Enable master mode, normal phase,
                                                 	 	 	 	 	 	 	 	 	 	 	 	 	 	// Enable talk, and SPI int disabled.
	SpiaRegs.SPIBRR =0x007F;
    SpiaRegs.SPICCR.all =0x008F;		         														// Relinquish SPI from Reset
    SpiaRegs.SPIPRI.bit.FREE = 1;                														// Set so breakpoints don't disturb xmission
    sys_watchword|=0x0200;																				// Update sys_watchword
}
//################################################################################################################################################################################################
//						End SPI Init
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						SPI FIFO Init
//################################################################################################################################################################################################
void spi_fifo_init()
{
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x2044;
    SpiaRegs.SPIFFCT.all=0x0;
    sys_watchword|=0x0080;																				// Update sys_watchword
}
//################################################################################################################################################################################################
//						End SPI FIFO Init
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						SPI Transmit
//################################################################################################################################################################################################
unsigned int spi_xmit(Uint16 a)
{
	unsigned int spi_test_ctr;
	spi_test_ctr=0;
	SpiaRegs.SPITXBUF=a;																				// Send data to SPI device
	while((SpiaRegs.SPIFFRX.bit.RXFFST!=1)&&(spi_test_ctr<10000))										// Wait until data is received
		spi_test_ctr++;
	if(spi_test_ctr<10000)
	{
		rdata = SpiaRegs.SPIRXBUF;																		// Set rdata to Receive buffer (SDO)
		return XMT_OK;
	}
	else
		return XMT_FAIL;
}
//################################################################################################################################################################################################
//						End SPI Transmit
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Select analog or digital frequency (controls U28 and U11)
//################################################################################################################################################################################################
void ana_or_dig_freq(int a)
{
	if (a == ANALOG)
	{
		ANA_FREQ_ON;
		DIG_FREQ_OFF;
	}
	else
	{
		DIG_FREQ_ON;
		ANA_FREQ_OFF;
	}
}
//################################################################################################################################################################################################
//						Select analog or digital frequency
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Select analog or digital AGC (controls U26 and U12)
//################################################################################################################################################################################################
void ana_or_dig_AGC(int a)
{
	if (a == ANALOG)
	{
		ANA_AGC_ON;
		DIG_AGC_OFF;
	}
	else
	{
		DIG_AGC_ON;
		ANA_AGC_OFF;
	}
}
//################################################################################################################################################################################################
//						End Select analog or digital AGC
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						AGC SOFT D TO A Sub Routine
//################################################################################################################################################################################################
void AGC_SOFT_D_TO_A(void)
{
	unsigned int k;
	unsigned int agc_error_avg;
	static unsigned int dig_agc_ctr;
	for(k=SAMPLE_WINDOW_VALS-1; k>0; k--)																// Initialized to 0 in ultrasound control sub routine
		sample_window_2[k]=sample_window_2[k-1];														// Move last reading down the buffer window (FIFO)
	sample_window_2[0]=agc_error_val;																	// Set current value of agc_error_val to position 0 of sample_window_2
	agc_error_avg=0;																					// Reset agc_error_avg
	for(k=0; k<SAMPLE_WINDOW_VALS; k++)																	// Add up all agc_error_val's
		agc_error_avg+=sample_window_2[k];
	agc_error_avg/=10;																					// Calculate average
	if((lock)||(restart_ctr>=MAX_RESTARTS))																// Wait for lock
	{
		if((DIG_AGC_ePWM<2500)&&(agc_error_avg<2048))													// With a limit on to how high to go, raise digital PWM and try to minimize error for minimum transient.
		{
			if(dig_agc_ctr<1)																			// Slight delay that seems to work well. Increments PWM only every second time thru routine. Allows averaging window to move.
				dig_agc_ctr++;
			else
			{
				dig_agc_ctr=0;																			// Initiate another delay
				DIG_AGC_ePWM++;																			// Increment PWM
			}
		}
		else
		{
			ana_or_dig_AGC(ANALOG);																		// Turn on analog control. Note: Hardware architecture requires BOTH these be turned on for analog control
			ana_agc_on=1;																				// Set ana_agc_on flag
		}
	}
}
//################################################################################################################################################################################################
//						End AGC SOFT D TO A Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						ANA RUNNING AND lockED Sub Routine
//################################################################################################################################################################################################
void ANA_RUNNING_AND_LOCKED(void)
{
	unsigned int j;
	unsigned int phase_error_avg;
	sys_watchword|=0x0300;																				// Update sys_watchword
	if(new_ad_vals)																						// Perform the following algorithms if the A/D has new values
	{
		for(j=SAMPLE_WINDOW_VALS-1; j>0; j--)															// Sliding window of phase error (FIFO)
			sample_window[j]=sample_window[j-1];
		sample_window[0]=phase_error_val;
		phase_error_avg=0;
		for(j=0; j<10; j++)
			phase_error_avg+=sample_window[j];
		phase_error_avg/=10;																			// Average phase error
		if((!lost_lock)&&(!restart_timer)&&(pll_on)&&(restart_ctr<MAX_RESTARTS))						// Monitor lock status
		{
			pll_ctr=0;
			if((phase_error_avg<2500)&&(phase_error_avg>1800)&&(vco_in_val<3500))						// Phase error within limits and VCO not too far?
			{
				if(capture_ctr<201)																		// True for > 200 times = lock
					capture_ctr++;																		// increment if sys is locked
				if(sys_stable&&lock)
				{
					vco_in_last_array[vco_in_last_position] = vco_in_val;
					if (vco_in_last_position<999)
					{
						vco_in_last_position++;
						array_location_newest = vco_in_last_position-1;
						array_location_oldest = vco_in_last_position;
					}
					else
					{
						vco_in_last_position=0;
						array_location_newest = 999;
						array_location_oldest = vco_in_last_position;
					}
					if (vco_in_last_ctr<1000)
						vco_in_last_ctr++;
					else if (abs(vco_in_last_array[array_location_newest]-vco_in_last_array[array_location_oldest])>20)
						vco_in_last = vco_in_last_array[array_location_oldest];
					else
						vco_in_last=vco_in_val-100;														// Subtract 100 for error correction while probe is loaded
				}
				ana_run_status=SYS_ANA_RUN_OK;															// Tell timer0 things OK
			}
			if((capture_ctr>200)&&(!lock))																// Sys locked if phase error is within limits 100 times
			{
				restart_timer=0;																		// Reset restart timers
				restart_ctr=0;
				lock=1;																					// Tell system it is locked
			}
			if(vco_in_val>=3500)																		// Initiate a restart if vco_in_val has flown up to top of window
				lost_lock=1;																			// Indicate to system that it has lost lock
		}
		else if((restart_ctr<MAX_RESTARTS)&&(pll_on||restart_timer))									// Go here if re-start required. PLL will be on @ first entry in process, after that, restart_timer will be on
		{																								// If no PLL and no restart_timer, system has turned on US but still in dig mode sitting at low window
			if(restart_timer==0)																		// Set up init conditions for re-start
			{
				ana_run_status=SYS_RESTART;																// Tell timer0 re-starting
				restarted+=20;																			// Counter used to decrement PHASE_COMMAND for each restart
				ultrasound_on_off(ULTRA_OFF);															// Turn ultrasound off
				sys_on=0;																				// Set sys_on indicator to 0
				if(START_PHASE>MIN_PHASE+40)
					START_PHASE-=40;
				if(START_PHASE<MIN_PHASE)
					START_PHASE=MIN_PHASE;
				PHASE_CMD_ePWM=START_PHASE;
				restart_timer++;																		// This controls re-start timing
				lock=0;																					// Tell system it is no longer locked
				agc_delay_ctr=0;
				if(!vco_in_last)
					restart_ctr++;																		// This counts number of attempts - then fails
				else
					restart_ctr=MAX_RESTARTS;
			}
			else if(restart_timer>=1000)																// Timer finished and ready to try capture
			{
				restart_timer=0;
				for(j=0; j<SAMPLE_WINDOW_VALS; j++)														// Phase error vals go in here and are averaged - no history for re-start
					sample_window[j]=0;
				lost_lock=0;																			// Update indicators
				sys_on=1;																				// Done resetting values allow system to turn on
			}
			else																						// Count up time to next re-start
			{
				sys_on=0;																				// Keep system off until restart timer has reached its limit
				restart_timer++;
			}
		}
		else if(!pll_on)																				// Sys has re-started, is in dig mode and waiting for signals to come up before allowing analog PLL control
		{
			if(pll_ctr<PLL_CTR_MAX)																		// Should not be in this mode too long
				pll_ctr++;
			else
				ana_run_status=FREQ_ERROR;
		}
		if(restart_ctr>=MAX_RESTARTS)
		{
			sys_on=1;																					// Allow system to run in digital frequency mode
			vco_freq_ctr=0;																				// Reset delay (used when setting vco_in_val to vco_in_last for digital frequency
		}
	}
}
//################################################################################################################################################################################################
//						End ANA RUNNING AND lockED Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						DETECT AGC FAULT Sub Routine
// Setup to fault on the broken box and ran into problems on my test box while testing because resistors in 4046 were not glued down.  Come back and test.
// To recreate issue remove R71 (not sure if the resistor was bad open or short, I'm guessing open)
//################################################################################################################################################################################################
void DETECT_AGC_FAULT(void)
{
	sys_watchword|=0x0400;
	if(start_ctr>100)
	{
		if ((agc_pwm_val>2200)&&((i_mot_val<100)||(volts_val<200)||(current_val<200))&&ana_agc_on)		// AGC has ramped up and is not coming back, (no control of AGC)
			agc_fault_ctr++;
		else
			agc_fault_ctr=0;
		if(agc_fault_ctr>5000)
		{
			ultrasound_on_off(ULTRA_OFF);
			ana_run_status=AGC_FAULT;
			Recoverable_Error();
		}
		if((agc_error_val<1000)&&(agc_pwm_val>2200))													// This is a normal overload
			overload_ctr++;
		else
			overload_ctr=0;
		if(overload_ctr>10000)
			ana_run_status=OVERLOAD_FAULT;
		else
			ana_run_status=SYS_ANA_RUN_OK;
	}
	else
		ana_run_status=SYS_ANA_RUN_OK;
}
//################################################################################################################################################################################################
//						End DETECT AGC FAULT Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Watch Dog Sub Routine
//################################################################################################################################################################################################
unsigned int Sys_Watchdog(void)
{
	unsigned int bone_ctr;
	#define WATCHDOG_MAX	3000
	if(sys_watchword==0xFFFF)
	{
		sys_watchword=0;
		WD_BONE_ON;																						// GIVE_DOG_BONE;
		bone_ctr=20;
		while(bone_ctr>0)
			bone_ctr--;
		WD_BONE_OFF;
		sys_watchdog_ctr=0;
		return SYS_DOG_OK;
	}
	else
	{
		sys_watchdog_ctr++;
		if(sys_watchdog_ctr>WATCHDOG_MAX)																// Issue fault if sys_watchword is not 0xFFFF after certain amount of time
		{																								// This will cause the DSP to reset
			ultrasound_on_off(ULTRA_OFF);
			return SYS_DOG_OUT;
		}
		else
			return SYS_DOG_OK;
	}
}
//################################################################################################################################################################################################
//						End Watch Dog Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Co Cal Routine
//################################################################################################################################################################################################
unsigned int Cal_I_mot(void)
{
	#define I_MOT_CTR_MAX	1000000
	unsigned int pot_change;
	unsigned int i_mot_valid_ctr;
	long unsigned int i_mot_ctr=0;
	unsigned int volts_ctr=0;
	DIG_FREQ_ePWM=LOW_VCO;																				// Start @ bottom of window
	while(VCO_IN>200)																					// Wait for bottom of window to stabilize
	{
		i_mot_ctr++;
		if(i_mot_ctr>=I_MOT_CTR_MAX)																	// Make sure it doesn't sit here forever
			return I_MOT_NG;
	}
	ultrasound_on_off(ULTRA_ON);																		// Turn on US
	i_mot_ctr=0;
	while(volts_ctr<75&&DIG_AGC_ePWM<HIGH_PWM)															// Bring voltage up so I_MOT is calculated correctly
	{
		if (new_ad_vals)
		{
			new_ad_vals = 0;
			if(volts_val<250)																			// Increase DIG_AGC if volts_val is < 250
			{
				DIG_AGC_ePWM++;
				volts_ctr=0;																			// Reset consecutive counter
			}
			else
				volts_ctr++;																			// Increment counter if volts_val is > 250 consecutively
		}
	}
	pot_val=START_POT_VAL;																				// Set the pot to the extreme value to pass the entire signal
	xmt_test_val=spi_xmit(0x1802);																		// Send initial command to POT to let it know the next command is to set the pot value
	if(xmt_test_val!=XMT_OK)
		Recoverable_Error();
	xmt_test_val=spi_xmit(pot_val);
	if(xmt_test_val!=XMT_OK)
		Recoverable_Error();
	i_mot_ctr=0;
	i_mot_valid_ctr=0;
	while((i_mot_valid_ctr<100)&&(pot_val<0x07ff)&&(i_mot_ctr<I_MOT_CTR_MAX))							// Limit travel of pot / Limit time in loop / Exit loop after 2 instances of 0 I motional
	{
		sys_watchword=0xFFF0;
		if(new_ad_vals)
		{
			DINT;																						// Disable interrupt
			if(pot_val<0x07f0)																			// Limit pot movement
			{
				if(i_mot_val>50)																		// Consider 50 A/D counts in the noise, goal is to zero out I_MOT
				{
					pot_val+=1;
					pot_change=1;																		// Write to pot only if there is change
					i_mot_valid_ctr=0;
				}
				else
				{
					pot_change=0;
					i_mot_valid_ctr++;
				}
				if(pot_change)																			// Change pot_val
				{
					xmt_test_val=spi_xmit(0x1802);
					if(xmt_test_val!=XMT_OK)
						Recoverable_Error();
					xmt_test_val=spi_xmit(pot_val);
					if(xmt_test_val!=XMT_OK)
						Recoverable_Error();
				}
			}
			new_ad_vals=0;																				// Need new A/D vales before next adjustment
			EINT;																						// Enable interrupt to get new A/D values
		}
		i_mot_ctr++;
	}
	ultrasound_on_off(ULTRA_OFF);																		// Turn off ultrasound before try to capture
	DIG_FREQ_ePWM=LOW_VCO;																				// Set digital frequency low to PLL to try and capture
	if((pot_val>=0x07fe)||(i_mot_ctr>=I_MOT_CTR_MAX))													// If sys could not zero out I motional, i_mot_ctr would have counted out
		return I_MOT_NG;
	else																								// System succussfully zeroed out I_MOT
	{
		co_done=1;
		return I_MOT_OK;
	}
}
//################################################################################################################################################################################################
//						End Co Cal Routine
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Ultrasound Control Sub Routine
//################################################################################################################################################################################################
void ultrasound_on_off(int a)
{
	unsigned int j;
	if (a == ULTRA_OFF)
	{
		while(AGC_CMD_ePWM>10&&agc_ramp_down_enabled)													// Slowly turn off system (avoid spikes)
		{
			sys_watchword|=0x00F0;
			if (agc_ramp_down_delay<500)
				agc_ramp_down_delay++;
			else
			{
				AGC_CMD_ePWM-=2;
				agc_ramp_down_delay = 0;
			}
		}
		agc_ramp_down_enabled = 0;
		ULTRASOUND_OFF;																					// Turn off ultra sound and restore flags and variables to initial state
		SYS_ON_OFF_2;
		DIG_AGC_ePWM=AGC_START_VAL;
		ana_or_dig_AGC(DIGITAL);
		agc_delay_ctr=0;
		if(!restarted)
		{
			PHASE_CMD_ePWM=START_PHASE;
			AGC_CMD_ePWM=SMALL_AGC;
		}
		ana_or_dig_freq(DIGITAL);
		pll_on=0;
		lock=0;
		for(j=0; j<10; j++)
			sample_window[j]=0;
		for(j=0; j<10; j++)
			sample_window_2[j]=0;
		ana_agc_on=0;
		capture_ctr=0;
		start_ctr=0;
		vco_adj_done = 0;
		vco_ctr = 0;
		sys_stable=0;
		Stable_Ctr=0;
		pll_ctr=0;
		agc_fault_ctr=0;
		if(vco_in_last>3700)
			vco_in_last=0;
	}
	else if (a == ULTRA_ON)
	{
		SYS_ON_ON_2;
		ULTRASOUND_ON;																					// Turn on ultra sound
		agc_ramp_down_enabled = 1;
	}
}
//################################################################################################################################################################################################
//						End Ultrasound Control Sub Routine
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Unrecoverable error Sub Routine
//################################################################################################################################################################################################
void Unrecoverable_Error(void)
{
	unsigned int bone_ctr;
	while(1)
	{
		ultrasound_on_off(ULTRA_OFF);
		sys_watchword|=0xFFF0;
		sys_fail=2;
		LGSTN_GRN_OFF;
		LGSTN_WHT_ON;
		SMSTN_GRN_OFF;
		SMSTN_WHT_ON;
		ERROR_RED_ON;
		ERROR_WHT_OFF;
		RED_LED(0);
		GRN_LED(0);
		WD_BONE_ON;																						// GIVE_DOG_BONE;
		bone_ctr=20;
		while(bone_ctr>0)
			bone_ctr--;
		WD_BONE_OFF;
	}
}
//################################################################################################################################################################################################
//						End Unrecoverable error Sub Routine
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Recoverable error Sub Routine
//################################################################################################################################################################################################
void Recoverable_Error(void)
{
	ultrasound_on_off(ULTRA_OFF);
	sys_fail=1;
	ERROR_WHT_OFF;
	ERROR_RED_ON;
	CHKPRB_WHT_OFF;
	CHKPRB_RED_ON;
	LGSTN_GRN_OFF;
	LGSTN_WHT_ON;
	SMSTN_GRN_OFF;
	SMSTN_WHT_ON;
}
//################################################################################################################################################################################################
//						End Recoverable error Sub Routine
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Error Lights Sub Routing
//################################################################################################################################################################################################

void Error_lights(void)
{
	sys_watchword |= 0x0F00;
	ERROR_WHT_OFF;
	ERROR_RED_ON;
	CHKPRB_WHT_OFF;
	CHKPRB_RED_ON;
	LGSTN_GRN_OFF;
	LGSTN_WHT_ON;
	SMSTN_GRN_OFF;
	SMSTN_WHT_ON;
	if (FSW_DETECT)
	{
		if ((mode==1)&&((!HSW_LARGE||!HSW_SMALL)))
			mode = 5;
		if ((mode==0)&&((HSW_LARGE&&HSW_SMALL)))
			hsw_sys_on=0;
	}
}
//################################################################################################################################################################################################
//						End Error Lights Sub Routing
//################################################################################################################################################################################################

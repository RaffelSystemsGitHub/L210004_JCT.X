
//Filename: 		Static Lumbar JCT Code
//Author:   		Caleb Schultz-Stachnik
//Date:   			2019/8/20
//File Version:		G
//Company:			Raffel Systems
//MCU: 				PIC16F1933

//Changelog:
//REV A: Initial Release
//REV B: Added all-zone-off -> massage-off feature, improved pin mapping for layout, PRCLOSE -> HOME
//REV C: Added independent massage + heat timers and timeout conditions
//REV D: Improved UART signal integrity (added ACK)
//REV E: MOTOR DIRECTION SPECIFIED FOR KLAUSNER AND SYNERGY
//REV F: Fix to allow hotswapping HC's, allows UART comm after disconnecting HC from JCT
//REV G: Fix for timeout -> 30 min, HR+PR not interrupt massage, AB deflate after timeout


//Includes
#include <stdio.h>
#include <htc.h>
#include <pic.h>
#include <stdbool.h>

__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_OFF & CP_ON & BOREN_OFF & CLKOUTEN_OFF);
__CONFIG(WRT_OFF & STVREN_OFF & BORV_LO  & LVP_OFF);

//The following is the sample UART communication code. This is included in the 
//handcontrol and the junction, with no variation between them. It is HIGHLY 
//recommended that this remains unchanged, and any changes occur in main() by 
//writing to the corresponding control register. 

/* A simple demonstration of serial communications which
 * incorporates the on-board hardware USART of the Microchip
 * PIC16Fxxx series of devices. */
 
#ifndef _SERIAL_H_
#define _SERIAL_H_

#define BAUD 9600      
#define FOSC 4000000L
#define NINE 0     /* Use 9bit communication? FALSE=8bit */

#define DIVIDER ((int)(FOSC/(16UL * BAUD) -1))
#define HIGH_SPEED 1

#if NINE == 1
#define NINE_BITS 0x40
#else
#define NINE_BITS 0
#endif

#if HIGH_SPEED == 1
#define SPEED 0x4
#else
#define SPEED 0
#endif

#if defined(_16F87) || defined(_16F88)
	#define RX_PIN TRISB2
	#define TX_PIN TRISB5
#else
	#define RX_PIN TRISC7
	#define TX_PIN TRISC6
#endif

/* Serial initialization */
#define init_comms()\
	RX_PIN = 1;	\
	TX_PIN = 1;		  \
	SPBRG = DIVIDER;     	\
	RCSTA = (NINE_BITS|0x90);	\
	TXSTA = (SPEED|NINE_BITS|0x20)

void putch(unsigned char);
unsigned char getch(void);
unsigned char getche(void);

#endif
 
void 
putch(unsigned char byte) 
{
    //long count = 0;
	/* output one byte */
	while(!TXIF){	/* set when register is empty */
		continue;
        /*
         * count++;
        if(count >= 10000000){
            break;
        }
        */
    }
	TXREG = byte;
}

unsigned char 
getch() {
    //long count = 0;
	/* retrieve one byte */
	while(!RCIF){	/* set when register is not empty */
        continue;
        /*
        count++;
        if(count >= 10000000){
            break;
        }
        */
    }
	return RCREG;	
}

unsigned char
getche(void)
{
	unsigned char c;
	putch(c = getch());
	return c;
}

//Define USART Commands
#define		MASSAGE_ON		'A'
#define		MASSAGE_OFF		'B'
#define		PR_OPEN			'C'
#define		PR_CLOSE		'D'
#define		PR_HOLD			'E'
#define		HR_OPEN			'F'
#define		HR_CLOSE		'G'
#define		HR_HOLD			'H'
#define		LUM_OPEN		'I'
#define		LUM_CLOSE		'J'
#define		LUM_HOLD		'K'
#define		Z0_ON			'L'
#define		Z0_OFF			'M'
#define		Z1_ON			'N'
#define		Z1_OFF			'O'
#define		Z2_ON			'P'
#define		Z2_OFF			'Q'
#define		Z3_ON			'R'
#define		Z3_OFF			'S'
#define		M_ALTERNATE		'T'
#define		M_WAVE			'U'
#define		M_PULSE			'V'
#define		HEAT_ON			'W'
#define		HEAT_OFF		'X'
#define     ACT_OFF         'Y'
#define     HOME            'Z'
#define     OP_GO           '+'
#define     SYNC            '?'
#define     SYNC_ACK        '!'
#define     ACK             '~'

//I/O Defines

#define		MOTOR_PIN		LATA1
#define		HR_OPEN_PIN		LATB1
#define		HR_CLOSE_PIN	LATB2
#define		PR_OPEN_PIN		LATB4
#define		PR_CLOSE_PIN	LATB3

#define		ZONE0_OUT		LATA7
#define		ZONE1_OUT		LATA5
#define		ZONE2_OUT		LATA4
#define		ZONE3_OUT		LATA3
#define		LUM_HOLD_PIN	LATA0
#define 	HEAT_OUT		LATA2


//Timer setup for 0.05 seconds
#define		PERIOD			50000							// period in uS
#define		XTAL			4000000							// crystal frequency - 4MHz
#define 	_XTAL_FREQ		4000000
#define		IPERIOD			(4 * 1000000 / XTAL)			// Period of instruction clock in uSeconds
#define		SCALE			2								// Timer 0 prescaler
#define		T0_TICKS 		256								// Number of counts for interrupt
#define		TICK_PERIOD		(SCALE * IPERIOD)				// Period (uSec) of one increment of timer 0
#define		RELOADS			((PERIOD/T0_TICKS)/TICK_PERIOD)
#define		SEC_RELOAD		200

//Global Constants
#define		Heat_TimeOut	36000							//30 min => 30*60/.05
#define		Massage_TimeOut	24000							//20 min => 20*60/.05
#define 	pwmPeriod 		100
#define		WAVE			0
#define		ALTERNATE		1
#define		PULSE			2
#define     VENT_TIME       200
#define     LONG_TOUT       20*60*29.5
#define     SHORT_TOUT      20*60*2
#define     ARR_LENGTH      5
#define     NELEMS(x)       (sizeof(x)/sizeof((x)[0]))

//Global Variables
volatile int w_count = 0;									//wave counter, only used for wave func timing
volatile int a_count = 0;									//alternate counter, only used for alternate func
volatile int p_count = 0;									//pulse counter, only used for pulse func timing
volatile int z_count = 0;									//pulse counter, only used for pulse zone switching

volatile bool zone0_en = true;	//init with all on so pump operates correctly
volatile bool zone1_en = true;
volatile bool zone2_en = true;
volatile bool zone3_en = true;
volatile bool alt_en   = true;	//massage func select needs initial value
volatile bool wav_en   = false;
volatile bool pul_en   = false;
volatile bool vent_t   = false;
volatile bool m_flag   = false;

volatile int  time_out = SEC_RELOAD;
volatile int  comm_timeout = 0;
volatile int  massage_timeout = 0;
volatile int  heat_timeout = 0;
volatile int  TimerReload = 0;
volatile int  pwm_count = 0;
volatile int  vent_c = VENT_TIME;
volatile char rc_error;
volatile char rc;
volatile char last_rc[ARR_LENGTH] = {0};
volatile char tx_char[ARR_LENGTH] = {0};
volatile bool heat_en = false;
volatile bool heat_reset = false;
volatile bool power = false;
volatile bool timer = false;

//Function Prototypes
void wave();
void alternate();
void pulse();
void jct_massage_on();
void jct_massage_off();
void jct_massage_reset();


//Main code
void main(void){
	//Internal Oscillator Setup
	OSCCON = 0b01101000; 	//Internal 16MHz
	init_comms();			//run header func for serial init
    
	// I/O defines
	// 1 = input, 0 = output	
	TRISA  = 0b01000000;
	//		   |||||||\-- LUM_HOLD_PIN
	//		   ||||||\--- MOTOR_PIN
	//		   |||||\---- HEAT_OUT
	//		   ||||\----- ZONE3_OUT
	//		   |||\------ ZONE2_OUT
	//		   ||\------- ZONE1_OUT
	//		   |\-------- unused
	//		   \--------- ZONE0_OUT
	ANSELA = 0b00000000;
	
	TRISB  = 0b11100001;
	//		   |||||||\-- unused
	//		   ||||||\--- OPEN_PR_PIN	IOC pin
	//		   |||||\---- CLOSE_PR_PIN	IOC pin
	//		   ||||\----- OPEN_HR_PIN	IOC pin
	//		   |||\------ CLOSE_HR_PIN	IOC pin
	//		   ||\------- unused
	//		   |\-------- unused
	//		   \--------- unused
	ANSELB = 0b00000000;
	
	TRISC  =  0b10111111;
	//		    |||||||\-- unused
	//		    ||||||\--- unused
	//		    |||||\---- unused
	//		    ||||\----- unused
	//		    |||\------ unused
	//		    ||\------- unused
	//		    |\-------- TX
	//		    \--------- RX
	//ANSELC =  0b00000000;		//not implemented on PIC16F1933
	
	//Interrupt Definition
	INTCON = 0b10100000;
	//		   |||||||\-- IOC Flag 						IOCIF
	//		   ||||||\--- External Flag					INTF
	//		   |||||\---- TMR0 Flag						TMR0IF
	//		   ||||\----- IOC Enable					IOCIE
	//		   |||\------ External Interrupt Enable		INTE
	//		   ||\------- TMR0 Interrupt Enable			TMR0IE
	//		   |\-------- Peripheral Interrupt Enable	PEIE
	//		   \--------- Global Interrupt Enable 		GIE
		
	//Baud Rate Definition 
	BAUDCON = 0b00000000;
	SPBRGL  = 0x0C; //=> 51 = 1200 baud
	SPBRGH  = 0b00000000;
	OPTION_REG = 0b00001000;//enable tmr0 prescale
	WPUB = 0b00000000;
	GIE  = 1;
	TMR0IE = 1;

	LATA = 0x00;	//clear all port outputs
	LATB = 0x00;
	LATC = 0x00;
	
	while(1){
        bool echo = true;
        do{
            char c;
            unsigned int q = 0;
            last_rc[0] = getch();
            if(last_rc[0] == SYNC){
                sync:
                q = 0;
                putch(SYNC_ACK);
                c = getch();
                if(c == ACK){
                    //echo = false;
                    for(unsigned int i = 0; i < NELEMS(last_rc); ++i){
                        last_rc[i] = 0;
                    }
                }                
            }
            else{
                q = 1;
            }
            
            for(unsigned int i = q; i < NELEMS(last_rc); ++i){
                last_rc[i] = getch();   //RX 5 commands from HC
                if(last_rc[i] == SYNC){
                    goto sync;
                }
            }
            for(unsigned int i = 0; i < NELEMS(last_rc); ++i){
                putch(last_rc[i]);      //Echo 5 commands back to HC
                __delay_us(5);
            }
            c = getch();            //ACK from HC, only ACK when all 5 echos match
            if(c == SYNC){
                goto sync;
            }
            else if(c == OP_GO){
                rc = last_rc[0];    //set correct command to execute
                echo = false;       //exit while         
            }
        }while(echo);  
		
		if(rc){	//set timer if there was a transmission
			timer = 1;
            if(rc != ACT_OFF){
                vent_t = false;
            }
		}
		if(OERR || FERR){
			rc_error = RCSTA;
			CREN = 0;	//clear error
            SPEN = 0;
			CREN = 1;	//re-enable
            SPEN = 1;
		}
        if(rc == ACT_OFF){
            PR_OPEN_PIN = 0;
            PR_CLOSE_PIN = 0;
            HR_OPEN_PIN = 0;
            HR_CLOSE_PIN = 0;
            if(power){
                
            }
            else if(!vent_t){
                power = false;
                MOTOR_PIN = 0;
                ZONE0_OUT = 0;
                LUM_HOLD_PIN = 0;
            }
        }
		if(rc == MASSAGE_ON){
			jct_massage_on();
            m_flag = true;
            massage_timeout = LONG_TOUT;
		}
		if(rc == MASSAGE_OFF){
			jct_massage_off();
            jct_massage_reset();
            heat_en = false;
            HEAT_OUT = 0;
            vent_t = true;
		}
		if(rc == PR_OPEN){
            if(heat_en){
				HEAT_OUT = 0;
                heat_en = false;
                heat_reset = true;
			}
			if(last_rc[0] == PR_CLOSE){
				PR_CLOSE_PIN = 0;
			}
            HR_OPEN_PIN = 0;
            HR_CLOSE_PIN = 0;
            PR_CLOSE_PIN = 0;
			PR_OPEN_PIN = 1;
            if(!m_flag){
                MOTOR_PIN = 0;
                ZONE0_OUT = 0;
                //vent_t = true;
                
            }
		}
		if(rc == PR_CLOSE){
			if(heat_en){
				HEAT_OUT = 0;
                heat_en = false;
                heat_reset = true;
			}
			if(last_rc[0] == PR_OPEN){
				PR_OPEN_PIN = 0;
			}
            if(!m_flag){
                MOTOR_PIN = 0;
                ZONE0_OUT = 0;
                //vent_t = true;
            }
            HR_OPEN_PIN = 0;
            PR_OPEN_PIN = 0;
            PR_CLOSE_PIN = 1;
            HR_CLOSE_PIN = 1;
		}
		if(rc == PR_HOLD){
			PR_OPEN_PIN  = 0;
			PR_CLOSE_PIN = 0;
            HR_OPEN_PIN  = 0;
            HR_CLOSE_PIN = 0;
			if(heat_reset){
				HEAT_OUT = 1;
                heat_en = true;
                heat_reset = false;
			}
		}
		if(rc == HR_OPEN){
			if(heat_en){
				HEAT_OUT = 0;
                heat_en = false;
                heat_reset = true;
			}
			if(last_rc[0] == HR_CLOSE){
				HR_CLOSE_PIN = 0;
			}
            if(!m_flag){
                MOTOR_PIN = 0;
                ZONE0_OUT = 0;
                //vent_t = true;
            }
            HR_OPEN_PIN = 1;
            PR_OPEN_PIN = 0;
            HR_CLOSE_PIN = 0;
            PR_CLOSE_PIN = 0;
		}
		if(rc == HR_CLOSE){
			if(heat_en){
				HEAT_OUT = 0;
                heat_en = false;
                heat_reset = true;
			}
			if(last_rc[0] == HR_OPEN){
				HR_OPEN_PIN = 0;
			}
            if(!m_flag){
                MOTOR_PIN = 0;
                ZONE0_OUT = 0;
                //vent_t = true;
            }
            HR_OPEN_PIN = 0;
            PR_OPEN_PIN = 0;
            HR_CLOSE_PIN = 1;
            PR_CLOSE_PIN = 0;
		}
		if(rc == HR_HOLD){
			HR_OPEN_PIN = 0;
			HR_CLOSE_PIN = 0;
			if(heat_reset){
				HEAT_OUT = 1;
                heat_en = true;
                heat_reset = false;
			}
		}
		if(rc == LUM_OPEN){
			if(last_rc[0] == LUM_CLOSE){
			}
			else{
                jct_massage_off();
                jct_massage_reset();
				//GIE = 0;
                HR_OPEN_PIN = 0;
                PR_OPEN_PIN = 0;
                HR_CLOSE_PIN = 0;
                PR_CLOSE_PIN = 0;                
				MOTOR_PIN = 1;
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
                if(heat_en){
                    HEAT_OUT = 0;
                    heat_en = false;
                    heat_reset = 1;
                }
			}
		}
		if(rc == LUM_CLOSE){
			if(last_rc[0] == LUM_OPEN){
			}
			else{
                jct_massage_off();
                jct_massage_reset();
				//GIE = 0;
                HR_OPEN_PIN = 0;
                PR_OPEN_PIN = 0;
                HR_CLOSE_PIN = 0;
                PR_CLOSE_PIN = 0;
				power = false;
				MOTOR_PIN = 0;
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
                if(heat_en){
                    HEAT_OUT = 0;
                    heat_en = false;
                    heat_reset = true;
                }
			}
		}
		if(rc == LUM_HOLD){
  			GIE = 1;
			power = false;
            HR_OPEN_PIN = 0;
            PR_OPEN_PIN = 0;
            HR_CLOSE_PIN = 0;
            PR_CLOSE_PIN = 0;
			MOTOR_PIN = 0;
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 0;
            if(heat_reset){
                HEAT_OUT = 1;
                heat_en = true;
                heat_reset = false;
            }
		}

		if(rc == Z0_ON){
			zone0_en = true;
            massage_timeout = LONG_TOUT;
		}
		if(rc == Z0_OFF){
			zone0_en = false;
            massage_timeout = LONG_TOUT;
		}
		if(rc == Z1_ON){
			zone1_en = true;
            massage_timeout = LONG_TOUT;
		}
		if(rc == Z1_OFF){
			zone1_en = false;
            massage_timeout = LONG_TOUT;
		}
		if(rc == Z2_ON){
			zone2_en = true;
            massage_timeout = LONG_TOUT;
		}
		if(rc == Z2_OFF){
			zone2_en = false;
            massage_timeout = LONG_TOUT;
		}
		if(rc == Z3_ON){
			zone3_en = true;
            massage_timeout = LONG_TOUT;
		}
		if(rc == Z3_OFF){
			zone3_en = false;
            massage_timeout = LONG_TOUT;
		}
		if(rc == M_ALTERNATE){
			alt_en = true;
			wav_en = false;
			pul_en = false;
            massage_timeout = LONG_TOUT;
		}
		if(rc == M_WAVE){
			alt_en = false;
			wav_en = true;
			pul_en = false;
            massage_timeout = LONG_TOUT;
		}
		if(rc == M_PULSE){
			alt_en = false;
			wav_en = false;
			pul_en = true;
            massage_timeout = LONG_TOUT;
		}
		if(rc == HEAT_ON){
			heat_en = true;
            heat_timeout = LONG_TOUT;
		}
		if(rc == HEAT_OFF){
			heat_en = false;
            HEAT_OUT = 0;
		}
	}
}
	
static void interrupt isr(void){
	if(TMR0IF){
		time_out--;
		if(!time_out){                  //timeout timing handling
			time_out = SEC_RELOAD;
			if(massage_timeout){
				massage_timeout--;
				if(!massage_timeout){ 	//turn off massage and heat after 20 min of no communication
					power = false;
                    jct_massage_off();
					jct_massage_reset();
					timer		= 0;
                    vent_t = true;
				}
			}
            if(heat_timeout){
                heat_timeout--;
                if(!heat_timeout){
                    heat_en = 0;
                    timer = 0;
                }
            }
            if(vent_t){
                if(vent_c > 0){
                    LUM_HOLD_PIN = 1;
                    --vent_c;
                }
                else{
                    LUM_HOLD_PIN = 0;
                    vent_c = VENT_TIME;
                    vent_t = false;
                }
            }
            else{
                vent_c = VENT_TIME;
            }
		}		
		
		pwm_count++;
		if(pwm_count >= pwmPeriod){
			pwm_count = 0;
		}
		if(pwm_count < 75){
			if(power){
				MOTOR_PIN = 1;
			}
		}
		else{
			if(power){
				MOTOR_PIN = 0;
			}
		}

		if(heat_en){
			if(pwm_count < 70){ 	//70% duty for heat on
				HEAT_OUT = 1;
			}
			else{
				HEAT_OUT = 0;
			}
		}
		else{
			HEAT_OUT = 0;
		}
		
		if(TimerReload){ 			//massage timing handling
			TimerReload--;
		}
		else{
			TimerReload = RELOADS;
			if(!power){             //turn off all zones and reset massage counters
				jct_massage_reset();
			}
			else{
				if(alt_en){
					alternate();    //internal global counter controls air solenoids
                    //wave();
				}
				if(wav_en){
					wave();
				}
				if(pul_en){
					pulse();
                    //wave();
				}				
			}	
		}
		TMR0IF = 0;					//clear timer0 flag
	}
}

void wave(){
    if(heat_en){
        HEAT_OUT = 1;
    }
    else{
        HEAT_OUT = 0;
    }
	if(zone0_en && !zone1_en && !zone2_en && !zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 130){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && !zone2_en && !zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 130){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(!zone0_en && !zone1_en && zone2_en && !zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 130){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(!zone0_en && !zone1_en && !zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 130){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(zone0_en && zone1_en && !zone2_en && !zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 330){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(zone0_en && !zone1_en && zone2_en && !zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 330){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}	
	else if(zone0_en && !zone1_en && !zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 330){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && zone2_en && !zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 330){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && !zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 330){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(!zone0_en && !zone1_en && zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 330){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(zone0_en && zone1_en && zone2_en && !zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 400){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 500){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 530){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(zone0_en && zone1_en && !zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 400){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 500){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 530){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(zone0_en && !zone1_en && zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 400){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 500){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 530){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 400){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 500){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 530){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else{
			w_count = 0;
		}
	}
	else if(zone0_en && zone1_en && zone2_en && zone3_en){
		if(w_count < 100){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 200){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 300){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 0;
			w_count++;
		}
		else if(w_count < 400){
			ZONE0_OUT = 1;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 500){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 1;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 600){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 1;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 700){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 1;
			w_count++;
		}
		else if(w_count < 730){
			ZONE0_OUT = 0;
			LUM_HOLD_PIN = 1;
			ZONE1_OUT = 0;
			ZONE2_OUT = 0;
			ZONE3_OUT = 0;
			w_count++;
		}		
		else{
			w_count = 0;
		}
	}
	else{
		ZONE0_OUT = 0;
		LUM_HOLD_PIN = 0;
		ZONE1_OUT = 0;
		ZONE2_OUT = 0;
		ZONE3_OUT = 0;
	}
}

void pulse(){	
    if(heat_en){
        HEAT_OUT = 1;
    }
    else{
        HEAT_OUT = 0;
    }
	if(zone0_en && !zone1_en && !zone2_en && !zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && !zone2_en && !zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(!zone0_en && !zone1_en && zone2_en && !zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(!zone0_en && !zone1_en && !zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(zone0_en && zone1_en && !zone2_en && !zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(zone0_en && !zone1_en && zone2_en && !zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(zone0_en && !zone1_en && !zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && zone2_en && !zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && !zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(!zone0_en && !zone1_en && zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(zone0_en && zone1_en && zone2_en && !zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 30){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(zone0_en && zone1_en && !zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 30){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(zone0_en && !zone1_en && zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 30){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(!zone0_en && zone1_en && zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 30){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}
	else if(zone0_en && zone1_en && zone2_en && zone3_en){
		if(z_count < 10){
			if(p_count < 20){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 20){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 30){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else if(z_count < 40){
			if(p_count < 20){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				p_count++;
			}
			else if(p_count < 30){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				p_count++;
			}
			else{
				p_count = 0;
				z_count++;
			}
		}
		else{
			z_count = 0;
		}
	}	
	else{
		ZONE0_OUT = 0;
		LUM_HOLD_PIN = 0;
		ZONE1_OUT = 0;
		ZONE2_OUT = 0;
		ZONE3_OUT = 0;
	}
	//return;
}

void alternate(){	
	if(heat_en){
        HEAT_OUT = 1;
    }
    else{
        HEAT_OUT = 0;
    }
		if(zone0_en && zone1_en && zone2_en && zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 1;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone0_en && zone1_en && zone2_en && !zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone0_en && zone1_en && zone3_en && !zone2_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
        else if(zone1_en && zone2_en && zone3_en && !zone0_en){
			if(a_count < 100){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 1;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone0_en && zone2_en && zone3_en && !zone1_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 1;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone0_en && zone1_en && !zone2_en && !zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone0_en && zone2_en && !zone1_en && !zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone0_en && zone3_en && !zone1_en && !zone2_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone1_en && zone2_en && !zone0_en && !zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone1_en && zone3_en && !zone0_en && !zone2_en){
			if(a_count < 100){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone2_en && zone3_en && !zone0_en && !zone1_en){
			if(a_count < 100){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 200){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone0_en && !zone1_en && !zone2_en && !zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 1;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 130){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone1_en && !zone0_en && !zone2_en && !zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 1;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 130){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone2_en && !zone0_en && !zone1_en && !zone3_en){
			if(a_count < 100){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 1;
				ZONE3_OUT = 0;
				a_count++;
			}
			else if(a_count < 130){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
		else if(zone3_en && !zone0_en && !zone1_en && !zone2_en){
			if(a_count < 100){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 1;
				a_count++;
			}
			else if(a_count < 130){
				ZONE0_OUT = 0;
				LUM_HOLD_PIN = 1;
				ZONE1_OUT = 0;
				ZONE2_OUT = 0;
				ZONE3_OUT = 0;
				a_count++;
			}
			else{
				a_count = 0;
			}
		}
	
	else{
		ZONE0_OUT = 0;
		LUM_HOLD_PIN = 0;
		ZONE1_OUT = 0;
		ZONE2_OUT = 0;
		ZONE3_OUT = 0;
	}
	//return;
}

void jct_massage_on(){
    power = true;
	zone0_en = true;
	zone1_en = true;
	zone2_en = true;
	zone3_en = true;
	alt_en = true;
	wav_en = false;
	pul_en = false;
    m_flag = true;
}

void jct_massage_off(){
    power = false;
    m_flag = false;
	MOTOR_PIN = 0;
    LUM_HOLD_PIN = 0;
    ZONE0_OUT = 0;
	LUM_HOLD_PIN = 0;
	ZONE1_OUT = 0;
	ZONE2_OUT = 0;
	ZONE3_OUT = 0;    
}

void jct_massage_reset(){
    zone0_en = false;
	zone1_en = false;
	zone2_en = false;
	zone3_en = false;
	a_count = 0;
	p_count = 0;
	w_count = 0;
	z_count = 0;
}
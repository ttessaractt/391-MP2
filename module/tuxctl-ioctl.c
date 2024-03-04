/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define MASK_BYTE	0x0F		//00001111
#define MASK_4BIT 	0xF0		//11110000
#define ADD_DEC 	0x10		//make dp in (in 7 seg input) 1
#define ONE			0x1 		//single bit mask
#define MASK		0x00F		//0000 0000 0000 1111


#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)


unsigned char button_array[2];		//array to hold data from buttons
unsigned int save_LED;				//save LED status

int tux_init(struct tty_struct* tty);		//initilize helper function
int set_buttons(struct tty_struct* tty, unsigned long arg);
int set_LED(struct tty_struct* tty, unsigned long arg);

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

	switch(a){		//when a is in the case

		case MTCP_BIOC_EVENT:		//button inputs will be in packets b & c
			button_array[0] = b;	//| 1 X X X | C | B | A | START |
			button_array[1] = c;	//| 1 X X X | right | down | left | up |
			break;

		case MTCP_RESET:			//reinitilize controller to state it was in before reset
			tux_init(tty);				// re-init
			set_LED(tty, save_LED);	// set LEDs
			break;

		case MTCP_ACK:
			break;

		default:
			return;
	}
	a = 0x00;
	b = 0x00;
	c = 0x00;
	return;
    /*printk("packet : %x %x %x\n", a, b, c); */
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
    switch (cmd) {
	case TUX_INIT:
		return tux_init(tty);	//initilize variables

	case TUX_BUTTONS:
		return set_buttons(tty, arg);

	case TUX_SET_LED:
		return set_LED(tty, arg);

	case TUX_LED_ACK:
		return 0;

	case TUX_LED_REQUEST:
		return 0;

	case TUX_READ_LED:
		return 0;

	default:
	    return -EINVAL;
    }
}

/*
 * tux_init
 *   DESCRIPTION: initilizes the TUX variables
 *   INPUTS: tty -- the device
 *   OUTPUTS: none
 *   RETURN VALUE: 0 on success, -EINVAL on failure
 *   SIDE EFFECTS: none
 */
int tux_init(struct tty_struct* tty){
	//init vars
	int check;
	unsigned char buf[2];		

	//set buffer
	buf[0] = MTCP_BIOC_ON;		//enable button interrupt on change
	buf[1] = MTCP_LED_USR;		//set LEDs to user mode

	//write 2 bytes to device
	check = tuxctl_ldisc_put(tty, buf, 2);

	//check if all bytes were written to device
	if (check != 0){
		return -EINVAL;
	}
	else{
		return 0;
	}
};


/*
 * set_buttons
 *   DESCRIPTION: changes the variable arg points to based on the button values
 *   INPUTS: tty -- the device
 *			 arg -- the 32 bit integer
 *   OUTPUTS: none
 *   RETURN VALUE: 0 on success, -EINVAL on failure
 *   SIDE EFFECTS: changes the low byte of arg
 */
int set_buttons(struct tty_struct* tty, unsigned long arg){
	//init variables
	int hold;
	int check;

	//vairables for bit swapping
	unsigned int i, j; 	// positions of bit sequences to swap
	unsigned int n;    	// number of consecutive bits in each sequence
	unsigned int b;    	// bits to swap reside in b
	unsigned int r;    	// bit-swapped result goes here
	unsigned int x;		//temp XOR

	//check if pointer is valid
	if ((int*)arg == NULL){		
		printk("NULL\n");
		return -EINVAL;
	}

	hold = 0;				//will eventually hold |right|left|down|up|c|b|a|start|

	//swap down and left bits
	i = 1;					//swap starting at bit 1 (L)
	j = 2;					//with bit 2 (D)
	n = 1;					//swap 1 bit
	b = button_array[1];	//bits to swap

	x = ((b >> i) ^ (b >> j)) & ((1U << n) - 1); // XOR temporary
	r = b ^ ((x << i) | (x << j));

	
	//combine 2 arrays into hold
	hold = button_array[0] & MASK_BYTE;		//move CBASTART to hold and mask top 4 bits
	hold |= (r & MASK_BYTE) << 4;			//mask top 4 bits, shift rightdownleftup to top 4 bits, add to hold

	//actually change the bits
	check = copy_to_user((int*)arg, &hold, sizeof(hold));

	//check copy worked
	if (check != 0){
		return -EINVAL;
	}
	else{
		return 0;
	}

};


/*
 * tux_init
 *   DESCRIPTION: sets the LEDs based on arg
 *   INPUTS: tty -- the device
 * 			 arg -- 32 bit integer, 
 *					- low 16 bits = number whose hex value is to be displayed on the 7-segment displays
 *					- low 4 bits of 3rd byte = which LEDs are on
 *					- low 4 bits of 4th byte = which decimal points should be turned on
 *   OUTPUTS: none
 *   RETURN VALUE: 0
 *   SIDE EFFECTS: changes LED display according to arg 
 */
int set_LED(struct tty_struct* tty, unsigned long arg){

	unsigned char buf[6];		// bytes to be sent to TUX
	int LED_vals[4];			//LED values

	//contains what aef(dp)gcbd numbers are, dp is added in later (if needed)
	int seven_segment_conversion[16] = {0xE7, 0x06,  0xCB, 0x8F, 0x2E, 0xAD, 0xED, 0x86, 0xEF, 0xAE, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8};
	
	int dec;		//whether the decimal bit is on/off
	int i;			//index for for loops
	int arg1;		//for bit shifting around arg
	int led;		// which LEDs need to be on
	
	arg1 = arg;

	//get low 4 bits of highest byte, whether decimal points should be turned on
	dec = ((arg1 >> 24) & MASK_BYTE);		
	
	//get low 4 bits of 3rd byte, which LEDs should be on
	led = (MASK_BYTE & (arg1 >> 16));

	//seperate low 16 bits of arg out over LEDs
	for(i = 0; i < 4 ; i++)
	{
		LED_vals[i] = (MASK & arg1);
		arg1 = arg1 >> 4;			
	}

	//convert the hex number to 7 segemnt (using seven_segment_conversion) and add decimal if there is one
	for(i = 0; i < 4 ; i++){
		if((led & ONE)){		//if LSB of LED is on
			if(dec & ONE){		//if LSB of decimal is 'on'
				buf[i+2] = seven_segment_conversion[LED_vals[i]] + ADD_DEC;
			}
			else{				//if LSB of decimal is 'off'
				buf[i+2] = seven_segment_conversion[LED_vals[i]];
			}
		}
		else{					//if LED is off
			buf[i+2] = 0x00;
		}
		led = led >> 1; 		//go to next LED
		dec = dec >> 1; 		//go to next decimal
	}

	//write 1st 2 entires to buffer
	buf[1] = 0xF;				//1st byte is bitmask of LEDs to set, we always set all them, off LEDs will be set to 0x00
	buf[0] = MTCP_LED_SET;		//set LED opcode
	
	//write to TUX
	tuxctl_ldisc_put(tty, buf, i+2); 
	return 0;
};


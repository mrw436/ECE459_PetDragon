#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <string.h>
#include "mbed.h"
#include "basic_rf.h"
#include "bmac.h"

#define NODE2

// Will control servos and receive values from NODE1 (formerly Mole)

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

nrk_task_type MOVE_TASK;
NRK_STK move_task_stack[NRK_APP_STACKSIZE];
void move_task (void);

void nrk_create_taskset ();

char tx_buf[RF_MAX_PAYLOAD_SIZE];
char rx_buf[RF_MAX_PAYLOAD_SIZE];

void all_LEDs_set();
void all_LEDs_clr();

DigitalIn pr_covered(p8); //pin is high when photoresistor covered

int flag = 0;
int timeup = 0;
int molehit = 1;
int waittime = 0;
int starttime=0;
int hit =0;
int readyy =0; //  need to stop one hit from counting multiple times towards players score, once I turn off led wait for next coordinator message

float move_vals[40]; //i'm really just sticking arbitrary numbers here at this point, they don't make a difference anyway

void forward(float);
void backward(float);
void move(float, float);
void stop();
void test();

PwmOut servoL(p25);
PwmOut servoR(p26);

RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;

// Counter for number of detected packets that have valid data
static uint16_t rxContentSuccess1 = 0;
static uint16_t rxContentSuccess3 = 0;
// Counter for number of detected packets from the preamble perspective, regardless of contents
static uint16_t mrfRxISRcallbackCounter = 0;
// Counter for number of sent packets
static uint16_t sentPackets = 0;

static uint8_t clearToTx = 0;

// Count number of detected packets and toggle a pin for each packet
void all_LEDs_set(){
		nrk_led_set(RED_LED);
		nrk_led_set(ORANGE_LED);
		nrk_led_set(GREEN_LED);
		nrk_led_set(BLUE_LED);
}

void all_LEDs_clr(){
		nrk_led_clr(RED_LED);
		nrk_led_clr(ORANGE_LED);
		nrk_led_clr(GREEN_LED);
		nrk_led_clr(BLUE_LED);
}

void mrfIsrCallback()
{		
	mrfRxISRcallbackCounter++;
}

int main(void)
{	
//	nrk_setup_ports();
	
	//PwmOut servoL(p25);
	//PwmOut servoR(p26);	
//	
//	servoL.period(0.020);
//	servoR.period(0.020);
//	servoL.pulsewidth(0.001505 - 0.001); // servo position determined by a pulsewidth between 1-2ms
//	servoR.pulsewidth(0.001505 + 0.001);
//	
	printf("what\r\n");
	nrk_setup_ports();
	nrk_init();
	bmac_task_config();
	nrk_create_taskset();
  nrk_start();

	return 0;
}

//void test(){
//		int count = 0;
//		while (count < 10) {
//        for(float offset=0.0; offset<0.001; offset+=0.0001) {
//            servoL.pulsewidth(0.001505 + offset); // servo position determined by a pulsewidth between 1-2ms
//						servoR.pulsewidth(0.001505 - offset); // servo position determined by a pulsewidth between 1-2ms
//            wait(1);
//						//printf("pw: %f\r\n", (0.001 + offset));
//						count++;
//        }
//    }
//}

//void forward(float offset){
//	printf("forward offset: %f \r\n", offset);
//		servoL.period(0.020);
//		servoR.period(0.020);
//		servoL.pulsewidth(0.001505 - offset); // servo position determined by a pulsewidth between 1-2ms
//		servoR.pulsewidth(0.001505 + offset); 	
//}

//void backward(float offset){
//		printf("backward offset: %f \r\n", offset);
//		servoL.period(0.020);
//		servoR.period(0.020);
//		servoL.pulsewidth(0.001505 + offset); // servo position determined by a pulsewidth between 1-2ms
//		servoR.pulsewidth(0.001505 - offset); 
//}

////ok so joystick left goes right and right goes left but when I tried to fix it it got fucked up somehow so I know this code works except for that
//void move(float lr_offset, float ud_offset){
//	servoL.period(0.020);
//	servoR.period(0.020);
//	servoL.pulsewidth(0.001505 + 0.001 * ((0.5 - lr_offset) + (0.5 - ud_offset))); // servo position determined by a pulsewidth between 1-2ms
//	servoR.pulsewidth(0.001505 + 0.001 * ((0.5 - lr_offset) + (ud_offset - 0.5)));
//}

//void stop(){
//	servoL.period(0);
//	servoR.period(0);
//	//servoL.pulsewidth(0.001505);
//	//servoR.pulsewidth(0.001300);
//}

void rx_task ()
{
  uint8_t i, len, rssi;
  int8_t val;
	char *local_rx_buf;
  nrk_time_t check_period;
  //printf ("rx_task PID=%d\r\n", nrk_get_pid ());
	//PwmOut servoL(p25);
	//PwmOut servoR(p26);	

  // init bmac
  bmac_init (RADIO_CHANNEL);
	bmac_auto_ack_disable();
	#ifdef NODE1 // was node 1
	clearToTx = 1;
	#endif
	
	
  // Enable AES 128 bit encryption
  // When encryption is active, messages from plaintext
  // source will still be received. 
	
	// Commented out by MB
  //bmac_encryption_set_key(aes_key,16);
  //bmac_encryption_enable();
	// bmac_encryption_disable();


  // By default the RX check rate is 200ms
  // below shows how to change that
  check_period.secs=0;
  check_period.nano_secs=20*NANOS_PER_MS;
  val=bmac_set_rx_check_rate(check_period);

  // The default Clear Channel Assement RSSI threshold.
  // Setting this value higher means that you will only trigger
  // receive with a very strong signal.  Setting this lower means
  // bmac will try to receive fainter packets.  If the value is set
  // too high or too low performance will suffer greatly.
   bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 


  // This sets the next RX buffer.
  // This can be called at anytime before releaseing the packet
  // if you wish to do a zero-copy buffer switch
  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

	
  while (1)
	{
    //nrk_led_toggle (RED_LED);
    // Wait until an RX packet is received
    val = bmac_wait_until_rx_pkt ();
		// Get the RX packet 
		local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
		//if( bmac_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
		if(len != 0){
			printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
			for (i = 0; i < len; i++)
				printf ("%c", rx_buf[i]);
			printf ("]\r\n");
			
			char *token = NULL;
			int index = 0;
			
			//if strok doesn't work I think you can use strtof instead in a different way and it'll still work
			
			//this is kind of unsafe because if any of the formats are wrong it'll error but just be aware of that
			for (token = strtok(rx_buf, ","); token != NULL; token = strtok(NULL, ","))
			{
				char *unconverted;
				move_vals[index] = strtof(token, NULL);//&unconverted);
				printf("%f \r\n", move_vals[index]);
				//if (!isspace(*unconverted) && *unconverted != 0)
				if (*unconverted != 0)					
				{
					/**
					 * Input string contains a character that's not valid
					 * in a floating point constant
					 */
				}
				index += 1;
			}
			index = 0;
		}
		float lr_offset = move_vals[0];
		float ud_offset = move_vals[1];
		printf("mv %f %f\r\n", lr_offset, ud_offset);

//		servoL.period(0.020);
//		servoR.period(0.020);
//		servoL.pulsewidth(0.001505 + 0.001 * ((0.5 - lr_offset) + (0.5 - ud_offset))); // servo position determined by a pulsewidth between 1-2ms
//		servoR.pulsewidth(0.001505 + 0.001 * ((0.5 - lr_offset) + (ud_offset - 0.5)));
		//printf("hi\r\n");
		
		rxContentSuccess1++;
		
		//move(move_vals[0], move_vals[1]);
		
		// Release the RX buffer so future packets can arrive 
		bmac_rx_pkt_release();
		
		// this is necessary
    nrk_wait_until_next_period ();

  }
}

uint8_t ctr_cnt[4];

void move_task(){
	printf("hi");
	nrk_signal_register (1);
	nrk_wait_until_next_period ();
}

void tx_task ()
{
  uint8_t j, i, val, len, cnt;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;
	
	// printf("tx_task PID=%d\r\n", nrk_get_pid ());
	
  // Wait until the rx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  while (!bmac_started ())
    nrk_wait_until_next_period ();

	nrk_time_t check_period;
	check_period.secs=0;
  check_period.nano_secs=20*NANOS_PER_MS;
  val=bmac_set_rx_check_rate(check_period);

  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=bmac_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));


  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
  cnt = 0;
	
	int packetsToTx = 100; // was 1000

	#ifdef NODE1 // was node 1
  while (packetsToTx != 0)
	#endif
	#ifdef NODE2 // was node 2
  while (1)
	#endif
	{
		//nrk_led_toggle(BLUE_LED);
    // Build a TX packet
    //sprintf (tx_buf, "This is a test %d", cnt);
    //nrk_led_set (BLUE_LED);
		//printf("clearToTx %d\n",clearToTx);
		
		// want clearToTx marked yes only once time is up, 
		if (clearToTx == 1)
		{
			if (molehit == 1)
			{
				printf("Mole hit");
				sprintf (tx_buf, "yes");
				molehit = 0;
				timeup=1;
			} else if (molehit == 0)
			{
				printf("Mole missed");
				sprintf (tx_buf, "no");
				timeup=1;
			}
			
			// want to ignore molehits when timeup=1
			
			char *token = NULL;
			int index = 0;
			
			//if strok doesn't work I think you can use strtof instead in a different way and it'll still work
			
			//this is kind of unsafe because if any of the formats are wrong it'll error but just be aware of that
			for (token = strtok(rx_buf, ","); token != NULL; token = strtok(NULL, ","))
			{
				char *unconverted;
				move_vals[index] = strtof(token, NULL);//&unconverted);
				printf("%f \r\n", move_vals[index]);
				//if (!isspace(*unconverted) && *unconverted != 0)
				if (*unconverted != 0)					
				{
					/**
					 * Input string contains a character that's not valid
					 * in a floating point constant
					 */
				}
				index += 1;
			}
			index = 0;
			
			bmac_addr_decode_dest_mac(0x000A);
			
			// Auto ACK is an energy efficient link layer ACK on packets
			// If Auto ACK is enabled, then bmac_tx_pkt() will return failure
			// if no ACK was received. In a broadcast domain, the ACK's will
			// typically collide.  To avoid this, one can use address decoding. 
			// The functions are as follows:
			// bmac_auto_ack_enable();
			//	 bmac_auto_ack_disable();

			// Address decoding is a way of preventing the radio from receiving
			// packets that are not address to a particular node.  This will 
			// supress ACK packets from nodes that should not automatically ACK.
			// The functions are as follows:
			// bmac_addr_decode_set_my_mac(uint16_t MAC_ADDR); 
			  // 0xFFFF is broadcast
			// bmac_addr_decode_enable();
			// bmac_addr_decode_disable();
	/*
			 ctr_cnt[0]=cnt; 
			 if(ctr_cnt[0]==255) ctr_cnt[1]++; 
			 if(ctr_cnt[1]==255) ctr_cnt[2]++; 
			 if(ctr_cnt[2]==255) ctr_cnt[3]++; 
			 // You need to increase the ctr on each packet to make the 
			 // stream cipher not repeat.
			 bmac_encryption_set_ctr_counter(&ctr_cnt,4);

	*/  // For blocking transmits, use the following function call.
			// For this there is no need to register

			 val=bmac_tx_pkt(tx_buf, strlen(tx_buf));
		
			#ifndef NODE1 //only if NOT coordinator
			clearToTx = 0;
			#endif
		
			 if(val==NRK_OK) cnt++;
			 else ;//printf("NO ack or Reserve Violated! \r\n");

			// This function shows how to transmit packets in a
			// non-blocking manner  
			// val = bmac_tx_pkt_nonblocking(tx_buf, strlen (tx_buf));
			// printf ("Tx packet enqueued\r\n");
			// This functions waits on the tx_done_signal
			//ret = nrk_event_wait (SIG(tx_done_signal));

			// Just check to be sure signal is okay
			//if(ret & SIG(tx_done_signal) == 0 ) 
			//printf ("TX done signal error\r\n");
		 
			// If you want to see your remaining reservation
			// printf( "reserve=%d ",bmac_tx_reserve_get() );
			
			// Task gets control again after TX complete
			//printf("Tx task sent data!\r\n");
			//nrk_led_clr (BLUE_LED);
			//printf("tx_task PID=%d\r\n", nrk_get_pid ());
			
			packetsToTx--;
			sentPackets++;
		}
    nrk_wait_until_next_period (); // was not commented
  }
}

void nrk_create_taskset ()
{
	// Activate both tasks on both nodes
	// Choose node by macro defined on top of this file
  // Tx task runs mode often on NODE2 to make sure all received packets are returned promptly, witout blocking the receiving buffer

  RX_TASK.task = rx_task;
  nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 0;
  RX_TASK.period.nano_secs = 50*NANOS_PER_MS;
  RX_TASK.cpu_reserve.secs = 0;
  RX_TASK.cpu_reserve.nano_secs = 0;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk( &TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 0;
	#ifdef NODE1
  TX_TASK.period.nano_secs = 200*NANOS_PER_MS;
  #endif
	#ifdef NODE2
  TX_TASK.period.nano_secs = 150*NANOS_PER_MS;
  #endif
	TX_TASK.cpu_reserve.secs = 0;
  TX_TASK.cpu_reserve.nano_secs = 0;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0; 
	nrk_activate_task (&TX_TASK);
	
	MOVE_TASK.task = move_task;
  nrk_task_set_stk( &MOVE_TASK, move_task_stack, NRK_APP_STACKSIZE);
  MOVE_TASK.prio = 2;
  MOVE_TASK.FirstActivation = TRUE;
  MOVE_TASK.Type = BASIC_TASK;
  MOVE_TASK.SchType = PREEMPTIVE;
  MOVE_TASK.period.secs = 0;
  MOVE_TASK.period.nano_secs = 50*NANOS_PER_MS;
  MOVE_TASK.cpu_reserve.secs = 0;
  MOVE_TASK.cpu_reserve.nano_secs = 0;
  MOVE_TASK.offset.secs = 0;
  MOVE_TASK.offset.nano_secs = 0;
  //nrk_activate_task (&MOVE_TASK);
}
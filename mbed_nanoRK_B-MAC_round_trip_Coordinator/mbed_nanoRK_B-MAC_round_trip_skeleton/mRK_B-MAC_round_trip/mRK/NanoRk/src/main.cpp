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

#define NODE1

// Will have joystick and send values to servo controller (formerly coordinator)

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();

char tx_buf[RF_MAX_PAYLOAD_SIZE];
char rx_buf[RF_MAX_PAYLOAD_SIZE];

//mine
int playerscore = 0;

void forward(float);
void backward(float);
void move(float, float);
void stop();
void test();

PwmOut servoL(p25);
PwmOut servoR(p26);

AnalogIn ud_pin(p15);
AnalogIn lr_pin(p16);

float lr_pinval;
float ud_pinval;

char str_to_send[50]; //12 is for 6 each for the floats, 1 for comma, 1 for NULL at end
char lr_val[20]; //these were 6 now we're making them overkill
char ud_val[20];

RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;

// Counter for number of detected packets that have valid data
static uint16_t playersuccess = 0;
static uint16_t playerfail = 0;
// Counter for number of detected packets from the preamble perspective, regardless of contents
static uint16_t mrfRxISRcallbackCounter = 0;
// Counter for number of sent packets
static uint16_t sentPackets = 0;

static uint8_t clearToTx = 0;

int selected_node = 0;
char waittime[] = "5";

// Count number of detected packets and toggle a pin for each packet
void mrfIsrCallback()
{		
	mrfRxISRcallbackCounter++;
}

int main(void)
{	
	
	//READ THIS: eventually use interrupts to move when change in pin is detected?
	//use button pressed in conjunction with moving joystick to avoid voltage jumps that sometimes occur when idle?
	///*
	nrk_setup_ports();
	
	nrk_init();
	bmac_task_config();
	nrk_create_taskset();
  nrk_start();
	//*/
	
	// Servo Testing
//	servoL.period(0.020);          // servo requires a 20ms period
//	servoR.period(0.020);          // servo requires a 20ms period
//	//threshold for stopping is approximately 0.001505
//	servoL.pulsewidth(0.001305);
//	servoR.pulsewidth(0.001305);
//	wait(1);
//  test();
//	stop();
	
	//Joystick/ADC Testing
	///*
	while(1){
		printf("LR %f \r\n", lr_pin.read());
		printf("UD %f \r\n", ud_pin.read());
		
		bool lr_rest = (47. < lr_pin.read()*100.0) && (lr_pin.read()*100.0 < 52.);
		bool ud_rest = (47. < ud_pin.read()*100.0) && (ud_pin.read()*100.0 < 52.);
		
		if(lr_rest && ud_rest){
			printf("stop\r\n");
			stop();
		}
//		} else if(ud_pin.read()*100.0 > 52.){
//				printf("forward\r\n");
//				forward(ud_pin.read()*0.001);
//		} else if(ud_pin.read()*100.0 < 47.){
//				printf("backward\r\n");
//				backward(0.001*(1 - ud_pin.read()));
//		}
			else {
				move(lr_pin.read(), ud_pin.read());
			}
			wait(0.2);
		
//		printf("(LR) percentage: %3.3f%%, normalized: 0x%04X\r\n", lr_pin.read()*100.0, lr_pin.read_u16());
//		printf("(UD) percentage: %3.3f%%, normalized: 0x%04X\r\n", ud_pin.read()*100.0, ud_pin.read_u16());
//		wait(1);
	}
	//*/
	return 0;
}

void test(){
		int count = 0;
		while (count < 10) {
        for(float offset=0.0; offset<0.001; offset+=0.0001) {
            servoL.pulsewidth(0.001505 + offset); // servo position determined by a pulsewidth between 1-2ms
						servoR.pulsewidth(0.001505 - offset); // servo position determined by a pulsewidth between 1-2ms
            wait(1);
						//printf("pw: %f\r\n", (0.001 + offset));
						count++;
        }
    }
}

void forward(float offset){
	printf("forward offset: %f \r\n", offset);
		servoL.period(0.020);
		servoR.period(0.020);
		servoL.pulsewidth(0.001505 - offset); // servo position determined by a pulsewidth between 1-2ms
		servoR.pulsewidth(0.001505 + offset); 	
}

void backward(float offset){
		printf("backward offset: %f \r\n", offset);
		servoL.period(0.020);
		servoR.period(0.020);
		servoL.pulsewidth(0.001505 + offset); // servo position determined by a pulsewidth between 1-2ms
		servoR.pulsewidth(0.001505 - offset); 
}

//ok so joystick left goes right and right goes left but when I tried to fix it it got fucked up somehow so I know this code works except for that
void move(float lr_offset, float ud_offset){
	servoL.period(0.020);
	servoR.period(0.020);
	servoL.pulsewidth(0.001505 + 0.001 * ((0.5 - lr_offset) + (0.5 - ud_offset))); // servo position determined by a pulsewidth between 1-2ms
	servoR.pulsewidth(0.001505 + 0.001 * ((0.5 - lr_offset) + (ud_offset - 0.5)));
}

void stop(){
	servoL.period(0);
	servoR.period(0);
	//servoL.pulsewidth(0.001505);
	//servoR.pulsewidth(0.001300);
}

void rx_task ()
{
  uint8_t i, len, rssi;
  int8_t val;
	char *local_rx_buf;
  nrk_time_t check_period;
  //printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  // init bmac
  bmac_init (RADIO_CHANNEL);
	bmac_auto_ack_disable();
	#ifdef NODE1
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
		//printf("got packet? %s\r\n", rx_buf);
		
		//for some reason this needs to happen in rx task in order for it to work!! no clue why
		lr_pinval = lr_pin.read();
		ud_pinval = ud_pin.read();
		printf("%f, %f\r\n", lr_pinval, ud_pinval);
		sprintf(str_to_send, "%f,%f", lr_pinval, ud_pinval);
		printf("%s\r\n", str_to_send);

		// Get the RX packet 
		local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
		//if( bmac_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
		//printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
		//for (i = 0; i < len; i++)
		//  printf ("%c", rx_buf[i]);
		//printf ("]\r\n");
		//nrk_led_clr (ORANGE_LED);
	 
		// Change something to make sure the buffer isn't the same if no buffer fill from Rx occurs
		rx_buf[1] = '0';
		
		// Release the RX buffer so future packets can arrive 
		bmac_rx_pkt_release ();
		
		// this is necessary
    nrk_wait_until_next_period ();

  }
}

uint8_t ctr_cnt[4];

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
	printf("tx1: %f,%f\r\n", lr_pinval, ud_pinval);

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
	
	int packetsToTx = 100; // total of 10 moles pop up
	
	//lr_pinval = lr_pin.read();
	//ud_pinval = ud_pin.read();

	#ifdef NODE1
  while (packetsToTx != 0)
	#endif

  //while (1)
	{
		//nrk_led_toggle(BLUE_LED);
    // Build a TX packet
    //sprintf (tx_buf, "This is a test %d", cnt);
    //nrk_led_set (BLUE_LED);
		
		if (clearToTx == 1)
		{
			//str_to_send[] = "Geeks-for-Geeks";
			printf("tx2: %f,%f\r\n", lr_pinval, ud_pinval);
			//sprintf(str_to_send, "%f,%f", lr_pinval, ud_pinval);
			printf("tx3: %s\r\n", str_to_send);
//			strcat(str_to_send, lr_val);
//			strcat(str_to_send, ",");
//			strcat(str_to_send, ud_val);
			sprintf (tx_buf, "%s", str_to_send); //can I ignore this warning pls
			
			//selected_node = rand() % 3 + 1;
			selected_node = 1; //for testing
			//printf("Coordinator sending 5\r\n");
			if (selected_node==1) 
			{
				printf("Coordinator sending %s to Node 2\r\n", tx_buf);
				bmac_addr_decode_dest_mac(0x000B); // send to node 2
			}
			else if (selected_node==2)
			{
				bmac_addr_decode_dest_mac(0x000C); // send to node 3
			}
			else if (selected_node==3)
			{
				bmac_addr_decode_dest_mac(0x000D); // send to node 4
			}
			
			
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
		
			#ifdef NODE2
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
    nrk_wait_until_next_period ();
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
  TX_TASK.period.nano_secs = 1000*NANOS_PER_MS; // **make this 10 seconds
  //TX_TASK.period.secs = 3;
	#endif
	#ifdef NODE2
  TX_TASK.period.nano_secs = 100*NANOS_PER_MS;
  #endif
	TX_TASK.cpu_reserve.secs = 0;
  TX_TASK.cpu_reserve.nano_secs = 0;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
	nrk_activate_task (&TX_TASK);
}
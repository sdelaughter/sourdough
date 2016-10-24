#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;


int the_window_size = 1;

//Set parameters
float kp = -0.025;
float ki = -0.00006;
float kd = -0.02;
int rtt_goal = 90;
int error_reset_threshold = 20;

int w = 1;
int rtt_error = 0;
int rtt_error_old = 0;
int rtt_error_sum = 0;
int d_error = 0;

/*Best so far
	
float kp = -0.025;
float ki = -0.00006;
float kd = -0.02;
int rtt_goal = 80;
int error_reset_threshold = 10;

Average capacity: 5.04 Mbits/s
Average throughput: 3.35 Mbits/s (66.5% utilization)
95th percentile per-packet queueing delay: 58 ms
95th percentile signal delay: 127 ms
Power: 26.38
http://cs344g.keithw.org/report?samd-1477198917-shahneey
*/

//int last_rtt = 0;
//int rtt_threshold = 5;
//int increase_amount = 1;
//float decrease_ratio = 0.75;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  
  int rtt = timestamp_ack_received - send_timestamp_acked;
  rtt_error = rtt - rtt_goal;
  if (rtt_error <= error_reset_threshold){ //&& rtt_error >= (-1 * error_reset_threshold)){
	  rtt_error_sum = 0;
  } else{
	  rtt_error_sum += rtt_error;
  }
  d_error = rtt_error - rtt_error_old;
  rtt_error_old = rtt_error;
  
  w = (kp * rtt_error) + (ki * rtt_error_sum) + (kd * d_error);

  if (rtt_error <= (-1 * error_reset_threshold)){
	  the_window_size += 1;
  }  else{
	  if (the_window_size + w < 0){
		  the_window_size = 0;
	  } else{
	      the_window_size += w;
	  }    
  }
    
  /*
  if (last_rtt == 0 ){
	  last_rtt = rtt;
  }
  
  if (rtt - last_rtt > rtt_threshold){
	  the_window_size *= decrease_ratio;
  } else{
	  the_window_size += increase_amount;
  }
  
  last_rtt = rtt;
  */
  
  cerr << "\n\n        Time: " << timestamp_ack_received
       << "\n      Window: " << the_window_size
       << "\n           w: " << w
       << "\n         RTT: " << rtt
       << "\n   RTT Error: " << rtt_error
       << "\n   Error Sum: " << rtt_error_sum
       << "\nError Change: " << d_error;
  
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* A packet was lost */
void Controller::packet_lost( void ){
	//the_window_size *= 0.5;
}


/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 100; /* timeout of one second */
}

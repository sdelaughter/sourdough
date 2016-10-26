#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

//Authors: Sam DeLaughter and Brandon Karpf
//Last Updated: 10/25/16
//MIT 6.829 - Fall 2016 - PSet 2

/*This controller uses PID control to attempt to maintain a stable RTT with
	high throughput when sending traffic over a cellular network with an
	unpredictable and highly variable link capacity.
*/

//Initialize a window of 1
int the_window_size = 1;

//Set parameters
float kp = -0.065;             //The proportional coefficient
float ki = -0.00003;           //The integral coefficient
float kd = -0.007;             //The derrivative coefficient
float min_rtt = 62;
float max_rtt = 105;
float error_sum_bound = 600;
float error_threshold_nudge = 0.395;
float error_threshold_nudge2 = 0.0;//0.65;

//Initialize variables (don't change these values)
float w = 1;
float error = 0;
float error_old = 0;
float error_sum = 0;
float d_error = 0;
int rtt_old = 0;
int n = 0;
float rtt_sum = 0;
float rtt_avg = 0;


/*Best results achieved so far:
	
Average capacity: 5.04 Mbits/s
Average throughput: 3.23 Mbits/s (64.0% utilization)
95th percentile per-packet queueing delay: 54 ms
95th percentile signal delay: 94 ms
Power: 34.36 MBits/s^2 (Average Throughput / Signal Delay)
http://cs344g.keithw.org/report/?brandonandsam-1477437081-eishielu

//Parameters Used
float kp = -0.065;             //The proportional coefficient
float ki = -0.00003;           //The integral coefficient
float kd = -0.007;             //The derrivative coefficient
float min_rtt = 62;
float max_rtt = 105;
float error_sum_bound = 600;
float error_threshold_nudge = 0.395;
float error_threshold_nudge2 = 0.0;//0.65;
*/


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
  //Measure the RTT for the ACKed packet
  int rtt = timestamp_ack_received - send_timestamp_acked;
  //If this is the first ACK received, initialize rtt_old to the current rtt
  if (rtt_old == 0){
	  rtt_old = rtt;
  }
  /*Set the error value based on the RTT's difference from the last RTT
  	and its distance from max_rtt and min_rtt*/
  int error = 0;
  if (rtt > max_rtt){
	  /*If the RTT is above a certain threshold, set the error to
	  	the difference between the RTT and that threshold, then decrease
	  	the error slightly to nudge it back within the threshold faster*/
	  error = (rtt - max_rtt) + ((rtt - max_rtt) * error_threshold_nudge2);
  } else if (rtt < min_rtt){
	  /*If the RTT is below a certain threshold, set the error to
	  	the difference between the RTT and that threshold, then increase
	  	the error slightly to nudge it back withint the threshold faster*/
	  error = (rtt - min_rtt) + ((rtt - min_rtt) * error_threshold_nudge);
  }else{
	  /*Otherwise, set the error to the diffence between the current RTT
	  	and the RTT from the previous ACK*/
      error = rtt - rtt_old;
  }
  
  //add the current error to the error sum
  error_sum += error;
  
  //Bound the error sum within a certain range
  if (error_sum > error_sum_bound){
	  error_sum = error_sum_bound;
  } else if (error_sum < (-1 * error_sum_bound)){
	  error_sum = (-1 * error_sum_bound);
  }

  //Calculate the change in error from the last ACK
  d_error = error - error_old;
  //Store the current error to calculate d_error on the next ACK
  error_old = error;
  
  //Calculate the change to the window size with a PID controller
  if (error > 0){
       w = ((1.0*kp) * error) + ((1.0*ki) * error_sum) + ((1.0*kd) * d_error); 
  } else if (error < 0){
       w = ((0.9*kp) * error) + ((1.1*ki) * error_sum) + ((0.6*kd) * d_error);
  }
  else{
       w = ((1.0*kp) * error) + ((1.0*ki) * error_sum) + ((1.0*kd) * d_error);
  }

  if (w < (-0.5 * the_window_size)){
      w = (-0.5 * the_window_size);
  }
  
  if (the_window_size + w < 1){
	  //Make sure the window size never drops below 1
	  the_window_size = 1;
  } else{
	  //Update the window size
      the_window_size += w;
  }
  
  //Store the current RTT to reference in the next ACK's calculations
  rtt_old = rtt;

   //Uncomment this block to print timestamp, RTT, and window size for each ACK
/*	  
  cerr << "\n\n        Time: " << timestamp_ack_received
       << "\n      Window: " << the_window_size
       << "\n         RTT: " << rtt
       << "\n      RTTavg: " << rtt_avg
       << "\n       w_min: " << w_min
       << "\n       w_max: " << w_max;
*/  
  
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 95; /* timeout of 95ms */
}

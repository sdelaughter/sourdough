#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;


int the_window_size = 1;
int increase_amount = 1;
float decrease_ratio = 0.5;
int rtt_threshold = 10;
uint64_t send_threshold = 1;
uint64_t recv_threshold = 6;
uint64_t last_send_timestamp_acked = 150000;
uint64_t last_recv_timestamp_acked = 150000;
uint64_t last_timestamp_ack_received = 150000;


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
  
  //uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
  if ( send_timestamp_acked - last_send_timestamp_acked < send_threshold){
	  if ( timestamp_ack_received - last_timestamp_ack_received < recv_threshold){
		  the_window_size += increase_amount;
	  } else{
		  the_window_size *= decrease_ratio;
	  }
  }
  else{
	  the_window_size += increase_amount;
  }
  
  cerr << "\n" << timestamp_ack_received << ": " << the_window_size;
  
  /*
  cerr << "\n\n           the_window_size: " << the_window_size;
  cerr << "\n   last_send_timestamp_acked: " << last_send_timestamp_acked;
  cerr << "\n        send_timestamp_acked: " << send_timestamp_acked;
  cerr << "\n   last_recv_timestamp_acked: " << last_recv_timestamp_acked;
  cerr << "\n        recv_timestamp_acked: " << recv_timestamp_acked;
  cerr << "\n last_timestamp_ack_received: " << send_timestamp_acked;
  cerr << "\n      timestamp_ack_received: " << send_timestamp_acked;
  */
  
  
  last_send_timestamp_acked = send_timestamp_acked;
  last_recv_timestamp_acked = recv_timestamp_acked;
  last_timestamp_ack_received = timestamp_ack_received;

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

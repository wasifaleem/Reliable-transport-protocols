#include "../include/simulator.h"
#include <iostream>
#include <cstring>
#include <iomanip>
#include <cmath>
/* ******************************************************************
 ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for PA2, unidirectional data transfer 
   protocols (from A to B). Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

/********* STUDENTS WRITE THE NEXT SEVEN ROUTINES *********/
#define DEBUG_LOG(AorB, str) do { std::cout << std::setprecision(5) << std::setw(8) << get_sim_time() << "T: " << std::setw(3) << __LINE__  << "L: " << AorB << " : " << str << std::endl; } while( false )
#define DEBUG_A(str) DEBUG_LOG('A', str)
#define DEBUG_B(str) DEBUG_LOG('B', str)

std::ostream &operator<<(std::ostream &, const pkt &);

std::ostream &operator<<(std::ostream &, const msg &);

static float initial_rtt = 10.0f,
        alpha = 0.125f,
        beta = 0.25f,
        SampleRTT = initial_rtt,
        EstimatedRTT = initial_rtt,
        DevRTT = 0.0f;

float TimeoutInterval();

// A
static volatile int A_seq = 0;

static pkt *pkt_in_transit = NULL;
static float sent_time = 0.0f;;
static bool retransmitted = false;

// B
static volatile int B_seq = 0;

pkt *make_pkt(int seq, int ack, struct msg *msg);

pkt *make_ack(int ack);

int make_checksum(struct pkt *pkt);

bool is_corrupt(struct pkt pkt);

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message) {
    if (pkt_in_transit == NULL) {
        struct pkt *pkt = make_pkt(A_seq, 0, &message);
        tolayer3(0, *pkt);

        sent_time = get_sim_time();
        retransmitted = false;

        starttimer(0, TimeoutInterval());
        pkt_in_transit = pkt;
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet) {
    if (!is_corrupt(packet) && packet.acknum == A_seq) {
        stoptimer(0);

        if (!retransmitted) {
            SampleRTT = get_sim_time() - sent_time;
            EstimatedRTT = ((1 - alpha) * EstimatedRTT + (alpha * SampleRTT));
            DevRTT = ((1 - beta) * DevRTT + (beta * fabsf(SampleRTT - EstimatedRTT)));
        }

        //  received ack
        if (pkt_in_transit != NULL) {
            delete pkt_in_transit;
            pkt_in_transit = NULL;
        }
        // toggle seq
        A_seq ^= 1;
    }
}

/* called when A's timer goes off */
void A_timerinterrupt() {
    if (pkt_in_transit != NULL) {
        struct pkt packet = (*pkt_in_transit);
        tolayer3(0, packet);
        retransmitted = true;
        DEBUG_A("\033[31;1m" << "TIMEOUT RESENT: " << packet << "\033[0m");
        starttimer(0, TimeoutInterval() * 2);
    }
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init() {
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet) {
    bool corrupt = is_corrupt(packet);
    if (!corrupt && packet.seqnum == B_seq) {
        pkt *ack_pkt = make_ack(B_seq);
        tolayer3(1, *ack_pkt);
        tolayer5(1, packet.payload);
        // toggle seq
        B_seq ^= 1;
        delete ack_pkt;
    } else if (corrupt || packet.seqnum == (B_seq ^ 1)) {
        pkt *ack_pkt = make_ack(B_seq ^ 1);
        tolayer3(1, *ack_pkt);
        delete ack_pkt;
    }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init() {

}


float TimeoutInterval() {
    float TimeoutInterval = EstimatedRTT + 4 * DevRTT;
    DEBUG_A("Estimated TimeoutInterval: " << TimeoutInterval);
    return TimeoutInterval;
}

pkt *make_pkt(int seq, int ack, struct msg *msg) {
    struct pkt *pkt = new struct pkt();
    pkt->seqnum = seq;
    pkt->acknum = ack;
    if (msg != NULL) {
        memcpy(pkt->payload, (*msg).data, 20);
    }
    pkt->checksum = make_checksum(pkt);
    return pkt;
}

int make_checksum(struct pkt *pkt) {
    int checksum = 0;
    checksum += pkt->seqnum;
    checksum += pkt->acknum;
    for (int i = 0; i < 20; ++i) {
        checksum += pkt->payload[i];
    }
    return checksum;
}


bool is_corrupt(struct pkt pkt) {
    return make_checksum(&pkt) != pkt.checksum;
}

pkt *make_ack(int ack) {
    return make_pkt(0, ack, NULL);
}

std::ostream &operator<<(std::ostream &os, const msg &m) {
    return os << "{msg: " << std::string(m.data, 20) << '}';
}

std::ostream &operator<<(std::ostream &os, const pkt &p) {
    os << "{seq: " << p.seqnum << ", ack:" << p.acknum << ", chks:" << p.checksum;
    if (p.payload[0] != '\0') {
        os << ", payload:" << std::string(p.payload, 20);
    }
    os << '}';
    return os;
}
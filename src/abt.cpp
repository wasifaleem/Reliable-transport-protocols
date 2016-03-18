#include "../include/simulator.h"
#include <iostream>
#include <cstring>
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

static const float timeout = 5.0;

// A
static volatile int A_seq = 0;
static pkt *pkt_in_transit = NULL;

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
        starttimer(0, timeout);
        pkt_in_transit = pkt;
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet) {
    if (!is_corrupt(packet) && packet.acknum == A_seq) {
        stoptimer(0);
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
        tolayer3(0, *pkt_in_transit);
        starttimer(0, timeout);
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

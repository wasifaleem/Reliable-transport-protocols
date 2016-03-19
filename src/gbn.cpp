#include "../include/simulator.h"
#include <iostream>
#include <cstring>
#include <vector>
#include <queue>
#include <iomanip>

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

#define DEBUG_LOG(AorB, str) do { std::cout << std::setprecision(5) << std::setw(8) << get_sim_time() << " : " << AorB << " : " << str << std::endl; } while( false )
#define DEBUG_A(str) DEBUG_LOG('A', str)
#define DEBUG_B(str) DEBUG_LOG('B', str)

std::ostream &operator<<(std::ostream &, const pkt &);

std::ostream &operator<<(std::ostream &, const msg &);

static const float timeout = 12.0; // TODO: RTT ?

// A
static std::vector<pkt *> sndpkt(1000);
static std::queue<msg> buffer;
static int base = 1, nextseqnum = 1, N;

void trySendBuffered();

// B
static volatile int expectedseqnum = 1;

pkt *make_pkt(int seq, int ack, struct msg *msg);

pkt *make_ack(int ack);

int make_checksum(struct pkt *pkt);

bool is_corrupt(struct pkt pkt);

float expected_timeout();


/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message) {
    if (nextseqnum < base + N) {
        sndpkt[nextseqnum] = make_pkt(nextseqnum, 0, &message);
        tolayer3(0, *sndpkt[nextseqnum]);
        if (base == nextseqnum) {
            starttimer(0, expected_timeout());
        }
        nextseqnum++;
        DEBUG_A("Sent: " << message);
    } else {
        buffer.push(message);
        DEBUG_A("Buffered: " << message);
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet) {
    if (!is_corrupt(packet)) {
        if (packet.acknum == base) {
            DEBUG_A("Receive ACK: " << packet);
            base = packet.acknum + 1; // TODO: free ack'd pkt, handle duplicate
            if (base == nextseqnum) {
                stoptimer(0);
                trySendBuffered();
            }
        } else {
            DEBUG_A("Receive DUP-ACK: " << packet);
        }
    } else {
        DEBUG_A("Receive CORRUPT ACK: " << packet);
    }
}

void trySendBuffered() {
    while (!buffer.empty() && nextseqnum < base + N) {
        struct msg message = buffer.front();
        sndpkt[nextseqnum] = make_pkt(nextseqnum, 0, &message);
        tolayer3(0, *sndpkt[nextseqnum]);
        if (base == nextseqnum) {
            starttimer(0, expected_timeout());
        }
        nextseqnum++;
        buffer.pop();
        DEBUG_A("Sent buffered: " << message);
    }
}

/* called when A's timer goes off */
void A_timerinterrupt() {
    DEBUG_A("Start Timer interrupt");
    starttimer(0, expected_timeout());
    for (int i = base; (i < nextseqnum && sndpkt[i] != NULL); ++i) {
        DEBUG_A("Re-Sending: " << (*sndpkt[i]));
        tolayer3(0, *sndpkt[i]);
    }
    DEBUG_A("End Timer interrupt");
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init() {
    N = getwinsize();
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet) {
    if (!is_corrupt(packet) && packet.seqnum == expectedseqnum) {
        DEBUG_B("Received: " << packet);
        tolayer5(1, packet.payload);

        struct pkt *ack = make_ack(expectedseqnum);
        DEBUG_B("Sending ACK: " << *ack);
        tolayer3(1, *ack);
        delete ack;

        expectedseqnum++;
    } else {
        DEBUG_B("Re-sending ACK: " << expectedseqnum - 1);
        struct pkt *ack = make_ack(expectedseqnum - 1);
        tolayer3(1, *ack);
        delete ack;
    }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init() {
}

float expected_timeout() {
    int n = (nextseqnum - base);
    if (n < 1) {
        n = 1;
    }
    float t = timeout * n;
    DEBUG_A("Expected timeout: " << t);
    return t;
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
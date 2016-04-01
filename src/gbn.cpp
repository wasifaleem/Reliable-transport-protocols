#include "../include/simulator.h"
#include <iostream>
#include <cstring>
#include <vector>
#include <queue>
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

pkt make_pkt(int seq, int ack, const struct msg *msg);

pkt make_ack(int ack);

int make_checksum(const struct pkt &pkt);

bool is_corrupt(const struct pkt &pkt);

static float initial_rtt = 10.0f,
        alpha = 0.125f,
        beta = 0.25f,
        SampleRTT = initial_rtt,
        EstimatedRTT = initial_rtt,
        DevRTT = 0.0f;

float TimeoutInterval();

// A
struct buffer {
    struct pkt pkt;
    bool retransmitted;
    float sent_time;
};
static std::vector<buffer> sndpkt(1100);
static std::queue<msg> buffer;
static int base = 1, nextseqnum = 1, N;

void send_buffered();

// B
static volatile int expectedseqnum = 1;

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message) {
    if (nextseqnum < base + N) {
        struct buffer b = {make_pkt(nextseqnum, 0, &message), false, get_sim_time()};
        sndpkt[nextseqnum] = b;
        tolayer3(0, sndpkt[nextseqnum].pkt);
        if (base == nextseqnum) {
            starttimer(0, TimeoutInterval());
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
        if (packet.acknum < base) {
            DEBUG_A("\033[31;1m" << "Receive ACK: " << packet.acknum << " is less than BASE: " << base << " ignoring" <<
                    "\033[0m");
            return;
        }
        DEBUG_A("\033[1;1m" << "Receive ACK: " << sndpkt[packet.acknum].pkt << "\033[0m");

        if (!sndpkt[packet.acknum].retransmitted) {
            SampleRTT = get_sim_time() - sndpkt[packet.acknum].sent_time;
            EstimatedRTT = ((1 - alpha) * EstimatedRTT + (alpha * SampleRTT));
            DevRTT = ((1 - beta) * DevRTT + (beta * fabsf(SampleRTT - EstimatedRTT)));
        }

        base = packet.acknum + 1;
        if (base == nextseqnum) {
            stoptimer(0);
            send_buffered();
        }
    } else {
        DEBUG_A("Receive CORRUPT ACK: " << packet);
    }
}

void send_buffered() {
    while (!buffer.empty() && nextseqnum < base + N) {
        struct msg message = buffer.front();
        struct buffer b = {make_pkt(nextseqnum, 0, &message), false, get_sim_time()};
        sndpkt[nextseqnum] = b;
        tolayer3(0, sndpkt[nextseqnum].pkt);
        if (base == nextseqnum) {
            starttimer(0, TimeoutInterval());
        }
        nextseqnum++;
        buffer.pop();
        DEBUG_A("Sent buffered: " << message);
    }
}

/* called when A's timer goes off */
void A_timerinterrupt() {
    starttimer(0, TimeoutInterval() * 2);
    for (int i = base; (i < nextseqnum); ++i) {
        tolayer3(0, sndpkt[i].pkt);
        sndpkt[i].retransmitted = true;
        DEBUG_A("\033[31;1m" << "TIMEOUT RESENT: " << sndpkt[i].pkt << "\033[0m");
    }
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
        DEBUG_B("\033[32;1m" << "Received: " << packet << "\033[0m");
        tolayer5(1, packet.payload);

        struct pkt ack = make_ack(expectedseqnum);
        DEBUG_B("Sending ACK: " << ack);
        tolayer3(1, ack);

        expectedseqnum++;
    } else {
        DEBUG_B("Re-sending ACK: " << expectedseqnum - 1);
        struct pkt ack = make_ack(expectedseqnum - 1);
        tolayer3(1, ack);
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

pkt make_pkt(int seq, int ack, const struct msg *msg) {
    struct pkt pkt = {seq, ack};
    if (msg != NULL) {
        memcpy(pkt.payload, (*msg).data, 20);
    }
    pkt.checksum = make_checksum(pkt);
    return pkt;
}

int make_checksum(const struct pkt &pkt) {
    int checksum = 0;
    checksum += pkt.seqnum;
    checksum += pkt.acknum;
    for (int i = 0; i < 20; ++i) {
        checksum += pkt.payload[i];
    }
    return checksum;
}


bool is_corrupt(const struct pkt &pkt) {
    return make_checksum(pkt) != pkt.checksum;
}

pkt make_ack(int ack) {
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
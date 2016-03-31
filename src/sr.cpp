#include "../include/simulator.h"
#include <iostream>
#include <cstring>
#include <set>
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

/* Timer */
static float initial_rtt = 10.0f,
        alpha = 0.125f,
        beta = 0.25f,
        SampleRTT = initial_rtt,
        EstimatedRTT = initial_rtt,
        DevRTT = 0;

float TimeoutInterval();

static const float CLOCK_TICK = 0.1;

struct timer {
    int seq;
    float time;

    bool operator<(const timer &t) const {
        return time > t.time;
    }

    friend std::ostream &operator<<(std::ostream &os, const timer &t) {
        return os << "{seq: " << t.seq << ", time:" << t.time << '}';
    }
};

static std::priority_queue<timer> timers;
static std::set<int> cancelled_timers;

void start_timer(int seq, float time);

void cancel_timer(int seq);

/* Timer */

// A
struct A_buffer {
    struct pkt pkt;
    bool acked;
    bool retransmitted;
    float sent_time;
};
static std::vector<A_buffer> A_sndpkt(1100);
static std::queue<msg> A_buffer;
static int send_base = 1, nextseqnum = 1, N;

void send_buffered();

// B
struct B_buffer {
    struct pkt pkt;
    bool acked;
};
static std::vector<B_buffer> B_rcvpkt(1100);
static volatile int rcvbase = 1;


/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message) {
    if (nextseqnum < send_base + N) {
        struct A_buffer b = {make_pkt(nextseqnum, 0, &message), false, false, get_sim_time()};
        A_sndpkt[nextseqnum] = b;
        DEBUG_A("Sending: " << A_sndpkt[nextseqnum].pkt);
        tolayer3(0, A_sndpkt[nextseqnum].pkt);
        start_timer(nextseqnum, TimeoutInterval());
        nextseqnum++;
    } else {
        A_buffer.push(message);
        DEBUG_A("Buffered: " << message);
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet) {
    if (!is_corrupt(packet)) {
        if (!A_sndpkt[packet.acknum].acked) {
            cancel_timer(packet.acknum);
            A_sndpkt[packet.acknum].acked = true;
            DEBUG_A("\033[1;1m" << "Receive ACK: " << A_sndpkt[packet.acknum].pkt << "\033[0m");

            if (!A_sndpkt[packet.acknum].retransmitted) {
                SampleRTT = get_sim_time() - A_sndpkt[packet.acknum].sent_time;
                EstimatedRTT = ((1 - alpha) * EstimatedRTT + (alpha * SampleRTT));
                DevRTT = ((1 - beta) * DevRTT + (beta * fabsf(SampleRTT - EstimatedRTT)));
            }

            if (packet.acknum == send_base) {
                for (std::vector<std::pair<pkt, bool> >::size_type i = (unsigned long) (send_base);
                     i != A_sndpkt.size() && A_sndpkt[i].acked; i++) {
                    send_base++;
                    DEBUG_A("Advancing send_base to: " << send_base);
                }
                send_buffered();
            }
        } else {
            DEBUG_A("Receive DUP-ACK: " << packet);
        }
    } else {
        DEBUG_A("Receive CORRUPT ACK: " << packet);
    }
}

void send_buffered() {
    while (!A_buffer.empty() && nextseqnum < send_base + N) {
        struct msg message = A_buffer.front();
        struct A_buffer b = {make_pkt(nextseqnum, 0, &message), false, false, get_sim_time()};
        A_sndpkt[nextseqnum] = b;
        DEBUG_A("Sending buffered: " << A_sndpkt[nextseqnum].pkt);
        tolayer3(0, A_sndpkt[nextseqnum].pkt);
        start_timer(nextseqnum, TimeoutInterval());
        nextseqnum++;
        A_buffer.pop();
    }
}

/* called when A's timer goes off */
void A_timerinterrupt() {
    starttimer(0, CLOCK_TICK);
    while (!timers.empty()) {
        timer t = timers.top();
        if (t.time <= get_sim_time()) {
            if (!cancelled_timers.count(t.seq)) {
                DEBUG_A("\033[31;1m" << "TIMEOUT Re-Sending: " << A_sndpkt[t.seq].pkt << "\033[0m");
                tolayer3(0, A_sndpkt[t.seq].pkt);
                A_sndpkt[t.seq].retransmitted = true;
                start_timer(t.seq, TimeoutInterval());
            } else {
//                DEBUG_A("\033[31;1m" << "TIMEOUT Cancelled: " << t << "\033[0m");
            }
            timers.pop();
        } else {
//            DEBUG_A("\033[31;1m" << "TIMEOUT BREAK: " << t << "\033[0m");
            break;
        }
    }
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init() {
    N = getwinsize();
    starttimer(0, CLOCK_TICK);
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet) {
    if (!is_corrupt(packet)) {
        if (packet.seqnum >= rcvbase && packet.seqnum < rcvbase + N) {
            struct pkt ack = make_ack(packet.seqnum);
            DEBUG_B("Sending ACK: " << ack);
            tolayer3(1, ack);

            if (!B_rcvpkt[packet.seqnum].acked) {
                struct B_buffer b = {packet, true};
                B_rcvpkt[packet.seqnum] = b;
            }

            if (packet.seqnum == rcvbase) {
                for (std::vector<B_buffer>::size_type i = (unsigned long) (rcvbase);
                     i != B_rcvpkt.size() && B_rcvpkt[i].acked; i++) {
                    DEBUG_B("\033[32;1m" << "Received: " << B_rcvpkt[i].pkt << "\033[0m");
                    tolayer5(1, B_rcvpkt[i].pkt.payload);
                    rcvbase++;
                    DEBUG_A("Advancing rcvbase to: " << rcvbase);
                }
            }

        } else if (packet.seqnum >= rcvbase - N && packet.seqnum < rcvbase) {
            struct pkt ack = make_ack(packet.seqnum);
            DEBUG_B("Sending DUP-ACK: " << ack);
            tolayer3(1, ack);
        }
    } else {
        DEBUG_B("Receive CORRUPT pkt, ignoring: " << packet);
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

void start_timer(int seq, float time) {
    timer t = {seq, time + get_sim_time()};
    timers.push(t);
}

void cancel_timer(int seq) {
    if (!cancelled_timers.count(seq)) {
        cancelled_timers.insert(seq);
    }
}
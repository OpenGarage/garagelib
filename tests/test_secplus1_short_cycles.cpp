// Sec+ 1.0 gap predictor: buffered or late-read cycles and millisecond rounding.
// Build without and with -DOG_SEC1_DIAGNOSTICS; uses only the public Gap API.
#include "secplus1_receiver.h"
#include <cassert>
#include <cstdio>
using SecPlus1Rx::Gap;

struct Exchange { uint32_t q, latency; };

// Feed exchanges at their reply read time and return the first millisecond in
// [from, to) at which a pending press would be allowed to transmit.
template <size_t N>
static uint32_t first_ready(const Exchange (&trace)[N], uint32_t from, uint32_t to) {
  Gap g; size_t next=0;
  for (uint32_t t=trace[0].q; t!=to; ++t) {
    while (next<N && trace[next].q+trace[next].latency==t) {
      g.exchange(trace[next].q,trace[next].q+trace[next].latency); ++next;
    }
    if (uint32_t(t-from)<uint32_t(to-from) && g.ready(t)) return t;
  }
  return UINT32_MAX;
}

static void train(Gap &g, uint32_t start, unsigned n=4, uint32_t interval=250, uint32_t latency=9) {
  for (unsigned i=0;i<n;++i) g.exchange(start+i*interval,start+i*interval+latency);
}

static void garage_expiry_replay() {
  // Frozen history from the Nov 19 garage expiry (queued 7808890, expired
  // 7810979), preceded by three clean polls at the same 250 ms cadence.
  const Exchange trace[]={
    {7806814,9},{7807064,10},{7807314,9},
    {7807564,9},{7807813,10},{7808063,10},{7808313,10},{7808563,10},
    {7808833,0},                       // read 20 ms late: pair buffered
    {7809063,8},{7809314,8},           // millisecond rounding of adjacent bytes
    {7809563,9},{7809813,9},{7810063,9},
    {7810353,0},                       // read 40 ms late: pair buffered
    {7810563,9},{7810812,10}};
  const uint32_t sent=first_ready(trace,7808890,7810979);
  std::printf("expiry replay: first transmit opportunity %u ms after queueing\n",unsigned(sent-7808890));
  assert(sent==7809063+8+10); // the first clean cycle after the command

  // Training survives the whole trace: every clean cycle opens its window.
  Gap g;
  for (const auto &e:trace) {
    g.exchange(e.q,e.q+e.latency);
    const uint32_t reply=e.q+e.latency;
    if (e.q<7807564) continue;
    if (e.latency<Gap::MIN_CLEAN_MS) {
      for (uint32_t age=0;age<=250;++age) assert(!g.ready(reply+age));
    } else {
      assert(!g.ready(reply+9) && g.ready(reply+10) && g.ready(reply+45) && !g.ready(reply+46));
    }
  }
}

static void rejected_cadences() {
  // A query earlier than the learned cadence allows means a faster panel.
  { Gap g; train(g,1000); g.exchange(1950,1950);   // 200 ms after the last clean query
    g.exchange(2250,2259); assert(!g.ready(2269)); }
  // Two buffered cycles decoded in the same pass: the second cannot be a new poll.
  { Gap g; train(g,1000); g.exchange(2020,2020); g.exchange(2020,2020);
    g.exchange(2250,2259); assert(!g.ready(2269)); }
  // Limit of four buffered cycles before a clean exchange must re-anchor.
  { Gap g; train(g,1000);
    for (unsigned k=1;k<=4;++k) g.exchange(1750+k*250+20,1750+k*250+20);
    g.exchange(1750+5*250,1750+5*250+9); assert(g.ready(1750+5*250+19)); }
  { Gap g; train(g,1000);
    for (unsigned k=1;k<=5;++k) g.exchange(1750+k*250+20,1750+k*250+20);
    g.exchange(1750+6*250,1750+6*250+9); assert(!g.ready(1750+6*250+19)); }
  // A buffered read up to MAX_LATE_MS past the learned maximum stays attributable.
  { Gap g; train(g,1000); g.exchange(2000+Gap::MAX_LATE_MS,2000+Gap::MAX_LATE_MS);
    g.exchange(2250,2259); assert(g.ready(2269)); }
  { Gap g; train(g,1000); g.exchange(2001+Gap::MAX_LATE_MS,2001+Gap::MAX_LATE_MS);
    g.exchange(2250,2259); assert(!g.ready(2269)); }
  // A missed poll after a buffered cycle fails the cadence check on the clean exchange.
  { Gap g; train(g,1000); g.exchange(2020,2020);
    g.exchange(2500,2509); assert(!g.ready(2519)); }
  // The clean exchange after a buffered cycle keeps the usual +/-20 ms tolerance.
  { Gap g; train(g,1000); g.exchange(2020,2020);
    g.exchange(2270,2279); assert(g.ready(2289)); }
  { Gap g; train(g,1000); g.exchange(2020,2020);
    g.exchange(2271,2280); assert(!g.ready(2290)); }
  // No learned interval yet: a buffered cycle restarts learning at the next
  // clean exchange, which then needs three more clean intervals.
  { Gap g; train(g,1000,1); g.exchange(1270,1270);
    g.exchange(1500,1509); g.exchange(1750,1759); g.exchange(2000,2009);
    assert(!g.ready(2019));
    g.exchange(2250,2259); assert(g.ready(2269)); }
  // Partly trained: a kept buffered cycle continues learning across the gap.
  { Gap g; train(g,1000,2); g.exchange(1520,1520);
    g.exchange(1750,1759); assert(!g.ready(1769));
    g.exchange(2000,2009); assert(g.ready(2019)); }
  // A reply read late (over 60 ms) is skipped like a buffered cycle: no send
  // in that cycle, training kept, and the same cadence checks apply.
  { Gap g; train(g,1000); g.exchange(2000,2063);
    for (uint32_t age=0;age<=250;++age) assert(!g.ready(2063+age));
    g.exchange(2250,2259); assert(g.ready(2269)); }
  { Gap g; train(g,1000); g.exchange(1950,2013);   // query 200 ms after the last
    g.exchange(2250,2259); assert(!g.ready(2269)); }
  for (uint32_t latency=Gap::MIN_CLEAN_MS;latency<=Gap::MAX_CLEAN_MS;++latency) {
    Gap g; train(g,1000); g.exchange(2000,2000+latency);
    assert(g.ready(2000+latency+10)==(latency+10+160<=250));
  }
  // Readings of 1-5 ms never authorize a send, even with training intact.
  for (uint32_t latency=1;latency<Gap::MIN_CLEAN_MS;++latency) {
    Gap g; train(g,1000); g.exchange(2000,2000+latency);
    for (uint32_t age=0;age<=250;++age) assert(!g.ready(2000+latency+age));
  }
}

static void wraparound() {
  Gap g; const uint32_t base=UINT32_MAX-900;
  train(g,base); g.exchange(base+1020,base+1020);
  g.exchange(base+1250,base+1258);
  assert(g.ready(base+1268) && !g.ready(base+1267));
}

int main() {
  garage_expiry_replay(); rejected_cadences(); wraparound();
  std::puts("Sec+ 1.0 short cycles: expiry replay, late replies, cadence rejection, skip limit and wraparound passed");
}

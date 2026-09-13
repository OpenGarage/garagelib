#define OG_SEC1_DIAGNOSTICS
#include "../secplus1_receiver.h"
#include <cassert>
#include <cstdio>
int main() {
  SecPlus1Rx::Gap gap;
  for(unsigned q=1000;q<=1750;q+=250) gap.exchange(q,q+30);
  assert(gap.learned_count()==3 && gap.diagnostic.max_count==3);
  assert(gap.diagnostic.exchanges==4 && gap.diagnostic.latency==30);
  assert(gap.diagnostic.interval==250 && gap.ready(1790));
  gap.exchange(2000,2000); // buffered query/reply decoded together
  assert(gap.diagnostic.bad_latency==1 && gap.diagnostic.zero_latency==1);
  assert(gap.learned_count()==0 && !gap.ready(2010));
  gap.exchange(2250,2259); // valid millisecond-quantized byte spacing
  assert(gap.diagnostic.bad_latency==1 && gap.diagnostic.latency==9);
  gap.exchange(2500,2508); // still below the accepted floor
  assert(gap.diagnostic.bad_latency==2 && gap.diagnostic.latency==8);
  assert(gap.diagnostic.zero_latency==1);
  std::puts("Sec+ 1.0 diagnostics: learned, batched, and short-latency observations passed");
}

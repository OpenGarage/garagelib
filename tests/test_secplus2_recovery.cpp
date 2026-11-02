#include <Arduino.h>
#include <SoftwareSerial.h>
#include <secplus.h>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include "../secplus2_recovery.h"
#define private public
#include "../garagelib.cpp"
#undef private
using namespace SecPlus2Recovery;
static void status(SecPlus2::Garage &g,uint32_t id=0x12342908) {
  uint8_t packet[19];
  assert(encode_wireline(100,id,0x81 | (2<<8),packet)==0);
  assert(g.process_packet(packet)==0);
}
static void policy() {
  Session s; uint32_t now=1000; s.start(now);
  for (auto wait : {30000U,60000U,120000U,240000U,300000U,300000U}) {
    assert(s.due(now)==Query::OPENINGS); s.sent(now,true);
    assert(s.due(now+99)==Query::NONE);
    assert(s.due(now+100)==Query::STATUS); s.sent(now+100,true);
    assert(s.due(now+200)==Query::NONE);
    s.tick(now+5000); assert(s.phase==Session::Phase::BACKOFF);
    s.status(now+5001); s.tick(now+5001); assert(!s.ready(now+5001));
    s.tick(now+5000+wait-1); assert(s.phase==Session::Phase::BACKOFF);
    now+=5000+wait; s.tick(now); assert(s.phase==Session::Phase::SEEKING);
  }
  s.sent(now,true); s.status(now+1); s.tick(now+1); assert(!s.ready(now+1));
  s.sent(now+100,true); s.tick(now+101); assert(s.ready(now+101));
  assert(s.due(now+10100)==Query::NONE);
  assert(s.due(now+10101)==Query::STATUS);
  s.sent(now+10101,false); assert(s.phase==Session::Phase::BACKOFF);
  s.tick(now+40101); assert(s.phase==Session::Phase::SEEKING); // successful observation reset backoff
  Session wrap; const uint32_t start=UINT32_MAX-50; wrap.start(start); wrap.sent(start,true);
  assert(wrap.due(uint32_t(start+100))==Query::STATUS);
  wrap.sent(uint32_t(start+100),true); wrap.status(uint32_t(start+101)); wrap.tick(uint32_t(start+101));
  assert(wrap.ready(uint32_t(start+101)));
  wrap.tick(uint32_t(start+30101)); assert(!wrap.ready(uint32_t(start+30101)));
}
static void integrated() {
  SecPlus2::Garage g(0x56782908,5,15);
  test_now=1000; g.begin(); g.reset_state();
  auto *uart=SecPlusCommon::sw_serial;
  g.loop(); assert(uart->tx.size()==19);
  uint32_t rc0,rc1,data0,data1; uint64_t fixed0,fixed1;
  assert(decode_wireline(uart->tx.data(),&rc0,&fixed0,&data0)==0);
  assert((((fixed0>>24)&0xf00)|(data0&0xff))==static_cast<unsigned>(SecPlus2::Command::GET_OPENINGS));
  test_now=1100; g.loop(); assert(uart->tx.size()==38);
  assert(decode_wireline(uart->tx.data()+19,&rc1,&fixed1,&data1)==0);
  assert((((fixed1>>24)&0xf00)|(data1&0xff))==static_cast<unsigned>(SecPlus2::Command::GET_STATUS));
  assert(rc1==((rc0+1)&0xfffffff));
  status(g); g.loop(); assert(g.recovery.ready(test_now));
  // After synchronized status observation, fetch openings once (the first may be ignored).
  uint32_t rc2,data2; uint64_t fixed2;
  test_now=1200; g.loop(); assert(uart->tx.size()==57);
  assert(decode_wireline(uart->tx.data()+38,&rc2,&fixed2,&data2)==0);
  assert((((fixed2>>24)&0xf00)|(data2&0xff))==static_cast<unsigned>(SecPlus2::Command::GET_OPENINGS));
  assert(rc2==((rc1+1)&0xfffffff));
  test_now=1300; g.loop(); assert(uart->tx.size()==57); // not repeated
  // Own RX echo neither supplies freshness nor double-applies a light toggle.
  uint8_t light[19];
  assert(encode_wireline(100,(uint64_t(0x200)<<24)|0x56782908,0x81|(2<<8),light)==0);
  const bool before=g.state.light_state;
  g.process_packet(light); assert(g.state.light_state==before);
  g.process_packet(light,false); assert(g.state.light_state!=before);
  g.process_packet(light); assert(g.state.light_state!=before);
  // A press colliding before transmission expires instead of replaying later.
  assert(g.open_door()==0); test_rx_high=1; test_now=1400; g.loop();
  assert(uart->tx.size()==57);
  test_now=3300; g.loop(); assert(g.buf.get_size()==0 && g.expired_commands==1);
  test_rx_high=0; test_now=3400; g.loop(); assert(uart->tx.size()==57);
  // Uncertain press delivery still releases; controls fault, status queries continue.
  assert(g.open_door()==0); uart->short_write=true; g.loop();
  assert(g.release_pending && g.control_fault && g.buf.get_size()==1);
  assert(g.command_pending()); // ID rotation must not interrupt release cleanup
  uart->short_write=false; test_now=3650; g.loop(); assert(!g.release_pending && g.buf.get_size()==0);
  assert(!g.command_pending());
  assert(g.close_door()<0);
  size_t count=uart->tx.size(); test_now=11100; g.loop(); assert(uart->tx.size()==count+19);
  // Opener remains absent >90 s while OG keeps its counter and does bounded retries.
  g.reset_state(); uart->tx.clear(); test_now=12000; g.loop(); test_now=12100; g.loop();
  const uint32_t after_pair=g.rolling_code;
  test_now=16100; g.loop(); assert(g.recovery.phase==Session::Phase::BACKOFF);
  assert(!g.status_valid() && g.get_door_state()==SecPlusCommon::DoorStatus::UNKNOWN);
  test_now=46100; g.loop(); test_now=46200; g.loop();
  assert(g.rolling_code==after_pair+2);
  test_now=51100; g.loop();
  status(g); g.loop(); assert(!g.recovery.ready(test_now)); // unsolicited frame cannot bypass paced retry
  test_now=111100; g.loop(); test_now=111200; g.loop(); status(g); g.loop();
  assert(g.recovery.ready(test_now)); assert(g.toggle_light()==0);
  assert(g.command_pending()); // queued, even before a write reaches the bus
  test_now+=100; g.loop();
  // A failed query heals through the same retry path, not a reboot-only latch.
  test_now+=10000; uart->short_write=true; g.loop();
  assert(g.query_failures>0 && !g.control_fault && g.recovery.phase==Session::Phase::BACKOFF);
  uart->short_write=false; test_now+=30000; g.loop(); test_now+=100; g.loop(); status(g); g.loop();
  assert(g.recovery.ready(test_now));
  g.stop(); assert(g.toggle_door()<0);
  SecPlusCommon::delete_sw_serial();
}
static void corrupt_release() {
  SecPlus2::Garage g(0x56782908,5,15);
  test_now=1000; g.begin(); g.reset_state();
  auto *uart=SecPlusCommon::sw_serial;
  g.loop(); test_now=1100; g.loop(); status(g); g.loop();
  assert(g.open_door()==0);
  test_now=1200; g.loop();
  assert(g.release_pending && g.buf.get_size()==1);
  // Fault injection only: the production encoder does not create this packet.
  std::fill(g.buf.get_head(),g.buf.get_head()+19,0xff);
  const size_t sent=uart->tx.size();
  test_now=1450; g.loop();
  assert(g.control_fault && !g.release_pending && g.buf.get_size()==0);
  assert(uart->tx.size()==sent && g.close_door()<0);
  test_now=11100; g.loop();
  assert(uart->tx.size()==sent+19); // query service survives; no action replay
  uint32_t rolling,data; uint64_t fixed;
  assert(decode_wireline(uart->tx.data()+sent,&rolling,&fixed,&data)==0);
  assert((((fixed>>24)&0xf00)|(data&0xff))==static_cast<unsigned>(SecPlus2::Command::GET_STATUS));
  g.stop(); SecPlusCommon::delete_sw_serial();
}
static void openings_refresh_priority() {
  SecPlus2::Garage g(0x56782908,5,15);
  test_now=1000; g.begin(); g.reset_state();
  auto *uart=SecPlusCommon::sw_serial;
  g.loop(); test_now=1100; g.loop(); status(g); g.loop();
  assert(g.ask_for_openings);
  assert(g.open_door()==0);
  test_now=1200; g.loop();
  assert(g.release_pending && g.ask_for_openings);
  test_now=1450; g.loop();
  assert(!g.release_pending && g.ask_for_openings);
  const size_t count=uart->tx.size();
  test_now=1490; g.loop(); assert(uart->tx.size()==count+19 && !g.ask_for_openings);
  uint32_t rolling,data; uint64_t fixed;
  assert(decode_wireline(uart->tx.data()+count,&rolling,&fixed,&data)==0);
  assert((((fixed>>24)&0xf00)|(data&0xff))==static_cast<unsigned>(SecPlus2::Command::GET_OPENINGS));
  uint8_t reply[19]; const auto command=static_cast<unsigned>(SecPlus2::Command::OPENINGS);
  assert(encode_wireline(101,0x12342908|((uint64_t(command)&0xf00)<<24),
         (command&0xff)|(239U<<24),reply)==0);
  assert(g.process_packet(reply)==0 && g.state.openings==239);
  test_now=1600; g.loop(); assert(uart->tx.size()==count+19);
  // Re-entering OBSERVED after stale-link recovery schedules one new refresh.
  test_now=32000; g.loop(); test_now=32100; g.loop(); status(g); g.loop();
  assert(g.recovery.ready(test_now) && g.ask_for_openings);
  const size_t recovery_count=uart->tx.size();
  test_now=32200; g.loop(); assert(uart->tx.size()==recovery_count+19 && !g.ask_for_openings);
  test_now=32300; g.loop(); assert(uart->tx.size()==recovery_count+19);
  g.stop(); SecPlusCommon::delete_sw_serial();
}
int main() { policy(); integrated(); corrupt_release(); openings_refresh_priority(); std::puts("Sec+ 2.0 recovery: policy, outage, collisions, partial writes, corrupt release, self echo and no-replay tests passed"); }

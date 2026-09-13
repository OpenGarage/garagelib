#include <Arduino.h>
#include <SoftwareSerial.h>
#include <secplus.h>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <string>
#include "../secplus1_receiver.h"
#define private public
#include "../garagelib.cpp"
#undef private

static void frame(SecPlus1Rx::Receiver &r,uint8_t query,uint8_t reply,uint32_t now) {
  r.feed(query,SecPlus1Rx::parity(query),now);
  r.feed(reply,SecPlus1Rx::parity(reply),now+30);
}
static void learn(SecPlus1Rx::Receiver &r,uint32_t start=1000,uint32_t interval=250,uint32_t latency=30) {
  for (unsigned i=0;i<4;++i) {
    r.feed(0x38,SecPlus1Rx::parity(0x38),start+i*interval);
    r.feed(0x05,SecPlus1Rx::parity(0x05),start+i*interval+latency);
  }
}
static void receiver_tests() {
  SecPlus1Rx::Receiver r;
  frame(r,0x38,5,1000); assert(!r.door.valid);
  frame(r,0x38,5,1750); assert(r.door.valid && r.door.value==5);
  frame(r,0x38,1,2500); assert(r.door.value==5);
  frame(r,0x38,5,3250); assert(r.door.value==5);
  frame(r,0x38,1,4000); frame(r,0x38,1,4750); assert(r.door.value==1);
  frame(r,0x3a,0x5c,5000); assert(!r.light_lock.valid);
  frame(r,0x3a,0x5c,5750); assert(r.light_lock.valid);
  r.tick(19779); assert(r.door.valid);
  r.tick(19780); assert(!r.door.valid && r.light_lock.valid);
  r.tick(20780); assert(!r.light_lock.valid);
  frame(r,0x38,5,21000); assert(!r.door.valid);
  frame(r,0x38,5,21750); assert(r.door.valid);
  // Bad parity between candidates prevents confirmation; it cannot flip a light.
  frame(r,0x3a,0x58,22000);
  r.feed(0x3a,SecPlus1Rx::parity(0x3a),22500);
  r.feed(0x5c,!SecPlus1Rx::parity(0x5c),22530);
  frame(r,0x3a,0x58,22750); assert(!r.light_lock.valid && r.parity_errors==1);
  frame(r,0x3a,0x58,23500); assert(r.light_lock.valid && r.light_lock.value==8);
  frame(r,0x38,0x31,24000); assert(r.door.value==5 && r.invalid_frames==1);
  r.loss(); assert(!r.door.valid && !r.light_lock.valid);
  frame(r,0x38,5,25000); r.tick(40100);
  frame(r,0x38,5,40200); assert(!r.door.valid);
  frame(r,0x39,1,41000); assert(r.obstruction_valid && r.obstruction);
  r.tick(56030); assert(!r.obstruction_valid);
  // Timeout and all relevant non-payload bytes invalidate a prediction.
  for (auto byte : {0x31,0x33,0x35,0x37,0xff}) {
    SecPlus1Rx::Receiver e; learn(e);
    assert(e.gap.ready(1790));
    e.feed(byte,SecPlus1Rx::parity(byte),1790);
    assert(!e.gap.ready(1790));
  }
  SecPlus1Rx::Receiver e; learn(e); e.feed(0x38,SecPlus1Rx::parity(0x38),1800); e.tick(1900);
  assert(!e.request && !e.gap.ready(1900));
}
static void gap_tests() {
  for (uint32_t latency=0;latency<9;++latency) {
    SecPlus1Rx::Receiver r; learn(r,1000,250,latency);
    assert(!r.gap.ready(1800));
  }
  // Mixed 9/10 ms observations must retain the same 250 ms panel cadence.
  SecPlus1Rx::Receiver mixed;
  for(unsigned i=0;i<4;++i) {
    const uint32_t q=1000+i*250;
    mixed.feed(0x38,SecPlus1Rx::parity(0x38),q);
    mixed.feed(5,SecPlus1Rx::parity(5),q+9+(i%2));
  }
  assert(!mixed.gap.ready(1769)); // still wait 10 ms AFTER the reply
  assert(mixed.gap.ready(1770));
  mixed.gap.exchange(2000,2061); // upper bound remains unchanged
  assert(!mixed.gap.ready(2071));
  for (uint32_t interval=180;interval<=400;++interval) {
    for (uint32_t latency=9;latency<=60;++latency) {
      SecPlus1Rx::Receiver r; learn(r,1000,interval,latency);
      const uint32_t reply=1000+3*interval+latency;
      assert(r.gap.insufficient(reply)==(interval<170+latency));
      for (uint32_t age=0;age<=450;++age) {
        const bool expected=age>=10 && age<=45 && latency+age<interval && interval-latency-age>=160;
        assert(r.gap.ready(reply+age)==expected);
      }
    }
  }
  SecPlus1Rx::Receiver wrap;
  learn(wrap,UINT32_MAX-700);
  assert(wrap.gap.ready(uint32_t(UINT32_MAX-700+790)));
}
static void transaction_tests() {
  SecPlus1::Garage g(5,15);
  test_now=1000; g.begin();
  auto *uart=SecPlusCommon::sw_serial;
  learn(g.receiver); test_now=1780;
  assert(g.toggle_light()==0); g.loop(); assert(uart->tx.empty());
  test_now=1790; g.loop(); assert(uart->tx==std::vector<uint8_t>{0x32});
  assert(g.toggle_door()<0); // no overlap
  test_now=1829; g.loop(); assert(uart->tx.size()==1);
  test_now=1830; g.loop(); assert(uart->tx==std::vector<uint8_t>({0x32,0x33}));
  test_now=1870; g.loop(); assert(uart->tx==std::vector<uint8_t>({0x32,0x33,0x33}));
  // Consumed prediction cannot dispatch another press without new exchanges.
  assert(!g.receiver.gap.ready(1870));
  assert(g.toggle_door()==0); test_now=3870; g.loop();
  assert(uart->tx.size()==3 && g.expired_commands==1);
  learn(g.receiver,4000); test_now=4790; g.loop(); assert(uart->tx.size()==3); // no late replay
  // Partial press write still gets a release tail, never a second press.
  uart->short_write=true; assert(g.toggle_door()==0); g.loop();
  assert(uart->tx.back()==0x30 && g.control_fault);
  uart->short_write=false;
  test_now=4830; g.loop(); test_now=4870; g.loop();
  assert(uart->tx[uart->tx.size()-2]==0x31 && uart->tx.back()==0x31);
  assert(g.toggle_door()<0);
  // No panel: Toggle does not require known door state.
  g.reset_state(); uart->tx.clear(); test_now=30000;
  g.panel_emu_status=SecPlus1::PanelEmuStatus::ACTIVE;
  assert(!g.door_valid()); assert(g.toggle_door()==0); g.loop(); assert(uart->tx.back()==0x30);
  g.stop(); assert(uart->tx==std::vector<uint8_t>({0x30,0x31,0x31}));
  assert(!g.pending_press && !g.releases_left);
  assert(g.toggle_door()<0); // stopped transport cannot enqueue work
  // Physical RX activity blocks new presses and they expire unsent.
  g.begin(); uart=SecPlusCommon::sw_serial; g.panel_emu_status=SecPlus1::PanelEmuStatus::ACTIVE;
  test_rx_high=1; assert(g.toggle_door()==0); test_now+=100; g.loop(); assert(uart->tx.empty());
  test_now+=1900; g.loop(); assert(uart->tx.empty() && !g.pending_press);
  test_rx_high=0;
  // Actual UART pump checks parity and publishes confirmation/expiry.
  g.begin(); uart=SecPlusCommon::sw_serial;
  for (unsigned n=0;n<2;++n) {
    test_now=40000+n*750; uart->rx.push_back(0x38); uart->parity_bit=SecPlus1Rx::parity(0x38); g.loop();
    test_now+=30; uart->rx.push_back(5); uart->parity_bit=SecPlus1Rx::parity(5); g.loop();
    assert(g.door_valid()==(n==1));
  }
  assert(g.get_door_state()==SecPlusCommon::DoorStatus::CLOSED);
  test_now+=15000; g.loop(); assert(!g.door_valid() && g.get_door_state()==SecPlusCommon::DoorStatus::UNKNOWN);
  // Release deadline defeats a backlog; it never replays a press.
  g.begin(); uart=SecPlusCommon::sw_serial;
  g.panel_emu_status=SecPlus1::PanelEmuStatus::ACTIVE;
  test_now+=100; assert(g.toggle_door()==0); g.loop(); assert(uart->tx.size()==1);
  uart->rx.assign(100,0xff); test_now+=40; g.loop(); assert(uart->tx.size()==1);
  test_now+=960; g.loop(); assert(uart->tx.size()==2 && uart->tx.back()==0x31);
  test_now+=40; g.loop(); assert(uart->tx.size()==3 && uart->tx.back()==0x31);
  SecPlusCommon::delete_sw_serial();
}
static void panel_emulation_setting_tests() {
  SecPlus1::Garage g(5,15);
  test_now=1000;
  g.set_panel_emulation_disabled(true);
  g.begin();
  assert(g.get_panel_emu_status()==0);
  assert(std::string(g.get_panel_description())=="Disabled");
  test_now=22000; g.loop();
  assert(g.get_panel_emu_status()==0 && SecPlusCommon::sw_serial->tx.empty());
  // With a panel present, disabling only emulation must preserve gap-aware actions.
  learn(g.receiver,22000);
  test_now=22790;
  assert(g.toggle_light()==0); g.loop();
  assert(SecPlusCommon::sw_serial->tx==std::vector<uint8_t>{0x32});
  g.finish_release();
  g.set_panel_emulation_disabled(false);
  g.reset_state();
  assert(g.get_panel_emu_status()==2);
  test_now=44001; g.loop();
  assert(g.get_panel_emu_status()==1);
  SecPlusCommon::delete_sw_serial();
}
int main() {
  receiver_tests(); gap_tests(); transaction_tests(); panel_emulation_setting_tests();
  std::puts("Sec+ 1.0: receiver, 11492 timing combinations, wraparound and transaction tests passed");
}

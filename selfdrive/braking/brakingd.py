#!/usr/bin/env python3
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
import cereal.messaging as messaging
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.boardd.boardd import can_list_to_can_capnp

from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS
from opendbc.can.packer import CANPacker

class BrakingPq:
  def __init__(self):
    self.frame = 0
    config_realtime_process(2, Priority.CTRL_HIGH)

    # Setup sockets
    self.pm = messaging.PubMaster(['sendcan'])

    self.rk = Ratekeeper(100, print_delay_threshold=0.01) #10 ms, about controlsd time

    #Fun stuff
    self.create_braking_control = volkswagencan.create_pq_braking_control
    self.packer_pt = CANPacker(DBC_FILES.pq)

  def step(self):
    can_sends = []
    if self.frame%2 == 0:
      idx = (self.frame / 2) % 16
      can_sends.append(self.create_braking_control(self.packer_pt, CANBUS.br, 0, idx, False, False, False))

    self.frame = self.frame + 1

    self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=True))

  def thread(self):
    while True:
      self.step()
      self.rk.keep_time()

if __name__ == "__main__":
  main()

def main():
  braking = BrakingPq()
  braking.thread()

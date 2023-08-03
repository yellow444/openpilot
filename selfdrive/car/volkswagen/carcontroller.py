from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import mqbcan, pqcan
from selfdrive.car.volkswagen.values import PQ_CARS, DBC_FILES, CANBUS, NetworkLocation, MQB_LDW_MESSAGES, \
    BUTTON_STATES, CarControllerParams as P
from selfdrive.config import Conversions as CV
from opendbc.can.packer import CANPacker

LongCtrlState = car.CarControl.Actuators.LongControlState
VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController():
    def __init__(self, dbc_name, CP, VM):
        self.CCS = pqcan if CP.carFingerprint in PQ_CARS else mqbcan
        self.apply_steer_last = 0
        self.epbFreigabe = False
        self.coastingAccel = -0.42735
        
        self.hcaSameTorqueCount = 0
        self.hcaEnabledFrameCount = 0
        self.graButtonStatesToSend = None
        self.graMsgSentCount = 0
        self.graMsgStartFramePrev = 0
        self.graMsgBusCounterPrev = 0
        
        if CP.carFingerprint in PQ_CARS:
            self.packer_pt = CANPacker(DBC_FILES.pq)
            self.ldw_step = P.PQ_LDW_STEP
        
        else:
            self.packer_pt = CANPacker(DBC_FILES.mqb)
            self.ldw_step = P.MQB_LDW_STEP
        
        if CP.networkLocation == NetworkLocation.fwdCamera:
            self.ext_can = CANBUS.pt
        else:
            self.ext_can = CANBUS.cam
        
        self.steer_rate_limited = False
    
    def update(self, enabled, CS, frame, ext_bus, actuators, visual_alert, left_lane_visible, right_lane_visible,
               left_lane_depart, right_lane_depart):
        """ Controls thread """
        
        can_sends = []
        
        # **** Steering Controls ************************************************ #
        
        if frame % P.HCA_STEP == 0:
            # Logic to avoid HCA state 4 "refused":
            #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
            #   * Don't steer at standstill
            #   * Don't send > 3.00 Newton-meters torque
            #   * Don't send the same torque for > 6 seconds
            #   * Don't send uninterrupted steering for > 360 seconds
            # One frame of HCA disabled is enough to reset the timer, without zeroing the
            # torque value. Do that anytime we happen to have 0 torque, or failing that,
            # when exceeding ~1/3 the 360 second timer.
            
            if enabled and CS.out.vEgo > CS.CP.minSteerSpeed and not (
                    CS.out.standstill or CS.out.steerError or CS.out.steerWarning):
                new_steer = int(round(actuators.steer * P.STEER_MAX))
                apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
                self.steer_rate_limited = new_steer != apply_steer
                
                if apply_steer == 0:
                    hcaEnabled = False
                    self.hcaEnabledFrameCount = 0
                else:
                    self.hcaEnabledFrameCount += 1
                    if self.hcaEnabledFrameCount >= 118 * (100 / P.HCA_STEP):  # 118s
                        hcaEnabled = False
                        self.hcaEnabledFrameCount = 0
                    else:
                        hcaEnabled = True
                        if self.apply_steer_last == apply_steer:
                            self.hcaSameTorqueCount += 1
                            if self.hcaSameTorqueCount > 1.9 * (100 / P.HCA_STEP):  # 1.9s
                                apply_steer -= (1, -1)[apply_steer < 0]
                                self.hcaSameTorqueCount = 0
                        else:
                            self.hcaSameTorqueCount = 0
            else:
                hcaEnabled = False
                apply_steer = 0
            
            self.apply_steer_last = apply_steer
            idx = (frame / P.HCA_STEP) % 16
            can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, idx, hcaEnabled))
        
        # **** ACC Controls ***************************************************** #
        if (frame % P.ACC_STEP == 0) and CS.CP.openpilotLongitudinalControl:
            idx = (frame / P.EPB_STEP) % 16
            acc_type = 0  # TODO: detect ACC type installed
            
            acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, enabled)
            accel = clip(actuators.accel, P.ACCEL_MIN, P.ACCEL_MAX) if enabled else 0
            stopping = actuators.longControlState == LongCtrlState.stopping
            starting = actuators.longControlState == LongCtrlState.starting
            
            can_sends.append(
                self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, idx, CS.acc_type, enabled, accel,
                                                  acc_control,
                                                  stopping, starting))
        
        # **** HUD Controls ***************************************************** #
        if (frame % P.ACC_STEP == 0) and CS.CP.openpilotLongitudinalControl:
            idx = (frame / P.EPB_STEP) % 16
            set_speed = CS.out.cruiseState.speed * CV.MS_TO_KPH
            metric = True  # TODO: detect kp_h or mph from car
            acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, enabled)
            
            can_sends.append(
                self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, idx, acc_hud_status, set_speed, metric, 0))
        
        if frame % self.ldw_step == 0:
            hca_enabled = True if enabled and not CS.out.standstill else False
            
            # FIXME: drive this down to the MQB/PQ specific create_hud_control functions
            if visual_alert in [VisualAlert.steerRequired, VisualAlert.ldw]:
                hud_alert = 4
            else:
                hud_alert = 0
            
            can_sends.append(self.create_hud_control(self.packer_pt, CANBUS.pt, hca_enabled,
                                                     CS.out.steeringPressed, hud_alert, left_lane_visible,
                                                     right_lane_visible, CS.ldw_lane_warning_left,
                                                     CS.ldw_lane_warning_right, CS.ldw_side_dlc_tlc,
                                                     CS.ldw_dlc, CS.ldw_tlc, CS.out.standstill,
                                                     left_lane_depart, right_lane_depart))
        
        # **** GAS Controls ***************************************************** #
        if (frame % P.GAS_STEP == 0) and CS.CP.enableGasInterceptor:
            apply_gas = 0
            if enabled:
                speed = CS.out.vEgo
                cd = 0.31
                frontalArea = 2.3
                drag = 0.5 * cd * frontalArea * (speed ** 2)
                
                mass = 1250
                g = 9.81
                rollingFrictionCoefficient = 0.02
                friction = mass * g * rollingFrictionCoefficient
                
                desiredAcceleration = actuators.accel
                acceleration = mass * desiredAcceleration
                
                driveTrainLosses = 0  # 600 for the engine, 200 for trans, low speed estimate
                powerNeeded = (drag + friction + acceleration) * speed + driveTrainLosses
                POWER_LOOKUP_BP = [0, 25000 * 1.6 / 2.6,
                                   75000]  # 160NM@1500rpm=25kW but with boost, no boost means *1.6/2.6
                PEDAL_LOOKUP_BP = [227, 1250 * 0.4,
                                   1250 * 100 / 140]  # Not max gas, max gas gives 140hp, we want at most 100 hp, also 40% throttle might prevent an upshift
                
                GAS_MULTIPLIER_BP = [0, 0.1, 0.2, 0.4, 8.3]
                GAS_MULTIPLIER_V = [1.15, 1.15, 1.2, 1.25, 1.]
                
                powerNeeded_mult = interp(CS.out.vEgo, [20 / 3.6, 40 / 3.6], [2, 1])
                powerNeeded = int(round(powerNeeded * powerNeeded_mult))
                apply_gas = int(round(interp(powerNeeded, POWER_LOOKUP_BP, PEDAL_LOOKUP_BP)))
                apply_gas = int(round(apply_gas * int(round(interp(speed, GAS_MULTIPLIER_BP, GAS_MULTIPLIER_V)))))
            else:
                apply_gas = 0
        
            # can_sends.append(self.create_gas_control(self.packer_pt, CANBUS.cam, apply_gas, frame // 2))
        
        # **** AWV Controls ***************************************************** #
        
        if (frame % P.AWV_STEP == 0) and CS.CP.enableGasInterceptor:
            green_led = 1 if enabled else 0
            orange_led = 1 if self.epbFreigabe else 0
            
            idx = (frame / P.AWV_STEP) % 16
            
            can_sends.append(self.CCS.create_awv_control(self.packer_pt, CANBUS.pt, idx, orange_led, green_led,
                                                         CS.mAWV))
        
        # **** EPB Controls ***************************************************** #
        
        if (frame % P.EPB_STEP == 0) and CS.CP.enableGasInterceptor:
            epbFreigabe = self.epbFreigabe
            apply_brake = interp(actuators.accel, P.BRAKE_LOOKUP_BP, P.BRAKE_LOOKUP_V)
            
            if enabled:
                if apply_brake < self.coastingAccel:
                    if not epbFreigabe:
                        epbFreigabe = True
                        apply_brake = 0
                else:
                    epbFreigabe = False
                    apply_brake = 0
            else:
                epbFreigabe = False
                apply_brake = 0
            self.epbFreigabe = epbFreigabe
            
            idx = (frame / P.EPB_STEP) % 16
        
        # can_sends.append(self.create_epb_control(self.packer_pt, CANBUS.br, apply_brake, epbFreigabe, idx))
        
        # **** ACC Button Controls ********************************************** #
        
        # FIXME: this entire section is in desperate need of refactoring
        
        if frame > self.graMsgStartFramePrev + P.GRA_VBP_STEP:
            if not enabled and CS.out.cruiseState.enabled:
                # Cancel ACC if it's engaged with OP disengaged.
                self.graButtonStatesToSend = BUTTON_STATES.copy()
                self.graButtonStatesToSend["cancel"] = True
            elif enabled and CS.out.standstill:
                # Blip the Resume button if we're engaged at standstill.
                # FIXME: This is a naive implementation, improve with visiond or radar input.
                # A subset of MQBs like to "creep" too aggressively with this implementation.
                self.graButtonStatesToSend = BUTTON_STATES.copy()
                self.graButtonStatesToSend["resumeCruise"] = True
        
        if CS.graMsgBusCounter != self.graMsgBusCounterPrev:
            self.graMsgBusCounterPrev = CS.graMsgBusCounter
            if self.graButtonStatesToSend is not None:
                if self.graMsgSentCount == 0:
                    self.graMsgStartFramePrev = frame
                idx = (CS.graMsgBusCounter + 1) % 16
                # can_sends.append(self.create_acc_buttons_control(self.packer_pt, self.ext_can, self.graButtonStatesToSend, CS, idx))
                self.graMsgSentCount += 1
                if self.graMsgSentCount >= P.GRA_VBP_COUNT:
                    self.graButtonStatesToSend = None
                    self.graMsgSentCount = 0
        
        return can_sends

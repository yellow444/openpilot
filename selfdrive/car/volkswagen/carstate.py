import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import PQ_CARS, DBC_FILES, CANBUS, NetworkLocation, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.buttonStates = BUTTON_STATES.copy()

    if CP.carFingerprint in PQ_CARS:
      can_define = CANDefine(DBC_FILES.pq)
      self.get_can_parser = self.get_pq_can_parser
      self.get_cam_can_parser = self.get_pq_cam_can_parser
      self.update = self.update_pq
      self.hca_status_values = can_define.dv["Lenkhilfe_2"]["LH2_Sta_HCA"]
      if CP.transmissionType == TransmissionType.automatic:
        self.shifter_values = can_define.dv["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"]
      if CP.enableGasInterceptor:
        self.openpilot_enabled = False
    else:
      can_define = CANDefine(DBC_FILES.mqb)
      self.get_can_parser = self.get_mqb_can_parser
      self.get_cam_can_parser = self.get_mqb_cam_can_parser
      self.update = self.update_mqb
      self.hca_status_values = can_define.dv["LH_EPS_03"]["EPS_HCA_Status"]
      if CP.transmissionType == TransmissionType.automatic:
        self.shifter_values = can_define.dv["Getriebe_11"]["GE_Fahrstufe"]
      elif CP.transmissionType == TransmissionType.direct:
        self.shifter_values = can_define.dv["EV_Gearshift"]["GearPosition"]

  def update_mqb(self, pt_cp, cam_cp, ext_cp, trans_type):

    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["ESP_19"]["ESP_VL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_VR_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HR_Radgeschw_02"],
    )

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgo < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["LH_EPS_03"]["EPS_Berechneter_LW"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_BLW"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["ESP_02"]["ESP_Gierrate"] * (1, -1)[int(pt_cp.vl["ESP_02"]["ESP_VZ_Gierrate"])] * CV.DEG_TO_RAD

    # Verify EPS readiness to accept steering commands
    hca_status = self.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerError = hca_status in ["DISABLED", "FAULT"]
    ret.steerWarning = hca_status in ["INITIALIZING", "REJECTED"]

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    self.esp_hold_confirmation = pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"]

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
    elif trans_type == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["EV_Gearshift"]["GearPosition"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_14"]["MO_Kuppl_schalter"]
      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gateway_72"]["ZV_FT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_BT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HBFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HD_offen"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_01"]["KBI_MFA_v_Einheit_02"]

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapter on Front Assist with Braking: Golf Family for all MQB
    ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
    ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

    # Update ACC radar status.
    self.tsk_status = pt_cp.vl["TSK_06"]["TSK_Status"]
    if self.tsk_status == 2:
      # ACC okay and enabled, but not currently engaged
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
    elif self.tsk_status in [3, 4, 5]:
      # ACC okay and enabled, currently regulating speed (3) or driver accel override (4) or overrun coast-down (5)
      ret.cruiseState.available = True
      ret.cruiseState.enabled = True
    else:
      # ACC okay but disabled (1), or a radar visibility or other fault/disruption (6 or 7)
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    if self.CP.pcmCruise:
      ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw"] * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:
        ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Hoch"])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Runter"])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Abbrechen"])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Setzen"])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Wiederaufnahme"])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Verstellung_Zeitluecke"])
    ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
    ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    self.graHauptschalter = pt_cp.vl["GRA_ACC_01"]["GRA_Hauptschalter"]
    self.graTypHauptschalter = pt_cp.vl["GRA_ACC_01"]["GRA_Typ_Hauptschalter"]
    self.graButtonTypeInfo = pt_cp.vl["GRA_ACC_01"]["GRA_ButtonTypeInfo"]
    self.graTipStufe2 = pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Stufe_2"]
    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    self.graMsgBusCounter = pt_cp.vl["GRA_ACC_01"]["COUNTER"]

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0

    return ret

  def update_pq(self, pt_cp, cam_cp, ext_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds.fl = pt_cp.vl["Bremse_3"]["Radgeschw__VL_4_1"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["Bremse_3"]["Radgeschw__VR_4_1"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["Bremse_3"]["Radgeschw__HL_4_1"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["Bremse_3"]["Radgeschw__HR_4_1"] * CV.KPH_TO_MS

    self.Stillstand = pt_cp.vl["Bremse_5"]["BR5_Stillstand"]

    self.mAWV = cam_cp.vl["mAWV"]

    self.ACC_Coding = cam_cp.vl["ACA_Codierung"]

    self.acc_type = ext_cp.vl['ACC_System']['ACS_Typ_ACC']

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit_S"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]["BR5_Giergeschw"] * (1, -1)[int(pt_cp.vl["Bremse_5"]["BR5_Vorzeichen"])] * CV.DEG_TO_RAD

    # Verify EPS readiness to accept steering commands
    hca_status = self.hca_status_values.get(pt_cp.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
    ret.steerError = hca_status in ["DISABLED", "FAULT"]
    ret.steerWarning = hca_status in ["REJECTED"]

    # Update gas, brakes, and gearshift
    if not self.CP.enableGasInterceptor:
      ret.gas = pt_cp.vl["Motor_3"]['Fahrpedal_Rohsignal'] / 100.0
      ret.gasPressed = ret.gas > 0

    ret.brake = pt_cp.vl["Bremse_5"]["Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["Bremstestschalter"])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_1"]["Kupplungsschalter"]
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Rueckfahr"])
      if reverse_light:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    # TODO: need to locate signals for other three doors if possible
    ret.doorOpen = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    # TODO: read PQ Einheiten here
    self.displayMetricUnits = False

    # Consume blind-spot monitoring info/warning LED states, if available. The
    # info signal (LED on) is enabled whenever a vehicle is detected in the
    # driver's blind spot. The warning signal (LED flashing) is enabled if the
    # driver shows possibly hazardous intent toward a BSM detected vehicle, by
    # setting the turn signal in that direction, or (for cars with factory Lane
    # Assist) approaches the lane boundary in that direction. Size of the BSM
    # detection box is dynamic based on speed and road curvature.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assist Systems,
    # pages 32-35.
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    # TODO: Consume lane departure data from factory camera, if present
    self.ldw_lane_warning_left = False
    self.ldw_lane_warning_right = False
    self.ldw_side_dlc_tlc = None
    self.ldw_dlc = None
    self.ldw_tlc = None

    self.ldw_stock_values = False
    # TODO: Consume FCW/AEB data from factory radar, if present

    # Update ACC radar status.
    ret.cruiseState.available = bool(pt_cp.vl["Motor_5"]["GRA_Hauptschalter"])
    ret.cruiseState.enabled = pt_cp.vl["Motor_2"]["GRA_Status"] in (1, 2)

    # Set override flag for openpilot enabled state.
    if self.CP.enableGasInterceptor and pt_cp.vl["Motor_2"]['GRA_Status'] in [1, 2]:
      self.openpilot_enabled = True

    # Check if Gas or Brake pressed and cancel override
    if self.CP.enableGasInterceptor and (ret.gasPressed or ret.brakePressed):
      self.openpilot_enabled = False

    # Override openpilot enabled if gas interceptor installed
    if self.CP.enableGasInterceptor and self.openpilot_enabled:
      ret.cruiseState.enabled = True

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_V_Wunsch"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Up_kurz"]) or bool(pt_cp.vl["GRA_Neu"]["GRA_Up_lang"])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Down_kurz"]) or bool(pt_cp.vl["GRA_Neu"]["GRA_Down_lang"])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Abbrechen"])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Neu_Setzen"])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Recall"])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Zeitluecke"])
    ret.leftBlinker = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"])
    ret.rightBlinker = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    self.graHauptschalter = pt_cp.vl["GRA_Neu"]["GRA_Hauptschalt"]
    self.graSenderCoding = pt_cp.vl["GRA_Neu"]["GRA_Sender"]
    self.graTypHauptschalter = False
    self.graButtonTypeInfo = False
    self.graTipStufe2 = False

    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    self.graMsgBusCounter = pt_cp.vl["GRA_Neu"]["GRA_Neu_Zaehler"]

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = False #bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])  # FIXME: need to include an EPB check as well
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["ESP_Passiv_getastet"])

    if self.CP.enableGasInterceptor:
      self.currentSpeed = ret.vEgo

    return ret

  @staticmethod
  def get_mqb_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("EPS_Berechneter_LW", "LH_EPS_03", 0),       # Absolute steering angle
      ("EPS_VZ_BLW", "LH_EPS_03", 0),               # Steering angle sign
      ("LWI_Lenkradw_Geschw", "LWI_01", 0),         # Absolute steering rate
      ("LWI_VZ_Lenkradw_Geschw", "LWI_01", 0),      # Steering rate sign
      ("ESP_VL_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, front left
      ("ESP_VR_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, front right
      ("ESP_HL_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, rear left
      ("ESP_HR_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, rear right
      ("ESP_Gierrate", "ESP_02", 0),                # Absolute yaw rate
      ("ESP_VZ_Gierrate", "ESP_02", 0),             # Yaw rate sign
      ("ZV_FT_offen", "Gateway_72", 0),             # Door open, driver
      ("ZV_BT_offen", "Gateway_72", 0),             # Door open, passenger
      ("ZV_HFS_offen", "Gateway_72", 0),            # Door open, rear left
      ("ZV_HBFS_offen", "Gateway_72", 0),           # Door open, rear right
      ("ZV_HD_offen", "Gateway_72", 0),             # Trunk or hatch open
      ("Comfort_Signal_Left", "Blinkmodi_02", 0),   # Left turn signal including comfort blink interval
      ("Comfort_Signal_Right", "Blinkmodi_02", 0),  # Right turn signal including comfort blink interval
      ("AB_Gurtschloss_FA", "Airbag_02", 0),        # Seatbelt status, driver
      ("AB_Gurtschloss_BF", "Airbag_02", 0),        # Seatbelt status, passenger
      ("ESP_Fahrer_bremst", "ESP_05", 0),           # Brake pedal pressed
      ("ESP_Bremsdruck", "ESP_05", 0),              # Brake pressure applied
      ("MO_Fahrpedalrohwert_01", "Motor_20", 0),    # Accelerator pedal value
      ("EPS_Lenkmoment", "LH_EPS_03", 0),           # Absolute driver torque input
      ("EPS_VZ_Lenkmoment", "LH_EPS_03", 0),        # Driver torque input sign
      ("EPS_HCA_Status", "LH_EPS_03", 3),           # EPS HCA control status
      ("ESP_Tastung_passiv", "ESP_21", 0),          # Stability control disabled
      ("ESP_Haltebestaetigung", "ESP_21", 0),       # ESP hold confirmation
      ("KBI_MFA_v_Einheit_02", "Einheiten_01", 0),  # MPH vs KMH speed display
      ("KBI_Handbremse", "Kombi_01", 0),            # Manual handbrake applied
      ("TSK_Status", "TSK_06", 0),                  # ACC engagement status from drivetrain coordinator
      ("GRA_Hauptschalter", "GRA_ACC_01", 0),       # ACC button, on/off
      ("GRA_Abbrechen", "GRA_ACC_01", 0),           # ACC button, cancel
      ("GRA_Tip_Setzen", "GRA_ACC_01", 0),          # ACC button, set
      ("GRA_Tip_Hoch", "GRA_ACC_01", 0),            # ACC button, increase or accel
      ("GRA_Tip_Runter", "GRA_ACC_01", 0),          # ACC button, decrease or decel
      ("GRA_Tip_Wiederaufnahme", "GRA_ACC_01", 0),  # ACC button, resume
      ("GRA_Verstellung_Zeitluecke", "GRA_ACC_01", 0),  # ACC button, time gap adj
      ("GRA_Typ_Hauptschalter", "GRA_ACC_01", 0),   # ACC main button type
      ("GRA_Tip_Stufe_2", "GRA_ACC_01", 0),         # unknown related to stalk type
      ("GRA_ButtonTypeInfo", "GRA_ACC_01", 0),      # unknown related to stalk type
      ("COUNTER", "GRA_ACC_01", 0),                 # GRA_ACC_01 CAN message counter
    ]

    checks = [
      # sig_address, frequency
      ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
      ("LH_EPS_03", 100),   # From J500 Steering Assist with integrated sensors
      ("ESP_19", 100),      # From J104 ABS/ESP controller
      ("ESP_05", 50),       # From J104 ABS/ESP controller
      ("ESP_21", 50),       # From J104 ABS/ESP controller
      ("Motor_20", 50),     # From J623 Engine control module
      ("TSK_06", 50),       # From J623 Engine control module
      ("ESP_02", 50),       # From J104 ABS/ESP controller
      ("GRA_ACC_01", 33),   # From J533 CAN gateway (via LIN from steering wheel controls)
      ("Gateway_72", 10),   # From J533 CAN gateway (aggregated data)
      ("Airbag_02", 5),     # From J234 Airbag control module
      ("Kombi_01", 2),      # From J285 Instrument cluster
      ("Blinkmodi_02", 1),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("Einheiten_01", 1),  # From J??? not known if gateway, cluster, or BCM
    ]

    if CP.transmissionType == TransmissionType.automatic:
      signals += [("GE_Fahrstufe", "Getriebe_11", 0)]  # Auto trans gear selector position
      checks += [("Getriebe_11", 20)]  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.direct:
      signals += [("GearPosition", "EV_Gearshift", 0)]  # EV gear selector position
      checks += [("EV_Gearshift", 10)]  # From J??? unknown EV control module
    elif CP.transmissionType == TransmissionType.manual:
      signals += [("MO_Kuppl_schalter", "Motor_14", 0),  # Clutch switch
                  ("BCM1_Rueckfahrlicht_Schalter", "Gateway_72", 0)]  # Reverse light from BCM
      checks += [("Motor_14", 10)]  # From J623 Engine control module

    if CP.networkLocation == NetworkLocation.fwdCamera:

      # Radars are here on CANBUS.pt
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.pt)

  @staticmethod
  def get_pq_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("LH3_BLW", "Lenkhilfe_3", 0),                # Absolute steering angle
      ("LH3_BLWSign", "Lenkhilfe_3", 0),            # Steering angle sign
      ("LH3_LM", "Lenkhilfe_3", 0),                 # Absolute driver torque input
      ("LH3_LMSign", "Lenkhilfe_3", 0),             # Driver torque input sign
      ("LH2_Sta_HCA", "Lenkhilfe_2", 0),            # Steering rack HCA status
      ("Lenkradwinkel_Geschwindigkeit", "Lenkwinkel_1", 0),  # Absolute steering rate
      ("Lenkradwinkel_Geschwindigkeit_S", "Lenkwinkel_1", 0),  # Steering rate sign
      ("Radgeschw__VL_4_1", "Bremse_3", 0),         # ABS wheel speed, front left
      ("Radgeschw__VR_4_1", "Bremse_3", 0),         # ABS wheel speed, front right
      ("Radgeschw__HL_4_1", "Bremse_3", 0),         # ABS wheel speed, rear left
      ("Radgeschw__HR_4_1", "Bremse_3", 0),         # ABS wheel speed, rear right
      ("Gurtschalter_Fahrer", "Airbag_1", 0),       # Seatbelt status, driver
      ("Gurtschalter_Beifahrer", "Airbag_1", 0),    # Seatbelt status, passenger
      ("Bremstestschalter", "Motor_2", 0),          # Brake pedal pressed (brake light test switch)
      ("Bremslichtschalter", "Motor_2", 0),         # Brakes applied (brake light switch)
      ("Soll_Geschwindigkeit_bei_GRA_Be", "Motor_2", 0), #CruiseControl Setspeed
      ("Fahrpedal_Rohsignal", "Motor_3", 0),        # Accelerator pedal value
      ("ESP_Passiv_getastet", "Bremse_1", 0),       # Stability control disabled
      ("GRA_Hauptschalter", "Motor_5", 0),             # ACC main switch
      ("GRA_Status", "Motor_2", 0),                 # ACC engagement status
      ("GK1_Fa_Tuerkont", "Gate_Komf_1", 0),        # Door open, driver
      # TODO: locate passenger and rear door states
      ("GK1_Blinker_li", "Gate_Komf_1", 0),         # Left turn signal on
      ("GK1_Blinker_re", "Gate_Komf_1", 0),         # Right turn signal on
      ("Bremsinfo", "Kombi_1", 0),                  # Manual handbrake applied
      ("GRA_Hauptschalt", "GRA_Neu", 0),            # ACC button, on/off
      ("GRA_Abbrechen", "GRA_Neu", 0),              # ACC button, cancel
      ("GRA_Neu_Setzen", "GRA_Neu", 0),             # ACC button, set
      ("GRA_Up_lang", "GRA_Neu", 0),                # ACC button, increase or accel, long press
      ("GRA_Down_lang", "GRA_Neu", 0),              # ACC button, decrease or decel, long press
      ("GRA_Up_kurz", "GRA_Neu", 0),                # ACC button, increase or accel, short press
      ("GRA_Down_kurz", "GRA_Neu", 0),              # ACC button, decrease or decel, short press
      ("GRA_Recall", "GRA_Neu", 0),                 # ACC button, resume
      ("GRA_Zeitluecke", "GRA_Neu", 0),             # ACC button, time gap adj
      ("GRA_Neu_Zaehler", "GRA_Neu", 0),            # ACC button, time gap adj
      ("GRA_Sender", "GRA_Neu", 0),                 # GRA Sender Coding

      ("Bremsdruck", "Bremse_5", 0),  # Brake pressure applied
      ("BR5_Sign_Druck", "Bremse_5", 0),  # Brake pressure applied sign (???)
      ("BR5_Giergeschw", "Bremse_5", 0),  # Absolute yaw rate
      ("BR5_Sta_Gierrate", "Bremse_5", 0),
      ("BR5_Vorzeichen", "Bremse_5", 0),  # Yaw rate sign
      ("BR5_Stillstand", "Bremse_5", 0),
      ("BR5_Sta_Druck", "Bremse_5", 0),
      ("ESP_Rollenmodus_Deaktivieren", "Bremse_5", 0),
      ("ESP_Anforderung_EPB", "Bremse_5", 0),
      ("ESP_Stat_FallBack_eBKV", "Bremse_5", 0),
      ("ESP_Autohold_aktiv", "Bremse_5", 0),
      ("ESP_Autohold_Standby", "Bremse_5", 0),
      ("BR5_Anhi_Sta", "Bremse_5", 0),
      ("BR5_Anhi_akt", "Bremse_5", 0),
      ("BR5_v_Ueberw", "Bremse_5", 0),
      ("BR5_Bremslicht", "Bremse_5", 0),
      ("BR5_Notbremsung", "Bremse_5", 0),
      ("BR5_Fahrer_tritt_ZBR_Schw", "Bremse_5", 0),
      ("BR5_AWV2_Bremsruck", "Bremse_5", 0),
      ("BR5_AWV2_Fehler", "Bremse_5", 0),
      ("BR5_ZT_Rueckk_Umsetz", "Bremse_5", 0),
      ("BR5_ANB_CM_Rueckk_Umsetz", "Bremse_5", 0),
      ("BR5_HDC_bereit", "Bremse_5", 0),
      ("BR5_ECD_Lampe", "Bremse_5", 0),
      ("BR5_Druckgueltig", "Bremse_5", 0),
    ]

    checks = [
      # sig_address, frequency
      ("Bremse_1", 100),          # From J104 ABS/ESP controller
      ("Bremse_3", 100),          # From J104 ABS/ESP controller
      ("Lenkhilfe_3", 100),       # From J500 Steering Assist with integrated sensors
      ("Lenkwinkel_1", 100),      # From J500 Steering Assist with integrated sensors
      ("Motor_3", 100),           # From J623 Engine control module
      ("Airbag_1", 50),           # From J234 Airbag control module
      ("Bremse_5", 50),           # From J104 ABS/ESP controller
      ("GRA_Neu", 50),            # From J??? steering wheel control buttons
      ("Kombi_1", 50),            # From J285 Instrument cluster
      ("Motor_2", 50),            # From J623 Engine control module
      ("Motor_5", 50),            # From J623 Engine control module
      ("Lenkhilfe_2", 20),        # From J500 Steering Assist with integrated sensors
      ("Gate_Komf_1", 10),        # From J533 CAN gateway
    ]

    if CP.transmissionType == TransmissionType.automatic:
      signals += [("Waehlhebelposition__Getriebe_1_", "Getriebe_1", 0)]  # Auto trans gear selector position
      checks += [("Getriebe_1", 100)]  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.manual:
      signals += [("Kupplungsschalter", "Motor_1", 0),  # Clutch switch
                  ("GK1_Rueckfahr", "Gate_Komf_1", 0)]  # Reverse light from BCM
      checks += [("Motor_1", 100)]  # From J623 Engine control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Extended CAN devices other than the camera are here on CANBUS.pt
      signals += PqExtraSignals.fwd_radar_signals
      checks += PqExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += PqExtraSignals.bsm_radar_signals
        checks += PqExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.pq, signals, checks, CANBUS.pt)

  @staticmethod
  def get_mqb_cam_can_parser(CP):
    signals = []
    checks = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        # sig_name, sig_address, default
        ("LDW_SW_Warnung_links", "LDW_02", 0),      # Blind spot in warning mode on left side due to lane departure
        ("LDW_SW_Warnung_rechts", "LDW_02", 0),     # Blind spot in warning mode on right side due to lane departure
        ("LDW_Seite_DLCTLC", "LDW_02", 0),          # Direction of most likely lane departure (left or right)
        ("LDW_DLC", "LDW_02", 0),                   # Lane departure, distance to line crossing
        ("LDW_TLC", "LDW_02", 0),                   # Lane departure, time to line crossing
      ]
      checks += [
        # sig_address, frequency
        ("LDW_02", 10)      # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    # TODO: Re-enable checks enforcement with CP.enableStockCamera
    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.cam, enforce_checks=False)

  @staticmethod
  def get_pq_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("LDW_Lampe_gruen", "LDW_Status", 0),               # Just to check camera for CAN bus validity

      ("AWV_Text", "mAWV", 0),
      ("AWV_1_Freigabe", "mAWV", 0),
      ("AWV_1_Prefill", "mAWV", 0),
      ("AWV_1_Parameter", "mAWV", 0),
      ("AWV_only", "mAWV", 0),
      ("AWV_CityANB_Auspraegung", "mAWV", 0),
      ("AWV_Halten", "mAWV", 0),
      ("ANB_Teilbremsung_Freigabe", "mAWV", 0),
      ("AWV_2_Status", "mAWV", 0),
      ("AWV_2_Fehler", "mAWV", 0),
      ("AWV_2_SU_Warnzeit", "mAWV", 0),
      ("AWV_2_SU_Bremsruck", "mAWV", 0),
      ("AWV_2_SU_Gong", "mAWV", 0),
      ("AWV_2_SU_Lampe", "mAWV", 0),
      ("AWV_2_Umfeldwarn", "mAWV", 0),
      ("AWV_2_Freigabe", "mAWV", 0),
      ("AWV_2_Ruckprofil", "mAWV", 0),
      ("AWV_2_Warnton", "mAWV", 0),
      ("AWV_2_Warnsymbol", "mAWV", 0),
      ("AWV_Infoton", "mAWV", 0),
      ("AWV_2_Gurtstraffer", "mAWV", 0),
      ("AWV_Konfiguration_Menueanf", "mAWV", 0),
      ("AWV_Konfiguration_Vorw_Menueanf", "mAWV", 0),
      ("AWV_Konfiguration_Status", "mAWV", 0),
      ("AWV_Konfiguration_Vorw_Status", "mAWV", 0),
      ("AWV_2_Abstandswarnung", "mAWV", 0),
      ("ANB_Zielbremsung_Freigabe", "mAWV", 0),
      ("ANB_CM_Anforderung", "mAWV", 0),
      ("ANB_Ziel_Teilbrems_Verz_Anf", "mAWV", 0),
      ("ACS_Typ_ACC", "ACC_System", 0)
    ]

    checks = [
      # sig_address, frequency
      #("LDW_Status", 20)        # From R242 Driver assistance camera
      ("mAWV", 50),
      ("ACC_System", 50)
    ]

    if CP.enableGasInterceptor:
      signals += [("INTERCEPTOR_GAS", "GAS_SENSOR", 0), ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0)]
      checks += [("GAS_SENSOR", 50)]

    if CP.networkLocation == NetworkLocation.gateway:
      # Extended CAN devices other than the camera are here on CANBUS.cam
      #signals += PqExtraSignals.fwd_radar_signals
      #checks += PqExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += PqExtraSignals.bsm_radar_signals
        checks += PqExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.pq, signals, checks, CANBUS.cam)

class MqbExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACC_Wunschgeschw", "ACC_02", 0),              # ACC set speed
    ("AWV2_Freigabe", "ACC_10", 0),                 # FCW brake jerk release
    ("ANB_Teilbremsung_Freigabe", "ACC_10", 0),     # AEB partial braking release
    ("ANB_Zielbremsung_Freigabe", "ACC_10", 0),     # AEB target braking release
  ]
  fwd_radar_checks = [
    ("ACC_10", 50),                                 # From J428 ACC radar control module
    ("ACC_02", 17),                                 # From J428 ACC radar control module
  ]
  bsm_radar_signals = [
    ("SWA_Infostufe_SWA_li", "SWA_01", 0),          # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_01", 0),            # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_01", 0),          # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_01", 0),            # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_01", 20),                                 # From J1086 Lane Change Assist
  ]

class PqExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACA_Codierung", "ACC_GRA_Anzeige", 0),
    ("ACA_StaACC", "ACC_GRA_Anzeige", 0),           # ACC drivetrain coordinator status
    ("ACA_V_Wunsch", "ACC_GRA_Anzeige", 0),         # ACC set speed
  ]
  fwd_radar_checks = [
    ("ACC_GRA_Anzeige", 25),                        # From J428 ACC radar control module
  ]
  bsm_radar_signals = [
    ("SWA_Infostufe_SWA_li", "SWA_1", 0),           # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_1", 0),             # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_1", 0),           # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_1", 0),             # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_1", 20),                                  # From J1086 Lane Change Assist
  ]

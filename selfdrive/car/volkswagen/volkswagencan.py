# CAN controls for MQB platform Volkswagen, Audi, Skoda and SEAT.
# PQ35/PQ46/NMS, and any future MLB, to come later.

from selfdrive.car import crc8_pedal

def create_mqb_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "SET_ME_0X3": 0x3,
    "Assist_Torque": abs(apply_steer),
    "Assist_Requested": lkas_enabled,
    "Assist_VZ": 1 if apply_steer < 0 else 0,
    "HCA_Available": 1,
    "HCA_Standby": not lkas_enabled,
    "HCA_Active": lkas_enabled,
    "SET_ME_0XFE": 0xFE,
    "SET_ME_0X07": 0x07,
  }
  return packer.make_can_msg("HCA_01", bus, values, idx)

def create_mqb_hud_control(packer, bus, enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                           ldw_stock_values, left_lane_depart, right_lane_depart):
  # Lane color reference:
  # 0 (LKAS disabled) - off
  # 1 (LKAS enabled, no lane detected) - dark gray
  # 2 (LKAS enabled, lane detected) - light gray on VW, green or white on Audi depending on year or virtual cockpit.  On a color MFD on a 2015 A3 TDI it is white, virtual cockpit on a 2018 A3 e-Tron its green.
  # 3 (LKAS enabled, lane departure detected) - white on VW, red on Audi

  values = ldw_stock_values.copy()
  values.update({
    "LDW_Status_LED_gelb": 1 if enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if left_lane_depart else 1 + left_lane_visible,
    "LDW_Lernmodus_rechts": 3 if right_lane_depart else 1 + right_lane_visible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)

def create_mqb_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Hauptschalter": CS.graHauptschalter,
    "GRA_Abbrechen": buttonStatesToSend["cancel"],
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"],
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
    "GRA_Codierung": 2,
    "GRA_Tip_Stufe_2": CS.graTipStufe2,
    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }
  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)

def create_pq_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "HCA_Zaehler": idx,
    "LM_Offset": abs(apply_steer),
    "LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_Status": 7 if (lkas_enabled and apply_steer != 0) else 3,
    "Vib_Freq": 16,
  }

  dat = packer.make_can_msg("HCA_1", bus, values)[2]
  values["HCA_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("HCA_1", bus, values)
def create_pq_acc_control(packer, bus, idx, enabled, accel_req, stopping, acc_type):
  values = {
    "ACS_Zaehler": idx,                             # ADR Counter
    "ACS_Sta_ADR": 1 if enabled else 2,             # ADR Status (0 inactive / 1 active / 2 passive / 3 irreversible fault)
    "ACS_StSt_Info": 1,                             # StartStop request (0 Allow stop / 1 Engine start not needed / 2 Engine start / 3 failure)
    "ACS_MomEingriff": 0,                           # Torque intervention (Prevent Whiplash?) (0 Allow Whiplash / 1 Don't allow whiplash)
    "ACS_Typ_ACC": acc_type,                        # ADR Type (0 Basic ACC / 1 ACC Follow2Stop / 2 unused / 3 ACC not installed)
    "ACS_FreigSollB": 1 if enabled else 0,          # Activation of ACS_Sollbeschl (0 not allowed / 1 allowed)
    "ACS_Sollbeschl": accel_req if enabled else 10.23,    # Acceleration Request (2046(10.23) ADR Inactive / 2047(10.235) Fault)
    "ACS_Anhaltewunsch": 1 if stopping else 0,      # Stopping Request (0 No stop request / 1 Vehicle stopping)
    "ACS_zul_Regelabw": 0.4 if enabled else 1.27,     # Allowed request deviation (254 ADR not active / 255 Fault) (Use this offset if more comfort can be achieved)
    "ACS_max_AendGrad": 1 if enabled else 0,      # Allowed gradient changes (0 Neutral value, 254 Neutral value, 255 Fault) (Unknown)
  }

  dat = packer.make_can_msg("ACC_System", bus, values)[2]
  values["ACS_Checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("ACC_System", bus, values)

def create_pq_aca_control(packer, bus , idx, enabled, set_speed, metric, acc_coding):
  values = {
    "ACA_StaACC": 3 if enabled else 2,         # ADR Status in cluster (0 Switch Off / 2 ACC Pasive / 3 ACC Active / 4 ACC in Background / 6 ACC reversible off / 7 ACC irreversible off)
    "ACA_Fahrerhinw": 0,                      # ADR Driver Warning (Max Limit reached) (0 Off / 1 On)
    "ACA_AnzDisplay": 1 if enabled else 0,     # ADR Display Status (0 No Display / 1 Display)
    "ACA_Zeitluecke": 1,                      # Display set time gap (0 Not defined / 1-15 different distances for display in cluster)
    "ACA_V_Wunsch": set_speed,                # Display set speed (255 Not set (yet) / 0-254 Actual KM/h setpoint)
    "ACA_kmh_mph": 0 if metric else 1,        # Display KMh or Mph (0 Km/h / 1 Mph)
    "ACA_Akustik1": 0,                        # Soft cluster gong (0 Off / 1 On)
    "ACA_Akustik2": 0,                        # Hard cluster buzzer (0 Off / 1 On)
    "ACA_PrioDisp": 1,                        # Display Priority (0 High Prio / 1 Prio / 2 Low Prio / 3 No Request)
    "ACA_gemZeitl": 0,                        # Avarage Follow distance (0 No lead / 1-15 Actual avarge distance)
    "ACA_Codierung": acc_coding,                       # Coding (0 ACC / 1 GRA)
    "ACA_Zaehler": idx,                       # Counter
  }

  dat = packer.make_can_msg("ACC_GRA_Anzeige", bus, values)[2]
  values["ACA_Checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("ACC_GRA_Anzeige", bus, values)


def create_pq_awv_control(packer, bus, idx, led_orange, led_green, mAWV):
  Fehler = mAWV["AWV_2_Fehler"]

  mAWV["AWV_Zaehler"] = idx

  mAWV["AWV_2_Status"] = 1 if led_green else 0
  mAWV["AWV_2_Fehler"] = 1 if led_orange else 0

  mAWV["AWV_only"] = 0

  if Fehler:
    mAWV["AWV_2_Fehler"] = 1

  dat = packer.make_can_msg("mAWV", bus, mAWV)[2]
  mAWV["AWV_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("mAWV", bus, mAWV)

def create_pedal_control(packer, bus, apply_gas, idx):
  # Common gas pedal msg generator
  enable = apply_gas > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    if (apply_gas < 227):
      apply_gas = 227
    values["GAS_COMMAND"] = apply_gas
    values["GAS_COMMAND2"] = apply_gas

  dat = packer.make_can_msg("GAS_COMMAND", bus, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", bus, values)

def create_pq_bremse8_control(packer, bus, idx, bremse8):

  bremse8["BR8_Zaehler"] = idx
  dat = packer.make_can_msg("Bremse_8", bus, bremse8)[2]
  bremse8["BR8_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]

  return packer.make_can_msg("Bremse_8", bus, bremse8)

def create_pq_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                          ldw_lane_warning_left, ldw_lane_warning_right, ldw_side_dlc_tlc, ldw_dlc, ldw_tlc,
                          standstill, left_lane_depart, right_lane_depart):
  if hca_enabled:
    left_lane_hud = 3 if left_lane_depart else 1 + left_lane_visible
    right_lane_hud = 3 if right_lane_depart else 1 + right_lane_visible
  else:
    left_lane_hud = 0
    right_lane_hud = 0

  values = {
    "Right_Lane_Status": right_lane_hud,
    "Left_Lane_Status": left_lane_hud,
    "SET_ME_X1": 1,
    "Kombi_Lamp_Orange": 1 if hca_enabled and steering_pressed else 0,
    "Kombi_Lamp_Green": 1 if hca_enabled and not steering_pressed else 0,
  }
  return packer.make_can_msg("LDW_1", bus, values)

def create_pq_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Neu_Zaehler": idx,
    "GRA_Sender": CS.graSenderCoding,
    "GRA_Abbrechen": 1 if (buttonStatesToSend["cancel"] or CS.buttonStates["cancel"]) else 0,
    "GRA_Hauptschalt": CS.graHauptschalter,
  }

  dat = packer.make_can_msg("GRA_Neu", bus, values)[2]
  values["GRA_Checksum"] = dat[1] ^ dat[2] ^ dat[3]
  return packer.make_can_msg("GRA_Neu", bus, values)

def create_pq_epb_control(packer, bus, brake_req, enable, idx):
  values = {
    "EP1_Zaehler": idx,
    "EP1_Failure_Sta": 0,
    "EP1_Sta_EPB": 0,
    "EP1_Spannkraft": 0,
    "EP1_Schalterinfo": 0,
    "EP1_Verzoegerung": brake_req,                    #Brake request in m/s2
    "EP1_Freigabe_Ver": 1 if enable else 0,           #Allow braking pressure to build.
    "EP1_Fkt_Lampe": 0,
    "EP1_Bremslicht": 1 if enable else 0,             #Enable brake lights
    "EP1_HydrHalten": 1 if enable else 0,             #Disengage DSG
    "EP1_AutoHold_active": 1 if enable else 0          #Disengage DSG
  }

  dat = packer.make_can_msg("mEPB_1", bus, values)[2]
  values["EP1_Checksum"] = dat[0] ^ dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6]
  return packer.make_can_msg("mEPB_1", bus, values)

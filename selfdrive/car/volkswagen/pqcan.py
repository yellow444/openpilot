from selfdrive.car import crc8_pedal

def acc_control_value(main_switch_on, acc_faulted, long_active):
  if long_active:
    acc_control = 1
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    hud_status = 6
  elif long_active:
    hud_status = 3
  elif main_switch_on:
    hud_status = 2
  else:
    hud_status = 0

  return hud_status

def create_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "HCA_Zaehler": idx,
    "LM_Offset": abs(apply_steer),
    "LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_Status": 5 if (lkas_enabled and apply_steer != 0) else 3,
    "Vib_Freq": 16,
  }

  dat = packer.make_can_msg("HCA_1", bus, values)[2]
  values["HCA_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("HCA_1", bus, values)

def create_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Neu_Zaehler": idx,
    "GRA_Sender": CS.graSenderCoding,
    "GRA_Abbrechen": 1 if (buttonStatesToSend["cancel"] or CS.buttonStates["cancel"]) else 0,
    "GRA_Hauptschalt": CS.graHauptschalter,
  }

  dat = packer.make_can_msg("GRA_Neu", bus, values)[2]
  values["GRA_Checksum"] = dat[1] ^ dat[2] ^ dat[3]
  return packer.make_can_msg("GRA_Neu", bus, values)

def create_lka_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                          ldw_lane_warning_left, ldw_lane_warning_right, ldw_side_dlc_tlc, ldw_dlc, ldw_tlc,
                          standstill, left_lane_depart, right_lane_depart):
  if hca_enabled:
    left_lane_hud = 3 if left_lane_depart else 1 + left_lane_visible
    right_lane_hud = 3 if right_lane_depart else 1 + right_lane_visible
  else:
    left_lane_hud = 0
    right_lane_hud = 0

  values = {
    "LDW_Lampe_gelb": 1 if hca_enabled and steering_pressed else 0,
    "LDW_Lampe_gruen": 1 if hca_enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": left_lane_hud,
    "LDW_Lernmodus_rechts": right_lane_hud,
    "LDW_Textbits": hud_alert,
  }

  return packer.make_can_msg("LDW_Status", bus, values)

def create_acc_hud_control(packer, bus , idx, acc_hud_status, set_speed, metric, lead_distance):
  values = {
    "ACA_Zaehler": idx,
    "ACA_StaACC": acc_hud_status,
    "ACA_Zeitluecke": 2,
    "ACA_V_Wunsch": set_speed,
    "ACA_gemZeitl": lead_distance,
    "ACA_PrioDisp": 3,
    "ACA_kmh_mph": 0 if metric else 1,
    # TODO: restore dynamic pop-to-foreground/highlight behavior with ACA_PrioDisp and ACA_AnzDisplay
    # TODO: ACA_kmh_mph handling probably needed to resolve rounding errors in displayed setpoint
  }

  dat = packer.make_can_msg("ACC_GRA_Anzeige", bus, values)[2]
  values["ACA_Checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("ACC_GRA_Anzeige", bus, values)

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

def create_awv_control(packer, bus, idx, led_orange, led_green, mAWV):
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

def create_acc_accel_control(packer, bus, idx, acc_type, acc_enabled, accel, acc_control, stopping, starting):
  values = {
    "ACS_Zaehler": idx,
    "ACS_Sta_ADR": acc_control,
    "ACS_StSt_Info": acc_enabled,
    "ACS_Typ_ACC": acc_type,
    "ACS_Anhaltewunsch": acc_type == 1 and stopping,
    "ACS_FreigSollB": acc_enabled,
    "ACS_Sollbeschl": accel if acc_enabled else 3.01,
    "ACS_zul_Regelabw": 0.2 if acc_enabled else 1.27,
    "ACS_max_AendGrad": 3.0 if acc_enabled else 5.08,
  }

  dat = packer.make_can_msg("ACC_System", bus, values)[2]
  values["ACS_Checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("ACC_System", bus, values)

def create_epb_control(packer, bus, brake_req, enable, idx):
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

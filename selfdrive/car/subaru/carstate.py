import copy
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.car.subaru.values import CAR, DBC, STEER_THRESHOLD

def get_powertrain_can_parser(CP):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("Steer_Torque_Sensor", "Steering_Torque", 0),
    ("Steering_Angle", "Steering_Torque", 0),
    ("Cruise_On", "CruiseControl", 0),
    ("Cruise_Activated", "CruiseControl", 0),
    ("Brake_Pedal", "Brake_Pedal", 0),
    ("Throttle_Pedal", "Throttle", 0),
    ("LEFT_BLINKER", "Dashlights", 0),
    ("RIGHT_BLINKER", "Dashlights", 0),
    ("SEATBELT_FL", "Dashlights", 0),
    ("FL", "Wheel_Speeds", 0),
    ("FR", "Wheel_Speeds", 0),
    ("RL", "Wheel_Speeds", 0),
    ("RR", "Wheel_Speeds", 0),
    ("DOOR_OPEN_FR", "BodyInfo", 1),
    ("DOOR_OPEN_FL", "BodyInfo", 1),
    ("DOOR_OPEN_RR", "BodyInfo", 1),
    ("DOOR_OPEN_RL", "BodyInfo", 1),
  ]

  checks = [
    # sig_address, frequency
    ("Dashlights", 10),
    ("Wheel_Speeds", 50),
    ("Steering_Torque", 50),
  ]

  if CP.carFingerprint == CAR.IMPREZA:
    signals += [
      ("Units", "Dash_State", 1),
    ]
    checks += [
      ("BodyInfo", 10),
      ("CruiseControl", 20),
    ]

  if CP.carFingerprint in [CAR.OUTBACK, CAR.LEGACY, CAR.FORESTER]:
    signals += [
      ("LKA_Lockout", "Steering_Torque", 0),
    ]
    checks += [
      ("CruiseControl", 50),
    ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

def get_camera_can_parser(CP):
  signals = [
  ]

  checks = [
  ]

  if CP.carFingerprint == CAR.IMPREZA:
    signals += [
      ("Cruise_Set_Speed", "ES_DashStatus", 0),
      ("Counter", "ES_Distance", 0),
      ("Signal1", "ES_Distance", 0),
      ("Signal2", "ES_Distance", 0),
      ("Main", "ES_Distance", 0),
      ("Signal3", "ES_Distance", 0),

      ("Checksum", "ES_LKAS_State", 0),
      ("Counter", "ES_LKAS_State", 0),
      ("Keep_Hands_On_Wheel", "ES_LKAS_State", 0),
      ("Empty_Box", "ES_LKAS_State", 0),
      ("Signal1", "ES_LKAS_State", 0),
      ("LKAS_ACTIVE", "ES_LKAS_State", 0),
      ("Signal2", "ES_LKAS_State", 0),
      ("Backward_Speed_Limit_Menu", "ES_LKAS_State", 0),
      ("LKAS_ENABLE_3", "ES_LKAS_State", 0),
      ("Signal3", "ES_LKAS_State", 0),
      ("LKAS_ENABLE_2", "ES_LKAS_State", 0),
      ("Signal4", "ES_LKAS_State", 0),
      ("LKAS_Left_Line_Visible", "ES_LKAS_State", 0),
      ("Signal6", "ES_LKAS_State", 0),
      ("LKAS_Right_Line_Visible", "ES_LKAS_State", 0),
      ("Signal7", "ES_LKAS_State", 0),
      ("FCW_Cont_Beep", "ES_LKAS_State", 0),
      ("FCW_Repeated_Beep", "ES_LKAS_State", 0),
      ("Throttle_Management_Activated", "ES_LKAS_State", 0),
      ("Traffic_light_Ahead", "ES_LKAS_State", 0),
      ("Right_Depart", "ES_LKAS_State", 0),
      ("Signal5", "ES_LKAS_State", 0),
    ]
    checks += [
      ("ES_DashStatus", 10),
    ]
  if CP.carFingerprint in [CAR.OUTBACK, CAR.LEGACY, CAR.FORESTER]:
    signals += [
      ("Brake_On", "ES_CruiseThrottle", 0),
      ("Button", "ES_CruiseThrottle", 0),
      ("CloseDistance", "ES_CruiseThrottle", 0),
      ("Counter", "ES_CruiseThrottle", 0),
      ("Cruise_Activatedish", "ES_CruiseThrottle", 0),
      ("DistanceSwap", "ES_CruiseThrottle", 0),
      ("ES_Error", "ES_CruiseThrottle", 0),
      ("NEW_SIGNAL_1", "ES_CruiseThrottle", 0),
      ("Unknown", "ES_CruiseThrottle", 0),
      ("SET_0_1", "ES_CruiseThrottle", 0),
      ("SET_0_2", "ES_CruiseThrottle", 0),
      ("SET_0_3", "ES_CruiseThrottle", 0),
      ("SET_0_4", "ES_CruiseThrottle", 0),
      ("SET_1", "ES_CruiseThrottle", 0),
      ("SET_2", "ES_CruiseThrottle", 0),
      ("NEW_SIGNAL_9", "ES_CruiseThrottle", 0),
      ("Standstill", "ES_CruiseThrottle", 0),
      ("Standstill_2", "ES_CruiseThrottle", 0),
      ("Throttle_Cruise", "ES_CruiseThrottle", 0),
    ]

  if CP.carFingerprint in [CAR.OUTBACK, CAR.LEGACY]:
    signals += [
      ("Not_Ready_Startup", "ES_DashStatus", 0),
      ("Cruise_Set_Speed", "ES_DashStatus", 0),
    ]

    checks += [
      ("ES_DashStatus", 10),
    ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # initialize can parser
    self.left_blinker_cnt = 0
    self.right_blinker_cnt = 0

  def update(self, cp, cp_cam):

    self.pedal_gas = cp.vl["Throttle"]['Throttle_Pedal']
    self.brake_pressure = cp.vl["Brake_Pedal"]['Brake_Pedal']
    self.user_gas_pressed = self.pedal_gas > 0
    self.brake_pressed = self.brake_pressure > 0
    self.brake_lights = bool(self.brake_pressed)

    self.v_wheel_fl = cp.vl["Wheel_Speeds"]['FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = cp.vl["Wheel_Speeds"]['FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = cp.vl["Wheel_Speeds"]['RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = cp.vl["Wheel_Speeds"]['RR'] * CV.KPH_TO_MS

    self.v_cruise_pcm = cp_cam.vl["ES_DashStatus"]['Cruise_Set_Speed']

    self.v_ego_raw = (self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.
    # Kalman filter, even though Subaru raw wheel speed is heaviliy filtered by default
    self.v_ego, self.a_ego = self.update_speed_kf(self.v_ego_raw)

    self.standstill = self.v_ego_raw < 0.01

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    # continuous blinker signals for assisted lane change
    self.left_blinker_cnt = 50 if cp.vl["Dashlights"]['LEFT_BLINKER'] else max(self.left_blinker_cnt - 1, 0)
    self.left_blinker_on = self.left_blinker_cnt > 0

    self.right_blinker_cnt = 50 if cp.vl["Dashlights"]['RIGHT_BLINKER'] else max(self.right_blinker_cnt - 1, 0)
    self.right_blinker_on = self.right_blinker_cnt > 0

    self.steer_torque_driver = cp.vl["Steering_Torque"]['Steer_Torque_Sensor']
    self.acc_active = cp.vl["CruiseControl"]['Cruise_Activated']
    self.main_on = cp.vl["CruiseControl"]['Cruise_On']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.car_fingerprint]
    self.angle_steers = cp.vl["Steering_Torque"]['Steering_Angle']
    self.door_open = any([cp.vl["BodyInfo"]['DOOR_OPEN_RR'],
      cp.vl["BodyInfo"]['DOOR_OPEN_RL'],
      cp.vl["BodyInfo"]['DOOR_OPEN_FR'],
      cp.vl["BodyInfo"]['DOOR_OPEN_FL']])

    if self.car_fingerprint == CAR.IMPREZA:
      self.seatbelt_unlatched = cp.vl["Dashlights"]['SEATBELT_FL'] == 1
      self.v_cruise_pcm = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"] * CV.MPH_TO_KPH
      self.steer_not_allowed = 0
      self.es_distance_msg = copy.copy(cp_cam.vl["ES_Distance"])
      self.es_lkas_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])
      # 1 = imperial, 6 = metric
      if cp.vl["Dash_State"]['Units'] == 1:
        self.v_cruise_pcm *= CV.MPH_TO_KPH

    if self.car_fingerprint in [CAR.OUTBACK, CAR.LEGACY, CAR.FORESTER]:
      self.seatbelt_unlatched = False # FIXME: stock ACC disengages on unlatch so this is fine for now, signal has not yet been found
      self.steer_not_allowed = cp.vl["Steering_Torque"]["LKA_Lockout"]
      self.button = cp_cam.vl["ES_CruiseThrottle"]["Button"]
      self.brake_hold = cp_cam.vl["ES_CruiseThrottle"]["Standstill"]
      self.close_distance = cp_cam.vl["ES_CruiseThrottle"]["CloseDistance"]
      self.es_accel_msg = copy.copy(cp_cam.vl["ES_CruiseThrottle"])
      
    if self.car_fingerprint in [CAR.FORESTER]:
      self.v_cruise_pcm = 0
      self.ready = True

    if self.car_fingerprint in [CAR.OUTBACK, CAR.LEGACY]:
      self.v_cruise_pcm = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"]
      if cp.vl["Dash_State"]['Units'] == 1:
        self.v_cruise_pcm *= CV.MPH_TO_KPH
      self.ready = not cp_cam.vl["ES_DashStatus"]["Not_Ready_Startup"]

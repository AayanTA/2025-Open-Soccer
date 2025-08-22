# sim_hardware.py — compatibility shim for simulator

import math

_address_to_sim = {}

def bind_address_to_robot(i2c_address, sim_robot):
    _address_to_sim[i2c_address] = sim_robot

def get_sim_robot(i2c_address):
    return _address_to_sim.get(i2c_address, None)

class PowerfulBLDCDriver:
    def __init__(self, i2c, address):
        self._addr = address
        self._sim_robot = None
        self._last_speed = 0
    def _ensure_bound(self):
        if self._sim_robot is None:
            self._sim_robot = get_sim_robot(self._addr)
    def set_speed(self, speed_int):
        self._ensure_bound()
        if self._sim_robot is None: return
        MAX = 60000000.0
        normalized = max(-1.0, min(1.0, float(speed_int) / MAX))
        self._sim_robot.set_wheel_from_address(self._addr, normalized)
    # stubbed calibration/config methods...
    def set_current_limit_foc(self, v): pass
    def set_id_pid_constants(self, a,b): pass
    def set_iq_pid_constants(self, a,b): pass
    def set_speed_pid_constants(self, a,b,c): pass
    def set_position_pid_constants(self, a,b,c): pass
    def set_position_region_boundary(self, a): pass
    def set_speed_limit(self, s): pass
    def configure_operating_mode_and_sensor(self, a,b): pass
    def configure_command_mode(self, m): pass
    def set_ELECANGLEOFFSET(self,v): pass
    def set_SINCOSCENTRE(self,v): pass

class BNO08X_I2C:
    def __init__(self,i2c,address=0x74): self._sim_robot=None
    def enable_feature(self,feature): pass
    @property
    def game_rotation_vector(self):
        if self._sim_robot is None: return None
        yaw_deg = self._sim_robot.state.theta
        yaw = math.radians(yaw_deg)
        return (0.0,0.0,math.sin(yaw/2.0),math.cos(yaw/2.0),3)

class SimAPI:
    def __init__(self,sim_robot): self.sim_robot=sim_robot
    def setMotorSpeeds(self,translationAngleDeg,translationSpeedNormalized,rotation):
        arr=self.sim_robot.cmd_to_wheel_norms(translationAngleDeg,translationSpeedNormalized,rotation)
        self.sim_robot.apply_wheel_norms(arr)
    def spinAround(self,norm_speed):
        m=norm_speed
        self.sim_robot.apply_wheel_norms([m,m,-m,-m])
    def get_relative_yaw(self): return self.sim_robot.get_relative_yaw()

def bind_IMU_to_robot(bno_obj,sim_robot): setattr(bno_obj,"_sim_robot",sim_robot)

class board: SCL=None; SDA=None
class busio:
    class I2C:
        def __init__(self, scl, sda, frequency=None):
            pass

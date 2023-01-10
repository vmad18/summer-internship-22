import rospy

class ControlLoop(object):

    def __init__(self, err, p:float = 0.0, i:float = 0.0, d:float = 0.0):
        
        self.error_func:function = err

        self.kP:float = p
        self.kI:float = i
        self.kD:float = d
        
        self.te:float = 0.0

        self.pt:int = rospy.Time().now().nsecs
        self.pe:float = 0.0


    def _dt(self)-> int:
        now:int = rospy.Time().now().nsecs
        dt:int = now - self.pt
        self.pt = now
        
        return dt/1e6


    def _dedt(self, error:float)-> float:
        #   rospy.loginfo(f"{error} {self.pe}")
        de:float = error - self.pe
        self.pe = error 

        dt:float = self._dt()
        if dt == 0: return 0
        return de/dt


    def execute(self, setpoint:float) -> float:
        error:float = self.error_func(setpoint)
        self.te += error

        dt = self._dt()

        prop = self.kP*error
        integral = self.kI*dt*self.te
        derivative = self.kD*self._dedt(error)


        #rospy.loginfo(f"ERROR: {error} P: {prop} I: {integral} D:{derivative} {prop + integral+ derivative}")
        return prop + integral + derivative



#kinda pointless right now
class TurnPID(ControlLoop):
    pass
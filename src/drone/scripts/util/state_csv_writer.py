import rospy

class StateCsvLogger:

    def __init__(self, filename):
        self._counter = 0
        self.filename = filename
        with open(self.filename, 'w') as f:
            f.write('timestampns;pitch;roll;yaw;vgx;vgy;vgz;templ;temph;tof;h;bat;baro;time;agx;agy;agz;\r\n')
            f.close()
        rospy.loginfo("Logging State to " + self.filename)

    def log(self, state):
        try:
            if state:
                #write to csv using pandas
                with open(self.filename, 'a') as f:
                    f.write('%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%.2f;%d;%.2f;%.2f;%.2f\r\n' 
                            % (rospy.Time.now().to_nsec(),
                                state.get('pitch'),
                                state.get('roll'),
                                state.get('yaw'),
                                state.get('vgx'),
                                state.get('vgy'),
                                state.get('vgz'),
                                state.get('templ'),
                                state.get('temph'),
                                state.get('tof'),
                                state.get('h'),
                                state.get('bat'),
                                state.get('baro'),
                                state.get('time'),
                                state.get('agx'),
                                state.get('agy'),
                                state.get('agz')
                                ))
                    f.close()
                self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
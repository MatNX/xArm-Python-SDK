namespace XArm.Lite6;

public class Lite6ArmClient : IDisposable
{
    private readonly IArmTransport _transport;
    private readonly TimeSpan _defaultTimeout;
    private readonly UxbusClient _uxbus;

    public Lite6ArmClient(IArmTransport? transport = null, TimeSpan? defaultTimeout = null)
    {
        _transport = transport ?? new TcpArmTransport();
        _defaultTimeout = defaultTimeout ?? TimeSpan.FromSeconds(2);
        _uxbus = new UxbusClient(_transport);
    }

    public bool IsConnected => _transport.IsConnected;

    public void Connect(string host, int port = 502, TimeSpan? timeout = null)
    {
        _transport.Connect(host, port, timeout ?? _defaultTimeout);
    }

    public void Disconnect()
    {
        _transport.Disconnect();
    }

    public void MotionEnable(bool enable, int servoId = 8, TimeSpan? timeout = null)
    {
        _uxbus.MotionEnable(servoId, enable, timeout ?? _defaultTimeout);
    }

    public void EnableMotion()
    {
        MotionEnable(true);
    }

    public void DisableMotion()
    {
        MotionEnable(false);
    }

    public void SetMode(int mode, int? detectionParam = null, TimeSpan? timeout = null)
    {
        _uxbus.SetMode(mode, detectionParam, timeout ?? _defaultTimeout);
    }

    public void SetState(int state, TimeSpan? timeout = null)
    {
        _uxbus.SetState(state, timeout ?? _defaultTimeout);
    }

    public void CleanError(TimeSpan? timeout = null)
    {
        _uxbus.CleanError(timeout ?? _defaultTimeout);
    }

    public void CleanWarning(TimeSpan? timeout = null)
    {
        _uxbus.CleanWarning(timeout ?? _defaultTimeout);
    }

    public int GetCommandCount(TimeSpan? timeout = null)
    {
        return _uxbus.GetCommandCount(timeout ?? _defaultTimeout);
    }

    public void MoveJoints(JointPositions joints, double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
    {
        var payload = new[]
        {
            (float)joints.J1,
            (float)joints.J2,
            (float)joints.J3,
            (float)joints.J4,
            (float)joints.J5,
            (float)joints.J6,
            0f
        };

        _uxbus.MoveJoint(payload, (float)speed, (float)acceleration, (float)time, timeout ?? _defaultTimeout);
    }

    public void MoveLinear(Pose pose, double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
    {
        var payload = new[]
        {
            (float)pose.X,
            (float)pose.Y,
            (float)pose.Z,
            (float)pose.Roll,
            (float)pose.Pitch,
            (float)pose.Yaw
        };

        _uxbus.MoveLine(payload, (float)speed, (float)acceleration, (float)time, timeout ?? _defaultTimeout);
    }

    public void MoveHome(double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
    {
        _uxbus.MoveHome((float)speed, (float)acceleration, (float)time, timeout ?? _defaultTimeout);
    }

    public JointPositions GetJointPositions()
    {
        var joints = _uxbus.GetJointPositions(_defaultTimeout);
        return new JointPositions(
            joints.Length > 0 ? joints[0] : 0,
            joints.Length > 1 ? joints[1] : 0,
            joints.Length > 2 ? joints[2] : 0,
            joints.Length > 3 ? joints[3] : 0,
            joints.Length > 4 ? joints[4] : 0,
            joints.Length > 5 ? joints[5] : 0);
    }

    public Pose GetPose()
    {
        var pose = _uxbus.GetTcpPose(_defaultTimeout);
        return new Pose(
            pose.Length > 0 ? pose[0] : 0,
            pose.Length > 1 ? pose[1] : 0,
            pose.Length > 2 ? pose[2] : 0,
            pose.Length > 3 ? pose[3] : 0,
            pose.Length > 4 ? pose[4] : 0,
            pose.Length > 5 ? pose[5] : 0);
    }

    public int GetState(TimeSpan? timeout = null)
    {
        return _uxbus.GetState(timeout ?? _defaultTimeout);
    }

    public (int ErrorCode, int WarningCode) GetErrorWarning(TimeSpan? timeout = null)
    {
        return _uxbus.GetErrorWarning(timeout ?? _defaultTimeout);
    }

    public ArmStatus GetStatus()
    {
        var state = GetState();
        var (error, warning) = GetErrorWarning();
        return new ArmStatus(
            IsConnected,
            state,
            error,
            warning);
    }

    public string GetFirmwareVersion()
    {
        return _uxbus.GetVersion(_defaultTimeout);
    }

    public string GetRobotSerialNumber()
    {
        return _uxbus.GetRobotSerialNumber(_defaultTimeout);
    }

    public void Dispose()
    {
        _transport.Dispose();
    }
}

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

    public void SetBrake(int axisId, bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetBrake(axisId, enable, timeout ?? _defaultTimeout);
    }

    public void SetReportTorqueOrCurrent(bool reportCurrent, TimeSpan? timeout = null)
    {
        _uxbus.SetReportTorqueOrCurrent(reportCurrent, timeout ?? _defaultTimeout);
    }

    public bool GetReportTorqueOrCurrent(TimeSpan? timeout = null)
    {
        var mode = _uxbus.GetReportTorqueOrCurrent(timeout ?? _defaultTimeout);
        return mode != 0;
    }

    public void SetCartesianVelocityContinuous(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetCartesianVelocityContinuous(enable, timeout ?? _defaultTimeout);
    }

    public void SetReducedMode(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetReducedMode(enable, timeout ?? _defaultTimeout);
    }

    public bool GetReducedMode(TimeSpan? timeout = null)
    {
        var mode = _uxbus.GetReducedMode(timeout ?? _defaultTimeout);
        return mode != 0;
    }

    public void SetReducedLineSpeed(double speed, TimeSpan? timeout = null)
    {
        _uxbus.SetReducedLineSpeed((float)speed, timeout ?? _defaultTimeout);
    }

    public void SetReducedJointSpeed(double speed, TimeSpan? timeout = null)
    {
        _uxbus.SetReducedJointSpeed((float)speed, timeout ?? _defaultTimeout);
    }

    public ReducedState GetReducedState(bool includeJointRanges = false, TimeSpan? timeout = null)
    {
        return _uxbus.GetReducedState(includeJointRanges, timeout ?? _defaultTimeout);
    }

    public void SetReducedJointRange(double[] ranges, TimeSpan? timeout = null)
    {
        var payload = new float[Math.Min(ranges.Length, 14)];
        for (var i = 0; i < payload.Length; i++)
        {
            payload[i] = (float)ranges[i];
        }

        _uxbus.SetReducedJointRange(payload, timeout ?? _defaultTimeout);
    }

    public void SetFenseOn(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetFenseOn(enable, timeout ?? _defaultTimeout);
    }

    public void SetCollisionRebound(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetCollisionRebound(enable, timeout ?? _defaultTimeout);
    }

    public void SetXyzLimits(int[] xyzLimits, TimeSpan? timeout = null)
    {
        _uxbus.SetXyzLimits(xyzLimits, timeout ?? _defaultTimeout);
    }

    public void SetTimer(int secondsLater, int timerId, int functionCode, int param1 = 0, int param2 = 0, TimeSpan? timeout = null)
    {
        _uxbus.SetTimer(secondsLater, timerId, functionCode, param1, param2, timeout ?? _defaultTimeout);
    }

    public void CancelTimer(int timerId, TimeSpan? timeout = null)
    {
        _uxbus.CancelTimer(timerId, timeout ?? _defaultTimeout);
    }

    public void SetWorldOffset(Pose offset, TimeSpan? timeout = null)
    {
        var payload = new[]
        {
            (float)offset.X,
            (float)offset.Y,
            (float)offset.Z,
            (float)offset.Roll,
            (float)offset.Pitch,
            (float)offset.Yaw
        };

        _uxbus.SetWorldOffset(payload, timeout ?? _defaultTimeout);
    }

    public void CounterReset(TimeSpan? timeout = null)
    {
        _uxbus.CounterReset(timeout ?? _defaultTimeout);
    }

    public void CounterPlus(TimeSpan? timeout = null)
    {
        _uxbus.CounterPlus(timeout ?? _defaultTimeout);
    }

    public void SetAllowApproxMotion(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetAllowApproxMotion(enable, timeout ?? _defaultTimeout);
    }

    public bool GetAllowApproxMotion(TimeSpan? timeout = null)
    {
        var mode = _uxbus.GetAllowApproxMotion(timeout ?? _defaultTimeout);
        return mode != 0;
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

    public void MoveServoJoints(JointPositions joints, double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
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

        _uxbus.MoveServoJoint(payload, (float)speed, (float)acceleration, (float)time, timeout ?? _defaultTimeout);
    }

    public void MoveServoLinear(Pose pose, double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
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

        _uxbus.MoveServoCartesian(payload, (float)speed, (float)acceleration, (float)time, timeout ?? _defaultTimeout);
    }

    public void MoveHome(double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
    {
        _uxbus.MoveHome((float)speed, (float)acceleration, (float)time, timeout ?? _defaultTimeout);
    }

    public void MoveCircle(Pose pose1, Pose pose2, double speed, double acceleration, double time, double percent, TimeSpan? timeout = null)
    {
        var pose1Payload = new[]
        {
            (float)pose1.X,
            (float)pose1.Y,
            (float)pose1.Z,
            (float)pose1.Roll,
            (float)pose1.Pitch,
            (float)pose1.Yaw
        };
        var pose2Payload = new[]
        {
            (float)pose2.X,
            (float)pose2.Y,
            (float)pose2.Z,
            (float)pose2.Roll,
            (float)pose2.Pitch,
            (float)pose2.Yaw
        };

        _uxbus.MoveCircle(pose1Payload, pose2Payload, (float)speed, (float)acceleration, (float)time, (float)percent, timeout ?? _defaultTimeout);
    }

    public void SleepInstruction(double seconds, TimeSpan? timeout = null)
    {
        _uxbus.SleepInstruction((float)seconds, timeout ?? _defaultTimeout);
    }

    public void SetTcpOffset(Pose offset, TimeSpan? timeout = null)
    {
        var payload = new[]
        {
            (float)offset.X,
            (float)offset.Y,
            (float)offset.Z,
            (float)offset.Roll,
            (float)offset.Pitch,
            (float)offset.Yaw
        };

        _uxbus.SetTcpOffset(payload, timeout ?? _defaultTimeout);
    }

    public void SetTcpJerk(double jerk, TimeSpan? timeout = null)
    {
        _uxbus.SetTcpJerk((float)jerk, timeout ?? _defaultTimeout);
    }

    public void SetTcpMaxAcceleration(double maxAcceleration, TimeSpan? timeout = null)
    {
        _uxbus.SetTcpMaxAcceleration((float)maxAcceleration, timeout ?? _defaultTimeout);
    }

    public void SetJointJerk(double jerk, TimeSpan? timeout = null)
    {
        _uxbus.SetJointJerk((float)jerk, timeout ?? _defaultTimeout);
    }

    public void SetJointMaxAcceleration(double maxAcceleration, TimeSpan? timeout = null)
    {
        _uxbus.SetJointMaxAcceleration((float)maxAcceleration, timeout ?? _defaultTimeout);
    }

    public void SetTcpLoad(double mass, Vector3 centerOfGravity, TimeSpan? timeout = null)
    {
        var com = new[] { (float)centerOfGravity.X, (float)centerOfGravity.Y, (float)centerOfGravity.Z };
        _uxbus.SetTcpLoad((float)mass, com, timeout ?? _defaultTimeout);
    }

    public void SetCollisionSensitivity(int sensitivity, TimeSpan? timeout = null)
    {
        _uxbus.SetCollisionSensitivity((byte)sensitivity, timeout ?? _defaultTimeout);
    }

    public void SetTeachSensitivity(int sensitivity, TimeSpan? timeout = null)
    {
        _uxbus.SetTeachSensitivity((byte)sensitivity, timeout ?? _defaultTimeout);
    }

    public void SetGravityDirection(Vector3 gravityDirection, TimeSpan? timeout = null)
    {
        var gravity = new[] { (float)gravityDirection.X, (float)gravityDirection.Y, (float)gravityDirection.Z };
        _uxbus.SetGravityDirection(gravity, timeout ?? _defaultTimeout);
    }

    public void SetSafeLevel(int level, TimeSpan? timeout = null)
    {
        _uxbus.SetSafeLevel((byte)level, timeout ?? _defaultTimeout);
    }

    public int GetSafeLevel(TimeSpan? timeout = null)
    {
        return _uxbus.GetSafeLevel(timeout ?? _defaultTimeout);
    }

    public void CleanConfiguration(TimeSpan? timeout = null)
    {
        _uxbus.CleanConfiguration(timeout ?? _defaultTimeout);
    }

    public void SaveConfiguration(TimeSpan? timeout = null)
    {
        _uxbus.SaveConfiguration(timeout ?? _defaultTimeout);
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

    public JointTorques GetJointTorques()
    {
        var torques = _uxbus.GetJointTorques(_defaultTimeout);
        return new JointTorques(
            torques.Length > 0 ? torques[0] : 0,
            torques.Length > 1 ? torques[1] : 0,
            torques.Length > 2 ? torques[2] : 0,
            torques.Length > 3 ? torques[3] : 0,
            torques.Length > 4 ? torques[4] : 0,
            torques.Length > 5 ? torques[5] : 0);
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

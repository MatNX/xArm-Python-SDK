using System.Buffers.Binary;
using System.Text;
using System.Text.Json;

namespace XArm.Lite6;

public class Lite6ArmClient : IDisposable
{
    private readonly IArmTransport _transport;
    private readonly TimeSpan _defaultTimeout;
    private readonly UxbusClient _uxbus;
    private string? _controllerHost;

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
        _controllerHost = host;
    }

    public void Disconnect()
    {
        _transport.Disconnect();
        _controllerHost = null;
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

    public void SystemControl(int value, TimeSpan? timeout = null)
    {
        _uxbus.SystemControl(value, timeout ?? _defaultTimeout);
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

    public IReadOnlyList<TrajectoryInfo> GetTrajectories(string? host = null, TimeSpan? timeout = null)
    {
        var controllerHost = ResolveControllerHost(host);
        var payload = JsonSerializer.Serialize(new { cmd = "xarm_list_trajs" });
        using var document = PostJsonDocument($"http://{controllerHost}:18333/cmd", payload, timeout ?? _defaultTimeout);
        if (!document.RootElement.TryGetProperty("res", out var res) || res.ValueKind != JsonValueKind.Array)
        {
            throw new InvalidOperationException("Invalid trajectory list response payload.");
        }

        var code = res[0].GetInt32();
        if (code != 0)
        {
            throw new InvalidOperationException($"Failed to list trajectories (code {code}).");
        }

        if (res.GetArrayLength() < 2 || res[1].ValueKind != JsonValueKind.Array)
        {
            return Array.Empty<TrajectoryInfo>();
        }

        var trajectories = new List<TrajectoryInfo>();
        foreach (var item in res[1].EnumerateArray())
        {
            if (!item.TryGetProperty("name", out var nameElement) || nameElement.ValueKind != JsonValueKind.String)
            {
                continue;
            }

            var name = nameElement.GetString() ?? string.Empty;
            var count = item.TryGetProperty("count", out var countElement) && countElement.TryGetDouble(out var countValue)
                ? countValue
                : 0;
            trajectories.Add(new TrajectoryInfo(name, count / 100d));
        }

        return trajectories;
    }

    public void StartRecordTrajectory(TimeSpan? timeout = null)
    {
        _uxbus.SetRecordTrajectory(true, timeout ?? _defaultTimeout);
    }

    public void StopRecordTrajectory(string? filename = null, bool save = true, TimeSpan? timeout = null)
    {
        _uxbus.SetRecordTrajectory(false, timeout ?? _defaultTimeout);
        if (save && !string.IsNullOrWhiteSpace(filename))
        {
            SaveRecordTrajectory(filename, wait: true, timeout);
        }
    }

    public double GetRecordSeconds(TimeSpan? timeout = null)
    {
        var payload = _uxbus.GetCommonInfo(50, timeout ?? _defaultTimeout);
        if (payload.Length < 4)
        {
            return 0;
        }

        return BinaryPrimitives.ReadSingleLittleEndian(payload.AsSpan(0, 4));
    }

    public void SaveRecordTrajectory(string filename, bool wait = true, TimeSpan? timeout = null)
    {
        var actualTimeout = timeout ?? _defaultTimeout;
        _uxbus.SaveTrajectory(filename, actualTimeout);
        if (wait)
        {
            WaitForTrajectoryStatus(TrajectoryRwStatus.SaveSuccess, TrajectoryRwStatus.SaveFailed, actualTimeout, filename, "save");
        }
    }

    public void LoadTrajectory(string filename, bool wait = true, TimeSpan? timeout = null)
    {
        var actualTimeout = timeout ?? _defaultTimeout;
        _uxbus.LoadTrajectory(filename, actualTimeout);
        if (wait)
        {
            WaitForTrajectoryStatus(TrajectoryRwStatus.LoadSuccess, TrajectoryRwStatus.LoadFailed, actualTimeout, filename, "load");
        }
    }

    public void PlaybackTrajectory(int times = 1, string? filename = null, bool wait = false, int doubleSpeed = 1, TimeSpan? timeout = null)
    {
        if (!string.IsNullOrWhiteSpace(filename))
        {
            LoadTrajectory(filename, true, timeout);
        }

        _uxbus.PlaybackTrajectory(times > 0 ? times : -1, doubleSpeed, timeout ?? _defaultTimeout);
        if (wait)
        {
            WaitForPlaybackCompletion(timeout ?? _defaultTimeout);
        }
    }

    public TrajectoryRwStatus GetTrajectoryRwStatus(TimeSpan? timeout = null)
    {
        var status = _uxbus.GetTrajectoryRwStatus(timeout ?? _defaultTimeout);
        return Enum.IsDefined(typeof(TrajectoryRwStatus), status) ? (TrajectoryRwStatus)status : TrajectoryRwStatus.Idle;
    }

    public void DeleteTrajectory(string filename, string? host = null, TimeSpan? timeout = null)
    {
        var controllerHost = ResolveControllerHost(host);
        var payload = JsonSerializer.Serialize(new
        {
            cmd = "Core.command.xarm_delete_traj",
            args = Array.Empty<object>(),
            kwargs = new { filename }
        });
        using var document = PostJsonDocument($"http://{controllerHost}:18333/v2/api", payload, timeout ?? _defaultTimeout);
        if (!document.RootElement.TryGetProperty("code", out var codeElement))
        {
            throw new InvalidOperationException("Invalid delete trajectory response payload.");
        }

        var code = codeElement.GetInt32();
        if (code != 0)
        {
            throw new InvalidOperationException($"Failed to delete trajectory (code {code}).");
        }
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

    public void EmergencyStop(TimeSpan? timeout = null)
    {
        var actualTimeout = timeout ?? _defaultTimeout;
        SetState(4, actualTimeout);
        MotionEnable(true, 8, actualTimeout);
        SetState(0, actualTimeout);
    }

    public void CleanError(TimeSpan? timeout = null)
    {
        _uxbus.CleanError(timeout ?? _defaultTimeout);
    }

    public void CleanWarning(TimeSpan? timeout = null)
    {
        _uxbus.CleanWarning(timeout ?? _defaultTimeout);
    }

    public void SetSelfCollisionDetection(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetSelfCollisionDetection(enable, timeout ?? _defaultTimeout);
    }

    public void SetCollisionToolModel(CollisionToolType toolType, double[]? parameters = null, TimeSpan? timeout = null)
    {
        var payload = parameters is null ? Array.Empty<float>() : Array.ConvertAll(parameters, item => (float)item);
        _uxbus.SetCollisionToolModel((byte)toolType, payload, timeout ?? _defaultTimeout);
    }

    public void SetSimulationRobot(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetSimulationRobot(enable, timeout ?? _defaultTimeout);
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

    public void MoveJointsBlend(JointPositions joints, double speed, double acceleration, double radius, TimeSpan? timeout = null)
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

        _uxbus.MoveJointB(payload, (float)speed, (float)acceleration, (float)radius, timeout ?? _defaultTimeout);
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

    public void MoveLinearBlend(Pose pose, double speed, double acceleration, double time, double radius, TimeSpan? timeout = null)
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

        _uxbus.MoveLineB(payload, (float)speed, (float)acceleration, (float)time, (float)radius, timeout ?? _defaultTimeout);
    }

    public void MoveLinearCommon(Pose pose, double speed, double acceleration, double time, double radius, byte coordinate, bool isAxisAngle, byte onlyCheckType = 0, TimeSpan? timeout = null)
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

        _uxbus.MoveLineCommon(payload, (float)speed, (float)acceleration, (float)time, (float)radius, coordinate, isAxisAngle, onlyCheckType, timeout ?? _defaultTimeout);
    }

    public void MoveLinearTool(Pose pose, double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
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

        _uxbus.MoveLineTool(payload, (float)speed, (float)acceleration, (float)time, timeout ?? _defaultTimeout);
    }

    public void MoveLinearAxisAngle(Pose axisAnglePose, double speed, double acceleration, double time, byte coordinate = 0, bool relative = false, TimeSpan? timeout = null)
    {
        var payload = new[]
        {
            (float)axisAnglePose.X,
            (float)axisAnglePose.Y,
            (float)axisAnglePose.Z,
            (float)axisAnglePose.Roll,
            (float)axisAnglePose.Pitch,
            (float)axisAnglePose.Yaw
        };

        _uxbus.MoveLineAxisAngle(payload, (float)speed, (float)acceleration, (float)time, coordinate, relative, timeout ?? _defaultTimeout);
    }

    public void MoveRelative(Pose pose, double speed, double acceleration, double time, double radius, bool isJointMotion = false, bool isAxisAngle = false, TimeSpan? timeout = null)
    {
        var payload = new[]
        {
            (float)pose.X,
            (float)pose.Y,
            (float)pose.Z,
            (float)pose.Roll,
            (float)pose.Pitch,
            (float)pose.Yaw,
            0f
        };

        _uxbus.MoveRelative(payload, (float)speed, (float)acceleration, (float)time, (float)radius, isJointMotion, isAxisAngle, timeout ?? _defaultTimeout);
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

    public void MoveServoJoints(JointPositions joints, TimeSpan? timeout = null)
    {
        MoveServoJoints(joints, 0, 0, 0, timeout);
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

    public void MoveServoLinearAxisAngle(Pose axisAnglePose, double speed, double acceleration, int toolCoord = 0, bool relative = false, TimeSpan? timeout = null)
    {
        var payload = new[]
        {
            (float)axisAnglePose.X,
            (float)axisAnglePose.Y,
            (float)axisAnglePose.Z,
            (float)axisAnglePose.Roll,
            (float)axisAnglePose.Pitch,
            (float)axisAnglePose.Yaw
        };

        _uxbus.MoveServoCartesianAxisAngle(payload, (float)speed, (float)acceleration, toolCoord, relative, timeout ?? _defaultTimeout);
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

    public void MoveCircleCommon(Pose pose1, Pose pose2, double speed, double acceleration, double time, double percent, byte coordinate, bool isAxisAngle, byte onlyCheckType = 0, TimeSpan? timeout = null)
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

        _uxbus.MoveCircleCommon(pose1Payload, pose2Payload, (float)speed, (float)acceleration, (float)time, (float)percent, coordinate, isAxisAngle, onlyCheckType, timeout ?? _defaultTimeout);
    }

    public void SetPosition(Pose pose, double speed, double acceleration, double time = 0, double? radius = null, bool relative = false, TimeSpan? timeout = null)
    {
        if (relative)
        {
            MoveRelative(pose, speed, acceleration, time, (float)(radius ?? -1), false, false, timeout);
            return;
        }

        if (radius.HasValue && radius.Value >= 0)
        {
            MoveLinearBlend(pose, speed, acceleration, time, radius.Value, timeout);
            return;
        }

        MoveLinear(pose, speed, acceleration, time, timeout);
    }

    public void SetToolPosition(Pose pose, double speed, double acceleration, double time = 0, double? radius = null, TimeSpan? timeout = null)
    {
        if (radius.HasValue)
        {
            MoveLinearCommon(pose, speed, acceleration, time, radius.Value, 1, false, 0, timeout);
            return;
        }

        MoveLinearTool(pose, speed, acceleration, time, timeout);
    }

    public void SetPositionAxisAngle(Pose axisAnglePose, double speed, double acceleration, double time = 0, double? radius = null, bool isToolCoord = false, bool relative = false, TimeSpan? timeout = null)
    {
        if (!isToolCoord && relative)
        {
            MoveRelative(axisAnglePose, speed, acceleration, time, (float)(radius ?? -1), false, true, timeout);
            return;
        }

        MoveLinearCommon(axisAnglePose, speed, acceleration, time, (float)(radius ?? -1), (byte)(isToolCoord ? 1 : 0), true, 0, timeout);
    }

    public void SetServoAngle(JointPositions joints, double speed, double acceleration, double time = 0, double? radius = null, TimeSpan? timeout = null)
    {
        if (radius.HasValue && radius.Value >= 0)
        {
            MoveJointsBlend(joints, speed, acceleration, radius.Value, timeout);
            return;
        }

        MoveJoints(joints, speed, acceleration, time, timeout);
    }

    public void SetServoAngle(int servoId, double angle, double speed, double acceleration, double time = 0, double? radius = null, TimeSpan? timeout = null)
    {
        var joints = GetJointPositions();
        var updated = servoId switch
        {
            1 => joints with { J1 = angle },
            2 => joints with { J2 = angle },
            3 => joints with { J3 = angle },
            4 => joints with { J4 = angle },
            5 => joints with { J5 = angle },
            6 => joints with { J6 = angle },
            _ => throw new ArgumentOutOfRangeException(nameof(servoId), "Servo id must be between 1 and 6.")
        };

        SetServoAngle(updated, speed, acceleration, time, radius, timeout);
    }

    public void SetServoAngleJ(JointPositions joints, TimeSpan? timeout = null)
    {
        MoveServoJoints(joints, timeout);
    }

    public void SetServoCartesian(Pose pose, double speed, double acceleration, double time = 0, TimeSpan? timeout = null)
    {
        MoveServoLinear(pose, speed, acceleration, time, timeout);
    }

    public void SetServoCartesianAxisAngle(Pose axisAnglePose, double speed, double acceleration, int toolCoord = 0, bool relative = false, TimeSpan? timeout = null)
    {
        MoveServoLinearAxisAngle(axisAnglePose, speed, acceleration, toolCoord, relative, timeout);
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

    public void SetGripperEnable(bool enable, TimeSpan? timeout = null)
    {
        _uxbus.SetGripperEnable(enable, timeout ?? _defaultTimeout);
    }

    public void SetGripperMode(int mode, TimeSpan? timeout = null)
    {
        _uxbus.SetGripperMode(mode, timeout ?? _defaultTimeout);
    }

    public void SetGripperSpeed(int speed, TimeSpan? timeout = null)
    {
        _uxbus.SetGripperSpeed(speed, timeout ?? _defaultTimeout);
    }

    public int GetGripperPosition(TimeSpan? timeout = null)
    {
        return _uxbus.GetGripperPosition(timeout ?? _defaultTimeout);
    }

    public void SetGripperPosition(int position, TimeSpan? timeout = null)
    {
        _uxbus.SetGripperPosition(position, timeout ?? _defaultTimeout);
    }

    public void SetGripperZero(TimeSpan? timeout = null)
    {
        _uxbus.SetGripperZero(timeout ?? _defaultTimeout);
    }

    public int GetGripperErrorCode(TimeSpan? timeout = null)
    {
        return _uxbus.GetGripperErrorCode(timeout ?? _defaultTimeout);
    }

    public void CleanGripperError(TimeSpan? timeout = null)
    {
        _uxbus.CleanGripperError(timeout ?? _defaultTimeout);
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

    private string ResolveControllerHost(string? host)
    {
        if (!string.IsNullOrWhiteSpace(host))
        {
            return host;
        }

        return _controllerHost ?? throw new InvalidOperationException("Controller host is not set. Connect or provide a host.");
    }

    private static JsonDocument PostJsonDocument(string url, string payload, TimeSpan timeout)
    {
        using var client = new HttpClient { Timeout = timeout };
        using var content = new StringContent(payload, Encoding.UTF8, "application/json");
        using var response = client.PostAsync(url, content).GetAwaiter().GetResult();
        response.EnsureSuccessStatusCode();
        var responsePayload = response.Content.ReadAsStringAsync().GetAwaiter().GetResult();
        return JsonDocument.Parse(responsePayload);
    }

    private void WaitForTrajectoryStatus(TrajectoryRwStatus successStatus, TrajectoryRwStatus failureStatus, TimeSpan timeout, string filename, string operation)
    {
        var deadline = DateTime.UtcNow + timeout;
        while (DateTime.UtcNow < deadline)
        {
            var status = GetTrajectoryRwStatus(timeout: _defaultTimeout);
            if (status == successStatus)
            {
                return;
            }

            if (status == failureStatus)
            {
                throw new InvalidOperationException($"Trajectory {operation} failed for {filename} (status {status}).");
            }

            Thread.Sleep(TimeSpan.FromMilliseconds(100));
        }

        throw new TimeoutException($"Trajectory {operation} timed out for {filename}.");
    }

    private void WaitForPlaybackCompletion(TimeSpan timeout)
    {
        var deadline = DateTime.UtcNow + timeout;
        var pollTimeout = TimeSpan.FromMilliseconds(250);
        while (DateTime.UtcNow < deadline)
        {
            var state = GetState(pollTimeout);
            if (state == 0 || state == 1)
            {
                return;
            }

            Thread.Sleep(TimeSpan.FromMilliseconds(100));
        }

        throw new TimeoutException("Trajectory playback timed out.");
    }

    public void Dispose()
    {
        _transport.Dispose();
    }
}

using System.Globalization;

namespace XArm.Lite6;

public class Lite6ArmClient : IDisposable
{
    private readonly IArmTransport _transport;
    private readonly TimeSpan _defaultTimeout;

    public Lite6ArmClient(IArmTransport? transport = null, TimeSpan? defaultTimeout = null)
    {
        _transport = transport ?? new TcpArmTransport();
        _defaultTimeout = defaultTimeout ?? TimeSpan.FromSeconds(2);
    }

    public bool IsConnected => _transport.IsConnected;

    public void Connect(string host, int port = 30001, TimeSpan? timeout = null)
    {
        _transport.Connect(host, port, timeout ?? _defaultTimeout);
    }

    public void Disconnect()
    {
        _transport.Disconnect();
    }

    public void EnableMotion()
    {
        SendCommand("MOTION_ENABLE 1");
    }

    public void DisableMotion()
    {
        SendCommand("MOTION_ENABLE 0");
    }

    public void SetMode(int mode)
    {
        SendCommand($"SET_MODE {mode}");
    }

    public void SetState(int state)
    {
        SendCommand($"SET_STATE {state}");
    }

    public void MoveJoints(JointPositions joints, double speed, double acceleration)
    {
        var command = string.Format(
            CultureInfo.InvariantCulture,
            "MOVE_J {0:F4} {1:F4} {2:F4} {3:F4} {4:F4} {5:F4} {6:F3} {7:F3}",
            joints.J1,
            joints.J2,
            joints.J3,
            joints.J4,
            joints.J5,
            joints.J6,
            speed,
            acceleration);
        SendCommand(command);
    }

    public void MoveLinear(Pose pose, double speed, double acceleration)
    {
        var command = string.Format(
            CultureInfo.InvariantCulture,
            "MOVE_L {0:F3} {1:F3} {2:F3} {3:F3} {4:F3} {5:F3} {6:F3} {7:F3}",
            pose.X,
            pose.Y,
            pose.Z,
            pose.Roll,
            pose.Pitch,
            pose.Yaw,
            speed,
            acceleration);
        SendCommand(command);
    }

    public JointPositions GetJointPositions()
    {
        var response = SendCommand("GET_JOINTS");
        return ParseJoints(response);
    }

    public Pose GetPose()
    {
        var response = SendCommand("GET_POSE");
        return ParsePose(response);
    }

    public ArmStatus GetStatus()
    {
        var response = SendCommand("GET_STATUS");
        var parts = response.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 5)
        {
            throw new FormatException("Unexpected status response.");
        }

        return new ArmStatus(
            IsConnected,
            parts[0] == "1",
            int.Parse(parts[1], CultureInfo.InvariantCulture),
            int.Parse(parts[2], CultureInfo.InvariantCulture),
            double.Parse(parts[3], CultureInfo.InvariantCulture),
            parts.Length > 4 ? parts[4] : null);
    }

    public double GetBatteryVoltage()
    {
        var response = SendCommand("GET_BATTERY");
        return double.Parse(response, CultureInfo.InvariantCulture);
    }

    public string GetFirmwareVersion()
    {
        return SendCommand("GET_FIRMWARE");
    }

    public void Stop()
    {
        SendCommand("STOP");
    }

    public void ClearError()
    {
        SendCommand("CLEAR_ERROR");
    }

    public void Dispose()
    {
        _transport.Dispose();
    }

    private string SendCommand(string command, TimeSpan? timeout = null)
    {
        return _transport.SendCommand(command, timeout ?? _defaultTimeout);
    }

    private static JointPositions ParseJoints(string response)
    {
        var parts = response.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 6)
        {
            throw new FormatException("Expected 6 joint values.");
        }

        return new JointPositions(
            double.Parse(parts[0], CultureInfo.InvariantCulture),
            double.Parse(parts[1], CultureInfo.InvariantCulture),
            double.Parse(parts[2], CultureInfo.InvariantCulture),
            double.Parse(parts[3], CultureInfo.InvariantCulture),
            double.Parse(parts[4], CultureInfo.InvariantCulture),
            double.Parse(parts[5], CultureInfo.InvariantCulture));
    }

    private static Pose ParsePose(string response)
    {
        var parts = response.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 6)
        {
            throw new FormatException("Expected 6 pose values.");
        }

        return new Pose(
            double.Parse(parts[0], CultureInfo.InvariantCulture),
            double.Parse(parts[1], CultureInfo.InvariantCulture),
            double.Parse(parts[2], CultureInfo.InvariantCulture),
            double.Parse(parts[3], CultureInfo.InvariantCulture),
            double.Parse(parts[4], CultureInfo.InvariantCulture),
            double.Parse(parts[5], CultureInfo.InvariantCulture));
    }
}

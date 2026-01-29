namespace XArm.Lite6;

public record JointPositions(double J1, double J2, double J3, double J4, double J5, double J6);

public record JointTorques(double J1, double J2, double J3, double J4, double J5, double J6);

public record Pose(double X, double Y, double Z, double Roll, double Pitch, double Yaw);

public record Vector3(double X, double Y, double Z);

public record ArmStatus(
    bool IsConnected,
    int State,
    int ErrorCode,
    int WarningCode);

public record ReducedState(
    bool ReducedModeEnabled,
    short[] TcpBoundary,
    double TcpSpeed,
    double JointSpeed,
    double[]? JointRanges,
    bool? FenseEnabled,
    bool? CollisionReboundEnabled);

public record TrajectoryInfo(string Name, double DurationSeconds);

public enum TrajectoryRwStatus
{
    Idle = 0,
    Loading = 1,
    LoadSuccess = 2,
    LoadFailed = 3,
    Saving = 4,
    SaveSuccess = 5,
    SaveFailed = 6
}

public enum MotionType
{
    Joint,
    Linear
}

public enum CollisionToolType
{
    None = 0,
    XarmGripper = 1,
    XarmVacuumGripper = 2,
    XarmBioGripper = 3,
    Robotiq2F85 = 4,
    Robotiq2F140 = 5,
    Cylinder = 21,
    Box = 22
}

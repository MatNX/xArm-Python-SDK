namespace XArm.Lite6;

public record JointPositions(double J1, double J2, double J3, double J4, double J5, double J6);

public record Pose(double X, double Y, double Z, double Roll, double Pitch, double Yaw);

public record ArmStatus(
    bool IsConnected,
    int State,
    int ErrorCode,
    int WarningCode);

public enum MotionType
{
    Joint,
    Linear
}

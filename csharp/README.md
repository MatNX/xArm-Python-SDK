# Lite6Arm C# Library (Experimental)

This folder contains a minimal C# class library that mirrors common Lite6 robotic arm
operations such as connecting, sending motion commands, and reading state. The implementation
now uses the same UXBUS Modbus/TCP framing as the Python SDK for the core commands.

## Usage example

```csharp
using XArm.Lite6;

using var arm = new Lite6ArmClient();
arm.Connect("192.168.1.10");
arm.EnableMotion();
arm.SetMode(0);
arm.SetState(0);
arm.CleanError();
arm.SetTcpOffset(new Pose(0, 0, 0, 0, 0, 0));
arm.SetTcpJerk(0.5);
arm.SetTcpMaxAcceleration(0.5);
arm.SetJointJerk(0.5);
arm.SetJointMaxAcceleration(0.5);

arm.MoveJoints(new JointPositions(0, 0, 0, 0, 0, 0), speed: 0.5, acceleration: 0.2);
arm.MoveHome(speed: 0.5, acceleration: 0.2);
var pose = arm.GetPose();
var status = arm.GetStatus();
var version = arm.GetFirmwareVersion();
var serial = arm.GetRobotSerialNumber();
var commandCount = arm.GetCommandCount();
```

## Notes

* Lite6 uses the UXBUS TCP control port (default 502) for commands; the client mirrors the
  Python SDK's Modbus/TCP framing for motion, state, and pose queries.
* This library intentionally covers a small, practical subset of the Python SDK. Extend it
  with additional UXBUS registers as needed.

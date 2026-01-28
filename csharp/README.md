# Lite6Arm C# Library (Experimental)

This folder contains a minimal C# class library that mirrors common Lite6 robotic arm
operations such as connecting, sending motion commands, and reading state.

## Usage example

```csharp
using XArm.Lite6;

using var arm = new Lite6ArmClient();
arm.Connect("192.168.1.10");
arm.EnableMotion();
arm.SetMode(0);
arm.SetState(0);

arm.MoveJoints(new JointPositions(0, 0, 0, 0, 0, 0), speed: 0.5, acceleration: 0.2);
var pose = arm.GetPose();
var status = arm.GetStatus();
```

## Notes

* The transport sends line-delimited ASCII commands. Replace the command strings
  with real Lite6 protocol messages as needed.
* This library is intended as a starting point for a full C# implementation.

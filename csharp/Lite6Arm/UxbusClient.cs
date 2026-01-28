using System.Buffers.Binary;
using System.Text;

namespace XArm.Lite6;

public enum UxbusState
{
    Ok = 0,
    ErrorCode = 1,
    WarningCode = 2,
    Timeout = 3,
    LengthError = 4,
    TransactionMismatch = 5,
    ProtocolMismatch = 6,
    FunctionMismatch = 7,
    NotTcp = 8,
    StateNotReady = 9,
    Invalid = 10,
    Other = 11,
    ParameterError = 12
}

public sealed class UxbusException : Exception
{
    public UxbusException(UxbusState state, string message) : base(message)
    {
        State = state;
    }

    public UxbusState State { get; }
}

internal sealed class UxbusClient
{
    private const ushort PrivateProtocolId = 0x02;
    private const ushort TransactionIdMax = 65535;

    private readonly IArmTransport _transport;
    private readonly object _sync = new();
    private ushort _transactionId = 1;

    public UxbusClient(IArmTransport transport)
    {
        _transport = transport;
    }

    public void MotionEnable(int servoId, bool enable, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.MotionEnable, new[] { (byte)servoId, (byte)(enable ? 1 : 0) }, timeout);
    }

    public void SetMode(int mode, int? detectionParam, TimeSpan timeout)
    {
        if (detectionParam.HasValue)
        {
            SetNu8(UxbusRegister.SetMode, new[] { (byte)mode, (byte)detectionParam.Value }, timeout);
            return;
        }

        SetNu8(UxbusRegister.SetMode, new[] { (byte)mode }, timeout);
    }

    public void SetState(int state, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetState, new[] { (byte)state }, timeout);
    }

    public int GetState(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetState, 1, timeout);
        return response.Length > 0 ? response[0] : 0;
    }

    public (int Error, int Warning) GetErrorWarning(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetError, 2, timeout);
        var error = response.Length > 0 ? response[0] : 0;
        var warning = response.Length > 1 ? response[1] : 0;
        return (error, warning);
    }

    public string GetVersion(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetVersion, 40, timeout);
        var end = Array.IndexOf(response, (byte)0);
        var length = end >= 0 ? end : response.Length;
        return Encoding.ASCII.GetString(response, 0, length).Trim();
    }

    public string GetRobotSerialNumber(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetRobotSn, 40, timeout);
        var end = Array.IndexOf(response, (byte)0);
        var length = end >= 0 ? end : response.Length;
        return Encoding.ASCII.GetString(response, 0, length).Trim();
    }

    public int GetCommandCount(TimeSpan timeout)
    {
        var response = GetNu16(UxbusRegister.GetCmdnum, 1, timeout);
        return response.Length > 0 ? response[0] : 0;
    }

    public void SetBrake(int axisId, bool enable, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetBrake, new[] { (byte)axisId, (byte)(enable ? 1 : 0) }, timeout);
    }

    public void SetReportTorqueOrCurrent(bool reportCurrent, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.ReportTauOrI, new[] { (byte)(reportCurrent ? 1 : 0) }, timeout);
    }

    public int GetReportTorqueOrCurrent(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetReportTauOrI, 1, timeout);
        return response.Length > 0 ? response[0] : 0;
    }

    public void SetCartesianVelocityContinuous(bool enable, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetCartVContinue, new[] { (byte)(enable ? 1 : 0) }, timeout);
    }

    public void SetReducedMode(bool enable, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetReducedMode, new[] { (byte)(enable ? 1 : 0) }, timeout);
    }

    public int GetReducedMode(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetReducedMode, 1, timeout);
        return response.Length > 0 ? response[0] : 0;
    }

    public void SetReducedLineSpeed(float speed, TimeSpan timeout)
    {
        SetFp32(UxbusRegister.SetReducedTrsv, new[] { speed }, timeout);
    }

    public void SetReducedJointSpeed(float speed, TimeSpan timeout)
    {
        SetFp32(UxbusRegister.SetReducedP2pv, new[] { speed }, timeout);
    }

    public ReducedState GetReducedState(bool includeJointRanges, TimeSpan timeout)
    {
        var length = includeJointRanges ? 79 : 21;
        var payload = GetNu8(UxbusRegister.GetReducedState, length, timeout);
        var reducedMode = payload.Length > 0 && payload[0] != 0;
        var boundary = ReadInt16BigEndian(payload, 1, 6);
        var tcpSpeed = ReadSingleLittleEndian(payload, 13);
        var jointSpeed = ReadSingleLittleEndian(payload, 17);

        double[]? jointRanges = null;
        bool? fenseOn = null;
        bool? collisionRebound = null;

        if (includeJointRanges && payload.Length >= 79)
        {
            jointRanges = ReadSinglesLittleEndian(payload, 21, 14);
            fenseOn = payload[77] != 0;
            collisionRebound = payload[78] != 0;
        }

        return new ReducedState(reducedMode, boundary, tcpSpeed, jointSpeed, jointRanges, fenseOn, collisionRebound);
    }

    public void SetReducedJointRange(float[] ranges, TimeSpan timeout)
    {
        var payload = new float[14];
        Array.Copy(ranges, payload, Math.Min(ranges.Length, 14));
        SetFp32(UxbusRegister.SetReducedJRange, payload, timeout);
    }

    public void SetFenseOn(bool enable, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetFenseOn, new[] { (byte)(enable ? 1 : 0) }, timeout);
    }

    public void SetCollisionRebound(bool enable, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetCollisReb, new[] { (byte)(enable ? 1 : 0) }, timeout);
    }

    public void SetXyzLimits(int[] xyzLimits, TimeSpan timeout)
    {
        var payload = new int[6];
        Array.Copy(xyzLimits, payload, Math.Min(xyzLimits.Length, 6));
        SetInt32(UxbusRegister.SetLimitXyz, payload, timeout);
    }

    public void SetTimer(int secondsLater, int timerId, int functionCode, int param1, int param2, TimeSpan timeout)
    {
        SetInt32(UxbusRegister.SetTimer, new[] { secondsLater, timerId, functionCode, param1, param2 }, timeout);
    }

    public void CancelTimer(int timerId, TimeSpan timeout)
    {
        SetInt32(UxbusRegister.CancelTimer, new[] { timerId }, timeout);
    }

    public void SetWorldOffset(float[] offset, TimeSpan timeout)
    {
        var payload = new float[6];
        Array.Copy(offset, payload, Math.Min(offset.Length, 6));
        SetFp32(UxbusRegister.SetWorldOffset, payload, timeout);
    }

    public void CounterReset(TimeSpan timeout)
    {
        SetNu8(UxbusRegister.CounterReset, Array.Empty<byte>(), timeout);
    }

    public void CounterPlus(TimeSpan timeout)
    {
        SetNu8(UxbusRegister.CounterPlus, Array.Empty<byte>(), timeout);
    }

    public void SetAllowApproxMotion(bool enable, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetAllowApproxMotion, new[] { (byte)(enable ? 1 : 0) }, timeout);
    }

    public int GetAllowApproxMotion(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetAllowApproxMotion, 1, timeout);
        return response.Length > 0 ? response[0] : 0;
    }

    public void CleanError(TimeSpan timeout)
    {
        SetNu8(UxbusRegister.CleanErr, Array.Empty<byte>(), timeout);
    }

    public void CleanWarning(TimeSpan timeout)
    {
        SetNu8(UxbusRegister.CleanWar, Array.Empty<byte>(), timeout);
    }

    public float[] GetJointPositions(TimeSpan timeout)
    {
        return GetFp32(UxbusRegister.GetJointPos, 7, timeout);
    }

    public float[] GetTcpPose(TimeSpan timeout)
    {
        return GetFp32(UxbusRegister.GetTcpPose, 6, timeout);
    }

    public void MoveJoint(float[] joints, float speed, float acceleration, float time, TimeSpan timeout)
    {
        var payload = new float[10];
        Array.Copy(joints, payload, Math.Min(joints.Length, 7));
        payload[7] = speed;
        payload[8] = acceleration;
        payload[9] = time;
        SetFp32(UxbusRegister.MoveJoint, payload, timeout);
    }

    public void MoveLine(float[] pose, float speed, float acceleration, float time, TimeSpan timeout)
    {
        var payload = new float[9];
        Array.Copy(pose, payload, Math.Min(pose.Length, 6));
        payload[6] = speed;
        payload[7] = acceleration;
        payload[8] = time;
        SetFp32(UxbusRegister.MoveLine, payload, timeout);
    }

    public void MoveServoJoint(float[] joints, float speed, float acceleration, float time, TimeSpan timeout)
    {
        var payload = new float[10];
        Array.Copy(joints, payload, Math.Min(joints.Length, 7));
        payload[7] = speed;
        payload[8] = acceleration;
        payload[9] = time;
        SetFp32(UxbusRegister.MoveServoJoint, payload, timeout);
    }

    public void MoveServoCartesian(float[] pose, float speed, float acceleration, float time, TimeSpan timeout)
    {
        var payload = new float[9];
        Array.Copy(pose, payload, Math.Min(pose.Length, 6));
        payload[6] = speed;
        payload[7] = acceleration;
        payload[8] = time;
        SetFp32(UxbusRegister.MoveServoCart, payload, timeout);
    }

    public void MoveHome(float speed, float acceleration, float time, TimeSpan timeout)
    {
        var payload = new[] { speed, acceleration, time };
        SetFp32(UxbusRegister.MoveHome, payload, timeout);
    }

    public void MoveCircle(float[] pose1, float[] pose2, float speed, float acceleration, float time, float percent, TimeSpan timeout)
    {
        var payload = new float[16];
        Array.Copy(pose1, 0, payload, 0, Math.Min(pose1.Length, 6));
        Array.Copy(pose2, 0, payload, 6, Math.Min(pose2.Length, 6));
        payload[12] = speed;
        payload[13] = acceleration;
        payload[14] = time;
        payload[15] = percent;
        SetFp32(UxbusRegister.MoveCircle, payload, timeout);
    }

    public void SleepInstruction(float seconds, TimeSpan timeout)
    {
        SetFp32(UxbusRegister.SleepInstruction, new[] { seconds }, timeout);
    }

    public void SetTcpOffset(float[] offset, TimeSpan timeout)
    {
        var payload = new float[6];
        Array.Copy(offset, payload, Math.Min(offset.Length, 6));
        SetFp32(UxbusRegister.SetTcpOffset, payload, timeout);
    }

    public void SetTcpJerk(float jerk, TimeSpan timeout)
    {
        SetFp32(UxbusRegister.SetTcpJerk, new[] { jerk }, timeout);
    }

    public void SetTcpMaxAcceleration(float maxAcceleration, TimeSpan timeout)
    {
        SetFp32(UxbusRegister.SetTcpMaxAcc, new[] { maxAcceleration }, timeout);
    }

    public void SetJointJerk(float jerk, TimeSpan timeout)
    {
        SetFp32(UxbusRegister.SetJointJerk, new[] { jerk }, timeout);
    }

    public void SetJointMaxAcceleration(float maxAcceleration, TimeSpan timeout)
    {
        SetFp32(UxbusRegister.SetJointMaxAcc, new[] { maxAcceleration }, timeout);
    }

    public void SetTcpLoad(float mass, float[] centerOfGravity, TimeSpan timeout)
    {
        var payload = new float[4];
        payload[0] = mass;
        Array.Copy(centerOfGravity, 0, payload, 1, Math.Min(centerOfGravity.Length, 3));
        SetFp32(UxbusRegister.SetLoadParam, payload, timeout);
    }

    public void SetCollisionSensitivity(byte value, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetCollisSens, new[] { value }, timeout);
    }

    public void SetTeachSensitivity(byte value, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetTeachSens, new[] { value }, timeout);
    }

    public void SetGravityDirection(float[] gravityDirection, TimeSpan timeout)
    {
        var payload = new float[3];
        Array.Copy(gravityDirection, payload, Math.Min(gravityDirection.Length, 3));
        SetFp32(UxbusRegister.SetGravityDir, payload, timeout);
    }

    public void SetSafeLevel(byte level, TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SetSafeLevel, new[] { level }, timeout);
    }

    public int GetSafeLevel(TimeSpan timeout)
    {
        var response = GetNu8(UxbusRegister.GetSafeLevel, 1, timeout);
        return response.Length > 0 ? response[0] : 0;
    }

    public void CleanConfiguration(TimeSpan timeout)
    {
        SetNu8(UxbusRegister.CleanConf, Array.Empty<byte>(), timeout);
    }

    public void SaveConfiguration(TimeSpan timeout)
    {
        SetNu8(UxbusRegister.SaveConf, Array.Empty<byte>(), timeout);
    }

    public float[] GetJointTorques(TimeSpan timeout)
    {
        return GetFp32(UxbusRegister.GetJointTau, 7, timeout);
    }

    private void SetInt32(UxbusRegister register, int[] values, TimeSpan timeout)
    {
        var payload = new byte[values.Length * 4];
        for (var i = 0; i < values.Length; i++)
        {
            BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(i * 4, 4), values[i]);
        }

        var response = SendRequest((byte)register, payload, timeout);
        response.EnsureSuccess();
    }

    private void SetNu8(UxbusRegister register, byte[] data, TimeSpan timeout)
    {
        var response = SendRequest((byte)register, data, timeout);
        response.EnsureSuccess();
    }

    private byte[] GetNu8(UxbusRegister register, int count, TimeSpan timeout)
    {
        var response = SendRequest((byte)register, Array.Empty<byte>(), timeout);
        response.EnsureSuccess();
        if (response.Payload.Length < count)
        {
            return response.Payload;
        }

        var result = new byte[count];
        Buffer.BlockCopy(response.Payload, 0, result, 0, count);
        return result;
    }

    private ushort[] GetNu16(UxbusRegister register, int count, TimeSpan timeout)
    {
        var response = SendRequest((byte)register, Array.Empty<byte>(), timeout);
        response.EnsureSuccess();
        if (response.Payload.Length < count * 2)
        {
            return Array.Empty<ushort>();
        }

        var result = new ushort[count];
        for (var i = 0; i < count; i++)
        {
            result[i] = BinaryPrimitives.ReadUInt16BigEndian(response.Payload.AsSpan(i * 2, 2));
        }

        return result;
    }

    private void SetFp32(UxbusRegister register, float[] values, TimeSpan timeout)
    {
        var payload = new byte[values.Length * 4];
        for (var i = 0; i < values.Length; i++)
        {
            BinaryPrimitives.WriteSingleLittleEndian(payload.AsSpan(i * 4, 4), values[i]);
        }

        var response = SendRequest((byte)register, payload, timeout);
        response.EnsureSuccess();
    }

    private float[] GetFp32(UxbusRegister register, int count, TimeSpan timeout)
    {
        var response = SendRequest((byte)register, Array.Empty<byte>(), timeout);
        response.EnsureSuccess();
        var values = new float[count];
        var length = Math.Min(response.Payload.Length, count * 4);
        var available = length / 4;
        for (var i = 0; i < available; i++)
        {
            values[i] = BinaryPrimitives.ReadSingleLittleEndian(response.Payload.AsSpan(i * 4, 4));
        }

        return values;
    }

    private static short[] ReadInt16BigEndian(byte[] buffer, int start, int count)
    {
        var result = new short[count];
        for (var i = 0; i < count; i++)
        {
            var offset = start + i * 2;
            if (offset + 1 >= buffer.Length)
            {
                return result;
            }

            result[i] = BinaryPrimitives.ReadInt16BigEndian(buffer.AsSpan(offset, 2));
        }

        return result;
    }

    private static double ReadSingleLittleEndian(byte[] buffer, int start)
    {
        if (start + 3 >= buffer.Length)
        {
            return 0;
        }

        return BinaryPrimitives.ReadSingleLittleEndian(buffer.AsSpan(start, 4));
    }

    private static double[] ReadSinglesLittleEndian(byte[] buffer, int start, int count)
    {
        var result = new double[count];
        for (var i = 0; i < count; i++)
        {
            var offset = start + i * 4;
            if (offset + 3 >= buffer.Length)
            {
                return result;
            }

            result[i] = BinaryPrimitives.ReadSingleLittleEndian(buffer.AsSpan(offset, 4));
        }

        return result;
    }

    private UxbusResponse SendRequest(byte functionCode, byte[] payload, TimeSpan timeout)
    {
        lock (_sync)
        {
            var transactionId = _transactionId;
            _transactionId = (ushort)(_transactionId % TransactionIdMax + 1);

            var request = BuildRequest(transactionId, functionCode, payload);
            var responseBytes = _transport.SendAndReceive(request, timeout);
            return ParseResponse(responseBytes, transactionId, functionCode);
        }
    }

    private static byte[] BuildRequest(ushort transactionId, byte functionCode, byte[] payload)
    {
        var length = (ushort)(payload.Length + 1);
        var buffer = new byte[7 + payload.Length];
        BinaryPrimitives.WriteUInt16BigEndian(buffer.AsSpan(0, 2), transactionId);
        BinaryPrimitives.WriteUInt16BigEndian(buffer.AsSpan(2, 2), PrivateProtocolId);
        BinaryPrimitives.WriteUInt16BigEndian(buffer.AsSpan(4, 2), length);
        buffer[6] = functionCode;
        if (payload.Length > 0)
        {
            Buffer.BlockCopy(payload, 0, buffer, 7, payload.Length);
        }

        return buffer;
    }

    private static UxbusResponse ParseResponse(byte[] response, ushort transactionId, byte functionCode)
    {
        if (response.Length < 8)
        {
            throw new UxbusException(UxbusState.LengthError, "Response is too short.");
        }

        var respTransaction = BinaryPrimitives.ReadUInt16BigEndian(response.AsSpan(0, 2));
        var respProtocol = BinaryPrimitives.ReadUInt16BigEndian(response.AsSpan(2, 2));
        var respFunction = response[6];

        if (respTransaction != transactionId)
        {
            throw new UxbusException(UxbusState.TransactionMismatch, "Transaction id mismatch.");
        }

        if (respProtocol != PrivateProtocolId)
        {
            throw new UxbusException(UxbusState.ProtocolMismatch, "Protocol identifier mismatch.");
        }

        if (respFunction != functionCode)
        {
            throw new UxbusException(UxbusState.FunctionMismatch, "Function code mismatch.");
        }

        var stateByte = response[7];
        var payload = response.Length > 8 ? response[8..] : Array.Empty<byte>();
        var isReady = (stateByte & 0x10) == 0;
        var state = MapState(stateByte);

        return new UxbusResponse(state, isReady, payload);
    }

    private static UxbusState MapState(byte stateByte)
    {
        if ((stateByte & 0x08) != 0)
        {
            return UxbusState.Invalid;
        }

        if ((stateByte & 0x40) != 0)
        {
            return UxbusState.ErrorCode;
        }

        if ((stateByte & 0x20) != 0)
        {
            return UxbusState.WarningCode;
        }

        return UxbusState.Ok;
    }
}

internal readonly record struct UxbusResponse(UxbusState State, bool IsReady, byte[] Payload)
{
    public void EnsureSuccess()
    {
        if (State != UxbusState.Ok)
        {
            throw new UxbusException(State, $"Uxbus command failed with state {State}.");
        }
    }
}

internal enum UxbusRegister : byte
{
    GetVersion = 0x01,
    GetRobotSn = 0x02,
    GetReportTauOrI = 0x05,
    GetAllowApproxMotion = 0x07,
    MotionEnable = 0x0B,
    SetState = 0x0C,
    GetState = 0x0D,
    GetCmdnum = 0x0E,
    GetError = 0x0F,
    CleanErr = 0x10,
    CleanWar = 0x11,
    SetBrake = 0x12,
    SetMode = 0x13,
    MoveLine = 0x15,
    MoveJoint = 0x17,
    MoveHome = 0x19,
    SleepInstruction = 0x1A,
    MoveCircle = 0x1B,
    MoveServoJoint = 0x1D,
    MoveServoCart = 0x1E,
    SetTcpJerk = 0x1F,
    SetTcpMaxAcc = 0x20,
    SetJointJerk = 0x21,
    SetJointMaxAcc = 0x22,
    SetTcpOffset = 0x23,
    SetLoadParam = 0x24,
    SetCollisSens = 0x25,
    SetTeachSens = 0x26,
    CleanConf = 0x27,
    SaveConf = 0x28,
    GetTcpPose = 0x29,
    GetJointPos = 0x2A,
    SetReducedTrsv = 0x2F,
    SetReducedP2pv = 0x30,
    GetReducedMode = 0x31,
    SetReducedMode = 0x32,
    SetGravityDir = 0x33,
    SetLimitXyz = 0x34,
    GetReducedState = 0x35,
    GetJointTau = 0x37,
    SetSafeLevel = 0x38,
    GetSafeLevel = 0x39,
    SetReducedJRange = 0x3A,
    SetFenseOn = 0x3B,
    SetCollisReb = 0x3C,
    SetAllowApproxMotion = 0x42,
    ReportTauOrI = 0x46,
    SetTimer = 0x47,
    CancelTimer = 0x48,
    SetWorldOffset = 0x49,
    CounterReset = 0x4A,
    CounterPlus = 0x4B,
    SetCartVContinue = 0x50
}

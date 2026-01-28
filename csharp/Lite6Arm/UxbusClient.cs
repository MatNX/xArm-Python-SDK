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
    MotionEnable = 0x0B,
    SetState = 0x0C,
    GetState = 0x0D,
    GetError = 0x0F,
    SetMode = 0x13,
    MoveLine = 0x15,
    MoveJoint = 0x17,
    GetTcpPose = 0x29,
    GetJointPos = 0x2A
}

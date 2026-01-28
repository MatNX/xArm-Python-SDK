using System.Net.Sockets;

namespace XArm.Lite6;

public interface IArmTransport : IDisposable
{
    bool IsConnected { get; }
    void Connect(string host, int port, TimeSpan timeout);
    void Disconnect();
    byte[] SendAndReceive(byte[] request, TimeSpan timeout);
}

public sealed class TcpArmTransport : IArmTransport
{
    private TcpClient? _client;
    private NetworkStream? _stream;

    public bool IsConnected => _client?.Connected ?? false;

    public void Connect(string host, int port, TimeSpan timeout)
    {
        if (IsConnected)
        {
            return;
        }

        var client = new TcpClient();
        var connectTask = client.ConnectAsync(host, port);
        if (!connectTask.Wait(timeout))
        {
            client.Dispose();
            throw new TimeoutException($"Timed out connecting to {host}:{port}.");
        }

        _client = client;
        _stream = client.GetStream();
    }

    public void Disconnect()
    {
        _stream?.Dispose();
        _client?.Dispose();
        _stream = null;
        _client = null;
    }

    public byte[] SendAndReceive(byte[] request, TimeSpan timeout)
    {
        if (_stream is null)
        {
            throw new InvalidOperationException("Not connected.");
        }

        _stream.WriteTimeout = (int)timeout.TotalMilliseconds;
        _stream.ReadTimeout = (int)timeout.TotalMilliseconds;

        _stream.Write(request, 0, request.Length);
        _stream.Flush();

        var header = ReadExact(7);
        var length = (header[4] << 8) | header[5];
        var remaining = Math.Max(0, length - 1);
        var payload = remaining > 0 ? ReadExact(remaining) : Array.Empty<byte>();

        var response = new byte[header.Length + payload.Length];
        Buffer.BlockCopy(header, 0, response, 0, header.Length);
        if (payload.Length > 0)
        {
            Buffer.BlockCopy(payload, 0, response, header.Length, payload.Length);
        }

        return response;
    }

    private byte[] ReadExact(int count)
    {
        if (_stream is null)
        {
            throw new InvalidOperationException("Not connected.");
        }

        var buffer = new byte[count];
        var readTotal = 0;
        while (readTotal < count)
        {
            var read = _stream.Read(buffer, readTotal, count - readTotal);
            if (read == 0)
            {
                throw new IOException("Connection closed while reading data.");
            }

            readTotal += read;
        }

        return buffer;
    }

    public void Dispose()
    {
        Disconnect();
    }
}

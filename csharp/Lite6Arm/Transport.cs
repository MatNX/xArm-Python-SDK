using System.Net.Sockets;
using System.Text;

namespace XArm.Lite6;

public interface IArmTransport : IDisposable
{
    bool IsConnected { get; }
    void Connect(string host, int port, TimeSpan timeout);
    void Disconnect();
    string SendCommand(string command, TimeSpan timeout);
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

    public string SendCommand(string command, TimeSpan timeout)
    {
        if (_stream is null)
        {
            throw new InvalidOperationException("Not connected.");
        }

        var payload = Encoding.ASCII.GetBytes(command + "\n");
        _stream.Write(payload, 0, payload.Length);
        _stream.Flush();

        _stream.ReadTimeout = (int)timeout.TotalMilliseconds;
        using var buffer = new MemoryStream();
        var temp = new byte[256];
        int read;
        do
        {
            read = _stream.Read(temp, 0, temp.Length);
            if (read > 0)
            {
                buffer.Write(temp, 0, read);
            }
        } while (read > 0 && !_stream.DataAvailable);

        return Encoding.ASCII.GetString(buffer.ToArray()).Trim();
    }

    public void Dispose()
    {
        Disconnect();
    }
}

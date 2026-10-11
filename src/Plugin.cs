
using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using FanControl.Plugins;

namespace FanControl.ArduinoInterface
{
    public class ArduinoPlugin : IPlugin, IDisposable
    {
        private SerialPort _serial;
        private ArduinoFanControlSensor[] _fans;
        private Timer _reconnectTimer;

        private readonly object _serialLock = new object();

        private const int BaudRate = 115200;
        private const int ReadTimeoutMs = 1500;
        private const int WriteTimeoutMs = 1500;
        private const int StartupDelayMs = 2000;
        private const int MaxAttempts = 3;

        public string Name => "Arduino Auto Fan Controller";

        public void Initialize()
        {
            Log("Plugin initialization started.");

            DetectAndConnectArduino();

            _reconnectTimer = new Timer(
                _ => CheckReconnect(),
                null,
                5000,
                5000
            );
        }

        public void Load(IPluginSensorsContainer container)
        {
            lock (_serialLock)
            {
                if (_fans == null)
                {
                    Log("No fans available during Load.");
                    return;
                }

                foreach (var fan in _fans)
                {
                    container.ControlSensors.Add(fan);
                }

                Log(_fans.Length + " fan sensors loaded.");
            }
        }

        private void DetectAndConnectArduino()
        {
            lock (_serialLock)
            {
                string port = ReadArduinoPortFromFile();

                if (string.IsNullOrWhiteSpace(port))
                {
                    Log("Arduino port not configured.");
                    return;
                }

                if (_serial != null && _serial.IsOpen)
                    return;

                for (int attempt = 1; attempt <= MaxAttempts; attempt++)
                {
                    SerialPort candidate = null;

                    try
                    {
                        Log("Connection attempt " + attempt +
                            "/" + MaxAttempts + " on " + port);

                        candidate = new SerialPort(port, BaudRate)
                        {
                            DtrEnable = true,
                            NewLine = "\n",
                            ReadTimeout = ReadTimeoutMs,
                            WriteTimeout = WriteTimeoutMs
                        };

                        candidate.Open();

                        // Arduino may reset when the COM port opens.
                        Thread.Sleep(StartupDelayMs);

                        candidate.DiscardInBuffer();
                        candidate.DiscardOutBuffer();

                        // Request fan count.
                        candidate.WriteLine("NUM_FANS");

                        string response = candidate.ReadLine().Trim();

                        int fanCount;
                        if (!int.TryParse(response, out fanCount) ||
                            fanCount < 1 || fanCount > 8)
                        {
                            throw new IOException(
                                "Invalid fan count: " + response
                            );
                        }

                        Log("Arduino reports " + fanCount + " fans.");

                        // Request fan names.
                        string[] fanNames = new string[fanCount];

                        candidate.WriteLine("FAN_NAMES");

                        for (int i = 0; i < fanCount; i++)
                        {
                            fanNames[i] = candidate.ReadLine().Trim();

                            if (string.IsNullOrWhiteSpace(fanNames[i]))
                            {
                                throw new IOException(
                                    "Empty name for fan " + (i + 1)
                                );
                            }
                        }

                        // The connection is now validated.
                        _serial = candidate;

                        // Create sensors only once.
                        if (_fans == null)
                        {
                            _fans = new ArduinoFanControlSensor[fanCount];

                            for (int i = 0; i < fanCount; i++)
                            {
                                _fans[i] = new ArduinoFanControlSensor(
                                    "arduino_fan_" + (i + 1),
                                    fanNames[i],
                                    i + 1,
                                    () => _serial,
                                    _serialLock
                                );
                            }
                        }
                        else if (_fans.Length != fanCount)
                        {
                            Log("Warning: fan count changed. " +
                                "Restart FanControl to reload sensors.");
                        }

                        // Restore the last requested speeds.
                        foreach (var fan in _fans)
                        {
                            fan.Resend();
                        }

                        Log("Arduino connected successfully.");
                        return;
                    }
                    catch (Exception ex)
                    {
                        Log("Connection attempt " + attempt +
                            " failed: " + ex.ToString());

                        try
                        {
                            candidate?.Close();
                            candidate?.Dispose();
                        }
                        catch { }

                        _serial = null;

                        if (attempt < MaxAttempts)
                            Thread.Sleep(750);
                    }
                }

                Log("All connection attempts failed.");
            }
        }

        private string ReadArduinoPortFromFile()
        {
            try
            {
                string filePath = Path.Combine(
                    AppDomain.CurrentDomain.BaseDirectory,
                    "Plugins",
                    "arduino_port.txt"
                );

                if (!File.Exists(filePath))
                {
                    Log("Missing configuration file: " + filePath);
                    return null;
                }

                string port = File.ReadAllText(filePath).Trim();

                return string.IsNullOrWhiteSpace(port)
                    ? null
                    : port;
            }
            catch (Exception ex)
            {
                Log("Port configuration error: " + ex.Message);
                return null;
            }
        }

        private void CheckReconnect()
        {
            try
            {
                lock (_serialLock)
                {
                    if (_serial == null || !_serial.IsOpen)
                    {
                        DetectAndConnectArduino();
                    }
                }
            }
            catch (Exception ex)
            {
                Log("Reconnect error: " + ex.Message);
            }
        }

        private void Log(string message)
        {
            try
            {
                string logPath = Path.Combine(
                    AppDomain.CurrentDomain.BaseDirectory,
                    "Plugins",
                    "ArduinoInterface.log"
                );

                File.AppendAllText(
                    logPath,
                    DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") +
                    " | " + message + Environment.NewLine
                );
            }
            catch
            {
                // Logging must never crash FanControl.
            }
        }

        public void Close()
        {
            _reconnectTimer?.Dispose();

            lock (_serialLock)
            {
                try
                {
                    _serial?.Close();
                    _serial?.Dispose();
                }
                catch { }

                _serial = null;
            }
        }

        public void Dispose()
        {
            Close();
        }
    }

    public class ArduinoFanControlSensor : IPluginControlSensor
    {
        public string Id { get; }
        public string Name { get; }
        public float? Value { get; private set; }

        private readonly int _fanNumber;
        private readonly Func<SerialPort> _getSerial;
        private readonly object _serialLock;

        public ArduinoFanControlSensor(
            string id,
            string name,
            int fanNumber,
            Func<SerialPort> getSerial,
            object serialLock)
        {
            Id = id;
            Name = name;
            _fanNumber = fanNumber;
            _getSerial = getSerial;
            _serialLock = serialLock;

            Value = 30;
        }

        public void Reset()
        {
            Set(30);
        }

        public void Update()
        {
        }

        public void Set(float val)
        {
            // Limit the requested speed to 0-100%.
            if (float.IsNaN(val) || float.IsInfinity(val))
                return;

            val = Math.Max(0f, Math.Min(100f, val));

            Value = val;

            SendSpeed(val);
        }

        public void Resend()
        {
            SendSpeed(Value ?? 30f);
        }

        private void SendSpeed(float val)
        {
            lock (_serialLock)
            {
                SerialPort serial = _getSerial();

                if (serial == null || !serial.IsOpen)
                    return;

                try
                {
                    int pwm = (int)Math.Round(val * 255f / 100f);

                    serial.WriteLine(
                        "SET_FAN:" + _fanNumber + ":" + pwm
                    );
                }
                catch
                {
                    // Reconnection is handled by the plugin.
                }
            }
        }
    }
}

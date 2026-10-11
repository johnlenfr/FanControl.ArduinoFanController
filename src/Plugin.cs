using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using Microsoft.Win32;
using FanControl.Plugins;

namespace FanControl.ArduinoInterface
{
    public class ArduinoPlugin : IPlugin, IDisposable
    {
        private SerialPort _serial;
        private ArduinoFanControlSensor[] _fans;
        private ArduinoRpmSensor[] _rpmSensors;
        private Timer _keepAliveTimer;
        private Timer _reconnectTimer;

        private readonly object _serialLock = new object();

        private const int BaudRate = 115200;
        private const int ReadTimeoutMs = 1500;
        private const int WriteTimeoutMs = 1500;
        private const int StartupDelayMs = 2000;
        private const int MaxAttempts = 3;

        // Windows USB identifiers for the Arduino Uno tested on this PC.
        private const string ArduinoUsbId = "VID_2341&PID_0043";
        private const string ArduinoUsbSerial = "64938333932351503152";

        public string Name => "Arduino Auto Fan Controller";

        public void Initialize()
        {
            Log("Plugin initialization started.");
            DetectAndConnectArduino();
            _reconnectTimer = new Timer(_ => CheckReconnect(), null, 5000, 5000);
            // The Arduino switches to 50% after five seconds without a command.
            _keepAliveTimer = new Timer(_ => KeepAlive(), null, 2000, 2000);
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
                    container.ControlSensors.Add(fan);

                if (_rpmSensors != null)
                    foreach (var sensor in _rpmSensors)
                        container.FanSensors.Add(sensor);

                Log(_fans.Length + " controls and " + (_rpmSensors == null ? 0 : _rpmSensors.Length) + " RPM sensors loaded.");
            }
        }

        private void DetectAndConnectArduino()
        {
            lock (_serialLock)
            {
                if (_serial != null && _serial.IsOpen)
                    return;

                string port = FindArduinoPort();
                if (string.IsNullOrWhiteSpace(port))
                {
                    Log("Arduino port not found.");
                    return;
                }

                for (int attempt = 1; attempt <= MaxAttempts; attempt++)
                {
                    SerialPort candidate = null;
                    try
                    {
                        Log("Connection attempt " + attempt + "/" + MaxAttempts + " on " + port);
                        candidate = new SerialPort(port, BaudRate)
                        {
                            DtrEnable = true,
                            NewLine = "\n",
                            ReadTimeout = ReadTimeoutMs,
                            WriteTimeout = WriteTimeoutMs
                        };
                        candidate.Open();

                        // Opening a COM port commonly resets an Arduino Uno.
                        Thread.Sleep(StartupDelayMs);
                        candidate.DiscardInBuffer();
                        candidate.DiscardOutBuffer();

                        candidate.WriteLine("NUM_FANS");
                        string response = candidate.ReadLine().Trim();
                        int fanCount;
                        if (!int.TryParse(response, out fanCount) || fanCount < 1 || fanCount > 8)
                            throw new IOException("Invalid fan count: " + response);

                        Log("Arduino reports " + fanCount + " fans.");

                        string[] fanNames = new string[fanCount];
                        candidate.WriteLine("FAN_NAMES");
                        for (int i = 0; i < fanCount; i++)
                        {
                            fanNames[i] = candidate.ReadLine().Trim();
                            if (string.IsNullOrWhiteSpace(fanNames[i]))
                                throw new IOException("Empty name for fan " + (i + 1));
                        }

                        _serial = candidate;

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
                            Log("Warning: fan count changed. Restart FanControl to reload sensors.");
                        }

                        if (_rpmSensors == null)
                        {
                            _rpmSensors = new ArduinoRpmSensor[]
                            {
                                new ArduinoRpmSensor("arduino_rpm_1", "Arduino FAN 1 RPM", 1, () => _serial, _serialLock),
                                new ArduinoRpmSensor("arduino_rpm_2", "Arduino FAN 2 RPM", 2, () => _serial, _serialLock)
                            };
                        }
                        foreach (var fan in _fans)
                            fan.Resend();

                        Log("Arduino connected successfully.");
                        return;
                    }
                    catch (Exception ex)
                    {
                        Log("Connection attempt " + attempt + " failed: " + ex);
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

        // Writable user directory, including when FanControl is installed in Program Files.
        private static string UserDataDirectory => Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "FanControl.ArduinoInterface"
        );

        private string FindArduinoPort()
        {
            string autoPort = DetectArduinoUsbPort();
            if (!string.IsNullOrWhiteSpace(autoPort))
            {
                Log("USB auto-detection: " + autoPort);
                return autoPort;
            }

            try
            {
                // Only the per-user fallback port file is supported.
                // Never read a legacy file from the FanControl Plugins directory.
                string path = Path.Combine(UserDataDirectory, "arduino_port.txt");
                if (!File.Exists(path))
                {
                    Log("No USB Arduino found and no fallback port file at " + path);
                    return null;
                }

                string port = File.ReadAllText(path).Trim();
                if (string.IsNullOrWhiteSpace(port))
                {
                    Log("Configured port file is empty: " + path);
                    return null;
                }

                // Fail early on stale configurations, rather than trying an
                // unavailable COM port repeatedly.
                if (!Array.Exists(SerialPort.GetPortNames(),
                    p => string.Equals(p, port, StringComparison.OrdinalIgnoreCase)))
                {
                    Log("Configured COM port is not available: " + port + " (" + path + ")");
                    return null;
                }

                Log("Using configured fallback port: " + port + " from " + path);
                return port;
            }
            catch (Exception ex)
            {
                Log("Port configuration error: " + ex.Message);
                return null;
            }
        }

        private string DetectArduinoUsbPort()
        {
            try
            {
                string registryPath =
                    @"SYSTEM\CurrentControlSet\Enum\USB\" +
                    ArduinoUsbId + @"\" + ArduinoUsbSerial +
                    @"\Device Parameters";

                using (RegistryKey key = Registry.LocalMachine.OpenSubKey(registryPath))
                {
                    if (key == null)
                    {
                        Log("Arduino USB registry entry not found.");
                        return null;
                    }

                    string port = key.GetValue("PortName") as string;
                    if (string.IsNullOrWhiteSpace(port))
                    {
                        Log("No PortName in Arduino USB registry entry.");
                        return null;
                    }

                    // The registry may retain devices that are not connected.
                    bool available = Array.Exists(
                        SerialPort.GetPortNames(),
                        p => string.Equals(p, port, StringComparison.OrdinalIgnoreCase)
                    );

                    if (!available)
                    {
                        Log("Arduino USB port is not currently available: " + port);
                        return null;
                    }

                    Log("Arduino Uno detected: " + port);
                    return port;
                }
            }
            catch (Exception ex)
            {
                Log("USB detection error: " + ex.Message);
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
                        DetectAndConnectArduino();
                }
            }
            catch (Exception ex)
            {
                Log("Reconnect error: " + ex.Message);
            }
        }

        private void KeepAlive()
        {
            try
            {
                lock (_serialLock)
                {
                    if (_serial == null || !_serial.IsOpen || _fans == null) return;
                    foreach (var fan in _fans) fan.Resend();
                }
            }
            catch (Exception ex) { Log("Keep-alive error: " + ex.Message); }
        }

        private void Log(string message)
        {
            try
            {
                Directory.CreateDirectory(UserDataDirectory);
                string path = Path.Combine(UserDataDirectory, "ArduinoInterface.log");
                File.AppendAllText(
                    path,
                    DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") +
                    " | " + message + Environment.NewLine
                );
            }
            catch
            {
                // Logging must never prevent the fan controller from starting.
            }
        }

        public void Close()
        {
            _reconnectTimer?.Dispose();
            _keepAliveTimer?.Dispose();
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

        public void Dispose() => Close();
    }

    public class ArduinoFanControlSensor : IPluginControlSensor2
    {
        public string Id { get; }
        public string Name { get; }
        public float? Value { get; private set; }

        // Match exact Id values of the RPM sensors created by this plugin.
        // Channels 3-5 have no TACH input, so remain unpaired.
        public string PairedFanSensorId { get; }

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
            PairedFanSensorId = fanNumber == 1 ? "arduino_rpm_1"
                              : fanNumber == 2 ? "arduino_rpm_2"
                              : null;
            _getSerial = getSerial;
            _serialLock = serialLock;
            Value = 30;
        }

        public void Reset() => Set(30);
        public void Update() { }

        public void Set(float val)
        {
            if (float.IsNaN(val) || float.IsInfinity(val))
                return;
            val = Math.Max(0f, Math.Min(100f, val));
            Value = val;
            SendSpeed(val);
        }

        public void Resend() => SendSpeed(Value ?? 30f);

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
                    serial.WriteLine("SET_FAN:" + _fanNumber + ":" + pwm);
                }
                catch
                {
                    // Existing reconnect timer handles closed ports.
                }
            }
        }
    }
    // Read-only FanControl RPM sensor. Only channels 1 and 2 have TACH inputs.
    public class ArduinoRpmSensor : IPluginSensor
    {
        public string Id { get; }
        public string Name { get; }
        public float? Value { get; private set; }

        private readonly int _number;
        private readonly Func<SerialPort> _serialProvider;
        private readonly object _lock;
        private long _lastPollTicks;

        public ArduinoRpmSensor(string id, string name, int number,
            Func<SerialPort> serialProvider, object serialLock)
        {
            Id = id;
            Name = name;
            _number = number;
            _serialProvider = serialProvider;
            _lock = serialLock;
            Value = null;
        }

        public void Update()
        {
            // FanControl calls Update frequently; limit serial traffic to ~1 Hz.
            long now = DateTime.UtcNow.Ticks;
            if (now - _lastPollTicks < TimeSpan.TicksPerSecond) return;
            lock (_lock)
            {
                now = DateTime.UtcNow.Ticks;
                if (now - _lastPollTicks < TimeSpan.TicksPerSecond) return;
                _lastPollTicks = now;
                SerialPort port = _serialProvider();
                if (port == null || !port.IsOpen) { Value = null; return; }
                try
                {
                    // All read/write access shares the same lock as SET_FAN.
                    port.WriteLine("GET_RPM:" + _number);
                    string line = port.ReadLine().Trim();
                    string prefix = "RPM:" + _number + ":";
                    if (!line.StartsWith(prefix, StringComparison.Ordinal))
                        throw new IOException("Unexpected RPM response: " + line);
                    float parsed;
                    if (!float.TryParse(line.Substring(prefix.Length),
                        System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture,
                        out parsed) || parsed < 0 || parsed > 20000)
                        throw new IOException("Invalid RPM response: " + line);
                    Value = parsed;
                }
                catch (Exception)
                {
                    Value = null;
                }
            }
        }
    }

}

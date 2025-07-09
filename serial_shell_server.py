import serial
import subprocess

SERIAL_PORT = '/dev/ttyUSB0'  # Orin'deki RFD900x portu
BAUDRATE = 57600              # Telemetri baudrate

def run_shell_command(cmd):
    try:
        # Komutu çalıştır, çıktı ve hatayı yakala
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        output = result.stdout + result.stderr
        if not output:
            output = "(Komut çıktı vermedi)"
        return output
    except Exception as e:
        return f"Hata: {str(e)}"

def main():
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        print(f"{SERIAL_PORT} üzerinde dinleme başladı...")
        buffer = ''
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode(errors='ignore')
                buffer += data

                # Komut satırı sonu '\n' geldiğinde komutu çalıştır
                if '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        print(f"Gelen komut: {line}")
                        output = run_shell_command(line)
                        # Sonucu seri porta gönder
                        ser.write(output.encode() + b'\n')
                        print(f"Gönderilen çıktı:\n{output}")

if __name__ == '__main__':
    main()

# MAJAAAM LOGGER — NTC + rõhk + IMU + QMC5883P + Δh (lihtne lõppvariant)
# - COM4, START
# - eemaldab ainult ms-loenduri
# - leiab rõhu (900..1100 hPa) ja hoiab alles VIIMASE temp-sarnase väärtuse enne rõhku (NTC)
# - väljund algab: [NTC kui leitud], pressure, IMU, QMC5883P, ..., delta_h_m
# - Δh = (P0 - P) * 8.3 (m/hPa), P0 võetakse esimeselt healt rõhurealt
# - ekraan + CSV töölauale

import serial
import time
import os

PORT = "COM4"
BAUD = 115200
ENC = "utf-8"

CSV_PATH = os.path.join(os.path.expanduser("~"),
                        "Desktop",
                        "maajaam_telemeetria_delta_simple.csv")

# NTC realistlik vahemik (°C). Vajadusel kohenda.
NTC_MIN_C = -40.0
NTC_MAX_C = 125.0

def ts():
    t = time.localtime()
    ms = int((time.time() % 1) * 1000)
    return time.strftime("%Y/%m/%d %H:%M:%S", t) + f".{ms:03d}"

def find_csv_start(line: str) -> int:
    for i in range(len(line)):
        if line[i].isdigit() and ("," in line[i:]):
            return i
    return -1

def clean_and_extract(line: str):
    """
    1) Leia numbrite algus, split CSV.
    2) Eemalda esimene väli (ms-loendur).
    3) Kuni esimese rõhuni (900..1100 hPa) pea meeles VIIMANE temp-sarnane väärtus → NTC.
    4) Tagasta (telemetry_str, pressure_hpa).
       Telemetry algab kas "NTC,pressure,..." või "pressure,...", kui NTC-d ei leitud.
    """
    s = find_csv_start(line)
    if s < 0:
        return "", None

    parts = line[s:].strip().split(",")
    if len(parts) < 2:
        return "", None

    # 1) eemalda ms
    parts = parts[1:]

    ntc_val = None
    p_idx = None

    for idx, val in enumerate(parts):
        try:
            v = float(val)
        except:
            continue

        # rõhk?
        if 900.0 <= v <= 1100.0:
            p_idx = idx
            break

        # temp-kandidaat (enne rõhku)
        if NTC_MIN_C <= v <= NTC_MAX_C:
            ntc_val = v  # viimase temp-sarnase jätame meelde

    if p_idx is None:
        return "", None

    out_fields = []
    if ntc_val is not None:
        out_fields.append(f"{ntc_val:.2f}")   # NTC esimeseks
    out_fields.extend(parts[p_idx:])          # rõhk ja ülejäänud (IMU, QMC5883P, ...)

    tele = ",".join(out_fields)
    try:
        pressure_hpa = float(parts[p_idx])
    except:
        pressure_hpa = None

    return tele, pressure_hpa

def main():
    print("=== MAJAAAM NTC + Δh LOGGER (final) ===")
    print("Avan pordi:", PORT)

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.1,
        write_timeout=0.1,
        rtscts=False,
        dsrdtr=False
    )
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass

    print("[OK] Port avatud.")
    print("-> START")
    ser.write(b"START\r\n")
    ser.flush()

    # CSV päis
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", encoding=ENC) as f:
            f.write("pc_time;telemetry;delta_h_m\n")

    buf = bytearray()
    p0 = None
    last_idle = time.time()

    while True:
        chunk = ser.read(256)
        now = time.time()

        if chunk:
            buf.extend(chunk)
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break

                raw = buf[:i+1]
                del buf[:i+1]

                line = raw.decode(ENC, errors="replace").strip()
                tele, pressure_hpa = clean_and_extract(line)
                if not tele or pressure_hpa is None:
                    continue

                if p0 is None:
                    p0 = pressure_hpa
                    print(f"[INFO] Baasrõhk P0 seatud: {p0:.2f} hPa")

                delta_h = (p0 - pressure_hpa) * 8.3  # m

                stamp = ts()
                out_line = f"{stamp} | {tele},{delta_h:.2f}"
                print(out_line)

                with open(CSV_PATH, "a", encoding=ENC) as f:
                    f.write(f"{stamp};{tele};{delta_h:.2f}\n")

        else:
            if now - last_idle > 5:
                print("(vaikus)")
                last_idle = now

if __name__ == "__main__":
    main()
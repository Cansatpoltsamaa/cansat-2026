# -*- coding: utf-8 -*-
"""
AUTOMAATNE PC KELLA SAATMINE + MAJAAMA LOGIJA
------------------------------------------------
Teeb kolm asja:
 1. Avab maajaama COM pordi
 2. Saadab INIMELOETAVA PC KELLA CanSat'ile:
       YYYY/MM/DD HH:MM:SS.mmm
 3. Logib telemeetriat TXT ja CSV failidesse
"""

import os
import time
from datetime import datetime
import serial

# -------------------- SÄTTED --------------------
BASE_DIR = r"C:\Users\Kasutaja\Desktop\Maajaam 2026\maajaam_final_2026"
PORT = "COM4"
BAUD = 115200
ENC = "utf-8"
# -------------------------------------------------

def pc_timestamp():
    """Tagastab PC aja kujul: YYYY/MM/DD HH:MM:SS.mmm"""
    t = time.localtime()
    ms = int((time.time() % 1) * 1000)
    return time.strftime("%Y/%m/%d %H:%M:%S", t) + f".{ms:03d}"

def make_paths():
    os.makedirs(BASE_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = f"maajaam_{ts}"
    txt_path = os.path.join(BASE_DIR, base + ".txt")
    csv_path = os.path.join(BASE_DIR, base + ".csv")
    return txt_path, csv_path

def main():
    print(f"Üritan avada {PORT} @ {BAUD} ...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print("VIGA: Porti ei saa avada ->", e)
        return

    print("OK – port avatud.")
    time.sleep(1.0)  # anna ESP32-le aega

    # ---- 1) SAADA PC KELL MAJAAMASSE ----
    ts = pc_timestamp()
    ser.write((ts + "\n").encode(ENC))
    print(f"[PC->GS] Saadetud kell: {ts}")

    # ---- 2) VALMISTA LOGIFAILID ----
    txt_path, csv_path = make_paths()
    print(f"TXT logi -> {txt_path}")
    print(f"CSV logi -> {csv_path}")
    print("Telemeetria algab...\n")

    with open(txt_path, "w", encoding=ENC) as txt_f, \
         open(csv_path, "w", encoding=ENC) as csv_f:

        csv_f.write("local_time;epoch_ms;raw\n")
        csv_f.flush()

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode(ENC, errors="ignore").rstrip("\n")

                now_local = datetime.now().isoformat(timespec="seconds")
                epoch_ms = int(time.time() * 1000)

                print(f"{now_local} | {line}")

                txt_f.write(f"{now_local} [{epoch_ms}] {line}\n")
                txt_f.flush()

                csv_f.write(f"{now_local};{epoch_ms};{line}\n")
                csv_f.flush()

        except KeyboardInterrupt:
            print("\nLogimine lõpetatud (Ctrl+C).")
            ser.close()

if __name__ == "__main__":
    main()
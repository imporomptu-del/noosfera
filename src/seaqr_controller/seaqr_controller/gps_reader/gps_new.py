import serial
import time
import pynmea2
import csv
import os
from datetime import datetime, timezone, date

# ==== CONFIG ====
SERIAL_PORT = "/dev/ttyTHS1"   # твій порт
BAUD_RATE   = 9600
OUTPUT_DIR  = "/home/a/Projects/ros2_ws/data/gps/"
ROTATE_SEC  = 60               # ротація файлу щохвилини
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ==== Helpers ====
def utc_now_iso():
    return datetime.now(timezone.utc).isoformat()

def new_filename():
    ts = datetime.now(timezone.utc).strftime('%Y_%m_%dT%H_%M_%S_%f')[:-3]
    return os.path.join(OUTPUT_DIR, f"gps_{ts}.csv")

def open_csv():
    f = open(new_filename(), "w", newline="")
    w = csv.writer(f)
    w.writerow([
        "timestamp",            # ISO-UTC час запису (момент надходження)
        "sentence_type",        # останній тип речення, що тригерив запис
        "latitude",             # десяткові градуси (float або "")
        "longitude",            # десяткові градуси (float або "")
        "altitude_m",           # метри
        "speed_knots",          # вузли
        "course_degrees",       # 0..360
        "num_satellites",       # ціле
        "hdop",                 # float
        "raw_nmea"              # сире речення що тригерило запис
    ])
    print(f"Created new GPS log file: {f.name}")
    return f, w, time.time()

def safe_float(x, nd=None):
    if x in ("", None):
        return ""
    try:
        v = float(x)
        if nd is not None:
            return round(v, nd)
        return v
    except Exception:
        return ""

def should_rotate(start_ts):
    return (time.time() - start_ts) >= ROTATE_SEC

# ==== Runtime state ====
# Ми зберігаємо останні значення, щоби заповнювати рядок, коли приходить валідний фікс
state = {
    "lat": "",
    "lon": "",
    "alt": "",
    "num_sats": "",
    "hdop": "",
    "speed_kn": "",
    "course_deg": "",
    "rmc_status": "",     # 'A' або 'V'
    "rmc_date": None,     # datetime.date від RMC (для прив’язки часу)
}

# Open serial and CSV
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
current_file, writer, file_start = open_csv()

def write_row(trigger_sentence_type, raw_line):
    """Пише один нормалізований рядок, якщо є хоч щось корисне."""
    row = [
        utc_now_iso(),
        trigger_sentence_type,
        state["lat"],
        state["lon"],
        state["alt"],
        state["speed_kn"],
        state["course_deg"],
        state["num_sats"],
        state["hdop"],
        raw_line
    ]
    writer.writerow(row)
    current_file.flush()

def handle_gga(msg, raw):
    # msg.latitude / msg.longitude у pynmea2 вже в десяткових градусах або ''.
    state["lat"] = safe_float(msg.latitude, 7)
    state["lon"] = safe_float(msg.longitude, 7)
    state["alt"] = safe_float(getattr(msg, "altitude", ""), 2)
    state["num_sats"] = safe_float(getattr(msg, "num_sats", ""), 0)
    state["hdop"] = safe_float(getattr(msg, "horizontal_dil", ""), 2)

    # Якість фіксу: 0 = немає; >0 = є
    fix_quality = safe_float(getattr(msg, "gps_qual", ""))
    if fix_quality != "" and fix_quality > 0:
        write_row("GGA", raw)

def handle_rmc(msg, raw):
    # Статус: A = valid, V = void
    state["rmc_status"] = getattr(msg, "status", "") or ""
    # Координати, якщо прийшли в RMC — оновлюємо
    if getattr(msg, "latitude", "") not in ("", None):
        state["lat"] = safe_float(msg.latitude, 7)
    if getattr(msg, "longitude", "") not in ("", None):
        state["lon"] = safe_float(msg.longitude, 7)

    # Швидкість/курс
    state["speed_kn"] = safe_float(getattr(msg, "spd_over_grnd", ""), 3)
    state["course_deg"] = safe_float(getattr(msg, "true_course", ""), 2)

    # Дата з RMC (DDMMYY)
    rmc_dat = getattr(msg, "datestamp", None)
    if isinstance(rmc_dat, date):
        state["rmc_date"] = rmc_dat

    if state["rmc_status"] == "A":  # валідний фікс
        write_row("RMC", raw)

try:
    while True:
        if should_rotate(file_start):
            current_file.close()
            current_file, writer, file_start = open_csv()

        raw = ser.readline().decode("ascii", errors="replace").strip()
        if not raw:
            continue

        # Друкуй для дебагу за потреби
        # print("Raw NMEA:", raw)

        if not raw.startswith("$"):
            continue

        try:
            msg = pynmea2.parse(raw)
        except pynmea2.ParseError:
            continue

        stype = getattr(msg, "sentence_type", "").upper()

        if stype == "GGA":
            handle_gga(msg, raw)
        elif stype == "RMC":
            handle_rmc(msg, raw)
        else:
            # Інші повідомлення (GSA/GSV/..) для якості моніторингу,
            # але в таблицю не пишемо, поки немає валідного фіксу.
            pass

        # невелика пауза, аби не грузити CPU
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    try:
        current_file.close()
    except Exception:
        pass
    try:
        ser.close()
    except Exception:
        pass

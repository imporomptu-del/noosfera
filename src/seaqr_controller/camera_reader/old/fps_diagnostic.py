import zwoasi as asi

ASI_LIB = "/usr/local/lib/libASICamera2.so"  # змініть, якщо інший шлях
asi.init(ASI_LIB)

cams = asi.list_cameras()
if not cams:
    raise SystemExit("SDK не бачить ZWO камери.")
print("Камери:", cams)

cam = asi.Camera(0)
props = cam.get_camera_property()
print("\n=== Властивості камери ===")
for k,v in props.items():
    print(f"{k}: {v}")

# Підтримувані типи зображень ≈ формати (аналог CAPS)
types = []
candidates = (asi.ASI_IMG_RAW8, asi.ASI_IMG_RAW16, asi.ASI_IMG_Y8, asi.ASI_IMG_Y16, asi.ASI_IMG_RGB24)
names = {asi.ASI_IMG_RAW8:"RAW8", asi.ASI_IMG_RAW16:"RAW16", asi.ASI_IMG_Y8:"Y8", asi.ASI_IMG_Y16:"Y16", asi.ASI_IMG_RGB24:"RGB24"}
for t in candidates:
    try:
        cam.set_image_type(t)
        types.append(names[t])
    except Exception:
        pass
print("\nФормати (Image Types):", types)

w, h = props['MaxWidth'], props['MaxHeight']
print(f"\nМакс. роздільність: {w}x{h}")

print("\n=== Контролі ===")
for c in cam.get_controls():
    rw = "rw" if c['IsWritable'] else "ro"
    print(f"{c['Name']:25} {rw}  range=({c['MinValue']},{c['MaxValue']})  default={c['DefaultValue']}")

# Швидкий тест відео
cam.set_control_value(asi.ASI_EXPOSURE, 5000)   # 5 ms
cam.set_control_value(asi.ASI_GAIN, 100)
cam.set_image_type(asi.ASI_IMG_RAW8)
cam.set_roi_format(w, h, 1, asi.ASI_IMG_RAW8)   # повний кадр, bin=1
cam.start_video_capture()
buf = cam.get_video_data(timeout=2000)
print(f"\nОк. Отримано байт кадру: {len(buf)}")
cam.stop_video_capture()
cam.close()


#!/home/a/ros2/bin python3
from  yaml import safe_load as safe_load
class ConfigLoader:
    def __init__(self, config_path):
        self.config = self.load_yaml_config(config_path)

    def load_yaml_config(self, path):
        with open(path, 'r') as file:
            return safe_load(file)

    def get(self, property_path, default=None):
        keys = property_path.split('.')
        value = self.config
        try:
            for key in keys:
                value = value[key]
            return value
        except KeyError:
            return default
        
    def format_camera_configs(configs, caps_lookup=None):
        """
        configs: list like [{'id': 0, 'name': '...', 'settings': {...}}, ...]
        caps_lookup (optional): dict -> {camera_name_or_id: {setting: (min, max, default)}}
        If not given, we omit (Min/Max/Default).
        """
        lines = []
        for cam in configs:
            cam_id = cam.get("id", "?")
            name = cam.get("name", f"Camera {cam_id}")
            settings = cam.get("settings", {})
            lines.append(f"📷 {name} (id: {cam_id})")

            # choose capability map by name first, then id, else None
            caps = None
            if caps_lookup:
                caps = caps_lookup.get(name) or caps_lookup.get(cam_id)

            # We’ll print settings in a stable order: alpha, but prioritize common keys
            priority = ["Gain", "Exposure", "WB_R", "WB_B", "Offset", "BandWidth",
                        "Flip", "AutoExpMaxGain", "AutoExpMaxExpMS",
                        "AutoExpTargetBrightness", "HighSpeedMode"]
            ordered = sorted(settings.items(), key=lambda kv: (priority.index(kv[0]) if kv[0] in priority else 999, kv[0]))

            for key, val in ordered:
                if caps and key in caps:
                    mmin, mmax, default = caps[key]
                    lines.append(f"  • {key}: {val} (Min: {mmin}, Max: {mmax}, Default: {default})")
                else:
                    lines.append(f"  • {key}: {val}")
            lines.append("")  # blank line between cameras
        return "\n".join(lines).rstrip()    


    def get_cam_by_id(self, cam_name):
        for cam in self.config.get('cameras', []):
            if cam.get('name') == cam_name:
                return cam
        return None  # Camera name not found

    def get_camera_names(self):
        return [cam.get('name') for cam in self.config.get('cameras', []) if 'name' in cam]
    

# if __name__ == "__main__":
#     main()    
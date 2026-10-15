"""ffprobe helper for the video tests (None when ffprobe is missing)."""
import json, shutil, subprocess


def probe(path):
    exe = shutil.which("ffprobe")
    if not exe:
        return None
    r = subprocess.run([exe, "-v", "error", "-count_frames", "-select_streams", "v:0",
                        "-show_entries", "stream=codec_name,width,height,nb_read_frames",
                        "-of", "json", path], capture_output=True, text=True, check=True)
    s = json.loads(r.stdout)["streams"][0]
    return {"codec": s["codec_name"], "w": s["width"], "h": s["height"],
            "frames": int(s["nb_read_frames"])}

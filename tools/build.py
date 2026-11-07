"""Build only this workspace; never flash or modify reference projects."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

# profile -> (IDF target, profile defaults applied after sdkconfig.defaults)
# pre_v3/v3_0/v3_1 select the ESP32-P4 chip revision (Waveshare ESP32-P4-ETH);
# s31 selects the Espressif ESP32-S31-Function-CoreBoard-1.
PROFILES = {
    "pre_v3": ("esp32p4", "config/esp32p4_rev_pre_v3.defaults"),
    "v3_0": ("esp32p4", "config/esp32p4_rev_v3_0.defaults"),
    "v3_1": ("esp32p4", "config/esp32p4_rev_v3_1.defaults"),
    "s31": ("esp32s31", "config/esp32s31_function_coreboard_1.defaults"),
}
# ESP-IDF v6.1 still classifies these targets as preview targets.
PREVIEW_TARGETS = {"esp32s31"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=list(PROFILES), default="v3_1")
    parser.add_argument("--build-dir", default=None)
    parser.add_argument("--define", action="append", default=[])
    parser.add_argument("--reconfigure", action="store_true")
    args = parser.parse_args()
    env = dict(os.environ)
    idf = Path(env.get("IDF_PATH", "C:/esp/v6.1/esp-idf"))
    tools = Path(env.get("IDF_TOOLS_PATH", "C:/Espressif/tools"))
    python = Path(sys.executable)
    installation = tools / "eim_idf.json"
    if installation.exists():
        data = json.loads(installation.read_text())
        selected = next((x for x in data["idfInstalled"] if x["id"] == data["idfSelectedId"]), None)
        if selected:
            idf = Path(selected["path"])
            tools = Path(selected["idfToolsPath"])
            python = Path(selected["python"])
    installation_tools = tools
    if not (tools / "tools").is_dir() and (tools / "cmake").is_dir():
        tools = tools.parent
    env.update(IDF_PATH=str(idf), IDF_TOOLS_PATH=str(tools), IDF_PYTHON_ENV_PATH=str(python.parent.parent), PYTHONUTF8="1")
    result = subprocess.run([str(python), str(idf / "tools/idf_tools.py"), "export", "--format", "key-value"], env=env, capture_output=True, text=True)
    if result.returncode:
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        return result.returncode
    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            env[key] = value.replace("%PATH%", env.get("PATH", "")).replace("$PATH", env.get("PATH", ""))
    if list(installation_tools.glob("espidf.constraints.*.txt")):
        env["IDF_TOOLS_PATH"] = str(installation_tools)
    env["PATH"] = str(python.parent) + os.pathsep + env["PATH"]
    build_dir = ROOT / (args.build_dir or f"build_{args.profile}")
    if not build_dir.resolve().is_relative_to(ROOT):
        parser.error("build directory must be inside the ESP32PTPL3Follower workspace")
    target, profile_defaults = PROFILES[args.profile]
    defaults = str(ROOT / "sdkconfig.defaults") + ";" + str(ROOT / profile_defaults)
    command = [str(python), str(idf / "tools/idf.py")]
    if target in PREVIEW_TARGETS:
        command.append("--preview")
    command += ["-C", str(ROOT), "-B", str(build_dir), "-DIDF_TARGET=" + target, "-DSDKCONFIG=" + str(build_dir / "sdkconfig"), "-DSDKCONFIG_DEFAULTS=" + defaults]
    command.extend("-D" + value for value in args.define)
    command.append("reconfigure" if args.reconfigure else "build")
    return subprocess.call(command, env=env, cwd=ROOT)


if __name__ == "__main__":
    raise SystemExit(main())

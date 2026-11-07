import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


sys.dont_write_bytecode = True
from run_tests import ESP_CLANG, ROOT, execute, windows_environment


SCENARIOS = (
    "lifecycle",
    "metadata",
    "snapshots",
    "steps",
    "learned_frequency",
    "init_mutex_failure",
    "init_sdk_failure",
    "step_failure",
    "preempted_bridge",
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--compiler")
    parser.add_argument("--vcvars", type=Path)
    args = parser.parse_args()
    env = windows_environment(args.vcvars) if os.name == "nt" else os.environ.copy()
    compiler = args.compiler
    if not compiler:
        compiler = str(ESP_CLANG) if os.name == "nt" and ESP_CLANG.is_file() else (
            shutil.which("clang++", path=env.get("PATH")) or
            shutil.which("g++", path=env.get("PATH")) or
            shutil.which("cl.exe", path=env.get("PATH")))
    if not compiler:
        raise RuntimeError("No native C++ compiler found; supply --compiler.")
    if not args.compiler and compiler == str(ESP_CLANG):
        targets = subprocess.run([compiler, "--print-targets"], env=env,
                                 capture_output=True, text=True, check=True)
        if "x86-64" not in targets.stdout and "x86_64" not in targets.stdout:
            print("ESP Clang has no x86-64 backend; using native Visual Studio compiler.", flush=True)
            compiler = shutil.which("cl.exe", path=env.get("PATH"))
            if not compiler:
                raise RuntimeError("ESP Clang lacks x86-64 support and native cl.exe was not found.")
    msvc = Path(compiler).name.lower() in ("cl", "cl.exe")
    includes = [ROOT / "tests" / "clock_host", ROOT / "main"]
    if msvc:
        flags = ["/nologo", "/std:c++17", "/EHsc", "/W4", "/WX", "/permissive-", "/O2",
                 "/D_CRT_SECURE_NO_WARNINGS"]
        flags += [f"/I{path}" for path in includes]
    else:
        flags = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2"]
        flags += [f"-I{path}" for path in includes]
        if os.name == "nt":
            flags += ["--target=x86_64-pc-windows-msvc", "-D_CRT_SECURE_NO_WARNINGS"]
    sources = [ROOT / "tests" / "clock_tests.cpp", ROOT / "main" / "clock_owner.cpp",
               ROOT / "main" / "follower_core.cpp", ROOT / "tests" / "clock_host" / "clock_host.cpp"]
    failures = 0
    with tempfile.TemporaryDirectory(prefix="ESP32PTPL3Follower-clock-tests-") as temporary:
        build = Path(temporary)
        for name in ("clock_owner.cpp", "ESP32PTPL3FollowerSettings.h", "setting_types.h"):
            shutil.copyfile(ROOT / "main" / name, build / name)
        sources[1] = build / "clock_owner.cpp"
        # Exercise both supported targets' settings branches; clock behavior is target-neutral.
        for target in ("ESP32P4", "ESP32S31"):
            define = [f"/DCONFIG_IDF_TARGET_{target}=1"] if msvc else [f"-DCONFIG_IDF_TARGET_{target}=1"]
            executable = build / (f"clock_tests_{target.lower()}" + (".exe" if os.name == "nt" else ""))
            output = [f"/Fe:{executable}"] if msvc else ["-o", executable]
            print(f"Simulated target: {target}", flush=True)
            if execute([compiler, *flags, *define, *sources, *output], env, build):
                failures += 1
                continue
            for scenario in SCENARIOS:
                if execute([executable, scenario], env, build):
                    failures += 1
        print("SCOPE: Real clock owner/core with deterministic host clock and mutex stubs; no hardware, transport, crypto, or target timing claims.", flush=True)
    print(f"{'FAIL' if failures else 'PASS'}: {failures} failed build/test process(es)", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)

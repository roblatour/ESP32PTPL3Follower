import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
ESP_CLANG = Path(r"C:\Espressif\tools\esp-clang\esp-21.1.3_20260408\esp-clang\bin\clang++.exe")


def windows_environment(vcvars):
    if not vcvars:
        base = Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
        candidates = [base / "Microsoft Visual Studio" / "2022" / edition /
                      "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
                      for edition in ("Community", "Professional", "Enterprise", "BuildTools")]
        vcvars = next((path for path in candidates if path.is_file()), None)
        if not vcvars:
            vswhere = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
            if vswhere.is_file():
                result = subprocess.run([str(vswhere), "-latest", "-products", "*", "-requires",
                                         "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
                                        check=True, capture_output=True, text=True)
                install = result.stdout.strip()
                if install:
                    vcvars = Path(install) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    if not vcvars:
        if os.environ.get("VSCMD_ARG_TGT_ARCH") == "x64":
            return os.environ.copy()
        raise RuntimeError("Visual Studio x64 development environment not found; supply --vcvars.")
    command_shell = os.environ.get("COMSPEC", "cmd.exe")
    result = subprocess.run(f'"{command_shell}" /d /s /c "call "{vcvars}" >nul && set"',
                            check=False, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"Visual Studio initialization failed: {result.stdout}{result.stderr}")
    env = {key.upper(): value for key, value in os.environ.items()}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator and key:
            env[key.upper()] = value
    return env


def execute(command, env, cwd):
    print(subprocess.list2cmdline([str(part) for part in command]), flush=True)
    return subprocess.run([str(part) for part in command], env=env, cwd=cwd, check=False).returncode


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
    if msvc:
        flags = ["/nologo", "/std:c++17", "/EHsc", "/W4", "/WX", "/permissive-", "/O2", "/D_CRT_SECURE_NO_WARNINGS"]
    else:
        flags = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2"]
        if os.name == "nt":
            flags += ["--target=x86_64-pc-windows-msvc", "-D_CRT_SECURE_NO_WARNINGS"]
    failures = 0
    with tempfile.TemporaryDirectory(prefix="ESP32PTPL3Follower-tests-") as temporary:
        build = Path(temporary)
        for name, sources in (
            ("core_tests", [ROOT / "tests" / "core_tests.cpp", ROOT / "main" / "follower_core.cpp"]),
            ("udp_tests", [ROOT / "tests" / "udp_tests.cpp", ROOT / "main" / "follower_core.cpp"]),
        ):
            executable = build / (name + (".exe" if os.name == "nt" else ""))
            output = [f"/Fe:{executable}"] if msvc else ["-o", executable]
            if execute([compiler, *flags, *sources, *output], env, build):
                failures += 1
                continue
            cases = [["UTC0"], ["EST5EDT"]] if name == "core_tests" else [[]]
            if name == "core_tests" and os.name != "nt":
                cases.append(["EST5EDT,M3.2.0/2,M11.1.0/2"])
            for case in cases:
                if execute([executable, *case], env, build):
                    failures += 1
        if os.name == "nt":
            print("LIMITATION: Microsoft CRT does not support full POSIX TZ rules; UTC0 and EST5EDT were run in separate processes. Explicit-rule rejection is tested; newlib rule semantics require a POSIX/target run.", flush=True)
        print("SCOPE: UDP IPv4/IPv6 framing, PTP parsing, core, Sync rate, MAC validation and time conversions. Run run_clock_tests.py for clock-owner simulation. No hardware/accuracy claims.", flush=True)
    print(f"{'FAIL' if failures else 'PASS'}: {failures} failed build/test process(es)", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)

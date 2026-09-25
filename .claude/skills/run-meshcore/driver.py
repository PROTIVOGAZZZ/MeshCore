#!/usr/bin/env python3
"""Offline PlatformIO driver for MeshCore.

Builds firmware targets and runs the native googletest suite in containers
where the PlatformIO registry (api/dl.registry.platformio.org) is blocked but
GitHub (git) and the Ubuntu apt mirror are reachable.

It never edits the repo: each run copies the source tree into
$MESHCORE_OFFLINE/work/<name>/ and rewrites registry library specs there to
git URLs (see LIB_MAP), then runs `pio` in the copy.

  driver.py setup                      one-time: toolchain, platforms, libs
  driver.py build <env> [--src DIR]    build a firmware env, write UF2, print memory report
  driver.py test [--env E] [-f GLOB]   run the native googletest suites (CI: native + native_kiss_modem)
  driver.py mem <env> [--src DIR]      re-print memory report of the last build
"""
import argparse, glob, hashlib, json, os, re, shutil, signal, subprocess, sys

signal.signal(signal.SIGPIPE, signal.SIG_DFL)  # quiet when piped into head

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
CACHE = os.path.abspath(os.environ.get("MESHCORE_OFFLINE", os.path.expanduser("~/.cache/meshcore-offline")))
PIO_HOME = os.environ.get("PLATFORMIO_CORE_DIR", os.path.expanduser("~/.platformio"))

# registry spec (owner/name, version ignored) -> replacement lib_deps line(s)
LIB_MAP = {
    "rweather/Crypto": "symlink://{cache}/libs/arduinolibs/libraries/Crypto",
    "adafruit/RTClib": "https://github.com/adafruit/RTClib.git#2.1.4",
    "melopero/Melopero RV3028": "https://github.com/melopero/Melopero_RV-3028_Arduino_Library.git#1.1.0",
    "electroniccats/CayenneLPP": "https://github.com/ElectronicCats/CayenneLPP.git#1.6.1",
    "stevemarple/MicroNMEA": "https://github.com/stevemarple/MicroNMEA.git#v2.0.6",
    "adafruit/Adafruit BME280 Library": "https://github.com/adafruit/Adafruit_BME280_Library.git",
    "zinggjm/GxEPD2": "https://github.com/ZinggJM/GxEPD2.git#1.6.2",
    "bakercp/CRC32": "https://github.com/bakercp/CRC32.git",
    "densaugeo/base64": "https://github.com/Densaugeo/base64_arduino.git",
    # sensor/display libs shared by many variants (HEAD of each repo unless a tag is noted)
    "adafruit/Adafruit AHTX0": "https://github.com/adafruit/Adafruit_AHTX0.git",
    "adafruit/Adafruit BME680 Library": "https://github.com/adafruit/Adafruit_BME680.git",
    "adafruit/Adafruit BMP085 Library": "https://github.com/adafruit/Adafruit-BMP085-Library.git",
    "adafruit/Adafruit BMP280 Library": "https://github.com/adafruit/Adafruit_BMP280_Library.git",
    "adafruit/Adafruit BusIO": "https://github.com/adafruit/Adafruit_BusIO.git",
    "adafruit/Adafruit DRV2605 Library": "https://github.com/adafruit/Adafruit_DRV2605_Library.git",
    "adafruit/Adafruit EPD": "https://github.com/adafruit/Adafruit_EPD.git#4.6.1",
    "adafruit/Adafruit GFX Library": "https://github.com/adafruit/Adafruit-GFX-Library.git",
    "adafruit/Adafruit INA219": "https://github.com/adafruit/Adafruit_INA219.git",
    "adafruit/Adafruit INA260 Library": "https://github.com/adafruit/Adafruit_INA260.git",
    "adafruit/Adafruit INA3221 Library": "https://github.com/adafruit/Adafruit_INA3221.git",
    "adafruit/Adafruit LIS3DH": "https://github.com/adafruit/Adafruit_LIS3DH.git",
    "adafruit/Adafruit MLX90614 Library": "https://github.com/adafruit/Adafruit-MLX90614-Library.git",
    "adafruit/Adafruit NeoPixel": "https://github.com/adafruit/Adafruit_NeoPixel.git",
    "adafruit/Adafruit SH110X": "https://github.com/adafruit/Adafruit_SH110x.git",
    "adafruit/Adafruit SHT4x Library": "https://github.com/adafruit/Adafruit_SHT4X.git",
    "adafruit/Adafruit SHTC3 Library": "https://github.com/adafruit/Adafruit_SHTC3.git",
    "adafruit/Adafruit SSD1306": "https://github.com/adafruit/Adafruit_SSD1306.git",
    "adafruit/Adafruit ST7735 and ST7789 Library": "https://github.com/adafruit/Adafruit-ST7735-Library.git",
    "adafruit/Adafruit_VL53L0X": "https://github.com/adafruit/Adafruit_VL53L0X.git",
    "arduino-libraries/Arduino_LPS22HB": "https://github.com/arduino-libraries/Arduino_LPS22HB.git",
    "bodmer/TFT_eSPI": "https://github.com/Bodmer/TFT_eSPI.git",
    "boschsensortec/BSEC Software Library": "https://github.com/boschsensortec/BSEC-Arduino-library.git",
    "end2endzone/NonBlockingRTTTL": "https://github.com/end2endzone/NonBlockingRtttl.git",
    "finitespace/BME280": "https://github.com/finitespace/BME280.git",
    "lewisxhe/XPowersLib": "https://github.com/lewisxhe/XPowersLib.git",
    "lovyan03/LovyanGFX": "https://github.com/lovyan03/LovyanGFX.git",
    "olikraus/U8g2": "https://github.com/olikraus/U8g2_Arduino.git",
    "robtillaart/INA226": "https://github.com/RobTillaart/INA226.git",
    "sensirion/Sensirion I2C SHT4x": "https://github.com/Sensirion/arduino-core.git\n  https://github.com/Sensirion/arduino-i2c-sht4x.git",
    "sparkfun/SparkFun u-blox GNSS Arduino Library": "https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library.git",
    "ESP32Async/ESPAsyncWebServer": "https://github.com/ESP32Async/ESPAsyncWebServer.git#v3.10.3",
}
# transitive deps some libs declare; installed up front so PIO never asks the registry
PRELOAD = [
    "https://github.com/bblanchon/ArduinoJson.git#v7.4.2",
    "https://github.com/adafruit/Adafruit_BusIO.git",
    "https://github.com/adafruit/Adafruit_Sensor.git",
    "https://github.com/adafruit/Adafruit-GFX-Library.git",
]


def sh(cmd, **kw):
    print("+", cmd if isinstance(cmd, str) else " ".join(cmd), flush=True)
    return subprocess.run(cmd, shell=isinstance(cmd, str), check=True, **kw)


def write_pkg(path, name, version, kind="tool"):
    os.makedirs(path, exist_ok=True)
    with open(os.path.join(path, "package.json"), "w") as f:
        json.dump({"name": name, "version": version, "system": "*"}, f)


# ---------------------------------------------------------------- setup
def setup(_args):
    if not shutil.which("arm-none-eabi-gcc"):
        sh("apt-get install -y -q gcc-arm-none-eabi libnewlib-arm-none-eabi || "
           "(apt-get update -q && apt-get install -y -q gcc-arm-none-eabi libnewlib-arm-none-eabi)")
    if not shutil.which("pio"):
        sh([sys.executable, "-m", "pip", "install", "-q", "platformio"])
    os.makedirs(CACHE, exist_ok=True)
    pk = os.path.join(CACHE, "pkgs")

    # toolchain = Ubuntu's arm-none-eabi-gcc, wrapped as a PIO package
    tc = os.path.join(pk, "tc", "bin")
    os.makedirs(tc, exist_ok=True)
    for f in glob.glob("/usr/bin/arm-none-eabi-*"):
        dst = os.path.join(tc, os.path.basename(f))
        if not os.path.lexists(dst):
            os.symlink(f, dst)
    write_pkg(os.path.join(pk, "tc"), "toolchain-gccarmnoneeabi", "1.70201.0")
    # packages the nordicnrf52 platform insists on but a build never runs
    write_pkg(os.path.join(pk, "tool-sreccat"), "tool-sreccat", "1.164.0")
    write_pkg(os.path.join(pk, "tool-adafruit-nrfutil"), "tool-adafruit-nrfutil", "1.503.0")

    # CMSIS 5.7.0 headers (the DSP .a in git is an LFS pointer -> replace with an empty archive)
    cm = os.path.join(pk, "framework-cmsis")
    write_pkg(cm, "framework-cmsis", "2.50700.0")
    if not os.path.isdir(os.path.join(cm, "CMSIS", "Core", "Include")):
        src = os.path.join(CACHE, "cmsis5")
        if not os.path.isdir(src):
            sh(["git", "clone", "-q", "--depth", "1", "--branch", "5.7.0", "--filter=blob:none",
                "--sparse", "https://github.com/ARM-software/CMSIS_5", src])
            sh(["git", "-C", src, "sparse-checkout", "set", "CMSIS/Core/Include", "CMSIS/DSP/Include"])
        shutil.copytree(os.path.join(src, "CMSIS"), os.path.join(cm, "CMSIS"), dirs_exist_ok=True)
    lib = os.path.join(cm, "CMSIS", "DSP", "Lib", "GCC")
    os.makedirs(lib, exist_ok=True)
    stub = os.path.join(lib, "libarm_cortexM4lf_math.a")
    if not os.path.exists(stub):
        c, o = os.path.join(CACHE, "d.c"), os.path.join(CACHE, "d.o")
        open(c, "w").write("void __cmsis_dummy(void){}\n")
        sh(["arm-none-eabi-gcc", "-c", c, "-o", o])
        sh(["arm-none-eabi-ar", "rcs", stub, o])

    # SCons: PIO core needs tool-scons from the registry; install from PyPI instead
    sc = os.path.join(PIO_HOME, "packages", "tool-scons")
    if not os.path.exists(os.path.join(sc, "scons.py")):
        os.makedirs(sc, exist_ok=True)
        sh([sys.executable, "-m", "pip", "install", "-q", "--target", os.path.join(sc, "lib"), "scons==4.8.1"])
        open(os.path.join(sc, "scons.py"), "w").write(
            "import sys,os\nsys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),'lib'))\n"
            "from SCons.Script.Main import main\nmain()\n")
        write_pkg(sc, "tool-scons", "4.41101.0")
        json.dump({"type": "tool", "name": "tool-scons", "version": "4.41101.0",
                   "spec": {"owner": "platformio", "id": None, "name": "tool-scons",
                            "requirements": "~4.41101.0", "uri": None}},
                  open(os.path.join(sc, ".piopm"), "w"))

    # platforms from GitHub (the nrf52 arduino builder is a git submodule)
    for name, url, tag in [("platform-nordicnrf52", "https://github.com/platformio/platform-nordicnrf52", "v11.0.0"),
                           ("platform-native", "https://github.com/platformio/platform-native", "v1.2.1")]:
        d = os.path.join(CACHE, name)
        if not os.path.isdir(d):
            sh(["git", "clone", "-q", "--depth", "1", "--branch", tag, "--recurse-submodules",
                "--shallow-submodules", url, d])

    libs = os.path.join(CACHE, "libs")
    os.makedirs(libs, exist_ok=True)
    if not os.path.isdir(os.path.join(libs, "arduinolibs")):  # Crypto lives in a subdir -> symlink://
        sh(["git", "clone", "-q", "--depth", "1", "https://github.com/rweather/arduinolibs.git",
            os.path.join(libs, "arduinolibs")])
    gt = os.path.join(libs, "googletest")
    if not os.path.isdir(gt):  # registry package = repo + a library.json that PIO's googletest runner needs
        sh(["git", "clone", "-q", "--depth", "1", "--branch", "v1.17.0", "https://github.com/google/googletest.git", gt])
        json.dump({"name": "googletest", "version": "1.17.0",
                   "build": {"flags": ["-Igoogletest/include", "-Igoogletest"],
                             "srcFilter": ["-<*>", "+<googletest/src/gtest-all.cc>"],
                             "includeDir": "googletest/include"}},
                  open(os.path.join(gt, "library.json"), "w"), indent=1)
    print("setup OK:", CACHE)


# ---------------------------------------------------------------- source copy + ini rewrite
def work_dir(src, name):
    tag = "" if os.path.abspath(src) == REPO else "-" + hashlib.sha1(os.path.abspath(src).encode()).hexdigest()[:8]
    return os.path.join(CACHE, "work", name + tag)


def prepare(src, name):
    work = work_dir(src, name)
    os.makedirs(work, exist_ok=True)
    shutil.copytree(src, work, dirs_exist_ok=True, symlinks=True,
                    ignore=shutil.ignore_patterns(".git", ".pio", ".claude"))
    inis = [os.path.join(work, "platformio.ini")] + glob.glob(os.path.join(work, "variants", "*", "platformio.ini"))
    unmapped = set()
    for f in inis:
        t = open(f).read()
        out = []
        for line in t.split("\n"):
            m = re.match(r"^(\s+)([\w.-]+/[^@;\n]+?)\s*@\s*[^;\n]+$", line)
            if m and "://" not in line:
                key = m.group(2).strip()
                if key in LIB_MAP:
                    line = m.group(1) + LIB_MAP[key].format(cache=CACHE).replace("\n  ", "\n" + m.group(1))
                elif key == "google/googletest":
                    pass  # pre-seeded by `test`
                else:
                    unmapped.add(key)
            out.append(line)
        t = "\n".join(out)
        if f.endswith(os.path.join(work, "platformio.ini")):
            pk = os.path.join(CACHE, "pkgs")
            t = re.sub(r"(\[nrf52_base\]\nextends = arduino_base\n)platform = nordicnrf52\nplatform_packages =",
                       lambda m: m.group(1) + f"platform = file://{CACHE}/platform-nordicnrf52\nplatform_packages =\n"
                       f"  toolchain-gccarmnoneeabi @ symlink://{pk}/tc\n"
                       f"  tool-sreccat @ symlink://{pk}/tool-sreccat\n"
                       f"  tool-adafruit-nrfutil @ symlink://{pk}/tool-adafruit-nrfutil\n"
                       f"  framework-cmsis @ symlink://{pk}/framework-cmsis", t)
            t = t.replace("platform = native\n", f"platform = file://{CACHE}/platform-native\n")
            t = t.replace("lib_deps =\n  SPI\n  Wire\n",
                          "lib_deps =\n  SPI\n  Wire\n" + "".join(f"  {u}\n" for u in PRELOAD), 1)
        open(f, "w").write(t)
    return work, unmapped


def libdeps_fixup(work, env, log, direct=()):
    """Strip dependency declarations from installed libs; stub libs PIO still tries to fetch."""
    d = os.path.join(work, ".pio", "libdeps", env)
    for f in glob.glob(d + "/*/library.json"):
        j = json.load(open(f))
        if j.pop("dependencies", None) is not None:
            json.dump(j, open(f, "w"), indent=1)
    for f in glob.glob(d + "/*/library.properties"):
        lines = [l for l in open(f) if not l.startswith("depends=")]
        open(f, "w").writelines(lines)
    # registry installs ("Installing owner/Name @ req") that never report "Name@x has been installed"
    m = [(o, n, r) for o, n, r in re.findall(r"Library Manager: Installing (?:([\w.-]+)/)?([\w .-]+?) @ (\S+)", log)
         if not re.search(re.escape(n) + r"@\S+ has been installed", log) and f"{o}/{n}" not in direct]
    for owner, name, req in m:
        s = os.path.join(d, name)
        if not os.path.isdir(s):
            print(f"stubbing unreachable transitive lib {owner}/{name}")
            os.makedirs(s)
            json.dump({"name": name, "version": req.lstrip("^~"), "platforms": "native"},
                      open(os.path.join(s, "library.json"), "w"))
            json.dump({"type": "library", "name": name, "version": req.lstrip("^~"),
                       "spec": {"owner": owner, "id": None, "name": name, "requirements": req, "uri": None}},
                      open(os.path.join(s, ".piopm"), "w"))
    return bool(m)


def run_pio(work, env, pio_args, direct=()):
    # 1) install lib_deps without their declared dependencies (those would go to the registry);
    #    a few git libs still pull a registry package (base64 -> throwtheswitch/Unity): stub it, retry
    for attempt in range(4):
        p = subprocess.run(["pio", "pkg", "install", "-e", env, "--skip-dependencies"], cwd=work,
                           capture_output=True, text=True)
        log = p.stdout + p.stderr
        if p.returncode == 0:
            break
        if not libdeps_fixup(work, env, log, direct):
            return p.returncode, log
    libdeps_fixup(work, env, "", direct)
    # 2) build/test; strip + stub again if PIO still tries the registry
    for attempt in range(4):
        p = subprocess.run(["pio"] + pio_args, cwd=work, capture_output=True, text=True)
        log = p.stdout + p.stderr
        if "HTTPClientError" not in log or not libdeps_fixup(work, env, log, direct):
            break
    return p.returncode, log


# ---------------------------------------------------------------- memory report
def mem(env, work=None):
    work = work or work_dir(REPO, env)
    elf = os.path.join(work, ".pio", "build", env, "firmware.elf")
    if not os.path.exists(elf):
        sys.exit(f"no {elf}; build first")
    nm = subprocess.run(["arm-none-eabi-nm", "-S", "--size-sort", "-C", elf], capture_output=True, text=True).stdout
    syms = subprocess.run(["arm-none-eabi-nm", "-C", elf], capture_output=True, text=True).stdout
    addr = {l.split()[-1]: int(l.split()[0], 16) for l in syms.splitlines()
            if l.split()[-1] in ("__HeapBase", "__HeapLimit", "__data_start__", "__bss_end__")}
    if "__HeapBase" in addr and "__HeapLimit" in addr:
        print(f"heap (before runtime allocations): {addr['__HeapLimit'] - addr['__HeapBase']} bytes "
              f"[{addr['__HeapBase']:#x}..{addr['__HeapLimit']:#x}]")
    print("largest RAM symbols:")
    ram = [l for l in nm.splitlines() if len(l.split()) >= 4 and l.split()[2] in "bBdD"]
    for l in ram[-12:]:
        a, s, t, n = l.split(None, 3)
        print(f"  {int(s, 16):7d}  {n}")


# ---------------------------------------------------------------- commands
def build(a):
    work, unmapped = prepare(os.path.abspath(a.src), a.env)
    rc, log = run_pio(work, a.env, ["run", "-e", a.env], direct=unmapped)
    logf = os.path.join(work, "build.log")
    open(logf, "w").write(log)
    for l in log.splitlines():
        if re.search(r"error:|RAM:|Flash:|HTTPClientError|VCSBase|Error \d", l):
            print(l)
    elf = os.path.join(work, ".pio", "build", a.env, "firmware.elf")
    if not os.path.exists(elf) or "RAM:" not in log:
        pm = re.search(r"Platform Manager: Installing (\S+)", log)
        if pm and "file://" not in pm.group(1):
            print(f"platform {pm.group(1)} is registry-only: this driver builds nrf52 (+native) envs only")
        if unmapped:
            print("registry libs without a LIB_MAP entry (add git URLs in driver.py):", sorted(unmapped))
        sys.exit(f"BUILD FAILED, full log: {logf}")
    hexf = os.path.join(work, ".pio", "build", a.env, "firmware.hex")
    outdir = os.path.join(CACHE, "out")
    os.makedirs(outdir, exist_ok=True)
    if os.path.exists(hexf) and os.path.exists(os.path.join(work, "bin", "uf2conv", "uf2conv.py")):
        uf2 = os.path.join(outdir, os.path.basename(work) + ".uf2")
        subprocess.run([sys.executable, os.path.join(work, "bin", "uf2conv", "uf2conv.py"), hexf,
                        "-c", "-f", "0xADA52840", "-o", uf2], check=True, capture_output=True)
        print("UF2:", uf2)
    mem(a.env, work)
    print("full log:", logf)


def test(a):
    work, _ = prepare(os.path.abspath(a.src), "native")
    # PIO's googletest runner demands google/googletest@^1.17.0 from the registry: pre-seed it per env
    for env in a.env:
        seed = os.path.join(work, ".pio", "libdeps", env, "googletest")
        if not os.path.exists(os.path.join(seed, ".piopm")):
            shutil.rmtree(seed, ignore_errors=True)
            shutil.copytree(os.path.join(CACHE, "libs", "googletest"), seed, ignore=shutil.ignore_patterns(".git"))
            json.dump({"type": "library", "name": "googletest", "version": "1.17.0",
                       "spec": {"owner": "google", "id": None, "name": "googletest", "requirements": "1.17.0",
                                "uri": None}},
                      open(os.path.join(seed, ".piopm"), "w"))
    args = ["test"] + [x for env in a.env for x in ("-e", env)] + (["-f", a.filter] if a.filter else [])
    p = subprocess.run(["pio"] + args, cwd=work, capture_output=True, text=True)
    log = p.stdout + p.stderr
    logf = os.path.join(work, "test.log")
    open(logf, "w").write(log)
    for l in log.splitlines():
        if re.search(r"\[(PASSED|FAILED|ERRORED|SKIPPED)\]|test cases|error:|HTTPClientError", l):
            print(l)
    print("full log:", logf)
    sys.exit(p.returncode)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("setup").set_defaults(fn=setup)
    b = sp.add_parser("build"); b.add_argument("env"); b.add_argument("--src", default=REPO); b.set_defaults(fn=build)
    t = sp.add_parser("test"); t.add_argument("--src", default=REPO)
    t.add_argument("--env", action="append", help="default: native + native_kiss_modem (as CI)")
    t.add_argument("-f", "--filter", help="test dir glob, e.g. test_utils")
    t.set_defaults(fn=lambda a: test(a) if a.env else test(argparse.Namespace(**{**vars(a), "env": ["native", "native_kiss_modem"]})))
    m = sp.add_parser("mem"); m.add_argument("env"); m.add_argument("--src", default=REPO)
    m.set_defaults(fn=lambda a: mem(a.env, work_dir(os.path.abspath(a.src), a.env)))
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()

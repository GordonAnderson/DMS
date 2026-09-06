import os
import re
import shutil

Import("env")

# Matches the `Version` string in src/DMS.cpp, e.g.
#   const char *Version = "DMS, version 1.5 Sep 5, 2026";
# -> "1.5". Reading it out of the source rather than duplicating a version
# number here means firmware/'s filename always matches what the GVER
# command reports on the device.
VERSION_RE = re.compile(r'Version\s*=\s*"[^"]*version\s+([0-9]+\.[0-9]+)')

def get_firmware_version(project_dir):
    dms_cpp = os.path.join(project_dir, "src", "DMS.cpp")
    try:
        with open(dms_cpp, "r") as f:
            contents = f.read()
    except OSError:
        return None
    match = VERSION_RE.search(contents)
    return match.group(1) if match else None

def convert_to_uf2(source, target, env):
    project_dir = env["PROJECT_DIR"]

    # Get the concrete path strings
    bin_file = str(target[0])
    uf2_file = bin_file.replace(".bin", ".uf2")

    print("\n==================================================")
    print(f"UF2 SCRIPT TRIGGERED!")
    print(f"Source Binary: {bin_file}")
    print(f"Target UF2:    {uf2_file}")
    print("==================================================\n")

    # Run the tool explicitly with python3
    cmd = f"python3 uf2conv.py -b 0x0000 -c -o {uf2_file} {bin_file}"
    env.Execute(cmd)

    # Also drop versioned copies into firmware/ at the project root, named
    # from the Version string embedded in src/DMS.cpp (e.g. DMS-v1.5.uf2,
    # DMS-v1.5.bin). Rebuilding at the same version (e.g. mid-way through a
    # batch of fixes not yet released) overwrites those same files until
    # the version bumps.
    version = get_firmware_version(project_dir)
    if not version:
        print("UF2 SCRIPT: could not find a Version string in src/DMS.cpp - skipping firmware/ copy\n")
        return

    firmware_dir = os.path.join(project_dir, "firmware")
    os.makedirs(firmware_dir, exist_ok=True)
    for src_file, ext in ((uf2_file, "uf2"), (bin_file, "bin")):
        versioned_path = os.path.join(firmware_dir, f"DMS-v{version}.{ext}")
        shutil.copyfile(src_file, versioned_path)
        print(f"UF2 SCRIPT: copied to {versioned_path}")
    print("")

# Print immediately when PlatformIO parses the ini file to confirm it's loaded
print("\n---> UF2 Extra Script Loaded successfully into PlatformIO Build Pipeline <---\n")

# Use a broader build target hook to guarantee it intercepts the binary creation
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", convert_to_uf2)

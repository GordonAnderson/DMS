import os
Import("env")

def convert_to_uf2(source, target, env):
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

# Print immediately when PlatformIO parses the ini file to confirm it's loaded
print("\n---> UF2 Extra Script Loaded successfully into PlatformIO Build Pipeline <---\n")

# Use a broader build target hook to guarantee it intercepts the binary creation
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", convert_to_uf2)
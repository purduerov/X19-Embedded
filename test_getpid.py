import subprocess
print("Finding arm-none-eabi-gcc:")
subprocess.run(["which", "arm-none-eabi-gcc"])
print("Checking path...")
subprocess.run(["echo", "$PATH"], shell=True)

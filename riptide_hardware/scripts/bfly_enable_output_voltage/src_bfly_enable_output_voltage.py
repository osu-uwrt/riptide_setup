# Import the FlyCapture2 SDK Python wrapper
import PyCapture2 as pc

# This is the address for the OUTPUT_VOLTAGE_ENABLE register on the FLIR BlackFly USB3
OUTPUT_VOLTAGE_ENABLE = int(0x19D0)
# Index of the camera to set (default is 0)
CAMERA_INDEX = 0

ENABLED = 0x80000001
DISABLED = 0x80000000

bus = pc.BusManager()

numCams = bus.getNumOfCameras()
if numCams == 0:
    print("[BFLY EOV] No cameras found. Exiting...")
    exit()

print("[BFLY EOV] Found %d camera(s)." % numCams)

# Connect to a specific camera to read/write values to
uid = bus.getCameraFromIndex(CAMERA_INDEX)
cam = pc.Camera()
cam.connect(uid)

# Read current status. If disabled, attempt to write the correct value.
read = cam.readRegister(OUTPUT_VOLTAGE_ENABLE)
if read == DISABLED:
    cam.writeRegister(OUTPUT_VOLTAGE_ENABLE, ENABLED)

# Check to make sure it was actually enabled and notify the user
read = cam.readRegister(OUTPUT_VOLTAGE_ENABLE)
if read == ENABLED: 
    print("[BFLY EOV] 3.3V output enabled on camera %d" % CAMERA_INDEX)
else:
    print("[BFLY EOV] Failed to enable output voltage on camera %d" % CAMERA_INDEX)

print("[BFLY EOV] Readback: %s" % hex(read))
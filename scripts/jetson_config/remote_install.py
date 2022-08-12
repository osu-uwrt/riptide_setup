import os
from posixpath import expanduser
from unittest import result
from fabric import Connection
from subprocess import Popen, PIPE, CalledProcessError, call
from glob import glob
import argparse
import time

# version of call that pipes stdout back
def execute(fullCmd, printOut=False):
    if printOut: print(fullCmd)
    proc = Popen(fullCmd, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if printOut:
        for line in iter(proc.stdout.readline, ""):
            print(line)
        for errLine in iter(proc.stderr.readline, ""):
            print(f"ERROR: {errLine}")
    proc.stdout.close()
    retCode = proc.wait()
    if retCode:
        raise CalledProcessError(retCode, fullCmd)

def remoteExec(cmd, username, address, printOut = False):
    result = Connection(f"{username}@{address}").run(cmd, hide=(not printOut))
    return result.exited

def testNetwork(pingAddress):
    if call(["ping", "-c", "1", pingAddress], stdout=open(os.devnull, 'wb')) != 0:
        print(f"Failed to ping {pingAddress}")
        return False
    return True

def testNetworkRemote(pingAddress, username, address):
    if remoteExec(f"ping -c 1 {pingAddress}", username, address) != 0:
        print(f"Failed to ping {pingAddress} on {username}@{address}")
        return False
    return True

def execLocalScript(localPath, args):
    scriptPath = os.path.join(os.getcwd(), localPath)
    if not os.path.isfile(scriptPath):
        print(f"Script {scriptPath}, does not exist")
        exit()
    callList = [scriptPath]
    callList.extend(args)
    return execute(callList, True)

def makeRemoteDir(remoteDir, username, address):
    remoteExec(f"mkdir -p {remoteDir}", username, address, False)

def xferSingleFile(localFile, username, address, destination):
    execute(["rsync", "-vzc", localFile, f"{username}@{address}:{destination}"], False)

def xferDir(localdir, username, address, destination):
    execute(["rsync", "-vrzc", "--delete", "--exclude=**/.git/", "--exclude=**/.vscode/", localdir, f"{username}@{address}:{destination}"], False)

def runRemoteCmd(remoteScript, username, address):
    remoteExec(remoteScript, username, address, True)

def querySelection(inputList):
    selection = -1
    if(len(inputList) > 1):
        print("Found multiple options:")
        for i in range(len(inputList)):
            print(f"{i + 1} {inputList[i]}")
        while selection < 1 or selection > len(inputList):
            selection = input("Select the option you want to use: ")
        selection -= 1        
    else:
        selection = 0
    return inputList[selection]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Setup remote Jetson target")
    parser.add_argument("address", type=str, help="Address of the jetson, Hostname or IP")
    parser.add_argument("-u", "--username", type=str, help="Username to login with", default="ros", required=False)
    args = parser.parse_args()

    HOME_DIR = expanduser("~")
    BASE_DIR = os.path.join(HOME_DIR, "osu-uwrt")
    ROS_DISTRO = "humble"
    SCRIPT_DIR = os.path.join(BASE_DIR, "riptide_setup", "scripts", "jetson_config")

    # test the connection first
    print(f"Testing connection to {args.address}")
    if not testNetwork(args.address):
        exit()
    print("    OK")

    # handle SSH keys
    print("\nChecking for SSH keys")
    sshkeys = glob(os.path.join(HOME_DIR, ".ssh", f"sshkey_{args.address}"))
    if(len(sshkeys) > 0):
        print(f"\n\nSSH key already exists for address: {args.address}")
    else:
        print(f"\n\nConfiguring SSH Keys for {args.username}@{args.address}")
        try:
            execLocalScript(os.path.join("config_host", "relationship.bash"), [args.address, args.username])
        except CalledProcessError as e:
            if e.returncode != 0 and e.returncode != 1:
                exit()
    print("    DONE")
        
    # make the remote directory
    print(f"\nCreating {BASE_DIR} on target")
    makeRemoteDir(BASE_DIR, args.username, args.address)
    print("    DONE")

    ROS_TAR = querySelection(glob(os.path.join(BASE_DIR, f"{ROS_DISTRO}*.tar.gz")))
    print(f"\nTransferring binary {ROS_TAR} to target, this may take a minute:")
    xferSingleFile(ROS_TAR, args.username, args.address, BASE_DIR)
    print("    DONE")

    print("\nTransferring scripts to target")
    riptideSetupDir = os.path.join(BASE_DIR, "riptide_setup")
    xferDir(riptideSetupDir, args.username, args.address, BASE_DIR)
    print("    DONE")
        
    print("\nTesting remote internet connection")
    if not testNetworkRemote("google.com", args.username, args.address):
        exit()
    print("    OK ")

    print("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!! Preparing to invoke setup scripts on the target !!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    time.sleep(1.0)

    print("\nInstalling dependencies:")
    scriptRun = os.path.join(SCRIPT_DIR, "unpack_install", "install_deps.bash")
    runRemoteCmd(scriptRun, args.username, args.address)
    print("    DONE")

    print("\nInstalling ROS on the target:")
    scriptRun = os.path.join(SCRIPT_DIR, "unpack_install", f"install_tar.bash {ROS_DISTRO} {ROS_TAR}")
    runRemoteCmd(scriptRun, args.username, args.address)
    print("    DONE")

    print("\nInstalling PyTorch on the target:")
    scriptRun = os.path.join(SCRIPT_DIR, "unpack_install", "pytorch_install.bash")
    runRemoteCmd(scriptRun, args.username, args.address)
    print("    DONE")
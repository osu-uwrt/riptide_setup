import os
from posixpath import expanduser
from getpass import getpass
from fabric import Connection, Config
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

def remoteExecResult(cmd, username, address, printOut=False, root=False, passwd=""):
    if root:
        connect = Connection(f"{username}@{address}", config=Config(overrides={"sudo": {"password": passwd}}))
        result = connect.sudo(cmd, hide=(not printOut))
        return (result.exited, str(result).rsplit("\n") if result else [])
    else:
        connect = Connection(f"{username}@{address}")
        result = connect.run(cmd, hide=(not printOut))
        return (result.exited, str(result).rsplit("\n") if result else [])

def remoteExec(cmd, username, address, printOut=False, root=False, passwd=""):
    if root:
        connect = Connection(f"{username}@{address}", config=Config(overrides={"sudo": {"password": passwd}}))
        result = connect.sudo(cmd, hide=(not printOut))
        return result.exited
    else:
        connect = Connection(f"{username}@{address}")
        result = connect.run(cmd, hide=(not printOut))
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
    return execute(callList, printOut=True)

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
    REM_BSE_DIR = os.path.join("/home", args.username, "osu-uwrt")
    ROS_DISTRO = "humble"
    # SCRIPT_DIR = os.path.join(BASE_DIR, "riptide_setup", "scripts", "jetson_config")
    REM_SCRIPT_DIR = os.path.join(REM_BSE_DIR, "riptide_setup", "scripts", "jetson_config")

    # test the connection first
    print(f"Testing connection to {args.address}")
    if not testNetwork(args.address):
        exit()
    print("    OK")

    # handle SSH keys
    print("\nChecking for SSH keys")
    targetPass = getpass(prompt="Enter password for remote user:\n")
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
    print(f"\nCreating {REM_BSE_DIR} on target")
    makeRemoteDir(REM_BSE_DIR, args.username, args.address)
    print("    DONE")

    TARS = glob(os.path.join(BASE_DIR, f"{ROS_DISTRO}*.tar.gz"))
    if len(TARS) > 0:
        ROS_TAR = querySelection(TARS)
        print(f"\nTransferring binary {ROS_TAR} to target, this may take a minute:")
        xferSingleFile(ROS_TAR, args.username, args.address, REM_BSE_DIR)
        print("    DONE")
    else:
        print("Skipping tarball transfer")

    print("\nTransferring scripts to target:")
    riptideSetupDir = os.path.join(BASE_DIR, "riptide_setup")
    xferDir(riptideSetupDir, args.username, args.address, REM_BSE_DIR)
    print("    DONE")

    print("\nAllowing passwordless sudo on target:")
    remoteExec(f'echo "{args.username} ALL=(ALL:ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/{args.username}',
        args.username, args.address, root=True, passwd=targetPass)
    print("    OK ")

    print("\nSynchronizing clocks between host and target:")
    scriptRun = os.path.join("config_target", "date_set.bash")
    execLocalScript(scriptRun, [args.address, args.username])
    print("    DONE")

    print("\nConfiguring JetPack settings on target:")
    scriptRun = os.path.join(REM_SCRIPT_DIR, "config_target", "configure_jetpack.bash")
    remoteExec(f"/bin/bash {scriptRun}", args.username, args.address, root=True, passwd=targetPass)
    print("    DONE")
        
    print("\nTesting remote internet connection")
    if not testNetworkRemote("google.com", args.username, args.address):
        exit()
    print("    OK ")

    _, remoteArch = remoteExecResult("uname -m", args.username, args.address, passwd=targetPass, printOut=True)
    if len(remoteArch) < 0 or not "aarch64" in remoteArch:
        cont = input("The connected system is not aarch64! Continue? y/n")
        if cont.lower() != "y" or cont.lower() != "yes":
            print("Aborting configuration")
            exit()

    print("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!! Preparing to invoke setup scripts on the target !!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    time.sleep(1.0)

    print("\nInstalling dependencies:")
    scriptRun = os.path.join(REM_SCRIPT_DIR, "unpack_install", "install_deps.bash")
    runRemoteCmd(scriptRun, args.username, args.address)
    print("    DONE")

    print("\nInstalling ROS on the target:")
    REM_TAR = os.path.join(REM_BSE_DIR, ROS_TAR[ROS_TAR.rindex('/') + 1: ])
    print(f"Using remote tar file {REM_TAR}")
    scriptRun = os.path.join(REM_SCRIPT_DIR, "unpack_install", f"install_tar.bash {ROS_DISTRO} {REM_TAR}")
    runRemoteCmd(scriptRun, args.username, args.address)

    print("\nSetting up .bashrc")
    scriptRun = os.path.join(REM_SCRIPT_DIR, "unpack_install", "setup_bashrc.bash")
    runRemoteCmd(scriptRun, args.username, args.address)

    print("    DONE")

    print("\nInstalling PyTorch on the target:")
    scriptRun = os.path.join(REM_SCRIPT_DIR, "unpack_install", "pytorch_install.bash")
    runRemoteCmd(scriptRun, args.username, args.address)
    print("    DONE")
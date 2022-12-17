import os
import argparse
from pathlib import Path
from subprocess import Popen, PIPE

# version of call that pipes stdout back
def execute(fullCmd, printOut=False):
    if printOut: print(fullCmd)
    stdoutText = []
    proc = Popen(fullCmd, stdout=PIPE, stderr=PIPE, universal_newlines=True)

    for line in iter(proc.stdout.readline, ""):
        stdoutText.append(line)
        if printOut: print(line)
    for errLine in iter(proc.stderr.readline, ""):
        if printOut: print(f"ERROR: {errLine}")

    proc.stdout.close()
    retCode = proc.wait()
    return(retCode, stdoutText)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Setup colcon ignore files for packages the system cannot run")
    parser.add_argument("package_pattern", type=str, help="The string pattern to test for")
    args = parser.parse_args()

    # the directory to test in
    curr_dir = os.getcwd()

    # get the output of colcon
    colcon_out = execute(["colcon", "list"])

    # make sure colcon ran okay
    if(colcon_out[0] != 0):
        print(f"Error running colcon: {colcon_out[0]}")
        exit(-10)

    # find the packages that match
    matches = []
    for package in colcon_out[1]:
        if args.package_pattern in package:
            tabs = (package.index("\t"), package.rindex("\t"))

            # make sure the tab locations dont match
            if tabs[0] == tabs[1]:
                print("Could not parse colcon list output")
            else:
                matches.append(package[tabs[0]+1 : tabs[1]])

    if len(matches) < 0:
        print("No matches found")
        exit(-9)

    # create the COLCON_IGNORE files 
    for path in matches:
        file = Path(curr_dir, path, "COLCON_IGNORE")
        print(f"Creating colcon ignore file {file}")
        file.touch()

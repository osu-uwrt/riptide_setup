# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Copyright 2022 Cole Tucker
# Licensed under the Apache License, Version 2.0
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint
from colcon_core.package_selection import get_packages
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.argument_parser.destination_collector import \
    DestinationCollectorDecorator
from colcon_core.task import add_task_arguments

from subprocess import Popen, PIPE, call
from fabric import Connection
import os, stat

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
    return retCode

def xferDir(localdir, username, address, destination):
    execute(["rsync", "-vrzc", "--delete", "--exclude=**/.git/", "--exclude=**/.vscode/", localdir, f"{username}@{address}:{destination}"], False)

def testNetwork(pingAddress):
    return call(["ping", "-c", "1", pingAddress], stdout=open(os.devnull, 'wb'))

def remoteExec(cmd, username, address, printOut=False, passwd=""):
    connect = Connection(f"{username}@{address}")
    result = 0
    try:
        result = connect.run(cmd, hide=(not printOut)).exited
    except Exception as e:
        pass
    return result

def makeRemoteDir(remoteDir, username, address):
    remoteExec(f"mkdir -p {remoteDir}", username, address, False)

def delRemoteDir(remoteDir, username, address):
    remoteExec(f"rm -rf {remoteDir}", username, address, False)

def createAndSendBuildScript(username, hostname, remote_dir, source_files, packages):
    DEPLOY_TEMPLATE = """
    #!/bin/bash

    # Make sure build dir exists
    if [ ! -d {0} ]; then 
        echo "Build called on a non-existant directory"
        exit -2
    fi

    # files to source
    {1}

    # switch directory
    cd {0}

    # run the build
    colcon build {2}
    """

    sources = ""
    for file in source_files:
        sources += f"source {file}\n"

    # make sure there are entries in the list otherwise do it all
    packages_to_build = ""
    if len(packages) > 0:
        packages_to_build = "--packages-select "
        for package in packages:
            packages_to_build += f"{package} "

    # write the template
    local_script_path = "/tmp/deploy_build.bash"
    with open(local_script_path, "w") as file1:
        # Writing data to a file
        formatted_template = DEPLOY_TEMPLATE.format(remote_dir, sources, packages_to_build)
        file1.write(formatted_template)

    # make the template executable
    st = os.stat(local_script_path)
    os.chmod(local_script_path, st.st_mode | stat.S_IEXEC)

    # transfer the script
    xferDir(local_script_path, username, hostname, local_script_path)

class DeployVerb(VerbExtensionPoint):
    """Cleans package workspaces."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102

        # enforce hostname
        parser.add_argument('hostname')

        # support a configurable username
        parser.add_argument(
            '--username',
            default='ros',
            help='The username to use on login '
                 '(default: ros)'
        )

        # support a configurable remote dir
        parser.add_argument(
            '--remote_dir',
            default='~/colcon_deploy',
            help='The remote directory to transfer files to '
                 '(default: ~/colcon_deploy)'
        )

        # support a clean build on the other end
        parser.add_argument(
            '--clean',
            action='store_true',
            help='Should the remote build be cleaned first?'
        )

        add_packages_arguments(parser)

        decorated_parser = DestinationCollectorDecorator(parser)
        add_task_arguments(decorated_parser, 'colcon_core.task.deploy')
        self.task_argument_destinations = decorated_parser.get_destinations()


        

    def main(self, *, context):  # noqa: D102

        USERNAME = context.args.username
        REMOTE_DIR = context.args.remote_dir
        HOSTNAME = context.args.hostname
        WANT_CLEAN = context.args.clean

        REM_SRC_DIR = os.path.join(REMOTE_DIR, "src")

        # the remote directories to clean when cleaning
        REM_DIRS_FOR_CLEAN = [
            REM_SRC_DIR,
            os.path.join(REMOTE_DIR, "install"),
            os.path.join(REMOTE_DIR, "build"),
            os.path.join(REMOTE_DIR, "log")
        ]
        

        # test connection to target
        ret_code = testNetwork(HOSTNAME)
        if ret_code != 0:
            print(f"Failed to ping {HOSTNAME}. Hostname unknown, or device offline")
            exit(-1)

        # attempt an rsync for the current directory to the target dir
        print("Synchronizing local packages to target")
        decorators = get_packages(
            context.args,
            additional_argument_names=self.task_argument_destinations,
            recursive_categories=('run', )
        )

        # grab out the descriptors from each package as we are only building 
        # on the target and not the host
        packages_for_xfer = [package.descriptor for package in decorators]

        # make sure the remote directory exists
        makeRemoteDir(REMOTE_DIR, USERNAME, HOSTNAME)

        # make sure we're not cleaning the entire directory
        if WANT_CLEAN:
            print("Cleaning remote directories")
            for dir in REM_DIRS_FOR_CLEAN:
                delRemoteDir(dir, USERNAME, HOSTNAME)

        # make sure the remote source directory exists
        makeRemoteDir(REM_SRC_DIR, USERNAME, HOSTNAME)

        # show packages for xfer to the user
        print("Selected packages:")
        packages_to_build = []
        for descriptor in packages_for_xfer:
            print(f"\t{descriptor.name}")
            packages_to_build.append(descriptor.name)
        print("\n\n")

        # get exlpicitly the package paths
        xfered = 0
        try:
            for descriptor in packages_for_xfer:
                print(f"Synchronizing >>> {descriptor.name}")
                xferDir(descriptor.path, USERNAME, HOSTNAME, REM_SRC_DIR)
                xfered += 1
        except Exception as e:
            print(f"Error synchronizing {descriptor.name}")
        
        # check that all were transferred
        if xfered != len(packages_for_xfer):
            print(f"Synchronized {xfered} of {len(packages_for_xfer)} packages")
            print(f"Failed to synchronize {len(packages_for_xfer) - xfered} of {len(packages_for_xfer)} packages")
            exit(-2)

        print(f"Synchronized {xfered} of {len(packages_for_xfer)} packages\n\n")

        # Now lets do a remote build
        print("Executing remote build")
        
        # detect if it is a clean build as everything needs to re-build
        if WANT_CLEAN:
            createAndSendBuildScript(USERNAME, HOSTNAME, REMOTE_DIR, 
                                    ["/opt/ros/humble/setup.bash"], 
                                    [])

        else:
            createAndSendBuildScript(USERNAME, HOSTNAME, REMOTE_DIR, 
                                    ["/opt/ros/humble/setup.bash"], 
                                    packages_to_build)

        # run the actual build
        remoteExec("/bin/bash /tmp/deploy_build.bash", USERNAME, HOSTNAME, True)
        
        


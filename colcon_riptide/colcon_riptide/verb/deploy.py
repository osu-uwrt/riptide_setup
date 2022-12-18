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
from fabric import Connection, Config
import os

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
    result = connect.run(cmd, hide=(not printOut))
    return result.exited

def makeRemoteDir(remoteDir, username, address):
    remoteExec(f"mkdir -p {remoteDir}", username, address, False)

def delRemoteDir(remoteDir, username, address):
    remoteExec(f"rm -rf {remoteDir}", username, address, False)

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
        for descriptor in packages_for_xfer:
            print(f"\t{descriptor.name}")
        print("\n\n")

        # get exlpicitly the package paths
        for descriptor in packages_for_xfer:
            print(f"Synchronizing >>> {descriptor.name}")
            xferDir(descriptor.path, USERNAME, HOSTNAME, REM_SRC_DIR)
        


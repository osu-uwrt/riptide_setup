# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Copyright 2022 Cole Tucker
# Licensed under the Apache License, Version 2.0
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint
from colcon_core.argument_parser.destination_collector import \
    DestinationCollectorDecorator
from colcon_core.task import add_task_arguments

from datetime import datetime
from fabric import Connection


class TimesyncVerb(VerbExtensionPoint):
    """synchronizes time between computers with no network"""

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

        decorated_parser = DestinationCollectorDecorator(parser)
        add_task_arguments(decorated_parser, 'colcon_core.task.deploy')
        self.task_argument_destinations = decorated_parser.get_destinations()
  

    def main(self, *, context):  # noqa: D102
        USERNAME = context.args.username
        HOSTNAME = context.args.hostname

        # get the current time string
        date_string = datetime.utcnow().strftime("%a %b %e %r UTC %Y")
        
        # ssh into the remote and set the time 
        date_cmd = f'sudo date --set="{date_string}"'

        cmd_ok = True
        cmd_ok &= remoteExec(date_cmd, USERNAME, HOSTNAME, False) == 0
        cmd_ok &= remoteExec("sudo hwclock --systohc", USERNAME, HOSTNAME, False) == 0
            
        if cmd_ok:
            print(f"Set time on {HOSTNAME} to {date_string}")
        else:
            print(f"Failed to set time on {USERNAME}@{HOSTNAME}")

        
def remoteExec(cmd, username, address, printOut=False, printErr = True):
    connect = Connection(f"{username}@{address}")
    result = -100
    try:
        result = connect.run(cmd, hide=(not printOut)).exited
    except Exception as e:
        if printErr: print(e)
    return result
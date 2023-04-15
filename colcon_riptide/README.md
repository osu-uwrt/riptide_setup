# Colcon Riptide
Colcon Riptide is a custom Colcon plugin supporting development tasks for OSU UWRT. It provides functionality to support target deployment and synchronization as well as time synchronization and other capabilities. The capabilities and their functions are described below. 

## Colcon Deploy
colcon_riptitde adds a command verb called `deploy`. The `deploy` verb has a single required argument for the target hostname ans supports several optional arguments. The exact list and their behavior are outlined below.

An exmple usage can be seen here:
`colcon deploy 192.168.1.105 --username abcd --archive --clean`

### Required Args
* `Hostname`: the hostname argument is the only required argument. This can be either a hostname or an IP address of the target machine. The hostname should already be set up with an SSH key to allow auto login beforehand. If this is not set, the deploy will error out.

### Optional Args
* `--username <name>`: This allows specification of the remote target username to invoke during SSH connections. By default the user is assumed to be `ros` which is UWRT's default target username.

* `--remote_dir <dir_name>`: This argument allows setting the workspace directory to deploy to on the remote target. By default it is set to `~/colcon_deploy`. All packages are transferred to `<dir_name>/src` on the target when deploying.

* `--clean`: This command line flag allows for cleaning the remote dir for the `build`, `install` and `log` directories. This does not delete sources for packages that have been transferred prior. This option will build all packages in the remote workspace unless the `--no_build` flag is used. This is done to make sure dependencies are re-built if there are multiple workspaces deployed. 

* `--no_build`: This command line flag allows skipping the build process on the target. This will cause colcon deploy to only schronize sources. The current build on the target will remain in its last known state.

* `--archive`: This command line flag allows archiving the workspace into a tarball. The tarball is then downloaded onto the host computer in the CWD. The file will be named `<hostname>_<remote_dir_name>_<timestamp>.tar.gz`.

* `--all`: -- TODO This command line flag allows rebuilding all packages in the remote workspace. This can be used to run a full build without having to clean the sources. This is intended to support a multiple workspace deployment.

* colcon package selection args: The command also supports the normal package selection args during the deploy process. This allows package selection for remote deployment and build.

## Colcon Timesync
colcon_riptitde adds a command verb called `timesync`. The `timesync` verb is designed to support synchronizing clocks between a host and a target machine. It has a single argument for the hostname. It requires that passwordless SSH be setup with the remote and passwordless sudo be configured on the remote as well. 

An exmple usage can be seen here:
`colcon timesync 192.168.1.105`

### Required Arguments
* `Hostname`: the hostname argument is the only required argument. This can be either a hostname or an IP address of the target machine. The hostname should already be set up with an SSH key to allow auto login beforehand. If this is not set, the timesync will error out.

### Optional Args
* `--username <name>`: This allows specification of the remote target username to invoke during SSH connections. By default the user is assumed to be `ros` which is UWRT's default target username.


# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import os.path

from colcon_core.package_identification import IgnoreLocationException
from colcon_core.package_identification \
    import PackageIdentificationExtensionPoint
from colcon_core.plugin_system import satisfies_version

IGNORE_MARKER = 'DEPLOY_IGNORE'


class IgnoreDeployPackageIdentification(PackageIdentificationExtensionPoint):
    """Ignore paths containing a `DEPLOY_IGNORE` file."""

    # the priority needs to be fairly high for deploy ignore
    # colcon ignore is 1000 so 999 should be good
    PRIORITY = 999

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageIdentificationExtensionPoint.EXTENSION_POINT_VERSION,
            '^1.0')

    def identify(self, desc):  # noqa: D102
        colcon_ignore = desc.path / IGNORE_MARKER
        if os.path.lexists(str(colcon_ignore)):
            raise IgnoreLocationException()
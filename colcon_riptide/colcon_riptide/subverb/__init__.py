# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Copyright 2022 Cole Tucker
# Licensed under the Apache License, Version 2.0

from pathlib import Path
import shutil

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import instantiate_extensions
from colcon_core.plugin_system import order_extensions_by_name
from scantree import RecursionFilter, scantree


logger = colcon_logger.getChild(__name__)


def add_deploy_subverb_arguments(parser):
    """
    Add the command line arguments for the deploy subverb extensions.

    :param parser: The argument parser
    """
    group = parser.add_argument_group(title='deploy subverb arguments')

    group.add_argument(
        '-y', '--yes',
        action='store_true',
        help='Automatic yes to prompts')

    filter_options = parser.add_argument_group(
        title='Clean filter arguments',
        description='Specify what files and directories to include. All '
        'files and directories (including symbolic links) are included '
        'by default. The --clean-match/--clean-ignore arguments '
        'allows for selection using glob/wildcard (".gitignore style") '
        'path matching. Paths relative to the root `directory` (i.e. '
        'excluding the name of the root directory itself) are matched '
        'against the provided patterns. For example, to only include '
        'Gcov Data files, use: `colcon clean workspace --clean-match '
        '"*.gcda"` or to exclude hidden files and directories use: '
        '`colcon clean workspace --clean-ignore ".*" ".*/"` '
        'which is short for '
        '`colcon clean workspace --clean-match "*" "!.*" "!.*/"`. '
    )
    # # FIXME: Find out how colcon help could display group description
    # filter_options = parser

    filter_options.add_argument(
        '--clean-match',
        nargs='+',
        default=None,
        help='One or several patterns for paths to include. NOTE: '
        'patterns with an asterisk must be in quotes ("*") or the '
        'asterisk preceded by an escape character (\\*).',
        metavar=''
    )
    filter_options.add_argument(
        '--clean-ignore',
        nargs='+',
        default=None,
        help='One or several patterns for paths to exclude. NOTE: '
        'patterns with an asterisk must be in quotes ("*") or the '
        'asterisk preceded by an escape character (\\*).',
        metavar=''
    )
    filter_options.add_argument(
        '--clean-no-linked-dirs',
        action='store_false',
        help='Do not include symbolic links to other directories.'
    )
    filter_options.add_argument(
        '--clean-no-linked-files',
        action='store_false',
        help='Do not include symbolic links to files.'
    )
    filter_options.set_defaults(
        clean_no_linked_dirs=True,
        clean_no_linked_files=True,
    )


def scan_directory(directory, recursion_filter):
    """
    Scan directory with recursion filter.

    The recursion filter includes match patterns or is None.

    :param directory: Path
    :param recursion_filter: RecursionFilter

    :rtype: list
    """
    base_paths = set()
    if not directory.exists():
        return base_paths

    if recursion_filter:
        tree = scantree(
            directory=directory,
            recursion_filter=recursion_filter,
            follow_links=False,
            include_empty=True)
        for filepath in tree.filepaths():
            filepath = Path(filepath)
            if directory in filepath.parents:  # pragma: no branch
                base_paths.add(filepath)
    else:
        base_paths.add(directory)
    return base_paths


def get_recursion_filter(args):
    """
    Get the recursion filter.

    The recursion filter includes match patterns or is None.

    :param args: The parsed command line arguments

    :rtype: RecursionFilter or None
    """
    if args.clean_match or args.clean_ignore:
        match_patterns = get_match_patterns(
            match=args.clean_match,
            ignore=args.clean_ignore)
        recursion_filter = RecursionFilter(
            linked_dirs=args.clean_no_linked_dirs,
            linked_files=args.clean_no_linked_files,
            match=match_patterns
        )
        return recursion_filter
    return None


def get_match_patterns(
    match=None,
    ignore=None
):
    """
    Compose match pattern from list of glob/wildcard (".gitignore style").

    # Arguments
        match: Optional[List[str]] - A list of match-patterns for files to
            *include*. Default `None` which is equivalent to `['*']`,
            i.e. everything is included (unless excluded by arguments below).
        ignore: Optional[List[str]] -  A list of match-patterns for files to
            *ignore*. Default `None` (no ignore patterns).
    """
    match = ['*'] if match is None else list(match)
    ignore = [] if ignore is None else list(ignore)

    match_spec = match + ['!' + ign for ign in ignore]

    def deduplicate(items):
        items_set = set()
        dd_items = []
        for item in items:
            if item not in items_set:
                dd_items.append(item)
                items_set.add(item)

        return dd_items

    return deduplicate(match_spec)


def get_subverb_extensions():
    """
    Get the available subverb extensions.

    The extensions are ordered by their entry point name.

    :rtype: OrderedDict
    """
    extensions = instantiate_extensions(__name__)
    for name, extension in extensions.items():
        extension.SUBVERB_NAME = name
    return order_extensions_by_name(extensions)


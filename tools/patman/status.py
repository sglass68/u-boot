# SPDX-License-Identifier: GPL-2.0+
#
# Copyright 2020 Google LLC
#
"""Talks to the patchwork service to figure out what patches have been reviewed
and commented on.
"""

import collections
import concurrent.futures
import io
from itertools import repeat
import re
import requests

from patman import commit
from patman.patchstream import PatchStream
from patman import terminal
from patman import tout

# Patches which are part of a multi-patch series are shown with a prefix like
# [prefix, version, sequence], for example '[RFC, v2, 3/5]'. All but the last
# part is optional. This decodes the string into groups. For single patches
# the [] part is not present:
# Groups: (ignore, ignore, ignore, prefix, version, sequence, subject)
RE_PATCH = re.compile(r'(\[(((.*),)?(.*),)?(.*)\]\s)?(.*)$')

# This decodes the sequence string into a patch number and patch count
RE_SEQ = re.compile(r'(\d+)/(\d+)')

def to_int(vals):
    """Convert a list of strings into integers, using 0 if not an integer

    Args:
        vals (list): List of strings

    Returns:
        list: List of integers, one for each input string
    """
    out = [int(val) if val.isdigit() else 0 for val in vals]
    return out


class Patch(dict):
    """Models a patch in patchwork

    This class records information obtained from patchwork

    Some of this information comes from the 'Patch' column:

        [RFC,v2,1/3] dm: Driver and uclass changes for tiny-dm

    This shows the prefix, version, seq, count and subject.

    The other properties come from other columns in the display.

    Properties:
        id (str): ID of the patch (typically an integer)
        seq (int): Sequence number within series (1=first) parsed from sequence
            string
        count (int): Number of patches in series, parsed from sequence string
        raw_subject (str): Entire subject line, e.g.
            "[1/2,v2] efi_loader: Sort header file ordering"
        prefix (str): Prefix string or None (e.g. 'RFC')
        version (str): Version string or None (e.g. 'v2')
        raw_subject (str): Raw patch subject
        subject (str): Patch subject with [..] part removed (same as commit
            subject)
    """
    def __init__(self, id):
        super().__init__()
        self.id = id

        self.seq = None
        self.count = None
        self.prefix = None
        self.version = None
        self.raw_subject = None
        self.subject = None

    # These make us more like a dictionary
    def __setattr__(self, name, value):
        self[name] = value

    def __getattr__(self, name):
        return self[name]

    def __hash__(self):
        return hash(frozenset(self.items()))

    def __str__(self):
        return self.raw_subject

    def parse_subject(self, raw_subject):
        """Parse the subject of a patch into its component parts

        See RE_PATCH for details. The parsed info is placed into seq, count,
        prefix, version, subject

        Args:
            raw_subject: Subject string to parse
        """
        self.raw_subject = raw_subject.strip()
        mat = RE_PATCH.search(raw_subject.strip())
        if not mat:
            raise ValueError("Cannot parse subject '%s'" % raw_subject)
        self.prefix, self.version, seq_info, self.subject = mat.groups()[3:]
        mat_seq = RE_SEQ.match(seq_info) if seq_info else False
        if mat_seq is None:
            self.version = seq_info
            seq_info = None
        if self.version and not self.version.startswith('v'):
            self.prefix = self.version
            self.version = None
        if seq_info:
            if mat_seq:
                self.seq = int(mat_seq.group(1))
                self.count = int(mat_seq.group(2))
        else:
            self.seq = 1
            self.count = 1

    def set_url(self, url):
        """Sets the URL for a patch

        Args:
            url (str): URL to set
        """
        self.url = url

def compare_with_series(series, patches):
    """Compare a list of patches with a series it came from

    This prints any problems as warnings

    Args:
        series: Series object
        patches: list of Patch objects

    Returns:
        tuple
            dict:
                key: Commit number (0...n-1)
                value: Patch object for that commit
            dict:
                key: Patch number  (0...n-1)
                value: Commit object for that patch
    """
    # Check the names match
    warnings = []
    patch_for_commit = {}
    all_patches = set(patches)
    for seq, cmt in enumerate(series.commits):
        pmatch = [p for p in all_patches if p.subject == cmt.subject]
        if len(pmatch) == 1:
            patch_for_commit[seq] = pmatch[0]
            all_patches.remove(pmatch[0])
        elif len(pmatch) > 1:
            warnings.append("Multiple patches match commit %d ('%s'):\n   %s" %
                            (seq + 1, cmt.subject,
                             '\n   '.join([p.subject for p in pmatch])))
        else:
            warnings.append("Cannot find patch for commit %d ('%s')" %
                            (seq + 1, cmt.subject))


    # Check the names match
    commit_for_patch = {}
    all_commits = set(series.commits)
    for seq, patch in enumerate(patches):
        cmatch = [c for c in all_commits if c.subject == patch.subject]
        if len(cmatch) == 1:
            commit_for_patch[seq] = cmatch[0]
            all_commits.remove(cmatch[0])
        elif len(cmatch) > 1:
            warnings.append("Multiple commits match patch %d ('%s'):\n   %s" %
                            (seq + 1, patch.subject,
                             '\n   '.join([c.subject for c in cmatch])))
        else:
            warnings.append("Cannot find commit for patch %d ('%s')" %
                            (seq + 1, patch.subject))

    return patch_for_commit, commit_for_patch, warnings

def call_rest_api(subpath):
    """Call the patchwork API and return the result as JSON

    Args:
        subpath: URL subpath to use

    Returns:
        Json result

    Raises:
        ValueError if the URL could not be read
    """
    url = 'https://patchwork.ozlabs.org/api/1.2/%s' % subpath
    response = requests.get(url)
    if response.status_code != 200:
        raise ValueError("Could not read URL '%s'" % url)
    return response.json()

def collect_patches(series, series_id, rest_api=call_rest_api):
    """Collect patch information about a series from patchwork

    Uses the Patchwork REST API to collect information provided by patchwork
    about the status of each patch.

    Args:
        series (Series): Series object corresponding to the local branch
            containing the series
        series_id (str): Patch series ID number
        rest_api: API function to call to access Patchwork, for testing

    Returns:
        list: List of patches sorted by sequence number, each a Patch object

    Raises:
        ValueError: if the URL could not be read or the web page does not follow
            the expected structure
    """
    data = rest_api('series/%s/' % series_id)

    # Get all the rows, which are patches
    patch_dict = data['patches']
    count = len(patch_dict)
    num_commits = len(series.commits)
    if count != num_commits:
        tout.Warning('Warning: Patchwork reports %d patches, series has %d' %
                     (count, num_commits))

    patches = []

    # Work through each row (patch) one at a time, collecting the information
    warn_count = 0
    for pw_patch in patch_dict:
        patch = Patch(pw_patch['id'])
        patch.parse_subject(pw_patch['name'])
        patches.append(patch)
    if warn_count > 1:
        tout.Warning('   (total of %d warnings)' % warn_count)

    # Sort patches by patch number
    patches = sorted(patches, key=lambda x: x.seq)
    return patches

def find_new_responses(new_rtag_list, seq, cmt, patch, rest_api=call_rest_api):
    """Find new rtags collected by patchwork that we don't know about

    This is designed to be run in parallel, once for each commit/patch

    Args:
        new_rtag_list (list): New rtags are written to new_rtag_list[seq]
            list, each a dict:
                key: Response tag (e.g. 'Reviewed-by')
                value: Set of people who gave that response, each a name/email
                    string
        seq (int): Position in new_rtag_list to update
        cmt (Commit): Commit object for this commit
        patch (Patch): Corresponding Patch object for this patch
        rest_api: API function to call to access Patchwork, for testing
    """
    if not patch:
        return

    # Get the content for the patch email itself as well as all comments
    data = rest_api('patches/%s/' % patch.id)
    pstrm = PatchStream.ProcessText(data['content'], True)

    rtags = collections.defaultdict(set)
    for response, people in pstrm.commit.rtags.items():
        rtags[response].update(people)

    data = rest_api('patches/%s/comments/' % patch.id)

    for comment in data:
        pstrm = PatchStream.ProcessText(comment['content'], True)
        for response, people in pstrm.commit.rtags.items():
            rtags[response].update(people)

    # Find the tags that are not in the commit
    new_rtags = collections.defaultdict(set)
    base_rtags = cmt.rtags
    for tag, people in rtags.items():
        for who in people:
            is_new = (tag not in base_rtags or
                      who not in base_rtags[tag])
            if is_new:
                new_rtags[tag].add(who)
    new_rtag_list[seq] = new_rtags

def show_responses(rtags, indent, is_new):
    """Show rtags collected

    Args:
        rtags (dict): review tags to show
            key: Response tag (e.g. 'Reviewed-by')
            value: Set of people who gave that response, each a name/email string
        indent (str): Indentation string to write before each line
        is_new (bool): True if this output should be highlighted

    Returns:
        int: Number of review tags displayed
    """
    col = terminal.Color()
    count = 0
    for tag, people in rtags.items():
        for who in people:
            terminal.Print(indent + '%s %s: ' % ('+' if is_new else ' ', tag),
                           newline=False, colour=col.GREEN, bright=is_new)
            terminal.Print(who, colour=col.WHITE, bright=is_new)
            count += 1
    return count

def check_patchwork_status(series, series_id, rest_api=call_rest_api):
    """Check the status of a series on Patchwork

    This finds review tags and comments for a series in Patchwork, displaying
    them to show what is new compared to the local series.

    Args:
        series (Series): Series object for the existing branch
        series_id (str): Patch series ID number
        branch (str): Existing branch to update
        rest_api: API function to call to access Patchwork, for testing
    """
    patches = collect_patches(series, series_id, rest_api)
    col = terminal.Color()
    count = len(series.commits)
    new_rtag_list = [None] * count
    review_list = [None] * count

    patch_for_commit, commit_for_patch, warnings = compare_with_series(series,
                                                                       patches)
    for warn in warnings:
        tout.Warning(warn)

    patch_list = [patch_for_commit.get(c) for c in range(len(series.commits))]

    with concurrent.futures.ThreadPoolExecutor(max_workers=16) as executor:
        futures = executor.map(
            find_new_responses, repeat(new_rtag_list), range(count),
            series.commits, patch_list, repeat(rest_api))
    for fresponse in futures:
        if fresponse:
            raise fresponse.exception()

    num_to_add = 0
    for seq, cmt in enumerate(series.commits):
        patch = patch_for_commit.get(seq)
        if not patch:
            continue
        terminal.Print('%3d %s' % (patch.seq, patch.subject[:50]),
                       colour=col.BLUE)
        cmt = series.commits[seq]
        base_rtags = cmt.rtags
        new_rtags = new_rtag_list[seq]

        indent = ' ' * 2
        show_responses(base_rtags, indent, False)
        num_to_add += show_responses(new_rtags, indent, True)

    terminal.Print("%d new response%s available in patchwork" %
                   (num_to_add, 's' if num_to_add != 1 else ''))

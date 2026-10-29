#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Brickwright contributors
"""Apply the bounded read-only observer patch to the frozen native baseline."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

EXPECTED = 'afdb16ddd4f4d5f2ad16cc8814abe0c80114464e'
ALLOWED = {'Jolt/Physics/Constraints/HingeConstraint.h',
           'Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h',
           'Jolt/Physics/Constraints/ConstraintPart/SpringPart.h'}

def prepare(root, patch, receipt, expected=EXPECTED, allowed=ALLOWED):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args])
    def index():
        return git('ls-files', '--stage', '-z')
    def census():
        entries = {}
        skipped = {line[2:].decode() for line in git('ls-files', '-v', '-z').split(b'\0')
                   if line.startswith(b'S ')}
        for row in index().split(b'\0'):
            if not row:
                continue
            metadata, name = row.split(b'\t', 1)
            mode, blob, stage = metadata.decode().split()
            path = name.decode()
            if stage != '0':
                raise RuntimeError('unmerged index')
            p = root / path
            entry = {'gitBlob': blob, 'gitMode': mode}
            if p.exists() or p.is_symlink():
                data = p.read_bytes()
                entry.update(materialized=True, sha256=hashlib.sha256(data).hexdigest(),
                             filesystemMode=p.lstat().st_mode & 0o777)
            elif path in skipped:
                if path.startswith('Jolt/') or path in allowed:
                    raise RuntimeError('compiled or observer input omitted: ' + path)
                entry.update(materialized=False, classification='skip-worktree; blob/mode only')
            else:
                raise RuntimeError('unexpected missing tracked file: ' + path)
            entries[path] = entry
        return entries
    if git('rev-parse', 'HEAD').decode().strip() != expected:
        raise RuntimeError('wrong native observational baseline')
    if git('status', '--porcelain', '--untracked-files=all').strip():
        raise RuntimeError('baseline is not pristine')
    before_index, before = index(), census()
    subprocess.run(['git', '-C', str(root), 'apply', '--check', str(patch.resolve())], check=True)
    subprocess.run(['git', '-C', str(root), 'apply', str(patch.resolve())], check=True)
    after = census()
    changed = {p for p in before if before[p] != after[p]}
    if changed != allowed or before.keys() != after.keys() or before_index != index():
        raise RuntimeError('observer patch changed unrelated entries or index')
    if any(before[p].get('filesystemMode') != after[p].get('filesystemMode') for p in before):
        raise RuntimeError('observer patch changed file modes')
    status = git('status', '--porcelain', '--untracked-files=all', '-z').split(b'\0')
    if {line.decode() for line in status if line} != {' M ' + p for p in allowed}:
        raise RuntimeError('unexpected baseline working-tree status')
    receipt.write_text(json.dumps({'schema': 'bw.native-afdb-observer-source.v1',
        'nativeBase': expected, 'nativeTree': git('rev-parse', 'HEAD^{tree}').decode().strip(),
        'classification': 'afdb plus read-only inline header observers; not pristine afdb bytes',
        'patchSha256': hashlib.sha256(patch.read_bytes()).hexdigest(),
        'indexSha256': hashlib.sha256(before_index).hexdigest(),
        'trackedEntryCount': len(before), 'indexUnchanged': True,
        'materializedEntryCount': sum(e['materialized'] for e in before.values()),
        'allOtherPresentEntriesUnchanged': True,
        'allUnmaterializedIndexEntriesUnchanged': True,
        'beforeEntries': before, 'afterEntries': after,
        'headers': {p: {'before': before[p], 'after': after[p]} for p in sorted(allowed)}}, indent=2) + '\n')

if __name__ == '__main__':
    prepare(*map(Path, sys.argv[1:]))

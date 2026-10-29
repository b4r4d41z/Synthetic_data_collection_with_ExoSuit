"""Git identity of the executing Synthetic code, separate from asset revisions."""
import hashlib
import os
from pathlib import Path
import re
import subprocess

REPOSITORY = 'https://github.com/b4r4d41z/Synthetic_data_collection_with_ExoSuit'


def recording_code_provenance(module_file):
    result = dict(repository=REPOSITORY, status='unavailable', commit_sha=None,
                  dirty=None, dirty_sha256=None)
    try:
        module = Path(module_file).resolve(strict=True)
        result['executing_module_sha256'] = hashlib.sha256(module.read_bytes()).hexdigest()
        root = next(p for p in module.parents if (p / '.git').exists())

        def git(*args):
            return subprocess.check_output(['git', '-C', str(root), *args],
                                           stderr=subprocess.DEVNULL, timeout=10)

        remote = git('remote', 'get-url', 'origin').decode().strip().removesuffix('.git')
        if remote not in (REPOSITORY, 'git@github.com:b4r4d41z/Synthetic_data_collection_with_ExoSuit'):
            raise ValueError('Executing module is not in the Synthetic repository')
        # A copied colcon install may be older than its checkout. Never claim
        # that HEAD describes the executing module in that case.
        source = root / 'exo2isaacsim/src/isaac_eff_control/isaac_eff_control' / module.name
        if source.read_bytes() != module.read_bytes():
            raise ValueError('Installed module differs from repository source')
        sha = git('rev-parse', '--verify', 'HEAD').decode().strip()
        if not re.fullmatch(r'[0-9a-f]{40}|[0-9a-f]{64}', sha):
            raise ValueError('Invalid Git object ID')
        diff = git('-c', 'diff.algorithm=myers', 'diff', '--binary', '--full-index',
                   '--no-color', '--no-renames', '--src-prefix=a/', '--dst-prefix=b/',
                   '--no-ext-diff', '--no-textconv', 'HEAD', '--')
        untracked = sorted(filter(None, git('ls-files', '--others', '--exclude-standard', '-z').split(b'\0')))
        digest = hashlib.sha256()

        def add(data):
            digest.update(len(data).to_bytes(8, 'big'))
            digest.update(data)

        add(diff)
        for name in untracked:
            path = root / os.fsdecode(name)
            add(name)
            add(b'symlink' if path.is_symlink() else b'file')
            add(str(path.lstat().st_mode & 0o777).encode())
            add(os.fsencode(os.readlink(path)) if path.is_symlink() else path.read_bytes())
        dirty = bool(diff or untracked)
        result.update(status='available', commit_sha=sha, dirty=dirty,
                      dirty_sha256=digest.hexdigest() if dirty else None,
                      dirty_digest_format='sha256-length-prefixed-head-diff-and-sorted-untracked-v1')
    except (OSError, ValueError, StopIteration, subprocess.SubprocessError) as error:
        result['unavailable_reason'] = str(error) or type(error).__name__
    return result

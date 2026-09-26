# Release preparation checklist

The author approved publication to `iori-shino/ros2-mobile-manipulator` with public visibility. The local pre-publication check results are recorded in `evidence/final_checks.json`; its GitHub flags describe the preparation checkpoint, before upload.

## Included

- The two ROS packages, portable acceptance utilities, current README and technical documents.
- Ten author-confirmed self-modelled STL files with SHA-256 inventory; sensor and room appearances use project primitives.
- Reviewed application screenshots, a partial example map and selected numeric test records with source hashes.
- No downloaded standard-part CAD, commercial CAD software or upstream dependency source.

## Excluded from the publication snapshot

- `.git` and all earlier development commits: old history includes local handoff paths and machine information.
- Local handoff/agent notes, SSH helpers, session PID records, raw desktop screenshots and long test streams.
- `build/`, `install/`, `log/`, caches, IDE state, credentials, generated maps and release working directories.

`.gitignore` alone does not remove a file from past commits. The release preparation tool copies an explicit allowlist into a new directory and never copies the old `.git`. Publish that reviewed snapshot, not the development repository's history.

## Checks performed before publication approval

The final check artifact records the actual outcome of:

1. Rebuilding both packages from the clean copied source with no dependency on the development overlay.
2. Resolving the documented launch files and script entry points.
3. Core/GUI ownership tests, isolated ROS tests, live GUI interface checks and the system health check.
4. A secret/path/IP pattern scan of the publication tree, file-size review, asset hash match, Python syntax/package XML and local Markdown-link checks.
5. Manual screenshot review; binary image contents cannot be certified by a text-only secret scanner.
6. Explicit source-tree/publication-tree Git status and confirmation that no remote push happened.

The automated scan reports locations without printing matched secret values. Pattern scanning is not a mathematical guarantee of absence of every possible secret; it is combined with the constrained file list and manual review.

## Publication decisions

- Approved repository: `iori-shino/ros2-mobile-manipulator`, public visibility.
- The author explicitly approved creating the public remote and pushing this reviewed snapshot.
- Publication uses authenticated GitHub CLI; credentials are not included in the repository.
- Whether to adopt an open-source licence, and whether code and CAD should share the same terms. No licence has been invented or granted on the author's behalf; the existing Proprietary metadata is retained.

No robot functionality is added as part of publication. Final résumé writing remains outside this task; `PROJECT_FACTS.md` is the factual handoff material.

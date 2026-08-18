# Offline campaign results

**Citable results live ONLY in clean `run_<full_git_hash>/` directories** whose
manifests report `git_dirty=false` and whose `checksums.sha256` covers every
artifact. Dirty diagnostics use unique
`run_<full_git_hash>_dirty_<UTC_timestamp>/` directories and must not be cited.

After the 2026-08-17 certificate repair, a new clean campaign must be run before
paper figures or tables are frozen. Earlier clean directories remain historical
evidence for their corresponding code versions, not evidence for the revised
correlated-odometry certificate.

Loose CSVs at this level are SUPERSEDED artifacts from before the
2026-08-16 verification review (stale code, invalid conditioning statistic,
incomplete smoother measurement — see EXECUTION_LOG.md "Session 4"). They are
retained only because the sandbox mount cannot delete files; do not cite them.
Delete them from Windows at leisure.

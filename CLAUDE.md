# Claude Code Rules for This Project

## Git Rules

- **NEVER push to GitHub automatically** - Only push when the user explicitly says "push" or "push to github"
- **NEVER commit automatically** (changed 2026-07-12; commits used to be fine) - Only commit when the user explicitly says "commit". Leave changes in the working tree after compile-checking them.
- Before committing when asked: run `git status`/`git diff --stat` and confirm the diff contains ONLY your intended changes (two-context stale-file sweeps have silently reverted other sessions' work twice)

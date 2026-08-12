# Branching workflow

Three tiers, from stable to throwaway:

```
main ◀── develop ◀── experiment/<topic>
stable     integration     playground
```

| Branch | Rule |
|--------|------|
| `main` | Always runnable. Only receives merges from `develop` after the demos/examples actually run. This is what visitors to the public repo see. |
| `develop` | Day-to-day work: new examples, docs, refactors. Merge to `main` once verified. |
| `experiment/<topic>` | Learning and testing — anything goes. Branch off `develop`. If it works out, merge back to `develop`; if not, delete it guilt-free. |

## Daily commands

```bash
# normal work
git checkout develop
# ... edit, commit ...
git push origin develop

# start an experiment (e.g. trying AprilTag detection)
git checkout develop
git checkout -b experiment/apriltag-vision
# ... hack freely, commit often, broken code is fine here ...
git push -u origin experiment/apriltag-vision      # only if you want a backup

# experiment succeeded -> fold it into develop
git checkout develop
git merge experiment/apriltag-vision
git branch -d experiment/apriltag-vision           # local cleanup
git push origin --delete experiment/apriltag-vision  # remote cleanup (if pushed)

# develop is verified -> release to main
git checkout main
git merge develop
git push
git checkout develop                               # go back to working
```

## Conventions

- **Naming**: `experiment/<short-topic>` for learning/testing,
  `feature/<name>` for planned additions, `fix/<name>` for bug fixes.
- **Verify before `main`**: run the touched demos/examples once
  (e.g. `ros2 launch examples/panda_arm/05_vision_sort.launch.py`) before
  merging `develop` into `main`.
- **Prune**: delete experiment branches after merging or abandoning;
  `git fetch --prune` cleans up stale remote refs locally.
- **Solo PRs are still useful**: opening a PR from `develop` to `main`
  (`gh pr create --base main`) gives you a diff review page and a place to
  run `/code-review` before releasing.

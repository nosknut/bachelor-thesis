# https://www.baeldung.com/ops/git-assume-unchanged-skip-worktree

git update-index --no-skip-worktree .vscode/arduino.json
git update-index --no-skip-worktree .vscode/c_cpp_properties.json
git update-index --no-skip-worktree .vscode/settings.json
git update-index --no-skip-worktree .vscode/tasks.json
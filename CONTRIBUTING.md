# Contributing to Experimental Robotics

This guide helps students contribute safely and effectively to this repository.

## Overview
We coordinate all work through GitHub issues first, then collaborate on feature branches and pull requests to keep `main` stable.

> [!IMPORTANT]
> Never push directly to main. All changes go through pull requests for review.

---

## Start With an Issue
1. **Search existing issues** to avoid duplicates. Comment on an open issue if you'd like to help.
2. **Open a new issue using the right template:**
   - `Bug Report` – problems with the environment (`.github/ISSUE_TEMPLATE/bug_report.md`)
   - `Feature Request` – enhancements or new content (`.github/ISSUE_TEMPLATE/feature_request.md`)
   - `Question` – clarifications or documentation gaps (`.github/ISSUE_TEMPLATE/question.md`)
3. **Fill out every section** of the template. Detailed logs, screenshots, environment info, and what you've already tried help triage faster.
4. **State your intent to work on it** (check the box in the feature template or add a short note) so the maintainer can assign you.

> [!NOTE]
> Issues are the single source of truth for who is working on what. Opening an issue comes before writing code, tests, or docs.

Once the maintainer confirms or labels the issue, coordinate next steps in the issue thread. Use the issue number in your branch name (e.g., `feature/42-lidar-filtering`) and mention the issue in your PR description to link them automatically.

---

## Coordinating Your Work
Keep all planning and status updates in the issue thread so everyone can see what's happening. Post a short update when you start, pause, or hand off work.

Maintainer: Omotoye  
Email: <omotoye.adekoya@edu.unige.it>

Use email only if something is urgent (for example, you need a quick approval or the issue has been quiet for a couple of days). Otherwise, keep the asynchronous discussion inside GitHub.

> [!TIP]
> Reference the issue number everywhere: in branch names (`feature/42-lidar-filtering`), commit messages (`Fix lidar reset #42`), and pull requests. It keeps history easy to follow.

For larger features, open a draft PR early from your fork, link it to the issue, and push incremental commits so others can follow progress. If overlap happens, coordinate directly in the issue to decide merge order; whoever goes second rebases or merges `upstream/main` afterward.

---

## Quick Start Workflow

### 1) Fork and Clone
Since you don't have direct push access, you must work through your fork:
```bash
# One-time: fork this repo on GitHub (click "Fork" button), then clone YOUR fork
git clone https://github.com/<your-username>/experimental_robotics.git
cd experimental_robotics

# One-time: add upstream to track the original repo
git remote add upstream https://github.com/RICE-unige/experimental_robotics.git
```

### 2) Create Your Feature Branch
Always start from an updated main:
```bash
# Update your fork's main from upstream
git fetch upstream
git checkout main
git merge upstream/main
git push origin main  # Push to YOUR fork

# Create your feature branch
git checkout -b feature/your-feature-name
```

Branch naming conventions:
- feature/description – New features (e.g., feature/lidar-filtering)
- fix/description – Bug fixes (e.g., fix/rviz-goal)
- docs/description – Docs only (e.g., docs/update-readme)
- refactor/description – Code cleanup (e.g., refactor/sensor-config)

Include the related GitHub issue number when possible (e.g., `feature/42-lidar-filtering`).

> [!TIP]
> Use specific names. feature/new-stuff is vague; feature/gazebo-camera-calibration is clear.

### 3) Make Your Changes
```bash
# Edit files and test locally (ideally inside the Docker environment)

# Stage and commit
git add .
git commit -m "Add lidar filtering for obstacle detection"

# Push to YOUR fork
git push origin feature/your-feature-name
```

> [!NOTE]
> Commit often with clear messages to ease reviews and tracking.

### 4) Keep Your Branch Updated
If your branch lives for a while, sync with main periodically:
```bash
# Fetch latest changes from upstream and your fork
git fetch upstream

# Merge upstream/main into your branch
git checkout feature/your-feature-name
git merge upstream/main

# Resolve conflicts if any, then push to your fork
git push origin feature/your-feature-name
```

---

## Creating a Pull Request

1) Push your branch to your fork:
```bash
git push origin feature/your-feature-name
```
2) On GitHub, go to your fork and click "Compare & pull request" (or create a new PR manually).
3) Ensure the PR is from `your-fork:feature/your-feature-name` → `RICE-unige:main`.

Title: Start with a verb and be specific.
> [!TIP]
> ✅ Add lidar-based obstacle detection for nav2  
> ❌ Update code

Description: What changed, why, how to test. Include screenshots/logs if helpful and add a line such as `Closes #42` so GitHub links the PR to the issue.

Drop the PR link in the original issue thread so others get notified. Email the maintainer only if a faster turnaround is needed; otherwise GitHub notifications are enough.

> [!NOTE]
> As a student, you cannot push to the main repository or merge PRs - only the maintainer (Omotoye) can merge. This keeps the main branch stable.

After your PR is merged:
```bash
# Sync your fork's main with upstream
git checkout main
git fetch upstream
git merge upstream/main
git push origin main  # Update your fork

# Delete your local feature branch
git branch -d feature/your-feature-name

# Optionally delete the remote branch on your fork
git push origin --delete feature/your-feature-name
```


## Issue Template Cheat Sheet
- **Bug Report** – Use when something breaks. Include reproduction steps, expected vs actual behaviour, environment details, logs, and screenshots.
- **Feature Request** – Pitch improvements or new scenarios. Explain the problem, outline your proposed solution, note any alternatives, and tick whether you want to implement it.
- **Question** – Ask for clarification. Provide context, what documentation you've checked, and what you have already tried.

General tips:
- Use descriptive titles (`[BUG] Docker build fails on ARM64`) so triage is fast.
- Fill every required field; placeholders exist so we get the info we need on the first pass.
- Apply labels if you have permission; otherwise the maintainer will handle it.

---

## Best Practices
**Do:**
- Open or claim an issue before touching code so work is visible
- Fork the repository first (you cannot push to the main repo directly)
- Reference the issue number in branches, commits, and PRs; open a draft PR early for larger features
- Sync with upstream main often; keep PRs small and focused
- Test in the Docker environment; add/update docs as needed
- Write clear commit messages and PR descriptions
- Keep your fork's main branch in sync with upstream

**Don't:**
- Don't commit directly to your fork's main branch (always use feature branches)
- Don't commit large binaries or secrets (use volumes, releases, or storage services instead)
- Don't create PRs with unresolved merge conflicts

---

## Security & Privacy
- Never commit secrets (API keys, tokens). Use environment variables or local config files not checked into git.
- Review diffs for accidental credentials before pushing.

---

## Questions
- Start with the `Question` issue template.
- Still stuck? Email the maintainer: <omotoye.adekoya@edu.unige.it>.

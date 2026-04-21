# AI Coordination — JONNY5

This file is the shared coordination channel between ChatGPT, Claude Code, and Alessandro.

Goal: allow safe, auditable collaboration through GitHub without touching runtime files blindly.

---

## Roles

### Alessandro

Owner and final decision maker.

Responsibilities:

- Approves actions before code-changing work.
- Runs local commands on Windows/Raspberry/STM32 when needed.
- Decides whether to commit, push, deploy, or test on hardware.
- Stops unsafe or unclear workflows.

### ChatGPT

Architecture reviewer, planning assistant, and external GitHub reviewer.

Typical tasks:

- Review commits, diffs, issues, and PRs.
- Produce engineering plans and safety checklists.
- Identify risks, regressions, and missing tests.
- Write task prompts for Claude Code.
- Create or update GitHub coordination issues/files when explicitly requested.

Limits:

- Does not directly control Alessandro's PC.
- Does not directly control Claude Code.
- Does not deploy to Raspberry/STM32.
- Should not modify runtime-critical files without explicit user request.

### Claude Code

Local code executor and repository worker.

Typical tasks:

- Inspect the local repository.
- Apply scoped patches.
- Run local tests and static checks.
- Commit and push when explicitly authorized.
- Report exact files changed and commands executed.

Limits to be respected:

- Do not modify code unless the task explicitly allows it.
- Do not refactor outside the requested scope.
- Do not deploy or touch runtime targets unless explicitly authorized.
- Do not delete files unless explicitly authorized.
- Do not auto-commit before tests unless explicitly requested.

---

## Safe Collaboration Workflow

### Standard loop

1. ChatGPT reviews the current GitHub state.
2. ChatGPT writes a precise task for Claude Code.
3. Alessandro pastes the task into Claude Code.
4. Claude Code performs only the requested work locally.
5. Claude Code reports:
   - files changed
   - commands executed
   - test results
   - residual risks
6. Alessandro decides whether to commit and push.
7. ChatGPT reviews the resulting commit or PR.

---

## Communication Channels

Use these in order of preference:

1. GitHub issue for discussions and decisions.
2. This file for persistent workflow rules and task state.
3. Commit messages for completed atomic changes.
4. Pull requests for larger reviewable changes.

Avoid using hidden local state as the only source of truth.

---

## Safety Rules for JONNY5

### Never do without explicit approval

- Deploy to Raspberry Pi runtime target.
- Flash STM32 firmware.
- Restart systemd services.
- Modify files under runtime target paths.
- Delete backups, logs, calibration files, or configuration files.
- Force-push main.
- Rewrite Git history.
- Run broad cleanup commands.

### Allowed only when scoped

- Edit source files.
- Edit dashboard files.
- Edit test scripts.
- Add documentation.
- Create small helper scripts.
- Run local tests.
- Commit and push.

---

## Required Task Format for Claude Code

Every Claude Code task should use this structure:

```text
REPO:
<absolute local path>

OBJECTIVE:
<single clear objective>

SCOPE:
<allowed files/directories>

DO NOT:
<forbidden actions>

STEPS:
1. <step>
2. <step>
3. <step>

VALIDATION:
<commands/tests to run>

OUTPUT REQUIRED:
- files changed
- commands executed
- test results
- residual risks

COMMIT/PUSH:
Allowed: yes/no
```

---

## Required Report Format from Claude Code

```text
EXECUTIVE SUMMARY
<what was done or found>

FILES CHANGED
<list>

COMMANDS RUN
<list>

VALIDATION RESULT
<test/check results>

RISKS / NOTES
<remaining concerns>

COMMIT STATUS
<not committed / committed hash / pushed hash>
```

---

## Current Coordination Issue

GitHub issue:

- #1 — Question: Can Claude Code coordinate with ChatGPT via GitHub/Internet workflow?

Purpose:

- Determine exactly how Claude Code can read/write GitHub issues, comments, files, branches, and PRs from Alessandro's local environment.
- Establish the safest bridge between Claude Code and ChatGPT.

---

## Current Active Topic

DLS ASSIST integration for JONNY5 VR teleoperation.

Recent state:

- DLS ASSIST integration committed in commit `7ae1232`.
- Technical review found no blocking risks.
- Hardening patch was applied locally by Claude Code:
  - physical backup in `verify_dls_assist_smoke.py`
  - clearer DLS gain UI label
  - independent reading of `assistMode` / `assistDls`
  - non-spam log for ASSIST mode transition
- Hardening patch was not committed/pushed at the time this coordination file was created.

Recommended next action:

- Commit and push the DLS hardening patch after Alessandro approval.
- Then ChatGPT reviews the new commit.

---

## First-Test DLS Parameters — Conservative Proposal

Use only for controlled test, with E-stop ready and no payload.

```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.08,
    "lambdaMax": 0.12,
    "manipThresh": 0.0005,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.08
  }
}
```

Rationale:

- Low gain for first contact.
- Increased damping for stability near singularity.
- Reduced joint/cartesian step limits.
- Mild null-space pull toward HOME.

---

## Notes

This file is not runtime configuration.
It is documentation and coordination only.
It must not be parsed by runtime services.

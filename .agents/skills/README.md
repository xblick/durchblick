# Agent skills of Durchblick

This directory holds the project's agent skills. A skill is a folder with a `SKILL.md` that teaches a coding agent one
procedure that recurs in this repository, for example building a node or benchmarking nodes.
An agent loads a skill's name and description at start, and its full text only when a task needs it.

The format is the open [Agent Skills](https://github.com/agentskills/agentskills) specification. Codex, Cursor, Gemini CLI, GitHub Copilot
and other agents read this directory directly. Claude Code reads `.claude/skills`, which is a symbolic link to this
directory.

## Rules for a skill

These rules follow the [best practices](https://github.com/agentskills/agentskills/tree/main/docs/skill-creation) of
the Agent Skills project.

**Scope and name**

1. **Name it `xblick-<verb>-<object>`**, for example `xblick-add-camera`. The prefix keeps a project skill
   apart from personal skills of the same name, which would otherwise shadow it.
2. **Cover one coherent unit of work.** A skill that is too narrow loads together with others for one task. A skill
   that is too broad triggers on the wrong tasks.
3. **Abstract the procedure.** A skill teaches how to approach a kind of task. It names no batch, seed, date or result
   of one experiment. Those belong in the documentation or the roadmap.
4. **Start from real work.** Write or improve a skill from a task that was actually done: the steps that worked, the
   corrections the user made, the facts the agent did not know.

**Frontmatter**

5. **Use only the portable fields**: `name`, `description`, `license`, `compatibility`, `metadata`,
   `allowed-tools`. Fields that only Claude Code knows break uploads to other tools.
6. **Write the description as an instruction to the agent**: "Use this skill when the user wants ...". Describe the
   user's intent, not the mechanics. List the indirect phrasings too ("can we run SAM 4 live"). Name the neighbouring
   skills it is not for. At most 1024 characters.

**Body**

7. **Add only what the agent lacks.** Cut every sentence the agent would get right without it: general knowledge,
   explanations of standard tools. Link the guide in `docs/` for the background.
8. **Lead with a progress checklist** for a workflow of several steps, and state when the work is done.
9. **Give defaults, not menus.** Name one way and mention an alternative briefly.
10. **Explain why** for each rule that is not obvious. An agent that knows the reason decides better in a case the
    skill did not foresee.
11. **Keep a `## Gotchas` section in `SKILL.md`**: facts that defy reasonable assumptions, each with its fix. When the
    user corrects the agent during a task, the correction belongs here.
12. **Tell the agent when to load a reference**: "Read `references/commands.md` now" or "if the render hangs". A bare
    "see references/" loads nothing.
13. **Give a template for the report** that the agent hands back.
14. **Stay under 500 lines.** Move detail into `references/`, one level deep.

**Scripts**

15. **Put repeated, deterministic work into a script** in `scripts/`. Call it with a path relative to the skill.
16. **Design the script for an agent**: no interactive prompts, `--help` with examples, error messages that say what
    to do, distinct exit codes, structured output (JSON lines) on stdout and diagnostics on stderr, bounded output.

**Review**

17. **Review a skill like code.** An agent runs its instructions with the user's permissions. A new or changed skill
    goes through a merge request.
18. **Keep always-on rules in `AGENTS.md`.** A skill holds a procedure that only some tasks need.

## Layout of a skill

```
xblick-<name>/
  SKILL.md                    the procedure, under 500 lines
  references/*.md             detail that SKILL.md loads on demand
  scripts/*                   deterministic steps
  evals/evals.json            test cases: prompt, expected output, assertions
  evals/trigger_queries.json  prompts that should and should not trigger the skill
```

## Evaluate

Two kinds of evaluation keep a skill honest. Run them when you write a skill and when you change it.

- **Output quality** (`evals/evals.json`): 2 to 3 realistic prompts, each with the expected output and checkable
  assertions. Run each prompt in a fresh session with the skill and without it (or with the previous version), grade
  every assertion with evidence, and compare. Remove assertions that pass in both runs, because they measure nothing.
- **Triggering** (`evals/trigger_queries.json`): about 8 prompts that should trigger the skill and 8 near misses that
  should not. Near misses share words with the skill but need something else. Run each 3 times and compute the
  trigger rate. Improve the description on part of the prompts and check it on the rest, so that it does not overfit
  to the wording of the test.

The [`skill-creator`](https://github.com/anthropics/skills/tree/main/skills/skill-creator) skill automates both loops
in Claude Code.

## Template

```markdown
---
name: xblick-<verb>-<object>
description: >
  Use this skill when the user wants <intent>, also when they only <indirect phrasing>. It <what it does in one
  sentence>. Not for <neighbouring task> (use xblick-<other>).
compatibility: Durchblick 0.5.x
---

# <Title>

Done means: <the observable end state>.

## Progress

- [ ] 1. ...
- [ ] 2. ...

## 1. ...

<Steps. One default per step. The reason where it is not obvious.>

## Report

<A template of what the agent hands back.>

## Gotchas

- <Fact that defies a reasonable assumption: what happens, and the fix.>

## When this skill was wrong

If a step failed or misled, propose a fix of this skill to the user after the task.
```

## Validate

`skills-ref` checks the frontmatter and the naming conventions. It lives in the Agent Skills reference repository.
Install it once in an environment of its own, then validate every skill:

```bash
git clone --depth 1 https://github.com/agentskills/agentskills.git /tmp/agentskills
(cd /tmp/agentskills/skills-ref && UV_PROJECT_ENVIRONMENT=/tmp/skills-ref-venv uv sync)
for d in .agents/skills/*/; do /tmp/skills-ref-venv/bin/skills-ref validate "$d"; done
```

All four skills passed on 2026-11-11. Claude Code also offers `claude plugin validate .claude/skills`.

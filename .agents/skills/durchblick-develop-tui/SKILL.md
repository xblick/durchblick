---
name: durchblick-develop-tui
description: >
  Use this skill when the user wants to change, extend or fix Durchblick's terminal interface (`durchblick` or
  `blick` without a command): its tabs Topics, Nodes, Graph, Rerun and Parameters, the connect dialog, keys, layout,
  or its screenshots in the documentation, also when they only describe what looks wrong ("the graph is hard to
  read", "setting a parameter in blick is clumsy", "the connect window should remember the host", "the table is cut off at 80
  columns"). It changes the interface without disturbing the nodes it watches, tests it headless and in a real
  terminal, and regenerates the screenshots. Not for a node's own parameters or options (use durchblick-build-node)
  and not for starting or measuring nodes (use durchblick-run-nodes or durchblick-benchmark-nodes).
compatibility: Durchblick 0.4.x, uv, virtui for the real-terminal tests, Playwright's Chromium for the screenshots
---

# Develop the terminal interface

Done means: the change works in a real terminal at 80 x 24 and at a large size, the headless and real-terminal tests
pass, the screenshots in `docs/media/tui/` are regenerated and inspected, and `docs/guides/tui.md`, the CHANGELOG and
the version say what changed.

Background: [Terminal interface](../../../docs/guides/tui.md), [Sessions](../../../docs/concepts/sessions.md),
[Parameters](../../../docs/reference/parameters.md).

## Progress

- [ ] 1. The wish restated as behaviour, and the cost to the nodes checked
- [ ] 2. Changed in the right module
- [ ] 3. Tested headless
- [ ] 4. Tested in a real terminal at both sizes
- [ ] 5. Screenshots regenerated and inspected
- [ ] 6. Documented and committed

## 1. Restate the wish and check its cost

Write the change as what the user sees and does, for example "Enter on a prefix folds it". Then ask what it costs the
running nodes, because the interface watches a robot's perception while it runs:

- **A query costs nothing** (admin space, `<base>/info`, `durchblick/**/status`, liveliness, parameter services).
- **A subscription can cost a lot.** A camera produces a representation (raw, compressed, shm, cuda) only while
  somebody subscribes to it, so subscribing to an image key makes the camera encode or copy frames. The interface
  subscribes by default only to `**/diagnostics`. Everything else stays behind an explicit key, as `m` is.
- **Nothing in the interface knows a particular node.** Names, forms and edges come from the admin space,
  descriptors and info records, so that a new node shows up without a change here.

If the wish needs a subscription by default, say so to the user and offer the explicit key instead.

## 2. Change the right module

| Module | Holds |
| --- | --- |
| `src/blick/tui.py` | the app, the Topics, Nodes and Rerun tabs, the connect dialog, `Connection`, presets, the key tree (`topic_tree`), short names |
| `src/blick/tui_graph.py` | the Graph tab: layout (layers, order, rows, lanes, compaction) and the `GraphView` widget |
| `src/blick/tui_parameters.py` | the Parameters tab: node list, form, set, step, events |
| `src/blick/survey.py` | what the interface reads from the network: `Network`, `Node.label`, `uses`, `connections`, `TrafficMeter` |
| `src/blick/session.py` | `endpoint` (what a person types as an address), opening sessions |

Keep logic that needs no widget in a plain function (`topic_tree`, `short_names`, `layout`, `step_of`). It is then
testable without an app. Update a table with `_sync`, which keeps the cursor; pass `ordered=True` for a tree.

## 3. Test headless

```bash
CUDA_VISIBLE_DEVICES= uv run pytest -q tests/test_tui.py tests/test_tui_graph.py tests/test_tui_parameters.py
```

`test_tui.py` builds a fake network (a router with the admin space on, a camera, a node that subscribes and
reports). Add a test for the new behaviour there, or a plain-function test in `test_tui_graph.py`. Wait for
conditions with `until(pilot, ...)`, never with a fixed sleep. A dialog's focus, a form's fields and a discovery
arrive some frames later.

## 4. Test in a real terminal

Headless tests miss a real terminal's size, layout and focus, so run the real-terminal tests every time:

```bash
CUDA_VISIBLE_DEVICES= uv run pytest -q tests/test_tui_pty.py      # virtui: blick in a pseudo-terminal, 170 x 45 and 80 x 24
```

They need virtui (`~/.local/bin/virtui`). Extend them for a new key or layout. Run them three times before you call a
change stable, because a timing fault shows only now and then. To look at a screen yourself, start `virtui daemon start`, then
`virtui --json run --cols 80 --rows 24 blick`, and use `virtui press`, `virtui wait --text` and `virtui screenshot`
with the session id it prints.

## 5. Regenerate the screenshots

```bash
CUDA_VISIBLE_DEVICES= uv run python tools/tui_screenshots.py       # docs/media/tui/*.png from a made-up network
```

The script builds its own network of realistic nodes in the process and drives the app headless. Extend its network
or its steps when the new behaviour needs something to show. Open every PNG and look at it before you commit: a
screenshot that shows an empty pane or a cut-off column documents a fault.

## 6. Document and commit

- `docs/guides/tui.md`: what the user sees and the keys, in the style of
  [Writing documentation](../../../docs/project/writing-docs.md).
- `CHANGELOG.md`: an entry under a new patch version, and that version in `CMakeLists.txt` and `README.md`.
- The module docstrings, which describe each tab's behaviour.
- A branch of your own, pushed; main only by fast-forward after the user agrees.

## Report

```
Change: <what the user now sees and does>
Cost to the nodes: <none: queries only | an explicit key subscribes to ...>
Tests: headless <n passed>, real terminal <n passed, runs>, full suite <n passed / not run>
Screenshots: <the PNGs regenerated, what each shows>
Open: <what is left, what was not reproducible>
Branch: <name, commit>
```

## Gotchas

- **A widget's `DEFAULT_CSS` is scoped to the widget.** A rule that starts at the screen (`Screen.-narrow
  ParametersPane ...`) never matches. Set `SCOPED_CSS = False` on the widget or put the rule in the app's CSS.
- **A toast covers text.** A notification over the screen hid the text that a real-terminal test waited for. Put a
  hint that belongs to a dialog into the dialog.
- **`remove_children` is not done when it returns.** A second build of a form mounted widgets with the ids of the old
  ones (DuplicateIds). Await the removal and build under a lock.
- **Textual's `Switch` is three rows tall.** It pushed a narrow form off the screen. Use `Checkbox(..., compact=True)`.
- **A `RadioSet` without focus shows its blurred cursor on the first button,** which looked like the chosen one.
  Hide `#<id>:blur > RadioButton.-selected > .toggle--label`.
- **An event must not overwrite a field with the cursor in it.** A node's parameter events would replace what the
  user types. Tests must move the focus away before they expect an event's value.
- **The interface's own session is in the admin space.** Leave it out of the graph, the subscribers and `uses`
  (`leave_out=[own zid]`), or it shows itself as a consumer of everything it measures.
- **Peers' links to each other are not in the admin space.** The interface can say how a client's data travels, not
  a peer's.
- **`COLUMNS` and `LINES` in the environment fix Textual's size.** `tui.run` removes them, because Python's readline
  sets them for child processes.
- **A thread that calls back after the app ended raises RuntimeError.** Catch it in every `call_from_thread` of a
  worker.
- **A port 7447 router on the development machine changes the start.** Without one, blick opens the connect dialog
  by itself. Tests that depend on either case check the port first.

## When this skill was wrong

If a step failed or misled, propose a fix of this skill to the user after the task.

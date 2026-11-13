"""The terminal interface's Parameters tab (tui.py): the nodes that offer parameters (their liveliness tokens) on the
left, a form made from the chosen node's descriptors on the right (parameters.py), one line per parameter: its
name, its field and what the descriptor says (type, range and step, unit, read-only, description). A number with
its range, step and unit, a checkbox for a bool, a choice for one of its choices, text otherwise (lists as YAML
``[...]``). A node opens when it is highlighted. Enter sets an edited field, Esc takes it back, and ↑/↓ step a number
(by its range's step, else 1 for an integer or a hundredth of the range) and set it at once; a checkbox or a choice
sets at once. A field edited and not yet set is marked; under a field stands a refused change's reason or that the
change was taken. Read-only ones (applied at the next start) are shown, not changeable. The values are kept current
by the node's ParameterEvents; "save as params file" writes them as a ROS params file. It knows no node: everything
comes from the descriptors."""

from __future__ import annotations

import asyncio
import threading
from pathlib import Path
from typing import ClassVar

import zenoh
from rich.text import Text
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import Button, Checkbox, Input, Label, OptionList, Select, Static
from textual.widgets.option_list import Option

from . import parameters
from .msg import ParameterDescriptor, ParameterType, ParameterValue
from .paths import expand_path

T = ParameterType


HELP = Text("Enter sets an edited field, Esc takes it back; ↑/↓ step a number and set it; a checkbox or a "
            "choice sets at once", style="dim")
TAKEN_S = 2.0  # how long "set" stays under a field


def describe(d: ParameterDescriptor) -> Text:
    """What the descriptor says, beside the field: type, range and step, unit, read-only, description."""
    out = Text()
    parts = [T.NAMES[d.type] if 0 <= d.type < len(T.NAMES) else str(d.type)]
    rng = (d.floating_point_range or d.integer_range or [None])[0]
    if rng is not None:
        parts.append(f"{rng.from_value:g} to {rng.to_value:g}" + (f" step {rng.step:g}" if rng.step else ""))
    unit = parameters.constraints(d).get("unit")
    if unit:
        parts.append(unit)
    out.append(", ".join(parts), style="dim")
    if d.read_only:
        out.append("  read-only", style="yellow")
    if d.description:
        out.append(f"  {d.description}")
    return out


def step_of(d: ParameterDescriptor) -> float | None:
    """How far ↑/↓ move a number: its range's step, else 1 for an integer, a hundredth of a double's range; None for
    a double without a range and for anything not a number."""
    rng = (d.floating_point_range or d.integer_range or [None])[0]
    if rng is not None and rng.step:
        return rng.step
    if d.type == T.INTEGER:
        return 1
    if d.type == T.DOUBLE and rng is not None:
        return (rng.to_value - rng.from_value) / 100
    return None


class ParameterInput(Input):
    """A parameter's text field: ↑/↓ step a number, Esc takes an edit back (the pane does both)."""

    BINDINGS: ClassVar = [Binding("up", "step(1)", "Step up", show=False),
                          Binding("down", "step(-1)", "Step down", show=False),
                          Binding("escape", "revert", "Take back", show=False)]

    def action_step(self, direction: int):
        self.parent_pane().step(self, direction)

    def action_revert(self):
        self.parent_pane().revert(self)

    def parent_pane(self) -> ParametersPane:
        return next(a for a in self.ancestors if isinstance(a, ParametersPane))


class ParametersPane(Vertical):
    """The tab's content; ``show_nodes`` with the nodes found (keys ``<namespace>/<name>``) and the session."""

    # not scoped: the rules for a narrow terminal start at the Screen (the interface's breakpoints); every rule names
    # this pane or its ids
    SCOPED_CSS = False
    DEFAULT_CSS = """
    ParametersPane { height: 1fr; }
    #param-top { height: 1fr; }
    #param-nodes { width: 30%; min-width: 20; height: 1fr; }
    #param-form { width: 1fr; height: 1fr; padding: 0 1; }
    ParametersPane .param-row { height: auto; }
    ParametersPane .param-line { height: auto; }
    ParametersPane .param-name { width: 24; text-style: bold; }
    ParametersPane .param-line > Input, ParametersPane .param-line > Select { width: 26; }
    ParametersPane .param-line > Checkbox { width: 26; }
    ParametersPane .param-info { width: 1fr; padding: 0 0 0 1; }
    ParametersPane .param-info-below { display: none; height: auto; padding: 0 0 0 24; }
    /* narrower than the interface's WIDE: the descriptor's text under the field, where it has the room */
    Screen.-narrow ParametersPane .param-info { display: none; }
    Screen.-narrow ParametersPane .param-info-below { display: block; padding: 0 0 0 18; }
    Screen.-narrow ParametersPane .param-name { width: 18; }
    Screen.-narrow ParametersPane .param-line > Input, Screen.-narrow ParametersPane .param-line > Select,
    Screen.-narrow ParametersPane .param-line > Checkbox { width: 1fr; }
    Screen.-narrow ParametersPane .param-error { padding: 0 0 0 18; }
    ParametersPane .param-error { height: auto; padding: 0 0 0 24; }
    ParametersPane Input.-edited { background: $warning 25%; }
    #param-help { height: auto; }
    #param-bottom { height: auto; }
    #param-path { width: 1fr; }
    #param-status { height: auto; padding: 0 1; }
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.session: zenoh.Session | None = None
        self.nodes: list[str] = []
        self.key: str | None = None
        self.client: parameters.ParameterClient | None = None
        self.descriptors: list[ParameterDescriptor] = []
        self.values: dict[str, ParameterValue] = {}
        self._updating = False  # a value shown from an event: no change to send
        self._generation = 0
        self._open_timer = None  # a highlighted node opens after a moment (moving through the list opens none)
        self._build_lock = asyncio.Lock()

    def compose(self) -> ComposeResult:
        with Horizontal(id="param-top"):
            yield OptionList(id="param-nodes", compact=True)
            yield VerticalScroll(Static("choose a node", id="param-hint"), id="param-form", can_focus=False)
        with Horizontal(id="param-bottom"):
            yield Input(id="param-path", placeholder="params file to save to, e.g. ~/camera.yaml", compact=True)
            yield Button("Save as params file", id="param-save", compact=True)
            yield Button("Reload", id="param-reload", compact=True)
        yield Static(id="param-status")

    # ------------------------------------------------------------------------------------------------- nodes

    def show_nodes(self, nodes: list[str], session: zenoh.Session | None):
        if session is not self.session:  # connected anew: nothing of the old session is valid
            self._close_client()
            self.session = session
            self.nodes = []
        if nodes == self.nodes:
            return
        self.nodes = nodes
        widget = self.query_one("#param-nodes", OptionList)
        highlighted = widget.highlighted
        widget.clear_options()
        widget.add_options([Option(Text(key), id=key) for key in nodes])
        if nodes:
            widget.highlighted = min(highlighted or 0, len(nodes) - 1)
        if not nodes:
            self.query_one("#param-status", Static).update(Text("no node offers parameters", style="yellow"))

    def on_option_list_option_selected(self, event: OptionList.OptionSelected):
        if event.option_list.id == "param-nodes":
            self.open(event.option.id)

    def on_option_list_option_highlighted(self, event: OptionList.OptionHighlighted):
        """A highlighted node opens after 0.3 s, unless another is highlighted meanwhile."""
        if event.option_list.id != "param-nodes" or event.option.id == self.key:
            return
        if self._open_timer is not None:
            self._open_timer.stop()
        key = event.option.id
        self._open_timer = self.set_timer(0.3, lambda: self.open(key) if key != self.key else None)

    def _close_client(self):
        if self.client is not None:
            self.client.close()
        self.client, self.key = None, None

    def open(self, key: str):
        """The node's descriptors and values (in a thread), then its form."""
        if self.session is None:
            return
        self._close_client()
        self._generation += 1
        generation = self._generation
        self.key = key
        self.client = parameters.ParameterClient(self.session, key)
        client = self.client
        self._status(f"asking {key} for its parameters...")

        def fetch():
            try:
                descriptors, values = client.all()
            except (TimeoutError, RuntimeError) as error:
                self.app.call_from_thread(self._status, f"{key}: {error}", "bold red")
                return
            self.app.call_from_thread(self._build, generation, descriptors, values)
            client.on_event(lambda e: self._from_thread(self._event, generation, e))

        threading.Thread(target=fetch, name=f"parameters {key}", daemon=True).start()

    def _from_thread(self, callback, *args):
        try:
            self.app.call_from_thread(callback, *args)
        except RuntimeError:  # the app has ended
            pass

    def _status(self, text: str, style: str = ""):
        self.query_one("#param-status", Static).update(Text(text, style=style))

    # -------------------------------------------------------------------------------------------------- form

    async def _build(self, generation: int, descriptors: list[ParameterDescriptor], values: list[ParameterValue]):
        if generation != self._generation:
            return
        form = self.query_one("#param-form", VerticalScroll)
        # the old form gone before the new one (the same ids): two opens in a row (a highlight, then Enter) build twice
        async with self._build_lock:
            await form.remove_children()
            if generation != self._generation:
                return
            self.descriptors = descriptors
            self.values = {d.name: v for d, v in zip(descriptors, values)}
            await self._mount_form(form, descriptors, values)

    async def _mount_form(self, form, descriptors: list[ParameterDescriptor], values: list[ParameterValue]):
        rows = [Static(HELP, id="param-help")] if descriptors else []
        for i, (d, v) in enumerate(zip(descriptors, values)):
            error = Static(id=f"param-error-{i}", classes="param-error")
            error.display = False
            rows.append(Vertical(Horizontal(Label(d.name, classes="param-name"), self._field(i, d, v),
                                            Label(describe(d), classes="param-info"), classes="param-line"),
                                 Label(describe(d), classes="param-info-below"), error, classes="param-row"))
        await (form.mount_all(rows) if rows else form.mount(Static("no parameters")))
        self._status(f"{self.key}: {len(descriptors)} parameters"
                     + (", all read-only" if descriptors and all(d.read_only for d in descriptors) else ""))

    def _field(self, i: int, d: ParameterDescriptor, v: ParameterValue):
        widget_id = f"param-{i}"
        choices = parameters.constraints(d).get("choices")
        if d.type == T.BOOL:
            return Checkbox("on", bool(v.value()), id=widget_id, disabled=d.read_only, compact=True)
        if choices:
            options = [(parameters.text(c), parameters.text(c)) for c in choices]
            current = parameters.text(v.value())
            if current not in [o[1] for o in options]:
                options.append((current, current))
            return Select(options, value=current, allow_blank=False, id=widget_id, disabled=d.read_only,
                          compact=True)
        kind = {T.INTEGER: "integer", T.DOUBLE: "number"}.get(d.type, "text")
        return ParameterInput(self._text(v), id=widget_id, type=kind, disabled=d.read_only, compact=True,
                              select_on_focus=True)

    @staticmethod
    def _text(v: ParameterValue) -> str:
        return v.value() if v.type == T.STRING else parameters.text(v.value())

    def _note(self, i: int, text: str, style: str = ""):
        """The line under a field: a reason, that it was set, that it is edited; none when empty."""
        widget = self.query_one(f"#param-error-{i}", Static)
        widget.update(Text(text, style=style))
        widget.display = bool(text)

    def on_input_changed(self, event: Input.Changed):
        i = self._index(event.input)
        if i is None or self._updating or i >= len(self.descriptors):
            return
        event.stop()
        node = self._text(self.values[self.descriptors[i].name])
        edited = event.value != node
        event.input.set_class(edited, "-edited")
        if edited:
            self._note(i, "edited: Enter sets it, Esc takes it back", "yellow")
        elif not event.input.has_class("-sending"):
            self._note(i, "")

    def step(self, widget: Input, direction: int):
        """↑/↓ on a number: one step from the field's value, kept in the range, set at once."""
        i = self._index(widget)
        d = self.descriptors[i]
        step = step_of(d)
        if step is None:
            self._note(i, "no range to step in: type the value", "yellow")
            return
        try:
            current = float(widget.value)
        except ValueError:
            current = float(self.values[d.name].value())
        rng = (d.floating_point_range or d.integer_range or [None])[0]
        value = current + direction * step
        if rng is not None:
            value = min(max(value, rng.from_value), rng.to_value)
        text = str(int(round(value))) if d.type == T.INTEGER else f"{round(value, 12):g}"
        widget.value = text
        self._submit(i, text)

    def revert(self, widget: Input):
        i = self._index(widget)
        self._show(i, self.values[self.descriptors[i].name], edited_too=True)
        widget.remove_class("-edited")
        self._note(i, "")

    def _index(self, widget) -> int | None:
        if widget.id and widget.id.startswith("param-") and widget.id[6:].isdigit():
            return int(widget.id[6:])
        return None

    def on_input_submitted(self, event: Input.Submitted):
        i = self._index(event.input)
        if i is not None:
            event.stop()
            self._submit(i, event.value)

    def on_checkbox_changed(self, event: Checkbox.Changed):
        i = self._index(event.checkbox)
        if i is not None and not self._updating:
            self._submit(i, event.value)

    def on_select_changed(self, event: Select.Changed):
        i = self._index(event.select)
        if i is not None and not self._updating and event.value is not Select.BLANK:
            self._submit(i, event.value)

    def _submit(self, i: int, given):
        """A field's value to the node (in a thread); its reason under the field when refused."""
        d = self.descriptors[i]
        try:
            value = parameters.from_text(d, given) if isinstance(given, str) else ParameterValue.of(given)
        except (TypeError, ValueError) as problem:
            self._note(i, f"not a {T.NAMES[d.type]}: {problem}", "bold red")
            return
        client, generation = self.client, self._generation
        self._note(i, "setting...", "dim")

        def send():
            try:
                result = client.set({d.name: value})[0]
                reason = "" if result.successful else result.reason
            except (TimeoutError, RuntimeError) as problem:
                reason = str(problem)
            self._from_thread(self._result, generation, i, reason)

        threading.Thread(target=send, name=f"set {d.name}", daemon=True).start()

    def _result(self, generation: int, i: int, reason: str):
        if generation != self._generation:
            return
        widget = self.query_one(f"#param-{i}")
        widget.remove_class("-edited")
        if reason:  # the field back to the node's value
            self._note(i, reason, "bold red")
            self._show(i, self.values[self.descriptors[i].name], edited_too=True)
            return
        self._note(i, "set", "green")

        def clear(generation=generation):
            if generation == self._generation and str(self.query_one(f"#param-error-{i}", Static).content) == "set":
                self._note(i, "")

        self.set_timer(TAKEN_S, clear)

    def _event(self, generation: int, event):
        if generation != self._generation:
            return
        index = {d.name: i for i, d in enumerate(self.descriptors)}
        for p in list(event.new_parameters) + list(event.changed_parameters):
            if p.name in index:
                self.values[p.name] = p.value
                self._show(index[p.name], p.value)
            elif event.new_parameters:  # a parameter declared later: the form anew
                self.open(self.key)
                return

    def _show(self, i: int, value: ParameterValue, edited_too: bool = False):
        widget = self.query_one(f"#param-{i}")
        if isinstance(widget, Input) and widget.has_focus and not edited_too:
            return  # being edited: an event does not overwrite it (a playback's position changes often)
        self._updating = True
        try:
            if isinstance(widget, Checkbox):
                widget.value = bool(value.value())
            elif isinstance(widget, Select):
                widget.value = parameters.text(value.value())
            else:
                widget.value = value.value() if value.type == T.STRING else parameters.text(value.value())
        finally:
            self.call_after_refresh(self._done_updating)

    def _done_updating(self):
        self._updating = False

    # -------------------------------------------------------------------------------------------- the buttons

    def on_button_pressed(self, event: Button.Pressed):
        if event.button.id == "param-reload" and self.key:
            event.stop()
            self.open(self.key)
        elif event.button.id == "param-save":
            event.stop()
            self.save(self.query_one("#param-path", Input).value.strip())

    def save(self, path: str):
        """The node's values now as a ROS params file at ``path``."""
        if self.client is None:
            self._status("choose a node first", "bold red")
            return
        if not path:
            self._status("a file to save to: the field beside the button", "bold red")
            return
        client = self.client

        def write():
            try:
                names = client.list()
                values = {n: v.value() for n, v in zip(names, client.get(names))}
                target = expand_path(path)
                Path(target).write_text(parameters.dump_params_file({client.name: values}))
                self._from_thread(self._status, f"{len(values)} parameters of {client.name} saved in {target}",
                                  "green")
            except (TimeoutError, RuntimeError, OSError, ValueError) as error:
                self._from_thread(self._status, f"not saved: {error}", "bold red")

        threading.Thread(target=write, name="save parameters", daemon=True).start()

    def close(self):
        self._close_client()

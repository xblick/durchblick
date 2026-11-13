"""The terminal interface's Parameters tab (tui_parameters.py), headless, against a fake node with parameters: the
node found by its token, a form made from its descriptors, a refused change's reason under its field, a change taken,
another program's change shown from the event, read-only fields, the params file saved."""


import pytest
from test_tui import fake, run_app, text_of, until  # noqa: F401 - fake is a fixture

from blick import tui
from blick.parameters import Parameters, read_params_file

NODE = "robot/fakecam/tracker"


@pytest.fixture
def node(fake):  # noqa: F811
    params = Parameters(fake.node_session, "tracker", "robot/fakecam")
    params.declare("threshold", 0.3, description="a box's score at least", range=(0.0, 1.0, 0.05))
    params.declare("max_boxes", 10, range=(1, 100, 1), unit="boxes")
    params.declare("mode", "fast", choices=["fast", "accurate"])
    params.declare("draw", True)
    params.declare("model", "base", read_only=True)
    params.declare("labels", ["cup", "box"])
    params.check_overrides()
    yield params
    params.close()


def field(app, name):
    pane = app.query_one(tui.ParametersPane)
    i = [d.name for d in pane.descriptors].index(name)
    return i, app.query_one(f"#param-{i}")


def test_parameters_tab(fake, node, tmp_path):  # noqa: F811
    app = tui.DurchblickApp(fake.connection(), tui.Presets(tmp_path / "presets.json"), discover_every=0.5)

    async def test(pilot):
        pane = app.query_one(tui.ParametersPane)
        await until(pilot, lambda: NODE in pane.nodes)
        await pilot.press("5")
        await pilot.pause(0.3)
        assert app.focused is app.query_one("#param-nodes")
        nodes = app.query_one("#param-nodes")
        nodes.highlighted = pane.nodes.index(NODE)  # highlighted: it opens, no Enter needed
        await until(pilot, lambda: len(pane.descriptors) == 6)
        await pilot.pause(0.3)
        form = text_of(app, "#param-status")
        assert f"{NODE}: 6 parameters" in form
        # one line per parameter: name, field, what the descriptor says
        lines = {str(row.query_one(".param-name").render()): str(row.query_one(".param-info").render())
                 for row in app.query("#param-form .param-line")}
        assert lines["threshold"].startswith("double, 0 to 1 step 0.05  a box's score")
        assert lines["max_boxes"].startswith("integer, 1 to 100 step 1, boxes")
        assert lines["model"].startswith("string  read-only")
        _, model = field(app, "model")
        assert model.disabled

        # a value outside the range: the node's reason under the field, the field back to the node's value
        i, threshold = field(app, "threshold")
        threshold.value = "1.5"
        threshold.focus()
        await pilot.press("enter")
        await until(pilot, lambda: "outside" in text_of(app, f"#param-error-{i}"))
        assert text_of(app, f"#param-error-{i}").strip() == "1.5 is outside 0 to 1"
        await until(pilot, lambda: threshold.value == "0.3")
        # edited and not yet set: marked; Esc takes it back
        threshold.value = "0.7"
        await pilot.pause(0.2)
        assert threshold.has_class("-edited") and "Enter sets it" in text_of(app, f"#param-error-{i}")
        await pilot.press("escape")
        await until(pilot, lambda: threshold.value == "0.3" and not threshold.has_class("-edited"))
        assert node["threshold"] == 0.3
        # taken: "set" under the field for a moment
        threshold.value = "0.45"
        await pilot.press("enter")
        await until(pilot, lambda: node["threshold"] == 0.45)
        await until(pilot, lambda: text_of(app, f"#param-error-{i}").strip() == "set")
        assert not threshold.has_class("-edited")
        await until(pilot, lambda: not app.query_one(f"#param-error-{i}").display, timeout=5)
        # ↑/↓ step by the range's step and set at once, kept in the range
        await pilot.press("up")
        await until(pilot, lambda: node["threshold"] == pytest.approx(0.5))
        await pilot.press("down", "down")
        await until(pilot, lambda: node["threshold"] == pytest.approx(0.4))
        j, boxes_field = field(app, "max_boxes")
        boxes_field.focus()
        boxes_field.value = "100"
        await pilot.press("enter")
        await until(pilot, lambda: node["max_boxes"] == 100)
        await pilot.press("up")  # at the range's end: stays
        await pilot.pause(0.5)
        assert node["max_boxes"] == 100 and boxes_field.value == "100"
        # a choice, a checkbox
        _, mode = field(app, "mode")
        mode.value = "accurate"
        await until(pilot, lambda: node["mode"] == "accurate")
        _, draw = field(app, "draw")
        draw.value = False
        await until(pilot, lambda: node["draw"] is False)
        # another program's change: shown from the node's event (in a field without the cursor: one being edited
        # keeps what is typed)
        app.query_one("#param-nodes").focus()
        node.set("max_boxes", 42)
        _, boxes = field(app, "max_boxes")
        await until(pilot, lambda: boxes.value == "42")
        # saved as a params file
        app.query_one("#param-path").value = str(tmp_path / "tracker.yaml")
        app.query_one("#param-save").press()
        await until(pilot, lambda: "saved in" in text_of(app, "#param-status"))
        assert read_params_file(tmp_path / "tracker.yaml", "/" + NODE) == {
            "draw": False, "labels": ["cup", "box"], "max_boxes": 42, "mode": "accurate", "model": "base",
            "threshold": 0.4}

    run_app(app, test, size=(170, 45))


@pytest.mark.parametrize("size", [(80, 24), (250, 70)])
def test_parameters_tab_fits(fake, node, tmp_path, size):  # noqa: F811
    app = tui.DurchblickApp(fake.connection(), tui.Presets(tmp_path / "presets.json"), discover_every=0.5)

    async def test(pilot):
        pane = app.query_one(tui.ParametersPane)
        await until(pilot, lambda: NODE in pane.nodes)
        await pilot.press("5")
        pane.open(NODE)
        await until(pilot, lambda: len(pane.descriptors) == 6)
        await pilot.pause(0.5)
        width, height = size
        for widget_id in ("#param-nodes", "#param-form", "#param-save", "#param-status"):
            r = app.query_one(widget_id).region
            assert r.width > 0 and r.height > 0 and r.right <= width and r.bottom <= height, (widget_id, r)
        assert app.screen.virtual_size.height <= height

    run_app(app, test, size=size)

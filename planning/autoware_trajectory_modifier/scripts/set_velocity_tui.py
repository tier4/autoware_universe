#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from threading import Thread

from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_internal_planning_msgs.msg import VelocityLimitClearCommand
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from textual import on
from textual.app import App
from textual.app import ComposeResult
from textual.containers import Horizontal
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import Button
from textual.widgets import Footer
from textual.widgets import Header
from textual.widgets import Input
from textual.widgets import Static

TOPIC = "/planning/scenario_planning/max_velocity_default"
CLEAR_TOPIC = "/planning/scenario_planning/clear_velocity_limit"


@dataclass(frozen=True)
class LimitConfig:
    max_velocity: float
    min_acceleration: float
    max_acceleration: float
    min_jerk: float
    max_jerk: float


class VelocityLimitPublisher(Node):
    """Long-lived ROS 2 node used by the TUI."""

    def __init__(self) -> None:
        super().__init__("velocity_limit_tui")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            VelocityLimit,
            TOPIC,
            qos,
        )

        self.clear_publisher = self.create_publisher(
            VelocityLimitClearCommand,
            CLEAR_TOPIC,
            qos,
        )

    def subscription_count(self) -> int:
        return self.publisher.get_subscription_count()

    def publish_limit(self, config: LimitConfig) -> int:
        msg = VelocityLimit()

        # Current autoware_internal_planning_msgs
        if hasattr(msg, "stamp"):
            msg.stamp = self.get_clock().now().to_msg()

        msg.max_velocity = float(config.max_velocity)
        msg.use_constraints = True

        msg.constraints.min_acceleration = float(config.min_acceleration)
        msg.constraints.max_acceleration = float(config.max_acceleration)
        msg.constraints.min_jerk = float(config.min_jerk)
        msg.constraints.max_jerk = float(config.max_jerk)

        # Keep compatibility with older message revisions.
        if hasattr(msg, "sender"):
            msg.sender = "velocity_limit_tui"

        self.publisher.publish(msg)

        return self.subscription_count()

    def publish_clear(self) -> None:
        msg = VelocityLimitClearCommand()
        msg.command = True
        msg.sender = "velocity_limit_tui"
        self.clear_publisher.publish(msg)


class ConfirmPublish(ModalScreen[bool]):
    """Second-click confirmation before actually publishing."""

    CSS = """
    ConfirmPublish {
        align: center middle;
    }

    #dialog {
        width: 68;
        height: auto;
        border: thick $accent;
        background: $surface;
        padding: 1 2;
    }

    #summary {
        margin-bottom: 1;
    }

    #confirm-buttons {
        height: auto;
        align: center middle;
    }

    #confirm-buttons Button {
        margin: 0 1;
    }
    """

    BINDINGS = [
        ("escape", "cancel", "Cancel"),
    ]

    def __init__(self, config: LimitConfig) -> None:
        super().__init__()
        self.config = config

    def compose(self) -> ComposeResult:
        summary = (
            "[b]Publish VelocityLimit?[/b]\n\n"
            f"max_velocity       = {self.config.max_velocity:.3f} m/s\n"
            f"min_acceleration   = {self.config.min_acceleration:.3f} m/s²\n"
            f"max_acceleration   = {self.config.max_acceleration:.3f} m/s²\n"
            f"min_jerk           = {self.config.min_jerk:.3f} m/s³\n"
            f"max_jerk           = {self.config.max_jerk:.3f} m/s³\n\n"
            f"{TOPIC}"
        )

        with Vertical(id="dialog"):
            yield Static(summary, id="summary")

            with Horizontal(id="confirm-buttons"):
                yield Button(
                    "CONFIRM PUBLISH",
                    id="confirm",
                    variant="success",
                )
                yield Button(
                    "CANCEL",
                    id="cancel",
                    variant="error",
                )

    @on(Button.Pressed, "#confirm")
    def confirm(self) -> None:
        self.dismiss(True)

    @on(Button.Pressed, "#cancel")
    def cancel(self) -> None:
        self.dismiss(False)

    def action_cancel(self) -> None:
        self.dismiss(False)


class VelocityLimitApp(App):
    TITLE = "Autoware Velocity Limit"

    CSS = """
    Screen {
        layout: vertical;
    }

    #body {
        width: 90;
        height: auto;
        margin: 1 2;
        padding: 1 2;
        border: round $accent;
    }

    .parameter-label {
        margin-top: 1;
    }

    .control-row {
        height: 3;
    }

    .control-row Button {
        min-width: 9;
        margin-right: 1;
    }

    .control-row Input {
        width: 18;
        margin-right: 1;
    }

    #publish-row {
        height: 3;
        margin-top: 2;
    }

    #publish-row Button {
        width: 22;
        margin-right: 2;
    }

    #ros-status {
        margin-top: 1;
    }

    #last-publish {
        margin-top: 1;
        padding: 1;
        border: round $primary;
    }

    #warning {
        margin-top: 1;
        color: $warning;
    }
    """

    BINDINGS = [
        ("p", "request_publish", "Publish"),
        ("c", "clear_limit", "Clear Limit"),
        ("q", "quit", "Quit"),
    ]

    def __init__(self, ros_node: VelocityLimitPublisher) -> None:
        super().__init__()
        self.ros_node = ros_node

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        with Vertical(id="body"):
            yield Static(
                f"[b]Topic[/b]: {TOPIC}",
            )

            yield Static(
                "Max velocity [m/s]",
                classes="parameter-label",
            )
            with Horizontal(classes="control-row"):
                yield Button("-1.0", id="velocity-minus-1")
                yield Button("-0.1", id="velocity-minus-01")
                yield Input(value="5.0", id="max-velocity")
                yield Button("+0.1", id="velocity-plus-01")
                yield Button("+1.0", id="velocity-plus-1")

            yield Static(
                "Min acceleration [m/s²]  (negative value = deceleration limit)",
                classes="parameter-label",
            )
            with Horizontal(classes="control-row"):
                yield Button("-0.5", id="min-acc-minus-05")
                yield Button("-0.1", id="min-acc-minus-01")
                yield Input(value="-1.0", id="min-acceleration")
                yield Button("+0.1", id="min-acc-plus-01")
                yield Button("+0.5", id="min-acc-plus-05")

            yield Static(
                "Max acceleration [m/s²]",
                classes="parameter-label",
            )
            with Horizontal(classes="control-row"):
                yield Button("-0.5", id="max-acc-minus-05")
                yield Button("-0.1", id="max-acc-minus-01")
                yield Input(value="0.0", id="max-acceleration")
                yield Button("+0.1", id="max-acc-plus-01")
                yield Button("+0.5", id="max-acc-plus-05")

            yield Static(
                "Min jerk [m/s³]",
                classes="parameter-label",
            )
            with Horizontal(classes="control-row"):
                yield Button("-0.5", id="min-jerk-minus-05")
                yield Button("-0.1", id="min-jerk-minus-01")
                yield Input(value="-0.8", id="min-jerk")
                yield Button("+0.1", id="min-jerk-plus-01")
                yield Button("+0.5", id="min-jerk-plus-05")

            yield Static(
                "Max jerk [m/s³]",
                classes="parameter-label",
            )
            with Horizontal(classes="control-row"):
                yield Button("-0.5", id="max-jerk-minus-05")
                yield Button("-0.1", id="max-jerk-minus-01")
                yield Input(value="0.0", id="max-jerk")
                yield Button("+0.1", id="max-jerk-plus-01")
                yield Button("+0.5", id="max-jerk-plus-05")

            with Horizontal(id="publish-row"):
                yield Button(
                    "PUBLISH",
                    id="publish",
                    variant="success",
                )

                yield Button(
                    "CLEAR LIMIT",
                    id="clear_limit",
                    variant="warning",
                )

                yield Button(
                    "QUIT",
                    id="quit",
                    variant="error",
                )

            yield Static(
                "ROS matched subscribers: checking...",
                id="ros-status",
            )

            yield Static(
                "Last publish: never",
                id="last-publish",
            )

            yield Static(
                "P = publish   C = clear limit   Q = quit   All controls can also be used with the mouse.",
                id="warning",
            )

        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(
            0.5,
            self.update_ros_status,
        )

    def update_ros_status(self) -> None:
        try:
            count = self.ros_node.subscription_count()
            status = self.query_one("#ros-status", Static)

            if count > 0:
                status.update(f"[green]ROS matched subscribers: {count}[/green]")
            else:
                status.update("[yellow]ROS matched subscribers: 0[/yellow]")

        except Exception as exc:
            self.query_one("#ros-status", Static).update(f"[red]ROS status error: {exc}[/red]")

    @staticmethod
    def _parse_float(input_widget: Input, name: str) -> float:
        try:
            return float(input_widget.value)
        except ValueError as exc:
            raise ValueError(f"{name} is not a valid number: " f"{input_widget.value!r}") from exc

    def read_config(self) -> LimitConfig:
        max_velocity = self._parse_float(
            self.query_one("#max-velocity", Input),
            "max_velocity",
        )

        min_acceleration = self._parse_float(
            self.query_one("#min-acceleration", Input),
            "min_acceleration",
        )

        max_acceleration = self._parse_float(
            self.query_one("#max-acceleration", Input),
            "max_acceleration",
        )

        min_jerk = self._parse_float(
            self.query_one("#min-jerk", Input),
            "min_jerk",
        )

        max_jerk = self._parse_float(
            self.query_one("#max-jerk", Input),
            "max_jerk",
        )

        if max_velocity < 0.0:
            raise ValueError("max_velocity must be >= 0")

        return LimitConfig(
            max_velocity=max_velocity,
            min_acceleration=min_acceleration,
            max_acceleration=max_acceleration,
            min_jerk=min_jerk,
            max_jerk=max_jerk,
        )

    def bump(self, widget_id: str, delta: float) -> None:
        widget = self.query_one(
            f"#{widget_id}",
            Input,
        )

        try:
            current = float(widget.value)
        except ValueError:
            current = 0.0

        widget.value = f"{current + delta:.2f}"

    @on(Button.Pressed)
    def button_pressed(
        self,
        event: Button.Pressed,
    ) -> None:
        button_id = event.button.id

        match button_id:
            case "velocity-minus-1":
                self.bump("max-velocity", -1.0)
            case "velocity-minus-01":
                self.bump("max-velocity", -0.1)
            case "velocity-plus-01":
                self.bump("max-velocity", +0.1)
            case "velocity-plus-1":
                self.bump("max-velocity", +1.0)

            case "min-acc-minus-05":
                self.bump("min-acceleration", -0.5)
            case "min-acc-minus-01":
                self.bump("min-acceleration", -0.1)
            case "min-acc-plus-01":
                self.bump("min-acceleration", +0.1)
            case "min-acc-plus-05":
                self.bump("min-acceleration", +0.5)

            case "max-acc-minus-05":
                self.bump("max-acceleration", -0.5)
            case "max-acc-minus-01":
                self.bump("max-acceleration", -0.1)
            case "max-acc-plus-01":
                self.bump("max-acceleration", +0.1)
            case "max-acc-plus-05":
                self.bump("max-acceleration", +0.5)

            case "min-jerk-minus-05":
                self.bump("min-jerk", -0.5)
            case "min-jerk-minus-01":
                self.bump("min-jerk", -0.1)
            case "min-jerk-plus-01":
                self.bump("min-jerk", +0.1)
            case "min-jerk-plus-05":
                self.bump("min-jerk", +0.5)

            case "max-jerk-minus-05":
                self.bump("max-jerk", -0.5)
            case "max-jerk-minus-01":
                self.bump("max-jerk", -0.1)
            case "max-jerk-plus-01":
                self.bump("max-jerk", +0.1)
            case "max-jerk-plus-05":
                self.bump("max-jerk", +0.5)

            case "publish":
                self.action_request_publish()

            case "clear_limit":
                self.action_clear_limit()

            case "quit":
                self.exit()

    def action_request_publish(self) -> None:
        try:
            config = self.read_config()
        except ValueError as exc:
            self.query_one("#last-publish", Static).update(
                f"[red]Invalid configuration: {exc}[/red]"
            )
            return

        self.push_screen(
            ConfirmPublish(config),
            self.after_confirmation,
        )

    def after_confirmation(self, confirmed: bool) -> None:
        if not confirmed:
            return

        try:
            config = self.read_config()
            subscriber_count = self.ros_node.publish_limit(config)
            now = datetime.now().strftime("%H:%M:%S")

            if subscriber_count > 0:
                state = "[green]PUBLISHED[/green]"
            else:
                state = "[yellow]PUBLISHED (no matched subscriber)[/yellow]"

            self.query_one("#last-publish", Static).update(
                f"{state}  {now}\n"
                f"velocity={config.max_velocity:.3f} m/s   "
                f"min_acc={config.min_acceleration:.3f} m/s²   "
                f"min_jerk={config.min_jerk:.3f} m/s³   "
                f"matched_subscribers={subscriber_count}"
            )

        except Exception as exc:
            self.query_one("#last-publish", Static).update(f"[red]Publish failed: {exc}[/red]")

    def action_clear_limit(self) -> None:
        try:
            self.ros_node.publish_clear()
            now = datetime.now().strftime("%H:%M:%S")
            self.query_one("#last-publish", Static).update(
                f"[bold cyan]LIMIT CLEARED[/bold cyan]  {now}\n"
                f"Published VelocityLimitClearCommand to:\n"
                f"{CLEAR_TOPIC}"
            )
        except Exception as exc:
            self.query_one("#last-publish", Static).update(f"[red]Clear limit failed: {exc}[/red]")


def main() -> None:
    rclpy.init()

    ros_node = VelocityLimitPublisher()

    # Keeping an executor running means /clock, ROS graph events,
    # parameters, etc. continue to function normally while Textual
    # owns the main thread.
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    ros_thread = Thread(
        target=executor.spin,
        daemon=True,
    )
    ros_thread.start()

    try:
        VelocityLimitApp(ros_node).run()
    finally:
        executor.shutdown()
        ros_thread.join(timeout=1.0)
        ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

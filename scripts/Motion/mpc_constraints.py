"""Which of robot-core's MPC constraints each solve presses against.

Each control step the MPC picks a voltage plan U (left and right for each of
its V control stages) that minimises its cost subject to G*U <= b. Rows of
G*U <= b, in robot-core's assembleConstraints order (F is the prediction
horizon):

    slew        2 * 2V rows   each step's voltage change, down then up,
                              within MPC_DELTA_U_MIN / MPC_DELTA_U_MAX
    voltage     2 * 2V rows   |u| <= MPC_V_MAX, low then high
    battery     2 * 2V rows   |u| <= the battery voltage, low then high
    field       2 * 2F rows   predicted |x|, |y| <= the field limits
    wheel_speed 2 * 2F rows   predicted |omega| <= MPC_OMEGA_WHEEL_MAX

Input rows go stage by stage, left then right. Output rows go predicted
step by step (1 to F), x then y or left then right. A row is active, the
solution pressing against it, when its slack b - G*U is within
ACTIVE_TOLERANCE.

ConstraintLog writes one CSV line per control step (config.
MPC_CONSTRAINT_LOG) and, optionally, every row of every step
(config.MPC_CONSTRAINT_ROWS). No Isaac Sim imports.
"""

import csv
import math

import drivetrain_config as config


# Slack (V, in or rad/s) below which a row counts as active. OSQP solves to
# about 1e-3, so this is well clear of solver noise.
ACTIVE_TOLERANCE = 0.05

# The limits summarised per step: (name, rows' group, bound). Battery rows
# fold into voltage, since with a full battery they're the same limit.
LIMITS = (
    ("slew_down", ("slew",), "low"),
    ("slew_up", ("slew",), "high"),
    ("voltage_low", ("voltage", "battery"), "low"),
    ("voltage_high", ("voltage", "battery"), "high"),
    ("field", ("field",), None),
    ("wheel_speed", ("wheel_speed",), None),
)
# Short labels for the first move's limits.
LABELS = {
    "slew_down": "slew-",
    "slew_up": "slew+",
    "voltage_low": "V-",
    "voltage_high": "V+",
    "field": "field",
    "wheel_speed": "speed",
}


def row_layout(control, prediction):
    """Describe every row as (group, bound, stage, item).

    stage is the control stage (0 is the move about to be sent) for input
    rows, and the predicted step (1 to prediction) for output rows.
    """
    rows = []
    for group in ("slew", "voltage", "battery"):
        for bound in ("low", "high"):
            for i in range(2 * control):
                rows.append((group, bound, i // 2, ("left", "right")[i % 2]))
    for group, items in (("field", ("x", "y")),
                         ("wheel_speed", ("left", "right"))):
        for bound in ("low", "high"):
            for j in range(2 * prediction):
                rows.append((group, bound, j // 2 + 1, items[j % 2]))
    return rows


def _limit_names(group, bound):
    return [
        name for name, groups, limit_bound in LIMITS
        if group in groups and limit_bound in (None, bound)
    ]


class PathTally:
    """How often a path's first moves were held at each limit."""

    def __init__(self):
        self.steps = 0
        self.limited = {name: 0 for name, _, _ in LIMITS}

    def summary(self):
        if not self.steps:
            return "  MPC limits: no solves"
        parts = [
            f"{LABELS[name]} {count / self.steps:.0%}"
            for name, count in self.limited.items()
        ]
        return (
            f"  MPC first move held at a limit ({self.steps} solves): "
            + ", ".join(parts)
        )


class ConstraintLog:
    """Records the active constraints of each MPC solve to CSV files."""

    def __init__(self, bridge, summary_file, rows_file=None):
        self.bridge = bridge
        self.layout = row_layout(*bridge.horizons)
        self.tally = PathTally()
        self._summary_handle = open(summary_file, "w", newline="")
        self._summary = csv.writer(self._summary_handle)
        self._summary.writerow(
            ["time_s", "path", "x_in", "y_in", "heading_deg", "v_left",
             "v_right", "off_path_in", "first_left", "first_right"]
            + [name for name, _, _ in LIMITS]
            + ["largest_dual_row", "largest_dual"]
        )
        self._rows_handle = None
        self._rows = None
        if rows_file is not None:
            self._rows_handle = open(rows_file, "w", newline="")
            self._rows = csv.writer(self._rows_handle)
            self._rows.writerow(
                ["time_s", "path", "group", "bound", "stage", "item", "lhs",
                 "rhs", "slack", "dual", "active"]
            )

    def start_path(self):
        """Begin a new path's tally."""
        self.tally = PathTally()

    def record(self, time, path, x, y, heading, off_path):
        """Log the bridge's last solve. Call right after a solved step."""
        solve = self.bridge.constraints()
        if solve is None:
            return
        lhs, rhs, dual, inputs = solve

        counts = {name: 0 for name, _, _ in LIMITS}
        first = {"left": [], "right": []}
        largest_dual = (0.0, None)
        for index, (group, bound, stage, item) in enumerate(self.layout):
            slack = rhs[index] - lhs[index]
            active = slack <= ACTIVE_TOLERANCE
            if dual[index] > largest_dual[0]:
                largest_dual = (dual[index], index)
            if self._rows is not None:
                self._rows.writerow(
                    [f"{time:.2f}", path, group, bound, stage, item,
                     f"{lhs[index]:.4f}", f"{rhs[index]:.4f}",
                     f"{slack:.4f}", f"{dual[index]:.4g}", int(active)]
                )
            if not active:
                continue
            for name in _limit_names(group, bound):
                counts[name] += 1
                # The first move: control stage 0, or the first predicted
                # step for the output limits.
                first_stage = 1 if group in ("field", "wheel_speed") else 0
                if stage == first_stage and item in first:
                    label = LABELS[name]
                    if label not in first[item]:
                        first[item].append(label)

        self.tally.steps += 1
        for name in counts:
            held = any(
                LABELS[name] in labels for labels in first.values()
            )
            self.tally.limited[name] += held

        dual_row = ""
        if largest_dual[1] is not None:
            group, bound, stage, item = self.layout[largest_dual[1]]
            dual_row = f"{group} {bound} stage {stage} {item}"
        self._summary.writerow(
            [f"{time:.2f}", path, f"{x:.2f}", f"{y:.2f}",
             f"{math.degrees(heading):.1f}", f"{inputs[0]:.2f}",
             f"{inputs[1]:.2f}", f"{off_path:.2f}",
             " ".join(first["left"]), " ".join(first["right"])]
            + [counts[name] for name, _, _ in LIMITS]
            + [dual_row, f"{largest_dual[0]:.4g}"]
        )

    def close(self):
        for handle in (self._summary_handle, self._rows_handle):
            if handle is not None and not handle.closed:
                handle.close()


def open_log(bridge):
    """A ConstraintLog for bridge as the config asks, or None if off."""
    if not config.MPC_CONSTRAINT_LOG:
        return None
    rows_file = (
        config.MPC_CONSTRAINT_ROWS_FILE if config.MPC_CONSTRAINT_ROWS
        else None
    )
    return ConstraintLog(bridge, config.MPC_CONSTRAINT_FILE, rows_file)

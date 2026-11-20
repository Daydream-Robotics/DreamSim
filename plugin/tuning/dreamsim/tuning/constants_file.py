"""Parses robot-core's constants.h into categories for the tuning UI.

constants.h groups its constants under category banners:

    /* - - - - - - - - - - [MPC] - - - - - - - - - - */
    // used by the Nano's MPC ...              <- category description

    // cost weights                             <- group label
    constexpr double MPC_P_X = 300.0; // output weight for x

A comment line right after a banner (followed by a blank line) describes
the category; a comment line directly above a run of constants labels that
group. Each constant keeps its line number and raw value text so it can be
written back later.

No Isaac Sim imports, so it can be tested with plain Python.
"""

import ast
import operator
import re
from dataclasses import dataclass, field
from pathlib import Path

BANNER = re.compile(r"^\s*/\*[\s\-]*\[([^\]]+)\][\s\-]*\*/\s*$")
CONSTANT = re.compile(
    r"^\s*(?:inline\s+)?(?:static\s+)?constexpr\s+(?P<type>.+?)\s+"
    r"(?P<name>[A-Za-z_]\w*)\s*=\s*(?P<rest>.*)$"
)
COMMENT = re.compile(r"^\s*//\s?(.*)$")

# Category for constants above the first banner.
UNCATEGORIZED = "UNCATEGORIZED"


@dataclass
class Constant:
    name: str
    ctype: str
    raw: str  # value text as written, e.g. "450.0/600.0"
    value: object  # evaluated number, bool or string; None if not simple
    comment: str
    line: int  # 1-based line number in the file
    category: str
    group: str

    @property
    def is_number(self):
        return isinstance(self.value, (int, float)) and not isinstance(
            self.value, bool
        )


@dataclass
class Group:
    name: str  # "" for constants with no label above them
    constants: list = field(default_factory=list)


@dataclass
class Category:
    name: str
    description: str = ""
    groups: list = field(default_factory=list)

    @property
    def constants(self):
        return [c for group in self.groups for c in group.constants]


def parse(path):
    """Parse constants.h at path into a list of Categories, in file order."""
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    categories = [Category(UNCATEGORIZED)]
    pending = []  # comment lines since the last blank or constant
    after_banner = False  # no constants or blank-separated text yet
    previous_was_constant = False

    for number, line in enumerate(lines, start=1):
        banner = BANNER.match(line)
        if banner:
            categories.append(Category(banner.group(1).strip()))
            pending, after_banner = [], True
            previous_was_constant = False
            continue

        if not line.strip():
            category = categories[-1]
            if after_banner and pending and not category.description:
                category.description = " ".join(pending)
            pending = []
            previous_was_constant = False
            continue

        comment = COMMENT.match(line)
        if comment:
            pending.append(comment.group(1).strip())
            previous_was_constant = False
            continue

        constant = CONSTANT.match(line)
        if not constant:
            pending = []
            previous_was_constant = False
            continue

        category = categories[-1]
        if pending or not previous_was_constant or not category.groups:
            category.groups.append(Group(" ".join(pending)))
        pending, after_banner = [], False
        previous_was_constant = True

        raw, trailing = _split_value(constant.group("rest"))
        category.groups[-1].constants.append(
            Constant(
                name=constant.group("name"),
                ctype=constant.group("type").strip(),
                raw=raw,
                value=_evaluate(raw),
                comment=trailing,
                line=number,
                category=category.name,
                group=category.groups[-1].name,
            )
        )

    return [c for c in categories if c.groups or c.description]


def _split_value(rest):
    """Split 'VALUE; // comment' into (VALUE, comment), honouring quotes."""
    quote = None
    for index, char in enumerate(rest):
        if quote:
            if char == quote and rest[index - 1] != "\\":
                quote = None
        elif char in "\"'":
            quote = char
        elif char == ";":
            trailing = rest[index + 1:].strip()
            comment = COMMENT.match(trailing)
            return rest[:index].strip(), comment.group(1).strip() if (
                comment
            ) else trailing
    return rest.strip(), ""


_OPERATORS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.USub: operator.neg,
    ast.UAdd: operator.pos,
}


def _evaluate(raw):
    """Evaluate a simple C++ literal or arithmetic expression, else None."""
    text = raw.strip()
    if text in ("true", "false"):
        return text == "true"
    if len(text) >= 2 and text[0] == text[-1] == '"':
        return text[1:-1]
    # C++ float suffixes, e.g. 1.0f
    text = re.sub(r"(?<=[\d.])[fF]\b", "", text)
    try:
        return _eval_node(ast.parse(text, mode="eval").body)
    except (SyntaxError, ValueError, TypeError, ZeroDivisionError):
        return None


def _eval_node(node):
    if isinstance(node, ast.Constant) and isinstance(
        node.value, (int, float)
    ):
        return node.value
    if isinstance(node, ast.BinOp) and type(node.op) in _OPERATORS:
        return _OPERATORS[type(node.op)](
            _eval_node(node.left), _eval_node(node.right)
        )
    if isinstance(node, ast.UnaryOp) and type(node.op) in _OPERATORS:
        return _OPERATORS[type(node.op)](_eval_node(node.operand))
    raise ValueError("not a simple expression")

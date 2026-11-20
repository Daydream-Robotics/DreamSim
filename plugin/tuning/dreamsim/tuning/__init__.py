"""DreamSim MPC tuning suite: an Isaac Sim extension for tuning robot-core's
MPC constants (shared/include/config/constants.h) against the sim.

Kit loads TuningExtension from here when the extension is enabled.
"""

from .extension import TuningExtension  # noqa: F401

"""After the search: inspect the condition of the object it found.

`views` keeps the best looks at the target the search collected on its way in;
`condition` asks a vision-language model what state the object is in from
them; `card` draws the verdict. Configured by the `inspection` group
(`core/config/inspection.py`), off unless a preset turns it on, and run by
`scripts/run_robot.py` once the episode has ended at the target.
"""
from .card import render_card
from .condition import ConditionInspector, InspectionReport, build_prompt, parse_verdict
from .views import TargetView, TargetViewBuffer, bearing_deg

__all__ = ["ConditionInspector", "InspectionReport", "TargetView", "TargetViewBuffer",
           "bearing_deg", "build_prompt", "parse_verdict", "render_card"]

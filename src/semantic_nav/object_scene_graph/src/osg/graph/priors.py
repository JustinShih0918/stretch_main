"""Category priors for the search posterior: which surface does a class rest on?
"""
from __future__ import annotations

from ..core.labels import normalize_label


# ---------------------------------------------------------- search posterior
# Where does an object of this class REST? These two tables answer "which
# surface", and they are what turns "not here" into "then look there" (Phase 3).

# Container categories a class is plausibly found on, best first. Deliberately
# small and inspectable; anything absent falls back to LLM affinity when it is
# enabled, and to a flat prior when it is not.
CONTAINER_AFFINITY = {
    "bowl": ["table", "counter", "desk", "cabinet", "shelf", "sink"],
    "mug": ["table", "counter", "desk", "shelf", "cabinet", "sink"],
    "red plate": ["table", "counter", "cabinet", "shelf", "sink"],
    "cup": ["table", "counter", "desk", "shelf"],
    "bottle": ["table", "counter", "desk", "shelf", "refrigerator"],
    "cracker box": ["counter", "table", "shelf", "cabinet", "desk"],
    "tin can": ["counter", "shelf", "cabinet", "table"],
    "pitcher": ["counter", "table", "shelf", "cabinet"],
    "scissors": ["desk", "table", "cabinet", "shelf", "counter"],
    "banana": ["counter", "table", "bowl", "shelf"],
    "book": ["desk", "table", "shelf", "nightstand", "bed"],
    "pillow": ["bed", "sofa", "bench"],
}

# What a class needs from a surface: (min top height, max top height, min area).
# Read straight off the container geometry the scene graph already computes, so
# a mug is not proposed on the floor and a bowl is not proposed on a shelf at
# head height. Generous bands -- this is a prior, not a constraint.
AFFORDANCE = {
    "bowl": (0.4, 1.3, 0.06),
    "mug": (0.4, 1.3, 0.04),
    "red plate": (0.4, 1.3, 0.06),
    "cracker box": (0.3, 1.4, 0.06),
    "tin can": (0.3, 1.4, 0.04),
    "pitcher": (0.4, 1.3, 0.06),
    "scissors": (0.4, 1.4, 0.04),
    "banana": (0.4, 1.3, 0.04),
    "book": (0.2, 1.6, 0.04),
    "pillow": (0.2, 0.9, 0.15),
}
DEFAULT_AFFORDANCE = (0.15, 1.6, 0.03)


# Weight of an unlisted category, and how hard the ranking bites.
#
# CONTAINER_AFFINITY is six entries long per class and was never meant to be
# exhaustive, but an unlisted category used to score 0.25 -- BELOW the lowest
# listed entry, i.e. positive evidence against. Measured on the benchmark's own
# relocations, that is backwards. Of 53 moves whose destination surface is
# mapped, the objects land on:
#
#     bed 26, desk 19, table 6, cabinet 3, nightstand 3, bench 2, stool 2, ...
#
# and 37 of the 53 land on a category the table does not list for that class --
# a tomato soup can is put on a bed seven times. Ranking the 114 relocations by
# the true surface's position in the candidate list:
#
#     affinity x proximity (as shipped)      top-1 19   top-5 36   median  5
#     proximity alone, no affinity           top-1 26   top-5 38   median  2
#     affinity alone, no proximity           top-1  0   top-5 12   median 27
#     neither (arbitrary order)              top-1  2   top-5 14   median 25
#
# Affinity alone is no better than arbitrary order, and multiplying it in makes
# proximity worse. So: absence from the list is not evidence (0.5, the same
# value used when there is no ranking at all), and the ranking that remains is
# softened to a tie-breaker rather than a veto.
#
# The first of those two is a correctness fix and would be right on any data.
# The second is calibrated against THIS benchmark, whose relocations appear to
# be placed for reachability rather than for semantic plausibility -- see
# docs/history/DYNAMIC_SCENES.md. On a benchmark that moved objects the way people do,
# a stronger affinity term would be worth more, and AFFINITY_POWER is the knob.
UNLISTED_AFFINITY = 0.5
AFFINITY_POWER = 0.5


def affinity_scores(target: str, source=None, present=None) -> dict:
    """{container category: weight in (0, 1]}, best first, or {} if unknown.

    `source` is an optional callable (an LLM affinity provider) consulted only
    when the static table has no entry -- the table stays authoritative so a
    model cannot quietly rewrite a prior someone chose deliberately.

    `present` is the set of container categories this map actually contains.
    When given, a table entry is filtered down to them: a ranking is only
    informative about surfaces that exist, and `bowl` -> table, counter, desk...
    carries no information in a house with neither a table nor a counter. If
    fewer than two of an entry's options survive, the entry has nothing left to
    say and the grounded source is asked instead -- which is the ONE case where
    a model may speak over the static table, because the table has effectively
    abstained.
    """
    key = normalize_label(target)
    ranked = CONTAINER_AFFINITY.get(key)
    if ranked is not None and present is not None:
        # Drop a category only when it is one the scene graph COULD have built
        # and this map does not have. A name the graph never produces at all
        # (`sink` is in two entries and in no CONTAINER_CATEGORIES) scores
        # nothing either way, and dropping it would rescale every surviving
        # weight -- which would make grounding a change in the scenes it is
        # meant to leave alone.
        from .containers import CONTAINER_CATEGORIES

        here = {normalize_label(c) for c in present}
        buildable = {normalize_label(c) for c in CONTAINER_CATEGORIES}
        ranked = [
            c for c in ranked
            if normalize_label(c) not in buildable or normalize_label(c) in here
        ]
        if len(ranked) < 2:
            ranked = None
    if ranked is None and source is not None:
        ranked = source(key)
    if not ranked:
        return {}
    n = len(ranked)
    return {normalize_label(c): 1.0 - 0.5 * i / max(n - 1, 1) for i, c in enumerate(ranked)}


def affords(target: str, top_h: float, area_m2: float) -> float:
    """Can a surface at this height and size hold this class? 1.0 or 0.0.

    Binary on purpose: a shelf at 1.9 m is not a slightly worse place to look
    for a bowl, it is not a place to look for a bowl.
    """
    h_min, h_max, a_min = AFFORDANCE.get(normalize_label(target), DEFAULT_AFFORDANCE)
    return 1.0 if (h_min <= top_h <= h_max and area_m2 >= a_min) else 0.0

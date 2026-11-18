"""The height prior at commit (`verification.commit_max_above_storey_m`).

A one-shot track whose centre sits metres above the floor the agent stands on
is usually a mis-projected sliver. The gate asks for corroboration rather than
striking the track off, and it can never empty the candidate list: a verdict
that leaves the agent nothing to commit to is worse than the behaviour it
replaces.
"""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np


class _Track:
    def __init__(self, tid, n_obs):
        self.id = tid
        self.n_obs = n_obs


def _channel(limit, min_obs, floor_y, centres):
    from osg.agent.candidate import CandidatePolicy
    channel = CandidatePolicy.__new__(CandidatePolicy)
    channel.nav = SimpleNamespace(
        cfg=SimpleNamespace(verification=SimpleNamespace(
            commit_max_above_storey_m=limit, commit_high_min_obs=min_obs)),
        map=SimpleNamespace(floor_y=floor_y),
        object_layer=SimpleNamespace(center_of=lambda t: np.array(centres[t.id])),
        stats={},
    )
    return channel


def test_a_one_shot_track_metres_above_the_floor_is_held_back():
    """00800's toy airplane spent BOTH early attempts, by step 95, on
    single-observation tracks at y 5.64 and 5.09 above a 3.16 m floor."""
    from osg.agent.candidate import CandidatePolicy
    high, low = _Track(1, 1), _Track(2, 3)
    channel = _channel(1.0, 2, 3.16, {1: (0, 5.64, 0), 2: (0, 3.9, 0)})

    kept = CandidatePolicy._height_plausible(channel, [high, low])

    assert [t.id for t in kept] == [2]
    assert channel.nav.stats["commit_held_high_track"] == 1


def test_corroboration_lets_a_high_track_compete_again():
    """A real object CAN sit high on a shelf, so this asks for a second look
    rather than striking the track off."""
    from osg.agent.candidate import CandidatePolicy
    seen_twice = _Track(1, 2)
    channel = _channel(1.0, 2, 3.16, {1: (0, 5.64, 0)})
    assert CandidatePolicy._height_plausible(channel, [seen_twice]) == [seen_twice]


def test_when_every_candidate_is_high_the_best_is_still_offered():
    """Refusing to commit at all is how an agent ends an episode having done
    nothing."""
    from osg.agent.candidate import CandidatePolicy
    a, b = _Track(1, 1), _Track(2, 1)
    channel = _channel(1.0, 2, 3.16, {1: (0, 5.64, 0), 2: (0, 5.09, 0)})
    assert CandidatePolicy._height_plausible(channel, [a, b]) == [a, b]


def test_the_gate_is_off_by_default():
    from osg.agent.candidate import CandidatePolicy
    a = _Track(1, 1)
    channel = _channel(0.0, 2, 3.16, {1: (0, 99.0, 0)})
    assert CandidatePolicy._height_plausible(channel, [a]) == [a]


def test_no_floor_height_yet_means_no_gate():
    """Before the first frame latches the ground plane there is nothing to
    measure "above" from."""
    from osg.agent.candidate import CandidatePolicy
    a = _Track(1, 1)
    channel = _channel(1.0, 2, None, {1: (0, 99.0, 0)})
    assert CandidatePolicy._height_plausible(channel, [a]) == [a]

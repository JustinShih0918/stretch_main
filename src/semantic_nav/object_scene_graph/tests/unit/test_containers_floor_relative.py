"""A support surface is measured from the floor, not from y=0.

`container_top_h_m` is a band ABOVE THE FLOOR -- 0.2 to 1.4 m, the height range
a thing can be set down on -- and `containers.top_height` returns an absolute
world height. The two coincide only when the floor sits at y=0; a scene or a
robot frame whose floor is elsewhere shifts every surface through the band.
"""
from __future__ import annotations

import numpy as np

from osg.graph.scene_graph import SceneGraph
from osg.mapping.costmap import Costmap2D
from osg.objects.association import ObjectTrack
from osg.objects.ellipsoid import Ellipsoid

RAISED_FLOOR_Y = 2.86


class _Layer:
    def __init__(self, tracks):
        self._t = {t.id: t for t in tracks}

    def tracks(self, include_blacklisted: bool = False):
        return list(self._t.values())

    def center_of(self, track):
        return np.asarray(track.ellipsoid.center, dtype=float)


def _table(tid, floor_y):
    """A table whose top sits 0.75 m above the given floor."""
    centre = np.array([1.0, floor_y + 0.6, 2.0])
    track = ObjectTrack(
        id=tid, label="table",
        ellipsoid=Ellipsoid(center=centre, axes=np.array([0.6, 0.15, 0.6]), R=np.eye(3)),
    )
    track.best_score = 0.9
    for _ in range(5):
        track.observations.append(None)
    return track


def _graph(floor_relative):
    return SceneGraph(container_top_h_m=(0.2, 1.4), container_min_area_m2=0.0,
                      container_min_obs=1, container_min_score=0.0,
                      containers_floor_relative=floor_relative)


def _rebuild(graph, track, floor_y):
    costmap = Costmap2D(resolution=0.05, size_m=20.0)
    labels = np.zeros(costmap.grid.shape, dtype=np.int32)
    graph.rebuild(labels, costmap, _Layer([track]), floor_y=floor_y)


def test_a_floor_at_zero_is_unaffected():
    for relative in (False, True):
        graph = _graph(relative)
        _rebuild(graph, _table(1, 0.0), floor_y=0.0)
        assert len(graph.containers) == 1, f"floor_relative={relative}"


def test_a_raised_floor_hides_the_table_without_the_flag():
    graph = _graph(False)
    _rebuild(graph, _table(1, RAISED_FLOOR_Y), floor_y=RAISED_FLOOR_Y)
    assert graph.containers == {}


def test_a_raised_floor_keeps_the_table_with_it():
    graph = _graph(True)
    _rebuild(graph, _table(1, RAISED_FLOOR_Y), floor_y=RAISED_FLOOR_Y)
    assert len(graph.containers) == 1


def test_top_h_stays_an_absolute_height():
    """`build_container_candidates` subtracts the floor height itself; if the
    stored value were already relative both would subtract."""
    graph = _graph(True)
    _rebuild(graph, _table(1, RAISED_FLOOR_Y), floor_y=RAISED_FLOOR_Y)
    node = next(iter(graph.containers.values()))
    assert node.top_h > RAISED_FLOOR_Y

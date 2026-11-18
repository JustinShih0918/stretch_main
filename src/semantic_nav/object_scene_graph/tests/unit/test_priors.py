"""Category priors for the search posterior (osg.graph.priors)."""

# ------------------------------------------------------- grounded affinity


def test_a_complete_map_grounds_to_exactly_the_ungrounded_ranking():
    """00829 and 00880 contain all 14 container categories, so grounding cannot
    change them. This is what makes the A/B surgical: only 00848, the scene with
    no counter and no table, can move."""
    from osg.graph.containers import CONTAINER_CATEGORIES
    from osg.graph.priors import affinity_scores

    every = set(CONTAINER_CATEGORIES)
    for target in ("bowl", "cracker box", "red plate", "tin can"):
        assert affinity_scores(target, present=every) == affinity_scores(target)


def test_a_table_entry_is_filtered_to_the_surfaces_that_exist():
    """`bowl` ranks table > counter > desk > cabinet > shelf > sink. In a house
    with neither a table nor a counter the entry must not keep asserting them --
    and desk, not table, is then the best place to look."""
    from osg.graph.priors import affinity_scores

    present = {"desk", "cabinet", "shelf", "bed", "refrigerator"}
    scores = affinity_scores("bowl", present=present)
    # table and counter are buildable-but-absent, so they go; `sink` is not a
    # CONTAINER_CATEGORY at all, scores nothing either way, and stays so that
    # the surviving weights are not rescaled out from under a complete scene.
    assert "table" not in scores and "counter" not in scores
    assert max(scores, key=scores.get) == "desk"
    assert scores["desk"] == 1.0 and scores["cabinet"] < 1.0


def test_an_entry_with_nothing_left_defers_to_the_grounded_source():
    """The one case a model may speak over the static table: the table's options
    do not exist here, so it has abstained rather than been overruled."""
    from osg.graph.priors import affinity_scores

    called = {}

    def source(key):
        called["key"] = key
        return ["cabinet", "shelf"]

    # `pillow` ranks bed > sofa > bench; none of them are in this map.
    scores = affinity_scores("pillow", source=source, present={"cabinet", "shelf"})
    assert called["key"] == "pillow"
    assert max(scores, key=scores.get) == "cabinet"


def test_grounding_the_provider_scopes_its_cache_key():
    """A grounded answer must never overwrite the ungrounded one, and a complete
    scene must keep using the entry it already had -- byte for byte."""
    from osg.llm.affinity import AffinityProvider

    every = ["bed", "cabinet", "counter", "desk", "shelf", "table"]
    p = AffinityProvider(client=None, surfaces=every)
    p._cache = {"pitcher": ["counter", "table"], "pitcher@bed,cabinet,desk,shelf": ["cabinet"]}

    assert p("pitcher") == ["counter", "table"]
    p.ground(set(every))            # complete map -> no scoping at all
    assert p._scope == ""
    assert p("pitcher") == ["counter", "table"]
    p.ground({"bed", "cabinet", "desk", "shelf"})
    assert p("pitcher") == ["cabinet"]

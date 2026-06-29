from semantic_perception.locate_anything_parser import parse_labeled_boxes


def test_parses_multiple_labels_and_boxes():
    answer = (
        "<ref>person</ref>"
        "<box><100><200><300><600></box>"
        "<box><500><100><700><500></box>"
        "<ref>car</ref><box><200><500><800><900></box>"
    )

    assert parse_labeled_boxes(answer, 1000, 500) == [
        {
            "label": "person",
            "x": 100,
            "y": 100,
            "width": 200,
            "height": 200,
        },
        {
            "label": "person",
            "x": 500,
            "y": 50,
            "width": 200,
            "height": 200,
        },
        {
            "label": "car",
            "x": 200,
            "y": 250,
            "width": 600,
            "height": 200,
        },
    ]


def test_clamps_and_normalizes_reversed_corners():
    answer = "<ref>curtain</ref><box><1200><900><-100><100></box>"

    assert parse_labeled_boxes(answer, 640, 480) == [
        {
            "label": "curtain",
            "x": 0,
            "y": 48,
            "width": 640,
            "height": 384,
        }
    ]


def test_ignores_none_malformed_and_zero_area():
    assert parse_labeled_boxes("<box>None</box>", 640, 480) == []
    assert parse_labeled_boxes("not structured", 640, 480) == []
    assert (
        parse_labeled_boxes(
            "<ref>chair</ref><box><20><20><20><100></box>", 640, 480
        )
        == []
    )


def test_parses_valid_boxes_when_other_labels_are_none():
    answer = (
        "<ref>human</ref><box>None</box>"
        "<ref>person</ref><box><423><164><845><821></box>"
        "<ref>curtain</ref><box>None</box>"
        "<ref>wall</ref><box><0><0><999><998></box><|im_end|>"
    )

    assert parse_labeled_boxes(answer, 1280, 720) == [
        {
            "label": "person",
            "x": 541,
            "y": 118,
            "width": 541,
            "height": 473,
        },
        {
            "label": "wall",
            "x": 0,
            "y": 0,
            "width": 1279,
            "height": 719,
        },
    ]


def test_accepts_unlabeled_box_for_single_instruction():
    result = parse_labeled_boxes(
        "<box><250><250><750><750></box>",
        200,
        100,
        fallback_label="red curtain",
    )

    assert result == [
        {
            "label": "red curtain",
            "x": 50,
            "y": 25,
            "width": 100,
            "height": 50,
        }
    ]

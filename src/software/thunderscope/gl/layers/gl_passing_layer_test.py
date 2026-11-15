import proto.import_all_protos as protos
from software.thunderscope.gl.layers.gl_passing_layer import GLPassingLayer


def _pass_with_rating(passer, receiver):
    pass_with_rating = protos.PassWithRating(rating=0.5)
    pass_with_rating.pass_.passer_point.x_meters = passer[0]
    pass_with_rating.pass_.passer_point.y_meters = passer[1]
    pass_with_rating.pass_.receiver_point.x_meters = receiver[0]
    pass_with_rating.pass_.receiver_point.y_meters = receiver[1]
    pass_with_rating.pass_.pass_speed_m_per_s = 3.0
    return pass_with_rating


def test_draws_one_line_per_best_pass():
    layer = GLPassingLayer("passing")
    vis = protos.PassVisualization()
    vis.best_passes.append(_pass_with_rating((0, 0), (1, 1)))
    vis.best_passes.append(_pass_with_rating((0, 0), (2, -1)))
    layer.pass_visualization_buffer.put(vis)

    layer.refresh_graphics()

    assert len(layer.pass_graphics) == 2


def test_shrinks_when_fewer_passes_arrive():
    layer = GLPassingLayer("passing")
    vis = protos.PassVisualization()
    vis.best_passes.append(_pass_with_rating((0, 0), (1, 1)))
    vis.best_passes.append(_pass_with_rating((0, 0), (2, -1)))
    layer.pass_visualization_buffer.put(vis)
    layer.refresh_graphics()

    vis = protos.PassVisualization()
    vis.best_passes.append(_pass_with_rating((0, 0), (1, 1)))
    layer.pass_visualization_buffer.put(vis)
    layer.refresh_graphics()

    assert len(layer.pass_graphics) == 1


if __name__ == "__main__":
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))

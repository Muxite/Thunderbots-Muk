import time
from typing import override

import proto.import_all_protos as protos
from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from software.thunderscope.gl.helpers.observable_list import ObservableList
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class GLPassingLayer(GLLayer):
    """GLLayer that visualizes passes"""

    # The pass generator is created and destroyed as we move in and out of offensive plays.
    # If we no longer receive new passes, we need to remove the old one.
    PASS_VISUALIZATION_TIMEOUT_S = 0.5

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLPassingLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.pass_visualization_buffer = ThreadSafeBuffer(
            buffer_size, protos.PassVisualization
        )
        self.cached_pass_vis = protos.PassVisualization()
        self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S

        self.pass_graphics = ObservableList(self._graphics_changed)

    @override
    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        pass_vis = self.pass_visualization_buffer.get(block=False, return_cached=False)

        if not pass_vis:
            pass_vis = self.cached_pass_vis

            # If we haven't received pass visualizations for a bit, skip updating graphics
            if time.time() > self.timeout:
                self.pass_graphics.clear()
                return
        else:
            # We received new pass data, so lets update our timeout
            self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S
            self.cached_pass_vis = pass_vis

        # One line per candidate pass. Committed passes are drawn by the attacker layer,
        # which is where the commitment flag lives, so these are all uncommitted.
        best_passes = list(pass_vis.best_passes)
        self.pass_graphics.resize(
            len(best_passes),
            lambda: GLPolygon(
                outline_color=Colors.UNCOMMITTED_PASS_VISUALIZATION_COLOR
            ),
        )

        for graphic, pass_with_rating in zip(self.pass_graphics, best_passes):
            pass_proto = pass_with_rating.pass_
            graphic.set_points(
                [
                    [
                        pass_proto.passer_point.x_meters,
                        pass_proto.passer_point.y_meters,
                    ],
                    [
                        pass_proto.receiver_point.x_meters,
                        pass_proto.receiver_point.y_meters,
                    ],
                ]
            )

from __future__ import annotations

import sys
import unittest
from pathlib import Path


WALL_TOOL_PROJECT_ROOT = Path(__file__).resolve().parents[2]
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
WALL_TOOL_3D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_3d"
for path in (WALL_TOOL_3D_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from coppeliasim_vector_tool.control_ui import (  # noqa: E402
    append_connected_stroke,
    simplify_drawn_path,
    smooth_preview_path,
)
from coppeliasim_vector_tool.interactive_session import SessionConfig  # noqa: E402


class CoppeliaControlUITests(unittest.TestCase):
    def test_freehand_path_is_spacing_filtered_and_bounded(self) -> None:
        points = tuple((0.01 * index, 2.0) for index in range(700))
        simplified = simplify_drawn_path(points)
        self.assertLessEqual(len(simplified), 48)
        self.assertEqual(simplified[0], points[0])
        self.assertEqual(simplified[-1], points[-1])
        self.assertTrue(all(a != b for a, b in zip(simplified, simplified[1:])))

    def test_session_rejects_invalid_or_unbounded_user_commands(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least one"):
            SessionConfig(path=(), path_speed_m_s=0.14, corner_speed_m_s=0.04)
        with self.assertRaisesRegex(ValueError, "Path speed|path speed"):
            SessionConfig(path=((0.0, 2.0),), path_speed_m_s=0.5, corner_speed_m_s=0.04)
        with self.assertRaisesRegex(ValueError, "corner speed"):
            SessionConfig(path=((0.0, 2.0),), path_speed_m_s=0.10, corner_speed_m_s=0.12)

    def test_session_contract_preserves_planned_path_and_view(self) -> None:
        config = SessionConfig(
            path=((-1.0, 1.5), (1.0, 1.5), (1.0, 2.0)),
            path_speed_m_s=0.12,
            corner_speed_m_s=0.04,
            camera="payload",
        )
        self.assertEqual(len(config.path), 3)
        self.assertEqual(config.camera, "payload")

    def test_clicks_and_drag_strokes_append_as_one_connected_path(self) -> None:
        clicked = append_connected_stroke([], [(-0.8, 1.4)])
        clicked = append_connected_stroke(clicked, [(0.2, 1.6)])
        dragged = [(0.4, 1.7), (0.6, 1.85), (0.8, 2.0)]
        combined = append_connected_stroke(clicked, dragged)
        self.assertEqual(combined, [(-0.8, 1.4), (0.2, 1.6), *dragged])
        self.assertLessEqual(len(append_connected_stroke(combined, dragged * 30)), 48)

    def test_smooth_preview_passes_through_all_planned_endpoints(self) -> None:
        points = [(-1.0, 1.2), (1.0, 1.2), (1.0, 2.2), (-1.0, 2.2)]
        preview = smooth_preview_path(points, samples_per_segment=8)
        self.assertEqual(preview[0], points[0])
        self.assertEqual(preview[-1], points[-1])
        for point in points:
            self.assertTrue(any(abs(sample[0] - point[0]) < 1e-9 and abs(sample[1] - point[1]) < 1e-9 for sample in preview))
        self.assertGreater(len(preview), len(points))


if __name__ == "__main__":
    unittest.main()

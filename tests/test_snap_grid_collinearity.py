import math, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'engine'))
from import_bcbend import on_axis


class SnapGridCollinearityTest(unittest.TestCase):
    def test_exact_tolerance_without_declared_grid(self):
        self.assertTrue(on_axis([50., 5e-6], [0., 0.], [100., 0.], 0.))
        self.assertFalse(on_axis([50., 2e-5], [0., 0.], [100., 0.], 0.))

    def test_snapped_points_on_declared_grid_are_collinear(self):
        r = 1e-4*25.4*math.sqrt(2)/2
        # A span beyond the lead: tilt of the snapped lead line grows with t.
        self.assertTrue(on_axis([200., 0.0023], [0., 0.], [100., 0.], r))
        self.assertFalse(on_axis([50., 0.01], [0., 0.], [100., 0.], r))


if __name__ == '__main__':
    unittest.main()

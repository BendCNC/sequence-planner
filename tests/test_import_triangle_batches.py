from pathlib import Path
import sys
import unittest

import shapely
from shapely.geometry import GeometryCollection, Point, Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "engine"))
from import_bcbend import _triangle_rows


class TriangulatedFaceBatchTests(unittest.TestCase):
    def check_rows(self, collection):
        expected = [(t.area, list(t.exterior.coords)[:3])
                    for t in collection.geoms if not t.area < 1e-14]
        actual = [(a, [tuple(p) for p in xyz]) for a, xyz in _triangle_rows(collection)]
        self.assertEqual(actual, expected)

    def test_holes_multiple_batches_and_zero_area(self):
        region = Point(0, 0).buffer(10, quad_segs=41).difference(Point(0, 0).buffer(3, quad_segs=25))
        faces = shapely.constrained_delaunay_triangles(region)
        self.assertGreater(len(faces.geoms), 128)
        self.check_rows(faces)
        triangle = Polygon([(0, 0), (2, 0), (0, 1)])
        tiny = Polygon([(0, 0), (1e-8, 0), (0, 1e-8)])
        quad = Polygon([(0, 0), (2, 0), (2, 1), (0, 1)])
        self.check_rows(GeometryCollection([triangle] * 63 + [quad, tiny, triangle]))

    def test_preserves_z_and_m_ordinates_and_the_scalar_error(self):
        for dimension, points in [('Z', '0 0 3, 2 0 2, 0 2 4, 0 0 3'),
                                  ('M', '0 0 0, 2 0 7, 0 2 9, 0 0 0')]:
            with self.subTest(dimension=dimension):
                if dimension == 'M' and shapely.geos_version < (3, 12, 0):
                    continue
                tri = shapely.from_wkt(f'POLYGON {dimension} (({points}))')
                self.check_rows(GeometryCollection([tri] * 66))
        self.check_rows(GeometryCollection([]))
        triangle = Polygon([(0, 0), (2, 0), (0, 1)])
        unexpected = GeometryCollection([triangle] * 4 + [shapely.MultiPolygon([triangle])])
        with self.assertRaises(AttributeError):
            list(_triangle_rows(unexpected))


if __name__ == '__main__':
    unittest.main()

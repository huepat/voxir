using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System.Collections.Generic;

namespace HuePat.VoxIR.Datasets.Util.RayTracing {
    public class Ray {
        public Vector3d Origin { get; private set; }
        public Vector3d Direction { get; private set; }
        public Vector3d InverseDirection { get; private set; }

        public Ray(
                Vector3d origin,
                Vector3d direction) {

            Origin = origin;
            Direction = direction;
            InverseDirection = new Vector3d(
                direction.X == 0.0 ? double.MaxValue : 1.0 / direction.X,
                direction.Y == 0.0 ? double.MaxValue : 1.0 / direction.Y,
                direction.Z == 0.0 ? double.MaxValue : 1.0 / direction.Z);
        }

        public List<double> Intersect(
                AABox box) {

            double minDistance;
            double maxDistance;
            List<double> intersections = new List<double>();

            if (!Intersects(
                    box,
                    out minDistance,
                    out maxDistance)) {
                return intersections;
            }

            if (minDistance > 0.0) {
                intersections.Add(minDistance);
            }
            intersections.Add(maxDistance);

            return intersections;
        }

        public bool Intersects(
                Triangle triangle,
                out double distance) {

            double num, denom;

            distance = -1.0;

            num = Vector3d.Dot(
                triangle.Centroid - Origin, 
                triangle.Normal);
            denom = Vector3d.Dot(
                triangle.Normal, 
                Direction);

            if (denom.Abs() < double.Epsilon) {
                // not entirely correct, ray could lie within plane
                return false;
            }

            distance = num / denom;
            if (distance < 0
                    && distance.Abs() > double.Epsilon) {
                return false;
            }

            return ContainsCoplanar(
                triangle,
                At(distance));
        }

        private bool Intersects(
                AABox box,
                out double minDistance,
                out double maxDistance) {

            double tMinX = (box.Min.X - Origin.X) * InverseDirection.X;
            double tMaxX = (box.Max.X - Origin.X) * InverseDirection.X;
            double tMinY = (box.Min.Y - Origin.Y) * InverseDirection.Y;
            double tMaxY = (box.Max.Y - Origin.Y) * InverseDirection.Y;
            double tMinZ = (box.Min.Z - Origin.Z) * InverseDirection.Z;
            double tMaxZ = (box.Max.Z - Origin.Z) * InverseDirection.Z;

            minDistance = System.Math.Max(
                System.Math.Max(
                    System.Math.Min(
                        tMinX,
                        tMaxX),
                    System.Math.Min(
                        tMinY,
                        tMaxY)),
                System.Math.Min(
                    tMinZ,
                    tMaxZ));

            maxDistance = System.Math.Min(
                System.Math.Min(
                    System.Math.Max(
                        tMinX,
                        tMaxX),
                    System.Math.Max(
                        tMinY,
                        tMaxY)),
                System.Math.Max(
                    tMinZ,
                    tMaxZ));

            return maxDistance >= 0.0
                && maxDistance >= minDistance;
        }

        private Vector3d At(
                double distance) {

            if (distance < 0.0) {
                return Origin;
            }
            return Origin + distance * Direction;
        }

        private bool ContainsCoplanar(
                Triangle triangle,
                Vector3d position) {

            return SameSide(
                    position, 
                    triangle.Corner1,
                    triangle.Corner2,
                    triangle.Corner3)
                && SameSide(
                    position,
                    triangle.Corner2,
                    triangle.Corner1,
                    triangle.Corner3)
                && SameSide(
                    position,
                    triangle.Corner3,
                    triangle.Corner1,
                    triangle.Corner2);
        }

        private bool SameSide(
                Vector3d p1, 
                Vector3d p2, 
                Vector3d A, 
                Vector3d B) {

            Vector3d d = B - A;

            return Vector3d.Dot(
                Vector3d.Cross(
                    d, 
                    p1 - A), 
                Vector3d.Cross(
                    d, 
                    p2 - A)) >= double.Epsilon;
        }
    }
}
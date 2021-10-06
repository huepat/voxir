using HuePat.VoxIR.Datasets.Util.RayTracing;
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.Datasets.Util.Octree {
    public enum OctTreeCellFilter { ALL, LEAF, CONTAINS_GEOMETRIES }

    class Octree {
        private Cell root;

        public Octree(
                List<(Mesh, GroundTruthInfo)> groundTruthSurfaces) {

            root = new ParallelSplitCell(groundTruthSurfaces);
        }

        public (Triangle, GroundTruthInfo) GetNearest(
                double distanceThreshold,
                Vector3d position) {

            double distance;
            Cell candidateCell;
            (Triangle, GroundTruthInfo) candidateTriangle;
            List<(Triangle, GroundTruthInfo)> extendedCandidates;

            candidateCell = GetNearest(
                position,
                root);
            candidateTriangle = GetNearest(
                distanceThreshold,
                position,
                candidateCell.Triangles);

            while (candidateTriangle == (null, null)) {
                candidateCell = candidateCell.Parent;
                candidateTriangle = GetNearest(
                    distanceThreshold,
                    position,
                    candidateCell.Triangles);
            }

            distance = candidateTriangle.Item1.DistanceTo(position);
            extendedCandidates = GetCandidates(
                root,
                cell => !cell.ApproximateEquals(candidateCell)
                    && cell.DistanceTo(position) < distance);
            extendedCandidates.Add(candidateTriangle);

            candidateTriangle = GetNearest(
                distanceThreshold,  
                position,
                extendedCandidates);

            return candidateTriangle;
        }

        public List<Intersection> Intersect(
                double distanceTheshold,
                Ray ray) {

            List<(Triangle, GroundTruthInfo)> candidates = GetIntersectionCandidates(
                distanceTheshold,
                ray);
            
            return Intersect(
                distanceTheshold,
                ray,
                candidates);
        }

        private Cell GetNearest(
                Vector3d position,
                Cell cell) {

            double value;
            double minValue = double.MaxValue;
            int minIndex = 0;

            if (cell.IsLeaf) {
                return cell;
            }

            for (int i = 0; i < cell.Children.Count; i++) {

                value = cell
                    .Children[i]
                    .DistanceTo(position);

                if (value < minValue) {
                    minValue = value;
                    minIndex = i;
                }
            }

            return GetNearest(
                position,
                cell.Children[minIndex]);
        }

        private (Triangle, GroundTruthInfo) GetNearest(
                double distanceThreshold,
                Vector3d position,
                IReadOnlyList<(Triangle, GroundTruthInfo)> candidates) {

            double distance;
            double minDistance = double.MaxValue;
            (Triangle, GroundTruthInfo) nearest = (null, null);
            
            foreach ((Triangle, GroundTruthInfo) candidate in candidates) {

                distance = candidate.Item1.DistanceTo(position);

                if (distance < minDistance
                        && distance <= distanceThreshold) {

                    minDistance = distance;
                    nearest = candidate;
                }
            }

            return nearest;
        }

        private List<(Triangle, GroundTruthInfo)> GetCandidates(
                Cell cell,
                Predicate<Cell> cellFilter) {

            List<(Triangle, GroundTruthInfo)> candidates 
                = new List<(Triangle, GroundTruthInfo)>();

            if (!cellFilter(cell)) {
                return candidates;
            }

            if (cell.IsLeaf) {
                candidates.AddRange(cell.Triangles);
                return candidates;
            }

            foreach (Cell child in cell.Children) {
                candidates.AddRange(
                    GetCandidates(
                        child,
                        cellFilter));
            }

            return candidates;
        }

        private List<(Triangle, GroundTruthInfo)> GetIntersectionCandidates(
                double distanceTheshold,
                Ray ray) {

            List<double> intersections;

            List<(Triangle, GroundTruthInfo)> candidates = GetCandidates(
                root,
                cell => {

                    intersections = ray.Intersect(cell);
                    if (intersections.Count == 0) {
                        return false;
                    }

                    if (!cell.Contains(ray.Origin)) {
                        return intersections
                            .Any(intersection => intersection <= distanceTheshold);
                    }

                    return true;
                });

            return candidates;
        }

        public List<Intersection> Intersect(
                double distanceTheshold,
                Ray ray,
                List<(Triangle, GroundTruthInfo)> triangles) {

            double distance;
            List<double> boxIntersections;
            List<Intersection> intersections = new List<Intersection>();

            foreach ((Triangle, GroundTruthInfo) triangle in triangles) {

                boxIntersections = ray.Intersect(triangle.Item1.BBox);

                if (boxIntersections.Count != 0
                        && boxIntersections
                            .Any(intersection => intersection <= distanceTheshold)
                        && ray.Intersects(
                            triangle.Item1,
                            out distance)) {

                    intersections.Add(
                        new Intersection(
                            distance,
                            triangle));
                }
            }

            return intersections;
        }
    }
}
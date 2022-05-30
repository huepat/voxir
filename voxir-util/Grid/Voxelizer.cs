using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Util.Grid {
    public static class Voxelizer {
        public static (int, int, int) DetermineGridSize(
                double resolution,
                ref AABox gridExtent) {

            gridExtent = new AABox(
                gridExtent.Min - new Vector3d(resolution),
                gridExtent.Max + new Vector3d(resolution));

            return (
                (int)((gridExtent.Max.Y - gridExtent.Min.Y) / resolution).Ceil(),
                (int)((gridExtent.Max.X - gridExtent.Min.X) / resolution).Ceil(),
                (int)((gridExtent.Max.Z - gridExtent.Min.Z) / resolution).Ceil()
            );
        }

        public static bool[,,] VoxelizePointCloud(
                double resolution,
                (int, int, int) gridSize,
                Vector3d offset,
                PointCloud pointCloud) {

            bool[,,] grid = new bool[
                gridSize.Item1,
                gridSize.Item2,
                gridSize.Item3];

            Parallel.For(
                0,
                pointCloud.Count,
                j => {

                    (int, int, int) voxel = ToVoxel(
                        resolution,
                        offset,
                        pointCloud[j]);

                    grid[
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3] = true;
                });

            return grid;
        }

        public static void VoxelizeMesh(
                double resolution,
                int[,,][] grid,
                Vector3d offset,
                Mesh mesh,
                Func<Triangle, AABox, int[]> triangleCallback,
                Func<int[], int[], int[]> fusionCallback) {

            object @lock = new object();
            int[] gridSize = new int[] {
                grid.GetLength(0),
                grid.GetLength(1),
                grid.GetLength(2)
            };
            Dictionary<(int, int, int), int[]> voxelStates
                = new Dictionary<(int, int, int), int[]>();

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    mesh.Count),
                () => new Dictionary<(int, int, int), int[]>(),
                (partition, loopState, localVoxelStates) => {

                    for (int j = partition.Item1; j < partition.Item2; j++) {

                        VoxelizeTriangle(
                            resolution,
                            gridSize,
                            offset,
                            mesh[j].Geometry,
                            localVoxelStates,
                            triangleCallback,
                            fusionCallback);
                    }

                    return localVoxelStates;
                },
                localVoxelStates => {

                    lock (@lock) {
                        foreach ((int, int, int) voxel in localVoxelStates.Keys) {

                            if (voxelStates.ContainsKey(voxel)) {

                                voxelStates[voxel] = fusionCallback(
                                    voxelStates[voxel],
                                    localVoxelStates[voxel]);
                            }
                            else {
                                voxelStates.Add(
                                    voxel,
                                    localVoxelStates[voxel]);
                            }
                        }
                    }
                });

            PopulateGrid(
                grid,
                voxelStates,
                fusionCallback);
        }

        private static (int, int, int) ToVoxel(
                double resolution,
                Vector3d offset,
                Point point) {

            (double, double, double) gridCoordinate = ToGridCoordinate(
                resolution,
                point.Position,
                offset);

            return (
                (int)gridCoordinate.Item1,
                (int)gridCoordinate.Item2,
                (int)gridCoordinate.Item3
            );
        }

        private static (double, double, double) ToGridCoordinate(
                double resolution,
                Vector3d coordinate,
                Vector3d offset) {

            return (
                (coordinate.Y - offset.Y) / resolution,
                (coordinate.X - offset.X) / resolution,
                (coordinate.Z - offset.Z) / resolution
            );
        }

        private static void VoxelizeTriangle(
                double resolution,
                int[] gridSize,
                Vector3d offset,
                Triangle triangle,
                Dictionary<(int, int, int), int[]> voxelStates,
                Func<Triangle, AABox, int[]> triangleCallback,
                Func<int[], int[], int[]> fusionCallback) {

            int i, r, c;
            int[] gridMin, gridMax;
            AABox bBox;
            AABox voxel;

            bBox = triangle.BBox;
            gridMin = bBox.Min.ToGridCoordinate(
                true,
                resolution,
                gridSize,
                offset);
            gridMax = bBox.Max.ToGridCoordinate(
                false,
                resolution,
                gridSize,
                offset);

            for (i = gridMin[0]; i <= gridMax[0]; i++) {
                for (r = gridMin[1]; r <= gridMax[1]; r++) {
                    for (c = gridMin[2]; c <= gridMax[2]; c++) {

                        voxel = GetVoxelGeometry(
                            i,
                            r,
                            c,
                            resolution,
                            offset);

                        if (triangle.Intersects(voxel)) {
                            if (!voxelStates.ContainsKey((i, r, c))) {

                                voxelStates.Add(
                                    (i, r, c),
                                    triangleCallback(
                                        triangle,
                                        voxel));
                            }
                            else {

                                voxelStates[(i, r, c)] = fusionCallback(
                                    voxelStates[(i, r, c)],
                                    triangleCallback(
                                        triangle,
                                        voxel));
                            }
                        }
                    }
                }
            }
        }

        private static int[] ToGridCoordinate(
                this Vector3d coordinate,
                bool isMin,
                double resolution,
                int[] gridSize,
                Vector3d offset) {

            int[] result;
            (double, double, double) gridCoordinate = ToGridCoordinate(
                resolution,
                coordinate,
                offset);

            if (isMin) {
                result = new int[] {
                    (int)gridCoordinate.Item1.Floor(),
                    (int)gridCoordinate.Item2.Floor(),
                    (int)gridCoordinate.Item3.Floor()
                };
            }
            else {
                result = new int[] {
                    (int)gridCoordinate.Item1.Ceil(),
                    (int)gridCoordinate.Item2.Ceil(),
                    (int)gridCoordinate.Item3.Ceil()
                };
            }

            for (int i = 0; i <= 2; i++) {
                if (result[i] < 0) {
                    result[i]++;
                }
                if (result[i] >= gridSize[i]) {
                    result[i]--;
                }
            }

            return result;
        }

        private static AABox GetVoxelGeometry(
                int i,
                int r,
                int c,
                double resolution,
                Vector3d offset) {

            return AABox.FromCenterAndSize(
                new Vector3d(
                    offset.X + r * resolution,
                    offset.Y + i * resolution,
                    offset.Z + c * resolution),
                new Vector3d(resolution));
        }

        private static void PopulateGrid(
                int[,,][] grid,
                Dictionary<(int, int, int), int[]> voxelStates,
                Func<int[], int[], int[]> fusionCallback) {

            List<(int, int, int)> voxels = voxelStates.Keys.ToList();

            Parallel.For(
                0,
                voxels.Count,
                j => {

                    grid[
                        voxels[j].Item1,
                        voxels[j].Item2,
                        voxels[j].Item3] = fusionCallback(
                            grid[
                                voxels[j].Item1,
                                voxels[j].Item2,
                                voxels[j].Item3],
                            voxelStates[voxels[j]]);
                });
        }
    }
}
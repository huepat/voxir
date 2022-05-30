using HuePat.VoxIR.Util.Geometry;
using HuePat.VoxIR.Util.Grid;
using OpenTK.Mathematics;
using System.Threading.Tasks;

namespace HuePat.VoxIR.DataPreparation {
    public static class Voxelization {
        private const int DIRECTION_DOWN_NORMAL_COUNTER_INDEX = 0;
        private const int DIRECTION_UP_NORMAL_COUNTER_INDEX = 1;
        private const int DIRECTION_ELSE_NORMAL_COUNTER_INDEX = 2;
        private const double DIRECTION_UP_AND_DOWN_MIN_FRACTION = 0.3;
        public static readonly Vector3d DIRECTION_DOWN = new Vector3d(0, -1, 0);
        public static readonly Vector3d DIRECTION_UP = new Vector3d(0, 1, 0);

        public static byte[,,] Voxelize(
                double resolution,
                AABox gridExtent,
                Mesh mesh) {

            (int, int, int) gridSize;
            int[,,][] normalCounterGrid;

            if (gridExtent == null) {
                gridExtent = mesh.BBox;
            }

            gridSize = Voxelizer.DetermineGridSize(
                resolution,
                ref gridExtent);

            normalCounterGrid = new int[
                gridSize.Item1,
                gridSize.Item2,
                gridSize.Item3][];

            Voxelizer.VoxelizeMesh(
                resolution,
                normalCounterGrid,
                gridExtent.Min,
                mesh,
                (triangle, voxel) => {

                    if (Vector3d.CalculateAngle(
                            triangle.Normal, 
                            DIRECTION_DOWN).Abs() < Parameters.DIRECTION_DOWN_ANGLE_THRESHOLD) {

                        return new int[] { 1, 0, 0 };
                    }
                    if (Vector3d.CalculateAngle(
                            triangle.Normal, 
                            DIRECTION_UP).Abs() < Parameters.DIRECTION_UP_ANGLE_THRESHOLD) {

                        return new int[] { 0, 1, 0 };
                    }

                    return new int[] { 0, 0, 1 };
                },
                (counter1, counter2) => {

                    if (counter1 == null) {
                        return counter2;
                    }

                    return new int[] {
                        counter1[0] + counter2[0],
                        counter1[1] + counter2[1],
                        counter1[2] + counter2[2]
                    };
                });

            return InitializeNormalGrid(normalCounterGrid);
        }

        private static byte[,,] InitializeNormalGrid(
                int[,,][] normalCounterGrid) {

            byte[,,] normalGrid = new byte[
                normalCounterGrid.GetLength(0),
                normalCounterGrid.GetLength(1),
                normalCounterGrid.GetLength(2)];

            Parallel.For(
                0,
                normalGrid.GetLength(2),
                c => {

                    int i, r;

                    for (i = 0; i < normalGrid.GetLength(0); i++) {
                        for (r = 0; r < normalGrid.GetLength(1); r++) {

                            normalGrid[i, r, c] = normalCounterGrid[i, r, c].ToByte();
                        }
                    }
                });

            return normalGrid;
        }

        private static byte ToByte(
                this int[] normalCounterValue) {

            double sum;

            if (normalCounterValue == null) {
                return NormalGridValues.EMPTY;
            }

            if (normalCounterValue[DIRECTION_ELSE_NORMAL_COUNTER_INDEX] >= normalCounterValue[DIRECTION_DOWN_NORMAL_COUNTER_INDEX]
                    && normalCounterValue[DIRECTION_ELSE_NORMAL_COUNTER_INDEX] >= normalCounterValue[DIRECTION_UP_NORMAL_COUNTER_INDEX]) {
                return NormalGridValues.NORMAL_HORIZONTAL;
            }

            sum = normalCounterValue[DIRECTION_ELSE_NORMAL_COUNTER_INDEX]
                + normalCounterValue[DIRECTION_UP_NORMAL_COUNTER_INDEX]
                + normalCounterValue[DIRECTION_DOWN_NORMAL_COUNTER_INDEX];

            if (normalCounterValue[DIRECTION_UP_NORMAL_COUNTER_INDEX] / sum >= DIRECTION_UP_AND_DOWN_MIN_FRACTION
                    && normalCounterValue[DIRECTION_DOWN_NORMAL_COUNTER_INDEX] / sum >= DIRECTION_UP_AND_DOWN_MIN_FRACTION) {
                return NormalGridValues.NORMAL_UP_AND_DOWN;
            }

            if (normalCounterValue[DIRECTION_DOWN_NORMAL_COUNTER_INDEX] >= normalCounterValue[DIRECTION_UP_NORMAL_COUNTER_INDEX]) {
                return NormalGridValues.NORMAL_DOWN;
            }

            return NormalGridValues.NORMAL_UP;
        }
    }
}
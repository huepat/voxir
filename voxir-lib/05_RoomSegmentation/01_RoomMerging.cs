using HuePat.VoxIR.Util.Grid;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.RoomSegmentation {
    public static class RoomMerging {
        public static void MergeIndoorSpace(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int maxWallThickness = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(resolution);
            int maxWallThicknessDiagonal = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(
                    resolution.GetVoxelSizeDiagonal());
            bool[,,] isClosed = new bool[
                    reconstructionGrid.GetLength(0),
                    reconstructionGrid.GetLength(1),
                    reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    MergeMultiRoomVoxelClassValues(
                        i,
                        reconstructionGrid);

                    CloseUnobstructedGapsInIndoorSpace(
                        i,
                        maxWallThickness,
                        maxWallThicknessDiagonal,
                        isClosed,
                        normalGrid,
                        reconstructionGrid);

                });
        }

        private static void MergeMultiRoomVoxelClassValues(
                int i,
                int[,,][] reconstructionGrid) {

            int r, c;
            int[] voxelState;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];

                    if (voxelState != null) {

                        reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                            0,
                            voxelState
                                .GetVoxelClassValues()
                                .MergeVoxelClassValues());
                    }
                }
            }
        }

        private static int[] MergeVoxelClassValues(
                this int[] voxelClassValues) {

            if (voxelClassValues.Contains(VoxelClassValues.WALL)
                    || voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    || voxelClassValues.Contains(VoxelClassValues.CEILING)) {

                return voxelClassValues
                    .Where(voxelClassValue => voxelClassValue == VoxelClassValues.WALL
                        || voxelClassValue == VoxelClassValues.FLOOR
                        || voxelClassValue == VoxelClassValues.CEILING)
                    .ToArray();
            }

            if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                return new int[] {
                    VoxelClassValues.INTERIOR_OBJECT
                };
            }

            return new int[] {
                VoxelClassValues.EMPTY_INTERIOR
            };
        }

        private static void CloseUnobstructedGapsInIndoorSpace(
                int i,
                int maxWallThickness,
                int maxWallThicknessDiagonal,
                bool[,,] isClosed,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int dr, r, r2, dc, c, c2, closingDistance;
            int[] voxelState;
            int[] voxelClassValues;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    voxelClassValues = voxelState.GetVoxelClassValues(0);
                    if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                            && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr == 0 && dc == 0) {
                                continue;
                            }

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 < 0 || c2 < 0
                                    || r2 >= reconstructionGrid.GetLength(1)
                                    || c2 >= reconstructionGrid.GetLength(2)
                                    || reconstructionGrid[i, r2, c2] != null) {
                                continue;
                            }

                            if (CanClose(
                                    i,
                                    r,
                                    c,
                                    dr,
                                    dc,
                                    dr != 0 && dc != 0 ?
                                        maxWallThicknessDiagonal :
                                        maxWallThickness,
                                    reconstructionGrid,
                                    out closingDistance)) {

                                CloseGap(
                                    i, 
                                    r, 
                                    c,
                                    dr, 
                                    dc, 
                                    closingDistance,
                                    isClosed,
                                    normalGrid,
                                    reconstructionGrid);
                            }
                        }
                    }
                }
            }
        }

        private static bool CanClose(
                int i,
                int r,
                int c,
                int dr,
                int dc,
                int maxD,
                int[,,][] reconstructionGrid,
                out int closingDistance) {

            int r2, c2, d;
            int[] voxelState;
            int[] voxelClassValues;

            closingDistance = -1;

            for (d = 2; d <= maxD; d++) {

                r2 = r + d * dr;
                c2 = c + d * dc;
                if (r2 < 0 || c2 < 0
                    || r2 >= reconstructionGrid.GetLength(1)
                    || c2 >= reconstructionGrid.GetLength(2)) {
                    return false;
                }

                voxelState = reconstructionGrid[i, r2, c2];
                if (voxelState == null) {
                    continue;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(0);
                if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                        || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                        || voxelClassValues.Contains(VoxelClassValues.FLOOR)
                        || voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                    closingDistance = d - 1;
                    return true;
                }

                return false;
            }
            return false;
        }

        private static void CloseGap(
                int i, 
                int r,
                int c,
                int dr,
                int dc,
                int closingDistance,
                bool[,,] isClosed,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int r2, c2, d;

            for (d = 1; d <= closingDistance; d++) {

                r2 = r + d * dr;
                c2 = c + d * dc;

                isClosed[i, r2, c2] = true;

                reconstructionGrid[i, r2, c2] = VoxelState.CreateVoxelState(
                    0,
                    normalGrid[i, r2, c2] == NormalGridValues.EMPTY ?
                        VoxelClassValues.EMPTY_INTERIOR :
                        VoxelClassValues.INTERIOR_OBJECT);
            }
        }
    }
}
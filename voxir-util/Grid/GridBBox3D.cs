using System.Collections.Generic;

namespace HuePat.VoxIR.Util.Grid {
    public class GridBBox3D {
        private (int, int, int) min;
        private (int, int, int) max;

        public (int, int, int) Size {
            get {
                return (
                    max.Item1 - min.Item1 + 1,
                    max.Item2 - min.Item2 + 1,
                    max.Item3 - min.Item3 + 1);
            }
        }

        public (int, int, int) Min {
            get {
                return min;
            }
        }

        public (int, int, int) Max {
            get {
                return max;
            }
        }

        public GridBBox3D(
                IEnumerable<(int, int, int)> voxels) 
                    : this() {

            Add(voxels);
        }

        public GridBBox3D() {

            min = (int.MaxValue, int.MaxValue, int.MaxValue);
            max = (int.MinValue, int.MinValue, int.MinValue);
        }

        public void Add(
                IEnumerable<(int, int, int)> voxels) {

            foreach ((int, int, int) voxel in voxels) {
                Add(voxel);
            }
        }

        public void Add(
                (int, int, int) voxel) {

            if (voxel.Item1 < min.Item1) {
                min.Item1 = voxel.Item1;
            }
            if (voxel.Item2 < min.Item2) {
                min.Item2 = voxel.Item2;
            }
            if (voxel.Item3 < min.Item3) {
                min.Item3 = voxel.Item3;
            }
            if (voxel.Item1 > max.Item1) {
                max.Item1 = voxel.Item1;
            }
            if (voxel.Item2 > max.Item2) {
                max.Item2 = voxel.Item2;
            }
            if (voxel.Item3 > max.Item3) {
                max.Item3 = voxel.Item3;
            }
        }

        public bool Contains(
                (int, int, int) voxel) {

            return voxel.Item1 >= min.Item1 && voxel.Item1 <= max.Item1
                && voxel.Item2 >= min.Item2 && voxel.Item2 <= max.Item2
                && voxel.Item3 >= min.Item3 && voxel.Item3 <= max.Item3;
        }
    }
}
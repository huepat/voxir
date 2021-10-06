using System.Collections.Generic;

namespace HuePat.VoxIR.Util.Grid {
    public class GridBBox2D {
        public (int, int) Size { get; private set; }
        public (int, int) Min { get; private set; }
        public (int, int) Max { get; private set; }

        public GridBBox2D(IEnumerable<(int, int)> pixels) {

            int[] min = new int[] {
                    int.MaxValue,
                    int.MaxValue
            };
            int[] max = new int[] {
                    int.MinValue,
                    int.MinValue
            };

            foreach ((int, int) pixel in pixels) {
                if (pixel.Item1 < min[0]) {
                    min[0] = pixel.Item1;
                }
                if (pixel.Item2 < min[1]) {
                    min[1] = pixel.Item2;
                }
                if (pixel.Item1 > max[0]) {
                    max[0] = pixel.Item1;
                }
                if (pixel.Item2 > max[1]) {
                    max[1] = pixel.Item2;
                }
            }

            Min = (min[0], min[1]);
            Max = (max[0], max[1]);
            Size = (
                max[0] - min[0] + 1,
                max[1] - min[1] + 1
            );
        }
    }
}
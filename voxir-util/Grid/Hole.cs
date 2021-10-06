using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.Util.Grid {
    public class Hole : IEnumerable<(int, int)> {
        public GridBBox2D BBox { get; private set; }
        public List<(int, int)> Interior { get; private set; }
        public List<(int, int)> Border { get; private set; }

        public Hole(
                List<(int, int)> interior,
                List<(int, int)> border) {

            Interior = interior;
            Border = border;
            BBox = new GridBBox2D(this);
        }

        public IEnumerator<(int, int)> GetEnumerator() {

            return Interior
                .Concat(Border)
                .GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator() {

            return GetEnumerator();
        }

        public void Remove(
                HashSet<(int, int)> pixels) {

            Interior = Remove(Interior, pixels);
            Border = Remove(Border, pixels);
        }

        private List<(int, int)> Remove(
                List<(int, int)> pixels,
                HashSet<(int, int)> removePixels) {

            List<(int, int)> result = new List<(int, int)>();

            foreach ((int, int) pixel in pixels) {
                if (!removePixels.Contains(pixel)) {
                    result.Add(pixel);
                }
            }

            return result;
        }
    }
}
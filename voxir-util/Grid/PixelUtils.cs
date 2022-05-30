namespace HuePat.VoxIR.Util.Grid {
    public static class PixelUtils {
        private static readonly double SQRT_2 = 2.0.Sqrt();

        public static double GetDistance(
                (int, int) pixel1,
                (int, int) pixel2) {

            (int, int) distance = (
                (pixel1.Item1 - pixel2.Item1).Abs(),
                (pixel1.Item2 - pixel2.Item2).Abs());

            return SQRT_2
                    * (distance.Item1 < distance.Item2 ?
                        distance.Item1 :
                        distance.Item2)
                + (distance.Item1 - distance.Item2).Abs();
        }
    }
}
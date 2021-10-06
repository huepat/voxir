using HuePat.VoxIR.Util.Geometry;

namespace HuePat.VoxIR.Datasets.Util.RayTracing {
    public class Intersection {
        public double Distance { get; private set; }
        public (Triangle, GroundTruthInfo) Triangle { get; private set; }

        public Intersection(
                double distance,
                (Triangle, GroundTruthInfo) triangle) {

            Distance = distance;
            Triangle = triangle;
        }
    }
}
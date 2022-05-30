using OpenTK.Mathematics;

namespace HuePat.VoxIR.Util.Geometry {
    public interface IGeometrySet: IFiniteGeometry {
        void Rotate(
            double angle,
            Vector3d axis,
            Vector3d anchor);

        Vector3d GetCentroid();
    }
}
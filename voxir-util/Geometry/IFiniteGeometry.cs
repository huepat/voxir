namespace HuePat.VoxIR.Util.Geometry {
    public interface IFiniteGeometry : IGeometry {
        Mesh Mesh { get; }
        AABox BBox { get; }
    }
}
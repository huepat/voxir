using HuePat.VoxIR.Util.Geometry;
using System;

namespace HuePat.VoxIR.IO.PLY.Writing {
    public interface IEncoder : IDisposable {
        void Encode(Point point);

        void Encode(
            Point point,
            Color color);

        void Encode(
            int offset,
            Face face);
    }
}
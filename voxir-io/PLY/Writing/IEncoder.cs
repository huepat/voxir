using HuePat.VoxIR.Util.Geometry;
using System;
using System.Collections.Generic;

namespace HuePat.VoxIR.IO.PLY.Writing {
    public interface IEncoder : IDisposable {
        void Encode(Point point);

        void Encode(
            Point point,
            Color color,
            IList<float>? additionalFloatProperties = null);

        void Encode(
            int offset,
            Face face);
    }
}
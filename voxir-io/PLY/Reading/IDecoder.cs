using HuePat.VoxIR.Util.Geometry;
using System.Collections.Generic;

namespace HuePat.VoxIR.IO.PLY.Reading {
    interface IDecoder {
        List<Point> ReadPoints(
            string file,
            Header header);

        Mesh ReadMesh(
            bool switchNormals,
            bool createBBox,
            bool useParallelForBBox,
            string file,
            Header header);
    }
}
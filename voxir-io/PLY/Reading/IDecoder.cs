using HuePat.VoxIR.Util.Geometry;
using System.Collections.Generic;

namespace HuePat.VoxIR.IO.PLY.Reading {
    interface IDecoder {
        Mesh ReadMesh(
            bool swithNormals,
            string file,
            Header header);
    }
}
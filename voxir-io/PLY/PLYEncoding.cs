using System;

namespace HuePat.VoxIR.IO.PLY {
    public enum PLYEncoding {
        BINARY_LITTLE_ENDIAN,
        BINARY_BIG_ENDIAN,
        ASCII
    }

    static class Extensions {
        public static string GetString(
                this PLYEncoding encoding) {

            switch (encoding) {
                case PLYEncoding.BINARY_LITTLE_ENDIAN:
                    return "binary_little_endian";
                case PLYEncoding.BINARY_BIG_ENDIAN:
                    return "binary_big_endian";
                case PLYEncoding.ASCII:
                    return "ascii";
            }

            throw new ArgumentException();
        }
    }
}
using System.Collections.Generic;

namespace HuePat.VoxIR {
    public class NormalGridValues {
        public const byte EMPTY = 0;
        public const byte NORMAL_DOWN = 1;
        public const byte NORMAL_UP = 2;
        public const byte NORMAL_HORIZONTAL = 3;
    }

    public static class VoxelClassValues {
        public const int CEILING = 0;
        public const int EMPTY_INTERIOR = 1;
        public const int INTERIOR_OBJECT = 2;
        public const int FLOOR = 3;
        public const int WALL = 4;
        public const int WALL_OPENING = 5;

        public static readonly Dictionary<int, string> Labels = new Dictionary<int, string> {
            { CEILING, "CEILING" },
            { EMPTY_INTERIOR, "EMPTY_INTERIOR" },
            { INTERIOR_OBJECT, "INTERIOR_OBJECT" },
            { FLOOR, "FLOOR" },
            { WALL, "WALL" },
            { WALL_OPENING, "WALL_OPENING" }
        };
    }

    public static class PixelClassValues {
        public const int ROOM = 0;
        public const int WALL = 1;
        public const int HOLE_INTERIOR = 2;
        public const int HOLE_BORDER = 3;
    }
}
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.IO.Visualization {
    public enum BackgroundColor {
        BLACK,
        WHITE
    }

    public static class Definitions {
        public static class VisualizationVoxelClassValues {
            public const int NONE = -1;
            public const int CEILING = 0;
            public const int EMPTY_INTERIOR = 1;
            public const int INTERIOR_OBJECT = 2;
            public const int FLOOR = 3;
            public const int WALL_CEILING = 4;
            public const int WALL = 5;
            public const int WALL_FLOOR = 6;
            public const int WALL_OPENING = 7;
            public const int FLOOR_AND_CEILING = 8;
        }

        public static class EvaluationErrorColors {
            public static readonly Color FALSE_POSITIVE_COLOR = Color.Cyan;
            public static readonly Color FALSE_NEGATIVE_COLOR = Color.Yellow;
            public static readonly Color BOTH_ERRORS_COLOR = Color.Pink;
        }

        public static Dictionary<int, Color> VOXEL_CLASS_COLORS = new Dictionary<int, Color> {
            { VisualizationVoxelClassValues.NONE, Color.Yellow },
            { VisualizationVoxelClassValues.CEILING, Color.Red },
            { VisualizationVoxelClassValues.EMPTY_INTERIOR, Color.Purple },
            { VisualizationVoxelClassValues.INTERIOR_OBJECT, Color.DarkGray },
            { VisualizationVoxelClassValues.FLOOR, Color.Green },
            { VisualizationVoxelClassValues.WALL_CEILING, Color.LightRed },
            { VisualizationVoxelClassValues.WALL, new Color(200, 200, 200) },
            { VisualizationVoxelClassValues.WALL_FLOOR, Color.LightGreen },
            { VisualizationVoxelClassValues.WALL_OPENING, Color.LightBlue },
            { VisualizationVoxelClassValues.FLOOR_AND_CEILING, Color.Orange }
        };

        public static int ToVisualizationVoxelClassValue(
                this int[] voxelClassValues) {

            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    && voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                return VisualizationVoxelClassValues.FLOOR_AND_CEILING;
            }

            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                return VisualizationVoxelClassValues.WALL_FLOOR;
            }

            if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                    && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                return VisualizationVoxelClassValues.WALL_CEILING;
            }

            if (voxelClassValues.Contains(VoxelClassValues.WALL)) {
                return VisualizationVoxelClassValues.WALL;
            }

            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                return VisualizationVoxelClassValues.FLOOR;
            }

            if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                return VisualizationVoxelClassValues.CEILING;
            }

            if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                return VisualizationVoxelClassValues.INTERIOR_OBJECT;
            }

            if (voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                return VisualizationVoxelClassValues.WALL_OPENING;
            }

            if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)) {
                return VisualizationVoxelClassValues.EMPTY_INTERIOR;
            }

            return VisualizationVoxelClassValues.NONE;
        }
    }
}
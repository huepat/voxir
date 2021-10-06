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

        public static class WallNormalColors {
            public static readonly Color NORMAL_COLOR_NONE = Color.LightGray;
            public static readonly Color NORMAL_COLOR_MULTIPLE_ROOMS = Color.Brown;
            public static readonly Color NORMAL_COLOR_OTHER = Color.DarkGray;

            public static readonly Dictionary<(int, int), Color> NORMAL_DIRECTION_COLORS = new Dictionary<(int, int), Color> {
                { (0, 1), Color.Red },
                { (1, 0), Color.Green },
                { (0, -1), Color.Blue },
                { (-1, 0), Color.Yellow },
                { (1, 1), Color.Pink },
                { (1, -1), Color.Orange },
                { (-1, -1), Color.Cyan },
                { (-1, 1), Color.Purple },
            };
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

        public static Color GetWallNormalsColor(
                this (int, int)[][] normals) {

            if (normals.Length == 0) {
                return WallNormalColors.NORMAL_COLOR_NONE;
            }

            if (normals.Length > 1) {
                return WallNormalColors.NORMAL_COLOR_MULTIPLE_ROOMS;
            }

            if (normals[0].Length > 1) {
                return WallNormalColors.NORMAL_COLOR_OTHER;
            }

            return WallNormalColors.NORMAL_DIRECTION_COLORS[normals[0][0]];
        }
    }
}
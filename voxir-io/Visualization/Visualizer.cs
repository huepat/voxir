using HuePat.VoxIR.IO.PLY.Writing;
using HuePat.VoxIR.Util.Geometry;
using HuePat.VoxIR.Util.Grid;
using HuePat.VoxIR.VoxelModelRefinement;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.IO.Visualization {
    public static class Visualizer {
        private static readonly IVoxelMesher DEFAULT_VOXEL_MESHER = new GridFrameVoxelMesher();

        public static void Write(
                this PLYWriter writer,
                int i,
                int r,
                int c,
                Color color) {

            writer.Write(
                i,
                r,
                c,
                color,
                DEFAULT_VOXEL_MESHER);
        }

        public static void Write(
                this PLYWriter writer,
                int i,
                int r,
                int c,
                Color color,
                IVoxelMesher voxelMesher) {

            writer.Write(
                voxelMesher.Mesh(
                    i,
                    r,
                    c),
                color);
        }

        public static void VisualizeMeshRampSpacesAsPLY(
                string file,
                bool[] isRampSpace,
                Mesh mesh) {

            if (isRampSpace.Length != mesh.Count) {
                throw new ArgumentException(
                    "Size of 'isRampSpace' must equal face count of mesh.");
            }

            using (PLYWriter writer = new PLYWriter(file)) {
                for (int j = 0; j < mesh.Count; j++) {

                    writer.Write(
                        mesh[j].Mesh,
                        isRampSpace[j] ?
                            Color.Green :
                            Color.Red);
                }
            }
        }

        public static void VisualizeMeshClassificationAsPLY(
                string file,
                int[] classValues,
                Mesh mesh) {

            if (classValues.Length != mesh.Count) {
                throw new ArgumentException(
                    "Size of 'classValues' must equal face count of mesh.");
            }

            using (PLYWriter writer = new PLYWriter(file)) {
                for (int j = 0; j < mesh.Count; j++) {

                    writer.Write(
                        mesh[j].Mesh,
                        Definitions.VOXEL_CLASS_COLORS[
                            new int[] {
                                classValues[j] 
                            }.ToVisualizationVoxelClassValue()]);
                }
            }
        }

        public static void VisualizeMeshSpacePartitioningAsPLY(
                string file,
                int[] roomIds,
                Mesh mesh) {

            if (roomIds.Length != mesh.Count) {
                throw new ArgumentException(
                    "Size of 'classValues' must equal face count of mesh.");
            }

            HashSet<int> distinctRoomIds = new HashSet<int>();
            Dictionary<int, Color> colors;

            distinctRoomIds.AddRange(roomIds);
            colors = Color.GetRandomColorMapping(distinctRoomIds);

            using (PLYWriter writer = new PLYWriter(file)) {
                for (int j = 0; j < mesh.Count; j++) {

                    writer.Write(
                        mesh[j].Mesh,
                        colors[roomIds[j]]);
                }
            }
        }

        public static void VisualizeVoxelClassificationAsPLY(
                string file,
                int[,,][] grid) {

            VisualizeVoxelClassificationAsPLY(
                file,
                grid,
                DEFAULT_VOXEL_MESHER);
        }

        public static void VisualizeVoxelClassificationAsPLY(
                string file,
                int[,,][] grid,
                IVoxelMesher voxelMesher) { 

            int i, r, c;
            int[] voxelState;
            int[] voxelClassValues;

            using (PLYWriter writer = new PLYWriter(file) { Encoding = PLY.PLYEncoding.ASCII }) {
                for (i = 0; i < grid.GetLength(0); i++) {
                    for (r = 0; r < grid.GetLength(1); r++) {
                        for (c = 0; c < grid.GetLength(2); c++) {

                            voxelState = grid[i, r, c];
                            if (voxelState == null) {
                                continue;
                            }

                            voxelClassValues = voxelState
                                .GetVoxelClassValues()
                                .Distinct()
                                .ToArray();

                            if (voxelClassValues.Length == 1 
                                    && voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)) {
                                continue;
                            }

                            writer.Write(
                                i, 
                                r, 
                                c,
                                Definitions.VOXEL_CLASS_COLORS[
                                    voxelClassValues.ToVisualizationVoxelClassValue()],
                                voxelMesher);
                        }
                    }
                }
            }
        }

        public static void VisualizeVoxelClassificationAsSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) {

            VisualizeVoxelClassificationAsISections(
                file,
                grid,
                backgroundColor);

            VisualizeVoxelClassificationAsRSections(
                file,
                grid,
                backgroundColor);

            VisualizeVoxelClassificationAsCSections(
                file,
                grid,
                backgroundColor);
        }

        public static void VisualizeVoxelClassificationAsISections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) { 

            int i, r, c;
            int[] voxelState;

            for (i = 0; i < grid.GetLength(0); i++) {

                using (ImageWriter writer = new ImageWriter(
                        grid.GetLength(1),
                        grid.GetLength(2),
                        FileSystemUtils.GetFileWithPostfix(
                            file,
                            $"_i{i}"))) {

                    for (r = 0; r < grid.GetLength(1); r++) {
                        for (c = 0; c < grid.GetLength(2); c++) {

                            voxelState = grid[i, r, c];
                            writer.Write(
                                r,
                                c,
                                voxelState == null ?
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.Black :
                                        Color.White :
                                    Definitions.VOXEL_CLASS_COLORS[
                                        voxelState
                                            .GetVoxelClassValues()
                                            .ToVisualizationVoxelClassValue()]);
                        }
                    }
                }
            }
        }

        public static void VisualizeVoxelClassificationAsRSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) {

            int i, r, c;
            int[] voxelState;

            for (r = 0; r < grid.GetLength(1); r++) {

                using (ImageWriter writer = new ImageWriter(
                        grid.GetLength(0),
                        grid.GetLength(2),
                        FileSystemUtils.GetFileWithPostfix(
                            file,
                            $"_r{r}"))) {

                    for (i = 0; i < grid.GetLength(0); i++) {
                        for (c = 0; c < grid.GetLength(2); c++) {

                            voxelState = grid[i, r, c];
                            writer.Write(
                                i,
                                c,
                                voxelState == null ?
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.Black :
                                        Color.White :
                                    Definitions.VOXEL_CLASS_COLORS[
                                        voxelState
                                            .GetVoxelClassValues()
                                            .ToVisualizationVoxelClassValue()]);
                        }
                    }
                }
            }
        }

        public static void VisualizeVoxelClassificationAsCSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) {

            int i, r, c;
            int[] voxelState;

            for (c = 0; c < grid.GetLength(2); c++) {

                using (ImageWriter writer = new ImageWriter(
                        grid.GetLength(0),
                        grid.GetLength(1),
                        FileSystemUtils.GetFileWithPostfix(
                            file,
                            $"_c{c}"))) {

                    for (i = 0; i < grid.GetLength(0); i++) {
                        for (r = 0; r < grid.GetLength(1); r++) {

                            voxelState = grid[i, r, c];
                            writer.Write(
                                i,
                                r,
                                voxelState == null ?
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.Black :
                                        Color.White :
                                    Definitions.VOXEL_CLASS_COLORS[
                                        voxelState
                                            .GetVoxelClassValues()
                                            .ToVisualizationVoxelClassValue()]);
                        }
                    }
                }
            }
        }

        public static void VisualizeVoxelClassificationPerRoomAsPLY(
                string file,
                int[,,][] grid) {

            VisualizeVoxelClassificationPerRoomAsPLY(
                file,
                grid,
                DEFAULT_VOXEL_MESHER);
        }

        public static void VisualizeVoxelClassificationPerRoomAsPLY(
                string file,
                int[,,][] grid,
                IVoxelMesher voxelMesher) {

            int i, r, c;
            int[] voxelState;
            int[] voxelClassValues;
            Dictionary<int, PLYWriter> writers = new Dictionary<int, PLYWriter>();

            for (i = 0; i < grid.GetLength(0); i++) {
                for (r = 0; r < grid.GetLength(1); r++) {
                    for (c = 0; c < grid.GetLength(2); c++) {

                        voxelState = grid[i, r, c];
                        if (voxelState == null) {
                            continue;
                        }

                        foreach (int roomId in voxelState.GetRoomIds()) {

                            voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                            if (voxelClassValues.Length == 1 &&
                                    voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)) {
                                continue;
                            }

                            if (!writers.ContainsKey(roomId)) {
                                writers.Add(
                                    roomId,
                                    file);
                            }

                            writers[roomId].Write(
                                i, 
                                r, 
                                c,
                                Definitions.VOXEL_CLASS_COLORS[
                                    voxelClassValues.ToVisualizationVoxelClassValue()],
                                voxelMesher);
                        }
                    }
                }
            }

            writers.Dispose();
        }

        public static void VisualizeSpacePartitioningAsPLY(
                string file,
                int[,,][] grid) {

            VisualizeSpacePartitioningAsPLY(
                file,
                grid,
                DEFAULT_VOXEL_MESHER);
        }

        public static void VisualizeSpacePartitioningAsPLY(
                string file,
                int[,,][] grid,
                IVoxelMesher voxelMesher) {

            int i, r, c;
            int[] voxelState;
            HashSet<int> roomIds;
            Dictionary<int, Color> colors;
            Dictionary<int, PLYWriter> writers = new Dictionary<int, PLYWriter>();

            roomIds = grid.GetRoomIds();
            colors = Color.GetRandomColorMapping(roomIds);

            for (i = 0; i < grid.GetLength(0); i++) {
                for (r = 0; r < grid.GetLength(1); r++) {
                    for (c = 0; c < grid.GetLength(2); c++) {

                        voxelState = grid[i, r, c];
                        if (voxelState == null) {
                            continue;
                        }

                        foreach (int roomId in voxelState.GetRoomIds()) {

                            if (DoAllNeighboursHaveSameRoomId(
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    grid)) {
                                continue;
                            }

                            if (!writers.ContainsKey(roomId)) {
                                writers.Add(
                                    roomId,
                                    file);
                            }

                            writers[roomId].Write(
                                i, 
                                r, 
                                c,
                                colors[roomId],
                                voxelMesher);
                        }
                    }
                }
            }

            writers.Dispose();
        }

        public static void VisualizeSpacePartitioningAsSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) { 

            HashSet<int> roomIds = grid
                .GetRoomIds()
                .Where(roomId => roomId > 0)
                .ToHashSet();

            Dictionary<int, Color> colors = Color.GetRandomColorMapping(roomIds);

            VisualizeSpacePartitioningAsISections(
                file,
                grid,
                backgroundColor,
                colors);
            VisualizeSpacePartitioningAsRSections(
                file,
                grid,
                backgroundColor,
                colors);
            VisualizeSpacePartitioningAsCSections(
                file,
                grid,
                backgroundColor,
                colors);
        }

        public static void VisualizeSpacePartitioningAsISections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) {

            HashSet<int> roomIds = grid
                .GetRoomIds()
                .Where(roomId => roomId > 0)
                .ToHashSet();
            
            Dictionary<int, Color> colors = Color.GetRandomColorMapping(roomIds);

            VisualizeSpacePartitioningAsISections(
                file,
                grid,
                backgroundColor,
                colors);
        }

        public static void VisualizeSpacePartitioningAsRSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) {

            HashSet<int> roomIds = grid
                .GetRoomIds()
                .Where(roomId => roomId > 0)
                .ToHashSet();

            Dictionary<int, Color> colors = Color.GetRandomColorMapping(roomIds);

            VisualizeSpacePartitioningAsRSections(
                file,
                grid,
                backgroundColor,
                colors); 
        }

        public static void VisualizeSpacePartitioningAsCSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor) {

            HashSet<int> roomIds = grid
                .GetRoomIds()
                .Where(roomId => roomId > 0)
                .ToHashSet();

            Dictionary<int, Color> colors = Color.GetRandomColorMapping(roomIds);

            VisualizeSpacePartitioningAsCSections(
                file,
                grid,
                backgroundColor,
                colors); 
        }

        public static void VisualizeWallNormalsAsPly(
                string file,
                int[,,][] reconstructionGrid) {

            VisualizeWallNormalsAsPly(
                file,
                reconstructionGrid,
                DEFAULT_VOXEL_MESHER);
        }

        public static void VisualizeWallNormalsAsPly(
                string file,
                int[,,][] reconstructionGrid,
                IVoxelMesher voxelMesher) {

            int i, r, c;
            int[] voxelState;
            int[] voxelClassValues;

            using (PLYWriter writer = new PLYWriter(file)) {

                for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                    for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                        for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                            voxelState = reconstructionGrid[i, r, c];
                            if (voxelState == null) {
                                continue;
                            }

                            voxelClassValues = voxelState.GetVoxelClassValues();
                            if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                                    && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                                continue;
                            }

                            writer.Write(
                                i,
                                r,
                                c,
                                voxelState
                                    .GetNormals()
                                    .GetWallNormalsColor(),
                                voxelMesher);
                        }
                    }
                }
            }
        }

        private static bool DoAllNeighboursHaveSameRoomId(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] grid) {

            int di, i2, dr, r2, dc, c2;
            int[] voxelState;

            for (di = -1; di <= 1; di++) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (di.Abs() + dr.Abs() + dc.Abs() != 1) {
                            continue;
                        }

                        i2 = i + di;
                        r2 = r + dr;
                        c2 = c + dc;
                        if (i2 < 0 || r2 < 0 || c2 < 0
                                || i2 >= grid.GetLength(0)
                                || r2 >= grid.GetLength(1)
                                || c2 >= grid.GetLength(2)) {
                            continue;
                        }

                        voxelState = grid[i2, r2, c2];
                        if (voxelState == null
                                || !voxelState.HasRoomId(roomId)) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        private static void VisualizeSpacePartitioningAsISections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor,
                Dictionary<int, Color> colors) {

            int i, r, c;
            int[] roomIds;
            int[] voxelState;

            for (i = 0; i < grid.GetLength(0); i++) {

                using (ImageWriter writer = new ImageWriter(
                        grid.GetLength(1),
                        grid.GetLength(2),
                        FileSystemUtils.GetFileWithPostfix(
                            file,
                            $"_i{i}"))) {

                    for (r = 0; r < grid.GetLength(1); r++) {
                        for (c = 0; c < grid.GetLength(2); c++) {

                            voxelState = grid[i, r, c];
                            if (voxelState == null) {
                                writer.Write(
                                    r,
                                    c,
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.Black :
                                        Color.White);
                                continue;
                            }

                            roomIds = voxelState.GetRoomIds();
                            if (roomIds.Any(roomId => roomId <= 0)) {
                                writer.Write(
                                    r,
                                    c,
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.White :
                                        Color.Black);
                                continue;
                            }

                            writer.Write(
                                r,
                                c,
                                colors[roomIds.First()]);
                        }
                    }
                }
            }
        }

        private static void VisualizeSpacePartitioningAsRSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor,
                Dictionary<int, Color> colors) {

            int i, r, c;
            int[] roomIds;
            int[] voxelState;

            for (r = 0; r < grid.GetLength(1); r++) {

                using (ImageWriter writer = new ImageWriter(
                        grid.GetLength(0),
                        grid.GetLength(2),
                        FileSystemUtils.GetFileWithPostfix(
                            file,
                            $"_r{r}"))) {

                    for (i = 0; i < grid.GetLength(0); i++) {
                        for (c = 0; c < grid.GetLength(2); c++) {

                            voxelState = grid[i, r, c];
                            if (voxelState == null) {
                                writer.Write(
                                    i,
                                    c,
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.Black :
                                        Color.White);
                                continue;
                            }

                            roomIds = voxelState.GetRoomIds();
                            if (roomIds.Any(roomId => roomId < 0)) {
                                writer.Write(
                                    i,
                                    c,
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.White :
                                        Color.Black);
                                continue;
                            }

                            writer.Write(
                                i,
                                c,
                                colors[roomIds.First()]);
                        }
                    }
                }
            }
        }

        private static void VisualizeSpacePartitioningAsCSections(
                string file,
                int[,,][] grid,
                BackgroundColor backgroundColor,
                Dictionary<int, Color> colors) {

            int i, r, c;
            int[] roomIds;
            int[] voxelState;

            for (c = 0; c < grid.GetLength(2); c++) {

                using (ImageWriter writer = new ImageWriter(
                        grid.GetLength(0),
                        grid.GetLength(1),
                        FileSystemUtils.GetFileWithPostfix(
                            file,
                            $"_c{c}"))) {

                    for (i = 0; i < grid.GetLength(0); i++) {
                        for (r = 0; r < grid.GetLength(1); r++) {

                            voxelState = grid[i, r, c];
                            if (voxelState == null) {
                                writer.Write(
                                    i,
                                    r,
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.Black :
                                        Color.White);
                                continue;
                            }

                            roomIds = voxelState.GetRoomIds();
                            if (roomIds.Any(roomId => roomId < 0)) {
                                writer.Write(
                                    i,
                                    r,
                                    backgroundColor == BackgroundColor.BLACK ?
                                        Color.White :
                                        Color.Black);
                                continue;
                            }

                            writer.Write(
                                i,
                                r,
                                colors[roomIds.First()]);
                        }
                    }
                }
            }
        }

        private static void Add(
                this Dictionary<int, PLYWriter> writers,
                int roomId,
                string file) {

            writers.Add(
                roomId,
                new PLYWriter(
                    FileSystemUtils.GetFileWithPostfix(
                        file,
                        roomId > 0 ?
                            $"R{roomId}" :
                            $"T{roomId.Abs()}")));
        }

        private static void Dispose(
                this Dictionary<int, PLYWriter> writers) {

            foreach (PLYWriter writer in writers.Values) {
                writer.Dispose();
            }
        }

        private static HashSet<int> GetRoomIds(
                this int[,,][] grid) { 

            object @lock = new object();
            HashSet<int> roomIds = new HashSet<int>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    grid.GetLength(0)),
                () => new HashSet<int>(),
                (partition, loopState, localRoomIds) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {
                        UpdateRoomIds(
                            i,
                            grid,
                            localRoomIds);
                    }

                    return localRoomIds;
                },
                localRoomIds => {
                    lock (@lock) {
                        roomIds.AddRange(localRoomIds);
                    }
                });

            return roomIds;
        }

        private static void UpdateRoomIds(
                int i,
                int[,,][] grid,
                HashSet<int> roomIds) { 

            int r, c;
            int[] voxelState;

            for (r = 0; r < grid.GetLength(1); r++) {
                for (c = 0; c < grid.GetLength(2); c++) {

                    voxelState = grid[i, r, c];

                    if (voxelState != null) {
                        roomIds.AddRange(
                            voxelState.GetRoomIds());
                    }
                }
            }
        }
    }
}
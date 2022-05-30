using HuePat.VoxIR.Util.Geometry;
using HuePat.VoxIR.Util.Grid;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Evaluation.ISPRS {
    static class GroundTruthVoxelizer {
        public static int[,,][] Voxelize(
                double resolution,
                (int, int, int) gridSize,
                Vector3d offset,
                Dictionary<int, List<Mesh>> groundTruth) {

            int[,,][] groundTruthGrid = new int[
                gridSize.Item1,
                gridSize.Item2,
                gridSize.Item3][];

            foreach (int groundTruthClassValue in groundTruth.Keys) {

                foreach (Mesh mesh in groundTruth[groundTruthClassValue]) {

                    Voxelizer.VoxelizeMesh(
                        resolution,
                        groundTruthGrid,
                        offset,
                        mesh,
                        (triangle, bBox) => new int[] { groundTruthClassValue },
                        (voxelState, voxelState2) => {

                            HashSet<int> newVoxelState = new HashSet<int>();

                            if (voxelState != null) {
                                newVoxelState.AddRange(voxelState);
                            }

                            newVoxelState.AddRange(voxelState2);

                            return newVoxelState.ToArray();
                        });
                }
            }

            Postprocess(groundTruthGrid);

            return groundTruthGrid;
        }

        private static void Postprocess(
                int[,,][] groundTruthGrid) {

            bool[,,] horizontalWallOpeningSurfaceGrid = new bool[
                groundTruthGrid.GetLength(0),
                groundTruthGrid.GetLength(1),
                groundTruthGrid.GetLength(2)];

            Parallel.For(
                0,
                groundTruthGrid.GetLength(1),
                r => {

                    ApplyHorizontalSurfaceModifiers(
                        r,
                        groundTruthGrid);

                    CreateInterior(
                         r,
                         horizontalWallOpeningSurfaceGrid,
                         groundTruthGrid);

                    RemoveOuterHorizontalSurfaces(
                        r,
                        groundTruthGrid);
                });

            Parallel.For(
                0,
                groundTruthGrid.GetLength(0),
                i => {

                    RemoveOuterVerticalSurfaces(
                        i,
                        horizontalWallOpeningSurfaceGrid,
                        groundTruthGrid);

                    Convert(
                        i,
                        groundTruthGrid);
                });
        }

        private static void ApplyHorizontalSurfaceModifiers(
                int r,
                int[,,][] groundTruthGrid) {

            bool isActive = false;
            int i, i2, i3, c;
            int[] voxelState;
            HashSet<int> newVoxelState;

            i2 = 0;

            for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                for (i = 0; i < groundTruthGrid.GetLength(0); i++) {

                    voxelState = groundTruthGrid[i, r, c];

                    if (isActive) {

                        if (voxelState != null
                                && voxelState.Contains(GroundTruthClassValues.NOT_FLOOR)) {

                            isActive = false;

                            for (i3 = i2; i3 <= i; i3++) {

                                newVoxelState = new HashSet<int>();

                                if (groundTruthGrid[i3, r, c] != null) {

                                    newVoxelState.AddRange(
                                        groundTruthGrid[i3, r, c]);
                                }

                                newVoxelState.Add(GroundTruthClassValues.NOT_CEILING);

                                groundTruthGrid[i3, r, c] = newVoxelState.ToArray();
                            }
                        }
                    }
                    else if (voxelState != null
                            && voxelState.Contains(GroundTruthClassValues.NOT_CEILING)) {

                        isActive = true;
                        i2 = i + 1;
                    }
                }

                for (i = 0; i < groundTruthGrid.GetLength(0); i++) {

                    voxelState = groundTruthGrid[i, r, c];

                    if (voxelState != null
                            && (voxelState.Contains(GroundTruthClassValues.NOT_CEILING)
                                || voxelState.Contains(GroundTruthClassValues.NOT_FLOOR)
                            && !voxelState.Contains(GroundTruthClassValues.WALL))) {

                        groundTruthGrid[i, r, c] = null;
                    }

                    if (voxelState != null
                            && voxelState.Contains(GroundTruthClassValues.OPENING_CEILING)
                            && !voxelState.Contains(GroundTruthClassValues.CEILING)) {

                        newVoxelState = new HashSet<int>();
                        newVoxelState.AddRange(voxelState);
                        newVoxelState.Add(GroundTruthClassValues.CEILING);
                        groundTruthGrid[i, r, c] = newVoxelState.ToArray();
                    }
                }
            }
        }

        private static void CreateInterior(
                int r,
                bool[,,] horizontalWallOpeningSurfaceGrid,
                int[,,][] groundTruthGrid) {

            bool isInCeiling = false;
            bool isInInterior = false;
            bool isInWall = false;
            int i, i2, i3, c;
            int[] voxelState;
            HashSet<int> newVoxelState;

            i2 = 0;

            for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                for (i = 0; i < groundTruthGrid.GetLength(0); i++) {

                    voxelState = groundTruthGrid[i, r, c];

                    if (!isInCeiling
                            && !isInInterior) {

                        if (voxelState == null) {
                            continue;
                        }

                        if (voxelState.Contains(GroundTruthClassValues.CEILING)) {

                            if (voxelState.Contains(GroundTruthClassValues.WALL)) {
                                isInWall = true;
                            }
                            else if (!isInWall) {
                                isInCeiling = true;
                            }
                        }

                        if (isInWall
                                && voxelState.Contains(GroundTruthClassValues.FLOOR)) {

                            isInWall = false;
                        }
                    }
                    else if (isInCeiling) {

                        if (voxelState == null) {

                            isInInterior = true;
                            isInCeiling = false;
                            i2 = i;
                        }
                        else if (!voxelState.Contains(GroundTruthClassValues.CEILING)) {

                            isInCeiling = false;
                        }
                    }
                    else if (isInInterior) {

                        if (voxelState == null) {

                            continue;
                        }
                        else if (voxelState.Contains(GroundTruthClassValues.FLOOR)) {

                            isInInterior = false;

                            if (voxelState.Contains(GroundTruthClassValues.CEILING)) {
                                isInCeiling = true;
                            }

                            for (i3 = i2; i3 < i; i3++) {

                                groundTruthGrid[i3, r, c] = new int[] {
                                    GroundTruthClassValues.INTERIOR
                                };
                            }
                        }
                        else {
                            isInCeiling = voxelState.Contains(GroundTruthClassValues.CEILING);
                            isInInterior = false;
                        }
                    }
                }

                for (i = 0; i < groundTruthGrid.GetLength(0); i++) {

                    voxelState = groundTruthGrid[i, r, c];

                    if (!isInInterior) {

                        if (voxelState != null
                                && voxelState.Contains(GroundTruthClassValues.OPENING_CEILING)) {

                            isInInterior = true;
                            i2 = i + 1;
                            newVoxelState = new HashSet<int>();
                            newVoxelState.AddRange(voxelState);
                            newVoxelState.Add(GroundTruthClassValues.CEILING);
                            groundTruthGrid[i, r, c] = newVoxelState.ToArray();
                            horizontalWallOpeningSurfaceGrid[i, r, c] = true;
                        }
                    }
                    else {

                        if (voxelState == null) {
                            continue;
                        }

                        if (voxelState != null
                                && (voxelState.Contains(GroundTruthClassValues.FLOOR)
                                    || (voxelState.Contains(GroundTruthClassValues.WALL)
                                        && !voxelState.Contains(GroundTruthClassValues.OPENING_CEILING)))) {

                            isInInterior = false;

                            for (i3 = i2; i3 < i; i3++) {

                                if (groundTruthGrid[i3, r, c] == null) {

                                    groundTruthGrid[i3, r, c] = new int[] {
                                        GroundTruthClassValues.INTERIOR
                                    };
                                }
                            }

                            if (!voxelState.Contains(GroundTruthClassValues.FLOOR)) {
                                newVoxelState = new HashSet<int>();
                                newVoxelState.AddRange(voxelState);
                                newVoxelState.Add(GroundTruthClassValues.FLOOR);
                                groundTruthGrid[i, r, c] = newVoxelState.ToArray();
                            }

                            horizontalWallOpeningSurfaceGrid[i, r, c] = true;
                        }
                    }
                }
            }
        }

        private static void RemoveOuterHorizontalSurfaces(
                int r,
                int[,,][] groundTruthGrid) {

            int i, i2, c;
            int[] voxelState, voxelState2;

            for (c = 0; c < groundTruthGrid.GetLength(2); c++) {
                for (i = 0; i < groundTruthGrid.GetLength(0); i++) {

                    voxelState = groundTruthGrid[i, r, c];

                    if (voxelState == null
                            || voxelState.Contains(GroundTruthClassValues.INTERIOR)) {

                        continue;
                    }

                    if (voxelState.Contains(GroundTruthClassValues.CEILING)
                            && !voxelState.Contains(GroundTruthClassValues.WALL)) {

                        i2 = i + 1;

                        if (i2 >= groundTruthGrid.GetLength(0)) {
                            groundTruthGrid[i, r, c] = null;
                        }
                        else {
                            voxelState2 = groundTruthGrid[i2, r, c];

                            if (voxelState2 == null
                                    || !voxelState2.Contains(GroundTruthClassValues.INTERIOR)) {

                                groundTruthGrid[i, r, c] = null;
                            }
                        }
                    }

                    if (voxelState.Contains(GroundTruthClassValues.FLOOR)
                            && !voxelState.Contains(GroundTruthClassValues.WALL)) {

                        i2 = i - 1;

                        if (i2 < 0) {
                            groundTruthGrid[i, r, c] = null;
                        }
                        else {
                            voxelState2 = groundTruthGrid[i2, r, c];

                            if (voxelState2 == null
                                    || !voxelState2.Contains(GroundTruthClassValues.INTERIOR)) {

                                groundTruthGrid[i, r, c] = null;
                            }
                        }
                    }
                }
            }
        }

        private static void RemoveOuterVerticalSurfaces(
                int i,
                bool[,,] horizontalWallOpeningSurfaceGrid,
                int[,,][] groundTruthGrid) {

            int r, c;
            int[] voxelState;

            for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                    voxelState = groundTruthGrid[i, r, c];

                    if (voxelState == null
                            || horizontalWallOpeningSurfaceGrid[i, r, c]
                            || !voxelState.Contains(GroundTruthClassValues.WALL)) {
                        continue;
                    }

                    if (!NeighboursInterior(
                            i,
                            r,
                            c,
                            groundTruthGrid)) {

                        groundTruthGrid[i, r, c] = null;
                    }
                }
            }
        }

        private static bool NeighboursInterior(
                int i,
                int r,
                int c,
                int[,,][] groundTruthGrid) {

            int dr, r2, dc, c2;
            int[] voxelState;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    r2 = r + dr;
                    c2 = c + dc;

                    if (r2 < 0 || c2 < 0
                            || r2 >= groundTruthGrid.GetLength(1)
                            || c2 >= groundTruthGrid.GetLength(2)) {
                        continue;
                    }

                    voxelState = groundTruthGrid[i, r2, c2];

                    if (voxelState != null
                            && (voxelState.Contains(GroundTruthClassValues.INTERIOR)
                                || ((voxelState.Contains(GroundTruthClassValues.CEILING)
                                        || voxelState.Contains(GroundTruthClassValues.FLOOR))
                                    && !voxelState.Contains(GroundTruthClassValues.WALL)))) {

                        return true;
                    }
                }
            }

            return false;
        }

        private static void Convert(
                int i,
                int[,,][] groundTruthGrid) {

            int r, c;
            int[] voxelClassValues;

            for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                    if (groundTruthGrid[i, r, c] != null) {

                        voxelClassValues = Convert(groundTruthGrid[i, r, c])
                            .Where(voxelClassValue => voxelClassValue != VoxelClassValues.EMPTY_INTERIOR)
                            .ToArray();

                        groundTruthGrid[i, r, c] = voxelClassValues.Length == 0 ?
                            null :
                            VoxelState.CreateVoxelState(
                                0,
                                voxelClassValues);
                    }
                }
            }
        }

        private static int[] Convert(
                int[] voxelState) {

            return voxelState
                .Where(groundTruthClassValue => groundTruthClassValue != GroundTruthClassValues.NOT_CEILING
                    && groundTruthClassValue != GroundTruthClassValues.NOT_FLOOR
                    && groundTruthClassValue != GroundTruthClassValues.OPENING_CEILING)
                .Select(groundTruthClassValue => {

                    switch (groundTruthClassValue) {

                        case GroundTruthClassValues.CEILING:
                            return VoxelClassValues.CEILING;

                        case GroundTruthClassValues.FLOOR:
                            return VoxelClassValues.FLOOR;

                        case GroundTruthClassValues.WALL:
                            return VoxelClassValues.WALL;

                        case GroundTruthClassValues.INTERIOR:
                            return VoxelClassValues.EMPTY_INTERIOR;

                        default:
                            throw new ApplicationException();
                    }
                })
                .ToArray();
        }
    }
}
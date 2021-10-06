using System.IO;

namespace HuePat.VoxIR.IO.Binary {
    public static class BinaryWriter {
        public static void ExportBinary(
                string file,
                int[,,][] reconstructionGrid) {

            int i, r, c, j;
            int[] voxelState;

            using (System.IO.BinaryWriter writer = new System.IO.BinaryWriter(
                    File.Create(file))) {

                writer.Write(reconstructionGrid.GetLength(0));
                writer.Write(reconstructionGrid.GetLength(1));
                writer.Write(reconstructionGrid.GetLength(2));

                for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                    for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                        for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                            voxelState = reconstructionGrid[i, r, c];
                            if (voxelState == null) {
                                writer.Write(0);
                                continue;
                            }

                            writer.Write(voxelState.Length);

                            for (j = 0; j < voxelState.Length; j++) {
                                writer.Write(voxelState[j]);
                            }
                        }
                    }
                }
            }
        }
    }
}
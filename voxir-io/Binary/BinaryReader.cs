using System.IO;

namespace HuePat.VoxIR.IO.Binary {
    public static class BinaryReader {
        public static int[,,][] ReadReconstructionGrid(
                string file) {

            int i, r, c, j;
            int arraySize;
            int[,,][] reconstructionGrid;

            using (System.IO.BinaryReader reader = new System.IO.BinaryReader(
                    File.Open(
                        file, 
                        FileMode.Open))) {

                i = r = c = 0;
                reconstructionGrid = new int[
                    reader.ReadInt32(),
                    reader.ReadInt32(),
                    reader.ReadInt32()][];
                
                do {

                    arraySize = reader.ReadInt32();

                    if (arraySize > 0) {
                        reconstructionGrid[i, r, c] = new int[arraySize];
                    }

                    for (j = 0; j < arraySize; j++) {
                        reconstructionGrid[i, r, c][j] = reader.ReadInt32();
                    }

                    c++;
                    if (c == reconstructionGrid.GetLength(2)) {
                        c = 0;
                        r++;
                    }

                    if (r == reconstructionGrid.GetLength(1)) {
                        r = 0;
                        i++;
                    }

                    if (i == reconstructionGrid.GetLength(0)) {
                        break;
                    }
                } while (true);
            }

            return reconstructionGrid;
        }
    }
}
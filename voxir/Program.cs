using HuePat.VoxIR.IO;
using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.IO.PLY.Writing;
using HuePat.VoxIR.IO.Visualization;
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System.Collections.Generic;

namespace HuePat.VoxIR {
    public static class Program {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // VoxIR: Voxel-Based Indoor Reconstruction
        //-----------------------------------------
        //
        // This solution contains the code for the publication: 
        //
        //      Hübner, P.; Weinmann, M.; Wursthorn, S. & Hinz, S. (2021).
        //      Automatic Voxel-based 3D Indoor Reconstruction and Room Partitioning from Triangle Meshes.
        //      ISPRS Journal of Photogrammetry and Remote Sensing, 2021, 181, 254-278
        //      https://www.sciencedirect.com/science/article/abs/pii/S0924271621001799
        //
        // This Console Project only contains a simple executable example to demonstrate, how VoxIR is used.
        // For more detailed documentation, see the ReadMe of the repository:
        //      https://github.com/huepat/voxir
        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        public static void Main(string[] args) {

            // Create a mesh (simple demo scenario of two boxes):
            Mesh mesh = Mesh.Merge(new Mesh[] {
                new AABox(
                    new Vector3d(0.0, 0.0, 0.0),
                    new Vector3d(5.0, 10.0, 2.5)).Mesh,
                new AABox(
                    new Vector3d(5.0, 0.0, 0.0),
                    new Vector3d(15.0, 5.0, 2.5)).Mesh
            });

            // Note: You can load triangle meshes with the PLYReader (for more information, see the ReadMe of the repository):
            // mesh = new PLYReader().ReadMesh("path/to/MeshFile.ply");

            // Export the created demo 'house' to a PLY file
            // (you can find the created output in 'path/to/voxir/voxir/bin/Debug/net5.0' or 'path/to/voxir/voxir/bin/Release/net5.0')
            using (PLYWriter writer = new PLYWriter("./demo_mesh.ply")) {
                writer.Write(
                    mesh,
                    Color.LightGray);
            }

            // We have created the demo mesh with the z-axis pointing upwards.
            // VoxIR however expects that the y-axis is pointing downwards with respect to the building geometry
            // (as is the case with the VoxIR datasets published alongside this code).
            // We thus have to rotate the created mesh accordingly (rotate 90° around the x-axis):
            mesh.Rotate(
                90.0.DegreeToRadian(),
                new Vector3d(1.0, 0.0, 0.0),
                mesh.GetCentroid());

            // process the demo mesh with VoxIR:
            int[,,][] grid = HuePat.VoxIR.VoxIR.Process(
                0.05, // 5 cm voxel resolution
                null, // determine grid extent from mesh bounding box
                mesh,
                out HashSet<int> rampSpaceIds);

            // Export a visualization of the semantic voxel classes as PLY file (see ReadMe for further information)
            Visualizer.VisualizeVoxelClassificationAsPLY(
                "./demo_grid_voxelClasses.ply",
                grid);

            // Export a visualization of the space partitioning as PLY file (see ReadMe for further information)
            Visualizer.VisualizeSpacePartitioningAsPLY(
                "./demo_grid_spacePartitioning.ply",
                grid);
        }
    }
}
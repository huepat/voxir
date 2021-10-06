using HuePat.VoxIR.Util.Grid;
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
        // For the VoxIR datasets published along with the paper and the code, see the ReadMe of the repository:
        //      https://github.com/huepat/voxir
        //
        // This solution contains multiple class library projects:
        //    - voxir-lib: The actual voxel-based indoor reconstruction procedure
        //                 outputting a voxel grid of type int[,,][] for Input of Type HuePat.VoxIR.Util.Geometry.Mesh
        //    - voxir-definitions: global definitions used in all the other class libraries
        //    - voxir-util: utility methods used in the other class libraries for
        //                  1) Handling of triangle meshes (HuePat.VoxIR.Util.Geometry)
        //                  2) Handling of the reconstruction voxel grid (HuePat.VoxIR.Util.Grid)
        //    - voxir-io: IO methods for
        //                1) Reading input triangle meshes (as PLY)
        //                2) Visualizing triangle meshes and reconstruction results (as PLY or images)
        //    - voxir-datasets: Utility methods for handling the VoxIR datasets published along with the paper
        //    - voxir-evaluation: Methods for applying the evaluation procedure published in the paper
        //                        to the VoxIR datasets
        //
        // This Console project (voxir) is only an example of how to use the functions provided by the class library projects
        // (see commented examples in main method)
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        public static void Main(string[] args) {

            ////////////////////////////////
            // Example of how to use VoxIR:
            ////////////////////////////////

            // Create a PLYReader (can read ASCII as well as binary PLY files)
            HuePat.VoxIR.IO.PLY.Reading.PLYReader reader = new HuePat.VoxIR.IO.PLY.Reading.PLYReader() {

                // Options (listed with default values):

                // Normal vectors of triangles should be pointing towards the building interior
                // Use this to invert them, in case they are pointing consistently towards the outside in your dataset
                InvertNormals = false,

                // Set to false, if the vertex coordinates in your file are double values
                // (Only relevant for reading binary files)
                AreVertexCoordinatesFloat = true,

                // Indicates the number of bytes a linebreak is encoded in your binary PLY file
                // (Only relevant for reading binary files)
                LineBreakByteSize = 1,

                // The identifier used for designating the vertex coordinates in your PLY file header
                // (Note: The PLY file can contain an arbitrary number of other attributes besides the
                //  vertex coordinates without impairing the reading process)
                CoordinateIndentifiers = ("x", "y", "z")
            };

            // Load a triangle mesh
            HuePat.VoxIR.Util.Geometry.Mesh mesh = reader.ReadMesh("path/to/MeshFile.ply");

            // Note: VoxIR expects triangle meshes to be in a coordinate system where the Y-Axis points downwards
            // (with respect to the represented building geometry)
            // The X and Z axis are horizontal and do not need to be aligned with the building geometry in a Manhattan-World way
            // If your triangle mesh does not conform to this coordinate system definition, you need to rotate it before applying VoxIR
            // (See the implementation of the Matterport3DReader for an example of how to do this)
            mesh = HuePat.VoxIR.IO.Matterport3D.Matterport3DReader.LoadHouse("path/to/Matterport3DHouse.ply");

            // Apply VoxIR
            int[,,][] grid = HuePat.VoxIR.VoxIR.Process(
                0.05, // Resolution of the voxel grid [m]
                new Util.Geometry.AABox( // Extend of the voxel grid (Pass null to fit grid to mesh BBox, otherwise ensure the grid extent is larger than the mesh extent)
                    new OpenTK.Mathematics.Vector3d(-10.0), // Min grid coordinate [m]
                    new OpenTK.Mathematics.Vector3d(+10.0)), // Min grid coordinate [m]
                mesh,
                out HashSet<int> rampSpaceIds); // Outputs the room ids belonging to ramp spaces (see paper for definition of ramp space)



            /////////////////////////////////////////
            // Example of how to access voxel states:
            /////////////////////////////////////////

            // The grid coordinate system is denoted by (i, r, c) within the VoxIR code
            //  and corresponds to (y, x, z) in the metric coordinate system (see comment on mesh reading above)
            // I.e. i points downwards and r and c are horizontal grid axes
            int i = 0;
            int r = 0;
            int c = 0;

            // Access a voxel state by its grid coordinate
            int[] voxelState = grid[i, r, c];

            // A voxel is empty if its voxel state is null
            bool isEmpty = voxelState == null;

            // If a voxel is not empty, you can query its room ids
            // (Extension methods in HuePat.VoxIR.Util.Grid.VoxelState)
            int[] roomIds = voxelState.GetRoomIds();

            // Note: Negative room ids signalize transition spaces,
            //  positive room ids signify rooms
            // (Ramp spaces as a special type of room are listed in the output parameter rampSpaceIds (See above))
            // Resulting room ids are consecutive (1, 2, 3, ...) as initial rooms get merged during space partitioning
            // Furthermore, room ids are not identical between multiple applications of VoxIR due to parallelized operations
            // (The space partitioning results are however always the same, only the room ids can change)

            // Query the number of rooms a voxel belongs to
            int roomCount = voxelState.GetRoomCount();

            // Query if a voxel belongs to a certain room
            voxelState.HasRoomId(42);

            // Get voxel class values for a certain room id
            int[] voxelClassValues = voxelState.GetVoxelClassValues(roomIds[0]);

            // Note: Voxel class values can be of values:
            //      CEILING = 0
            //      EMPTY_INTERIOR = 1
            //      INTERIOR_OBJECT = 2
            //      FLOOR = 3
            //      WALL = 4
            //      WALL_OPENING = 5
            // A voxel can have multiple voxel class values from {CEILING, FLOOR, WALL} for one room id

            // Get all voxel class values from all room ids
            voxelClassValues = voxelClassValues.GetVoxelClassValues();



            ///////////////////////////////////////////////////////////////////
            // Example of how to save and load voxel grids to/from binary files
            ///////////////////////////////////////////////////////////////////

            // Save the voxel grid to a binary file
            HuePat.VoxIR.IO.Binary.BinaryWriter.ExportBinary(
                "path/to/file.bin",
                grid);

            // Load the voxel grid from a binary file
            HuePat.VoxIR.IO.Binary.BinaryReader.ReadReconstructionGrid("path/to/file.bin");



            //////////////////////////////////////////
            // Example of how to visualize voxel grids
            //////////////////////////////////////////

            // Visualize voxel classification as PLY
            // Colors: Yellow: voxel has no voxel class value although it has room ids (should not occur in final voxel grid)
            //         Red: CEILING
            //         Light Red: CEILING and WALL
            //         Green: FLOOR
            //         Light Green: FLOOR and WALL
            //         Orange: FLOOR and CEILING
            //         Dark Gray: INTERIOR_OBJECT
            //         Light Gray: WALL
            //         BLUE: WALL_OPENING
            // (Colors refer to voxel class values aggregated for all room ids per voxel)
            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeVoxelClassificationAsPLY(
                "path/to/file.ply",
                grid);

            // Note: The method above used HuePat.VoxIR.IO.Visualization.GridFrameVoxelMesher which 
            //  generates cubes of 1m^3 centered at grid coordinates (r, c, -i) in meter
            //  (This causes the z-Axis to point upwards with respect to the building geometry
            //  which is well-suited for viewing in prevalent PLY viewers)

            // If you want to overlay the grid visualization with the input triangle mesh, use:
            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeVoxelClassificationAsPLY(
                "path/to/file.ply",
                grid,
                new HuePat.VoxIR.IO.Visualization.MeshFrameVoxelMesher(
                    0.05, // Use same voxel resolution [m] as used when creating the voxel grid
                    mesh));

            // Visualize voxel classification as image slizes
            //  (Here for i-sections; similarly, methods for r and c sections exist)
            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeSpacePartitioningAsISections(
                "path/to/file.png", // All image file types supported by OpenCV can be used (Path is extended by "_iXXX" for XXX = slize number)
                grid,
                HuePat.VoxIR.IO.Visualization.BackgroundColor.BLACK); // Also possible: WHITE

            // Analogous to the methods presented above, HuePat.VoxIR.IO.Visualization.Visualizer
            //  has further visualization methods (VisualizeSpacePartitioning...) that visualize
            //  different rooms in different colors with black/white for transition spaces (depending on background color)



            ////////////////////////////////////////////////////////////////
            // Example of how to use the provided datasets with ground truth
            ////////////////////////////////////////////////////////////////

            // VoxIR is published together with four triangle meshes of different indoor environments captured with the Microsoft HoloLens (version 1)
            //  as described in the paper.
            // The VoxIR datasets are published together with manually generated ground truth data
            //  in the form of mesh segments per room and per semantic class (CEILING, FLOOR, WALL, WALL_OPENING, INTERIOR_OBJECT)
            //  as individual PLY files.
            // These segments are not clompletely identical to the unmodified datasets, as holes in room surfaces due to occlusion or inclompete acquisition
            //  were manually closed in the ground truth data.
            // This manual closing of holes was done by merging vertices on opposing sides of hole borders.
            // Thus, the ground truth mesh segment can also deviate from the mesh of the unmodified datasets in the vicinity of holes
            //  due to changed mesh topology and triangle geometry.
            // While this is not a problem for the voxel-based evaluation of VoxIR, scenarios are conceivable, where it might be desirable to
            //  transfer the ground truth information to the triangle meshes of the original datasets.
            // To this aim, suitable methods are provided which are described in the following.

            // Read ground truth mesh segments
            Dictionary<int, List<(HuePat.VoxIR.Util.Geometry.Mesh, HuePat.VoxIR.Datasets.GroundTruthInfo)>> groundTruthMeshSegments;
            groundTruthMeshSegments = HuePat.VoxIR.Datasets.GroundTruthReader.LoadGroundTruthMeshes("path/to/VoxIR_Datasets/GroundTruth/Office");

            // This results in a Dictionary, where the keys are the room ids (negative for transition spaces and positive for rooms)
            //  and the values are List of tuples of the mesh segments and GroundTruthInfos.
            // The GroundTruthInfos have the following properties:
            HuePat.VoxIR.Datasets.GroundTruthInfo groundTruthInfo = groundTruthMeshSegments[42][0].Item2;
            int roomId = groundTruthInfo.RoomId; // The room id (same as the key of the dictionary)
            int voxelClassValue = groundTruthInfo.ClassValue; // The voxel class value (CEILING, FLOOR, WALL, WALL_OPENING, INTERIOR_OBJECT)
            bool isRampSpace = groundTruthInfo.IsRampSpace; // Indicates, whether the respective room is a ramp space

            // As mentioned above, the ground truth mesh segments can deviate from the triangle mesh of the original dataset.
            // In order to transfer the ground truth data to the original triangle mesh, you can use the following methods:
            HuePat.VoxIR.Datasets.GroundTruth.TransferGroundTruthInfoToTestMesh(
                mesh, // The triangle mesh of the original dataset
                groundTruthMeshSegments, // The ground truth mesh segments
                out bool[] isRampSpacePerFace, // Array indicating per face index of the original triangle mesh, if the respective triangle belongs to a ramp space
                out int[] classValuePerFace, // Array of class values per face index of the original triangle mesh
                out int[] roomIdPerFace); // Array of room ids per face index of the original triangle mesh

            // The ground truth data transfered to the original triangle mesh can be visualizes via the following methods:

            // Visualizes ramp space triangles in green and other triangles in red
            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeMeshRampSpacesAsPLY(
                "path/to/file.ply",
                isRampSpacePerFace,
                mesh);

            // Visualizes the semantic class values of the triangles
            // Colors: Yellow: no class value (this can happen for noise triangles outside the windows/doors which are removed from the ground truth data)
            //         Red: CEILING
            //         Green: FLOOR
            //         Dark Gray: INTERIOR_OBJECT
            //         Light Gray: WALL
            //         BLUE: WALL_OPENING
            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeMeshClassificationAsPLY(
                "path/to/file.ply",
                classValuePerFace,
                mesh);

            // Visualizes the room ids of the triangles
            // Different room ids are visualized in different colors
            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeMeshSpacePartitioningAsPLY(
                "path/to/file.ply",
                roomIdPerFace,
                mesh);

            // Create a voxel grid from the ground truth mesh segments
            int[,,][] groundTruthGrid = HuePat.VoxIR.Datasets.GroundTruth.CreateGroundTruthGrid(
                0.05, // Resolution of the voxel grid [m]
                new Util.Geometry.AABox( // Extend of the voxel grid (Pass null to fit grid to mesh BBox, otherwise ensure the grid extent is larger than the mesh extent)
                    new OpenTK.Mathematics.Vector3d(-10.0), // Min grid coordinate [m]
                    new OpenTK.Mathematics.Vector3d(+10.0)), // Min grid coordinate [m]
                groundTruthMeshSegments); // The ground truth mesh segments

            // the ground truth voxel grid can be visualized with the same methods as demonstrated above for the voxel grid resulting from VoxIR



            ///////////////////////////////////////////////////////////////////////////////////////
            // Example of how to conduct an evaluation of VoxIR against the ground truth voxel grid
            ///////////////////////////////////////////////////////////////////////////////////////

            // All available evaluation methods require a parameter of type OutputConfig which allows to configure
            //  what kind of additional information should be outputted
            HuePat.VoxIR.Evaluation.IO.OutputConfig outputConfig = new Evaluation.IO.OutputConfig("path/to/OutputDirectory") {

                // Options (listed with default values):

                // Exports more detailed logs as txt files
                ExportExtendedLogs = false,

                // Exports visualizations as PLY
                ExportPLY = false,

                // Exports visualizations as images
                ExportImages = false,

                // Image file type in case ExportImages is set to true
                // (all image file types supported by OpenCV can be used)
                OutputImageFileType = "png",

                // Background color in case ExportImages is set to true
                BackgroundColor = HuePat.VoxIR.IO.Visualization.BackgroundColor.BLACK, // other option: WHITE

                // VoxelMesher in case ExportPLY is true
                // (GridFrameVoxelMesher, MeshFrameVoxelMesher (see above) or own implementations)
                VoxelMesher = new HuePat.VoxIR.IO.Visualization.GridFrameVoxelMesher()
            };

            // The evaluation can be applied to a single dataset and corresponding ground truth directory:
            // (The extend of the voxel grids are determined as the minimal BBox encompassing both, the test mesh and the ground truth meshes)
            HuePat.VoxIR.Evaluation.Result result = HuePat.VoxIR.Evaluation.Evaluation.Evaluate(
                "path/to/VoxIR_Datasets//Office.ply",
                "path/to/VoxIR_Datasets/GroundTruth/Office",
                0.05, // Resolution of the voxel grid [m]
                0.0.DegreeToRadian(), // rotation around the vertical axis to apply to both, test and ground truth meshes [rad]
                outputConfig);

            // The evaluation results contain the following properties (see paper for further details)
            double roomMappingError = result.RoomMappingError;
            double roomSegmentationPrecision = result.RoomSegmentationPrecision;
            double roomSegmentationRecall = result.RoomSegmentationRecall;
            double roomSegmentationF1Score = result.RoomSegmentationF1Score;
            Dictionary<int, double> voxelClassificationPrecision = result.VoxelClassificationPrecision;
            Dictionary<int, double> voxelClassificationRecall = result.VoxelClassificationRecall;
            Dictionary<int, double> voxelClassificationF1Score = result.VoxelClassificationF1Score;
            Dictionary<int, double> voxelClassificationNeighbourhoodPrecision = result.VoxelClassificationNeighbourhoodPrecision;
            Dictionary<int, double> voxelClassificationNeighbourhoodRecall = result.VoxelClassificationNeighbourhoodRecall;
            // The Dictionaries have keys of the following voxel class values:
            //      CEILING = 0
            //      EMPTY_INTERIOR = 1
            //      INTERIOR_OBJECT = 2
            //      FLOOR = 3
            //      WALL = 4
            //      WALL_OPENING = 5

            // The evaluation can be applied to the same dataset and ground truth directory pair with a list
            //  of multiple (resolution, rotation around the vertical axis) parameter tuples:
            // (In this case, no Result object gets returned; The results are written to a csv table in the output directory set in the OutputConfig)
            HuePat.VoxIR.Evaluation.Evaluation.Evaluate(
                "path/to/VoxIR_Datasets//Office.ply",
                "path/to/VoxIR_Datasets/GroundTruth/Office",
                new (double, double)[] {
                    (0.05, 0.0.DegreeToRadian()),
                    (0.05, 30.0.DegreeToRadian()),
                    (0.07, 0.0.DegreeToRadian()),
                },
                outputConfig);

            // The evaluation can be applied to multiple datasets, each time with the same set of parameters
            // E.g. for replicating the evaluation published in the tables in the paper, use:

            Evaluation.Evaluation.Evaluate(
                new (string, string)[] {
                    (
                        "path/to/VoxIR_Datasets/Office.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/Office"
                    ),
                    (
                        "path/to/VoxIR_Datasets/Attic.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/Attic"
                    ),
                    (
                        "path/to/VoxIR_Datasets/Basement.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/Basement"
                    ),
                    (
                        "path/to/VoxIR_Datasets/ResidentialHouse.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/ResidentialHouse"
                    )
                },
                new (double, double)[] {
                    (0.05, 0.00.DegreeToRadian()),
                    (0.05, 10.00.DegreeToRadian()),
                    (0.05, 20.00.DegreeToRadian()),
                    (0.05, 30.00.DegreeToRadian()),
                    (0.05, 40.00.DegreeToRadian()),
                    (0.05, 45.00.DegreeToRadian())
                },
                outputConfig);

            Evaluation.Evaluation.Evaluate(
                new (string, string)[] {
                    (
                        "path/to/VoxIR_Datasets/Office.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/Office"
                    ),
                    (
                        "path/to/VoxIR_Datasets/Attic.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/Attic"
                    ),
                    (
                        "path/to/VoxIR_Datasets/Basement.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/Basement"
                    ),
                    (
                        "path/to/VoxIR_Datasets/ResidentialHouse.ply",
                        "path/to/VoxIR_Datasets/GroundTruth/ResidentialHouse"
                    )
                },
                new (double, double)[] {
                    (0.03, 0.00.DegreeToRadian()),
                    (0.05, 0.00.DegreeToRadian()),
                    (0.07, 0.00.DegreeToRadian()),
                    (0.09, 0.00.DegreeToRadian()),
                    (0.11, 0.00.DegreeToRadian()),
                    (0.13, 0.00.DegreeToRadian()),
                    (0.15, 0.00.DegreeToRadian())
                },
                outputConfig);

            // Note: During refactoring for the publication of VoxIR, some changes were made to the code;
            //  furthermore, the alignment of the datasets with their respective coordinate systems has since changed
            //  (see Hübner, P.; Weinmann, M.; Wursthorn, S. & Hinz, S. 2021. Pose Normalization of Partially Manhattan
            //   World Conform Indoor Mapping Point Clouds and Triangle Meshes arXiv:2107.07778, 1-39)
            // The evaluation results attained with the method above are thus not identical to the tables in the paper
            // They are however overall in a range comparable to the published results.
        }
    }
}
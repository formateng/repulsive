namespace RepulsionOfPointsOnMesh
{
    using System;
    using System.Collections.Generic;
    using System.Drawing;
    using Grasshopper.Kernel;
    using Rhino;
    using Rhino.Geometry;

    /// <summary>
    /// For distributing holes on a mesh in regards to vertex colours.
    /// </summary>
    public class RepulsionOfPointsOnMesh : GH_Component
    {
        private List<Point3d> points;
        private List<Vector3d> velocities;
        private Point3d[] vertices;
        private List<double> radius;
        private List<Color> colours;

        public RepulsionOfPointsOnMesh()
            : base("Repulsion of points on mesh", "REPULSE", "Repulses points on a mesh, in regards to vertex colour and radius", "RCD", "Mesh Repulsion")
        {
        }

        /// <summary>
        /// Registers input parameters for component
        /// </summary>
        /// <param name="pManager"></param>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "List of points for repulsion", GH_ParamAccess.list);
            pManager.AddNumberParameter("Radius", "R", "Radius of points", GH_ParamAccess.list);
            pManager.AddMeshParameter("Mesh", "M", "Mesh on which points will move.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Damping", "D", "Damping the system. If damping=1, the system will keep running", GH_ParamAccess.item, 0.99);
            pManager.AddNumberParameter("Force", "F", "Force working between points", GH_ParamAccess.item, 15);
            pManager.AddColourParameter("Colour", "C", "Vertex Colours in same order as vertices", GH_ParamAccess.list);
            pManager.AddNumberParameter("Colour Sensitivity", "CI", "1 = if black, then NO repulsion, 0 = same repulsions anywhere on mesh", GH_ParamAccess.item, 0.7);
            pManager.AddBooleanParameter("Reset", "Reset", "Resets the system", GH_ParamAccess.item, true);
        }

        /// <summary>
        /// Registers output parameters for component
        /// </summary>
        /// <param name="pManager"></param>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "New Points", GH_ParamAccess.list);
            pManager.AddNumberParameter("Radius", "R", "New radius", GH_ParamAccess.list);
            pManager.AddLineParameter("Acceleration", "A", "Draws lines to represent the acceleration of a point", GH_ParamAccess.list);
            pManager.AddLineParameter("Velocity", "V", "Draws lines to represent to velocity of a point. A good way to check if the system has settled is to attached the following to this output: Curve Length -> Mass Addition -> Data Recorder -> Quick Graph", GH_ParamAccess.list);
            pManager.AddNumberParameter("Brigthnesses", "B", "Current brightness of mesh at location of point", GH_ParamAccess.list);
        }

        /// <summary>
        /// Returns error messages for debugging when run in Grasshopper
        /// </summary>
        /// <param name="DA"></param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
#if DEBUG
            try
            {
#endif
                SolveInstanceImplementation(DA);
#if DEBUG
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException(String.Format("{0} \n {1}", ex.Message, ex.StackTrace));
            }
#endif
        }
        /// <summary>
        /// Main Method. Takes input, calculates instance, return output
        /// </summary>
        /// <param name="DA"></param>
        private void SolveInstanceImplementation(IGH_DataAccess DA)
        {
            // 1. Declare placeholder variables and assign initial invalid data.
            //    This way, if the input parameters fail to supply valid data, we know when to abort.
            Mesh mesh = null;
            double damping = double.NaN;
            double force = double.NaN;
            bool reset = true;
            double colourImportance = double.NaN;

            // 2. Retrieve input data.
            if (!DA.GetData(7, ref reset)) { return; }
            if (reset)
            {
                this.points = new List<Point3d>();
                if ((!DA.GetDataList(0, this.points))) { return; }

                this.radius = new List<double>();
                if (!DA.GetDataList(1, this.radius)) { return; }

                this.colours = new List<Color>();
                if (!DA.GetDataList(5, this.colours)) { return; }
            }

            if (!DA.GetData(2, ref mesh)) { return; }
            if (!DA.GetData(3, ref damping)) { return; }
            if (!DA.GetData(4, ref force)) { return; }
            if (!DA.GetData(6, ref colourImportance)) { return; }

            // 3. Abort on invalid inputs.
            if (this.points == null) { return; }
            if (this.points.Count == 0) { return; }
            if (this.radius == null) { return; }
            if (this.radius.Count == 0) { return; }
            if (!mesh.IsValid) { return; }
            if (!RhinoMath.IsValidDouble(force)) { return; }
            if (!RhinoMath.IsValidDouble(damping)) { return; }
            if (!RhinoMath.IsValidDouble(colourImportance)) { return; }

            // 4. Initialize velocities and points and computate facenormals
            int numPoints = points.Count;
            if (reset)
            {
                ResetVelocities(numPoints);
                mesh.FaceNormals.ComputeFaceNormals();
                vertices = mesh.Vertices.ToPoint3dArray();
            }

            // 5. Calculate brightness for all points
            List<double> brightnesses = CalculateBrightnessForAllPoints(mesh, numPoints);

            // 6. Activate holes
            List<Hole> holes = CreateHolesWithInitialData(mesh, force, colourImportance, numPoints, brightnesses);

            // 7. Find new location
            foreach (Hole hole in holes)
            {
                hole.CalculateNewLocation(holes);
            }

            // 8. Save data for next iteration
            for (int i = 0; i < numPoints; i++)
            {
                this.points[i] = holes[i].Point;
                this.velocities[i] = holes[i].NewVelocity * damping;
            }

            // 9. Get new radius for output
            List<double> newRadius = new List<double>(numPoints);
            for (int i = 0; i < numPoints; i++)
            {
                newRadius.Add(holes[i].NewRadius);
            }

            // 10. Assign lines to output representing acceleration and velocity
            List<Line> accelerations = new List<Line>(numPoints);
            List<Line> velocityLines = new List<Line>(numPoints);
            for (int i = 0; i < numPoints; i++)
            {
                Hole hole = holes[i];
                Line tempLine = hole.AccelerationAsLine();
                accelerations.Add(tempLine);

                Point3d tempPnt = hole.Point;
                Line tempVelocityLine = new Line(tempPnt, tempPnt + velocities[i]);
                velocityLines.Add(tempVelocityLine);
            }

            // 11. Assign output.
            DA.SetDataList(0, points);
            DA.SetDataList(1, newRadius);
            DA.SetDataList(2, accelerations);
            DA.SetDataList(3, velocityLines);
            DA.SetDataList(4, brightnesses);
        }

        /// <summary>
        /// Initializes hole cutters on mesh
        /// </summary>
        /// <param name="mesh">Mesh on which holes are intented to be distributed</param>
        /// <param name="force">The size of the force working between the holes on the mesh.</param>
        /// <param name="colourSensitivity">Double between 0 and 1 to determine the coloursensitivity of a point</param>
        /// <param name="numPoints">number of holes on mesh</param>
        /// <param name="brightnesses">list of brightnesses for all points</param>
        /// <returns>List of holes</returns>
        private List<Hole> CreateHolesWithInitialData(Mesh mesh, double force, double colourSensitivity, int numPoints, List<double> brightnesses)
        {
            List<Hole> holes = new List<Hole>(numPoints);
            for (int i = 0; i < numPoints; i++)
            {
                Hole tempHole = new Hole(points[i], velocities[i], mesh, radius[i], force, vertices, brightnesses[i], colourSensitivity);
                holes.Add(tempHole);
            }

            return holes;
        }

        /// <summary>
        /// Interpolates between known colours which are given at vertices of the mesh to calculate the approximate colour at each of our points
        /// </summary>
        /// <param name="mesh">Mesh on which the points are constrained and the colours relate to</param>
        /// <param name="numPoints">The number of points we are working with</param>
        /// <returns>List of approximate colours for each point of all the holes.  The index in the list for the colour corresponds to the index of the point in the holes list.</returns>
        private List<double> CalculateBrightnessForAllPoints(Mesh mesh, int numPoints)
        {
            List<double> brightnesses = new List<double>(numPoints);
            MeshColourHelper colourExtension = new MeshColourHelper();
            foreach (Point3d p in points)
            {
                double tempBrightness = colourExtension.CalculatePointBrightness(mesh, p, this.colours);
                brightnesses.Add(tempBrightness);
            }

            return brightnesses;
        }

        /// <summary>
        /// Sets all velocities to zero
        /// </summary>
        /// <param name="numPoints">The number of points we are working with</param>
        private void ResetVelocities(int numPoints)
        {
            velocities = new List<Vector3d>(numPoints);
            for (int i = 0; i < numPoints; i++)
            {
                velocities.Add(Vector3d.Zero);
            }
        }



        public override Guid ComponentGuid
        {
            get { return new Guid("916BB9E8-CFBC-4992-80DC-9FB137D43A1B"); }

        }
    }
}

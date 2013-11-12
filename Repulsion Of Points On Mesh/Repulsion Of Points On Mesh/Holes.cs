namespace RepulsionOfPointsOnMesh
{
    using System;
    using System.Collections.Generic;
    using System.Drawing;
    using Rhino;
    using Rhino.Geometry;

    //TODO xml documentation
    /// <summary>
    /// Engine for distributing holes on mesh
    /// </summary>
    class Hole
    {
        Point3d point;
        Point3d newPoint;
        Point3d[] vertices;
        MeshPoint meshFullPoint;
        Mesh mesh;
        Vector3d previousVelocityVector;
        Vector3d newVelocityVector;
        Vector3d newAccelerationVector;
        double radius;
        double force;
        double brightness;
        double colourSensitivityRepulsion;
        double colourSensitivityRadius;
        MeshColourHelper ColourMethod = new MeshColourHelper();

        /// <summary>
        /// Initiliazing hole
        /// </summary>
        /// <param name="tempPoint">Centerpoint of hole</param>
        /// <param name="tempVelocityVector">Current velocity of hole</param>
        /// <param name="tempMesh">Mesh on which hole is</param>
        /// <param name="tempRadius">Radius of the hole</param>
        /// <param name="tempForce">Force with which holes are influencing each other</param>
        /// <param name="tempVertices">All vertices of mesh</param>
        /// <param name="tempBrightness">Brightness of hole</param>
        /// <param name="tempColourSensitivityRepulsion">Sensitivity of which a colour will affect the force between two points</param>
        public Hole(Point3d tempPoint, Vector3d tempVelocityVector, Mesh tempMesh, double tempRadius, double tempForce, Point3d[] tempVertices, double tempBrightness, double tempColourSensitivityRepulsion, double tempColourSensitivityRadius)
        {
            vertices = tempVertices;
            point = tempPoint;
            previousVelocityVector = tempVelocityVector;
            mesh = tempMesh;
            radius = tempRadius;
            force = tempForce;
            brightness = tempBrightness;
            colourSensitivityRepulsion = tempColourSensitivityRepulsion;
            colourSensitivityRadius = tempColourSensitivityRadius;
        }

        /// <summary>
        /// Plane aligned with the mesh face on which the point currently sits
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        private Plane FacePlane(int faceIndex)
        {
            Plane initialFacePlane = new Plane(vertices[this.mesh.Faces.GetFace(faceIndex).A], this.mesh.FaceNormals[faceIndex]);
            return initialFacePlane;
        }

        /// <summary>
        /// Get array of vertex positions for a given mesh face
        /// </summary>
        /// <param name="faceIndex"></param>
        /// <returns></returns>
        private Point3f[] GetVerticesOfFace(int faceIndex)
        {
            Point3f[] VertexTemporary = new Point3f[4];
            this.mesh.Faces.GetFaceVertices(faceIndex, out VertexTemporary[0], out VertexTemporary[1], out VertexTemporary[2], out VertexTemporary[3]);
            return VertexTemporary;
        }

        /// <summary>
        /// Calculates distance between a hole edge and all other holes. This distance is used to calculate the force between the points and assign an acceleration to a point.
        /// </summary>
        /// <param name="others"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        private void CalculateAcceleration(IList<Hole> others)
        {
            this.newAccelerationVector = Vector3d.Zero;
            double minimumDenominatorToAvoidDivideByZero = 5;
            for (int i = 0; i < others.Count; i++)
            {
                Hole other = others[i];
                Vector3d VectorPointToPointInfluence = other.point - this.point;
                double distanceBetweenPoints = VectorPointToPointInfluence.Length;
                if (distanceBetweenPoints < double.Epsilon)
                {
                    continue;
                }

                double distanceBetweenHoles = distanceBetweenPoints - other.radius - this.radius;
                if (distanceBetweenHoles <= 0) //checks if holes intersect.
                {
                    distanceBetweenHoles = 0;
                }

                VectorPointToPointInfluence.Unitize();
                VectorPointToPointInfluence *= -this.force / (minimumDenominatorToAvoidDivideByZero + distanceBetweenHoles * distanceBetweenHoles);
                VectorPointToPointInfluence = AdjustAccelerationForColourSensitivity(VectorPointToPointInfluence, other);
                this.newAccelerationVector += VectorPointToPointInfluence;
            }

            this.newAccelerationVector = this.ConstrainVectorToMeshFace(this.newAccelerationVector);
        }

        private Vector3d AdjustAccelerationForColourSensitivity(Vector3d vectorBetweenPoints, Hole other)
        {
            return (1 - this.colourSensitivityRepulsion) * vectorBetweenPoints + this.colourSensitivityRepulsion * other.brightness * vectorBetweenPoints;
        }

        /// <summary>
        /// Calculates the velocity of a point
        /// </summary>
        /// <param name="holes"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        private Vector3d CalculateVelocity(IList<Hole> holes)
        {
            this.CalculateAcceleration(holes);
            this.newVelocityVector = this.previousVelocityVector + this.newAccelerationVector;
            this.newVelocityVector = ConstrainVectorToMeshFace(this.newVelocityVector);
            return this.newVelocityVector;
        }

        private Vector3d ConstrainVectorToMeshFace(Vector3d vector)
        {
            Point3d pnt = this.FacePlane(mesh.FaceIndexOfClosestPoint(this.point)).ClosestPoint(this.point + vector);
            return pnt - this.point;
        }

        /// <summary>
        /// Calculates the new location of a point after being moved.
        /// </summary>
        /// <param name="holes"></param>
        public void CalculateNewLocation(IList<Hole> holes)
        {
            this.CalculateNewRadius();
            this.CalculateVelocity(holes);
            this.newPoint = this.point + this.newVelocityVector;
            this.meshFullPoint = mesh.ClosestMeshPoint(newPoint, 0.0);
            this.NewLocationCheck(holes, this.point, 1, this.newVelocityVector, mesh.FaceIndexOfClosestPoint(this.point));
        }

        private void CalculateNewRadius()
        {
            this.radius = this.radius * (1 - colourSensitivityRadius) + this.radius * (1 - this.brightness) * colourSensitivityRadius;
        }

        int recursionsStopper; //safety variable to avoid endless recursions.
        /// <summary>
        /// If the new location is on another face than the initial one, this method will loop until the final location of the itteretation is found.
        /// </summary>
        /// <param name="holes"></param>
        /// <param name="p"></param>
        /// <param name="parameter"></param>
        /// <param name="velocity"></param>
        /// <param name="faceIndex"></param>
        private void NewLocationCheck(IList<Hole> holes, Point3d p, double parameter, Vector3d velocity, int faceIndex)
        {
            recursionsStopper += 1;
            bool intersection;
            int edgeParameter;
            int vertexOneIndex;
            int vertexTwoIndex;
            double moveParameter;
            velocity = parameter * velocity;
            HasMovedOutsideFace(p, velocity, faceIndex, out intersection, out edgeParameter, out moveParameter);
            if (intersection)
            {
                GetVertexIndexesOfEdge(faceIndex, edgeParameter, out vertexOneIndex, out vertexTwoIndex);
                if (IsEdgeNaked(vertexOneIndex, vertexTwoIndex, faceIndex))
                {
                    this.newPoint = BounceOfBoundary(p, velocity, vertices[vertexOneIndex], vertices[vertexTwoIndex], mesh.FaceNormals[faceIndex]);
                    this.meshFullPoint = mesh.ClosestMeshPoint(this.newPoint, 0.0);
                }
                else
                {
                    Point3d intersectionPoint = p + velocity * moveParameter * 1.01; //multiplied by 1.01 to make sure that it doesn't cross same edge twice
                    Vector3d RotatedVelocityVector = RotateVectorToNewFace(mesh.FaceNormals[faceIndex], mesh.FaceNormals[getOtherFaceIndex(vertexOneIndex, vertexTwoIndex, faceIndex)], vertices[vertexOneIndex], vertices[vertexTwoIndex], velocity);
                    this.newPoint = intersectionPoint + RotatedVelocityVector * (1 - moveParameter);
                    this.meshFullPoint = mesh.ClosestMeshPoint(newPoint, 0.0);
                    this.newVelocityVector = RotatedVelocityVector;
                    if (recursionsStopper < 30)
                    {
                        this.NewLocationCheck(holes, intersectionPoint, (1 - moveParameter), newVelocityVector, getOtherFaceIndex(vertexOneIndex, vertexTwoIndex, faceIndex));
                    }
                }
            }
        }

        /// <summary>
        /// Check if a point has moved outside the face it was on in the initial position
        /// </summary>
        /// <param name="pointForMethod"></param>
        /// <param name="velocity"></param>
        /// <param name="faceIndex"></param>
        /// <param name="intersect"></param>
        /// <param name="edgeParameter"></param>
        /// <param name="moveParameter"></param>
        private void HasMovedOutsideFace(Point3d pointForMethod, Vector3d velocity, int faceIndex, out bool intersect, out int edgeParameter, out double moveParameter)
        {
            Line currentVelocity = new Line(pointForMethod, pointForMethod + velocity);
            List<Plane> planesPerpendicularToFaceEdges = this.CalculatePlanesParallelToFaceEdges(faceIndex);
            intersect = false;
            edgeParameter = 0;
            moveParameter = 1;
            for (int i = 0; i < planesPerpendicularToFaceEdges.Count; i++)
            {
                intersect = Rhino.Geometry.Intersect.Intersection.LinePlane(currentVelocity, planesPerpendicularToFaceEdges[i], out moveParameter);
                if (moveParameter < 0 || moveParameter > 1)
                {
                    intersect = false;
                }
                if (intersect)
                {
                    edgeParameter = i;
                    return;
                }
            }
        }

        /// <summary>
        /// Calculates a planes perpendencular to face plane, and parallel to face edges from a faceIndex
        /// </summary>
        /// <param name="faceIndex"></param>
        /// <returns></returns>
        private List<Plane> CalculatePlanesParallelToFaceEdges(int faceIndex)
        {
            Point3f[] faceVertices = this.GetVerticesOfFace(faceIndex);
            List<Plane> planesPerpendicularToFaceEdges = new List<Plane>(4);
            int numVertices = 3;
            if (mesh.Faces.GetFace(faceIndex).IsQuad)
            {
                numVertices = 4;
            }

            for (int i = 0; i < numVertices; i++)
            {
                int j = i + 1;
                if (i == numVertices - 1)
                {
                    j = 0;
                }

                Plane edge = this.EdgePlane(faceVertices[i], faceVertices[j], mesh.FaceNormals[faceIndex]);
                planesPerpendicularToFaceEdges.Add(edge);
            }

            return planesPerpendicularToFaceEdges;
        }

        /// <summary>
        /// Creates a plane perpendicular to edge of a mesh face from vertices and face normal
        /// </summary>
        /// <param name="vertexOne"></param>
        /// <param name="vertexTwo"></param>
        /// <param name="normal"></param>
        /// <returns></returns>
        private Plane EdgePlane(Point3d vertexOne, Point3d vertexTwo, Vector3d normal)
        {
            Point3d thirdPointOnMirrorPlane = vertexOne + normal;
            Plane plane = new Plane(vertexOne, vertexTwo, thirdPointOnMirrorPlane);
            return plane;
        }

        private void GetVertexIndexesOfEdge(int faceIndex, int edgeParameter, out int vertexOneIndex, out int vertexTwoIndex)
        {
            int numVertices = 3;
            if (mesh.Faces.GetFace(faceIndex).IsQuad)
            {
                numVertices = 4;
            }

            vertexOneIndex = 0;
            vertexTwoIndex = 0;
            if (edgeParameter == 0)
            {
                vertexOneIndex = mesh.Faces.GetFace(faceIndex).A;
                vertexTwoIndex = mesh.Faces.GetFace(faceIndex).B;
            }
            else if (edgeParameter == 1)
            {
                vertexOneIndex = mesh.Faces.GetFace(faceIndex).B;
                vertexTwoIndex = mesh.Faces.GetFace(faceIndex).C;
            }
            else if (edgeParameter == 2)
            {
                vertexOneIndex = mesh.Faces.GetFace(faceIndex).C;
                vertexTwoIndex = mesh.Faces.GetFace(faceIndex).D;
                if (numVertices == 3)
                {
                    vertexTwoIndex = mesh.Faces.GetFace(faceIndex).A;
                }
            }
            else if (edgeParameter == 3)
            {
                vertexOneIndex = mesh.Faces.GetFace(faceIndex).D;
                vertexTwoIndex = mesh.Faces.GetFace(faceIndex).A;
            }
        }

        /// <summary>
        /// Rotates a vector from around an edge.
        /// </summary>
        /// <param name="NormalOne"></param>
        /// <param name="NormalTwo"></param>
        /// <param name="VertexOne"></param>
        /// <param name="VertexTwo"></param>
        /// <param name="VectorToRotate"></param>
        /// <returns></returns>
        private Vector3d RotateVectorToNewFace(Vector3d NormalOne, Vector3d NormalTwo, Point3d VertexOne, Point3d VertexTwo, Vector3d VectorToRotate)
        {
            Double angle = Vector3d.VectorAngle(NormalOne, NormalTwo);
            Vector3d rotationAxis = VertexOne - VertexTwo;
            Vector3d newVector1 = VectorToRotate;
            newVector1.Rotate(angle, rotationAxis);
            Vector3d newVector2 = VectorToRotate;
            newVector2.Rotate(-angle, rotationAxis);
            double newAngle1 = Vector3d.VectorAngle(NormalTwo, newVector1);
            double newAngle2 = Vector3d.VectorAngle(NormalTwo, newVector2);
            if (Math.Abs(Math.PI / 2 - newAngle1) > Math.Abs(Math.PI / 2 - newAngle2))
            {
                return newVector2;
            }
            else
            {
                return newVector1;
            }
        }

        /// <summary>
        /// Makes a point bounce of boundary if the edge face is naked
        /// </summary>
        /// <param name="basePoint"></param>
        /// <param name="velocity"></param>
        /// <param name="vertexOne"></param>
        /// <param name="vertexTwo"></param>
        /// <param name="normalOfFace"></param>
        /// <returns></returns>
        private Point3d BounceOfBoundary(Point3d basePoint, Vector3d velocity, Point3d vertexOne, Point3d vertexTwo, Vector3d normalOfFace)
        {
            this.newPoint = basePoint + velocity;
            Point3d mirroredPoint = Transform.Mirror(EdgePlane(vertexOne, vertexTwo, normalOfFace)) * newPoint;
            this.newVelocityVector = Transform.Mirror(EdgePlane(vertexOne, vertexTwo, normalOfFace)) * velocity;
            return mirroredPoint;
        }

        /// <summary>
        /// Checks if an edge is naked
        /// </summary>
        /// <param name="vertexOneIndex"></param>
        /// <param name="vertexTwoIndex"></param>
        /// <param name="faceIndex"></param>
        /// <returns></returns>
        private bool IsEdgeNaked(int vertexOneIndex, int vertexTwoIndex, int faceIndex)
        {
            if (getOtherFaceIndex(vertexOneIndex, vertexTwoIndex, faceIndex) == -1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Gets the index of the neighboor face if to vertices and and the first faceindex is known
        /// </summary>
        /// <param name="vertexOneIndex"></param>
        /// <param name="vertexTwoIndex"></param>
        /// <param name="faceIndex"></param>
        /// <returns></returns>
        private int getOtherFaceIndex(int vertexOneIndex, int vertexTwoIndex, int faceIndex)
        {
            int numberOfSharedFaces = 0;
            int[] facesOne = mesh.Vertices.GetVertexFaces(vertexOneIndex);
            int[] facesTwo = mesh.Vertices.GetVertexFaces(vertexTwoIndex);
            List<int> sharedFaces = new List<int>(2);
            int otherFace = -1;
            for (int i = 0; i < facesOne.Length; i++)
            {
                for (int j = 0; j < facesTwo.Length; j++)
                {
                    if (facesOne[i] == facesTwo[j])
                    {
                        numberOfSharedFaces += 1;
                        sharedFaces.Add(facesOne[i]);
                    }
                }
            }
            if (numberOfSharedFaces < 2)
            {
                return otherFace;
            }
            else
            {
                for (int i = 0; i < sharedFaces.Count; i++)
                {
                    if (faceIndex != sharedFaces[i])
                    {
                        otherFace = sharedFaces[i];
                    }
                }
                return otherFace;
            }
        }

        /// <summary>
        /// returns Meshpoint
        /// </summary>
        public Point3d Point
        {
            get
            {
                return meshFullPoint.Point;
            }
        }

        /// <summary>
        /// Returns the velocity
        /// </summary>
        public Vector3d NewVelocity
        {
            get
            {
                return newVelocityVector;
            }
        }

        public double NewRadius
        {
            get
            {
                return this.radius;
            }
        }

        /// <summary>
        /// Draws lines representing the acceleration
        /// </summary>
        /// <param name="holes"></param>
        /// <returns></returns>
        public Line AccelerationAsLine()
        {
            Point3d endPoint = meshFullPoint.Point + this.newAccelerationVector;
            Line accelerationLine = new Line(meshFullPoint.Point, endPoint);
            return accelerationLine;
        }
    }
}

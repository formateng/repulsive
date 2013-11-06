using System;
using System.Collections.Generic;
    using Rhino;
    using Rhino.Geometry;

namespace RepulsionOfPointsOnMesh
{
    public static class MeshExtensions
    {
        /// <summary>
        /// Gets the point on the mesh that is closest to a given test point.  Similar to the ClosestPoint function except the returns a MeshPoint class which includes extra information beyond just the location of the closest point.
        /// </summary>
        /// <param name="theMesh"></param>
        /// <param name="testPoint"></param>
        /// <returns></returns>
        public static MeshPoint ClosestMeshPoint(this Mesh theMesh, Point3d testPoint)
        {
            return theMesh.ClosestMeshPoint(testPoint, 0.0);
        }

        /// <summary>
        /// Returns the index of the face closest to a point
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        public static int FaceIndexOfClosestPoint(this Mesh theMesh, Point3d testPoint)
        {
            return theMesh.ClosestMeshPoint(testPoint).FaceIndex;
        }
    }
}

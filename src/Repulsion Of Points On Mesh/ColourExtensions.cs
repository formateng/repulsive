using System;
using System.Collections.Generic;
using System.Drawing;
using Rhino;
using Rhino.Geometry;

namespace RepulsionOfPointsOnMesh
{
    public class MeshColourHelper
    {

        /// <summary>
        /// Returns the brightness of the mess at a points position
        /// </summary>
        /// <returns></returns>
        public double CalculatePointBrightness(Mesh mesh, Point3d p, IList<Color> colours)
        {
            double[] barycentricCoordinates = mesh.ClosestMeshPoint(p).T;
            MeshFace face = mesh.Faces.GetFace(mesh.FaceIndexOfClosestPoint(p));
            double brightnessVertexA = colours[face.A].GetBrightness() * barycentricCoordinates[0];
            double brightnessVertexB = colours[face.B].GetBrightness() * barycentricCoordinates[1];
            double brightnessVertexC = colours[face.C].GetBrightness() * barycentricCoordinates[2];
            double brightnessVertexD = colours[face.D].GetBrightness() * barycentricCoordinates[3];
            double brightness = brightnessVertexA + brightnessVertexB + brightnessVertexC + brightnessVertexD;
            return brightness;
        }
    }
}

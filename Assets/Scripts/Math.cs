using Emgu.CV;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Emgu.CV.Structure;
using UnityEngine;

public class Math : MonoBehaviour
{
    public static Matrix<float> Matrix4x4To3x3Array(Matrix4x4 mat)
    {
        var mtx = new Matrix<float>(3, 3);

        mtx[0, 0] = mat.m00;
        mtx[0, 1] = mat.m01;
        mtx[0, 2] = mat.m02;

        mtx[1, 0] = mat.m10;
        mtx[1, 1] = mat.m11;
        mtx[1, 2] = mat.m12;

        mtx[2, 0] = mat.m20;
        mtx[2, 1] = mat.m21;
        mtx[2, 2] = mat.m22;

        return mtx;
    }

    public static Matrix4x4 Matrix4x4To3x3Array(Matrix<double> mat)
    {
        var mtx = new Matrix4x4 {
            [0, 0] = (float)mat[0, 0],
            [0, 1] = (float)mat[0, 1],
            [0, 2] = (float)mat[0, 2],

            [1, 0] = (float)mat[1, 0],
            [1, 1] = (float)mat[1, 1],
            [1, 2] = (float)mat[1, 2],

            [2, 0] = (float)mat[2, 0],
            [2, 1] = (float)mat[2, 1],
            [2, 2] = (float)mat[2, 2],

            [3, 3] = 1
        };

        return mtx;
    }

    public static void Matrix4x4ToRvecAndTvec(Matrix4x4 position, out Matrix<float> rvec, out Matrix<float> tvec)
    {
        tvec = new Matrix<float>(new[] { position[0, 3], position[1, 3], position[2, 3] });
        rvec = new Matrix<float>(3, 1);

        position[0, 3] = position[1, 3] = position[2, 3] = 0;

        CvInvoke.Rodrigues(Matrix4x4To3x3Array(position), rvec);
    }

    public static MCvPoint3D32f[] Vector3ToMCvPoint3D32Fs(Vector3[] objectPoints)
    {
        return objectPoints.Select(x => new MCvPoint3D32f(x.x, x.y, x.z)).ToArray();
    }

    public static Matrix4x4? CalibrateCamera(Matrix4x4 intrinsicMatrix, Vector3[] objectPoints, Vector2[] imagePoints)
    {
        var rotVec = new Matrix<double>(3, 1);
        var vector = new Matrix<double>(3, 1);

        var res = CvInvoke.SolvePnP(
            Vector3ToMCvPoint3D32Fs(objectPoints),
            imagePoints.Select(x => new PointF(x.x, x.y)).ToArray(),
            Matrix4x4To3x3Array(intrinsicMatrix),
            new Mat(),
            rotVec,
            vector
        );

        if (!res) {
            return null;
        }

        var rotMat = new Matrix<double>(3, 3);

        CvInvoke.Rodrigues(rotVec, rotMat);

        var vec = new Vector3((float)vector[0, 0], (float)vector[1, 0], (float)vector[2, 0]);

        return MathematicaRayLoader.GetTransformMatrixFromRotAndVec(Matrix4x4To3x3Array(rotMat), vec, 1);
    }

    public static Matrix4x4?[] CalibrateCameras(CameraData[] cameraData, List<Vector3> objectPoints, List<Vector2>[] imagePoints) {
        var res = new Matrix4x4?[cameraData.Length];

        for (var i = 0; i < cameraData.Length; i++) {
            res[i] = CalibrateCamera(cameraData[i].intrinsicMatrix, objectPoints.ToArray(), imagePoints[i].ToArray());
        }

        return res;
    }


}

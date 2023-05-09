using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Web.UI.WebControls.WebParts;
using CameraState;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using RequestedCameraState;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using Capturing = InnerCameraState.Capturing;
using Error = InnerCameraState.Error;
using Stopped = InnerCameraState.Stopped;

namespace InnerCameraState
{
    interface IState { }

    class Capturing : IState {
        public VideoCapture capture;
        public double delta;
        public double grabbingTime;
        public double readingTime;
        public int exposure;
        public Vector2 resolution;

        public DateTime frameTime;
        public Mat image;
    }

    class Stopped : IState { }

    class Error : IState {
        public string message;
    }
}

namespace CameraState
{
    public interface IState {}

    public struct StartCapturing : IState { }

    public struct Capturing : IState {
        public double delta;
        public DateTime frameTime;
        public double grabbingTime;
        public double readingTime;
        public Mat image;
    }

    public struct Stopping : IState { }

    public struct Stopped : IState { }

    public struct Error : IState
    {
        public string message;
    }
}

namespace RequestedCameraState {
    public interface IState {}

    public struct Capture : IState {
        public int exposure;
        public Vector2 resolution;
    }

    public struct StopCapture : IState { }
}

public class CameraImage {
    public DateTime frameTime;
    public Mat image;
}

public struct NamedCameraImage
{
    public int id;
    public CameraImage image;
}

public class NamedCameraTexture {
    public int? id;
    public DateTime frameTime;
    public Texture2D texture;
    public Mat image;
}

public class CameraImageGetter
{
    private static Dictionary<int, RequestedCameraState.IState> m_requestedCameraStates = new();
    private static Dictionary<int, CameraState.IState> m_cameraStates = new();

    private static Task m_cameraImagesReadingTask;

    private static bool m_threadRunning;
    private static int m_fps = 30;

    private static Task CreateCameraCaptureTask()
    {
        return Task.Factory.StartNew(() =>
        {
            Dictionary<int, InnerCameraState.IState> innerCameraStates = new();
            Dictionary<int, RequestedCameraState.IState> newCameraStates = new();

            while (m_threadRunning)
            {
                lock (m_requestedCameraStates)
                {
                    foreach (var (camId, state) in m_requestedCameraStates)
                    {
                        newCameraStates[camId] = state;
                    }
                }

                // Update camera states
                foreach (var (camId, newState) in newCameraStates)
                {
                    var innerCameraState = innerCameraStates.GetValueOrDefault(camId, new Stopped());

                    switch (newState)
                    {
                        case RequestedCameraState.Capture requestedCapture:
                        {
                            switch (innerCameraState)
                            {
                                case Stopped or Error: {
                                    try
                                    {
                                        var newCamera = new VideoCapture(camId, VideoCapture.API.DShow);
                                        newCamera.Set(CapProp.FrameWidth, requestedCapture.resolution.x);
                                        newCamera.Set(CapProp.FrameHeight, requestedCapture.resolution.y);
                                        //newCamera.Set(CapProp.Buffersize, 1);
                                        newCamera.Set(CapProp.Exposure, requestedCapture.exposure);

                                        innerCameraState = new InnerCameraState.Capturing
                                        {
                                            capture = newCamera, delta = 0, frameTime = new DateTime(),
                                            exposure = requestedCapture.exposure, resolution = requestedCapture.resolution,
                                            image = new Mat()
                                        };
                                    }
                                    catch (Exception e)
                                    {
                                        innerCameraState = new Error { message = e.Message };
                                    }
                                } break;
                                case Capturing capturing:
                                {
                                    try
                                    {
                                        if (capturing.exposure != requestedCapture.exposure)
                                        {
                                            capturing.capture.Set(CapProp.Exposure, requestedCapture.exposure);
                                        }

                                        if (capturing.resolution != requestedCapture.resolution) {
                                            capturing.capture.Set(CapProp.FrameWidth, requestedCapture.resolution.x);
                                            capturing.capture.Set(CapProp.FrameHeight, requestedCapture.resolution.y);
                                        }

                                        if (capturing.exposure != requestedCapture.exposure ||
                                            capturing.resolution != requestedCapture.resolution)
                                        {
                                            innerCameraState = new InnerCameraState.Capturing
                                            {
                                                capture = capturing.capture,
                                                delta = capturing.delta,
                                                frameTime = capturing.frameTime,
                                                exposure = requestedCapture.exposure,
                                                resolution = requestedCapture.resolution,
                                                image = capturing.image
                                            };
                                        }
                                    }
                                    catch (Exception e) {
                                        innerCameraState = new Error { message = e.Message };
                                    }
                                }  break;
                            }
                        } break;
                        case RequestedCameraState.StopCapture:
                        {
                            switch (innerCameraState)
                            {
                                case Capturing capturing:
                                {
                                    try
                                    {
                                        capturing.capture.Dispose();

                                        innerCameraState = new Stopped();
                                    }
                                    catch (Exception e)
                                    {
                                        innerCameraState = new Error { message = e.Message };
                                    }
                                } break;
                                case Stopped:
                                break;
                            }
                        } break;
                    }

                    innerCameraStates[camId] = innerCameraState;
                }

                var frameTime = DateTime.Now;

                var camIds = innerCameraStates.Keys.ToArray();

                foreach (var camId in camIds) 
                {
                    try
                    {
                        if (innerCameraStates[camId] is Capturing capturing)
                        {
                            var startGrabTime = DateTime.Now;
                            var res = capturing.capture.Grab();
                            capturing.grabbingTime = (DateTime.Now - startGrabTime).TotalSeconds;
                            if (!res) {
                                capturing.capture.Dispose();
                                innerCameraStates[camId] = new Error { message = "Error at grabbing image" };
                            }
                        }
                    }
                    catch (Exception e)
                    {
                        innerCameraStates[camId] = new Error { message = e.Message };
                    }
                }

                foreach (var camId in camIds) {
                    try {
                        if (innerCameraStates[camId] is Capturing capturing) {
                            capturing.delta = (frameTime - capturing.frameTime).TotalSeconds;
                            capturing.frameTime = frameTime;
                            var startReadTime = DateTime.Now;
                            var res = capturing.capture.Read(capturing.image);
                            capturing.readingTime = (DateTime.Now - startReadTime).TotalSeconds;

                            if (!res) {
                                capturing.capture.Dispose();
                                innerCameraStates[camId] = new Error { message = "Error at reading image" };
                            }
                        }
                    }
                    catch (Exception e) {
                        innerCameraStates[camId] = new Error { message = e.Message };
                    }
                }

                var anyActive = false;
                lock (m_requestedCameraStates) lock (m_cameraStates) {
                    foreach (var (camId, cameraState) in innerCameraStates)
                    {
                        var currentState = m_cameraStates.GetValueOrDefault(camId, null);
                        var requestedState = m_requestedCameraStates.GetValueOrDefault(camId);
                        switch (cameraState)
                        {
                            case Capturing innerCapturing:
                            {
                                anyActive = true;
                                switch (currentState)
                                {
                                    case CameraState.Capturing capturing:
                                    {
                                        innerCapturing.image.CopyTo(capturing.image);

                                        currentState = new CameraState.Capturing {
                                            delta = innerCapturing.delta,
                                            frameTime = innerCapturing.frameTime,
                                            grabbingTime = innerCapturing.grabbingTime,
                                            readingTime = innerCapturing.readingTime,
                                            image = innerCapturing.image
                                        };
                                    } break;
                                    case CameraState.StartCapturing:
                                    {
                                        var image = new Mat();
                                        innerCapturing.image.CopyTo(image);

                                        currentState = new CameraState.Capturing {
                                            delta = innerCapturing.delta,
                                            frameTime = innerCapturing.frameTime,
                                            grabbingTime = innerCapturing.grabbingTime,
                                            readingTime = innerCapturing.readingTime,
                                            image = image
                                        };
                                    } break;
                                }
                            } break;
                            case Stopped:
                            {
                                if (currentState is not CameraState.Stopped && requestedState is StopCapture)
                                {
                                    currentState = new CameraState.Stopped();
                                }
                            } break;
                            case Error innerError:
                            {
                                currentState = new CameraState.Error{ message = innerError.message };
                                m_requestedCameraStates[camId] = new StopCapture();
                            } break;
                        }

                        m_cameraStates[camId] = currentState;
                    }
                }

                if (!anyActive) {
                    break;
                }
            }

            foreach (var (camId, camera) in innerCameraStates)
            {
                switch (camera) {
                    case Capturing capturing: {
                        capturing.capture.Stop();
                        capturing.capture.Dispose();
                    }
                    break;
                    case Stopped:
                    break;
                }
            }
        });
    }

    public static void StartCameraCapture(int camId, int exposure, Vector2 resolution)
    {
        lock (m_requestedCameraStates) lock (m_cameraStates)
        {
            m_requestedCameraStates[camId] = new Capture { exposure = exposure, resolution = resolution };
            m_cameraStates[camId] = new StartCapturing();
        }

        if (m_cameraImagesReadingTask is { Status: TaskStatus.Faulted or TaskStatus.Canceled or TaskStatus.RanToCompletion } or null) {
            m_threadRunning = true;
            m_cameraImagesReadingTask = CreateCameraCaptureTask();
        }
    }

    public static void SetCameraExposure(int camId, int exposure) {
        lock (m_requestedCameraStates) {
            if (!m_requestedCameraStates.TryGetValue(camId, out var state)) {
                return;
            }

            if (state is RequestedCameraState.Capture capture) {
                m_requestedCameraStates[camId] = new Capture { exposure = exposure, resolution = capture.resolution };
            }
        }
    }

    public static void SetCameraResolution(int camId, Vector2 resolution) {
        lock (m_requestedCameraStates) {
            if (!m_requestedCameraStates.TryGetValue(camId, out var state))
            {
                return;
            }

            if (state is RequestedCameraState.Capture capture) 
            {
                m_requestedCameraStates[camId] = new Capture { exposure = capture.exposure, resolution = resolution };
            } 
        }
    }

    public static void StopCameraCapture(int camId) {
        lock (m_requestedCameraStates) lock (m_cameraStates) {
            m_requestedCameraStates[camId] = new StopCapture();
            m_cameraStates[camId] = new CameraState.Stopping();
        }
    }

    public static CameraState.IState GetCameraState(int camId)
    {
        lock (m_cameraStates)
        {
            return m_cameraStates.GetValueOrDefault(camId, null);
        }
    }


    public static bool GetImage(ref Mat image, int camId)
    {
        lock (m_cameraStates)
        {
            if (!m_cameraStates.TryGetValue(camId, out var state))
            {
                return false;
            }

            if (state is not CameraState.Capturing capturing) 
            {
                return false;
            }

            image ??= new Mat();

            capturing.image.CopyTo(image);

            return true;
        }
    }

    public static void GetImages(NamedCameraImage[] images)
    {
        lock (m_cameraStates)
        {
            for (var i = 0; i < images.Length; i++)
            {
                if (!m_cameraStates.TryGetValue(images[i].id, out var state))
                {
                    continue;
                }

                if (state is not CameraState.Capturing capturing)
                {
                    continue;
                }

                images[i].image.image ??= new Mat();

                capturing.image.CopyTo(images[i].image.image);

                images[i].image.frameTime = capturing.frameTime;
            }
        }
    }

    public static void GetTexture(Mat image, ref Texture2D texture)
    {
        texture ??= new Texture2D(image.Width, image.Height, TextureFormat.RGB24, false);

        if (texture.format != TextureFormat.RGB24 || texture.width != image.Width ||
            texture.height != image.Height) {
            texture.Reinitialize(image.Width, image.Height, TextureFormat.RGB24, false);
        }

        texture.LoadRawTextureData(image.GetRawData());
        texture.Apply();
    }

    public static void GetTextures(NamedCameraTexture[] camIds)
    {
        lock (m_cameraStates)
        {
            foreach (var cameraImage in camIds)
            {
                if (cameraImage.id == null)
                {
                    continue;
                }

                if (!m_cameraStates.TryGetValue(cameraImage.id.Value, out var state))
                {
                    continue;
                }

                if (state is not CameraState.Capturing capturing)
                {
                    continue;
                }

                var image = capturing.image;

                GetTexture(image, ref cameraImage.texture);

                cameraImage.image ??= new Mat();

                capturing.image.CopyTo(cameraImage.image);
                cameraImage.frameTime = capturing.frameTime;
            }
        }
    }
}

using System;
using System.Collections;
using System.Collections.Generic;
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

    struct Capturing : IState {
        public VideoCapture capture;
        public double delta;
        public int exposure;

        public DateTime frameTime;
        public Mat image;
    }

    struct Stopped : IState { }

    struct Error : IState {
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
    public Texture2D image;
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
            Dictionary<int, CameraImage> cameraImages = new();

            while (m_threadRunning)
            {
                var startTime = DateTime.Now;

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
                        case RequestedCameraState.Capture capture:
                        {
                            switch (innerCameraState)
                            {
                                case Stopped or Error: {
                                    try
                                    {
                                        var newCamera = new VideoCapture(camId, VideoCapture.API.DShow);
                                        newCamera.Set(CapProp.Exposure, capture.exposure);

                                        innerCameraState = new InnerCameraState.Capturing
                                        {
                                            capture = newCamera, delta = 0, frameTime = new DateTime(),
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
                                        if (capturing.exposure != capture.exposure)
                                        {
                                            capturing.capture.Set(CapProp.Exposure, capture.exposure);
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
                                        capturing.capture.Stop();
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
                foreach (var camId in innerCameraStates.Keys) 
                {
                    try
                    {
                        if (innerCameraStates[camId] is Capturing capturing)
                        {
                            capturing.capture.Grab();
                        }
                    }
                    catch (Exception e)
                    {
                        innerCameraStates[camId] = new Error { message = e.Message };
                    }
                }

                foreach (var (camId, cameraState) in innerCameraStates)
                {
                    if (cameraState is not Capturing capturing)
                    {
                        continue;
                    }

                    capturing.delta = (frameTime - capturing.frameTime).TotalSeconds;
                    capturing.frameTime = frameTime;
                    capturing.capture.Read(capturing.image);
                }

                var anyActive = false;
                lock (m_cameraStates) {
                    foreach (var (camId, cameraState) in innerCameraStates)
                    {
                        var currentState = m_cameraStates.GetValueOrDefault(camId, null);
                        switch (cameraState)
                        {
                            case Capturing innerCapturing:
                            {
                                anyActive = true;
                                switch (currentState)
                                {
                                    case CameraState.Capturing capturing:
                                    {
                                        capturing.delta = innerCapturing.delta;
                                        capturing.frameTime = innerCapturing.frameTime;
                                        innerCapturing.image.CopyTo(capturing.image);
                                    } break;
                                    default:
                                    {
                                        var image = new Mat();
                                        innerCapturing.image.CopyTo(image);

                                        currentState = new CameraState.Capturing {
                                            delta = innerCapturing.delta,
                                            frameTime = innerCapturing.frameTime,
                                            image = image
                                        };
                                    } break;
                                }
                            } break;
                            case Stopped:
                            {
                                if (currentState is not CameraState.Stopped)
                                {
                                    currentState = new CameraState.Stopped();
                                }
                            } break;
                            case Error innerError:
                            {
                                currentState = new CameraState.Error{ message = innerError.message };
                            } break;
                        }

                        m_cameraStates[camId] = currentState;
                    }
                }

                if (!anyActive) {
                    break;
                }

                var delta = DateTime.Now - startTime;

                var remainingSleepTime = delta - new TimeSpan(0, 0, 0, 0, 1000 / m_fps);

                if (remainingSleepTime.Milliseconds > 0) {
                    Thread.Sleep(remainingSleepTime);
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

    public static void StartCameraCapture(int camId, int exposure)
    {
        lock (m_requestedCameraStates) lock (m_cameraStates)
        {
            m_requestedCameraStates[camId] = new Capture { exposure = exposure };
            m_cameraStates[camId] = new StartCapturing();
        }

        if (m_cameraImagesReadingTask is { Status: TaskStatus.Faulted or TaskStatus.Canceled or TaskStatus.RanToCompletion } or null) {
            m_threadRunning = true;
            m_cameraImagesReadingTask = CreateCameraCaptureTask();
        }
    }

    public static void SetCameraExposure(int camId, int exposure)
    {
        lock (m_requestedCameraStates) {
            m_requestedCameraStates[camId] = new Capture { exposure = exposure };
        }
    }

    public static void StopCameraCapture(int camId) {
        lock (m_requestedCameraStates) {
            m_requestedCameraStates[camId] = new StopCapture();
        }
    }

    public static CameraState.IState GetCameraState(int camId)
    {
        lock (m_cameraStates)
        {
            return m_cameraStates.GetValueOrDefault(camId, null);
        }
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

                cameraImage.image ??= new Texture2D(image.Width, image.Height, TextureFormat.RGB24, false);

                if (cameraImage.image.format != TextureFormat.RGB24 || cameraImage.image.width != image.Width ||
                    cameraImage.image.height != image.Height)
                {
                    cameraImage.image.Reinitialize(image.Width, image.Height, TextureFormat.RGB24, false);
                }

                cameraImage.image.LoadRawTextureData(image.GetRawData());
                cameraImage.image.Apply();

                cameraImage.frameTime = capturing.frameTime;
            }
        }
    }
}

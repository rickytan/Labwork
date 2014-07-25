// Depth2PointCloud.cpp : 定义控制台应用程序的入口点。
//

#include <windows.h>

#include <iostream>
#include <string>

#include <NuiApi.h>
#include <NuiSensorChooser.h>
#include <NuiKinectFusionApi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mesh.h"

using namespace std;
using namespace cv;


class OpenCVApp {
public:
    OpenCVApp()
        : m_pSensor(nullptr)
        , m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE)
        , m_hProcessStopEvent(INVALID_HANDLE_VALUE)
        , m_hDepthImageMutex(INVALID_HANDLE_VALUE)
        , m_hDepthStreamHandle(INVALID_HANDLE_VALUE)
        , m_hProcessThread(INVALID_HANDLE_VALUE)
        , m_imageResolution(NUI_IMAGE_RESOLUTION_640x480)
        , m_depthBufferSize(0)
        , m_depthBufferPitch(0)
        , m_pixelToMeterScale(0.f)
        , m_pDepthBuffer(nullptr)
        , m_pCameraPoseFinder(nullptr)
        , m_bNearMode(FALSE)
    {
    }
    ~OpenCVApp() {
        UnInitialize();
    }

    bool InitializeSensor() {
        HRESULT hr = m_sensorChooser.GetSensor(NUI_INITIALIZE_FLAG_USES_DEPTH, &m_pSensor);
        if (FAILED(hr)) {
            return false;
        }
        m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, L"Depth Image Event");
        hr = m_pSensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_DEPTH,
            m_imageResolution,
            0,
            2,
            m_hNextDepthFrameEvent,
            &m_hDepthStreamHandle
            );
        if (FAILED(hr))
            return false;

        DWORD width = 0, height = 0;
        NuiImageResolutionToSize(m_imageResolution, width, height);
        m_pixelToMeterScale = tanf(NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV * M_PI / 180.f * 0.5f) / (width * .5f);

        NuiSetDeviceStatusCallback(OpenCVApp::StatusCallbackProc, this);

        m_hDepthImageMutex = CreateMutex(NULL, FALSE, NULL);
        m_hProcessStopEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

        return true;
    }

    void UnInitialize() {
        if (m_pSensor) {
            m_pSensor->NuiShutdown();
            m_pSensor->Release();
            m_pSensor = NULL;
        }

        if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE) {
            CloseHandle(m_hNextDepthFrameEvent);
        }

        if (m_hDepthImageMutex != INVALID_HANDLE_VALUE)
            CloseHandle(m_hDepthImageMutex);

        if (m_hProcessStopEvent != INVALID_HANDLE_VALUE)
            CloseHandle(m_hProcessStopEvent);

        if (m_hProcessThread != INVALID_HANDLE_VALUE)
            CloseHandle(m_hProcessThread);

        if (m_pCameraPoseFinder) {
            m_pCameraPoseFinder->Release();
            m_pCameraPoseFinder = NULL;
        }

        if (m_pDepthBuffer) {
            delete[] m_pDepthBuffer;
        }

    }

    HRESULT UpdateDepthFrame() {
        NUI_IMAGE_FRAME imageFrame;

        HRESULT hr = m_pSensor->NuiImageStreamGetNextFrame(m_hDepthStreamHandle, 0, &imageFrame);
        if (FAILED(hr))
            return hr;
        // Lock frame texture to allow for copy
        INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
        
        //BOOL nearModeOperational = FALSE;
        //hr = m_pSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_hDepthStreamHandle, &imageFrame, &nearModeOperational, &pTexture);
        //if (FAILED(hr))
        //    return hr;

        NUI_LOCKED_RECT lockedRect;
        pTexture->LockRect(0, &lockedRect, NULL, 0);

        // Check if image is valid
        if (lockedRect.Pitch != 0)
        {
            // Copy image information into buffer
            BYTE* pBuffer = lockedRect.pBits;
            INT size = lockedRect.size;
            INT pitch = lockedRect.Pitch;

            // Only reallocate memory if the buffer size has changed
            if (size != m_depthBufferSize)
            {
                delete[] m_pDepthBuffer;
                m_pDepthBuffer = new BYTE[size];
                m_depthBufferSize = size;
            }
            memcpy_s(m_pDepthBuffer, size * sizeof(*m_pDepthBuffer), pBuffer, size);

            m_depthBufferPitch = pitch;
        }

        pTexture->UnlockRect(0);
        hr = m_pSensor->NuiImageStreamReleaseFrame(m_hDepthStreamHandle, &imageFrame);
        return hr;
    }

    HRESULT GetDepthImage(cv::Mat *depthImage) {
        // Check if image is valid
        if (m_depthBufferPitch == 0)
        {
            return E_NUI_FRAME_NO_DATA;
        }

        DWORD depthHeight, depthWidth;
        NuiImageResolutionToSize(m_imageResolution, depthWidth, depthHeight);

        // Copy image information into Mat
        USHORT* pBufferRun = reinterpret_cast<USHORT*>(m_pDepthBuffer);
        
        for (UINT y = 0; y < depthHeight; ++y)
        {
            // Get row pointer for depth Mat
            float* pDepthRow = depthImage->ptr<float>(y);

            for (UINT x = 0; x < depthWidth; ++x)
            {
                /*
                USHORT depth = pBufferRun[y * depthWidth + x].depth << NUI_IMAGE_PLAYER_INDEX_SHIFT;
                float depthf = 1.0f;
                if (m_bNearMode) {
                    depth = (depth < NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE || depth > NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE) ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : depth;
                    depthf = (1.0 * (depth - NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE) / (NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE - NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE));
                }
                else {
                    depth = (depth < NUI_IMAGE_DEPTH_MINIMUM || depth > NUI_IMAGE_DEPTH_MAXIMUM) ? NUI_IMAGE_DEPTH_MINIMUM : depth;
                    depthf = (1.0 * (depth - NUI_IMAGE_DEPTH_MINIMUM) / (NUI_IMAGE_DEPTH_MAXIMUM - NUI_IMAGE_DEPTH_MINIMUM));
                }
                pDepthRow[x] = (USHORT)(depthf * (USHORT)-1);
                */
                pDepthRow[x] = 0.001f * NuiDepthPixelToDepth(pBufferRun[y * depthWidth + x]);   // Depth in Meters
            }
        }

        return S_OK;
    }

    DWORD ProcessFrame()
    {
        bool stopProcess = false;
        while (!stopProcess)
        {
            HANDLE handles[] = { m_hProcessStopEvent, m_hNextDepthFrameEvent };

            DWORD eventId = WaitForMultipleObjects(2, handles, FALSE, INFINITE);
            switch (eventId)
            {
            case WAIT_OBJECT_0:
                stopProcess = true;
                break;
            case WAIT_OBJECT_0 + 1: {
                HRESULT hr = UpdateDepthFrame();
                if (FAILED(hr))
                    continue;
                
                WaitForSingleObject(m_hDepthImageMutex, INFINITE);
                hr = GetDepthImage(&m_depthImage);
                cv::flip(m_depthImage, m_depthImage, 1);
                ReleaseMutex(m_hDepthImageMutex);
                if (FAILED(hr))
                     continue;

            }
                break;
            default:
                break;
            }

        }
        return 0;
    }

    static void CALLBACK StatusCallbackProc(HRESULT hrStatus, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceName, void* pUserData)
    {
        OpenCVApp *app = reinterpret_cast<OpenCVApp *>(pUserData);

    }

    static DWORD CALLBACK ProcessThread(LPVOID lpParam) {
        OpenCVApp *app = reinterpret_cast<OpenCVApp *>(lpParam);
        return app->ProcessFrame();
    }

    void SaveToPLY()
    {
        Mesh mesh;
        int imageWidth = m_depthImage.cols;
        int imageHeight = m_depthImage.rows;
        int imageHalfWidth = imageWidth / 2;
        int imageHalfHeight = imageHeight / 2;
        for (int h = 0; h < m_depthImage.rows; ++h)
        {
            for (int w = 0; w < m_depthImage.cols; ++w)
            {
                float depth = m_depthImage.at<float>(h, w);
                if (depth == 0.f)
                    continue;
                float x = 1.0f * (w - imageHalfWidth) * m_pixelToMeterScale * depth;
                float y = -1.0f * (h - imageHalfHeight) * m_pixelToMeterScale * depth;
                vcg::tri::Allocator<Mesh>::VertexIterator vi = vcg::tri::Allocator<Mesh>::AddVertices(mesh, 1);
                vi->P() = Mesh::CoordType(x, y, -depth);
            }
        }
        vcg::tri::io::Exporter<Mesh>::Save(mesh, "point_cloud.ply");
    }

    void SetNearMode(BOOL bNearmode)
    {
        if (m_bNearMode != bNearmode)
        {
            m_bNearMode = bNearmode;
            m_pSensor->NuiImageStreamSetImageFrameFlags(m_hDepthStreamHandle, bNearmode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
        }
    }

    void Run() {
        m_hProcessThread = CreateThread(NULL, 0, &OpenCVApp::ProcessThread, this, 0, NULL);

        int keycode = 0;

        DWORD width, height;
        NuiImageResolutionToSize(m_imageResolution, width, height);
        m_depthImage.create(height, width, CV_32F);

        while (keycode = cv::waitKey(10))
        {
            if (keycode == 27) {
                SetEvent(m_hProcessStopEvent);
                break;
            }
            if (keycode == 'n') {
                SetNearMode(TRUE);
                cout << "Enable Near mode!" << endl;
            }
            if (keycode == 'N') {
                SetNearMode(FALSE);
                cout << "Disable Near mode!" << endl;
            }
            WaitForSingleObject(m_hDepthImageMutex, INFINITE);
            cv::imshow("Depth", m_depthImage);
            if (keycode == 's' ||
                keycode == 'S') {
                SaveToPLY();
            }
            ReleaseMutex(m_hDepthImageMutex);
        }
        WaitForSingleObject(m_hProcessThread, INFINITE);
    }

protected:
    cv::Mat m_depthImage;
    float m_pixelToMeterScale;

    BOOL m_bNearMode;

    NuiSensorChooser m_sensorChooser;
    INuiSensor *m_pSensor;

    NUI_IMAGE_RESOLUTION m_imageResolution;
    INuiFusionCameraPoseFinder *m_pCameraPoseFinder;

    HANDLE m_hNextDepthFrameEvent;
    HANDLE m_hProcessStopEvent;
    HANDLE m_hDepthImageMutex;

    HANDLE m_hProcessThread;

    HANDLE m_hDepthStreamHandle;

    INT m_depthBufferSize;
    BYTE *m_pDepthBuffer;
    INT m_depthBufferPitch;
};

int main(int argc, char* argv[])
{
    OpenCVApp app;
    if (app.InitializeSensor())
        app.Run();

    return 0;
}


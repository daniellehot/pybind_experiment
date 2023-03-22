#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvBuffer.h>
#include <PvPipeline.h>
#include <PvConfigurationReader.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

//PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 16 )

#define FILE_NAME ( "/home/daniel/jaiGO/myJaiStream/config.pvxml" )
#define DEVICE_CONFIGURATION_TAG ( "DeviceConfiguration" )
#define STREAM_CONFIGURAITON_TAG ( "StreamConfiguration" )

class JaiGo {
    private:
        PvString ConnectionID;
        PvDevice *Device = NULL;

        PvStream *Stream = NULL;
        PvPipeline *Pipeline = NULL;
        
        PvGenCommand *StartCommand = NULL;
        PvGenCommand *StopCommand = NULL;
        PvGenFloat *FrameRate = NULL;
        PvGenFloat *Bandwidth = NULL;

        //Find and Connect
        bool FindDevice( PvString *aConnectionID);
        PvDevice *ConnectToDevice( const PvString &aConnectionID );
        //StartStream()
        PvStream *OpenStream( const PvString &aConnectionID );
        //void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
        bool LoadDeviceAndStreamConfiguration(PvDevice *aDevice, PvStream *aStream);
        PvPipeline *CreatePipeline( PvDevice *aDevice, PvStream *aStream );
        //GetImages()
        void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );

    public:
        bool Connected = false;
        bool Streaming = false;

        //Stream statistics
        double FrameRateVal = 0.0;
        double BandwidthVal = 0.0;
        //Image
        int ImgWidth;
        int ImgHeight;
        cv::Mat Img;
        void FindAndConnect();
        void StartStream();
        bool GetImage();
        void CloseAndDisconnect();
};

void JaiGo::FindAndConnect()
{
    PvString lConnectionID;
    if (JaiGo::FindDevice(&lConnectionID))
    {   
        PvDevice *lDevice = NULL;
        lDevice = ConnectToDevice(lConnectionID);
        if (lDevice != NULL)
        {
            cout<<"Connection and Device Acquired"<<endl;
            this->ConnectionID = lConnectionID;
            this->Device = lDevice;
            this->Connected = true;
        }
    }
}   

bool JaiGo::FindDevice( PvString *aConnectionID)
{
    PvResult lResult;
    const PvDeviceInfo *lSelectedDI = NULL;
    PvSystem lSystem;

    cout << endl << "Detecting devices." << endl;
    
    lSystem.Find();
    vector<const PvDeviceInfo *> lDIVector;
    for ( uint32_t i = 0; i < lSystem.GetInterfaceCount(); i++ )
    {
        const PvInterface *lInterface = dynamic_cast<const PvInterface *>( lSystem.GetInterface( i ) );
        if ( lInterface != NULL )
        {
            //cout << "   " << lInterface->GetDisplayID().GetAscii() << endl;
            for ( uint32_t j = 0; j < lInterface->GetDeviceCount(); j++ )
            {
                const PvDeviceInfo *lDI = dynamic_cast<const PvDeviceInfo *>( lInterface->GetDeviceInfo( j ) );
                if ( lDI != NULL )
                {
                    lDIVector.push_back( lDI );
                    //cout << "[" << ( lDIVector.size() - 1 ) << "]" << "\t" << lDI->GetDisplayID().GetAscii() << endl;
                }					
            }
        }
    }
    
    if ( lDIVector.size() == 0)
    {
        cout << "No device found!" << endl;
        return false;
    } else 
    {
        cout << "Found "<< lDIVector.size() << " devices. \n";
        for (uint i = 0; i < lDIVector.size(); i++)
        {
            cout << i << ") " <<lDIVector[i]->GetDisplayID().GetAscii()<<endl;
        }
        cout << "Connecting to " << lDIVector.back()->GetDisplayID().GetAscii()<<endl;
        *aConnectionID = lDIVector.back()->GetConnectionID();
        return true;
    }
}

PvDevice* JaiGo::ConnectToDevice( const PvString &aConnectionID )
{
    PvDevice *lDevice;
    PvResult lResult;
    lDevice = PvDevice::CreateAndConnect( aConnectionID, &lResult );
    if ( lDevice == NULL )
    {
        cout << "Unable to connect to device: "
        << lResult.GetCodeString().GetAscii()
        << " ("
        << lResult.GetDescription().GetAscii()
        << ")" << endl;
    }
 
    return lDevice;
}

void JaiGo::StartStream()
{
    PvStream *lStream = NULL;
    lStream = JaiGo::OpenStream(this->ConnectionID);
    if ( lStream != NULL )
    {
        this->Stream = lStream;
        //if (JaiGo::LoadDeviceAndStreamConfiguration(this->Device, this->Stream))
        if (true)
        {
            PvPipeline *lPipeline = NULL;
            lPipeline = JaiGo::CreatePipeline(this->Device, lStream );
            if( lPipeline )
            {
                this->Pipeline = lPipeline;

                // Get device parameters need to control streaming
                PvGenParameterArray *lDeviceParams = this->Device->GetParameters();

                // Map the GenICam AcquisitionStart and AcquisitionStop commands
                this->StartCommand = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
                this->StopCommand = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

                // Note: the pipeline must be initialized before we start acquisition
                cout << "Starting pipeline" << endl;
                this->Pipeline->Start();

                // Get stream parameters
                PvGenParameterArray *lStreamParams = this->Stream->GetParameters();

                // Map a few GenICam stream stats counters
                this->FrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
                this->Bandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );

                // Enable streaming and send the AcquisitionStart command
                cout << "Enabling streaming and sending AcquisitionStart command." << endl;
                this->Device->StreamEnable();
                this->StartCommand->Execute();

                this->Streaming = true;
            }
        }
    }

}

PvStream* JaiGo::OpenStream( const PvString &aConnectionID )
{
    PvStream *lStream;
    PvResult lResult;

    // Open stream to the GigE Vision or USB3 Vision device
    cout << "Opening stream from device." << endl;
    lStream = PvStream::CreateAndOpen( aConnectionID, &lResult );
    if ( lStream == NULL )
    {
        cout << "Unable to stream from device." << endl;
    }

    return lStream;
}

bool JaiGo::LoadDeviceAndStreamConfiguration(PvDevice *aDevice, PvStream *aStream)
{
    PvConfigurationReader lReader;
    // Load all the information into a reader.
    cout << "Load information and configuration" << endl;
    lReader.Load( FILE_NAME );

    cout << "Restore configuration for a device with the configuration name" << endl;
    PvResult lResult = lReader.Restore( DEVICE_CONFIGURATION_TAG, aDevice);
    if ( !lResult.IsOK() )
    {
        cout << lResult.GetCodeString().GetAscii() << endl;
        return false;
    }

    cout << "Restore configuration for a stream with the configuration name" << endl;
    lResult = lReader.Restore( STREAM_CONFIGURAITON_TAG, aStream);
    if ( !lResult.IsOK() )
    {
        cout << lResult.GetCodeString().GetAscii() << endl;
        return false;
    }

    return true;
}

PvPipeline* JaiGo::CreatePipeline( PvDevice *aDevice, PvStream *aStream )
{
    // Create the PvPipeline object
    PvPipeline* lPipeline = new PvPipeline( aStream );

    if ( lPipeline != NULL )
    {        
        // Reading payload size from device
        uint32_t lSize = aDevice->GetPayloadSize();
    
        // Set the Buffer count and the Buffer size
        lPipeline->SetBufferCount( BUFFER_COUNT );
        lPipeline->SetBufferSize( lSize );
    }
    
    return lPipeline;
}

bool JaiGo::GetImage()
{ 
    //while loop scope
    bool receivedImage = false;
    PvBuffer *lBuffer = NULL;
    PvResult lOperationResult;

    // Retrieve next buffer
    PvResult lResult = this->Pipeline->RetrieveNextBuffer( &lBuffer, 1000, &lOperationResult );
    if ( lResult.IsOK() )
    {
        if ( lOperationResult.IsOK() )
        {
            //
            // We now have a valid buffer. This is where you would typically process the buffer.
            // -----------------------------------------------------------------------------------------
            // ...
            this->FrameRate->GetValue( this->FrameRateVal );
            this->Bandwidth->GetValue( this->BandwidthVal );
            this->BandwidthVal /= 1000000.0; //Conversion to Mb 

            if (lBuffer->GetPayloadType() == PvPayloadTypeImage)
            {
                this->ImgWidth = lBuffer->GetImage()->GetWidth();
                this->ImgHeight = lBuffer->GetImage()->GetHeight();
                
                cv::Mat cvImg = cv::Mat(lBuffer->GetImage()->GetHeight(), lBuffer->GetImage()->GetWidth(), CV_8U, lBuffer->GetDataPointer());
                cv::Mat cvColorImg;
                cvtColor(cvImg, cvColorImg, cv::COLOR_BayerRG2RGB);
                this->Img = cvColorImg;
                receivedImage = true;
                //cv::imshow("test", cvColorImg);
                //cv::waitKey(1);
            } 
            else
            {
                cout<< "Incorrect payload type. Start stream with PvPayloadTypeImage. Current type is " << lBuffer->GetPayloadType() << endl;
            }
        }
        else
        {
            // Non OK operational result
            cout << lOperationResult.GetCodeString().GetAscii() << endl;
        }

        // Release the buffer back to the pipeline
        this->Pipeline->ReleaseBuffer( lBuffer );
    }
    else
    {
        // Retrieve buffer failure
        cout << lResult.GetCodeString().GetAscii() << "\r";
    }

    if (receivedImage)
    {
        return true;
    } 
    else
    {
        return false;
    }
}


void JaiGo::CloseAndDisconnect()
{   
    if (this->StopCommand != NULL)
    {
        cout << "Sending AcquisitionStop command to the device" << endl;
        this->StopCommand->Execute();
    }

    if (this->Device != NULL)
    {
        // Disable streaming on the device
        cout << "Disable streaming on the controller." << endl;
        this->Device->StreamDisable();
    }

    if (this->Pipeline != NULL)
    {
        // Stop the pipeline
        cout << "Stop pipeline" << endl;
        this->Pipeline->Stop();

        cout << "Deleting pipeline" << endl;
        delete this->Pipeline;
    }

    if (this->Stream != NULL)
    {
        cout << "Closing stream" << endl;
        this->Stream->Close();
        PvStream::Free( this->Stream );
        //this->Streaming = false;
    }

    if (this->Device != NULL)
    {
        cout << "Disconnecting device" << endl;
        this->Device->Disconnect();
        PvDevice::Free( this->Device );
        this->Connected = false;
    }

}

int main() {
    JaiGo camera;
    camera.FindAndConnect();
    if (camera.Connected) 
    {
        camera.StartStream();
        if (camera.Streaming) 
        {
            int i = 0;
            while (camera.Streaming)
            {   
                i++;
                if (i == 100) camera.Streaming = false;

                if ( camera.GetImage() )
                {
                    cout<<"Image width "<<camera.ImgWidth<<" Height "<<camera.ImgHeight<<endl;
                    cout<<"Streaming Bandwidth "<<camera.BandwidthVal<<" Mb/s FPS "<< camera.FrameRateVal<<endl;
                    imshow("img", camera.Img);
                    cv::waitKey(1);
                };
               
            };
        }
    }
    camera.CloseAndDisconnect();

    return 0;
}

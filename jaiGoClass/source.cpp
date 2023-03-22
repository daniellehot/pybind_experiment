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

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 16 )

#define FILE_NAME ( "/home/daniel/jaiGO/myJaiStream/config.pvxml" )
#define DEVICE_CONFIGURATION_TAG ( "DeviceConfiguration" )
#define STREAM_CONFIGURAITON_TAG ( "StreamConfiguration" )

class JaiGo {
    private:
        PvString ConnectionID;
        PvDevice *Device;
        bool Connected = false;

        PvStream *Stream;
        PvPipeline *Pipeline;
        bool Streaming = false;

        PvGenCommand *StartCommand;
        PvGenCommand *StopCommand;
        
        double FrameRateVal = 0.0; 
        double BandwidthVal = 0.0;

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
        void FindAndConnect();
        void StartStream();
        cv::Mat* GetImage();
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
        if (JaiGo::LoadDeviceAndStreamConfiguration(this->Device, this->Stream))
        {
            PvPipeline *lPipeline = NULL;
            lPipeline = JaiGo::CreatePipeline(this->Device, lStream );
            if( lPipeline )
            {
                this->Pipeline = lPipeline;
                this->Streaming = true;

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
                PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
                PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );

                // Enable streaming and send the AcquisitionStart command
                cout << "Enabling streaming and sending AcquisitionStart command." << endl;
                this->Device->StreamEnable();
                this->StartCommand->Execute();
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

cv::Mat* JaiGo::GetImage()
{
    
    while ( !PvKbHit() )
    {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;

        // Retrieve next buffer
        PvResult lResult = aPipeline->RetrieveNextBuffer( &lBuffer, 1000, &lOperationResult );
        if ( lResult.IsOK() )
        {
            if ( lOperationResult.IsOK() )
            {
                //
                // We now have a valid buffer. This is where you would typically process the buffer.
                // -----------------------------------------------------------------------------------------
                // ...
                lFrameRate->GetValue( this->FrameRateVal );
                lBandwidth->GetValue( this->BandwidthVal );


                if (lBuffer->GetPayloadType() == PvPayloadTypeImage)
                {
                    cout << "W: " << dec << lBuffer->GetImage()->GetWidth() << " H: " << lBuffer->GetImage()->GetHeight();
                    cout << "  " << FrameRateVal << " FPS  " << ( BandwidthVal / 1000000.0 ) << " Mb/s   \r";
                    cv::Mat openCvImage = cv::Mat(lBuffer->GetImage()->GetHeight(), lBuffer->GetImage()->GetWidth(), CV_8U, lBuffer->GetDataPointer());
                    cv::Mat colorImg;
                    cvtColor(openCvImage, colorImg, cv::COLOR_BayerRG2RGB);
                    imshow("Img", colorImg);
                    cv::waitKey(1);
                } 
                else
                {
                    cout<< "Incorrect payload type. Start stream with PvPayloadTypeImage. Current type is " << lBuffer->GetPayloadType();
                }
            }
            else
            {
                // Non OK operational result
                cout << lOperationResult.GetCodeString().GetAscii() << "\r";
            }

            // Release the buffer back to the pipeline
            aPipeline->ReleaseBuffer( lBuffer );
        }
        else
        {
            // Retrieve buffer failure
            cout << lResult.GetCodeString().GetAscii() << "\r";
        }
    }

    PvGetChar(); // Flush key buffer for next stop.
    cout << endl << endl;

    // Tell the device to stop sending images.
    cout << "Sending AcquisitionStop command to the device" << endl;
    lStop->Execute();

    // Disable streaming on the device
    cout << "Disable streaming on the controller." << endl;
    aDevice->StreamDisable();

    // Stop the pipeline
    cout << "Stop pipeline" << endl;
    aPipeline->Stop();
}


void JaiGo::CloseAndDisconnect()
{   
    cout << "Deleting pipeline" << endl;
    delete this->Pipeline;

    cout << "Closing stream" << endl;
    this->Stream->Close();
    PvStream::Free( this->Stream );
    this->Streaming = false;

    cout << "Disconnecting device" << endl;
    this->Device->Disconnect();
    PvDevice::Free( this->Device );
    this->Connected = false;
}

int main() {
    JaiGo camera;
    camera.FindAndConnect();
    camera.StartStream();
    camera.CloseAndDisconnect();
    return 0;
}

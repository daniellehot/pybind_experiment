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

#define FILE_NAME ( "config.pvxml" )
#define DEVICE_CONFIGURATION_TAG ( "DeviceConfiguration" )
#define STREAM_CONFIGURAITON_TAG ( "StreamConfiguration" )

class JaiGo {
    private:
        string name;
        PvString lConnectionID;
    public:
        bool SelectDevice( PvString *aConnectionID, PvDeviceInfoType *aType = NULL );
        PvDevice *ConnectToDevice( const PvString &aConnectionID );
        PvStream *OpenStream( const PvString &aConnectionID );
        void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
        bool LoadDeviceAndStreamConfiguration(PvDevice *aDevice, PvStream *aStream);
        PvPipeline *CreatePipeline( PvDevice *aDevice, PvStream *aStream );
        void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );
};

bool JaiGo::SelectDevice( PvString *aConnectionID, PvDeviceInfoType *aType)
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
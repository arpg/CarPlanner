#include <CarPlanner/ROSCommander.h>

///////////////////////////////////////////////////////
ROSCommander::ROSCommander() : m_pThread(0), m_bIsStarted(false)
{
    m_MochaPort = 1643;
    m_ComPort = 1642;
    m_CarPort = 1640;

    if ( ( sockFD = socket( AF_INET, SOCK_DGRAM, 0 ) ) < 0 ) LOG(ERROR) << "Could not create socket";

    memset( (char*)&mochAddr, 0, addrLen );
    mochAddr.sin_family = AF_INET;
    mochAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    mochAddr.sin_port = htons( m_MochaPort );

    if ( ::bind( sockFD, (struct sockaddr*)&mochAddr, addrLen ) < 0 ) LOG(ERROR) << "Could not bind socket to port " << m_MochaPort;

    memset( (char*)&comAddr, 0, addrLen );
    comAddr.sin_family = AF_INET;
    comAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    comAddr.sin_port = htons( m_ComPort );

    memset( (char*)&carAddr, 0, addrLen );
    carAddr.sin_family = AF_INET;
    carAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    carAddr.sin_port = htons( m_CarPort );
}

/////////////////////////////////////////////////////////////
ROSCommander::~ROSCommander()
{
    Stop();
}

////////////////////////////////////////////////////////////
void ROSCommander::Start()
{
    LOG(INFO) << "Starting ROSCommander thread.";

    if( m_bIsStarted == true ) {
        LOG(ERROR) << "ROSCommander thread already started";
        //throw MochaException("The Localizer thread has already started.");
        return;
    }

    // m_pThread = new boost::thread([this] () { ROSCommander::_ThreadFunction(this); } );
    m_bIsStarted = true;

    //LOG(INFO) << "ROSCommander thread started.";
}

//////////////////////////////////////////////////////////////
void ROSCommander::Stop()
{
    if( m_bIsStarted == false ) {
        LOG(ERROR) << "No thread running!";
        //throw MochaException("No thread is running!");
        return;
    }

    m_pThread->interrupt();
    m_pThread->join();

    close(sockFD);

    m_bIsStarted = false;

    LOG(INFO) << "ROSCommander thread stopped.";
}

///////////////////////////////////////////////////////////////
void ROSCommander::SendCommand(ControlCommand command, bool sil/*=false*/)
{
    hal::CommanderMsg* Command = new hal::CommanderMsg();
    hal::WriteCommand( 0, std::max( std::min( command.m_dForce, 500.0 ), 0.0 ), command.m_dCurvature, command.m_dTorque, command.m_dT, command.m_dPhi, false, true/*true*/, Command );

    unsigned char buffer[Command->ByteSize() + 4];

    google::protobuf::io::ArrayOutputStream aos( buffer, sizeof(buffer) );
    google::protobuf::io::CodedOutputStream coded_output( &aos );
    coded_output.WriteVarint32( Command->ByteSize() );
    Command->SerializeToCodedStream( &coded_output );
    if ( sil ) {
        //send Command to BulletCarModel
        if ( sendto( sockFD, (char*)buffer, coded_output.ByteCount(), 0, (struct sockaddr*)&comAddr, addrLen ) < 0 ) { LOG(ERROR) << "Did not send message"; }
        //else { LOG(INFO) << "Sent Command"; }

    }
    else {
        //send Command to NinjaCar
        if ( sendto( sockFD, (char*)buffer, coded_output.ByteCount(), 0, (struct sockaddr*)&carAddr, addrLen ) < 0 ) LOG(ERROR) << "Did not send message";

    }
}
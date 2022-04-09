#include <CarPlanner/HALCommander.h>

///////////////////////////////////////////////////////
HALCommander::HALCommander()
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
HALCommander::~HALCommander()
{
    close(sockFD);
}

///////////////////////////////////////////////////////////////
void HALCommander::SendCommand(ControlCommand command, bool sil/*=false*/)
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
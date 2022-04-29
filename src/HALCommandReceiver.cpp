#include <CarPlanner/HALCommandReceiver.h>

///////////////////////////////////////////////////////
HALCommandReceiver::HALCommandReceiver()
{
    m_ComPort = 1642;
    m_MochaPort = 1643;

    if ( ( comSockFD = socket( AF_INET, SOCK_DGRAM, 0 ) ) < 0 ) LOG(ERROR) << "Could not create socket";
    if ( ( mochSockFD = socket( AF_INET, SOCK_DGRAM, 0 ) ) < 0 ) LOG(ERROR) << "Could not create socket";

    memset( (char*)&comAddr, 0, addrLen );
    comAddr.sin_family = AF_INET;
    comAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    comAddr.sin_port = htons( m_ComPort );

    memset( (char*)&mochAddr, 0, addrLen );
    mochAddr.sin_family = AF_INET;
    mochAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    mochAddr.sin_port = htons( m_MochaPort );

    if ( ::bind( comSockFD, (struct sockaddr*)&comAddr, addrLen ) < 0 ) LOG(ERROR) << "Could not bind socket to port " << m_ComPort;
    if ( ::bind( mochSockFD, (struct sockaddr*)&mochAddr, addrLen ) < 0 ) LOG(ERROR) << "Could not bind socket to port " << m_MochaPort;
}

/////////////////////////////////////////////////////////////
HALCommandReceiver::~HALCommandReceiver()
{
    close(comSockFD);
    close(mochSockFD);
}

/////////////////////////////////////////////////////////////////
void HALCommandReceiver::ReceiveCommand(ControlCommand &command, bool &bNoDelay, bool &bNoUpdate)
{
    int worldId;
    double force, curvature, dt, phi;
    Eigen::Vector3d torques;

    unsigned char comBuf[2048];
    unsigned char buf[2048];
    unsigned int comMsgSize = 0;

    hal::CommanderMsg* cmd = new hal::CommanderMsg();

    //UDP receive cmd from MochaGui
    int comRecvLen = recvfrom( comSockFD, comBuf, 2048, 0, (struct sockaddr*)&mochAddr, &addrLen );
    if ( comRecvLen > 0 ) {
        buf[comRecvLen] = 0;
        google::protobuf::io::ArrayInputStream ais( comBuf, comRecvLen );
        google::protobuf::io::CodedInputStream coded_input( &ais );
        coded_input.ReadVarint32( &comMsgSize );
        google::protobuf::io::CodedInputStream::Limit limit = coded_input.PushLimit( comMsgSize );
        cmd->ParseFromCodedStream( &coded_input );
        coded_input.PopLimit( limit );
        LOG(INFO) << "Received Command";
    }

    hal::ReadCommand( *cmd, &worldId, &force, &curvature, &torques, &dt, &phi, &bNoDelay, &bNoUpdate);
    command = ControlCommand(force, curvature, torques, dt, phi);
}
# 8 "./lib/NavX/SerialIO.cpp"
#include "SerialIO.h"

static const double IO_TIMEOUT_SECONDS = 1.0;

#define SERIALIO_DASHBOARD_DEBUG 

SerialIO::SerialIO( SerialPort::Port port_id,
                    uint8_t update_rate_hz,
                    bool processed_data,
                    IIOCompleteNotification *notify_sink,
                    IBoardCapabilities *board_capabilities ) {
    this->serial_port_id = port_id;
    ypr_update_data = {0};
    gyro_update_data = {0};
    ahrs_update_data = {0};;
    ahrspos_update_data = {0};
    board_id = {0};
    board_state = {0};
    this->notify_sink = notify_sink;
    this->board_capabilities = board_capabilities;
    serial_port = GetMaybeCreateSerialPort();
    this->update_rate_hz = update_rate_hz;
    if ( processed_data ) {
        update_type = MSGID_AHRSPOS_UPDATE;
    } else {
        update_type = MSGID_GYRO_UPDATE;
    }
}

SerialPort *SerialIO::ResetSerialPort()
{
    if (serial_port != 0) {
        try {
            delete serial_port;
        } catch (std::exception &ex) {

        }
        serial_port = 0;
    }
    serial_port = GetMaybeCreateSerialPort();
    return serial_port;
}

SerialPort *SerialIO::GetMaybeCreateSerialPort()
{
    if (serial_port == 0) {
        try {
            serial_port = new SerialPort(57600, serial_port_id);
            serial_port->SetReadBufferSize(256);
            serial_port->SetTimeout(1.0);
            serial_port->EnableTermination('\n');
            serial_port->Reset();
        } catch (std::exception &ex) {

            serial_port = 0;
        }
    }
    return serial_port;
}

void SerialIO::EnqueueIntegrationControlMessage(uint8_t action)
{
    next_integration_control_action = action;
    signal_transmit_integration_control = true;
}

void SerialIO::DispatchStreamResponse(IMUProtocol::StreamResponse& response) {
    board_state.cal_status = (uint8_t) (response.flags & NAV6_FLAG_MASK_CALIBRATION_STATE);
    board_state.capability_flags = (int16_t) (response.flags & ~NAV6_FLAG_MASK_CALIBRATION_STATE);
    board_state.op_status = 0x04;
    board_state.selftest_status = 0x07;
    board_state.accel_fsr_g = response.accel_fsr_g;
    board_state.gyro_fsr_dps = response.gyro_fsr_dps;
    board_state.update_rate_hz = (uint8_t) response.update_rate_hz;
    notify_sink->SetBoardState(board_state);


    if ( this->update_type == MSGID_AHRSPOS_UPDATE ) {
        if ( !board_capabilities->IsDisplacementSupported() ) {
            this->update_type = MSGID_AHRS_UPDATE;
            signal_retransmit_stream_config = true;
        }
    }
}

int SerialIO::DecodePacketHandler(char * received_data, int bytes_remaining) {
    int packet_length;
    long sensor_timestamp = 0;

    if ( (packet_length = IMUProtocol::decodeYPRUpdate(received_data, bytes_remaining, ypr_update_data)) > 0) {
        notify_sink->SetYawPitchRoll(ypr_update_data, sensor_timestamp);
    } else if ( ( packet_length = AHRSProtocol::decodeAHRSPosUpdate(received_data, bytes_remaining, ahrspos_update_data)) > 0) {
        notify_sink->SetAHRSPosData(ahrspos_update_data, sensor_timestamp);
    } else if ( ( packet_length = AHRSProtocol::decodeAHRSUpdate(received_data, bytes_remaining, ahrs_update_data)) > 0) {
        notify_sink->SetAHRSData(ahrs_update_data, sensor_timestamp);
    } else if ( ( packet_length = IMUProtocol::decodeGyroUpdate(received_data, bytes_remaining, gyro_update_data)) > 0) {
        notify_sink->SetRawData(gyro_update_data, sensor_timestamp);
    } else if ( ( packet_length = AHRSProtocol::decodeBoardIdentityResponse(received_data, bytes_remaining, board_id)) > 0) {
        notify_sink->SetBoardID(board_id);
    } else {
        packet_length = 0;
    }
    return packet_length;
}

void SerialIO::Run() {
    stop = false;
    bool stream_response_received = false;
    double last_stream_command_sent_timestamp = 0.0;
    double last_data_received_timestamp = 0;
    double last_second_start_time = 0;

    int partial_binary_packet_count = 0;
    int stream_response_receive_count = 0;
    int timeout_count = 0;
    int discarded_bytes_count = 0;
    int port_reset_count = 0;
    int updates_in_last_second = 0;
    int integration_response_receive_count = 0;

    try {
        serial_port->SetReadBufferSize(256);
        serial_port->SetTimeout(1.0);
        serial_port->EnableTermination('\n');
        serial_port->Flush();
        serial_port->Reset();
    } catch (std::exception &ex) {
        printf("SerialPort Run() Port Initialization Exception:  %s\n", ex.what());
    }

    char stream_command[256];
    char integration_control_command[256];
    IMUProtocol::StreamResponse response = {0};
    AHRSProtocol::IntegrationControl integration_control = {0};
    AHRSProtocol::IntegrationControl integration_control_response = {0};

    int cmd_packet_length = IMUProtocol::encodeStreamCommand( stream_command, update_type, update_rate_hz );
    try {
        serial_port->Reset();
        serial_port->Write( stream_command, cmd_packet_length );
        cmd_packet_length = AHRSProtocol::encodeDataGetRequest( stream_command, AHRS_DATA_TYPE::BOARD_IDENTITY, AHRS_TUNING_VAR_ID::UNSPECIFIED );
        serial_port->Write( stream_command, cmd_packet_length );
        serial_port->Flush();
        port_reset_count++;
        #ifdef SERIALIO_DASHBOARD_DEBUG
        SmartDashboard::PutNumber("navX Port Resets", (double)port_reset_count);
        #endif
        last_stream_command_sent_timestamp = Timer::GetFPGATimestamp();
    } catch (std::exception &ex) {
        printf("SerialPort Run() Port Send Encode Stream Command Exception:  %s\n", ex.what());
    }

    int remainder_bytes = 0;
    char received_data[256 * 3];
    char additional_received_data[256];
    char remainder_data[256];

    while (!stop) {
        try {




            if ( signal_transmit_integration_control ) {
                integration_control.action = next_integration_control_action;
                signal_transmit_integration_control = false;
                next_integration_control_action = 0;
                cmd_packet_length = AHRSProtocol::encodeIntegrationControlCmd( integration_control_command, integration_control );
                try {
                    serial_port->Write( integration_control_command, cmd_packet_length );
                } catch (std::exception ex) {
                    printf("SerialPort Run() IntegrationControl Send Exception:  %s\n", ex.what());
                }
            }

            if ( !stop && ( remainder_bytes == 0 ) && ( serial_port->GetBytesReceived() < 1 ) ) {
                delayMillis(1000/update_rate_hz);
            }

            int packets_received = 0;
            int bytes_read = serial_port->Read(received_data, sizeof(received_data));
            byte_count += bytes_read;





            if ( remainder_bytes > 0 ) {
                memcpy( received_data + bytes_read, remainder_data, remainder_bytes);
                bytes_read += remainder_bytes;
                remainder_bytes = 0;
            }

            if (bytes_read > 0) {
                last_data_received_timestamp = Timer::GetFPGATimestamp();
                int i = 0;

                while (i < bytes_read) {



                    int bytes_remaining = bytes_read - i;

                    if ( received_data[i] != PACKET_START_CHAR ) {

                        i++;
                        discarded_bytes_count++;
                        #ifdef SERIALIO_DASHBOARD_DEBUG
                            SmartDashboard::PutNumber("navX Discarded Bytes", (double)discarded_bytes_count);
                        #endif
                        continue;
                    } else {
                        if ( ( bytes_remaining > 2 ) &&
                                ( received_data[i+1] == BINARY_PACKET_INDICATOR_CHAR ) ) {

                            uint8_t total_expected_binary_data_bytes = received_data[i+2];
                            total_expected_binary_data_bytes += 2;
                            while ( bytes_remaining < total_expected_binary_data_bytes ) {




                                int additional_received_data_length =
                                        serial_port->Read(additional_received_data,sizeof(additional_received_data));
                                byte_count += additional_received_data_length;


                                 if ( additional_received_data_length > 0 ) {
                                    memcpy( received_data + bytes_remaining, additional_received_data, additional_received_data_length);
                                    bytes_remaining += additional_received_data_length;
                                 } else {

                                    i++;
                                    bytes_remaining--;
                                    partial_binary_packet_count++;
                                    #ifdef SERIALIO_DASHBOARD_DEBUG
                                        SmartDashboard::PutNumber("navX Partial Binary Packets", (double)partial_binary_packet_count);
                                    #endif
                                    continue;
                                }
                            }
                        }
                    }

                    int packet_length = DecodePacketHandler(received_data + i,bytes_remaining);
                    if (packet_length > 0) {
                        packets_received++;
                        update_count++;
                        last_valid_packet_time = Timer::GetFPGATimestamp();
                        updates_in_last_second++;
                        if ((last_valid_packet_time - last_second_start_time ) > 1.0 ) {
                            #ifdef SERIALIO_DASHBOARD_DEBUG
                                SmartDashboard::PutNumber("navX Updates Per Sec", (double)updates_in_last_second);
                            #endif
                            updates_in_last_second = 0;
                            last_second_start_time = last_valid_packet_time;
                        }
                        i += packet_length;
                    }
                    else
                    {
                        packet_length = IMUProtocol::decodeStreamResponse(received_data + i, bytes_remaining, response);
                        if (packet_length > 0) {
                            packets_received++;
                            DispatchStreamResponse(response);
                            stream_response_received = true;
                            i += packet_length;
                            stream_response_receive_count++;
                            #ifdef SERIALIO_DASHBOARD_DEBUG
                                SmartDashboard::PutNumber("navX Stream Responses", (double)stream_response_receive_count);
                            #endif
                        }
                        else {
                            packet_length = AHRSProtocol::decodeIntegrationControlResponse( received_data + i, bytes_remaining,
                                    integration_control_response );
                            if ( packet_length > 0 ) {

                                integration_response_receive_count++;
                                #ifdef SERIALIO_DASHBOARD_DEBUG
                                    SmartDashboard::PutNumber("navX Integration Control Response Count", integration_response_receive_count);
                                #endif
                                i += packet_length;
                            } else {



                                bool next_packet_start_found = false;
                                int x;
                                for ( x = 0; x < bytes_remaining; x++ ) {
                                    if ( received_data[i + x] != PACKET_START_CHAR) {
                                        x++;
                                    } else {
                                        i += x;
                                        bytes_remaining -= x;
                                        if ( x != 0 ) {
                                            next_packet_start_found = true;
                                        }
                                        break;
                                    }
                                }
                                bool discard_remainder = false;
                                if ( !next_packet_start_found && x == bytes_remaining ) {

                                    discard_remainder = true;
                                }
                                bool partial_packet = false;
                                if ( discard_remainder ) {

                                    i = bytes_remaining;
                                } else {
                                    if ( !next_packet_start_found ) {


                                        if ( ( bytes_remaining > 2 ) &&
                                                ( received_data[i+1] == BINARY_PACKET_INDICATOR_CHAR ) ) {

                                            int pkt_len = received_data[i+2];
                                            pkt_len += 2;
                                            if ( bytes_remaining >= pkt_len ) {
                                                bytes_remaining -= pkt_len;
                                                i += pkt_len;
                                                discarded_bytes_count += pkt_len;
                                                #ifdef SERIALIO_DASHBOARD_DEBUG
                                                    SmartDashboard::PutNumber("navX Discarded Bytes", (double)discarded_bytes_count);
                                                #endif
                                            } else {


                                                partial_packet = true;
                                            }
                                        } else {



                                            for ( x = 0; x < bytes_remaining; x++ ) {
                                                if ( received_data[i+x] == '\r') {
                                                    i += x+1;
                                                    bytes_remaining -= (x+1);
                                                    discarded_bytes_count += x+1;
                                                    if ( ( bytes_remaining > 0 ) && received_data[i] == '\n') {
                                                        bytes_remaining--;
                                                        i++;
                                                        discarded_bytes_count++;
                                                    }
                                                    #ifdef SERIALIO_DASHBOARD_DEBUG
                                                        SmartDashboard::PutNumber("navX Discarded Bytes", (double)discarded_bytes_count);
                                                    #endif
                                                    break;
                                                }


                                                if ( received_data[i+x] == '!') {
                                                    if ( x > 0 ) {
                                                        i += x;
                                                        bytes_remaining -= x;
                                                        discarded_bytes_count += x;
                                                        break;
                                                    } else {



                                                        if ( bytes_remaining < IMU_PROTOCOL_MAX_MESSAGE_LENGTH ) {

                                                            partial_packet = true;
                                                        } else {
                                                            i++;
                                                            bytes_remaining--;
                                                        }
                                                        break;
                                                    }
                                                }
                                            }
                                            if ( x == bytes_remaining ) {

                                                partial_packet = true;
                                            }
                                        }
                                    }
                                }
                                if ( partial_packet ) {
                                    if ( bytes_remaining > (int)sizeof(remainder_data)) {
                                        memcpy(remainder_data, received_data + i - sizeof(remainder_data), sizeof(remainder_data));
                                        remainder_bytes = sizeof(remainder_data);
                                    } else {
                                        memcpy(remainder_data, received_data + i, bytes_remaining);
                                        remainder_bytes = bytes_remaining;
                                    }
                                    i = bytes_read;
                                }
                            }
                        }
                    }
                }

                if ( ( packets_received == 0 ) && ( bytes_read == 256 ) ) {




                    serial_port->Flush();
                    serial_port->Reset();
                    port_reset_count++;
                    #ifdef SERIALIO_DASHBOARD_DEBUG
                        SmartDashboard::PutNumber("navX Port Resets", (double)port_reset_count);
                    #endif
                }

                bool retransmit_stream_config = false;
                if ( signal_retransmit_stream_config ) {
                    retransmit_stream_config = true;
                    signal_retransmit_stream_config = false;
                }




                if ( retransmit_stream_config ||
                        (!stream_response_received && ((Timer::GetFPGATimestamp() - last_stream_command_sent_timestamp ) > 3.0 ) ) ) {
                    cmd_packet_length = IMUProtocol::encodeStreamCommand( stream_command, update_type, update_rate_hz );
                    try {
                        ResetSerialPort();
                        last_stream_command_sent_timestamp = Timer::GetFPGATimestamp();
                        serial_port->Write( stream_command, cmd_packet_length );
                        cmd_packet_length = AHRSProtocol::encodeDataGetRequest( stream_command, AHRS_DATA_TYPE::BOARD_IDENTITY, AHRS_TUNING_VAR_ID::UNSPECIFIED );
                        serial_port->Write( stream_command, cmd_packet_length );
                        serial_port->Flush();
                    } catch (std::exception ex2) {
                        printf("SerialPort Run() Re-transmit Encode Stream Command Exception:  %s\n", ex2.what());
                    }
                }
                else {

                    if ( stream_response_received && ( serial_port->GetBytesReceived() == 0 ) ) {
                        delayMillis(1000/update_rate_hz);
                    }
                }






                if ( ( Timer::GetFPGATimestamp() - last_valid_packet_time ) > 1.0 ) {
                    last_stream_command_sent_timestamp = 0.0;
                    stream_response_received = false;
                }
            } else {

                if ( Timer::GetFPGATimestamp() - last_data_received_timestamp > 1.0 ) {
                    ResetSerialPort();
                }
            }
        } catch (std::exception &ex) {

            stream_response_received = false;
            timeout_count++;
            #ifdef SERIALIO_DASHBOARD_DEBUG
                SmartDashboard::PutNumber("navX Serial Port Timeout / Buffer Overrun", (double)timeout_count);
                SmartDashboard::PutString("navX Last Exception", ex.what());
            #endif
            ResetSerialPort();
        }
    }
}

bool SerialIO::IsConnected() {
    double time_since_last_update = Timer::GetFPGATimestamp() - this->last_valid_packet_time;
    return time_since_last_update <= IO_TIMEOUT_SECONDS;
}

double SerialIO::GetByteCount() {
    return byte_count;
}

double SerialIO::GetUpdateCount() {
    return update_count;
}

void SerialIO::SetUpdateRateHz(uint8_t update_rate) {
    update_rate_hz = update_rate;
}

void SerialIO::ZeroYaw() {
    EnqueueIntegrationControlMessage(NAVX_INTEGRATION_CTL_RESET_YAW);
}

void SerialIO::ZeroDisplacement() {
    EnqueueIntegrationControlMessage( NAVX_INTEGRATION_CTL_RESET_DISP_X |
                                      NAVX_INTEGRATION_CTL_RESET_DISP_Y |
                                      NAVX_INTEGRATION_CTL_RESET_DISP_Z );
}

void SerialIO::Stop() {
    stop = true;
}

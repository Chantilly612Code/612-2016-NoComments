# 8 "./lib/NavX/AHRS.cpp"
#include <sstream>
#include <string>
#include <iomanip>
#include "AHRS.h"
#include "IIOProvider.h"
#include "IIOCompleteNotification.h"
#include "IBoardCapabilities.h"
#include "InertialDataIntegrator.h"
#include "OffsetTracker.h"
#include "ContinuousAngleTracker.h"
#include "RegisterIOSPI.h"
#include "RegisterIOI2C.h"
#include "SerialIO.h"

static const uint8_t NAVX_DEFAULT_UPDATE_RATE_HZ = 60;
static const int YAW_HISTORY_LENGTH = 10;
static const int16_t DEFAULT_ACCEL_FSR_G = 2;
static const int16_t DEFAULT_GYRO_FSR_DPS = 2000;


static const uint32_t DEFAULT_SPI_BITRATE = 500000;
static const uint8_t NAVX_MXP_I2C_ADDRESS = 0x32;

class AHRSInternal : public IIOCompleteNotification, public IBoardCapabilities {
    AHRS *ahrs;
    friend class AHRS;
    AHRSInternal(AHRS* ahrs) {
        this->ahrs = ahrs;
    }
    virtual ~AHRSInternal() {}





    void SetYawPitchRoll(IMUProtocol::YPRUpdate& ypr_update, long sensor_timestamp) {
        ahrs->yaw = ypr_update.yaw;
        ahrs->pitch = ypr_update.pitch;
        ahrs->roll = ypr_update.roll;
        ahrs->compass_heading = ypr_update.compass_heading;
        ahrs->last_sensor_timestamp = sensor_timestamp;
    }

    void SetAHRSPosData(AHRSProtocol::AHRSPosUpdate& ahrs_update, long sensor_timestamp) {


        ahrs->yaw = ahrs_update.yaw;
        ahrs->pitch = ahrs_update.pitch;
        ahrs->roll = ahrs_update.roll;
        ahrs->compass_heading = ahrs_update.compass_heading;
        ahrs->yaw_offset_tracker->UpdateHistory(ahrs_update.yaw);




        ahrs->fused_heading = ahrs_update.fused_heading;


        ahrs->world_linear_accel_x = ahrs_update.linear_accel_x;
        ahrs->world_linear_accel_y = ahrs_update.linear_accel_y;
        ahrs->world_linear_accel_z = ahrs_update.linear_accel_z;


        ahrs->mpu_temp_c = ahrs_update.mpu_temp;


        ahrs->altitude = ahrs_update.altitude;
        ahrs->baro_pressure = ahrs_update.barometric_pressure;


        ahrs->is_moving =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_MOVING) != 0)
                        ? true : false);
        ahrs->is_rotating =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_YAW_STABLE) != 0)
                        ? false : true);
        ahrs->altitude_valid =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)
                        ? true : false);
        ahrs->is_magnetometer_calibrated =
                (((ahrs_update.cal_status &
                        NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)
                        ? true : false);
        ahrs->magnetic_disturbance =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)
                        ? true : false);

        ahrs->quaternionW = ahrs_update.quat_w;
        ahrs->quaternionX = ahrs_update.quat_x;
        ahrs->quaternionY = ahrs_update.quat_y;
        ahrs->quaternionZ = ahrs_update.quat_z;

        ahrs->velocity[0] = ahrs_update.vel_x;
        ahrs->velocity[1] = ahrs_update.vel_y;
        ahrs->velocity[2] = ahrs_update.vel_z;
        ahrs->displacement[0] = ahrs_update.disp_x;
        ahrs->displacement[1] = ahrs_update.disp_y;
        ahrs->displacement[2] = ahrs_update.disp_z;

        ahrs->yaw_angle_tracker->NextAngle(ahrs->GetYaw());
        ahrs->last_sensor_timestamp = sensor_timestamp;
    }

    void SetRawData(AHRSProtocol::GyroUpdate& raw_data_update, long sensor_timestamp) {
        ahrs->raw_gyro_x = raw_data_update.gyro_x;
        ahrs->raw_gyro_y = raw_data_update.gyro_y;
        ahrs->raw_gyro_z = raw_data_update.gyro_z;
        ahrs->raw_accel_x = raw_data_update.accel_x;
        ahrs->raw_accel_y = raw_data_update.accel_y;
        ahrs->raw_accel_z = raw_data_update.accel_z;
        ahrs->cal_mag_x = raw_data_update.mag_x;
        ahrs->cal_mag_y = raw_data_update.mag_y;
        ahrs->cal_mag_z = raw_data_update.mag_z;
        ahrs->mpu_temp_c = raw_data_update.temp_c;
        ahrs->last_sensor_timestamp = sensor_timestamp;
    }

    void SetAHRSData(AHRSProtocol::AHRSUpdate& ahrs_update, long sensor_timestamp) {


        ahrs->yaw = ahrs_update.yaw;
        ahrs->pitch = ahrs_update.pitch;
        ahrs->roll = ahrs_update.roll;
        ahrs->compass_heading = ahrs_update.compass_heading;
        ahrs->yaw_offset_tracker->UpdateHistory(ahrs_update.yaw);




        ahrs->fused_heading = ahrs_update.fused_heading;


        ahrs->world_linear_accel_x = ahrs_update.linear_accel_x;
        ahrs->world_linear_accel_y = ahrs_update.linear_accel_y;
        ahrs->world_linear_accel_z = ahrs_update.linear_accel_z;


        ahrs->mpu_temp_c = ahrs_update.mpu_temp;


        ahrs->altitude = ahrs_update.altitude;
        ahrs->baro_pressure = ahrs_update.barometric_pressure;


        ahrs->cal_mag_x = ahrs_update.cal_mag_x;
        ahrs->cal_mag_y = ahrs_update.cal_mag_y;
        ahrs->cal_mag_z = ahrs_update.cal_mag_z;


        ahrs->is_moving =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_MOVING) != 0)
                        ? true : false);
        ahrs->is_rotating =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_YAW_STABLE) != 0)
                        ? false : true);
        ahrs->altitude_valid =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)
                        ? true : false);
        ahrs->is_magnetometer_calibrated =
                (((ahrs_update.cal_status &
                        NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)
                        ? true : false);
        ahrs->magnetic_disturbance =
                (((ahrs_update.sensor_status &
                        NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)
                        ? true : false);

        ahrs->quaternionW = ahrs_update.quat_w;
        ahrs->quaternionX = ahrs_update.quat_x;
        ahrs->quaternionY = ahrs_update.quat_y;
        ahrs->quaternionZ = ahrs_update.quat_z;

        ahrs->last_sensor_timestamp = sensor_timestamp;

        ahrs->UpdateDisplacement( ahrs->world_linear_accel_x,
                ahrs->world_linear_accel_y,
                ahrs->update_rate_hz,
                ahrs->is_moving);

        ahrs->yaw_angle_tracker->NextAngle(ahrs->GetYaw());
    }

    void SetBoardID(AHRSProtocol::BoardID& board_id) {
        ahrs->board_type = board_id.type;
        ahrs->hw_rev = board_id.hw_rev;
        ahrs->fw_ver_major = board_id.fw_ver_major;
        ahrs->fw_ver_minor = board_id.fw_ver_minor;
    }

    void SetBoardState(IIOCompleteNotification::BoardState& board_state) {
        ahrs->update_rate_hz = board_state.update_rate_hz;
        ahrs->accel_fsr_g = board_state.accel_fsr_g;
        ahrs->gyro_fsr_dps = board_state.gyro_fsr_dps;
        ahrs->capability_flags = board_state.capability_flags;
        ahrs->op_status = board_state.op_status;
        ahrs->sensor_status = board_state.sensor_status;
        ahrs->cal_status = board_state.cal_status;
        ahrs->selftest_status = board_state.selftest_status;
     }




    bool IsOmniMountSupported()
    {
       return (((ahrs->capability_flags & NAVX_CAPABILITY_FLAG_OMNIMOUNT) !=0) ? true : false);
    }

    bool IsBoardYawResetSupported()
    {
        return (((ahrs->capability_flags & NAVX_CAPABILITY_FLAG_YAW_RESET) != 0) ? true : false);
    }

    bool IsDisplacementSupported()
    {
        return (((ahrs->capability_flags & NAVX_CAPABILITY_FLAG_VEL_AND_DISP) != 0) ? true : false);
    }
};
# 252 "./lib/NavX/AHRS.cpp"
AHRS::AHRS(SPI::Port spi_port_id, uint8_t update_rate_hz) {
    SPIInit(spi_port_id, DEFAULT_SPI_BITRATE, update_rate_hz);
}
# 277 "./lib/NavX/AHRS.cpp"
AHRS::AHRS(SPI::Port spi_port_id, uint32_t spi_bitrate, uint8_t update_rate_hz) {
    SPIInit(spi_port_id, spi_bitrate, update_rate_hz);
}
# 294 "./lib/NavX/AHRS.cpp"
AHRS::AHRS(I2C::Port i2c_port_id, uint8_t update_rate_hz) {
    I2CInit(i2c_port_id, update_rate_hz);
}
# 318 "./lib/NavX/AHRS.cpp"
AHRS::AHRS(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz) {
    SerialInit(serial_port_id, data_type, update_rate_hz);
}
# 329 "./lib/NavX/AHRS.cpp"
AHRS::AHRS(SPI::Port spi_port_id) {
    SPIInit(spi_port_id, DEFAULT_SPI_BITRATE, NAVX_DEFAULT_UPDATE_RATE_HZ);
}
# 341 "./lib/NavX/AHRS.cpp"
AHRS::AHRS(I2C::Port i2c_port_id) {
    I2CInit(i2c_port_id, NAVX_DEFAULT_UPDATE_RATE_HZ);
}
# 355 "./lib/NavX/AHRS.cpp"
AHRS::AHRS(SerialPort::Port serial_port_id) {
    SerialInit(serial_port_id, SerialDataType::kProcessedData, NAVX_DEFAULT_UPDATE_RATE_HZ);
}







float AHRS::GetPitch() {
    return pitch;
}







float AHRS::GetRoll() {
    return roll;
}
# 389 "./lib/NavX/AHRS.cpp"
float AHRS::GetYaw() {
    if ( ahrs_internal->IsBoardYawResetSupported() ) {
        return this->yaw;
    } else {
        return (float) yaw_offset_tracker->ApplyOffset(this->yaw);
    }
}
# 411 "./lib/NavX/AHRS.cpp"
float AHRS::GetCompassHeading() {
    return compass_heading;
}
# 423 "./lib/NavX/AHRS.cpp"
void AHRS::ZeroYaw() {
    if ( ahrs_internal->IsBoardYawResetSupported() ) {
        io->ZeroYaw();
    } else {
        yaw_offset_tracker->SetOffset();
    }
}
# 447 "./lib/NavX/AHRS.cpp"
bool AHRS::IsCalibrating() {
    return !((cal_status &
                NAVX_CAL_STATUS_IMU_CAL_STATE_MASK) ==
                    NAVX_CAL_STATUS_IMU_CAL_COMPLETE);
}
# 461 "./lib/NavX/AHRS.cpp"
bool AHRS::IsConnected() {
    return io->IsConnected();
}
# 475 "./lib/NavX/AHRS.cpp"
double AHRS::GetByteCount() {
    return io->GetByteCount();
}







double AHRS::GetUpdateCount() {
    return io->GetUpdateCount();
}
# 498 "./lib/NavX/AHRS.cpp"
long AHRS::GetLastSensorTimestamp() {
 return this->last_sensor_timestamp;
}
# 513 "./lib/NavX/AHRS.cpp"
float AHRS::GetWorldLinearAccelX()
{
    return this->world_linear_accel_x;
}
# 529 "./lib/NavX/AHRS.cpp"
float AHRS::GetWorldLinearAccelY()
{
    return this->world_linear_accel_y;
}
# 545 "./lib/NavX/AHRS.cpp"
float AHRS::GetWorldLinearAccelZ()
{
    return this->world_linear_accel_z;
}
# 558 "./lib/NavX/AHRS.cpp"
bool AHRS::IsMoving()
{
    return is_moving;
}
# 574 "./lib/NavX/AHRS.cpp"
bool AHRS::IsRotating()
{
    return is_rotating;
}
# 587 "./lib/NavX/AHRS.cpp"
float AHRS::GetBarometricPressure()
{
    return baro_pressure;
}
# 604 "./lib/NavX/AHRS.cpp"
float AHRS::GetAltitude()
{
    return altitude;
}
# 619 "./lib/NavX/AHRS.cpp"
bool AHRS::IsAltitudeValid()
{
    return this->altitude_valid;
}
# 639 "./lib/NavX/AHRS.cpp"
float AHRS::GetFusedHeading()
{
    return fused_heading;
}
# 653 "./lib/NavX/AHRS.cpp"
bool AHRS::IsMagneticDisturbance()
{
    return magnetic_disturbance;
}
# 669 "./lib/NavX/AHRS.cpp"
bool AHRS::IsMagnetometerCalibrated()
{
    return is_magnetometer_calibrated;
}
# 688 "./lib/NavX/AHRS.cpp"
float AHRS::GetQuaternionW() {
    return ((float)quaternionW / 16384.0f);
}
# 703 "./lib/NavX/AHRS.cpp"
float AHRS::GetQuaternionX() {
    return ((float)quaternionX / 16384.0f);
}
# 721 "./lib/NavX/AHRS.cpp"
float AHRS::GetQuaternionY() {
    return ((float)quaternionY / 16384.0f);
}
# 739 "./lib/NavX/AHRS.cpp"
float AHRS::GetQuaternionZ() {
    return ((float)quaternionZ / 16384.0f);
}





void AHRS::ResetDisplacement() {
    if (ahrs_internal->IsDisplacementSupported() ) {
        io->ZeroDisplacement();
    }
    else {
        integrator->ResetDisplacement();
    }
}
# 764 "./lib/NavX/AHRS.cpp"
void AHRS::UpdateDisplacement( float accel_x_g, float accel_y_g,
                                    int update_rate_hz, bool is_moving ) {
    integrator->UpdateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving);
}
# 777 "./lib/NavX/AHRS.cpp"
float AHRS::GetVelocityX() {
    return (ahrs_internal->IsDisplacementSupported() ? velocity[0] : integrator->GetVelocityX());
}
# 789 "./lib/NavX/AHRS.cpp"
float AHRS::GetVelocityY() {
    return (ahrs_internal->IsDisplacementSupported() ? velocity[1] : integrator->GetVelocityY());
}
# 801 "./lib/NavX/AHRS.cpp"
float AHRS::GetVelocityZ() {
    return (ahrs_internal->IsDisplacementSupported() ? velocity[2] : 0.f);
}
# 815 "./lib/NavX/AHRS.cpp"
float AHRS::GetDisplacementX() {
    return (ahrs_internal->IsDisplacementSupported() ? displacement[0] : integrator->GetVelocityX());
}
# 829 "./lib/NavX/AHRS.cpp"
float AHRS::GetDisplacementY() {
    return (ahrs_internal->IsDisplacementSupported() ? displacement[1] : integrator->GetVelocityY());
}
# 843 "./lib/NavX/AHRS.cpp"
float AHRS::GetDisplacementZ() {
    return (ahrs_internal->IsDisplacementSupported() ? displacement[2] : 0.f);
}

void AHRS::SPIInit( SPI::Port spi_port_id, uint32_t bitrate, uint8_t update_rate_hz ) {
    commonInit( update_rate_hz );
    io = new RegisterIO(new RegisterIO_SPI(new SPI(spi_port_id), bitrate), update_rate_hz, ahrs_internal, ahrs_internal);
    task = new Task("navX-MXP_IO", (FUNCPTR)AHRS::ThreadFunc,io);
}

void AHRS::I2CInit( I2C::Port i2c_port_id, uint8_t update_rate_hz ) {
    commonInit(update_rate_hz);
    io = new RegisterIO(new RegisterIO_I2C(new I2C(i2c_port_id, NAVX_MXP_I2C_ADDRESS)), update_rate_hz, ahrs_internal, ahrs_internal);
    task = new Task("navX-MXP_IO", (FUNCPTR)AHRS::ThreadFunc,io);
}

void AHRS::SerialInit(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz) {
    commonInit(update_rate_hz);
    bool processed_data = (data_type == SerialDataType::kProcessedData);
    io = new SerialIO(serial_port_id, update_rate_hz, processed_data, ahrs_internal, ahrs_internal);
    task = new Task("navX-MXP_IO", (FUNCPTR)AHRS::ThreadFunc,io);
}

void AHRS::commonInit( uint8_t update_rate_hz ) {

    ahrs_internal = new AHRSInternal(this);
    this->update_rate_hz = update_rate_hz;



    yaw_offset_tracker = new OffsetTracker(YAW_HISTORY_LENGTH);
    integrator = new InertialDataIntegrator();
    yaw_angle_tracker = new ContinuousAngleTracker();

    yaw =
            pitch =
                    roll =
                            compass_heading = 0.0f;
    world_linear_accel_x =
            world_linear_accel_y =
                    world_linear_accel_z = 0.0f;
    mpu_temp_c = 0.0f;
    fused_heading = 0.0f;
    altitude = 0.0f;
    baro_pressure = 0.0f;
    is_moving = false;
    is_rotating = false;
    baro_sensor_temp_c = 0.0f;
    altitude_valid = false;
    is_magnetometer_calibrated = false;
    magnetic_disturbance = false;
    quaternionW =
            quaternionX =
                    quaternionY =
                            quaternionZ = 0.0f;



    for ( int i = 0; i < 3; i++ ) {
        velocity[i] = 0.0f;
        displacement[i] = 0.0f;
    }



    raw_gyro_x =
            raw_gyro_y =
                    raw_gyro_z = 0.0f;
    raw_accel_x =
            raw_accel_y =
                    raw_accel_z = 0.0f;
    cal_mag_x =
            cal_mag_y =
                    cal_mag_z = 0.0f;


    update_rate_hz = 0;
    accel_fsr_g = DEFAULT_ACCEL_FSR_G;
    gyro_fsr_dps = DEFAULT_GYRO_FSR_DPS;
    capability_flags = 0;
    op_status =
            sensor_status =
                    cal_status =
                            selftest_status = 0;

    board_type =
            hw_rev =
                    fw_ver_major =
                            fw_ver_minor = 0;
    last_sensor_timestamp = 0;
    last_update_time = 0;

    table = 0;
    io = 0;
}
# 956 "./lib/NavX/AHRS.cpp"
double AHRS::GetAngle() {
    return yaw_angle_tracker->GetAngle();
}
# 968 "./lib/NavX/AHRS.cpp"
double AHRS::GetRate() {
    return yaw_angle_tracker->GetRate();
}
# 979 "./lib/NavX/AHRS.cpp"
void AHRS::Reset() {
    ZeroYaw();
}

static const float DEV_UNITS_MAX = 32768.0f;
# 993 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawGyroX() {
    return this->raw_gyro_x / (DEV_UNITS_MAX / (float)gyro_fsr_dps);
}
# 1005 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawGyroY() {
    return this->raw_gyro_y / (DEV_UNITS_MAX / (float)gyro_fsr_dps);
}
# 1017 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawGyroZ() {
    return this->raw_gyro_z / (DEV_UNITS_MAX / (float)gyro_fsr_dps);
}
# 1030 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawAccelX() {
    return this->raw_accel_x / (DEV_UNITS_MAX / (float)accel_fsr_g);
}
# 1043 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawAccelY() {
    return this->raw_accel_y / (DEV_UNITS_MAX / (float)accel_fsr_g);
}
# 1056 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawAccelZ() {
    return this->raw_accel_z / (DEV_UNITS_MAX / (float)accel_fsr_g);
}

static const float UTESLA_PER_DEV_UNIT = 0.15f;
# 1071 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawMagX() {
    return this->cal_mag_x / UTESLA_PER_DEV_UNIT;
}
# 1084 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawMagY() {
    return this->cal_mag_y / UTESLA_PER_DEV_UNIT;
}
# 1097 "./lib/NavX/AHRS.cpp"
float AHRS::GetRawMagZ() {
    return this->cal_mag_z / UTESLA_PER_DEV_UNIT;
 }
# 1108 "./lib/NavX/AHRS.cpp"
float AHRS::GetPressure() {

    return 0;
}
# 1123 "./lib/NavX/AHRS.cpp"
float AHRS::GetTempC()
{
    return this->mpu_temp_c;
}
# 1140 "./lib/NavX/AHRS.cpp"
AHRS::BoardYawAxis AHRS::GetBoardYawAxis() {
    BoardYawAxis yaw_axis;
    short yaw_axis_info = (short)(capability_flags >> 3);
    yaw_axis_info &= 7;
    if ( yaw_axis_info == OMNIMOUNT_DEFAULT) {
        yaw_axis.up = true;
        yaw_axis.board_axis = BoardAxis::kBoardAxisZ;
    } else {
        yaw_axis.up = (((yaw_axis_info & 0x01) != 0) ? true : false);
        yaw_axis_info >>= 1;
        switch ( yaw_axis_info ) {
        case 0:
            yaw_axis.board_axis = BoardAxis::kBoardAxisX;
            break;
        case 1:
            yaw_axis.board_axis = BoardAxis::kBoardAxisY;
            break;
        case 2:
        default:
            yaw_axis.board_axis = BoardAxis::kBoardAxisZ;
            break;
        }
    }
    return yaw_axis;
}
# 1176 "./lib/NavX/AHRS.cpp"
std::string AHRS::GetFirmwareVersion() {
    std::ostringstream os;
    os << (int)fw_ver_major << "." << (int)fw_ver_minor;
    std::string fw_version = os.str();
    return fw_version;
}





void AHRS::UpdateTable() {
    if (table != 0) {
        table->PutNumber("Value", GetYaw());
    }
}

void AHRS::StartLiveWindowMode() {
}

void AHRS::StopLiveWindowMode() {
}



void AHRS::InitTable(std::shared_ptr<ITable> itable) {
    table = itable;
    UpdateTable();
}

std::shared_ptr<ITable> AHRS::GetTable() const {
    return table;
}

std::string AHRS::GetSmartDashboardType() const {
    return "Gyro";
}
# 1224 "./lib/NavX/AHRS.cpp"
double AHRS::PIDGet() {
    return GetYaw();
}

int AHRS::ThreadFunc(IIOProvider *io_provider) {
    io_provider->Run();
    return 0;
}

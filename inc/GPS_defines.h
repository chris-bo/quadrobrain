/*
 * GPS_defines.h
 *
 *  Created on: Jul 10, 2015
 *      Author: bohni
 */

#ifndef GPS_DEFINES_H_
#define GPS_DEFINES_H_


/**********************************************************************************/
/* TODO GPS_TODOS:
 *
 * -check if intitializer works with standard config
 *
 * -poll/detect/decode NAV_SBAS ? -> msg length variable-> buffer size?
 *
 *
 * - decode flags in nav_sol /utc
 *
 */
/**********************************************************************************/

/* Check Timeouts !! */
/* Timeout for USART Communication
 * */
#define GPS_INIT_TIMEOUT                    0xfff
#define GPS_UART_TIMEOUT                    0xfff

/* Polled messages in standard config */

#define POLL_NAV_TIMEUTC
#define POLL_NAV_SOL
#define POLL_NAV_POSECEV
#define POLL_NAV_POSLLH
#define POLL_NAV_VELECEF
#define POLL_NAV_VELNED
#define POLL_NAV_DOP
#define POLL_NAV_STATUS


/* Infinite and continuous reception */
#define CONTINUOUS_RECEPTION

/* Config via pc -> no configChanges */
//#define CONFIG_VIA_PC

/* Settings for UpdateCFG*/
//#define CHANGE_DEFAULT_CFG_SBAS
//#define CHANGE_DEFAULT_CFG_NAV

/* Save Settings to Battery Backed RAM*/
#define SAVE_PORT_SETTINGS_TO_BBR


/* Message lengths
 * Payload + 8
 *
 * RX Buffer size must fit these lengths
 */
#define UBX_MSG_LENGTH_NAV_SOL              (52+8)
#define UBX_MSG_LENGTH_NAV_POSLLH           (28+8)
#define UBX_MSG_LENGTH_NAV_VELNED           (36+8)
#define UBX_MSG_LENGTH_NAV_VELECEF          (20+8)
#define UBX_MSG_LENGTH_NAV_POSECEF          (20+8)
#define UBX_MSG_LENGTH_NAV_TIMEUTC          (20+8)
#define UBX_MSG_LENGTH_NAV_DOP              (18+8)
#define UBX_MSG_LENGTH_NAV_STATUS           (16+8)
#define UBX_MSG_LENGTH_NAV_SBAS             ()

#define UBX_MSG_LENGTH_ACK                  (2+8)




/*   Transferstate
 * 2lsb: flags
 * 2mbs configuration flags to update all config settings
 */
#define GPS_COM_FLAG_TX_RUNNING             0x01
#define GPS_COM_FLAG_RX_RUNNING             0x02
#define GPS_COM_FLAG_DECODE_COMPLETE        0x04
#define GPS_COM_FLAG_LAST_UPDATE_COMPLETE   0x08
#define GPS_COM_FLAG_RECEPTION_ERROR        0x10
#define GPS_COM_FLAG_TRANSMISSION_ERROR     0x20
#define GPS_COM_FLAG_TIMEOUT                0x40
#define GPS_COM_FLAG_ERROR                  0x80

/* configuration
 * */

#define GPS_GET_NAV_SOL                     0x00010000
#define GPS_GET_LLH                         0x00020000
#define GPS_GET_VELNED                      0x00040000
#define GPS_GET_ECEF                        0x00080000

#define GPS_GET_VELECEF                     0x00100000
#define GPS_GET_DATE                        0x00200000
#define GPS_GET_NAV_DOP                     0x04000000
#define GPS_GET_NAV_STATUS                  0x08000000

#define GPS_GET_DATA_BITMASK                0x0FFF0000

#define GPS_HANDLER_HALT                    0x40000000
#define GPS_CONTINUOUS_REC                  0x80000000

/* Optimization of Polling Defines */

#ifdef POLL_NAV_VELECEF
#undef POLL_NAV_POSECEV
#undef POLL_NAV_VELECEF
#define POLL_NAV_SOL
#endif



/**********************************************************************************/

typedef struct {
    uint8_t msgid;
    uint8_t classid;
    uint8_t ack;
} GPS_ACK_t;


/**
 * Date struct for GPS
 *
 */
typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
} GPS_Date_t;

/**
 * Time struct for GPS
 *
 */
typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t hundredths;
    uint8_t validity;
} GPS_Time_t;


/**
 * struct for ECEF Data
 *
 */
typedef struct {

    int32_t x;      // Position X [cm]
    int32_t y;      // Position Y [cm]
    int32_t z;      // Position Z [cm]
    uint32_t pAcc;  // Position Accuracy Estimate [cm]

    int32_t vx;     // Velocity X [cm/s]
    int32_t vy;     // Velocity Y [cm/s]
    int32_t vz;     // Velocity Z [cm/s]
    uint32_t sAcc;  // Speed Accuracy Estimate [cm/s]

}GPS_ECEF_t;

/**
 * struct for NED Data
 *
 */
typedef struct {
    int32_t vN;         // Velocity north [cm/s]
    int32_t vE;         // Velocity east [cm/s]
    int32_t vD;         // Velocity down [cm/s]
    uint32_t speed;     // Speed (3D)[cm/s]
    uint32_t gSpeed;    // Groundspeed [cm/s]
    int32_t heading;    // Heading of motion 2-D [1e-5 deg]
    uint32_t sAcc;      // Speed Accuracy Estimate [cm/s]
    uint32_t cAcc;      // Heading Accuracy Estimate [1e-5 deg]

}GPS_NED_t;

/**
 * struct for LLH Data
 *
 */
typedef struct {

    int32_t lat;    // latitude [1e-7 deg]
    int32_t lon;    // longitude [1e-7 deg]
    int32_t h;      // height above ellipsoid [mm]
    int32_t hMSL;   // height above mean sea level [mm]

    uint32_t hAcc;  // Horizontal Accuracy Estimate [mm]
    uint32_t vAcc;  // Vertical Accuracy Estimate [mm]

}GPS_LLH_t;

/**
 * struct for dilution of precission (DOP) Data
 *
 */
typedef struct {
    uint16_t pDOP;  // Position DOP scale 0.01
    uint16_t gDOP;  // Geometric DOP
    uint16_t tDOP;  // Time DOP
    uint16_t vDOP;  // Vertical DOP
    uint16_t hDOP;  // Horizontal DOP
    uint16_t nDOP;  // Northing DOP
    uint16_t eDOP;  // Easting DOP
}GPS_DOP_t;

/**
 * Main struct to work with GPS
 *
 */
typedef struct {
    GPS_Date_t date;
    GPS_Time_t time;
    uint8_t numSV;  // Number of SVs used in Nav Solution

    uint8_t gpsFix;
    uint8_t flags;

    uint32_t iTOW;  // Time of Week [ms]

    GPS_DOP_t dop;

    int16_t gpsWeek;

    GPS_ECEF_t ecef_data;
    GPS_LLH_t llh_data;
    GPS_NED_t ned_data;

    uint32_t ttff;  // Time to first fix (millisecond time tag)
    uint32_t msss;  // Milliseconds since Startup / Reset

    uint8_t flags2;
    uint8_t FixStatus;
    uint16_t padding_dummy;

} GPS_Data_t;

/**********************************************************************************/
/* UBX Protocoll */

#define UBX_HEADER_0    0xB5
#define UBX_HEADER_1    0x62


/* UBX Protocoll Classes */
#define UBX_NAV         0x01
#define UBX_RXM         0x02
#define UBX_INF         0x04
#define UBX_ACK         0x05
#define UBX_CFG         0x06
#define UBX_MON         0x0A
#define UBX_AID         0x0B
#define UBX_TIM         0x0D
#define UBX_ESF         0x10


/* UBX_NAV CLASS */
/*
 * Navigation Results: i.e. Position, Speed, Time, Acc, Heading, DOP, SVs used.
 * Messages in the NAV Class output Navigation Data such as position, altitude
 * and velocity in a number offormats. Additionally, status flags and accuracy
 * figures are output.
 *
 */
#define UBX_NAV_AOPSTATUS       0x60
#define UBX_NAV_CLOCK           0x22
#define UBX_NAV_DGPS            0x31
#define UBX_NAV_DOP             0x04
#define UBX_NAV_EKFSTATUS       0x40
#define UBX_NAV_POSECEF         0x01
#define UBX_NAV_POSLLH          0x02
#define UBX_NAV_SBAS            0x32
#define UBX_NAV_SOL             0x06
#define UBX_NAV_STATUS          0x03
#define UBX_NAV_SVINFO          0x30
#define UBX_NAV_TIMEGPS         0x20
#define UBX_NAV_TIMEUTC         0x21
#define UBX_NAV_VELECEF         0x11
#define UBX_NAV_VELNED          0x12


/*  UBX_GFG CLASS */
/*
 * Configuration Input Messages: i.e. Set Dynamic Model, Set DOP Mask,
 * Set Baud Rate, etc..
 * The CFG Class can be used to configure the receiver and read out
 * current configuration values. Any messagesin Class CFG sent to the
 * receiver are acknowledged (with Message ACK-ACK) if processed successfully,
 * and rejected (with Message ACK-NAK) if processing the message failed.
 *
 */
#define UBX_CFG_ANT         0x13
#define UBX_CFG_CFG         0x09
#define UBX_CFG_DAT         0x06
#define UBX_CFG_EXF         0x12
#define UBX_CFG_ESFGWT      0x29
#define UBX_CFG_FXN         0x0E
#define UBX_CFG_INF         0x02
#define UBX_CFG_ITFM        0x39
#define UBX_CFG_MSG         0x01
#define UBX_CFG_NAV5        0x24
#define UBX_CFG_NAVX5       0x23
#define UBX_CFG_NMEA        0x17
#define UBX_CFG_NVS         0x22
#define UBX_CFG_PM2         0x3B
#define UBX_CFG_PM          0x32
#define UBX_CFG_PRT         0x00
#define UBX_CFG_RATE        0x08
#define UBX_CFG_RINV        0x34
#define UBX_CFG_RST         0x04
#define UBX_CFG_RXM         0x11
#define UBX_CFG_SBAS        0x16
#define UBX_CFG_TMODE2      0x3D
#define UBX_CFG_TMODE       0x1D
#define UBX_CFG_TP5         0x31
#define UBX_CFG_TP          0x07
#define UBX_CFG_USB         0x1B


/* UBX_ACK CLASS */
/*
 * Ack/Nack Messages: i.e. as replies to CFG Input Messages.
 * Messages in this class are sent as a result of a CFG message being received,
 * decoded and processed by the receiver.
 *
 */
#define UBX_ACK_ACK             0x01
#define UBX_ACK_NAK             0x00


/* UBX_RXM CLASS */
/*
 * Receiver Manager Messages: i.e. Satellite Status, RTC Status.
 * Messages in Class RXM output status and result data from the Receiver Manager.
 *
 */
#define UBX_RXM_ALM             0x30
#define UBX_RXM_EPH             0x31
#define UBX_RXM_PMREG           0x40
#define UBX_RXM_RAW             0x10
#define UBX_RXM_SFRB            0x11
#define UBX_RXM_SVSI            0x20


/* UBX_TIM CLASS*/
/*
 * Timing Messages: i.e. Timepulse Output, Timemark Results. Messages
 * in this class are output by the receiver, giving information on
 * Timepulse and Timemark measurements.
 *
 */

/* UBX_AID CLASS */
/*
 * AssistNow Aiding Messages: i.e. Ephemeris, Almanac, other A-GPS data input.
 * Messages in this class are used to send aiding data to the receiver.
 *
 *  NOT USED
 */

/* UBX_ESF CLASS */
/*
 * External Sensor Fusion Messages: i.e. External sensor measurements and status
 * information.
 *
 *  NOT USED
 */

/* UBX_INF CLASS */
/*
 * Information Messages: i.e. Printf-Style Messages, with IDs such as Error,
 * Warning, Notice. The INF Class is basically an output class that allows the
 * firmware and application code to output strings with a printf-style call.
 * All INF messages have an associated type to indicate the kind of message.
 *
 *  NOT USED
 */


/* UBX_MON CLASS */
/*
 * Monitoring Messages: i.e. Comunication Status, CPU Load, Stack Usage, Task
 * Status. Messages in this class are sent to report GPS receiver status, such
 * as CPU load, stack usage, I/O subsystem statistics etc.
 *
 *  NOT USED
 */


/* NMEA Message IDs for UBX protocol*/
#define NEO6_UBX_NMEA_DTM_0         0xF0
#define NEO6_UBX_NMEA_DTM_1         0x0A // Datum Reference
#define NEO6_UBX_NMEA_GBS_0         0xF0
#define NEO6_UBX_NMEA_GBS_1         0x09 // GNSS Satellite Fault Detection
#define NEO6_UBX_NMEA_GGA_0         0xF0
#define NEO6_UBX_NMEA_GGA_1         0x00 // Global positioning system fix data
#define NEO6_UBX_NMEA_GLL_0         0xF0
#define NEO6_UBX_NMEA_GLL_1         0x01 // Latitude and longitude, with time of position fix and status
#define NEO6_UBX_NMEA_GPQ_0         0xF0
#define NEO6_UBX_NMEA_GPQ_1         0x40 // Poll message
#define NEO6_UBX_NMEA_GRS_0         0xF0
#define NEO6_UBX_NMEA_GRS_1         0x06 // GNSS Range Residuals
#define NEO6_UBX_NMEA_GSA_0         0xF0
#define NEO6_UBX_NMEA_GSA_1         0x02 // GNSS DOP and Active Satellites
#define NEO6_UBX_NMEA_GST_0         0xF0
#define NEO6_UBX_NMEA_GST_1         0x07 // GNSS Pseudo Range Error Statistics
#define NEO6_UBX_NMEA_GSV_0         0xF0
#define NEO6_UBX_NMEA_GSV_1         0x03 // GNSS Satellites in View
#define NEO6_UBX_NMEA_RMC_0         0xF0
#define NEO6_UBX_NMEA_RMC_1         0x04 // Recommended Minimum data
#define NEO6_UBX_NMEA_THS_0         0xF0
#define NEO6_UBX_NMEA_THS_1         0x0E // True Heading and Status
#define NEO6_UBX_NMEA_TXT_0         0xF0
#define NEO6_UBX_NMEA_TXT_1         0x41 // Text Transmission
#define NEO6_UBX_NMEA_VTG_0         0xF0
#define NEO6_UBX_NMEA_VTG_1         0x05 // Course over ground and Ground speed
#define NEO6_UBX_NMEA_ZDA_0         0xF0
#define NEO6_UBX_NMEA_ZDA_1         0x08 // Time and Date
/**********************************************************************************/

/* Macros for bitoffsets and byteshifting
 *
 * buffer_32offset(y) y: datasheet value
 *
 * */
#define gps_rx_buffer_offset(y) GPS_RX_buffer[y+6]
#define gps_rx_buffer_16offset(y) gps_rx_buffer_16bit(y+6)
#define gps_rx_buffer_16bit(x) ((GPS_RX_buffer[x+1]<<8)|GPS_RX_buffer[x])
#define gps_rx_buffer_32offset(y) gps_rx_buffer_32bit(y+6)
#define gps_rx_buffer_32bit(x) ((GPS_RX_buffer[x+3]<<24)|(GPS_RX_buffer[x+2]<<16)|(GPS_RX_buffer[x+1]<<8)|GPS_RX_buffer[x])

/**********************************************************************************/
#endif /* GPS_DEFINES_H_ */

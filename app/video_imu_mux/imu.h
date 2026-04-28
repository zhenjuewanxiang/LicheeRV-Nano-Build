#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stddef.h>

/* packet header bytes */
#define IMU_PREAMBLE1   0x58u
#define IMU_PREAMBLE2   0x52u

/* known message class / id */
#define IMU_CLASS_TILT  0x2Cu
#define IMU_CLASS_TILT_COMPACT  0x21u
#define IMU_ID_TILT     0xB2u

/* tilt payload length */
#define IMU_TILT_PAYLOAD_LEN  65u
#define IMU_TILT_COMPACT_PAYLOAD_LEN  37u

/* parsed tilt/imu measurement */
typedef struct {
    double   system_time;  /* seconds */
    uint8_t  status;
    uint8_t  has_accel;
    uint8_t  has_quat;
    float    gyro[3];      /* deg/s  [x, y, z] */
    float    accel[3];     /* m/s^2  [x, y, z] */
    float    pitch;        /* deg */
    float    roll;         /* deg */
    float    yaw;          /* deg */
    float    temperature;  /* °C */
    float    quat[4];      /* [w, x, y, z] */
} imu_tilt_t;

/* raw packet */
typedef struct {
    uint8_t        cls;
    uint8_t        id;
    uint16_t       length;  /* payload byte count */
    const uint8_t *payload;
} imu_packet_t;

/* packet callback */
typedef void (*imu_packet_cb)(const imu_packet_t *pkt, void *user);

/* opaque parser handle */
typedef struct imu_parser imu_parser_t;

/**
 * @brief Create imu parser instance.
 * @param cb Packet callback.
 * @param user User context pointer.
 * @return Created parser handle.
 */
imu_parser_t *imu_parser_create(imu_packet_cb cb, void *user);

/**
 * @brief Destroy imu parser instance.
 * @param p Parser handle.
 * @return None.
 */
void          imu_parser_destroy(imu_parser_t *p);

/**
 * @brief Feed uart bytes into parser.
 * @param p Parser handle.
 * @param buf Input byte buffer.
 * @param len Input byte length.
 * @return None.
 */
void imu_parser_feed(imu_parser_t *p, const uint8_t *buf, size_t len);

/**
 * @brief Decode tilt packet payload.
 * @param pkt Input packet.
 * @param out Decoded tilt output.
 * @return 0 on success.
 */
int imu_decode_tilt(const imu_packet_t *pkt, imu_tilt_t *out);

/**
 * @brief Calculate fletcher checksum.
 * @param buf Input byte buffer.
 * @param length Input byte length.
 * @param ck Output checksum buffer.
 * @return None.
 */
void imu_checksum(const uint8_t *buf, size_t length, uint8_t ck[2]);

#endif /* IMU_H */

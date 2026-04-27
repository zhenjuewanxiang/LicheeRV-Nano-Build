#include "imu.h"

#include <stdlib.h>
#include <string.h>

/* maximum payload length */
#define IMU_MAX_PAYLOAD  1024u

/* parser state machine */
typedef enum {
    ST_PREAMBLE1 = 0,
    ST_PREAMBLE2,
    ST_CLASS,
    ST_ID,
    ST_LEN_LO,
    ST_LEN_HI,
    ST_PAYLOAD,
    ST_CKSUM_A,
    ST_CKSUM_B,
} imu_state_t;

struct imu_parser {
    imu_packet_cb  cb;
    void          *user;

    imu_state_t    state;
    uint8_t        cls;
    uint8_t        id;
    uint16_t       length;      /* expected payload bytes */
    uint16_t       payload_idx; /* bytes received so far */
    uint8_t        buf[IMU_MAX_PAYLOAD];
    uint8_t        ck_expected[2];
};

/**
 * @brief Calculate fletcher checksum.
 * @param buf Input byte buffer.
 * @param length Input byte length.
 * @param ck Output checksum buffer.
 * @return None.
 */
void imu_checksum(const uint8_t *buf, size_t length, uint8_t ck[2])
{
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < length; i++) {
        a = (uint8_t)(a + buf[i]);
        b = (uint8_t)(b + a);
    }
    ck[0] = a;
    ck[1] = b;
}

/**
 * @brief Create imu parser instance.
 * @param cb Packet callback.
 * @param user User context pointer.
 * @return Created parser handle.
 */
imu_parser_t *imu_parser_create(imu_packet_cb cb, void *user)
{
    imu_parser_t *p = (imu_parser_t *)calloc(1, sizeof(*p));
    if (!p)
        return NULL;
    p->cb   = cb;
    p->user = user;
    return p;
}

/**
 * @brief Destroy imu parser instance.
 * @param p Parser handle.
 * @return None.
 */
void imu_parser_destroy(imu_parser_t *p)
{
    free(p);
}

/**
 * @brief Reset parser state.
 * @param p Parser handle.
 * @return None.
 */
static void parser_reset(imu_parser_t *p)
{
    p->state       = ST_PREAMBLE1;
    p->payload_idx = 0;
}

/**
 * @brief Feed uart bytes into parser.
 * @param p Parser handle.
 * @param buf Input byte buffer.
 * @param len Input byte length.
 * @return None.
 */
void imu_parser_feed(imu_parser_t *p, const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = buf[i];

        switch (p->state) {
        case ST_PREAMBLE1:{
            if (b == IMU_PREAMBLE1)
                p->state = ST_PREAMBLE2;
        }break;

        case ST_PREAMBLE2:{
            if (b == IMU_PREAMBLE2)
                p->state = ST_CLASS;
            else
                //re-check current byte as a potential new preamble1
                p->state = (b == IMU_PREAMBLE1) ? ST_PREAMBLE2 : ST_PREAMBLE1;
        }break;

        case ST_CLASS:{
            p->cls   = b;
            p->state = ST_ID;
        }break;

        case ST_ID:{
            p->id    = b;
            p->state = ST_LEN_LO;
        }break;

        case ST_LEN_LO:{
            p->length = b;
            p->state  = ST_LEN_HI;
        }break;

        case ST_LEN_HI:{
            p->length |= (uint16_t)((uint16_t)b << 8);
            p->payload_idx = 0;
            if (p->length == 0) {
                /* no payload, verify checksum immediately */
                uint8_t hdr[4] = { p->cls, p->id,
                                   (uint8_t)(p->length & 0xFF),
                                   (uint8_t)(p->length >> 8) };
                imu_checksum(hdr, 4, p->ck_expected);
                p->state = ST_CKSUM_A;
            } else if (p->length > IMU_MAX_PAYLOAD) {
                parser_reset(p);
            } else {
                p->state = ST_PAYLOAD;
            }
        }break;

        case ST_PAYLOAD:{
            p->buf[p->payload_idx++] = b;
            if (p->payload_idx >= p->length) {
                /* compute expected checksum */
                uint8_t hdr[4] = { p->cls, p->id,
                                   (uint8_t)(p->length & 0xFF),
                                   (uint8_t)(p->length >> 8) };
                uint8_t ck_a = 0, ck_b = 0;
                /* run over header */
                for (int j = 0; j < 4; j++) {
                    ck_a = (uint8_t)(ck_a + hdr[j]);
                    ck_b = (uint8_t)(ck_b + ck_a);
                }
                /* continue over payload */
                for (uint16_t j = 0; j < p->length; j++) {
                    ck_a = (uint8_t)(ck_a + p->buf[j]);
                    ck_b = (uint8_t)(ck_b + ck_a);
                }
                p->ck_expected[0] = ck_a;
                p->ck_expected[1] = ck_b;
                p->state = ST_CKSUM_A;
            }
        }break;

        case ST_CKSUM_A:{
            if (b != p->ck_expected[0]) {
                parser_reset(p);
                //re-process this byte as potential preamble
                if (b == IMU_PREAMBLE1)
                    p->state = ST_PREAMBLE2;
            } else {
                p->state = ST_CKSUM_B;
            }
        }break;

        case ST_CKSUM_B:{
            if (b == p->ck_expected[1] && p->cb) {
                imu_packet_t pkt;
                pkt.cls     = p->cls;
                pkt.id      = p->id;
                pkt.length  = p->length;
                pkt.payload = p->buf;
                p->cb(&pkt, p->user);
            }
            parser_reset(p);
        }break;
        }
    }
}

/* little-endian load helpers */
/**
 * @brief Load float32 value from little-endian buffer.
 * @param p Input byte pointer.
 * @return Parsed float value.
 */
static float load_f32_le(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}

/**
 * @brief Load float64 value from little-endian buffer.
 * @param p Input byte pointer.
 * @return Parsed double value.
 */
static double load_f64_le(const uint8_t *p)
{
    double v;
    memcpy(&v, p, 8);
    return v;
}

/**
 * @brief Decode tilt packet payload.
 * @param pkt Input packet.
 * @param out Decoded tilt output.
 * @return 0 on success.
 */
int imu_decode_tilt(const imu_packet_t *pkt, imu_tilt_t *out)
{
    if (!pkt || !out)
        return -1;
    if (pkt->id != IMU_ID_TILT)
        return -1;
    if (pkt->cls != IMU_CLASS_TILT && pkt->cls != IMU_CLASS_TILT_COMPACT)
        return -1;
    if (pkt->length != IMU_TILT_PAYLOAD_LEN && pkt->length != IMU_TILT_COMPACT_PAYLOAD_LEN)
        return -1;

    memset(out, 0, sizeof(*out));

    const uint8_t *d = pkt->payload;

    out->system_time  = load_f64_le(d);      d += 8;
    out->status       = *d++;
    out->has_accel    = 0;
    out->has_quat     = 0;
    out->gyro[0]      = load_f32_le(d);      d += 4;
    out->gyro[1]      = load_f32_le(d);      d += 4;
    out->gyro[2]      = load_f32_le(d);      d += 4;

    if (pkt->cls == IMU_CLASS_TILT && pkt->length == IMU_TILT_PAYLOAD_LEN) {
        out->has_accel    = 1;
        out->has_quat     = 1;
        out->accel[0]     = load_f32_le(d);      d += 4;
        out->accel[1]     = load_f32_le(d);      d += 4;
        out->accel[2]     = load_f32_le(d);      d += 4;
        out->pitch        = load_f32_le(d);      d += 4;
        out->roll         = load_f32_le(d);      d += 4;
        out->yaw          = load_f32_le(d);      d += 4;
        out->temperature  = load_f32_le(d);      d += 4;
        out->quat[0]      = load_f32_le(d);      d += 4;
        out->quat[1]      = load_f32_le(d);      d += 4;
        out->quat[2]      = load_f32_le(d);      d += 4;
        out->quat[3]      = load_f32_le(d);
    } else if (pkt->cls == IMU_CLASS_TILT_COMPACT && pkt->length == IMU_TILT_COMPACT_PAYLOAD_LEN) {
        out->pitch        = load_f32_le(d);      d += 4;
        out->roll         = load_f32_le(d);      d += 4;
        out->yaw          = load_f32_le(d);      d += 4;
        out->temperature  = load_f32_le(d);
    } else {
        return -1;
    }

    return 0;
}

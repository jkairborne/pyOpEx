#pragma once
// MESSAGE CA_TRAJECTORY PACKING

#define MAVLINK_MSG_ID_CA_TRAJECTORY 150

MAVPACKED(
typedef struct __mavlink_ca_trajectory_t {
 uint32_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float roll; /*< Roll*/
 float pitch; /*< Pitch*/
 float yaw; /*< Yaw*/
 float rollrate; /*< RollRate*/
 float pitchrate; /*< PitchRate*/
 float yawrate; /*< YawRate*/
}) mavlink_ca_trajectory_t;

#define MAVLINK_MSG_ID_CA_TRAJECTORY_LEN 28
#define MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN 28
#define MAVLINK_MSG_ID_150_LEN 28
#define MAVLINK_MSG_ID_150_MIN_LEN 28

#define MAVLINK_MSG_ID_CA_TRAJECTORY_CRC 151
#define MAVLINK_MSG_ID_150_CRC 151



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CA_TRAJECTORY { \
    150, \
    "CA_TRAJECTORY", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ca_trajectory_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ca_trajectory_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ca_trajectory_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ca_trajectory_t, yaw) }, \
         { "rollrate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ca_trajectory_t, rollrate) }, \
         { "pitchrate", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ca_trajectory_t, pitchrate) }, \
         { "yawrate", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ca_trajectory_t, yawrate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CA_TRAJECTORY { \
    "CA_TRAJECTORY", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ca_trajectory_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ca_trajectory_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ca_trajectory_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ca_trajectory_t, yaw) }, \
         { "rollrate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ca_trajectory_t, rollrate) }, \
         { "pitchrate", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ca_trajectory_t, pitchrate) }, \
         { "yawrate", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ca_trajectory_t, yawrate) }, \
         } \
}
#endif

/**
 * @brief Pack a ca_trajectory message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 * @param rollrate RollRate
 * @param pitchrate PitchRate
 * @param yawrate YawRate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_trajectory_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_usec, float roll, float pitch, float yaw, float rollrate, float pitchrate, float yawrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECTORY_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollrate);
    _mav_put_float(buf, 20, pitchrate);
    _mav_put_float(buf, 24, yawrate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#else
    mavlink_ca_trajectory_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollrate = rollrate;
    packet.pitchrate = pitchrate;
    packet.yawrate = yawrate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECTORY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
}

/**
 * @brief Pack a ca_trajectory message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 * @param rollrate RollRate
 * @param pitchrate PitchRate
 * @param yawrate YawRate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_trajectory_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_usec,float roll,float pitch,float yaw,float rollrate,float pitchrate,float yawrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECTORY_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollrate);
    _mav_put_float(buf, 20, pitchrate);
    _mav_put_float(buf, 24, yawrate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#else
    mavlink_ca_trajectory_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollrate = rollrate;
    packet.pitchrate = pitchrate;
    packet.yawrate = yawrate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECTORY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
}

/**
 * @brief Encode a ca_trajectory struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ca_trajectory C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_trajectory_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ca_trajectory_t* ca_trajectory)
{
    return mavlink_msg_ca_trajectory_pack(system_id, component_id, msg, ca_trajectory->time_usec, ca_trajectory->roll, ca_trajectory->pitch, ca_trajectory->yaw, ca_trajectory->rollrate, ca_trajectory->pitchrate, ca_trajectory->yawrate);
}

/**
 * @brief Encode a ca_trajectory struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ca_trajectory C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_trajectory_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ca_trajectory_t* ca_trajectory)
{
    return mavlink_msg_ca_trajectory_pack_chan(system_id, component_id, chan, msg, ca_trajectory->time_usec, ca_trajectory->roll, ca_trajectory->pitch, ca_trajectory->yaw, ca_trajectory->rollrate, ca_trajectory->pitchrate, ca_trajectory->yawrate);
}

/**
 * @brief Send a ca_trajectory message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 * @param rollrate RollRate
 * @param pitchrate PitchRate
 * @param yawrate YawRate
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ca_trajectory_send(mavlink_channel_t chan, uint32_t time_usec, float roll, float pitch, float yaw, float rollrate, float pitchrate, float yawrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECTORY_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollrate);
    _mav_put_float(buf, 20, pitchrate);
    _mav_put_float(buf, 24, yawrate);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, buf, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#else
    mavlink_ca_trajectory_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollrate = rollrate;
    packet.pitchrate = pitchrate;
    packet.yawrate = yawrate;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, (const char *)&packet, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#endif
}

/**
 * @brief Send a ca_trajectory message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ca_trajectory_send_struct(mavlink_channel_t chan, const mavlink_ca_trajectory_t* ca_trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ca_trajectory_send(chan, ca_trajectory->time_usec, ca_trajectory->roll, ca_trajectory->pitch, ca_trajectory->yaw, ca_trajectory->rollrate, ca_trajectory->pitchrate, ca_trajectory->yawrate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, (const char *)ca_trajectory, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#endif
}

#if MAVLINK_MSG_ID_CA_TRAJECTORY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ca_trajectory_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_usec, float roll, float pitch, float yaw, float rollrate, float pitchrate, float yawrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollrate);
    _mav_put_float(buf, 20, pitchrate);
    _mav_put_float(buf, 24, yawrate);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, buf, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#else
    mavlink_ca_trajectory_t *packet = (mavlink_ca_trajectory_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->rollrate = rollrate;
    packet->pitchrate = pitchrate;
    packet->yawrate = yawrate;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, (const char *)packet, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#endif
}
#endif

#endif

// MESSAGE CA_TRAJECTORY UNPACKING


/**
 * @brief Get field time_usec from ca_trajectory message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint32_t mavlink_msg_ca_trajectory_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from ca_trajectory message
 *
 * @return Roll
 */
static inline float mavlink_msg_ca_trajectory_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from ca_trajectory message
 *
 * @return Pitch
 */
static inline float mavlink_msg_ca_trajectory_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from ca_trajectory message
 *
 * @return Yaw
 */
static inline float mavlink_msg_ca_trajectory_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rollrate from ca_trajectory message
 *
 * @return RollRate
 */
static inline float mavlink_msg_ca_trajectory_get_rollrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitchrate from ca_trajectory message
 *
 * @return PitchRate
 */
static inline float mavlink_msg_ca_trajectory_get_pitchrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yawrate from ca_trajectory message
 *
 * @return YawRate
 */
static inline float mavlink_msg_ca_trajectory_get_yawrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a ca_trajectory message into a struct
 *
 * @param msg The message to decode
 * @param ca_trajectory C-struct to decode the message contents into
 */
static inline void mavlink_msg_ca_trajectory_decode(const mavlink_message_t* msg, mavlink_ca_trajectory_t* ca_trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ca_trajectory->time_usec = mavlink_msg_ca_trajectory_get_time_usec(msg);
    ca_trajectory->roll = mavlink_msg_ca_trajectory_get_roll(msg);
    ca_trajectory->pitch = mavlink_msg_ca_trajectory_get_pitch(msg);
    ca_trajectory->yaw = mavlink_msg_ca_trajectory_get_yaw(msg);
    ca_trajectory->rollrate = mavlink_msg_ca_trajectory_get_rollrate(msg);
    ca_trajectory->pitchrate = mavlink_msg_ca_trajectory_get_pitchrate(msg);
    ca_trajectory->yawrate = mavlink_msg_ca_trajectory_get_yawrate(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CA_TRAJECTORY_LEN? msg->len : MAVLINK_MSG_ID_CA_TRAJECTORY_LEN;
        memset(ca_trajectory, 0, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
    memcpy(ca_trajectory, _MAV_PAYLOAD(msg), len);
#endif
}

/* Généré par tools/codegen.py depuis schema/link.yaml. Ne pas modifier à la main. */
#ifndef ARIANE_LINK_SCHEMA_H
#define ARIANE_LINK_SCHEMA_H

#include <stdint.h>

#define LINK_PROTO_VERSION 1
#define LINK_SOF 0xA5
#define LINK_MAX_PAYLOAD 512
#define LINK_SCHEMA_CRC 0x2CB70B58u

/* enum Robot */
#define LINK_ROBOT_ZHONX2 2
#define LINK_ROBOT_ZHONX3 3

/* enum FrameType */
#define LINK_FRAME_TYPE_HELLO 1
#define LINK_FRAME_TYPE_PING 2
#define LINK_FRAME_TYPE_CMD 3
#define LINK_FRAME_TYPE_ACK 4
#define LINK_FRAME_TYPE_RESEND 5
#define LINK_FRAME_TYPE_TELEMETRY 16
#define LINK_FRAME_TYPE_EVENT 17
#define LINK_FRAME_TYPE_LOG 18
#define LINK_FRAME_TYPE_HELLO_ACK 129
#define LINK_FRAME_TYPE_PONG 130
#define LINK_FRAME_TYPE_CMD_ACK 131
#define LINK_FRAME_TYPE_PARAM_INFO 132
#define LINK_FRAME_TYPE_PARAM_VALUE 133
#define LINK_FRAME_TYPE_DUMP_DATA 134

/* enum Command */
#define LINK_COMMAND_STOP 1
#define LINK_COMMAND_LED_SYNC 2
#define LINK_COMMAND_MOVE 3
#define LINK_COMMAND_TURN 4
#define LINK_COMMAND_EXPLORE 16
#define LINK_COMMAND_RUN 17
#define LINK_COMMAND_CALIB 32
#define LINK_COMMAND_TELEM_CFG 48
#define LINK_COMMAND_FLUSH 49
#define LINK_COMMAND_PARAM_LIST 64
#define LINK_COMMAND_PARAM_GET 65
#define LINK_COMMAND_PARAM_SET 66
#define LINK_COMMAND_PARAM_SAVE 67
#define LINK_COMMAND_DUMP 80
#define LINK_COMMAND_CONSOLE 96
#define LINK_COMMAND_HALT 112
#define LINK_COMMAND_REBOOT 113
#define LINK_COMMAND_BOOTLOADER 114

/* enum CmdStatus */
#define LINK_CMD_STATUS_STARTED 0
#define LINK_CMD_STATUS_DONE 1
#define LINK_CMD_STATUS_BUSY 2
#define LINK_CMD_STATUS_INVALID 3
#define LINK_CMD_STATUS_REFUSED 4
#define LINK_CMD_STATUS_ERROR 5
#define LINK_CMD_STATUS_UNSUPPORTED 6

/* enum CmdFlags */
#define LINK_CMD_FLAGS_LED_SYNC_BEFORE_START 1
#define LINK_CMD_FLAGS_WAIT_HAND 2
#define LINK_CMD_FLAGS_LED_BLIPS_DURING_MOTION 4

/* enum MoveKind */
#define LINK_MOVE_KIND_CELLS 0
#define LINK_MOVE_KIND_START_CELL 1

/* enum TurnKind */
#define LINK_TURN_KIND_IN_PLACE_CW90 0
#define LINK_TURN_KIND_IN_PLACE_CCW90 1
#define LINK_TURN_KIND_IN_PLACE_180 2
#define LINK_TURN_KIND_CURVE_CW90 3
#define LINK_TURN_KIND_CURVE_CCW90 4
#define LINK_TURN_KIND_UTURN 5

/* enum CalibKind */
#define LINK_CALIB_KIND_GYRO 0
#define LINK_CALIB_KIND_WALLS_FRONT 1
#define LINK_CALIB_KIND_WALLS_DIAG 2
#define LINK_CALIB_KIND_SPYPOST 3
#define LINK_CALIB_KIND_SPYWALL 4

/* enum Event */
#define LINK_EVENT_BOOT 1
#define LINK_EVENT_LED_SYNC 2
#define LINK_EVENT_START 3
#define LINK_EVENT_STOP 4
#define LINK_EVENT_FAULT 5
#define LINK_EVENT_CELL 6
#define LINK_EVENT_DECISION 7
#define LINK_EVENT_MOVE_BEGIN 8
#define LINK_EVENT_MOVE_END 9
#define LINK_EVENT_CMD_DONE 10
#define LINK_EVENT_DROPPED 11
#define LINK_EVENT_CALIB 12
#define LINK_EVENT_HAND_WAIT 13
#define LINK_EVENT_HAND_GO 14
#define LINK_EVENT_BATTERY 15
#define LINK_EVENT_RUN_DONE 16
#define LINK_EVENT_DUMP_DONE 17
#define LINK_EVENT_PARAMS_SAVED 18
#define LINK_EVENT_TELEM_CFG 19

/* enum LogLevel */
#define LINK_LOG_LEVEL_DEBUG 0
#define LINK_LOG_LEVEL_INFO 1
#define LINK_LOG_LEVEL_WARN 2
#define LINK_LOG_LEVEL_ERROR 3

/* enum ParamType */
#define LINK_PARAM_TYPE_I32 0
#define LINK_PARAM_TYPE_F32 1
#define LINK_PARAM_TYPE_BOOL 2

/* enum ParamFlags */
#define LINK_PARAM_FLAGS_PERSISTENT 1
#define LINK_PARAM_FLAGS_READ_ONLY 2

/* enum Heading */
#define LINK_HEADING_NORTH 0
#define LINK_HEADING_EAST 1
#define LINK_HEADING_SOUTH 2
#define LINK_HEADING_WEST 3
#define LINK_HEADING_UNKNOWN 255

/* enum MotionState */
#define LINK_MOTION_STATE_IDLE 0
#define LINK_MOTION_STATE_STRAIGHT 1
#define LINK_MOTION_STATE_TURN_IN_PLACE 2
#define LINK_MOTION_STATE_CURVE 3
#define LINK_MOTION_STATE_STOPPING 4
#define LINK_MOTION_STATE_WAIT_HAND 5
#define LINK_MOTION_STATE_LED_SYNC 6
#define LINK_MOTION_STATE_CALIB 7
#define LINK_MOTION_STATE_FAULT 8
#define LINK_MOTION_STATE_UNKNOWN 255

/* enum TelemetryFlags */
#define LINK_TELEMETRY_FLAGS_LED_ON 1
#define LINK_TELEMETRY_FLAGS_MOTION_BUSY 2
#define LINK_TELEMETRY_FLAGS_FAULT 4
#define LINK_TELEMETRY_FLAGS_WALL_FOLLOW 8
#define LINK_TELEMETRY_FLAGS_LINE_FOLLOW 16

/* enum WallBits */
#define LINK_WALL_BITS_FL 1
#define LINK_WALL_BITS_FR 2
#define LINK_WALL_BITS_DL 4
#define LINK_WALL_BITS_DR 8

#pragma pack(push, 1)
/* HELLO, téléphone -> robot */
typedef struct {
    uint8_t proto_version;
    uint8_t client;  /* 1 = application Android, 2 = outil PC */
    uint8_t flags;
    uint8_t reserved;
} link_hello_req_t;
#define LINK_HELLO_REQ_SIZE 4
_Static_assert(sizeof(link_hello_req_t) == 4, "link_hello_req_t packed size");

/* HELLO_ACK, robot -> téléphone */
typedef struct {
    uint8_t proto_version;
    uint8_t robot;  /* enum Robot */
    uint8_t telemetry_hz;
    uint8_t reserved;
    uint32_t schema_crc;  /* doit être égal à LINK_SCHEMA_CRC du décodeur */
    uint32_t build_id;  /* CRC32 de l'image firmware */
    char fw_name[16];
    uint32_t caps;  /* réservé, capacités */
    uint16_t ring_records;  /* capacité de l'anneau de télémétrie */
    uint16_t max_payload;
} link_hello_ack_t;
#define LINK_HELLO_ACK_SIZE 36
_Static_assert(sizeof(link_hello_ack_t) == 36, "link_hello_ack_t packed size");

typedef struct {
    uint32_t token;
} link_ping_t;
#define LINK_PING_SIZE 4
_Static_assert(sizeof(link_ping_t) == 4, "link_ping_t packed size");

typedef struct {
    uint32_t token;
    uint32_t t_robot_us;  /* temps robot à la réception du PING */
} link_pong_t;
#define LINK_PONG_SIZE 8
_Static_assert(sizeof(link_pong_t) == 8, "link_pong_t packed size");

/* début de toute trame CMD ; les arguments suivent selon la commande */
typedef struct {
    uint8_t cmd;  /* enum Command */
    uint8_t job;  /* identifiant choisi par l'app, renvoyé dans CMD_ACK et CMD_DONE */
} link_cmd_header_t;
#define LINK_CMD_HEADER_SIZE 2
_Static_assert(sizeof(link_cmd_header_t) == 2, "link_cmd_header_t packed size");

typedef struct {
    uint8_t cmd;
    uint8_t job;
    uint8_t status;  /* enum CmdStatus */
    uint8_t detail;
} link_cmd_ack_t;
#define LINK_CMD_ACK_SIZE 4
_Static_assert(sizeof(link_cmd_ack_t) == 4, "link_cmd_ack_t packed size");

typedef struct {
    uint8_t kind;  /* enum MoveKind */
    uint8_t cells;
    uint16_t speed_mm_s;
    uint16_t end_speed_mm_s;
    uint8_t flags;  /* enum CmdFlags */
} link_cmd_move_t;
#define LINK_CMD_MOVE_SIZE 7
_Static_assert(sizeof(link_cmd_move_t) == 7, "link_cmd_move_t packed size");

typedef struct {
    uint8_t kind;  /* enum TurnKind */
    uint16_t speed_mm_s;
    uint8_t flags;
} link_cmd_turn_t;
#define LINK_CMD_TURN_SIZE 4
_Static_assert(sizeof(link_cmd_turn_t) == 4, "link_cmd_turn_t packed size");

typedef struct {
    uint16_t speed_mm_s;  /* 0 = réglage du robot */
    uint8_t flags;
} link_cmd_explore_t;
#define LINK_CMD_EXPLORE_SIZE 3
_Static_assert(sizeof(link_cmd_explore_t) == 3, "link_cmd_explore_t packed size");

typedef struct {
    uint8_t run_type;  /* 1 ou 2 comme sur le robot */
    uint16_t speed_mm_s;  /* 0 = réglage du robot */
    uint8_t flags;
} link_cmd_run_t;
#define LINK_CMD_RUN_SIZE 4
_Static_assert(sizeof(link_cmd_run_t) == 4, "link_cmd_run_t packed size");

typedef struct {
    uint8_t kind;  /* enum CalibKind */
    uint8_t flags;
} link_cmd_calib_t;
#define LINK_CMD_CALIB_SIZE 2
_Static_assert(sizeof(link_cmd_calib_t) == 2, "link_cmd_calib_t packed size");

typedef struct {
    uint8_t pattern;  /* 0 = séquence de départ, 1 = bip court */
} link_cmd_led_sync_t;
#define LINK_CMD_LED_SYNC_SIZE 1
_Static_assert(sizeof(link_cmd_led_sync_t) == 1, "link_cmd_led_sync_t packed size");

typedef struct {
    uint8_t rate_hz;  /* 0 = stop, sinon 1..200 */
    uint8_t enable_log;
} link_cmd_telem_cfg_t;
#define LINK_CMD_TELEM_CFG_SIZE 2
_Static_assert(sizeof(link_cmd_telem_cfg_t) == 2, "link_cmd_telem_cfg_t packed size");

typedef struct {
    uint16_t id;
} link_cmd_param_id_t;
#define LINK_CMD_PARAM_ID_SIZE 2
_Static_assert(sizeof(link_cmd_param_id_t) == 2, "link_cmd_param_id_t packed size");

typedef struct {
    uint16_t id;
    float value;
} link_cmd_param_set_t;
#define LINK_CMD_PARAM_SET_SIZE 6
_Static_assert(sizeof(link_cmd_param_set_t) == 6, "link_cmd_param_set_t packed size");

typedef struct {
    uint32_t addr;
    uint32_t len;
} link_cmd_dump_t;
#define LINK_CMD_DUMP_SIZE 8
_Static_assert(sizeof(link_cmd_dump_t) == 8, "link_cmd_dump_t packed size");

/* ACK téléphone -> robot : dernier numéro reçu en séquence continue */
typedef struct {
    uint32_t telemetry_seq;
    uint32_t event_seq;
} link_stream_ack_t;
#define LINK_STREAM_ACK_SIZE 8
_Static_assert(sizeof(link_stream_ack_t) == 8, "link_stream_ack_t packed size");

typedef struct {
    uint8_t stream;  /* 0 = télémétrie, 1 = événements */
    uint8_t reserved;
    uint32_t from_seq;
    uint32_t to_seq;
} link_resend_t;
#define LINK_RESEND_SIZE 10
_Static_assert(sizeof(link_resend_t) == 10, "link_resend_t packed size");

/* en-tête de TELEMETRY, suivi de count enregistrements TelemetryRecord */
typedef struct {
    uint32_t first_seq;
    uint8_t count;
    uint8_t record_size;  /* = TelemetryRecord.SIZE, garde-fou */
} link_telemetry_batch_t;
#define LINK_TELEMETRY_BATCH_SIZE 6
_Static_assert(sizeof(link_telemetry_batch_t) == 6, "link_telemetry_batch_t packed size");

/* un échantillon périodique de l'état du robot */
typedef struct {
    uint32_t t_us;  /* timer robot 1 MHz, 32 bits, déroulé côté app */
    int8_t cell_x;
    int8_t cell_y;
    uint8_t heading;  /* enum Heading */
    uint8_t motion;  /* enum MotionState */
    int32_t enc_left_um;  /* distance encodeur gauche depuis le démarrage */
    int32_t enc_right_um;
    int32_t gyro_cdeg;  /* cap intégré par le gyro, centièmes de degré */
    int16_t speed_mm_s;
    int16_t omega_ddps;  /* vitesse de rotation, dixièmes de degré par seconde */
    uint16_t tel_fl_mm;
    uint16_t tel_fr_mm;
    uint16_t tel_dl_mm;
    uint16_t tel_dr_mm;
    uint8_t walls;  /* enum WallBits */
    uint8_t flags;  /* enum TelemetryFlags */
    uint16_t bat_mv;
    int16_t target_mm;  /* distance restante de la consigne courante */
} link_telemetry_record_t;
#define LINK_TELEMETRY_RECORD_SIZE 38
_Static_assert(sizeof(link_telemetry_record_t) == 38, "link_telemetry_record_t packed size");

/* début de toute trame EVENT ; les arguments suivent selon l'événement */
typedef struct {
    uint32_t seq;
    uint32_t t_us;
    uint8_t event;  /* enum Event */
    uint8_t len;  /* longueur des arguments */
} link_event_header_t;
#define LINK_EVENT_HEADER_SIZE 10
_Static_assert(sizeof(link_event_header_t) == 10, "link_event_header_t packed size");

typedef struct {
    uint8_t index;  /* 0..3 séquence de départ, 100+ bips en course */
    uint8_t state;  /* 1 = allumage, 0 = extinction */
} link_ev_led_sync_t;
#define LINK_EV_LED_SYNC_SIZE 2
_Static_assert(sizeof(link_ev_led_sync_t) == 2, "link_ev_led_sync_t packed size");

typedef struct {
    uint8_t mode;
} link_ev_start_t;
#define LINK_EV_START_SIZE 1
_Static_assert(sizeof(link_ev_start_t) == 1, "link_ev_start_t packed size");

typedef struct {
    uint8_t reason;
} link_ev_stop_t;
#define LINK_EV_STOP_SIZE 1
_Static_assert(sizeof(link_ev_stop_t) == 1, "link_ev_stop_t packed size");

typedef struct {
    uint8_t code;
    uint16_t detail;
} link_ev_fault_t;
#define LINK_EV_FAULT_SIZE 3
_Static_assert(sizeof(link_ev_fault_t) == 3, "link_ev_fault_t packed size");

typedef struct {
    int8_t x;
    int8_t y;
    uint8_t heading;
    uint8_t walls;  /* murs observés dans la cellule, bits N E S W */
} link_ev_cell_t;
#define LINK_EV_CELL_SIZE 4
_Static_assert(sizeof(link_ev_cell_t) == 4, "link_ev_cell_t packed size");

typedef struct {
    uint8_t direction;  /* 0 tout droit, 1 droite, 2 demi-tour, 3 gauche */
    int8_t x;
    int8_t y;
} link_ev_decision_t;
#define LINK_EV_DECISION_SIZE 3
_Static_assert(sizeof(link_ev_decision_t) == 3, "link_ev_decision_t packed size");

typedef struct {
    uint8_t kind;
    int16_t param;
    uint16_t speed_mm_s;
} link_ev_move_begin_t;
#define LINK_EV_MOVE_BEGIN_SIZE 5
_Static_assert(sizeof(link_ev_move_begin_t) == 5, "link_ev_move_begin_t packed size");

typedef struct {
    uint8_t kind;
    uint8_t status;
} link_ev_move_end_t;
#define LINK_EV_MOVE_END_SIZE 2
_Static_assert(sizeof(link_ev_move_end_t) == 2, "link_ev_move_end_t packed size");

typedef struct {
    uint8_t cmd;
    uint8_t job;
    uint8_t status;
} link_ev_cmd_done_t;
#define LINK_EV_CMD_DONE_SIZE 3
_Static_assert(sizeof(link_ev_cmd_done_t) == 3, "link_ev_cmd_done_t packed size");

typedef struct {
    uint16_t records;
    uint8_t events;
} link_ev_dropped_t;
#define LINK_EV_DROPPED_SIZE 3
_Static_assert(sizeof(link_ev_dropped_t) == 3, "link_ev_dropped_t packed size");

typedef struct {
    uint8_t kind;
    uint8_t status;
    float value;
} link_ev_calib_t;
#define LINK_EV_CALIB_SIZE 6
_Static_assert(sizeof(link_ev_calib_t) == 6, "link_ev_calib_t packed size");

typedef struct {
    uint16_t mv;
} link_ev_battery_t;
#define LINK_EV_BATTERY_SIZE 2
_Static_assert(sizeof(link_ev_battery_t) == 2, "link_ev_battery_t packed size");

typedef struct {
    uint32_t ms;
    uint8_t status;
} link_ev_run_done_t;
#define LINK_EV_RUN_DONE_SIZE 5
_Static_assert(sizeof(link_ev_run_done_t) == 5, "link_ev_run_done_t packed size");

typedef struct {
    uint8_t status;
} link_ev_dump_done_t;
#define LINK_EV_DUMP_DONE_SIZE 1
_Static_assert(sizeof(link_ev_dump_done_t) == 1, "link_ev_dump_done_t packed size");

/* début de toute trame LOG ; le texte UTF-8 suit, sans zéro final */
typedef struct {
    uint32_t t_us;
    uint8_t level;  /* enum LogLevel */
} link_log_header_t;
#define LINK_LOG_HEADER_SIZE 5
_Static_assert(sizeof(link_log_header_t) == 5, "link_log_header_t packed size");

typedef struct {
    uint16_t id;
    uint8_t type;  /* enum ParamType */
    uint8_t flags;  /* enum ParamFlags */
    char name[16];
    float min_value;
    float max_value;
    float value;
    uint16_t count;  /* nombre total de paramètres */
    uint16_t index;
} link_param_info_t;
#define LINK_PARAM_INFO_SIZE 36
_Static_assert(sizeof(link_param_info_t) == 36, "link_param_info_t packed size");

typedef struct {
    uint16_t id;
    uint8_t status;
    uint8_t type;
    float value;
} link_param_value_t;
#define LINK_PARAM_VALUE_SIZE 8
_Static_assert(sizeof(link_param_value_t) == 8, "link_param_value_t packed size");

/* début de DUMP_DATA ; len octets suivent */
typedef struct {
    uint32_t addr;
    uint16_t len;
} link_dump_header_t;
#define LINK_DUMP_HEADER_SIZE 6
_Static_assert(sizeof(link_dump_header_t) == 6, "link_dump_header_t packed size");

#pragma pack(pop)

#endif /* ARIANE_LINK_SCHEMA_H */

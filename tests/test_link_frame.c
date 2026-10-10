/*
 * Host check of the Ariane Link framing against the reference vectors of
 * ariane-tools/link (tests/link/vectors.json, copied by scripts/link/sync.sh):
 * CRC-16 known answer, encoding of each frame, parser resynchronisation
 * on a corrupted stream with stray bytes, byte-by-byte parsing, schema
 * constants. The JSON is read with a minimal scanner: no dependency.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "middleware/link/ariane_link_schema.h"
#include "middleware/link/link_frame.h"

static char *text;

/* Pointer to the value of the first "key" at or after from. */
static const char *value_of(const char *from, const char *key)
{
    char quoted[64];
    const char *p;

    snprintf(quoted, sizeof quoted, "\"%s\"", key);
    p = strstr(from, quoted);
    assert(p != NULL);
    p = strchr(p + strlen(quoted), ':');
    assert(p != NULL);
    p++;
    while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t')
        p++;
    return p;
}

static long int_value(const char *from, const char *key)
{
    return strtol(value_of(from, key), NULL, 10);
}

/* The string value, copied; the caller frees it. */
static char *string_value(const char *from, const char *key)
{
    const char *p = value_of(from, key);
    const char *end;
    char *out;

    assert(*p == '"');
    end = strchr(p + 1, '"');
    assert(end != NULL);
    out = malloc((size_t)(end - p));
    memcpy(out, p + 1, (size_t)(end - p - 1));
    out[end - p - 1] = '\0';
    return out;
}

static size_t unhex(const char *hex, uint8_t *out, size_t cap)
{
    size_t n = strlen(hex) / 2;
    assert(n <= cap);
    for (size_t i = 0; i < n; i++)
    {
        unsigned v;
        sscanf(hex + 2 * i, "%2x", &v);
        out[i] = (uint8_t)v;
    }
    return n;
}

static void hex(const uint8_t *data, size_t n, char *out)
{
    for (size_t i = 0; i < n; i++)
        snprintf(out + 2 * i, 3, "%02x", data[i]);
    out[2 * n] = '\0';
}

typedef struct
{
    int type, seq;
    char *payload, *frame;
} vector_t;

static vector_t vectors[16];
static int vector_count;

static void load(const char *path)
{
    FILE *f = fopen(path, "rb");
    long size;
    const char *p, *stream;

    assert(f != NULL);
    fseek(f, 0, SEEK_END);
    size = ftell(f);
    fseek(f, 0, SEEK_SET);
    text = malloc((size_t)size + 1);
    assert(fread(text, 1, (size_t)size, f) == (size_t)size);
    text[size] = '\0';
    fclose(f);

    p = value_of(text, "frames");
    stream = strstr(text, "\"stream\"");
    assert(stream != NULL);
    while ((p = strstr(p, "\"type\"")) != NULL && p < stream)
    {
        vector_t *v = &vectors[vector_count++];
        v->type = (int)int_value(p, "type");
        v->seq = (int)int_value(p, "seq");
        v->payload = string_value(p, "payload");
        v->frame = string_value(p, "frame");
        p = value_of(p, "frame");
    }
    assert(vector_count >= 5);
}

static void crc_known_answer(void)
{
    const char *p = value_of(text, "crc16_check");
    char *input = string_value(p, "input");
    long crc = int_value(p, "crc");

    assert(link_crc16((const uint8_t *)input, strlen(input), 0xFFFF) == (uint16_t)crc);
    assert(crc == 0x29B1);
    free(input);
    puts("crc16: CCITT-FALSE known answer 0x29B1");
}

static void encode_matches(void)
{
    for (int i = 0; i < vector_count; i++)
    {
        uint8_t payload[LINK_FRAME_MAX_PAYLOAD], frame[LINK_FRAME_OVERHEAD + LINK_FRAME_MAX_PAYLOAD];
        char got[2 * sizeof frame + 1];
        size_t len = unhex(vectors[i].payload, payload, sizeof payload);
        size_t n = link_frame_encode(frame, sizeof frame, (uint8_t)vectors[i].type, (uint8_t)vectors[i].seq,
                                     payload, (uint16_t)len);
        assert(n == LINK_FRAME_OVERHEAD + len);
        hex(frame, n, got);
        assert(strcmp(got, vectors[i].frame) == 0);
    }
    /* Oversize payload and small buffers are refused. */
    {
        uint8_t small[8], payload[4] = { 0 };
        assert(link_frame_encode(small, sizeof small, 1, 0, payload, 4) == 0);
        assert(link_frame_encode(small, sizeof small, 1, 0, payload, 1) == 8);
        assert(link_frame_encode(small, sizeof small, 1, 0, payload, LINK_FRAME_MAX_PAYLOAD + 1) == 0);
    }
    printf("encode: %d vectors identical to ariane-tools/link\n", vector_count);
}

static void parser_resync(void)
{
    const char *p = value_of(text, "stream");
    char *stream_hex = string_value(p, "hex");
    char *expect_stray = string_value(p, "expect_stray");
    long expect_crc_errors = int_value(p, "expect_crc_errors");
    const char *expect_frames = value_of(p, "expect_frames");
    uint8_t stream[2048], stray[64];
    char stray_hex[129];
    size_t n = unhex(stream_hex, stream, sizeof stream), stray_n = 0;
    int frames_seen = 0;
    link_parser_t parser;
    link_frame_t frame;

    link_parser_init(&parser);
    for (size_t i = 0; i < n; i++)
    {
        int r = link_parser_feed(&parser, stream[i], &frame);
        if (r == LINK_FEED_STRAY)
        {
            stray[stray_n++] = stream[i];
        }
        else if (r == LINK_FEED_FRAME)
        {
            /* The expected frame is vectors[expect_frames[frames_seen]]. */
            const char *q = expect_frames;
            uint8_t payload[LINK_FRAME_MAX_PAYLOAD];
            size_t len;
            long index;
            for (int k = 0; k <= frames_seen; k++)
            {
                q = strpbrk(q, "0123456789");
                assert(q != NULL);
                index = strtol(q, (char **)&q, 10);
            }
            len = unhex(vectors[index].payload, payload, sizeof payload);
            assert(frame.type == vectors[index].type && frame.seq == vectors[index].seq);
            assert(frame.len == len && memcmp(frame.payload, payload, len) == 0);
            frames_seen++;
        }
    }
    assert(frames_seen == 3);
    assert(parser.crc_errors == (uint32_t)expect_crc_errors);
    assert(parser.length_errors == 0);
    hex(stray, stray_n, stray_hex);
    assert(strcmp(stray_hex, expect_stray) == 0);
    free(stream_hex);
    free(expect_stray);
    printf("parser: %d frames, %lu CRC error, stray bytes %s as expected\n", frames_seen,
           (unsigned long)parser.crc_errors, stray_hex);
}

static void parser_byte_by_byte(void)
{
    char *all = string_value(text, "all_good_hex");
    uint8_t stream[2048];
    size_t n = unhex(all, stream, sizeof stream);
    int frames = 0;
    link_parser_t parser;
    link_frame_t frame;

    link_parser_init(&parser);
    for (size_t i = 0; i < n; i++)
        if (link_parser_feed(&parser, stream[i], &frame) == LINK_FEED_FRAME)
            frames++;
    assert(frames == vector_count);
    /* A length above the maximum drops the frame and resumes the hunt. */
    {
        uint8_t bad[] = { 0xA5, 0x10, 0x00, 0x01, 0x02, 0x41 };
        int strays = 0;
        for (size_t i = 0; i < sizeof bad; i++)
            if (link_parser_feed(&parser, bad[i], &frame) == LINK_FEED_STRAY)
                strays++;
        assert(parser.length_errors == 1 && strays == 1);
    }
    free(all);
    printf("parser: %d frames byte by byte, oversize length rejected\n", frames);
}

static void schema_constants(void)
{
    assert(int_value(text, "telemetry_record_size") == LINK_TELEMETRY_RECORD_SIZE);
    assert((uint32_t)strtoul(value_of(text, "schema_crc"), NULL, 10) == LINK_SCHEMA_CRC);
    assert(LINK_MAX_PAYLOAD == LINK_FRAME_MAX_PAYLOAD);
    assert(sizeof(link_hello_ack_t) == LINK_HELLO_ACK_SIZE);
    printf("schema: record size %d, CRC 0x%08lX as in vectors.json\n", LINK_TELEMETRY_RECORD_SIZE,
           (unsigned long)LINK_SCHEMA_CRC);
}

int main(int argc, char **argv)
{
    load(argc > 1 ? argv[1] : "tests/link/vectors.json");
    crc_known_answer();
    encode_matches();
    parser_resync();
    parser_byte_by_byte();
    schema_constants();
    puts("test_link_frame: OK");
    return 0;
}

/*
 * Protocol test for the Siemens keyboard/terminal controller model.
 *
 * Exercises the only exchange the firmware is known to implement -- the POST's
 * "ESC [ 2 y" identity query (v3 BIOS 0xfd8ab) and its seven byte answer --
 * plus the status bits that carry it and the cases that must stay silent.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <86box/kbc_siemens.h>

/* The device logs through log_open()/log_close(); the test does not need the
   logging back end. */
void *
log_open(const char *dev_name)
{
    (void) dev_name;
    return NULL;
}

void
log_close(void *priv)
{
    (void) priv;
}

static int failures;
static int checks;

static void
check(int cond, const char *what)
{
    checks++;

    if (!cond) {
        failures++;
        printf("FAIL %s\n", what);
    }
}

static void
send(kbc_siemens_t *kbc, const char *str)
{
    while (*str != '\0')
        kbc_siemens_rx(kbc, (uint8_t) *str++);
}

int
main(void)
{
    kbc_siemens_t *kbc;
    uint8_t        reply[8];
    int            i;

    kbc = kbc_siemens_init();
    check(kbc != NULL, "init");
    if (kbc == NULL)
        return 1;

    /* Nothing to say before being asked. */
    check((kbc_siemens_read_status(kbc) & 0x01) == 0, "status idle before a query");

    /* The exact exchange the POST performs: print "ESC [ 2 y", then read. */
    send(kbc, "\x1b[2y");
    check((kbc_siemens_read_status(kbc) & 0x01) == 0x01, "status: data available");

    memset(reply, 0, sizeof(reply));
    for (i = 0; i < 7; i++)
        reply[i] = kbc_siemens_read_data(kbc);

    check(memcmp(reply, "\x1bP2y0\x1b\\", 7) == 0, "identity answer bytes");
    check((reply[4] - 0x30) == 0, "fifth byte is '0', as the POST checks");
    check((kbc_siemens_read_status(kbc) & 0x01) == 0, "status idle after the answer");

    /* Ordinary text must not produce an answer. */
    send(kbc, "hello");
    check((kbc_siemens_read_status(kbc) & 0x01) == 0, "text stays silent");

    /* Another escape sequence with a final byte is ignored too. */
    send(kbc, "\x1b[1;31m");
    check((kbc_siemens_read_status(kbc) & 0x01) == 0, "colour sequence stays silent");

    /* A query split across writes still works, the way a slow 8031 would. */
    kbc_siemens_rx(kbc, 0x1b);
    kbc_siemens_rx(kbc, '[');
    kbc_siemens_rx(kbc, '2');
    kbc_siemens_rx(kbc, 'y');
    check((kbc_siemens_read_status(kbc) & 0x01) == 0x01, "split query answered");
    for (i = 0; i < 7; i++)
        reply[i] = kbc_siemens_read_data(kbc);
    check(memcmp(reply, "\x1bP2y0\x1b\\", 7) == 0, "split query answer bytes");

    /* A wider parameter must not match the "2" query. */
    send(kbc, "\x1b[22y");
    check((kbc_siemens_read_status(kbc) & 0x01) == 0, "parameter 22 stays silent");

    /* Key strokes go through the same channel. */
    kbc_siemens_key(kbc, 0x41);
    check((kbc_siemens_read_status(kbc) & 0x01) == 0x01, "key makes data available");
    check(kbc_siemens_read_data(kbc) == 0x41, "key byte delivered");

    kbc_siemens_close(kbc);

    printf("%i/%i checks passed\n", checks - failures, checks);
    return failures != 0;
}

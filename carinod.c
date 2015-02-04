/**
 * @file carinod.c
 * @brief 
 *
 * @date 18 sept. 2014
 * @author carrier.nicolas0@gmail.com
 */
#include <sys/types.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <termios.h>

#include <error.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>

#include <io_mon.h>
#include <io_src.h>
#include <io_io.h>
#include <io_utils.h>

#include "ci_command.h"

static bool debug = false;

/* TODO put this in a calibration file */
const int rightMax = 140;
const int leftMax = 51;

#define MAX_MSG 10

#define PROG_NAME "carinod"

struct msg {
	struct io_io_write_buffer buffer;
	struct ci_command cmd;
	bool busy;
};

struct car_state {
	struct ci_command old;
	struct ci_command new;
};

struct main_ctx {
	struct io_mon mon;
	struct io_io io;
	struct io_src_tmr tmr;
	struct io_src kbd;
	int fd;
	bool loop;
	struct msg msg[MAX_MSG];
	struct car_state car_state;
};

static struct main_ctx ctx;
static struct termios old_tios;

static void usage(int ret_code)
{
	printf("usage: "PROG_NAME" ARDUINO_TTY\n");

	exit(ret_code);
}

static void clean(void)
{
	io_close(&ctx.fd);
}

static void log_rx(const char *buf)
{
	fprintf(stderr, "[RX] '%s'\n", buf);
}

static void log_tx(const char *buf)
{
	fprintf(stderr, "[TX] '%s'\n", buf);
}

static int handle_arduino_message(struct main_ctx *ctx, const char *msg)
{
	/*
	 * TODO replace by the actual data handling
	 */
	if (msg[strlen(msg) - 1] == '\r')
		fprintf(stderr, "%s", msg);
	else
		fprintf(stderr, "%s: %s\r\n", __func__, msg);

	return 0;
}

static int read_cb(struct io_io *io, struct rs_rb *rb, void *data)
{
	int ret;
	struct main_ctx *ctx = data;
	char *data_ready;
	char *needle;
	size_t to_consume = rs_rb_get_read_length(rb);
	ptrdiff_t line_length;

	while (to_consume > 0) {
		data_ready = rs_rb_get_read_ptr(rb);

		needle = memchr(data_ready, '\n', to_consume);
		if (needle == NULL)
			return 1;
		*needle = '\0';

		handle_arduino_message(ctx, data_ready);

		line_length = needle - data_ready;
		ret = rs_rb_read_incr(rb, line_length + 1);
		if (ret < 0)
			fprintf(stderr, "rs_rb_read_incr:%s\n", strerror(-ret));

		to_consume = rs_rb_get_read_length(rb);
	}

	return 0;
}

static int fill_msg(struct ci_command *dst, struct ci_command *src)
{
	if (dst == NULL || src == NULL)
		return -EINVAL;

	memcpy(dst, src, CI_COMMAND_LENGTH);

	return ci_command_complete(dst);
}

static void buffer_write_cb(struct io_io_write_buffer *buffer,
		enum io_io_write_status status)
{
	struct msg *msg = rs_container_of(buffer, struct msg, buffer);
	struct main_ctx *ctx = buffer->data;

	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	/* the message structure can be reused */
	msg->busy = false;
	msg->cmd.beep = false;

	switch (status) {
	case IO_IO_WRITE_ERROR:
		fprintf(stderr, "message error\n");
		break;

	case IO_IO_WRITE_TIMEOUT:
		fprintf(stderr, "message timeout\n");
		break;

	case IO_IO_WRITE_ABORTED:
		fprintf(stderr, "message aborted\n");
		break;

	default:
		ctx->car_state.old = msg->cmd;
		ctx->car_state.new = msg->cmd;

		break;
	}
}

static bool states_differ(struct car_state *car_state)
{
	return memcmp(&car_state->old, &car_state->new, CI_COMMAND_LENGTH) != 0;
}

static void queue_command(struct main_ctx *ctx)
{
	int ret;
	struct msg *msg = ctx->msg; // TODO replace by message peak

	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	if (states_differ(&ctx->car_state)) {
		ret = fill_msg(&msg->cmd, &ctx->car_state.new);
		ctx->car_state.old.beep = ctx->car_state.new.beep = false;
		if (ret < 0) {
			fprintf(stderr, "fill_msg: %s", strerror(-ret));
			return;
		}
		msg->busy = true;

		io_io_write_buffer_init(&msg->buffer, buffer_write_cb, ctx,
				CI_COMMAND_LENGTH, &msg->cmd);

		ret = io_io_write_add(&ctx->io, &msg->buffer);
		if (ret < 0)
			fprintf(stderr, "io_io_write_add: %s", strerror(-ret));
	}
}

static void incr_motor_speed(int16_t *speed, int incr)
{
	int16_t s = *speed;

	s += incr;
	if (s > 255)
		s = 255;
	if (s < -255)
		s = -255;

	*speed = s;
}

static void incr_motors_speed(struct main_ctx *ctx, int incr)
{
	incr_motor_speed(&ctx->car_state.new.left_motor_speed, incr);
	incr_motor_speed(&ctx->car_state.new.right_motor_speed, incr);

	queue_command(ctx);
}

static void increase_motors_speed(struct main_ctx *ctx)
{
	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	incr_motors_speed(ctx, 1);
}

static void decrease_motors_speed(struct main_ctx *ctx)
{
	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	incr_motors_speed(ctx, -1);
}

static void incr_angle(struct main_ctx *ctx, uint8_t *angle, int incr,
		uint8_t min, uint8_t max)
{
	uint8_t a = *angle;

	if ((incr < 0 && a <= min) || (incr > 0 && a >= max))
		return;

	a += incr;

	*angle = a;

	queue_command(ctx);
}

static void increase_servo_angle(struct main_ctx *ctx)
{
	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	incr_angle(ctx, &ctx->car_state.new.servo_angle, 1, leftMax, rightMax);
}

static void decrease_servo_angle(struct main_ctx *ctx)
{
	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	incr_angle(ctx, &ctx->car_state.new.servo_angle, -1, leftMax, rightMax);
}

static void increase_camera_angle(struct main_ctx *ctx, uint8_t *angle)
{
	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	incr_angle(ctx, angle, 1, 0, 180);
}

static void decrease_camera_angle(struct main_ctx *ctx, uint8_t *angle)
{
	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	incr_angle(ctx, angle, -1, 0, 180);
}

static void beep(struct main_ctx *ctx)
{
	if (debug)
		fprintf(stderr, "%s\r\n", __func__);

	ctx->car_state.new.beep = true;

	queue_command(ctx);
}

static void kbd_cb(struct io_src *src)
{
	struct main_ctx *ctx = rs_container_of(src, struct main_ctx, kbd);
	ssize_t sret;
	char key;

	sret = read(STDIN_FILENO, &key, 1);
	if (sret == -1) {
		perror("io_io_write_add");
		return;
	}

	if (debug)
		fprintf(stderr, "pressed key %c\n\r", key);
	switch (key) {
	case 'q':
		ctx->loop = false;
		break;

	case 'c':
		increase_motors_speed(ctx);
		break;

	case 't':
		decrease_motors_speed(ctx);
		break;

	case 'n':
		increase_servo_angle(ctx);
		break;

	case 'h':
		decrease_servo_angle(ctx);
		break;

	case '.':
		increase_camera_angle(ctx, &ctx->car_state.new.camera_x_angle);
		break;

	case 'e':
		decrease_camera_angle(ctx, &ctx->car_state.new.camera_x_angle);
		break;

	case 'u':
		increase_camera_angle(ctx, &ctx->car_state.new.camera_z_angle);
		break;

	case 'o':
		decrease_camera_angle(ctx, &ctx->car_state.new.camera_z_angle);
		break;

	case ' ':
		beep(ctx);
		break;

	default:
		/* just drop the other keys */
		break;
	}
}

static int config_tty_arduino(int fd)
{
	int ret;
	struct termios tios;

	memset(&tios, 0, sizeof(tios));
	tios.c_cflag = CS8;
	cfsetospeed(&tios, B115200);
	cfsetispeed(&tios, B115200);

	ret = tcsetattr(fd, TCSANOW, &tios);
	if (-1 == ret)
		error(EXIT_FAILURE, errno, "tcsetattr");

	return 0;
}

void restore_stdin(void)
{
	char key;

	/* flush non processed characters */
	while (1 == read(STDIN_FILENO, &key, 1));

	/* restore terminal */
	tcsetattr(STDIN_FILENO, TCSANOW, &old_tios);
}

static void config_stdin(void)
{
	int ret;
	struct termios new_tios;

	ret = tcgetattr(0, &old_tios);
	if (-1 == ret)
		error(EXIT_FAILURE, errno, "tcgetattr");

	cfmakeraw(&new_tios);
	ret = tcsetattr(0, TCSANOW, &new_tios);
	atexit(restore_stdin);
	if (-1 == ret)
		error(EXIT_FAILURE, errno, "tcsetattr");
}

int main(int argc, char *argv[])
{
	int ret;
	const char *tty_arduino;

	if (argc != 2)
		usage(EXIT_FAILURE);
	if (strcmp("-h", argv[1]) == 0)
		usage(EXIT_SUCCESS);

	tty_arduino = argv[1];
	ctx.fd = open(tty_arduino, O_RDWR | O_CLOEXEC | O_NONBLOCK);
	if (ctx.fd < 0)
		error(EXIT_FAILURE, errno, "can't open %s", argv[1]);
	atexit(clean);

	config_tty_arduino(ctx.fd);

	/* set servo to their default positions */
	ctx.car_state.new.servo_angle = ctx.car_state.old.servo_angle =
			(rightMax + leftMax) / 2;
	ctx.car_state.new.camera_x_angle = ctx.car_state.old.camera_z_angle =
			90;
	ctx.car_state.new.camera_z_angle = ctx.car_state.old.camera_z_angle =
			90;

	ret = io_mon_init(&ctx.mon);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "io_mon_init");

	ret = io_io_init(&ctx.io, &ctx.mon, "arduino", ctx.fd, ctx.fd, true);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "io_io_init");

	ret = io_src_init(&ctx.kbd, STDIN_FILENO, IO_IN, kbd_cb);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "io_src_init");

	ret = io_mon_add_source(&ctx.mon, &ctx.kbd);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "io_mon_add_source");

	if (getenv("CARINO_LOG_TX") != NULL)
		io_io_log_tx(&ctx.io, log_tx);
	else
		io_io_log_tx(&ctx.io, NULL);
	if (getenv("CARINO_LOG_TX") != NULL)
		io_io_log_rx(&ctx.io, log_rx);
	else
		io_io_log_rx(&ctx.io, NULL);

	ret = io_io_read_start(&ctx.io, read_cb, &ctx, true);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "io_io_read_start");

	config_stdin();
	printf(PROG_NAME" (%jd) listening\r\n", (intmax_t)getpid());

	ctx.loop = true;
	while (ctx.loop) {
		ret = io_mon_poll(&ctx.mon, -1);
		if (ret < 0)
			error(EXIT_FAILURE, -ret, "io_mon_init");
	}

	/* cleanup all the mess */
	io_io_read_stop(&ctx.io);
	io_mon_remove_source(&ctx.mon, &ctx.kbd);
	io_io_clean(&ctx.io);
	io_close(&ctx.fd);
	io_mon_clean(&ctx.mon);

	printf(PROG_NAME" says bye bye\r\n");

	return EXIT_SUCCESS;
}


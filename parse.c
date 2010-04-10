/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2009, 2010 Daniel Beer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <assert.h>

#ifdef USE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif

#include "util.h"
#include "parse.h"
#include "stab.h"

static struct option *option_list;
static struct command *command_list;

void register_option(struct option *o)
{
	o->next = option_list;
	option_list = o;
}

static struct option *find_option(const char *name)
{
	struct option *o;

	for (o = option_list; o; o = o->next)
		if (!strcasecmp(o->name, name))
			return o;

	return NULL;
}

static struct command *find_command(const char *name)
{
	struct command *c;

	for (c = command_list; c; c = c->next)
		if (!strcasecmp(c->name, name))
			return c;

	return NULL;
}

void register_command(struct command *c)
{
	c->next = command_list;
	command_list = c;
}

static int interactive_call = 1;

int is_interactive(void)
{
	return interactive_call;
}

char *get_arg(char **text)
{
	char *start;
	char *rewrite;
	char *end;
	int qstate = 0;
	int qval = 0;

	if (!text)
		return NULL;

	start = *text;
	while (*start && isspace(*start))
		start++;

	if (!*start)
		return NULL;

	/* We've found the start of the argument. Parse it. */
	end = start;
	rewrite = start;
	while (*end) {
		switch (qstate) {
		case 0: /* Bare */
			if (isspace(*end))
				goto out;
			else if (*end == '"')
				qstate = 1;
			else
				*(rewrite++) = *end;
			break;

		case 1: /* In quotes */
			if (*end == '"')
				qstate = 0;
			else if (*end == '\\')
				qstate = 2;
			else
				*(rewrite++) = *end;
			break;

		case 2: /* Backslash */
			if (*end == '\\')
				*(rewrite++) = '\\';
			else if (*end == 'n')
				*(rewrite++) = '\n';
			else if (*end == 'r')
				*(rewrite++) = '\r';
			else if (*end == 't')
				*(rewrite++) = '\t';
			else if (*end >= '0' && *end <= '3') {
				qstate = 30;
				qval = *end - '0';
			} else if (*end == 'x') {
				qstate = 40;
				qval = 0;
			} else
				*(rewrite++) = *end;

			if (qstate == 2)
				qstate = 1;
			break;

		case 30: /* Octal */
		case 31:
			if (*end >= '0' && *end <= '7')
				qval = (qval << 3) | (*end - '0');

			if (qstate == 31) {
				*(rewrite++) = qval;
				qstate = 1;
			} else {
				qstate++;
			}
			break;

		case 40: /* Hex */
		case 41:
			if (isdigit(*end))
				qval = (qval << 4) | (*end - '0');
			else if (isupper(*end))
				qval = (qval << 4) | (*end - 'A' + 10);
			else if (islower(*end))
				qval = (qval << 4) | (*end - 'a' + 10);

			if (qstate == 41) {
				*(rewrite++) = qval;
				qstate = 1;
			} else {
				qstate++;
			}
			break;
		}

		end++;
	}
 out:
	/* Leave the text pointer at the end of the next argument */
	while (*end && isspace(*end))
		end++;

	*rewrite = 0;
	*text = end;
	return start;
}

int process_command(char *arg, int interactive)
{
	const char *cmd_text;
	int len = strlen(arg);

	while (len && isspace(arg[len - 1]))
		len--;
	arg[len] = 0;

	cmd_text = get_arg(&arg);
	if (cmd_text) {
		const struct command *cmd = find_command(cmd_text);

		if (cmd) {
			int old = interactive_call;
			int ret;

			interactive_call = interactive;
			ret = cmd->func(&arg);
			interactive_call = old;
			return 0;
		}

		fprintf(stderr, "unknown command: %s (try \"help\")\n",
			cmd_text);
		return -1;
	}

	return 0;
}

const char *type_text(option_type_t type)
{
	switch (type) {
	case OPTION_BOOLEAN:
		return "boolean";

	case OPTION_NUMERIC:
		return "numeric";

	case OPTION_TEXT:
		return "text";
	}

	return "unknown";
}

static const char *name_list[128];
static int num_names;
static int name_max_len;

static void name_start(void)
{
	num_names = 0;
	name_max_len = 0;
}

static void name_push(const char *text)
{
	if (num_names < ARRAY_LEN(name_list)) {
		int len = strlen(text);

		name_list[num_names++] = text;
		if (len > name_max_len)
			name_max_len = len;
	}
}

static int compare_name(const void *left, const void *right)
{
	return strcasecmp(*(const char *const *)left,
			  *(const char *const *)right);
}

static void name_list_show(void)
{
	int i;
	int max_len = name_max_len + 2;
	int rows, cols;

	qsort(name_list, num_names, sizeof(name_list[0]),
	      compare_name);

	cols = 72 / max_len;
	rows = (num_names + cols - 1) / cols;

	for (i = 0; i < rows; i++) {
		int j;

		printf("    ");
		for (j = 0; j < cols; j++) {
			int k = j * rows + i;

			if (k >= num_names)
				break;

			printf("%s", name_list[k]);
			for (k = strlen(name_list[k]); k < max_len; k++)
				printf(" ");
		}

		printf("\n");
	}
}

static int cmd_help(char **arg)
{
	const char *topic = get_arg(arg);

	if (topic) {
		const struct command *cmd = find_command(topic);
		const struct option *opt = find_option(topic);

		if (cmd) {
			printf("COMMAND: %s\n", cmd->name);
			fputs(cmd->help, stdout);
			if (opt)
				printf("\n");
		}

		if (opt) {
			printf("OPTION: %s (%s)\n", opt->name,
			       type_text(opt->type));
			fputs(opt->help, stdout);
		}

		if (!(cmd || opt)) {
			fprintf(stderr, "help: unknown command: %s\n", topic);
			return -1;
		}
	} else {
		const struct command *cmd;
		const struct option *opt;

		name_start();
		for (cmd = command_list; cmd; cmd = cmd->next)
			name_push(cmd->name);

		printf("Available commands:\n");
		name_list_show();
		printf("\n");

		name_start();
		for (opt = option_list; opt; opt = opt->next)
			name_push(opt->name);

		printf("Available options:\n");
		name_list_show();
		printf("\n");

		printf("Type \"help <topic>\" for more information.\n");
		printf("Press Ctrl+D to quit.\n");
	}

	return 0;
}

static struct command command_help = {
	.func = cmd_help,
	.name = "help",
	.help =
	"help [command]\n"
	"    Without arguments, displays a list of commands. With a command\n"
	"    name as an argument, displays help for that command.\n"
};

#ifndef USE_READLINE
#define LINE_BUF_SIZE 128

static char *readline(const char *prompt)
{
	char *buf = malloc(LINE_BUF_SIZE);

	if (!buf) {
		perror("readline: can't allocate memory");
		return NULL;
	}

	for (;;) {
		printf("(mspdebug) ");
		fflush(stdout);

		if (fgets(buf, LINE_BUF_SIZE, stdin))
			return buf;

		if (feof(stdin))
			break;

		printf("\n");
	}

	free(buf);
	return NULL;
}

#define add_history(x)
#endif

void reader_loop(void)
{
	printf("\n");
	cmd_help(NULL);
	printf("\n");

	do {
		for (;;) {
			char *buf = readline("(mspdebug) ");

			if (!buf)
				break;

			add_history(buf);
			process_command(buf, 1);
			free(buf);
		}
	} while (modify_prompt(MODIFY_ALL));

	printf("\n");
}

static void display_option(const struct option *o)
{
	printf("%32s = ", o->name);

	switch (o->type) {
	case OPTION_BOOLEAN:
		printf("%s", o->data.numeric ? "true" : "false");
		break;

	case OPTION_NUMERIC:
		printf("0x%x (%d)", o->data.numeric,
		       o->data.numeric);
		break;

	case OPTION_TEXT:
		printf("%s", o->data.text);
		break;
	}

	printf("\n");
}

static int parse_option(struct option *o, const char *word)
{
	switch (o->type) {
	case OPTION_BOOLEAN:
		o->data.numeric = (isdigit(word[0]) && word[0] > '0') ||
			word[0] == 't' || word[0] == 'y' ||
			(word[0] == 'o' && word[1] == 'n');
		break;

	case OPTION_NUMERIC:
		return addr_exp(word, &o->data.numeric);

	case OPTION_TEXT:
		strncpy(o->data.text, word, sizeof(o->data.text));
		o->data.text[sizeof(o->data.text) - 1] = 0;
		break;
	}

	return 0;
}

static int cmd_opt(char **arg)
{
	const char *opt_text = get_arg(arg);
	struct option *opt = NULL;

	if (opt_text) {
		opt = find_option(opt_text);
		if (!opt) {
			fprintf(stderr, "opt: no such option: %s\n",
				opt_text);
			return -1;
		}
	}

	if (**arg) {
		if (parse_option(opt, *arg) < 0) {
			fprintf(stderr, "opt: can't parse option: %s\n",
				*arg);
			return -1;
		}
	} else if (opt_text) {
		display_option(opt);
	} else {
		struct option *o;

		for (o = option_list; o; o = o->next)
			display_option(o);
	}

	return 0;
}

static struct command command_opt = {
	.name = "opt",
	.func = cmd_opt,
	.help =
	"opt [name] [value]\n"
	"    Query or set option variables. With no arguments, displays all\n"
	"    available options.\n"
};

int process_file(const char *filename)
{
	FILE *in;
	char buf[1024];
	int line_no = 0;

	in = fopen(filename, "r");
	if (!in) {
		fprintf(stderr, "read: can't open %s: %s\n",
			filename, strerror(errno));
		return -1;
	}

	while (fgets(buf, sizeof(buf), in)) {
		char *cmd = buf;

		line_no++;

		while (*cmd && isspace(*cmd))
			cmd++;

		if (*cmd == '#')
			continue;

		if (process_command(cmd, 0) < 0) {
			fprintf(stderr, "read: error processing %s (line %d)\n",
				filename, line_no);
			fclose(in);
			return -1;
		}
	}

	fclose(in);
	return 0;
}

static int cmd_read(char **arg)
{
	char *filename = get_arg(arg);

	if (!filename) {
		fprintf(stderr, "read: filename must be specified\n");
		return -1;
	}

	return process_file(filename);
}

static struct command command_read = {
	.name = "read",
	.func = cmd_read,
	.help =
	"read <filename>\n"
	"    Read commands from a file and evaluate them.\n"
};

static struct option option_color = {
	.name = "color",
	.type = OPTION_BOOLEAN,
	.help = "Colorize disassembly output.\n"
};

int colorize(const char *text)
{
	if (!option_color.data.numeric)
		return 0;

	return printf("\x1b[%s", text);
}

void parse_init(void)
{
	register_option(&option_color);

	register_command(&command_help);
	register_command(&command_opt);
	register_command(&command_read);
}

struct addr_exp_state {
	int     last_operator;
	int     data_stack[32];
	int     data_stack_size;
	int     op_stack[32];
	int     op_stack_size;
};

static int addr_exp_data(struct addr_exp_state *s, const char *text)
{
	int value;

	if (!s->last_operator || s->last_operator == ')') {
		fprintf(stderr, "syntax error at token %s\n", text);
		return -1;
	}

	/* Hex value */
	if (*text == '0' && text[1] == 'x')
		value = strtoul(text + 2, NULL, 16);
	else if (isdigit(*text))
		value = atoi(text);
	else if (stab_get(text, &value) < 0) {
		fprintf(stderr, "can't parse token: %s\n", text);
		return -1;
	}

	if (s->data_stack_size + 1 > ARRAY_LEN(s->data_stack)) {
		fprintf(stderr, "data stack overflow at token %s\n", text);
		return -1;
	}

	s->data_stack[s->data_stack_size++] = value;
	s->last_operator = 0;
	return 0;
}

static int addr_exp_pop(struct addr_exp_state *s)
{
	char op = s->op_stack[--s->op_stack_size];
	int data1 = s->data_stack[--s->data_stack_size];
	int data2 = 0;

	int result = 0;

	if (op != 'N')
		data2 = s->data_stack[--s->data_stack_size];

	assert (s->op_stack_size >= 0);
	assert (s->data_stack_size >= 0);

	switch (op) {
	case '+':
		result = data2 + data1;
		break;

	case '-':
		result = data2 - data1;
		break;

	case '*':
		result = data2 * data1;
		break;

	case '/':
		if (!data1)
			goto divzero;
		result = data2 / data1;
		break;

	case '%':
		if (!data1)
			goto divzero;
		result = data2 % data1;
		break;

	case 'N':
		result = -data1;
		break;
	}

	s->data_stack[s->data_stack_size++] = result;
	return 0;

 divzero:
	fprintf(stderr, "divide by zero\n");
	return -1;
}

static int can_push(struct addr_exp_state *s, char op)
{
	char top;

	if (!s->op_stack_size || op == '(')
		return 1;

	top = s->op_stack[s->op_stack_size - 1];

	if (top == '(')
		return 1;

	switch (op) {
	case 'N':
		return 1;

	case '*':
	case '%':
	case '/':
		return top == '+' || top == '-';

	default:
		break;
	}

	return 0;
}

static int addr_exp_op(struct addr_exp_state *s, char op)
{
	if (op == '(') {
		if (!s->last_operator || s->last_operator == ')')
			goto syntax_error;
	} else if (op == '-') {
		if (s->last_operator && s->last_operator != ')')
			op = 'N';
	} else {
		if (s->last_operator && s->last_operator != ')')
			goto syntax_error;
	}

	if (op == ')') {
		/* ) collapses the stack to the last matching ( */
		while (s->op_stack_size &&
		       s->op_stack[s->op_stack_size - 1] != '(')
			if (addr_exp_pop(s) < 0)
				return -1;

		if (!s->op_stack_size) {
			fprintf(stderr, "parenthesis mismatch: )\n");
			return -1;
		}

		s->op_stack_size--;
	} else {
		while (!can_push(s, op))
			if (addr_exp_pop(s) < 0)
				return -1;

		if (s->op_stack_size + 1 > ARRAY_LEN(s->op_stack)) {
			fprintf(stderr, "operator stack overflow: %c\n", op);
			return -1;
		}

		s->op_stack[s->op_stack_size++] = op;
	}

	s->last_operator = op;
	return 0;

 syntax_error:
	fprintf(stderr, "syntax error at operator %c\n", op);
	return -1;
}

static int addr_exp_finish(struct addr_exp_state *s, int *ret)
{
	if (s->last_operator && s->last_operator != ')') {
		fprintf(stderr, "syntax error at end of expression\n");
		return -1;
	}

	while (s->op_stack_size) {
		if (s->op_stack[s->op_stack_size - 1] == '(') {
			fprintf(stderr, "parenthesis mismatch: (\n");
			return -1;
		}

		if (addr_exp_pop(s) < 0)
			return -1;
	}

	if (s->data_stack_size != 1) {
		fprintf(stderr, "no data: stack size is %d\n",
			s->data_stack_size);
		return -1;
	}

	if (ret)
		*ret = s->data_stack[0];

	return 0;
}

int addr_exp(const char *text, int *addr)
{
	const char *text_save = text;
	int last_cc = 1;
	char token_buf[64];
	int token_len = 0;
	struct addr_exp_state s = {0};

	s.last_operator = '(';

	for (;;) {
		int cc;

		/* Figure out what class this character is */
		if (*text == '+' || *text == '-' ||
		    *text == '*' || *text == '/' ||
		    *text == '%' || *text == '(' ||
		    *text == ')')
			cc = 1;
		else if (!*text || isspace(*text))
			cc = 2;
		else if (isalnum(*text) || *text == '.' || *text == '_' ||
			 *text == '$' || *text == ':')
			cc = 3;
		else {
			fprintf(stderr, "illegal character in expression: %c\n",
				*text);
			return -1;
		}

		/* Accumulate and process token text */
		if (cc == 3) {
			if (token_len + 1 < sizeof(token_buf))
				token_buf[token_len++] = *text;
		} else if (token_len) {
			token_buf[token_len] = 0;
			token_len = 0;

			if (addr_exp_data(&s, token_buf) < 0)
				goto fail;
		}

		/* Process operators */
		if (cc == 1) {
			if (addr_exp_op(&s, *text) < 0)
				goto fail;
		}

		if (!*text)
			break;

		last_cc = cc;
		text++;
	}

	if (addr_exp_finish(&s, addr) < 0)
		goto fail;

	return 0;

 fail:
	fprintf(stderr, "bad address expression: %s\n", text_save);
	return -1;
}

static int modify_flags;

void modify_set(int flags)
{
	modify_flags |= flags;
}

void modify_clear(int flags)
{
	modify_flags &= ~flags;
}

int modify_prompt(int flags)
{
	char buf[32];

	if (!(interactive_call && (modify_flags & flags)))
		return 0;

	for (;;) {
		printf("Symbols have not been saved since modification. "
		       "Continue (y/n)? ");
		fflush(stdout);

		if (!fgets(buf, sizeof(buf), stdin)) {
			printf("\n");
			return 1;
		}

		if (toupper(buf[0]) == 'Y')
			return 0;
		if (toupper(buf[0]) == 'N')
			return 1;

		printf("Please answer \"y\" or \"n\".\n");
	}

	return 0;
}
#ifndef __GENERATOR_H__
#define __GENERATOR_H__

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <dirent.h>
#include <libgen.h>

#ifndef __WIN32__
#include <argp.h>
#include <stdbool.h>
#endif

#include <sys/types.h>

#define BUFSIZE	256

#if ENABLE_NLS
# include <libintl.h>
# define _(Text) gettext (Text)
#else
# define textdomain(Domain)
# define _(Text) Text
#endif
#define N_(Text) Text

#ifndef __WIN32__
/* argp option keys */
enum {
	DUMMY_KEY = 129,
	DIRECTORY_KEY
};
#endif

typedef enum {
	INVALID_PARAMTYPE = 0,
	NUMERIC,
	NUMERIC_CONST,
	STRING,
	BOOL,
	CHAR,
	ANY
} paramType;

typedef enum {
	INVALID_MODULETYPE = 0,
	APPLICATION,
	NODE,
	NETWORK,
	MAC,
	PHY,
	BATTERY,
	MOBILITY,
	ARP,
	UTILITY
} moduleType;

typedef struct _parameter {
	char* name;
	char* comment;
	paramType type;
	struct _parameter *next;
} Parameter;

typedef struct _module{
	char* name;
	moduleType type;
	Parameter* parameters;
	char** depends; // comma separated lists
	struct _module *next;
} Module;

#ifdef __cplusplus
extern "C" {
#endif

void init(const char* configPath, const char* sourcePath);
void finish(void);

void addConfig(char *option);
void addConfigInt(char *option, int value);
void addConfigString(char *option, const char *value);
void stripNewline(char *line);
bool checkTimeValue(char *line);
void writeConfig(char *option, const char *value, bool quoted);
bool empty(char *line);
bool checkNumber(char *line);
bool checkString(char *line);
void writeNode(unsigned nodeID, double x, double y, double z);
void startParameterSection(void);
int nedCheck(const struct dirent* entry);
char* stripComments(char* line);
char* getComments(char* line);
char* getParamName(char* line);
paramType getParamType(char* line);
Module* findBaseModules(void);
Module* findModules(moduleType type);

#ifdef __cplusplus
}
#endif

#endif


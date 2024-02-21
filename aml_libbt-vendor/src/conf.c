/******************************************************************************
*
*  Copyright (C) 2019-2021 Amlogic Corporation
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at:
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
******************************************************************************/

/******************************************************************************
*
*  Filename:      conf.c
*
*  Description:   Contains functions to conduct run-time module configuration
*                 based on entries present in the .conf file
*
******************************************************************************/

#define LOG_TAG "bt_vnd_conf"

#include <stdio.h>
#include <unistd.h>
#include <utils/Log.h>
#include <string.h>
#include "bt_vendor_aml.h"

/******************************************************************************
**  Externs
******************************************************************************/
int userial_set_port(char *p_conf_name, char *p_conf_value, int param);
int hw_set_patch_file_path(char *p_conf_name, char *p_conf_value, int param);
int hw_set_patch_file_name(char *p_conf_name, char *p_conf_value, int param);

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
int hw_set_patch_settlement_delay(char *p_conf_name, char *p_conf_value, int param);
#endif


/******************************************************************************
**  Local type definitions
******************************************************************************/

#define CONF_COMMENT '#'
#define CONF_DELIMITERS " =\n\r\t"
#define CONF_VALUES_DELIMITERS "=\n\r\t"
#define CONF_MAX_LINE_LEN 255

typedef int (conf_action_t)(char *p_conf_name, char *p_conf_value, int param);

typedef struct {
	const char *	conf_entry;
	conf_action_t * p_action;
	int		param;
} conf_entry_t;

unsigned int amlbt_poweron =AML_SDIO_EN;
unsigned int amlbt_chiptype =AML_W1U;
unsigned int amlbt_btrecovery = 0;
unsigned int amlbt_btsink = 0;
unsigned int amlbt_rftype = 0;
unsigned int amlbt_fw_mode = 0;

/******************************************************************************
**  Static variables
******************************************************************************/

/*
 * Current supported entries and corresponding action functions
 */
static const conf_entry_t conf_table[] = {
	{ "UartPort",		    userial_set_port,		   0 },
	{ "FwPatchFilePath",	    hw_set_patch_file_path,	   0 },
	{ "FwPatchFileName",	    hw_set_patch_file_name,	   0 },
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
	{ "FwPatchSettlementDelay", hw_set_patch_settlement_delay, 0 },
#endif
	{ (const char *)NULL,	    NULL,			   0 }
};


/*****************************************************************************
**   CONF INTERFACE FUNCTIONS
*****************************************************************************/

/*******************************************************************************
**
** Function        vnd_load_conf
**
** Description     Read conf entry from p_path file one by one and call
**                 the corresponding config function
**
** Returns         None
**
*******************************************************************************/
void vnd_load_conf(const char *p_path)
{
	FILE *p_file;
	char *p_name;
	char *p_value;
	conf_entry_t *p_entry;
	char line[CONF_MAX_LINE_LEN + 1]; /* add 1 for \0 char */

	ALOGI("Attempt to load conf from %s", p_path);

	if ((p_file = fopen(p_path, "r")) != NULL)
	{
		/* read line by line */
		while (fgets(line, CONF_MAX_LINE_LEN + 1, p_file) != NULL)
		{
			if (line[0] == CONF_COMMENT)
				continue;

			p_name = strtok(line, CONF_DELIMITERS);

			if (NULL == p_name)
			{
				continue;
			}

			p_value = strtok(NULL, CONF_DELIMITERS);

			if (NULL == p_value)
			{
				ALOGW("vnd_load_conf: missing value for name: %s", p_name);
				continue;
			}

			p_entry = (conf_entry_t *)conf_table;

			while (p_entry->conf_entry != NULL)
			{
				if (strcmp(p_entry->conf_entry, (const char *)p_name) == 0)
				{
					p_entry->p_action(p_name, p_value, p_entry->param);
					break;
				}

				p_entry++;
			}
		}

		fclose(p_file);
	}
	else
	{
		ALOGI("vnd_load_conf file >%s< not found", p_path);
	}
}

static char *aml_trim(char *str) {
    while (isspace(*str))
        ++str;

    if (!*str)
        return str;

    char *end_str = str + strlen(str) - 1;
    while (end_str > str && isspace(*end_str))
        --end_str;

    end_str[1] = '\0';
    return str;
}


void load_aml_stack_conf()
{
    char *split;
    FILE *fp = fopen(AML_VENDOR_LIB_CONF_FILE, "rt");
    if (!fp) {
      ALOGE("%s unable to open file '%s'", __func__,
        AML_VENDOR_LIB_CONF_FILE);
      return;
    }
    ALOGE("%s success to open file '%s'", __func__,
      AML_VENDOR_LIB_CONF_FILE);
    int line_num = 0;
    char line[1024];
    //char value[1024];
    while (fgets(line, sizeof(line), fp)) {
        char *line_ptr = aml_trim(line);
		char line_f[100];
        ++line_num;

        // Skip blank and comment lines.
        if (*line_ptr == '\0' || *line_ptr == '#' || *line_ptr == '[')
          continue;

        split = strchr(line_ptr, '=');
        if (!split) {
            ALOGE("%s no key/value separator found on line %d.", __func__, line_num);
            continue;
        }
		strncpy(line_f,line_ptr,strlen(line_ptr)-strlen(split));
       // *split = '\0';
       ALOGE("%s  %s  %s", __func__, aml_trim(line_f), aml_trim(split+1));
        char *endptr;
        if (!strcmp(aml_trim(line_f), "BtPowerOn")) {
            amlbt_poweron = strtol(aml_trim(split+1), &endptr, 0);
			ALOGE("%s amlbt_poweron '%d'", __func__, amlbt_poweron);
        }
        else if(!strcmp(aml_trim(line_f), "BtChip")) {
            amlbt_chiptype = strtol(aml_trim(split+1), &endptr, 0);
			ALOGE("%s amlbt_chiptype '%d'", __func__, amlbt_chiptype);
        }
        else if (!strcmp(aml_trim(line_f), "BtRecovery")) {
            amlbt_btrecovery = strtol(aml_trim(split+1), &endptr, 0);
            ALOGE("%s amlbt_btrecovery '%d'", __func__, amlbt_btrecovery);
        }
        else if (!strcmp(aml_trim(line_f), "BtSink")) {
            amlbt_btsink = strtol(aml_trim(split+1), &endptr, 0);
            ALOGE("%s amlbt_btsink '%d'", __func__, amlbt_btsink);
        }
        else if (!strcmp(aml_trim(line_f), "BtAntenna")) {
            amlbt_rftype = strtol(aml_trim(split+1), &endptr, 0);
            ALOGE("%s amlbt_rftype '%d'", __func__, amlbt_rftype);
        }
        else if (!strcmp(aml_trim(line_f), "FirmwareMode")) {
            amlbt_fw_mode = strtol(aml_trim(split+1), &endptr, 0);
            ALOGE("%s amlbt_fw_mode '%d'", __func__, amlbt_fw_mode);
        }
    }
    fclose(fp);
}


/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cmd_debug.h"
#include "cmd_util.h"
#include "driver/chip/hal_codec.h"

void HAL_I2S_REG_DEBUG()
{
        int i = 0;
        for (i = 0; i < 0x58; i = i+4)
                printf("REG:0X%x,VAL:0X%x\n",i,(*((__IO uint32_t *)(0x40042c00+i))));
}

void HAL_CODEC_REG_DEBUG()
{
	int32_t snd_soc_read(uint32_t reg);

        int i = 0;
        for (i = 0; i < 0x99; i = i+1) {
            printf("REG: 0X%x, VAL: 0X%x ; ",i,snd_soc_read(i));
			if (i%4 == 3)
					printf("\n");
		}
}


static enum cmd_status cmd_auddbg_i2s_reg_exec(char *cmd)
{
	void HAL_I2S_REG_DEBUG();
	HAL_I2S_REG_DEBUG();
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_auddbg_codec_reg_exec(char *cmd)
{
	void HAL_CODEC_REG_DEBUG();
	HAL_CODEC_REG_DEBUG();
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_auddbg_pa_exec(char *cmd)
{
	uint32_t on;
	cmd_sscanf(cmd, "%d", &on);

	if (on != 0 && on != 1)
	{
		CMD_ERR("pa enable only 0 or 1");
		return CMD_STATUS_INVALID_ARG;
	}

	HAL_Status HAL_CODEC_Trigger(AUDIO_Device dev, uint32_t on);
	HAL_CODEC_Trigger(AUDIO_DEVICE_SPEAKER, on);

	return CMD_STATUS_OK;

}

static enum cmd_status cmd_auddbg_codec_read_exec(char *cmd)
{
	int32_t cnt;
	uint32_t creg;
	cnt = cmd_sscanf(cmd, "0x%x", &creg);
	if (cnt != 1)
	{
		CMD_ERR("example: 0x10\n");
		return CMD_STATUS_INVALID_ARG;
	}

	int32_t snd_soc_read(uint32_t reg);
	printf("CODEC REG:0X%x, VAL:0X%x ;\n", creg, snd_soc_read(creg));
	return CMD_STATUS_OK;

}

static enum cmd_status cmd_auddbg_codec_write_exec(char *cmd)
{
	int32_t cnt;
	uint32_t creg;
	uint32_t val;
	cnt = cmd_sscanf(cmd, "0x%x 0x%x", &creg, &val);
	if (cnt != 2)
	{
		CMD_ERR("example: 0x10 0x10\n");
		return CMD_STATUS_INVALID_ARG;
	}

	int32_t snd_soc_write(uint32_t reg, uint32_t reg_val);
	int32_t snd_soc_read(uint32_t reg);
	snd_soc_write(creg, val);
	printf("CODEC REG:0X%x, VAL:0X%x ;\n", creg, snd_soc_read(creg));
	return CMD_STATUS_OK;
}

static struct cmd_data g_auddbg_cmds[] = {
	{ "i2s",     cmd_auddbg_i2s_reg_exec },
	{ "codec",    cmd_auddbg_codec_reg_exec },
	{ "pa",     cmd_auddbg_pa_exec },
	{ "read",    cmd_auddbg_codec_read_exec },
	{ "write",    cmd_auddbg_codec_write_exec },
};

enum cmd_status cmd_auddbg_exec(char *cmd)
{
	return cmd_exec(cmd, g_auddbg_cmds, cmd_nitems(g_auddbg_cmds));
}


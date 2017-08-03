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

#include "efpg_i.h"
#include "efpg_debug.h"
#include "driver/chip/hal_efuse.h"

static int efpg_efuse_read(uint8_t *data, uint32_t start_bit, uint32_t bit_num)
{
	uint8_t	   *p_data = data;
	uint32_t	bit_shift = start_bit & (32 - 1);
	uint32_t	word_idx = start_bit >> 5;

	uint64_t	buf = 0;
	uint32_t   *efuse_word = (uint32_t *)&buf;
	uint32_t	byte_num = (bit_num + 7) >> 3;
	uint32_t	byte_cnt = byte_num;
	uint32_t	bit_cnt;
	uint32_t	copy_size;

	efpg_memset(data, 0, byte_num);

	while (byte_cnt > 0) {
		if (HAL_EFUSE_Read((uint8_t)word_idx, &efuse_word[0]) != HAL_OK) {
			EFPG_ERR("efpg efuse read failed\n");
			return -1;
		}
		if (word_idx + 1 < 64) {
			if (HAL_EFUSE_Read((uint8_t)word_idx + 1, &efuse_word[1]) != HAL_OK) {
				EFPG_ERR("efpg efuse read failed\n");
				return -1;
			}
		} else {
			efuse_word[1] = 0;
		}
		buf = buf >> bit_shift;

		copy_size = (byte_cnt > sizeof(efuse_word[0])) ? sizeof(efuse_word[0]) : byte_cnt;
		efpg_memcpy(p_data, &efuse_word[0], copy_size);
		byte_cnt -= copy_size;
		p_data += copy_size;
		word_idx++;
	}

	bit_cnt = bit_num & (8 - 1);
	if (bit_cnt > 0)
		data[byte_num - 1] &= ((1 << bit_cnt) - 1);

	return 0;
}

static int efpg_efuse_write(uint8_t *data, uint32_t start_bit, uint32_t bit_num)
{
	uint8_t	   *p_data = data;
	uint32_t	bit_shift = start_bit & (32 - 1);
	uint32_t	word_idx = start_bit >> 5;

	uint64_t	buf = 0;
	uint32_t   *efuse_word = (uint32_t *)&buf;
	uint32_t	bit_cnt = bit_num;

	efpg_memcpy(&efuse_word[1], p_data, sizeof(efuse_word[1]));
	if (bit_cnt < 32)
		efuse_word[1] &= (1 << bit_cnt) - 1;
	efuse_word[1] = efuse_word[1] << bit_shift;

	if (HAL_EFUSE_Program((uint8_t)word_idx, efuse_word[1]) != HAL_OK) {
		EFPG_ERR("efpg efuse write failed\n");
		return -1;
	}

	word_idx++;
	bit_cnt -= (bit_cnt <= 32 - bit_shift) ? bit_cnt : 32 - bit_shift;

	while (bit_cnt > 0) {
		efpg_memcpy(&buf, p_data, sizeof(buf));
		buf = buf << bit_shift;
		if (bit_cnt < 32)
		efuse_word[1] &= (1 << bit_cnt) - 1;

		if (HAL_EFUSE_Program((uint8_t)word_idx, efuse_word[1]) != HAL_OK) {
			EFPG_ERR("efpg efuse write failed\n");
			return -1;
		}

		word_idx++;
		p_data += 4;
		bit_cnt -= (bit_cnt <= 32) ? bit_cnt : 32;
	}

	return 0;
}

static int efpg_boot_hash_cmp(const uint8_t *data, const uint8_t *buf, uint8_t *err_cnt,
							  uint8_t *err_1st_no, uint8_t *err_2nd_no)
{
	uint8_t byte_cnt;
	uint8_t bit_cnt;

	if (efpg_memcmp(data, buf, EFPG_BOOT_BUF_LEN) == 0)
		return 0;

	*err_cnt = 0;
	for (byte_cnt = 0; byte_cnt < EFPG_BOOT_BUF_LEN; byte_cnt++) {
		if (data[byte_cnt] == buf[byte_cnt])
			continue;

		for (bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
			if ((data[byte_cnt] & (0x1 << bit_cnt)) == (buf[byte_cnt] & (0x1 << bit_cnt)))
				continue;

			if (*err_cnt == 0) {
				*err_1st_no = (byte_cnt << 3) + bit_cnt;
				*err_cnt = 1;
			} else if (*err_cnt == 1) {
				*err_2nd_no = (byte_cnt << 3) + bit_cnt;
				*err_cnt = 2;
			} else {
				return -1;
			}
		}
	}

	return 0;
}

static uint16_t efpg_efuse_read_hosc(uint8_t *data)
{
	if (efpg_efuse_read(data, EFPG_HOSC_TYPE_START, EFPG_HOSC_TYPE_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	return EFPG_ACK_OK;
}

static uint16_t efpg_efuse_read_boot(uint8_t *data)
{
	uint8_t tmp = 0;
	uint8_t byte_cnt;
	uint8_t bit_cnt;

	/* flag */
	if (efpg_efuse_read(&tmp, EFPG_BOOT_FLAG_START, EFPG_BOOT_FLAG_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	if (tmp == 0) {
		EFPG_WARN("efpg efuse read boot flag: 0\n");
		return EFPG_ACK_NODATA_ERR;
	}

	/* hash */
	if (efpg_efuse_read(data, EFPG_BOOT_HASH_START, EFPG_BOOT_HASH_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	/* correct bit error */
	if (efpg_efuse_read(&tmp, EFPG_BOOT_1ST_EN_START, EFPG_BOOT_1ST_EN_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	if (tmp != 0) {
		if (efpg_efuse_read(&tmp, EFPG_BOOT_1ST_NO_START, EFPG_BOOT_1ST_NO_NUM) < 0)
			return EFPG_ACK_RW_ERR;
		byte_cnt = tmp >> 3;
		bit_cnt = tmp & 0x07;
		data[byte_cnt] ^= (0x1 << bit_cnt);

		if (efpg_efuse_read(&tmp, EFPG_BOOT_2ND_EN_START, EFPG_BOOT_2ND_EN_NUM) < 0)
			return EFPG_ACK_RW_ERR;

		if (tmp != 0) {
			if (efpg_efuse_read(&tmp, EFPG_BOOT_2ND_NO_START, EFPG_BOOT_2ND_NO_NUM) < 0)
				return EFPG_ACK_RW_ERR;
			byte_cnt = tmp >> 3;
			bit_cnt = tmp & 0x07;
			data[byte_cnt] ^= (0x1 << bit_cnt);
		}
	}

	return EFPG_ACK_OK;
}

static uint16_t efpg_efuse_read_dcxo(uint8_t *data)
{
	uint8_t idx = 1;
	uint8_t flag = 0;
	uint32_t start_bit;

	/* flag */
	if (efpg_efuse_read(&flag, EFPG_DCXO_FLAG_START, EFPG_DCXO_FLAG_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	if (flag == 0) {
		EFPG_WARN("efpg efuse read dcxo flag: 0\n");
		return EFPG_ACK_NODATA_ERR;
	}

	while ((flag & 0x3) == 0) {
		flag = flag >> 2;
		idx++;
	}

	/* DCXO TRIM */
	start_bit = EFPG_DCXO_TRIM_START + (idx - 1) * EFPG_DCXO_TRIM_NUM;
	if (efpg_efuse_read(data, start_bit, EFPG_DCXO_TRIM_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	return EFPG_ACK_OK;
}

static uint16_t efpg_efuse_read_pout(uint8_t *data)
{
	uint8_t idx = 1;
	uint8_t flag = 0;
	uint32_t start_bit;

	/* flag */
	if (efpg_efuse_read(&flag, EFPG_POUT_FLAG_START, EFPG_POUT_FLAG_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	if (flag == 0) {
		EFPG_WARN("efpg efuse read pout flag: 0\n");
		return EFPG_ACK_NODATA_ERR;
	}

	while ((flag & 0x3) == 0) {
		flag = flag >> 2;
		idx++;
	}

	/* POUT CAL */
	start_bit = EFPG_POUT_CAL_START + (idx - 1) * EFPG_POUT_CAL_NUM;
	if (efpg_efuse_read(data, start_bit, EFPG_POUT_CAL_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	return EFPG_ACK_OK;
}

static uint16_t efpg_efuse_read_mac(uint8_t *data)
{
	uint8_t idx = 1;
	uint32_t flag = 0;
	uint32_t start_bit;

	/* flag */
	if (efpg_efuse_read((uint8_t *)&flag, EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	if (flag == 0) {
		EFPG_WARN("efpg efuse read mac flag: 0\n");
		return EFPG_ACK_NODATA_ERR;
	}

	while ((flag & 0x3) == 0) {
		flag = flag >> 2;
		idx++;
	}

	/* MAC */
	start_bit = EFPG_MAC_ADDR_START + (idx - 1) * EFPG_MAC_ADDR_NUM;
	if (efpg_efuse_read(data, start_bit, EFPG_MAC_ADDR_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	return EFPG_ACK_OK;
}

static uint16_t efpg_efuse_write_hosc(uint8_t *data)
{
	uint8_t buf[EFPG_HOSC_BUF_LEN] = {0};

	if ((efpg_efuse_write(data, EFPG_HOSC_TYPE_START, EFPG_HOSC_TYPE_NUM) < 0)
		|| (efpg_efuse_read(buf, EFPG_HOSC_TYPE_START, EFPG_HOSC_TYPE_NUM) < 0))
		return EFPG_ACK_RW_ERR;

	if (efpg_memcmp(data, buf, EFPG_HOSC_BUF_LEN)) {
		EFPG_WARN("efpg efuse write hosc: write %#04x, read %#04x\n", data[0], buf[0]);
		return EFPG_ACK_DI_ERR;
	}

	return EFPG_ACK_OK;
}

static uint16_t efpg_efuse_write_boot(uint8_t *data)
{
	uint8_t tmp;
	uint8_t err_cnt = 0;
	uint8_t err_1st_no = 0;
	uint8_t err_2nd_no = 0;
	uint8_t buf[EFPG_BOOT_BUF_LEN] = {0};

	/* flag */
	tmp = 0x03;
	if ((efpg_efuse_write(&tmp, EFPG_BOOT_FLAG_START, EFPG_BOOT_FLAG_NUM) < 0)
		|| (efpg_efuse_read(&tmp, EFPG_BOOT_FLAG_START, EFPG_BOOT_FLAG_NUM) < 0))
		return EFPG_ACK_RW_ERR;

	if (tmp == 0) {
		EFPG_WARN("efpg efuse write boot flag: read 0\n");
		return EFPG_ACK_DI_ERR;
	}

	/* hash */
	if ((efpg_efuse_write(data, EFPG_BOOT_HASH_START, EFPG_BOOT_HASH_NUM) < 0)
		|| (efpg_efuse_read(buf, EFPG_BOOT_HASH_START, EFPG_BOOT_HASH_NUM) < 0))
		return EFPG_ACK_RW_ERR;

	if (efpg_boot_hash_cmp(data, buf, &err_cnt, &err_1st_no, &err_2nd_no) < 0) {
		EFPG_WARN("efpg efuse write boot hash: compare failed\n");
		return EFPG_ACK_DI_ERR;
	}

	/* error bit */
	if (err_cnt > 0) {
		/* bit en */
		tmp = 0x01;
		if ((efpg_efuse_write(&tmp, EFPG_BOOT_1ST_EN_START, EFPG_BOOT_1ST_EN_NUM) < 0)
			|| (efpg_efuse_read(&tmp, EFPG_BOOT_1ST_EN_START, EFPG_BOOT_1ST_EN_NUM) < 0))
			return EFPG_ACK_RW_ERR;

		if (tmp != 0x01) {
			EFPG_WARN("efpg efuse write boot 1st en: read %d\n", tmp);
			return EFPG_ACK_DI_ERR;
		}

		/* bit no */
		if ((efpg_efuse_write(&err_1st_no, EFPG_BOOT_1ST_NO_START, EFPG_BOOT_1ST_NO_NUM) < 0)
			|| (efpg_efuse_read(&tmp, EFPG_BOOT_1ST_NO_START, EFPG_BOOT_1ST_NO_NUM) < 0))
			return EFPG_ACK_RW_ERR;

		if (err_1st_no != tmp) {
			EFPG_WARN("efpg efuse write boot 1st no: write %d, read %d\n", err_1st_no, tmp);
			return EFPG_ACK_DI_ERR;
		}

		if (err_cnt == 2) {
			/* bit en */
			tmp = 0x01;
			if ((efpg_efuse_write(&tmp, EFPG_BOOT_2ND_EN_START, EFPG_BOOT_2ND_EN_NUM) < 0)
				|| (efpg_efuse_read(&tmp, EFPG_BOOT_2ND_EN_START, EFPG_BOOT_2ND_EN_NUM) < 0))
				return EFPG_ACK_RW_ERR;

			if (tmp != 0x01) {
				EFPG_WARN("efpg efuse write boot 2nd en: read %d\n", tmp);
				return EFPG_ACK_DI_ERR;
			}

			/* bit no */
			if ((efpg_efuse_write(&err_2nd_no, EFPG_BOOT_2ND_NO_START, EFPG_BOOT_2ND_NO_NUM) < 0)
				|| (efpg_efuse_read(&tmp, EFPG_BOOT_2ND_NO_START, EFPG_BOOT_2ND_NO_NUM) < 0))
				return EFPG_ACK_RW_ERR;

			if (err_2nd_no != tmp) {
				EFPG_WARN("efpg efuse write boot 2nd no: write %d, read %d\n", err_2nd_no, tmp);
				return EFPG_ACK_DI_ERR;
			}
		}
	}

	return EFPG_ACK_OK;
}

static uint16_t efpg_efuse_write_dcxo(uint8_t *data)
{
	uint8_t tmp;
	uint8_t idx = 0;
	uint8_t flag = 0;
	uint8_t buf[EFPG_DCXO_BUF_LEN] = {0};
	uint32_t start_bit;

	if (efpg_efuse_read(&flag, EFPG_DCXO_FLAG_START, EFPG_DCXO_FLAG_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	while (((flag & 0x3) == 0) && (idx < EFPG_DCXO_IDX_MAX)) {
		flag = flag >> 2;
		idx++;
	}

	while (idx > 0) {
		tmp = 0x3;
		start_bit = EFPG_DCXO_FLAG_START + (idx - 1) * 2;
		if ((efpg_efuse_write(&tmp, start_bit, 2) < 0)
			|| (efpg_efuse_read(&tmp, start_bit, 2) < 0))
			return EFPG_ACK_RW_ERR;

		if (tmp == 0) {
			idx--;
			continue;
		}

		start_bit = EFPG_DCXO_TRIM_START + (idx - 1) * EFPG_DCXO_TRIM_NUM;
		if ((efpg_efuse_write(data, start_bit, EFPG_DCXO_TRIM_NUM) < 0)
			|| (efpg_efuse_read(buf, start_bit, EFPG_DCXO_TRIM_NUM) < 0))
			return EFPG_ACK_RW_ERR;

		if (efpg_memcmp(data, buf, EFPG_DCXO_BUF_LEN)) {
			idx--;
			continue;
		}

		return EFPG_ACK_OK;
	}

	return EFPG_ACK_DI_ERR;
}

static uint16_t efpg_efuse_write_pout(uint8_t *data)
{
	uint8_t tmp;
	uint8_t idx = 0;
	uint8_t flag = 0;
	uint8_t buf[EFPG_POUT_BUF_LEN] = {0};
	uint32_t start_bit;

	if (efpg_efuse_read(&flag, EFPG_POUT_FLAG_START, EFPG_POUT_FLAG_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	while (((flag & 0x3) == 0) && (idx < EFPG_POUT_IDX_MAX)) {
		flag = flag >> 2;
		idx++;
	}

	while (idx > 0) {
		tmp = 0x3;
		start_bit = EFPG_POUT_FLAG_START + (idx - 1) * 2;
		if ((efpg_efuse_write(&tmp, start_bit, 2) < 0)
			|| (efpg_efuse_read(&tmp, start_bit, 2) < 0))
			return EFPG_ACK_RW_ERR;

		if (tmp == 0) {
			idx--;
			continue;
		}

		start_bit = EFPG_POUT_CAL_START + (idx - 1) * EFPG_POUT_CAL_NUM;
		if ((efpg_efuse_write(data, start_bit, EFPG_POUT_CAL_NUM) < 0)
			|| (efpg_efuse_read(buf, start_bit, EFPG_POUT_CAL_NUM) < 0))
			return EFPG_ACK_RW_ERR;

		if (efpg_memcmp(data, buf, EFPG_POUT_BUF_LEN)) {
			idx--;
			continue;
		}

		return EFPG_ACK_OK;
	}

	return EFPG_ACK_DI_ERR;
}

static uint16_t efpg_efuse_write_mac(uint8_t *data)
{
	uint8_t tmp;
	uint8_t idx = 0;
	uint32_t flag = 0;
	uint8_t buf[EFPG_MAC_BUF_LEN] = {0};
	uint32_t start_bit;

	if (efpg_efuse_read((uint8_t *)&flag, EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM) < 0)
		return EFPG_ACK_RW_ERR;

	while (((flag & 0x3) == 0) && (idx < EFPG_MAC_IDX_MAX)) {
		flag = flag >> 2;
		idx++;
	}

	while (idx > 0) {
		tmp = 0x3;
		start_bit = EFPG_MAC_FLAG_START + (idx - 1) * 2;
		if ((efpg_efuse_write(&tmp, start_bit, 2) < 0)
			|| (efpg_efuse_read(&tmp, start_bit, 2) < 0))
			return EFPG_ACK_RW_ERR;

		if (tmp == 0) {
			idx--;
			continue;
		}

		start_bit = EFPG_MAC_ADDR_START + (idx - 1) * EFPG_MAC_ADDR_NUM;
		if ((efpg_efuse_write(data, start_bit, EFPG_MAC_ADDR_NUM) < 0)
			|| (efpg_efuse_read(buf, start_bit, EFPG_MAC_ADDR_NUM) < 0))
			return EFPG_ACK_RW_ERR;

		if (efpg_memcmp(data, buf, EFPG_MAC_BUF_LEN)) {
			idx--;
			continue;
		}

		return EFPG_ACK_OK;
	}

	return EFPG_ACK_DI_ERR;
}

uint16_t efpg_read_area(efpg_area_t area, uint8_t *data)
{
	switch (area) {
	case EFPG_AREA_HOSC:
		return efpg_efuse_read_hosc(data);
	case EFPG_AREA_BOOT:
		return efpg_efuse_read_boot(data);
	case EFPG_AREA_DCXO:
		return efpg_efuse_read_dcxo(data);
	case EFPG_AREA_POUT:
		return efpg_efuse_read_pout(data);
	case EFPG_AREA_MAC:
		return efpg_efuse_read_mac(data);
	default :
		EFPG_WARN("efpg read area failed: area %d\n", area);
		return EFPG_ACK_RW_ERR;
	}
}

uint16_t efpg_write_area(efpg_area_t area, uint8_t *data)
{
	switch (area) {
	case EFPG_AREA_HOSC:
		return efpg_efuse_write_hosc(data);
	case EFPG_AREA_BOOT:
		return efpg_efuse_write_boot(data);
	case EFPG_AREA_DCXO:
		return efpg_efuse_write_dcxo(data);
	case EFPG_AREA_POUT:
		return efpg_efuse_write_pout(data);
	case EFPG_AREA_MAC:
		return efpg_efuse_write_mac(data);
	default :
		EFPG_WARN("efpg write area failed: area %d\n", area);
		return EFPG_ACK_RW_ERR;
	}
}

int efpg_read_efuse(uint8_t *data, uint32_t start_bit, uint32_t bit_num)
{
	if ((data == NULL)
		|| (start_bit >= EFUSE_BIT_NUM)
		|| (bit_num > EFUSE_BIT_NUM)
		|| (start_bit + bit_num > EFUSE_BIT_NUM)) {
		EFPG_ERR("efpg read efuse failed: data %p, start_bit %d, bit_num %d\n",
				 data, start_bit, bit_num);
		return -1;
	}

	return efpg_efuse_read(data, start_bit, bit_num);
}

int efpg_write_efuse(uint8_t *data, uint32_t start_bit, uint32_t bit_num)
{
	if ((data == NULL)
		|| (start_bit >= EFUSE_BIT_NUM)
		|| (bit_num > EFUSE_BIT_NUM)
		|| (start_bit + bit_num > EFUSE_BIT_NUM)) {
		EFPG_ERR("efpg write efuse failed: data %p, start_bit %d, bit_num %d\n",
				 data, start_bit, bit_num);
		return -1;
	}

	if (start_bit < EFPG_WR_PROTECT_BIT_NUM) {
		EFPG_ERR("efpg write efuse failed: start_bit %d, protect bit num %d\n",
				 start_bit, EFPG_WR_PROTECT_BIT_NUM);
		return -1;
	}

	return efpg_efuse_write(data, start_bit, bit_num);
}

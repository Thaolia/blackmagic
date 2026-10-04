/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2026 1BitSquared <info@1bitsquared.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file implements support for the Cmsemicon BAT32G135 (and pin/flash
 * compatible BAT32 parts), an ARM Cortex-M0+ device. It provides the memory
 * map and the Flash programming routines driving the on-chip Flash Memory
 * Controller (FMC).
 *
 * The FMC register layout and the erase/program sequences below are sourced
 * from the Cmsemicon BAT32G135 User Manual V0.11 (§28) and match a working
 * SWD implementation (raiden-pico swd_bat32_flash_program/sector_erase).
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

/* Code flash: 0x00000000, 64 KiB, 128 sectors of 512 bytes */
#define BAT32_CODE_FLASH_BASE   0x00000000U
#define BAT32_CODE_FLASH_SIZE   0x00010000U
#define BAT32_FLASH_SECTOR_SIZE 512U

/* SRAM: 0x20000000, 8 KiB */
#define BAT32_SRAM_BASE 0x20000000U
#define BAT32_SRAM_SIZE 0x00002000U

/* Flash Memory Controller (FMC) registers */
#define BAT32_FMC_FLSTS   0x40020000U /* bit0 OVF: operation complete (write-1-to-clear) */
#define BAT32_FMC_FLOPMD1 0x40020004U /* arming sequence, part 1 */
#define BAT32_FMC_FLOPMD2 0x40020008U /* arming sequence, part 2 */
#define BAT32_FMC_FLERMD  0x4002000cU /* erase mode selector */
#define BAT32_FMC_FLPROT  0x40020020U /* write/erase protection lock */

#define BAT32_FMC_FLSTS_OVF 0x00000001U

#define BAT32_FMC_FLPROT_UNLOCK 0x000000f1U
#define BAT32_FMC_FLPROT_LOCK   0x000000f0U

#define BAT32_FMC_FLERMD_NONE   0x00000000U
#define BAT32_FMC_FLERMD_SECTOR 0x00000010U

/* Datasheet: sector erase 4-5 ms, single-word program 24-30 us. Wide margins. */
#define BAT32_FMC_ERASE_TIMEOUT_MS   100U
#define BAT32_FMC_PROGRAM_TIMEOUT_MS 10U

static bool bat32_enter_flash_mode(target_s *target);
static bool bat32_flash_erase(target_flash_s *flash, target_addr_t addr, size_t len);
static bool bat32_flash_write(target_flash_s *flash, target_addr_t dest, const void *src, size_t len);

static void bat32_add_flash(target_s *const target)
{
	target_flash_s *flash = calloc(1, sizeof(*flash));
	if (!flash) { /* calloc failed: heap exhaustion */
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return;
	}

	flash->start = BAT32_CODE_FLASH_BASE;
	flash->length = BAT32_CODE_FLASH_SIZE;
	flash->blocksize = BAT32_FLASH_SECTOR_SIZE;
	flash->writesize = BAT32_FLASH_SECTOR_SIZE;
	flash->erase = bat32_flash_erase;
	flash->write = bat32_flash_write;
	flash->erased = 0xffU;
	target_add_flash(target, flash);
}

bool bat32_probe(target_s *const target)
{
	/*
	 * The BAT32 presents as a plain ARM Cortex-M0+ (part ID 0x4c0) with no
	 * known-constant identity register (DEV_ID reads as 0), so this probe is
	 * placed last in the Cortex-M0+ chain and claims the part by position: it
	 * catches an M0+ that none of the earlier, positively-identified drivers
	 * recognised. On a mixed bus this could mis-claim another unknown M0+.
	 */
	target->driver = "Cmsemicon BAT32";
	target->enter_flash_mode = bat32_enter_flash_mode;

	target_add_ram32(target, BAT32_SRAM_BASE, BAT32_SRAM_SIZE);
	bat32_add_flash(target);
	return true;
}

static bool bat32_enter_flash_mode(target_s *const target)
{
	/*
	 * The FMC programs/erases one array while the core must not be fetching
	 * from it, so hold the core halted for the whole Flash session.
	 */
	target_reset(target);
	target_halt_request(target);

	platform_timeout_s timeout;
	platform_timeout_set(&timeout, 500U);
	target_halt_reason_e reason = TARGET_HALT_RUNNING;
	while (reason == TARGET_HALT_RUNNING) {
		if (platform_timeout_is_expired(&timeout))
			return false;
		reason = target_halt_poll(target, NULL);
	}
	return true;
}

static bool bat32_fmc_wait_ovf(target_s *const target, const uint32_t timeout_ms)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, timeout_ms);
	while (!(target_mem32_read32(target, BAT32_FMC_FLSTS) & BAT32_FMC_FLSTS_OVF)) {
		if (target_check_error(target) || platform_timeout_is_expired(&timeout))
			return false;
	}
	/* OVF is write-1-to-clear */
	target_mem32_write32(target, BAT32_FMC_FLSTS, BAT32_FMC_FLSTS_OVF);
	return true;
}

static bool bat32_flash_erase(target_flash_s *const flash, const target_addr_t addr, const size_t len)
{
	(void)len; /* The Flash layer calls this once per blocksize (one sector) */
	target_s *const target = flash->t;

	/* Order per the vendor EraseChip(): mode, unlock, arming pair (0x55/0xAA), then the trigger write */
	target_mem32_write32(target, BAT32_FMC_FLERMD, BAT32_FMC_FLERMD_SECTOR);
	target_mem32_write32(target, BAT32_FMC_FLPROT, BAT32_FMC_FLPROT_UNLOCK);
	target_mem32_write32(target, BAT32_FMC_FLOPMD1, 0x55U);
	target_mem32_write32(target, BAT32_FMC_FLOPMD2, 0xaaU);
	target_mem32_write32(target, addr, 0xffffffffU);
	const bool result = bat32_fmc_wait_ovf(target, BAT32_FMC_ERASE_TIMEOUT_MS);

	target_mem32_write32(target, BAT32_FMC_FLERMD, BAT32_FMC_FLERMD_NONE);
	target_mem32_write32(target, BAT32_FMC_FLPROT, BAT32_FMC_FLPROT_LOCK);
	return result;
}

static bool bat32_flash_write(
	target_flash_s *const flash, const target_addr_t dest, const void *const src, const size_t len)
{
	target_s *const target = flash->t;
	const uint8_t *const data = src;

	target_mem32_write32(target, BAT32_FMC_FLPROT, BAT32_FMC_FLPROT_UNLOCK);

	bool result = true;
	for (size_t offset = 0; offset < len && result; ++offset) {
		/*
		 * The arming pair (0xAA/0x55, the inverse of the erase pair) must be
		 * re-written before every byte, exactly as the vendor ProgramPage() does.
		 */
		target_mem32_write32(target, BAT32_FMC_FLOPMD1, 0xaaU);
		target_mem32_write32(target, BAT32_FMC_FLOPMD2, 0x55U);
		target_mem32_write(target, dest + offset, &data[offset], 1U);
		result = bat32_fmc_wait_ovf(target, BAT32_FMC_PROGRAM_TIMEOUT_MS);
	}

	target_mem32_write32(target, BAT32_FMC_FLPROT, BAT32_FMC_FLPROT_LOCK);
	return result;
}

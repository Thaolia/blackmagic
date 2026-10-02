/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2026 Saket Sinha <saket.sinha89@gmail.com>
 * All rights reserved.
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

#include "general.h"
#include "adi.h"
#include "adiv5_internal.h"
#include "riscv_debug.h"
#include "target_probe.h"

/*
 * RISC-V DMI "gateway" AP, as implemented by the SpinalHDL SWD DTM used by VexRiscv and VexiiRiscv
 * (spinal.lib.com.swd + DebugTransportModuleSwd).
 *
 * This is a designer-defined AP on an ADIv5 MINDP SW-DP. Unlike the MEM-AP shaped DTM, the DM is not memory
 * mapped: the AP exposes the DMI through four bank 0 registers, and ignores SELECT.APBANKSEL.
 *
 *   0x00  AP_IDR       RO  identification constant (0x74726976, ASCII "triv")
 *   0x04  DMI_ADDR     RW  7-bit DMI word address
 *   0x08  DMI_DATA     RW  performs the DMI access at DMI_ADDR
 *   0x0c  POSTED_READ  RO  last DMI read result (RDBUFF semantics, not needed here)
 *
 * Because APBANKSEL is ignored, the generic AP scan's IDR read (bank 0xf, offset 0xfc) lands on POSTED_READ,
 * so the AP has to be identified at offset 0x00 before that scan runs.
 *
 * A DMI access in flight is signalled as a WAIT by the DP and retried by the SWD layer, so no polling happens here.
 */

/*
 * DPIDR[11:1] as sent on the wire, so this JEP-106 value is already shifted right by 1.
 * Correcting for that shift decodes 0x555 as 0xa55 (continuation 10, identity 0x55, Facebook Inc).
 * This DTM squats on that code. Parts from that vendor are unlikely to appear here, so keying on it is safe.
 */
#define RISCV_GATEWAY_DPIDR_DESIGNER 0x555U
#define RISCV_GATEWAY_DPIDR_PARTNO   0xbaU
#define RISCV_GATEWAY_DPIDR_VERSION  1U

#define RISCV_GATEWAY_AP_IDR_VALUE 0x74726976U
#define RISCV_GATEWAY_AP_IDR       ADIV5_AP_REG(0x00U)
#define RISCV_GATEWAY_DMI_ADDR     ADIV5_AP_REG(0x04U)
#define RISCV_GATEWAY_DMI_DATA     ADIV5_AP_REG(0x08U)

#define RISCV_GATEWAY_DMI_ADDR_WIDTH 7U
#define RISCV_GATEWAY_DMI_ADDR_MAX   ((1U << RISCV_GATEWAY_DMI_ADDR_WIDTH) - 1U)

typedef struct riscv_dmi_gateway {
	/* Must stay first: generic RISC-V code casts ADI-backed DMIs to riscv_dmi_ap_s */
	riscv_dmi_ap_s dmi_ap;
	/* Last value written to DMI_ADDR, so repeated accesses to one DM register (status polling) cost one frame */
	uint32_t cached_address;
	bool cached_address_valid;
} riscv_dmi_gateway_s;

static void riscv_gateway_dtm_handler(adiv5_access_port_s *ap);
static bool riscv_gateway_dmi_read(riscv_dmi_s *dmi, uint32_t address, uint32_t *value);
static bool riscv_gateway_dmi_write(riscv_dmi_s *dmi, uint32_t address, uint32_t value);

bool riscv_adi_gateway_dp_probe(adiv5_debug_port_s *const dp)
{
	if (dp->version != RISCV_GATEWAY_DPIDR_VERSION || dp->partno != RISCV_GATEWAY_DPIDR_PARTNO ||
		dp->designer_code != adi_decode_designer(RISCV_GATEWAY_DPIDR_DESIGNER))
		return false;

	adiv5_access_port_s probe_ap = {
		.dp = dp,
		.apsel = 0U,
	};
	const uint32_t ap_idr = adiv5_ap_read(&probe_ap, RISCV_GATEWAY_AP_IDR);
	if (adiv5_dp_error(dp) != 0U || ap_idr != RISCV_GATEWAY_AP_IDR_VALUE) {
		DEBUG_INFO("DP looks like a RISC-V DMI gateway, but AP IDR is 0x%08" PRIx32 "\n", ap_idr);
		return false;
	}
	DEBUG_INFO("RISC-V DMI gateway AP found: AP IDR 0x%08" PRIx32 "\n", ap_idr);

	adiv5_access_port_s *const ap = calloc(1, sizeof(*ap));
	if (!ap) { /* calloc failed: heap exhaustion */
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return true;
	}
	memcpy(ap, &probe_ap, sizeof(*ap));
	ap->idr = ap_idr;
	adiv5_ap_ref(ap);
	riscv_gateway_dtm_handler(ap);
	/* Drop our reference; the DMI holds its own if any harts were found */
	adiv5_ap_unref(ap);
	return true;
}

static void riscv_gateway_dtm_handler(adiv5_access_port_s *const ap)
{
	riscv_dmi_gateway_s *gateway = calloc(1, sizeof(*gateway));
	if (!gateway) { /* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}

	riscv_dmi_ap_s *const dmi_ap = &gateway->dmi_ap;
	dmi_ap->ap = ap;
	adiv5_ap_ref(ap);
	/* dev_index/idle_cycles of 0xff mark this as an ADI-backed DMI for the generic code */
	dmi_ap->dmi.dev_index = 0xffU;
	dmi_ap->dmi.idle_cycles = 0xffU;
	dmi_ap->dmi.designer_code = ap->dp->designer_code;
	dmi_ap->dmi.version = RISCV_DEBUG_NONSTANDARD; /* A custom DTM in the sense of the RISC-V Debug spec §6, p. 92 */
	dmi_ap->dmi.address_width = RISCV_GATEWAY_DMI_ADDR_WIDTH;

	dmi_ap->dmi.read = riscv_gateway_dmi_read;
	dmi_ap->dmi.write = riscv_gateway_dmi_write;
	riscv_dmi_init(&dmi_ap->dmi);

	/* If we failed to find any DMs or Harts, free the structure */
	if (!dmi_ap->dmi.ref_count) {
		adiv5_ap_unref(ap);
		free(gateway);
	}
}

static bool riscv_gateway_select(riscv_dmi_gateway_s *const gateway, const uint32_t address)
{
	if (address > RISCV_GATEWAY_DMI_ADDR_MAX) {
		/* Truncating would silently alias two DM registers onto one */
		DEBUG_ERROR(
			"DMI address 0x%" PRIx32 " exceeds the gateway's %u-bit range\n", address, RISCV_GATEWAY_DMI_ADDR_WIDTH);
		gateway->cached_address_valid = false;
		return false;
	}
	if (gateway->cached_address_valid && gateway->cached_address == address)
		return true;
	adiv5_ap_write(gateway->dmi_ap.ap, RISCV_GATEWAY_DMI_ADDR, address);
	gateway->cached_address = address;
	gateway->cached_address_valid = true;
	return true;
}

static bool riscv_gateway_dmi_read(riscv_dmi_s *const dmi, const uint32_t address, uint32_t *const value)
{
	riscv_dmi_gateway_s *const gateway = (riscv_dmi_gateway_s *)dmi;
	adiv5_access_port_s *const ap = gateway->dmi_ap.ap;
	if (!riscv_gateway_select(gateway, address))
		return false;
	/* AP reads are posted; the ADIv5 layer collects the result from RDBUFF */
	*value = adiv5_ap_read(ap, RISCV_GATEWAY_DMI_DATA);
	if (adiv5_dp_error(ap->dp) != 0U) {
		gateway->cached_address_valid = false;
		return false;
	}
	return true;
}

static bool riscv_gateway_dmi_write(riscv_dmi_s *const dmi, const uint32_t address, const uint32_t value)
{
	riscv_dmi_gateway_s *const gateway = (riscv_dmi_gateway_s *)dmi;
	adiv5_access_port_s *const ap = gateway->dmi_ap.ap;
	if (!riscv_gateway_select(gateway, address))
		return false;
	adiv5_ap_write(ap, RISCV_GATEWAY_DMI_DATA, value);
	if (adiv5_dp_error(ap->dp) != 0U) {
		gateway->cached_address_valid = false;
		return false;
	}
	return true;
}

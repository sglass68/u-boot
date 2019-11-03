/*
 * This file is part of the coreboot project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ACPIGEN_H
#define _ACPIGEN_H

#include <acpi_table.h>

struct acpi_cstate;
struct acpi_pld;
struct acpi_gpio;
struct acpi_tstate;

/* Values that can be returned for ACPI Device _STA method */
#define ACPI_STATUS_DEVICE_PRESENT	BIT(0)
#define ACPI_STATUS_DEVICE_ENABLED	BIT(1)
#define ACPI_STATUS_DEVICE_SHOW_IN_UI	BIT(2)
#define ACPI_STATUS_DEVICE_STATE_OK	BIT(3)

#define ACPI_STATUS_DEVICE_ALL_OFF	0
#define ACPI_STATUS_DEVICE_ALL_ON	(ACPI_STATUS_DEVICE_PRESENT |\
					 ACPI_STATUS_DEVICE_ENABLED |\
					 ACPI_STATUS_DEVICE_SHOW_IN_UI |\
					 ACPI_STATUS_DEVICE_STATE_OK)
#define ACPI_STATUS_DEVICE_HIDDEN_ON	(ACPI_STATUS_DEVICE_PRESENT |\
					 ACPI_STATUS_DEVICE_ENABLED |\
					 ACPI_STATUS_DEVICE_STATE_OK)

/* ACPI Op/Prefix Codes */
enum {
	ZERO_OP			= 0x00,
	ONE_OP			= 0x01,
	ALIAS_OP		= 0x06,
	NAME_OP			= 0x08,
	BYTE_PREFIX		= 0x0A,
	WORD_PREFIX		= 0x0B,
	DWORD_PREFIX		= 0x0C,
	STRING_PREFIX		= 0x0D,
	QWORD_PREFIX		= 0x0E,
	SCOPE_OP		= 0x10,
	BUFFER_OP		= 0x11,
	PACKAGE_OP		= 0x12,
	VARIABLE_PACKAGE_OP	= 0x13,
	METHOD_OP		= 0x14,
	EXTERNAL_OP		= 0x15,
	DUAL_NAME_PREFIX	= 0x2E,
	MULTI_NAME_PREFIX	= 0x2F,
	EXT_OP_PREFIX		= 0x5B,

	MUTEX_OP		= 0x01,
	EVENT_OP		= 0x01,
	SF_RIGHT_OP		= 0x10,
	SF_LEFT_OP		= 0x11,
	COND_REFOF_OP		= 0x12,
	CREATEFIELD_OP		= 0x13,
	LOAD_TABLE_OP		= 0x1f,
	LOAD_OP		= 0x20,
	STALL_OP		= 0x21,
	SLEEP_OP		= 0x22,
	ACQUIRE_OP		= 0x23,
	SIGNAL_OP		= 0x24,
	WAIT_OP		= 0x25,
	RST_OP			= 0x26,
	RELEASE_OP		= 0x27,
	FROM_BCD_OP		= 0x28,
	TO_BCD_OP		= 0x29,
	UNLOAD_OP		= 0x2A,
	REVISON_OP		= 0x30,
	DEBUG_OP		= 0x31,
	FATAL_OP		= 0x32,
	TIMER_OP		= 0x33,
	OPREGION_OP		= 0x80,
	FIELD_OP		= 0x81,
	DEVICE_OP		= 0x82,
	PROCESSOR_OP		= 0x83,
	POWER_RES_OP		= 0x84,
	THERMAL_ZONE_OP	= 0x85,
	INDEX_FIELD_OP		= 0x86,
	BANK_FIELD_OP		= 0x87,
	DATA_REGION_OP		= 0x88,

	ROOT_PREFIX		= 0x5C,
	PARENT_PREFIX		= 0x5D,
	LOCAL0_OP		= 0x60,
	LOCAL1_OP		= 0x61,
	LOCAL2_OP		= 0x62,
	LOCAL3_OP		= 0x63,
	LOCAL4_OP		= 0x64,
	LOCAL5_OP		= 0x65,
	LOCAL6_OP		= 0x66,
	LOCAL7_OP		= 0x67,
	ARG0_OP			= 0x68,
	ARG1_OP			= 0x69,
	ARG2_OP			= 0x6A,
	ARG3_OP			= 0x6B,
	ARG4_OP			= 0x6C,
	ARG5_OP			= 0x6D,
	ARG6_OP			= 0x6E,
	STORE_OP		= 0x70,
	REF_OF_OP		= 0x71,
	ADD_OP			= 0x72,
	CONCATENATE_OP		= 0x73,
	SUBTRACT_OP		= 0x74,
	INCREMENT_OP		= 0x75,
	DECREMENT_OP		= 0x76,
	MULTIPLY_OP		= 0x77,
	DIVIDE_OP		= 0x78,
	SHIFT_LEFT_OP		= 0x79,
	SHIFT_RIGHT_OP		= 0x7A,
	AND_OP			= 0x7B,
	NAND_OP			= 0x7C,
	OR_OP			= 0x7D,
	NOR_OP			= 0x7E,
	XOR_OP			= 0x7F,
	NOT_OP			= 0x80,
	FD_SHIFT_LEFT_BIT_OR	= 0x81,
	FD_SHIFT_RIGHT_BIT_OR	= 0x82,
	DEREF_OP		= 0x83,
	CONCATENATE_TEMP_OP	= 0x84,
	MOD_OP			= 0x85,
	NOTIFY_OP		= 0x86,
	SIZEOF_OP		= 0x87,
	INDEX_OP		= 0x88,
	MATCH_OP		= 0x89,
	CREATE_DWORD_OP		= 0x8A,
	CREATE_WORD_OP		= 0x8B,
	CREATE_BYTE_OP		= 0x8C,
	CREATE_BIT_OP		= 0x8D,
	OBJ_TYPE_OP		= 0x8E,
	CREATE_QWORD_OP		= 0x8F,
	LAND_OP			= 0x90,
	LOR_OP			= 0x91,
	LNOT_OP			= 0x92,
	LEQUAL_OP		= 0x93,
	LGREATER_OP		= 0x94,
	LLESS_OP		= 0x95,
	TO_BUFFER_OP		= 0x96,
	TO_DEC_STRING_OP	= 0x97,
	TO_HEX_STRING_OP	= 0x98,
	TO_INTEGER_OP		= 0x99,
	TO_STRING_OP		= 0x9C,
	CP_OBJ_OP		= 0x9D,
	MID_OP			= 0x9E,
	CONTINUE_OP		= 0x9F,
	IF_OP			= 0xA0,
	ELSE_OP			= 0xA1,
	WHILE_OP		= 0xA2,
	NOOP_OP			= 0xA3,
	RETURN_OP		= 0xA4,
	BREAK_OP		= 0xA5,
	COMMENT_OP		= 0xA9,
	BREAKPIONT_OP		= 0xCC,
	ONES_OP			= 0xFF,
};

#define FIELDLIST_OFFSET(X)		{ .type = OFFSET, \
					  .name = "",\
					  .bits = X * 8, \
					}
#define FIELDLIST_NAMESTR(X, Y)		{ .type = NAME_STRING, \
					  .name = X, \
					  .bits = Y, \
					}

#define FIELD_ANYACC			0
#define FIELD_BYTEACC			1
#define FIELD_WORDACC			2
#define FIELD_DWORDACC			3
#define FIELD_QWORDACC			4
#define FIELD_BUFFERACC			5
#define FIELD_NOLOCK			(0<<4)
#define FIELD_LOCK			(1<<4)
#define FIELD_PRESERVE			(0<<5)
#define FIELD_WRITEASONES		(1<<5)
#define FIELD_WRITEASZEROS		(2<<5)

enum field_type {
	OFFSET,
	NAME_STRING,
	FIELD_TYPE_MAX,
};

struct fieldlist {
	enum field_type type;
	const char *name;
	u32 bits;
};

#define OPREGION(rname, space, offset, len)	{.name = rname, \
						 .regionspace = space, \
						 .regionoffset = offset, \
						 .regionlen = len, \
						}

enum region_space {
	SYSTEMMEMORY,
	SYSTEMIO,
	PCI_CONFIG,
	EMBEDDEDCONTROL,
	SMBUS,
	CMOS,
	PCIBARTARGET,
	IPMI,
	GPIO_REGION,
	GPSERIALBUS,
	PCC,
	FIXED_HARDWARE = 0x7F,
	REGION_SPACE_MAX,
};

struct opregion {
	const char *name;
	enum region_space regionspace;
	unsigned long regionoffset;
	unsigned long regionlen;
};

#define DSM_UUID(DSM_UUID, DSM_CALLBACKS, DSM_COUNT, DSM_ARG) \
	{ .uuid = DSM_UUID, \
	.callbacks = DSM_CALLBACKS, \
	.count = DSM_COUNT, \
	.arg = DSM_ARG, \
	}

struct dsm_uuid {
	const char *uuid;
	void (**callbacks)(struct acpi_ctx *ctx, void *);
	size_t count;
	void *arg;
};

/*version 1 has 15 fields, version 2 has 19, and version 3 has 21 */
enum cppc_fields {
	CPPC_HIGHEST_PERF, /* can be DWORD */
	CPPC_NOMINAL_PERF, /* can be DWORD */
	CPPC_LOWEST_NONL_PERF, /* can be DWORD */
	CPPC_LOWEST_PERF, /* can be DWORD */
	CPPC_GUARANTEED_PERF,
	CPPC_DESIRED_PERF,
	CPPC_MIN_PERF,
	CPPC_MAX_PERF,
	CPPC_PERF_REDUCE_TOLERANCE,
	CPPC_TIME_WINDOW,
	CPPC_COUNTER_WRAP, /* can be DWORD */
	CPPC_REF_PERF_COUNTER,
	CPPC_DELIVERED_PERF_COUNTER,
	CPPC_PERF_LIMITED,
	CPPC_ENABLE, /* can be System I/O */
	CPPC_MAX_FIELDS_VER_1,
	CPPC_AUTO_SELECT = /* can be DWORD */
		CPPC_MAX_FIELDS_VER_1,
	CPPC_AUTO_ACTIVITY_WINDOW,
	CPPC_PERF_PREF,
	CPPC_REF_PERF, /* can be DWORD */
	CPPC_MAX_FIELDS_VER_2,
	CPPC_LOWEST_FREQ = /* can be DWORD */
		CPPC_MAX_FIELDS_VER_2,
	CPPC_NOMINAL_FREQ, /* can be DWORD */
	CPPC_MAX_FIELDS_VER_3,
};

struct cppc_config {
	u32 version; /* must be 1, 2, or 3 */
	/*
	 * The generic struct acpi_gen_regaddr structure is being used, though
	 * anything besides PPC or FFIXED generally requires checking
	 * if the OS has advertised support for it (via _OSC).
	 *
	 * NOTE: some fields permit DWORDs to be used.  If you
	 * provide a System Memory register with all zeros (which
	 * represents unsupported) then this will be used as-is.
	 * Otherwise, a System Memory register with a 32-bit
	 * width will be converted into a DWORD field (the value
	 * of which will be the value of 'addrl'.  Any other use
	 * of System Memory register is currently undefined.
	 * (i.e., if you have an actual need for System Memory
	 * then you'll need to adjust this kludge).
	 */
	struct acpi_gen_regaddr regs[CPPC_MAX_FIELDS_VER_3];
};

void acpigen_write_return_integer(struct acpi_ctx *ctx, u64 arg);
void acpigen_write_return_string(struct acpi_ctx *ctx, const char *arg);
void acpigen_write_len_f(struct acpi_ctx *ctx);
void acpigen_pop_len(struct acpi_ctx *ctx);
void acpigen_set_current(struct acpi_ctx *ctx, char *curr);
char *acpigen_get_current(struct acpi_ctx *ctx);
char *acpigen_write_package(struct acpi_ctx *ctx, int nr_el);
void acpigen_write_zero(struct acpi_ctx *ctx);
void acpigen_write_one(struct acpi_ctx *ctx);
void acpigen_write_ones(struct acpi_ctx *ctx);
void acpigen_write_byte(struct acpi_ctx *ctx, unsigned int data);
void acpigen_emit_byte(struct acpi_ctx *ctx, unsigned char data);
void acpigen_emit_ext_op(struct acpi_ctx *ctx, u8 op);
void acpigen_emit_word(struct acpi_ctx *ctx, unsigned int data);
void acpigen_emit_dword(struct acpi_ctx *ctx, unsigned int data);
void acpigen_emit_stream(struct acpi_ctx *ctx, const char *data, int size);
void acpigen_emit_string(struct acpi_ctx *ctx, const char *string);
void acpigen_emit_namestring(struct acpi_ctx *ctx, const char *namepath);
void acpigen_emit_eisaid(struct acpi_ctx *ctx, const char *eisaid);
void acpigen_write_word(struct acpi_ctx *ctx, unsigned int data);
void acpigen_write_dword(struct acpi_ctx *ctx, unsigned int data);
void acpigen_write_qword(struct acpi_ctx *ctx, u64 data);
void acpigen_write_integer(struct acpi_ctx *ctx, u64 data);
void acpigen_write_string(struct acpi_ctx *ctx, const char *string);
void acpigen_write_coreboot_hid(struct acpi_ctx *ctx,
				enum coreboot_acpi_ids id);
void acpigen_write_name(struct acpi_ctx *ctx, const char *name);
void acpigen_write_name_zero(struct acpi_ctx *ctx, const char *name);
void acpigen_write_name_one(struct acpi_ctx *ctx, const char *name);
void acpigen_write_name_string(struct acpi_ctx *ctx, const char *name,
			       const char *string);
void acpigen_write_name_dword(struct acpi_ctx *ctx, const char *name, u32 val);
void acpigen_write_name_qword(struct acpi_ctx *ctx, const char *name, u64 val);
void acpigen_write_name_byte(struct acpi_ctx *ctx, const char *name, u8 val);
void acpigen_write_name_integer(struct acpi_ctx *ctx, const char *name,
				u64 val);
void acpigen_write_scope(struct acpi_ctx *ctx, const char *name);
void acpigen_write_method(struct acpi_ctx *ctx, const char *name, int nargs);
void acpigen_write_method_serialized(struct acpi_ctx *ctx, const char *name,
				     int nargs);
void acpigen_write_device(struct acpi_ctx *ctx, const char *name);
void acpigen_write_ppc(struct acpi_ctx *ctx, u8 nr);
void acpigen_write_ppc_nvs(struct acpi_ctx *ctx);
void acpigen_write_empty_pct(struct acpi_ctx *ctx);
void acpigen_write_empty_ptc(struct acpi_ctx *ctx);
void acpigen_write_prw(struct acpi_ctx *ctx, u32 wake, u32 level);
void acpigen_write_sta(struct acpi_ctx *ctx, u8 status);
void acpigen_write_tpc(struct acpi_ctx *ctx, const char *gnvs_tpc_limit);
void acpigen_write_pss_package(struct acpi_ctx *ctx, u32 corefreq, u32 power,
			       u32 translat,
			u32 busmlat, u32 control, u32 status);
enum psd_coord {
	SW_ALL = 0xfc,
	SW_ANY = 0xfd,
	HW_ALL = 0xfe
};

void acpigen_write_psd_package(struct acpi_ctx *ctx, u32 domain, u32 numprocs,
			       enum psd_coord coordtype);
void acpigen_write_cst_package_entry(struct acpi_ctx *ctx,
				     struct acpi_cstate *cstate);
void acpigen_write_cst_package(struct acpi_ctx *ctx, struct acpi_cstate *entry,
			       int nentries);
typedef enum { CSD_HW_ALL = 0xfe } CSD_coord;
void acpigen_write_CSD_package(struct acpi_ctx *ctx, u32 domain, u32 numprocs,
			       CSD_coord coordtype, u32 index);
void acpigen_write_processor(struct acpi_ctx *ctx, u8 cpuindex, u32 pblock_addr, u8 pblock_len);
void acpigen_write_processor_package(struct acpi_ctx *ctx, const char *name,
				     uint first_core, uint core_count);
void acpigen_write_processor_cnot(struct acpi_ctx *ctx, const uint num_cores);
void acpigen_write_tss_package(struct acpi_ctx *ctx, int entries,
			       struct acpi_tstate *tstate_list);
void acpigen_write_tsd_package(struct acpi_ctx *ctx, u32 domain, u32 numprocs,
			       enum psd_coord coordtype);
void acpigen_write_mem32fixed(struct acpi_ctx *ctx, int readwrite, u32 base,
			      u32 size);
void acpigen_write_register_resource(struct acpi_ctx *ctx,
				     const struct acpi_gen_regaddr *addr);
void acpigen_write_irq(struct acpi_ctx *ctx, u16 mask);
void acpigen_write_resourcetemplate_header(struct acpi_ctx *ctx);
void acpigen_write_resourcetemplate_footer(struct acpi_ctx *ctx);
int acpigen_write_uuid(struct acpi_ctx *ctx, const char *uuid);
void acpigen_write_power_res(struct acpi_ctx *ctx, const char *name, u8 level,
			     u16 order, const char *dev_states[],
			     size_t dev_states_count);
void acpigen_write_sleep(struct acpi_ctx *ctx, u64 sleep_ms);
void acpigen_write_store(struct acpi_ctx *ctx);
void acpigen_write_store_ops(struct acpi_ctx *ctx, u8 src, u8 dst);
void acpigen_write_or(struct acpi_ctx *ctx, u8 arg1, u8 arg2, u8 res);
void acpigen_write_and(struct acpi_ctx *ctx, u8 arg1, u8 arg2, u8 res);
void acpigen_write_not(struct acpi_ctx *ctx, u8 arg, u8 res);
void acpigen_write_debug_string(struct acpi_ctx *ctx, const char *str);
void acpigen_write_debug_integer(struct acpi_ctx *ctx, u64 val);
void acpigen_write_debug_op(struct acpi_ctx *ctx, u8 op);
void acpigen_write_if(struct acpi_ctx *ctx);
void acpigen_write_if_and(struct acpi_ctx *ctx, u8 arg1, u8 arg2);
void acpigen_write_if_lequal_op_int(struct acpi_ctx *ctx, u8 op, u64 val);
void acpigen_write_else(struct acpi_ctx *ctx);
void acpigen_write_to_buffer(struct acpi_ctx *ctx, u8 src, u8 dst);
void acpigen_write_to_integer(struct acpi_ctx *ctx, u8 src, u8 dst);
void acpigen_write_byte_buffer(struct acpi_ctx *ctx, u8 *arr, size_t size);
void acpigen_write_return_byte_buffer(struct acpi_ctx *ctx, u8 *arr,
				      size_t size);
void acpigen_write_return_singleton_buffer(struct acpi_ctx *ctx, u8 arg);
void acpigen_write_return_byte(struct acpi_ctx *ctx, u8 arg);
void acpigen_write_upc(struct acpi_ctx *ctx, enum acpi_upc_type type);

typedef void (*hid_callback_func)(struct acpi_ctx *ctx, void *arg);

/*
 * Generate ACPI AML code for _DSM method.
 * This function takes as input uuid for the device, set of callbacks and
 * argument to pass into the callbacks. Callbacks should ensure that Local0 and
 * Local1 are left untouched. Use of Local2-Local7 is permitted in callbacks.
 */
int acpigen_write_dsm(struct acpi_ctx *ctx, const char *uuid,
		      hid_callback_func callbacks[], size_t count, void *arg);
int acpigen_write_dsm_uuid_arr(struct acpi_ctx *ctx, struct dsm_uuid *ids,
			       size_t count);

/*
 * Generate ACPI AML code for _CPC (struct acpi_ctx *ctx, Continuous Perfmance
 * Control). Execute the package function once to create a global table, then
 * execute the method function within each processor object to
 * create a method that points to the global table.
 */
int acpigen_write_cppc_package(struct acpi_ctx *ctx,
			       const struct cppc_config *config);
void acpigen_write_cppc_method(struct acpi_ctx *ctx);

/*
 * Generate ACPI AML code for _ROM method.
 * This function takes as input ROM data and ROM length.
 * The ROM length has to be multiple of 4096 and has to be less
 * than the current implementation limit of 0x40000.
 */
int acpigen_write_rom(struct acpi_ctx *ctx, void *bios, const size_t length);
/*
 * Generate ACPI AML code for OperationRegion
 * This function takes input region name, region space, region offset & region
 * length.
 */
void acpigen_write_opregion(struct acpi_ctx *ctx, struct opregion *opreg);
/*
 * Generate ACPI AML code for Field
 * This function takes input region name, fieldlist, count & flags.
 */
int acpigen_write_field(struct acpi_ctx *ctx, const char *name,
			struct fieldlist *l, size_t count, uint flags);
/*
 * Generate ACPI AML code for IndexField
 * This function takes input index name, data name, fieldlist, count & flags.
 */
int acpigen_write_indexfield(struct acpi_ctx *ctx, const char *idx, const char *data,
			     struct fieldlist *l, size_t count, uint flags);

/*
 * Soc-implemented functions for generating ACPI AML code for GPIO handling. All
 * these functions are expected to use only Local5, Local6 and Local7
 * variables. If the functions call into another ACPI method, then there is no
 * restriction on the use of Local variables. In case of get/read functions,
 * return value is expected to be stored in Local0 variable.
 *
 * All functions return 0 on success and -1 on error.
 */

/* Generate ACPI AML code to return Rx value of GPIO in Local0. */
int acpigen_soc_read_rx_gpio(struct acpi_ctx *ctx, uint gpio_num);

/* Generate ACPI AML code to return Tx value of GPIO in Local0. */
int acpigen_soc_get_tx_gpio(struct acpi_ctx *ctx, uint gpio_num);

/* Generate ACPI AML code to set Tx value of GPIO to 1. */
int acpigen_soc_set_tx_gpio(struct acpi_ctx *ctx, uint gpio_num);

/* Generate ACPI AML code to set Tx value of GPIO to 0. */
int acpigen_soc_clear_tx_gpio(struct acpi_ctx *ctx, uint gpio_num);

/*
 * Helper functions for enabling/disabling Tx GPIOs based on the GPIO
 * polarity. These functions end up calling acpigen_soc_{set,clear}_tx_gpio to
 * make callbacks into SoC acpigen code.
 *
 * Returns 0 on success and -1 on error.
 */
int acpigen_enable_tx_gpio(struct acpi_ctx *ctx, struct acpi_gpio *gpio);
int acpigen_disable_tx_gpio(struct acpi_ctx *ctx, struct acpi_gpio *gpio);

#endif

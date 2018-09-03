// SPDX-License-Identifier: GPL-2.0+
/*
 * Taken from depthcharge file of the same name
 *
 * TODO(sjg@chromium.org): Implement altfw & clean up this code
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <backlight.h>
#include <dm.h>
#include <panel.h>
#include <video.h>
#include <video_console.h>
#include <cros/cb_archive.h>
#include <cros/cb_gfx.h>
#include <cros/fwstore.h>
#include <cros/screens.h>
#include <cros/vboot.h>

#include <gbb_header.h>

/*
 * This is the base used to specify the size and the coordinate of the image.
 * For example, height = 40 means 4.0% of the canvas (=drawing area) height.
 */
#define VB_SCALE		1000		/* 100.0% */
#define VB_SCALE_HALF		(VB_SCALE / 2)	/* 50.0% */

/* Height of the text image per line relative to the canvas size */
#define VB_TEXT_HEIGHT		36	/* 3.6% */

/* Chrome logo size and distance from the divider */
#define VB_LOGO_HEIGHT		39	/* 3.9% */
#define VB_LOGO_LIFTUP		0

/* Indicate width or height is automatically set based on the other value */
#define VB_SIZE_AUTO		0

/* Height of the icons relative to the canvas size */
#define VB_ICON_HEIGHT		169	/* 16.9% */

/* Height of InsertDevices, RemoveDevices */
#define VB_DEVICE_HEIGHT	371	/* 37.1% */

/* Vertical position and size of the dividers */
#define VB_DIVIDER_WIDTH	900	/* 90.0% -> 5% padding on each side */
#define VB_DIVIDER_V_OFFSET	160	/* 16.0% */

/* Space between sections of text */
#define VB_PADDING		3	/* 0.3 % */

/* Downshift for vertical characters to match middle of text in Noto Sans */
#define VB_ARROW_V_OFF		3	/* 0.3 % */

#define RET_ON_ERR(function_call) do {					\
		VbError_t rv = (function_call);				\
		if (rv)							\
			return rv;					\
	} while (0)

static bool initialised;
static int  prev_lang_page_num = -1;
static int  prev_selected_index = -1;
static struct directory *base_graphics;
static struct directory *font_graphics;
static struct {
	/* current locale */
	u32 current;

	/* pointer to the localised graphics data and its locale */
	u32 archive_locale;
	struct directory *archive;

	/* number of supported language and codes: en, ja, .. */
	u32 count;
	char *codes[256];
} locale_data;

/* params structure for vboot draw functions */
struct params {
	u32 locale;
	u32 selected_index;
	u32 disabled_idx_mask;
	u32 redraw_base;
};

/* struct for passing around menu string arrays */
struct menu {
	const char *const *strings;
	u32 count;
};

static int load_archive(const char *str, struct directory **dest)
{
	struct vboot_info *vboot = vboot_get();
	struct fmap_entry fentry;
	struct directory *dir;
	struct dentry *entry;
	int size, ret, i;
	u8 *data;

	ret = cros_ofnode_find_locale(str, &fentry);
	if (ret) {
		log_err("Cannot read file '%s'\n", str);
		return VBERROR_INVALID_BMPFV;
	}

	ret = fwstore_load_image(vboot->fwstore, &fentry, &data, &size);

	/* convert endianness of archive header */
	dir = (struct directory *)data;
	dir->count = le32_to_cpu(dir->count);
	dir->size = le32_to_cpu(dir->size);

	/* validate the total size */
	if (dir->size != size) {
		log_err("archive size %x does not match region size %x\n",
			dir->size, size);
		return VBERROR_INVALID_BMPFV;
	}

	/* validate magic field */
	if (memcmp(dir->magic, CBAR_MAGIC, sizeof(CBAR_MAGIC))) {
		printf("%s: invalid archive magic\n", __func__);
		return VBERROR_INVALID_BMPFV;
	}

	/* validate count field */
	if (get_first_offset(dir) > dir->size) {
		printf("%s: invalid count\n", __func__);
		return VBERROR_INVALID_BMPFV;
	}

	/* convert endianness of file headers */
	entry = get_first_dentry(dir);
	for (i = 0; i < dir->count; i++) {
		entry[i].offset = le32_to_cpu(entry[i].offset);
		entry[i].size = le32_to_cpu(entry[i].size);
	}

	*dest = dir;

	return VBERROR_SUCCESS;
}

static VbError_t load_localised_graphics(u32 locale)
{
	char str[256];
	int ret;

	/* check whether we've already loaded the archive for this locale */
	if (locale_data.archive) {
		if (locale_data.archive_locale == locale)
			return VBERROR_SUCCESS;
		/* No need to keep more than one locale graphics at a time */
		free(locale_data.archive);
	}

	/* compose archive name using the language code */
	snprintf(str, sizeof(str), "locale_%s.bin", locale_data.codes[locale]);
	ret = load_archive(str, &locale_data.archive);
	if (ret) {
		log_err("Cannot read locale '%s'\n", str);
		return VBERROR_INVALID_BMPFV;
	}

	/* Remember what's cached */
	locale_data.archive_locale = locale;

	return VBERROR_SUCCESS;
}

static struct dentry *find_file_in_archive(const struct directory *dir,
					   const char *name)
{
	struct dentry *entry;
	uintptr_t start;
	int i;

	if (!dir) {
		printf("%s: archive not loaded\n", __func__);
		return NULL;
	}

	/* calculate start of the file content section */
	start = get_first_offset(dir);
	entry = get_first_dentry(dir);
	for (i = 0; i < dir->count; i++) {
		if (strncmp(entry[i].name, name, NAME_LENGTH))
			continue;
		/* validate offset & size */
		if (entry[i].offset < start ||
		    entry[i].offset + entry[i].size > dir->size ||
		    entry[i].offset > dir->size ||
		    entry[i].size > dir->size) {
			printf("%s: '%s' has invalid offset or size\n",
			       __func__, name);
			return NULL;
		}
		return &entry[i];
	}

	printf("%s: file '%s' not found\n", __func__, name);

	return NULL;
}

/*
 * Find and draw image in archive
 */
static VbError_t draw(struct directory *dir, const char *image_name,
		      s32 x, s32 y, s32 width, s32 height, u32 flags)
{
	struct dentry *file;
	void *bitmap;

	file = find_file_in_archive(dir, image_name);
	if (!file)
		return VBERROR_NO_IMAGE_PRESENT;
	bitmap = (u8 *)dir + file->offset;

	struct scale pos = {
		.x = { .n = x, .d = VB_SCALE, },
		.y = { .n = y, .d = VB_SCALE, },
	};
	struct scale dim = {
		.x = { .n = width, .d = VB_SCALE, },
		.y = { .n = height, .d = VB_SCALE, },
	};

	if (cbgfx_get_bitmap_dimension(bitmap, file->size, &dim))
		return VBERROR_UNKNOWN;

	if ((int64_t)dim.x.n * VB_SCALE <= (int64_t)dim.x.d * VB_DIVIDER_WIDTH)
		return cbgfx_draw_bitmap((u8 *)dir + file->offset, file->size,
				   &pos, &dim, flags);

	/*
	 * If we get here the image is too wide, so fit it to the content width.
	 * This only works if it is horizontally centered (x == VB_SCALE_HALF
	 * and flags & PIVOT_H_CENTER), but that applies to our current stuff
	 * which might be too wide (locale-dependent strings). Only exception is
	 * the "For help" footer, which was already fitted in its own function.
	 */
	printf("vbgfx: '%s' too wide, fitting to content width\n", image_name);
	dim.x.n = VB_DIVIDER_WIDTH;
	dim.x.d = VB_SCALE;
	dim.y.n = VB_SIZE_AUTO;
	dim.y.d = VB_SCALE;
	return cbgfx_draw_bitmap((u8 *)dir + file->offset, file->size,
			   &pos, &dim, flags);
}

static VbError_t draw_image(const char *image_name,
			    s32 x, s32 y, s32 width, s32 height,
			    u32 pivot)
{
	return draw(base_graphics, image_name, x, y, width, height, pivot);
}

static VbError_t draw_image_locale(const char *image_name, u32 locale,
				   s32 x, s32 y, s32 w, s32 h,
				   u32 flags)
{
	RET_ON_ERR(load_localised_graphics(locale));
	return draw(locale_data.archive, image_name, x, y, w, h, flags);
}

static VbError_t get_image_size(struct directory *dir, const char *image_name,
				s32 *width, s32 *height)
{
	struct dentry *file;
	VbError_t rv;

	file = find_file_in_archive(dir, image_name);
	if (!file)
		return VBERROR_NO_IMAGE_PRESENT;

	struct scale dim = {
		.x = { .n = *width, .d = VB_SCALE, },
		.y = { .n = *height, .d = VB_SCALE, },
	};

	rv = cbgfx_get_bitmap_dimension((u8 *)dir + file->offset, file->size,
					&dim);
	if (rv)
		return VBERROR_UNKNOWN;

	*width = dim.x.n * VB_SCALE / dim.x.d;
	*height = dim.y.n * VB_SCALE / dim.y.d;

	return VBERROR_SUCCESS;
}

static VbError_t get_image_size_locale(const char *image_name, u32 locale,
				       s32 *width, s32 *height)
{
	RET_ON_ERR(load_localised_graphics(locale));
	return get_image_size(locale_data.archive, image_name, width, height);
}

static int draw_icon(const char *image_name)
{
	return draw_image(image_name,
			  VB_SCALE_HALF, VB_SCALE_HALF,
			  VB_SIZE_AUTO, VB_ICON_HEIGHT,
			  PIVOT_H_CENTER | PIVOT_V_BOTTOM);
}

static int draw_text(const char *text, s32 x, s32 y,
		     s32 height, char pivot)
{
	s32 w, h;
	char str[256];

	while (*text) {
		sprintf(str, "idx%03d_%02x.bmp", *text, *text);
		w = 0;
		h = height;
		RET_ON_ERR(get_image_size(font_graphics, str, &w, &h));
		RET_ON_ERR(draw(font_graphics, str, x, y, VB_SIZE_AUTO, height,
				pivot));
		x += w;
		text++;
	}
	return VBERROR_SUCCESS;
}

static int get_text_width(const char *text, s32 *width, s32 *height)
{
	s32 w, h;
	char str[256];

	if (!strcmp(text, "None"))
		log_info("bad text\n");
	while (*text) {
		sprintf(str, "idx%03d_%02x.bmp", *text, *text);
		w = 0;
		h = *height;
		RET_ON_ERR(get_image_size(font_graphics, str, &w, &h));
		*width += w;
		text++;
	}
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_footer(struct vboot_info *vboot, u32 locale)
{
	GoogleBinaryBlockHeader *gbb;
	char *hwid = NULL;
	s32 x, y, w1, h1, w2, h2, w3, h3;
	s32 total;

	/*
	 * Draw help URL line: 'For help visit http://.../'. It consists of
	 * three parts: [for_help_left.bmp][URL][for_help_right.bmp].
	 * Since the widths vary, we need to get the widths first then calculate
	 * the horizontal positions of the images.
	 */
	w1 = VB_SIZE_AUTO;
	h1 = VB_TEXT_HEIGHT;
	/* Expected to fail in locales which don't have left part */
	get_image_size_locale("for_help_left.bmp", locale, &w1, &h1);

	w2 = VB_SIZE_AUTO;
	h2 = VB_TEXT_HEIGHT;
	RET_ON_ERR(get_image_size(base_graphics, "Url.bmp", &w2, &h2));

	w3 = VB_SIZE_AUTO;
	h3 = VB_TEXT_HEIGHT;
	/* Expected to fail in locales which don't have right part */
	get_image_size_locale("for_help_right.bmp", locale, &w3, &h3);

	total = w1 + VB_PADDING + w2 + VB_PADDING + w3;
	y = VB_SCALE - VB_DIVIDER_V_OFFSET;
	if (VB_DIVIDER_WIDTH - total >= 0) {
		/* Calculate position to centralise the images combined */
		x = (VB_SCALE - total) / 2;
		/* Expected to fail in locales which don't have left part */
		draw_image_locale("for_help_left.bmp", locale,
				  x, y, VB_SIZE_AUTO, VB_TEXT_HEIGHT,
				  PIVOT_H_LEFT | PIVOT_V_TOP);
		x += w1 + VB_PADDING;
		RET_ON_ERR(draw_image("Url.bmp", x, y, VB_SIZE_AUTO,
				      VB_TEXT_HEIGHT,
				      PIVOT_H_LEFT | PIVOT_V_TOP));
		x += w2 + VB_PADDING;
		/* Expected to fail in locales which don't have right part */
		draw_image_locale("for_help_right.bmp", locale,
				  x, y, VB_SIZE_AUTO, VB_TEXT_HEIGHT,
				  PIVOT_H_LEFT | PIVOT_V_TOP);
	} else {
		s32 pad;
		/* images are too wide. need to fit them to content width */
		printf("%s: help line overflowed. fit it to content width\n",
		       __func__);
		x = (VB_SCALE - VB_DIVIDER_WIDTH) / 2;
		/* Shrink all images */
		w1 = VB_DIVIDER_WIDTH * w1 / total;
		w2 = VB_DIVIDER_WIDTH * w2 / total;
		w3 = VB_DIVIDER_WIDTH * w3 / total;
		pad = VB_DIVIDER_WIDTH * VB_PADDING / total;

		/* Render using width as a base */
		draw_image_locale("for_help_left.bmp", locale,
				  x, y, w1, VB_SIZE_AUTO,
				  PIVOT_H_LEFT | PIVOT_V_TOP);
		x += w1 + pad;
		RET_ON_ERR(draw_image("Url.bmp", x, y, w2, VB_SIZE_AUTO,
				      PIVOT_H_LEFT | PIVOT_V_TOP));
		x += w2 + pad;
		draw_image_locale("for_help_right.bmp", locale,
				  x, y, w3, VB_SIZE_AUTO,
				  PIVOT_H_LEFT | PIVOT_V_TOP);
	}

	/*
	 * Draw model line: 'Model XYZ'. It consists of two parts: 'Model',
	 * which is locale dependent, and 'XYZ', a model name. Model name
	 * consists of individual font images: 'X' 'Y' 'Z'.
	 */
	gbb = vboot->cparams.gbb_data;
	if (gbb)
		hwid = (char *)((uintptr_t)gbb + gbb->hwid_offset);
	else
		hwid = "NOT FOUND";

	w1 = VB_SIZE_AUTO;
	h1 = VB_TEXT_HEIGHT;
	get_image_size_locale("model_left.bmp", locale, &w1, &h1);
	w1 += VB_PADDING;

	w2 = VB_SIZE_AUTO;
	h2 = VB_TEXT_HEIGHT;
	RET_ON_ERR(get_text_width(hwid, &w2, &h2));
	w2 += VB_PADDING;

	w3 = VB_SIZE_AUTO;
	h3 = VB_TEXT_HEIGHT;
	get_image_size_locale("model_right.bmp", locale, &w3, &h3);

	/* Calculate horizontal position to centralise the combined images */
	/*
	 * No clever way to redraw the combined images when they overflow but
	 * luckily there is plenty of space for just 'model' + model name.
	 */
	x = (VB_SCALE - w1 - w2 - w3) / 2;
	y += VB_TEXT_HEIGHT;
	draw_image_locale("model_left.bmp", locale,
			  x, y, VB_SIZE_AUTO, VB_TEXT_HEIGHT,
			  PIVOT_H_LEFT | PIVOT_V_TOP);
	x += w1;
	RET_ON_ERR(draw_text(hwid, x, y, VB_TEXT_HEIGHT,
			     PIVOT_H_LEFT | PIVOT_V_TOP));
	x += w2;
	draw_image_locale("model_right.bmp", locale,
			  x, y, VB_SIZE_AUTO, VB_TEXT_HEIGHT,
			  PIVOT_H_LEFT | PIVOT_V_TOP);

	return VBERROR_SUCCESS;
}

/*
 * Draws the language section at the top right corner. The language text image
 * is placed in the middle surrounded by arrows on each side.
 */
static VbError_t vboot_draw_language(struct vboot_info *vboot, u32 locale)
{
	s32 w, h, x;

	/*
	 * Right arrow starts from the right edge of the divider, which is
	 * positioned horizontally in the center.
	 */
	x = VB_SCALE_HALF + VB_DIVIDER_WIDTH / 2;

	/* Draw right arrow */
	if (vboot->detachable_ui) {
		w = VB_SIZE_AUTO;
		h = VB_TEXT_HEIGHT;
		RET_ON_ERR(draw_image("arrow_right.bmp", x,
				      VB_DIVIDER_V_OFFSET + VB_ARROW_V_OFF,
				      w, h, PIVOT_H_RIGHT | PIVOT_V_BOTTOM));
		RET_ON_ERR(get_image_size(base_graphics, "arrow_right.bmp",
					  &w, &h));
		x -= w + VB_PADDING;
	}

	/* Draw language name */
	w = VB_SIZE_AUTO;
	h = VB_TEXT_HEIGHT;
	RET_ON_ERR(draw_image_locale("language.bmp", locale, x,
				     VB_DIVIDER_V_OFFSET, w, h,
				     PIVOT_H_RIGHT | PIVOT_V_BOTTOM));
	RET_ON_ERR(get_image_size_locale("language.bmp", locale, &w, &h));

	if (vboot->detachable_ui) {
		x -= w + VB_PADDING;

		/* Draw left arrow */
		w = VB_SIZE_AUTO;
		h = VB_TEXT_HEIGHT;
		RET_ON_ERR(draw_image("arrow_left.bmp", x,
				      VB_DIVIDER_V_OFFSET + VB_ARROW_V_OFF,
				      w, h, PIVOT_H_RIGHT | PIVOT_V_BOTTOM));
	}

	return VBERROR_SUCCESS;
}

static VbError_t draw_base_screen(struct vboot_info *vboot, u32 locale,
				  int show_language)
{
	const struct rgb_colour white = { 0xff, 0xff, 0xff };

	if (cbgfx_clear_screen(&white))
		return VBERROR_UNKNOWN;
	RET_ON_ERR(draw_image("chrome_logo.bmp",
			      (VB_SCALE - VB_DIVIDER_WIDTH) / 2,
			      VB_DIVIDER_V_OFFSET - VB_LOGO_LIFTUP,
			      VB_SIZE_AUTO, VB_LOGO_HEIGHT,
			      PIVOT_H_LEFT | PIVOT_V_BOTTOM));

	if (show_language)
		RET_ON_ERR(vboot_draw_language(vboot, locale));

	RET_ON_ERR(draw_image("divider_top.bmp", VB_SCALE_HALF,
			      VB_DIVIDER_V_OFFSET, VB_DIVIDER_WIDTH,
			      VB_SIZE_AUTO, PIVOT_H_CENTER | PIVOT_V_TOP));
	RET_ON_ERR(draw_image("divider_btm.bmp", VB_SCALE_HALF,
			      VB_SCALE - VB_DIVIDER_V_OFFSET,
			      VB_DIVIDER_WIDTH, VB_SIZE_AUTO,
			      PIVOT_H_CENTER | PIVOT_V_BOTTOM));

	RET_ON_ERR(vboot_draw_footer(vboot, locale));

	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_base_screen(struct vboot_info *vboot,
					struct params *p)
{
	return draw_base_screen(vboot, p->locale, 1);
}

static VbError_t vboot_draw_base_screen_without_language
	(struct vboot_info *vboot, struct params *p)
{
	return draw_base_screen(vboot, p->locale, 0);
}

static VbError_t vboot_draw_blank(struct vboot_info *vboot, struct params *p)
{
	video_clear(vboot->video);

	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_menu(struct params *p, const struct menu *m)
{
	int i = 0;
	int yoffset;
	u32 flags;

	/* find starting point y offset */
	yoffset = 0 - m->count / 2;
	for (i = 0; i < m->count; i++) {
		if ((p->disabled_idx_mask & (1 << i)) != 0)
			continue;
		flags = PIVOT_H_CENTER | PIVOT_V_TOP;
		if (p->selected_index == i)
			flags |= INVERT_COLOURS;
		RET_ON_ERR(draw_image_locale(m->strings[i], p->locale,
					     VB_SCALE_HALF, VB_SCALE_HALF +
					     VB_TEXT_HEIGHT * yoffset,
					     VB_SIZE_AUTO, VB_TEXT_HEIGHT,
					     flags));
		if (!strncmp(m->strings[i], "lang.bmp", NAME_LENGTH)) {
			s32 w = VB_SIZE_AUTO, h = VB_TEXT_HEIGHT;

			RET_ON_ERR(get_image_size_locale(m->strings[i],
							 p->locale, &w, &h));
			RET_ON_ERR(draw_image("globe.bmp",
					      VB_SCALE_HALF + w / 2,
					      VB_SCALE_HALF +
					      VB_TEXT_HEIGHT * yoffset,
					      VB_SIZE_AUTO, VB_TEXT_HEIGHT,
					      PIVOT_H_LEFT | PIVOT_V_TOP));
		}
		yoffset++;
	}

	RET_ON_ERR(draw_image_locale("navigate.bmp", p->locale,
				     VB_SCALE_HALF, VB_SCALE -
				     VB_DIVIDER_V_OFFSET - VB_TEXT_HEIGHT,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_BOTTOM));

	return VBERROR_SUCCESS;
}

/* String arrays with bmp file names for detachable Menus */
static const char *const dev_warning_menu_files[] = {
	"dev_option.bmp", /* Developer Options */
	"debug_info.bmp", /* Show Debug Info */
	"enable_ver.bmp", /* Enable Root Verification */
	"power_off.bmp",  /* Power Off */
	"lang.bmp",       /* Language */
};

static const char *const dev_menu_files[] = {
	"boot_network.bmp", /* Boot Network Image */
	"boot_legacy.bmp",  /* Boot Legacy BIOS */
	"boot_usb.bmp",     /* Boot USB Image */
	"boot_dev.bmp",     /* Boot Developer Image */
	"cancel.bmp",       /* Cancel */
	"power_off.bmp",    /* Power Off */
	"lang.bmp",         /* Language */
};

static const char *const rec_to_dev_files[] = {
	"confirm_dev.bmp", /* Confirm enabling developer mode */
	"cancel.bmp",      /* Cancel */
	"power_off.bmp",   /* Power Off */
	"lang.bmp",        /* Language */
};

static const char *const dev_to_norm_files[] = {
	"confirm_ver.bmp", /* Confirm Enabling Verified Boot */
	"cancel.bmp",      /* Cancel */
	"power_off.bmp",   /* Power Off */
	"lang.bmp",        /* Language */
};

static const char *const options_files[] = {
	"debug_info.bmp",  /* Show Debug Info */
	"cancel.bmp",      /* Cancel */
	"power_off.bmp",   /* Power Off */
	"lang.bmp",        /* Language */
};

static VbError_t vboot_draw_developer_warning(struct vboot_info *vboot,
					      struct params *p)
{
	u32 locale = p->locale;

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_icon("VerificationOff.bmp"));
	RET_ON_ERR(draw_image_locale("verif_off.bmp", locale,
				     VB_SCALE_HALF, VB_SCALE_HALF,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	RET_ON_ERR(draw_image_locale("devmode.bmp", locale,
				     VB_SCALE_HALF, VB_SCALE_HALF +
				     VB_TEXT_HEIGHT * 2, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT,
				     PIVOT_H_CENTER | PIVOT_V_TOP));

	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_developer_warning_menu(struct vboot_info *vboot,
						   struct params *p)
{
	if (p->redraw_base)
		RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_image_locale("enable_hint.bmp", p->locale,
				     VB_SCALE_HALF,
				     VB_DIVIDER_V_OFFSET + VB_TEXT_HEIGHT,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	const struct menu m = { dev_warning_menu_files,
				ARRAY_SIZE(dev_warning_menu_files) };

	return vboot_draw_menu(p, &m);
}

static VbError_t vboot_draw_developer_menu(struct vboot_info *vboot,
					   struct params *p)
{
	if (p->redraw_base)
		RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	const struct menu m = { dev_menu_files, ARRAY_SIZE(dev_menu_files) };

	return vboot_draw_menu(p, &m);
}

static VbError_t vboot_draw_recovery_no_good(struct vboot_info *vboot,
					     struct params *p)
{
	u32 locale = p->locale;

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_image_locale("yuck.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF - VB_DEVICE_HEIGHT / 2,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT,
				     PIVOT_H_CENTER | PIVOT_V_BOTTOM));
	RET_ON_ERR(draw_image("BadDevices.bmp", VB_SCALE_HALF, VB_SCALE_HALF,
			      VB_SIZE_AUTO, VB_ICON_HEIGHT,
			      PIVOT_H_CENTER | PIVOT_V_CENTER));
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_recovery_insert(struct vboot_info *vboot,
					    struct params *p)
{
	const s32 h = VB_DEVICE_HEIGHT;

	u32 locale = p->locale;

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_image_locale("insert.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF - h / 2, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT,
				     PIVOT_H_CENTER | PIVOT_V_BOTTOM));
	RET_ON_ERR(draw_image("InsertDevices.bmp", VB_SCALE_HALF, VB_SCALE_HALF,
			      VB_SIZE_AUTO, h,
			      PIVOT_H_CENTER | PIVOT_V_CENTER));
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_recovery_to_dev(struct vboot_info *vboot,
					    struct params *p)
{
	u32 locale = p->locale;

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_image_locale("todev.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT * 4,
				     PIVOT_H_CENTER | PIVOT_V_CENTER));
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_recovery_to_dev_menu(struct vboot_info *vboot,
						 struct params *p)
{
	if (p->redraw_base)
		RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_image_locale("disable_warn.bmp", p->locale,
				     VB_SCALE_HALF,
				     VB_DIVIDER_V_OFFSET + VB_TEXT_HEIGHT,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	const struct menu m = { rec_to_dev_files,
				ARRAY_SIZE(rec_to_dev_files) };
	return vboot_draw_menu(p, &m);
}

static VbError_t vboot_draw_developer_to_norm(struct vboot_info *vboot,
					      struct params *p)
{
	u32 locale = p->locale;

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_icon("VerificationOff.bmp"));
	RET_ON_ERR(draw_image_locale("verif_off.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	RET_ON_ERR(draw_image_locale("tonorm.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF + VB_TEXT_HEIGHT * 2,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT * 4,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_developer_to_norm_menu(struct vboot_info *vboot,
						   struct params *p)
{
	if (p->redraw_base)
		RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_image_locale("confirm_hint.bmp", p->locale,
				     VB_SCALE_HALF,
				     VB_DIVIDER_V_OFFSET + VB_TEXT_HEIGHT,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	const struct menu m = { dev_to_norm_files,
				ARRAY_SIZE(dev_to_norm_files) };
	return vboot_draw_menu(p, &m);
}

static VbError_t vboot_draw_wait(struct vboot_info *vboot, struct params *p)
{
	/*
	 * Currently, language cannot be changed while EC software sync is
	 * taking place because keyboard is disabled.
	 */
	RET_ON_ERR(vboot_draw_base_screen_without_language(vboot, p));
	RET_ON_ERR(draw_image_locale("update.bmp", p->locale, VB_SCALE_HALF,
				     VB_SCALE_HALF, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_CENTER));
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_to_norm_confirmed(struct vboot_info *vboot,
					      struct params *p)
{
	u32 locale = p->locale;

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_icon("VerificationOn.bmp"));
	RET_ON_ERR(draw_image_locale("verif_on.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	RET_ON_ERR(draw_image_locale("reboot_erase.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF + VB_TEXT_HEIGHT * 2,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_os_broken(struct vboot_info *vboot,
				      struct params *p)
{
	u32 locale = p->locale;

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_icon("Warning.bmp"));
	RET_ON_ERR(draw_image_locale("os_broken.bmp", locale, VB_SCALE_HALF,
				     VB_SCALE_HALF, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_TOP));
	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_languages_menu(struct vboot_info *vboot,
					   struct params *p)
{
	int i = 0;

	/*
	 * There are too many languages to fit onto a page.  Let's try to list
	 * about 15 at a time. Since the explanatory text needs to fit on the
	 * bottom, center the list two entries higher than the screen center.
	 */
	const int lang_per_page = 15;
	const int yoffset_start = 0 - lang_per_page / 2 - 2;
	int yoffset = yoffset_start;
	int selected_index = p->selected_index % locale_data.count;

	locale_data.current = selected_index;

	int page_num = selected_index / lang_per_page;
	int page_start_index = lang_per_page * page_num;
	int total_pages = locale_data.count / lang_per_page;

	if (locale_data.count % lang_per_page > 0)
		total_pages++;

	/*
	 * redraw screen if we cross a page boundary
	 * or if we're instructed to do so (because of screen change)
	 */
	if (prev_lang_page_num != page_num || p->redraw_base)
		RET_ON_ERR(vboot_draw_base_screen(vboot, p));

	/* Print out page #s (1/5, 2/5, etc.) */
	char page_count[6];

	snprintf(page_count, sizeof(page_count), "%d/%d", page_num + 1,
		 total_pages);
	/* draw_text() cannot pivot center, so must fudge x-coord a little */
	RET_ON_ERR(draw_text(page_count, VB_SCALE_HALF - 20,
			     VB_DIVIDER_V_OFFSET, VB_TEXT_HEIGHT,
			     PIVOT_H_LEFT | PIVOT_V_BOTTOM));

	/*
	 * Check if we can just redraw some entries (staying on the
	 * same page) instead of the whole page because opening the
	 * archives for each language slows things down.
	 */
	int num_lang_to_draw = lang_per_page;
	int start_index = page_start_index;

	if (prev_lang_page_num == page_num && !p->redraw_base) {
		/* Redraw selected index and previously selected index */
		num_lang_to_draw = 2;
		start_index = min(prev_selected_index, selected_index);
		/* previous index invalid */
		if (prev_selected_index == -1) {
			start_index = selected_index;
			num_lang_to_draw = 1;
		}
		yoffset = yoffset_start + (start_index - page_start_index);
	}

	u32 flags;

	for (i = start_index;
	     i < start_index + num_lang_to_draw && i < locale_data.count;
	     i++, yoffset++) {
		flags = PIVOT_H_CENTER | PIVOT_V_TOP;
		if (selected_index == i)
			flags |= INVERT_COLOURS;
		RET_ON_ERR(draw_image_locale("language.bmp", i, VB_SCALE_HALF,
					     VB_SCALE_HALF +
					     VB_TEXT_HEIGHT * yoffset,
					     VB_SIZE_AUTO, VB_TEXT_HEIGHT,
					     flags));
	}
	prev_lang_page_num = page_num;
	prev_selected_index = selected_index;

	RET_ON_ERR(draw_image_locale("navigate.bmp", p->locale, VB_SCALE_HALF,
				     VB_SCALE - VB_DIVIDER_V_OFFSET -
				     VB_TEXT_HEIGHT, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_BOTTOM));

	return VBERROR_SUCCESS;
}

static void cons_string(struct udevice *cons, const char *str)
{
	while (*str)
		vidconsole_put_char(cons, *str++);
}

static void cons_text(struct vboot_info *vboot, int linenum, int seqnum,
		      const char *name, const char *desc)
{
	struct vidconsole_priv *uc_priv = dev_get_uclass_priv(vboot->console);
	char seq[2] = {'0' + seqnum, '\0'};
	int x, y;

	x = uc_priv->cols / 3;
	y = uc_priv->rows / 2 + linenum;
	vidconsole_position_cursor(vboot->console, x, y);
	if (seqnum != -1)
		cons_string(vboot->console, seq);

	vidconsole_position_cursor(vboot->console, x + 3, y);
	cons_string(vboot->console, name);

	vidconsole_position_cursor(vboot->console, x + 10, y);
	cons_string(vboot->console, desc);
}

static VbError_t vboot_draw_altfw_pick(struct vboot_info *vboot,
				       struct params *p)
{
	char msg[60];

	RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	RET_ON_ERR(draw_icon("VerificationOff.bmp"));
	sprintf(msg, "Press key 1-%c to select alternative boot loader:", '2');
	cons_text(vboot, 0, -1, msg, "");

	return VBERROR_SUCCESS;
}

static VbError_t vboot_draw_options_menu(struct vboot_info *vboot,
					 struct params *p)
{
	if (p->redraw_base)
		RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	const struct menu m = { options_files,
				ARRAY_SIZE(options_files) };

				return vboot_draw_menu(p, &m);
}

static VbError_t vboot_draw_altfw_menu(struct vboot_info *vboot,
				       struct params *p)
{
	int i;

	if (p->redraw_base)
		RET_ON_ERR(vboot_draw_base_screen(vboot, p));
	int yoffset = 0;
	u32 flags;

	i = 2;
	yoffset = 0;
	flags = PIVOT_H_CENTER | PIVOT_V_TOP;
	if (p->selected_index == i)
		flags |= INVERT_COLOURS;
	RET_ON_ERR(draw_image_locale("cancel.bmp", p->locale,
				     VB_SCALE_HALF,
				     VB_SCALE_HALF + VB_TEXT_HEIGHT * yoffset,
				     VB_SIZE_AUTO, VB_TEXT_HEIGHT, flags));

	RET_ON_ERR(draw_image_locale("navigate.bmp", p->locale, VB_SCALE_HALF,
				     VB_SCALE - VB_DIVIDER_V_OFFSET -
				     VB_TEXT_HEIGHT, VB_SIZE_AUTO,
				     VB_TEXT_HEIGHT * 2,
				     PIVOT_H_CENTER | PIVOT_V_BOTTOM));

	return 0;
}

/* we may export this in the future for the board customization */
struct vboot_ui_descriptor {
	u32 id;				/* VB_SCREEN_* */
	/* draw function */
	VbError_t (*draw)(struct vboot_info *vboot, struct params *p);
	const char *mesg;			/* fallback message */
};

static const struct vboot_ui_descriptor vboot_screens[] = {
	{
		.id = VB_SCREEN_BLANK,
		.draw = vboot_draw_blank,
		.mesg = NULL,
	},
	{
		.id = VB_SCREEN_DEVELOPER_WARNING,
		.draw = vboot_draw_developer_warning,
		.mesg = "OS verification is OFF\n"
			"Press SPACE to re-enable.\n",
	},
	{
		.id = VB_SCREEN_RECOVERY_NO_GOOD,
		.draw = vboot_draw_recovery_no_good,
		.mesg = "The device you inserted does not contain Chromium OS.\n",
	},
	{
		.id = VB_SCREEN_RECOVERY_INSERT,
		.draw = vboot_draw_recovery_insert,
		.mesg = "Chromium OS is missing or damaged.\n"
			"Please insert a recovery USB stick or SD card.\n",
	},
	{
		.id = VB_SCREEN_RECOVERY_TO_DEV,
		.draw = vboot_draw_recovery_to_dev,
		.mesg = "To turn OS verificaion OFF, press ENTER.\n"
			"Your system will reboot and local data will be cleared.\n"
			"To go back, press ESC.\n",
	},
	{
		.id = VB_SCREEN_DEVELOPER_TO_NORM,
		.draw = vboot_draw_developer_to_norm,
		.mesg = "OS verification is OFF\n"
			"Press ENTER to confirm you wish to turn OS verification on.\n"
			"Your system will reboot and local data will be cleared.\n"
			"To go back, press ESC.\n",
	},
	{
		.id = VB_SCREEN_WAIT,
		.draw = vboot_draw_wait,
		.mesg = "Your system is applying a critical update.\n"
			"Please do not turn off.\n",
	},
	{
		.id = VB_SCREEN_TO_NORM_CONFIRMED,
		.draw = vboot_draw_to_norm_confirmed,
		.mesg = "OS verification is ON\n"
			"Your system will reboot and local data will be cleared.\n",
	},
	{
		.id = VB_SCREEN_OS_BROKEN,
		.draw = vboot_draw_os_broken,
		.mesg = "Chromium OS may be broken.\n"
			"Remove media and initiate recovery.\n",
	},
	{
		.id = VB_SCREEN_DEVELOPER_WARNING_MENU,
		.draw = vboot_draw_developer_warning_menu,
		.mesg = "Developer Warning Menu\n",
	},
	{
		.id = VB_SCREEN_DEVELOPER_MENU,
		.draw = vboot_draw_developer_menu,
		.mesg = "Developer Menu\n",
	},
	{
		.id = VB_SCREEN_RECOVERY_TO_DEV_MENU,
		.draw = vboot_draw_recovery_to_dev_menu,
		.mesg = "Recovery to Dev Menu\n",
	},
	{
		.id = VB_SCREEN_DEVELOPER_TO_NORM_MENU,
		.draw = vboot_draw_developer_to_norm_menu,
		.mesg = "Developer to Norm Menu",
	},
	{
		.id = VB_SCREEN_LANGUAGES_MENU,
		.draw = vboot_draw_languages_menu,
		.mesg = "Languages Menu",
	},
	{
		.id = VB_SCREEN_OPTIONS_MENU,
		.draw = vboot_draw_options_menu,
		.mesg = "Options Menu",
	},
	{
		.id = VB_SCREEN_ALT_FW_PICK,
		.draw = vboot_draw_altfw_pick,
		.mesg = "Alternative Firmware Menu",
	},
	{
		.id = VB_SCREEN_ALT_FW_MENU,
		.draw = vboot_draw_altfw_menu,
		.mesg = "Alternative Firmware Menu",
	},
};

static const struct vboot_ui_descriptor *get_ui_descriptor(u32 id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vboot_screens); i++) {
		if (vboot_screens[i].id == id)
			return &vboot_screens[i];
	}
	return NULL;
}

static void print_fallback_message(struct vboot_info *vboot,
				   const struct vboot_ui_descriptor *desc)
{
	if (desc->mesg) {
		struct udevice *console = vboot->console;
		struct vidconsole_priv *priv = dev_get_uclass_priv(console);
		int cols = priv->cols;
		int rows = priv->rows;
		const char *s;

		/* We need a measure function for vidconsole */
		vidconsole_position_cursor(console,
					   (cols - strlen(desc->mesg)) / 2,
					   rows / 2);
		for (s = desc->mesg; *s; s++)
			vidconsole_put_char(console, *s);
	} else {
		video_clear(vboot->video);  //TODO: White
	}
}

static VbError_t draw_ui(struct vboot_info *vboot, u32 screen_type,
			 struct params *p)
{
	VbError_t rv = VBERROR_UNKNOWN;
	const struct vboot_ui_descriptor *desc;

	desc = get_ui_descriptor(screen_type);
	if (!desc) {
		printf("Not a valid screen type: 0x%x\n", screen_type);
		return VBERROR_INVALID_SCREEN_INDEX;
	}

	if (p->locale >= locale_data.count) {
		printf("Unsupported locale (%d)\n", p->locale);
		print_fallback_message(vboot, desc);
		return VBERROR_INVALID_PARAMETER;
	}

	/* if no drawing function is registered, fallback msg will be printed */
	if (desc->draw) {
		rv = desc->draw(vboot, p);
		if (rv)
			printf("Drawing failed (0x%x)\n", rv);
	}
	if (rv) {
		print_fallback_message(vboot, desc);
		return VBERROR_SCREEN_DRAW;
	}

	return VBERROR_SUCCESS;
}

static int vboot_init_locale(struct vboot_info *vboot)
{
	struct fmap_entry fentry;
	char *loc_start, *loc;
	u8 *locales;
	int size;
	int ret;

	ret = cros_ofnode_find_locale("locales", &fentry);
	if (ret)
		return log_msg_ret("Cannot read locales list\n", ret);

	locale_data.count = 0;

	/* Load locale list */
	ret = fwstore_load_image(vboot->fwstore, &fentry, &locales, &size);
	if (ret)
		return log_msg_ret("locale list not found", ret);

	/* Copy the file and null-terminate it */
	loc_start = malloc(size + 1);
	if (!loc_start) {
		free(locales);
		return log_msg_ret("cannot allocate locales", -ENOMEM);
	}
	memcpy(loc_start, locales, size);
	loc_start[size] = '\0';

	/* Parse the list */
	log_info("Supported locales:\n");
	loc = loc_start;
	while (loc - loc_start < size &&
	       locale_data.count < ARRAY_SIZE(locale_data.codes)) {
		char *lang = strsep(&loc, "\n");

		if (!lang || !strlen(lang))
			break;
		log_info(" %s,", lang);
		locale_data.codes[locale_data.count] = lang;
		locale_data.count++;
	}
	free(locales);

	log_info(" (%d locales)\n", locale_data.count);

	return 0;
}

static VbError_t vboot_init_screen(struct vboot_info *vboot)
{
	int ret;

	ret = uclass_first_device_err(UCLASS_VIDEO, &vboot->video);
	if (ret) {
		log_err("Cannot find video device (err=%d)\n", ret);
		return VBERROR_UNKNOWN;
	}

	ret = uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &vboot->console);
	if (ret) {
		log_err("Cannot find console device (err=%d)\n", ret);
		return VBERROR_UNKNOWN;
	}

	ret = uclass_first_device_err(UCLASS_PANEL, &vboot->panel);
	if (ret)
		log_warning("No panel found (cannot adjust backlight)\n");

	ret = cbgfx_init(vboot->video);
	if (ret) {
		log_err("cbgfx_init() failed (err=%d)\n", ret);
		return VBERROR_UNKNOWN;
	}

	/* create a list of supported locales */
	ret = vboot_init_locale(vboot);
	if (ret) {
		log_err("Failed to load locales (err=%d)\n", ret);
		return VBERROR_INVALID_BMPFV;
	}

	/*
	 * Load generic (location-free) graphics data, ignoring errors.
	 * Fallback screens will be drawn for missing data
	 */
	load_archive("vbgfx.bin", &base_graphics);

	/* load font graphics */
	load_archive("font.bin", &font_graphics);

	/* reset localised graphics. we defer loading it */
	locale_data.archive = NULL;

	initialised = true;

	return VBERROR_SUCCESS;
}

static void update_backlight(struct vboot_info *vboot, bool enable)
{
	int ret;

	if (!vboot->panel)
		return;
	ret = panel_set_backlight(vboot->panel, enable ? BACKLIGHT_DEFAULT :
				  BACKLIGHT_OFF);
	if (ret)
		log_warning("Failed to set backlight\n");
}

int vboot_draw_screen(u32 screen, u32 locale)
{
	struct vboot_info *vboot = vboot_get();

	printf("%s: screen=0x%x locale=%d\n", __func__, screen, locale);

	if (!initialised) {
		if (vboot_init_screen(vboot))
			return VBERROR_UNKNOWN;
	}
	update_backlight(vboot, screen != VB_SCREEN_BLANK);

	/*
	 * TODO: draw only locale dependent part if current_screen == screen
	 *
	 * setting selected_index value to 0xFFFFFFFF invalidates the field
	 */
	struct params p = { locale, 0xFFFFFFFF, 0, 1 };

	RET_ON_ERR(draw_ui(vboot, screen, &p));

	locale_data.current = locale;

	return VBERROR_SUCCESS;
}

int vboot_draw_ui(u32 screen, u32 locale,
		  u32 selected_index, u32 disabled_idx_mask,
		  u32 redraw_base)
{
	struct vboot_info *vboot = vboot_get();

	log_debug("screen=0x%x locale=%d, selected_index=%d,disabled_idx_mask=0x%x\n",
		  screen, locale, selected_index, disabled_idx_mask);

	if (!initialised) {
		if (vboot_init_screen(vboot))
			return VBERROR_UNKNOWN;
	}

	/* If the screen is blank, turn off the backlight; else turn it on */
	update_backlight(vboot, screen != VB_SCREEN_BLANK);

	struct params p = { locale, selected_index,
			    disabled_idx_mask, redraw_base };
	return draw_ui(vboot, screen, &p);
}

int vboot_get_locale_count(void)
{
	struct vboot_info *vboot = vboot_get();

	if (!initialised) {
		if (vboot_init_screen(vboot))
			return VBERROR_UNKNOWN;
	}
	return locale_data.count;
}

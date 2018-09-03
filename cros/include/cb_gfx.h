/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Taken from coreboot file payloads/libpayload/include/cbgfx.h
 *
 * Copyright 2018 Google LLC.
 */

#ifndef __CROS_CB_GFX_H
#define __CROS_CB_GFX_H

/* API error codes */
#define CBGFX_SUCCESS			0
/* unknown error */
#define CBGFX_ERROR_UNKNOWN		1
/* failed to initialise cbgfx library */
#define CBGFX_ERROR_INIT		2
/* drawing beyond screen or canvas boundary */
#define CBGFX_ERROR_BOUNDARY		3
/* invalid parameter */
#define CBGFX_ERROR_INVALID_PARAMETER	4
/* bitmap error: signature mismatch */
#define CBGFX_ERROR_BITMAP_SIGNATURE	0x10
/* bitmap error: unsupported format */
#define CBGFX_ERROR_BITMAP_FORMAT	0x11
/* bitmap error: invalid data */
#define CBGFX_ERROR_BITMAP_DATA		0x12
/* bitmap error: scaling out of range */
#define CBGFX_ERROR_SCALE_OUT_OF_RANGE	0x13
/* invalid framebuffer info */
#define CBGFX_ERROR_FRAMEBUFFER_INFO	0x14
/* invalid framebuffer address */
#define CBGFX_ERROR_FRAMEBUFFER_ADDR	0x15
/* portrait screen not supported */
#define CBGFX_ERROR_PORTRAIT_SCREEN	0x16

struct fraction {
	s32 n;
	s32 d;
};

struct scale {
	struct fraction x;
	struct fraction y;
};

struct vector {
	union {
		s32 x;
		s32 width;
	};
	union {
		s32 y;
		s32 height;
	};
};

struct rect {
	struct vector offset;
	struct vector size;
};

struct rgb_colour {
	u8 red;
	u8 green;
	u8 blue;
};

/*
 * Resolution of scale parameters used to describe height, width, coordinate,
 * etc. relative to the canvas. For example, if it's 100, scales range from 0 to
 * 100%.
 */
#define CANVAS_SCALE		100

/*
 * The coordinate system is expected to have (0, 0) at top left corner with
 * y values increasing towards bottom of screen.
 */

/** Clear the screen */
int cbgfx_clear_screen(const struct rgb_colour *rgb);

/**
 * Draw a bitmap image using position and size relative to the canvas
 *
 * 'Pivot' is a point of the image based on which the image is positioned.
 * For example, if a pivot is set to PIVOT_H_CENTER|PIVOT_V_CENTER, the image is
 * positioned so that pos_rel matches the center of the image.
 *
 * @bitmap	Pointer to the bitmap data, starting from file header
 * @size	Size of the bitmap data
 * @pos_rel	Coordinate of the pivot relative to the canvas
 * @dim_rel	Width and height of the image relative to the canvas
 *		width and height. They must not exceed 1 (=100%). If one
 *		is zero, it's derived from the other to keep the aspect
 *		ratio.
 * @flags	lower 8 bits is Pivot position. Use PIVOT_H_* and
 *		PIVOT_V_* flags.
 *		Bit 9 is bit to indicate if we invert the rendering.
 *		0 = render image as is, 1 = invert image.
 *
 * @return CBGFX_* error codes
 */
int cbgfx_draw_bitmap(const void *bitmap, size_t size,
		      const struct scale *pos_rel, const struct scale *dim_rel,
		      u32 flags);

/* Pivot flags. See the draw_bitmap description */
#define PIVOT_H_LEFT	BIT(0)
#define PIVOT_H_CENTER	BIT(1)
#define PIVOT_H_RIGHT	BIT(2)
#define PIVOT_V_TOP	BIT(3)
#define PIVOT_V_CENTER	BIT(4)
#define PIVOT_V_BOTTOM	BIT(5)
#define PIVOT_MASK	0x000000ff

/* invert flag */
#define INVERT_SHIFT	8
#define INVERT_COLOURS	BIT(INVERT_SHIFT)

/**
 * Get width and height of projected image
 *
 * It returns the width and height of the projected image. If the input height
 * is zero, it's derived from the input width to keep the aspect ratio, and vice
 * versa. If both are zero, the width and the height which can project the image
 * in the original size are returned.
 *
 * @bitmap	Pointer to the bitmap data, starting from file header
 * @sz		Size of the bitmap data
 * @dim_rel	Width and height of the image relative to the canvas
 *		width and height. They must not exceed 1 (=100%).
 *		On return, it contains automatically calculated width
 *		and/or height.
 *
 * @return CBGFX_* error codes
 */
int cbgfx_get_bitmap_dimension(const void *bitmap, size_t sz,
			       struct scale *dim_rel);

/**
 * cbgfx_init() - Initialise the library
 *
 * Sets up the canvas and the framebuffer
 *
 * @dev:	UCLASS_VIDEO device
 * @return 0 if OK, CBGFX_ERROR_... value on error
 */
int cbgfx_init(struct udevice *dev);

#endif /* __CROS_CB_GFX_H */

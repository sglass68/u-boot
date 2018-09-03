// SPDX-License-Identifier: BSD-3-Clause
/*
 * Taken from coreboot file of the same name
 *
 * TODO(sjg@chromium.org): Use U-Boot BMP support (may need scaling support)
 * and drop this file.
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY	LOGC_VBOOT

#include <common.h>
#include <bmp_layout.h>
#include <dm.h>
#include <mapmem.h>
#include <video.h>
#include <cros/cb_gfx.h>

/*
 * 'canvas' is the drawing area located in the center of the screen. It's a
 * square area, stretching vertically to the edges of the screen, leaving
 * non-drawing areas on the left and right. The screen is assumed to be
 * landscape.
 */
static struct rect canvas;
static struct rect screen;

struct cb_framebuffer {
	u32 tag;
	u32 size;

	u64 physical_address;
	u32 x_resolution;
	u32 y_resolution;
	u32 bytes_per_line;
	u8 bits_per_pixel;
	u8 red_mask_pos;
	u8 red_mask_size;
	u8 green_mask_pos;
	u8 green_mask_size;
	u8 blue_mask_pos;
	u8 blue_mask_size;
	u8 reserved_mask_pos;
	u8 reserved_mask_size;
};

/*
 * Framebuffer is assumed to assign a higher coordinate (larger x, y) to
 * a higher address
 */
static struct cb_framebuffer s_fbinfo, *fbinfo = &s_fbinfo;
static u8 *fbaddr;

#define PIVOT_H_MASK	(PIVOT_H_LEFT | PIVOT_H_CENTER | PIVOT_H_RIGHT)
#define PIVOT_V_MASK	(PIVOT_V_TOP | PIVOT_V_CENTER | PIVOT_V_BOTTOM)
#define ROUNDUP(x, y)	((((x) + ((y) - 1)) / (y)) * (y))
#define ABS(x)		((x) < 0 ? -(x) : (x))

static void add_vectors(struct vector *out, const struct vector *v1,
			const struct vector *v2)
{
	out->x = v1->x + v2->x;
	out->y = v1->y + v2->y;
}

static int is_valid_fraction(const struct fraction *f)
{
	return f->d != 0;
}

/*
 * Transform a vector:
 *	x' = x * a_x + offset_x
 *	y' = y * a_y + offset_y
 */
static int transform_vector(struct vector *out, const struct vector *in,
			    const struct scale *a, const struct vector *offset)
{
	if (!is_valid_fraction(&a->x) || !is_valid_fraction(&a->y))
		return CBGFX_ERROR_INVALID_PARAMETER;
	out->x = a->x.n * in->x / a->x.d + offset->x;
	out->y = a->y.n * in->y / a->y.d + offset->y;
	return CBGFX_SUCCESS;
}

/*
 * Returns 1 if v is exclusively within box, 0 if v is inclusively within box,
 * or -1 otherwise. Note that only the right and bottom edges are examined.
 */
static int within_box(const struct vector *v, const struct rect *bound)
{
	if (v->x < bound->offset.x + bound->size.width &&
	    v->y < bound->offset.y + bound->size.height)
		return 1;
	else if (v->x <= bound->offset.x + bound->size.width &&
		 v->y <= bound->offset.y + bound->size.height)
		return 0;
	else
		return -1;
}

static inline u32 calculate_colour(const struct rgb_colour *rgb, u8 invert)
{
	u32 colour = 0;

	colour |= (rgb->red >> (8 - fbinfo->red_mask_size))
		<< fbinfo->red_mask_pos;
	colour |= (rgb->green >> (8 - fbinfo->green_mask_size))
		<< fbinfo->green_mask_pos;
	colour |= (rgb->blue >> (8 - fbinfo->blue_mask_size))
		<< fbinfo->blue_mask_pos;
	if (invert)
		colour ^= 0xffffffff;
	return colour;
}

/*
 * Plot a pixel in a framebuffer. This is called from tight loops. Keep it slim
 * and do the validation at callers' site.
 */
static inline void set_pixel(struct vector *coord, u32 colour)
{
	const int bpp = fbinfo->bits_per_pixel;
	const int bpl = fbinfo->bytes_per_line;
	int i;
	u8 * const pixel = fbaddr + coord->y * bpl + coord->x * bpp / 8;

	for (i = 0; i < bpp / 8; i++)
		pixel[i] = (colour >> (i * 8));
}

int cbgfx_clear_screen(const struct rgb_colour *rgb)
{
	struct vector p;
	u32 colour = calculate_colour(rgb, 0);
	const int bpp = fbinfo->bits_per_pixel;
	const int bpl = fbinfo->bytes_per_line;

	/*
	 * If all significant bytes in colour are equal, fastpath through
	 * memset(). We assume that for 32bpp the high byte gets ignored anyway
	 */
	if ((((colour >> 8) & 0xff) == (colour & 0xff)) &&
	    (bpp == 16 || (((colour >> 16) & 0xff) == (colour & 0xff)))) {
		memset(fbaddr, colour & 0xff, screen.size.height * bpl);
	} else {
		for (p.y = 0; p.y < screen.size.height; p.y++)
			for (p.x = 0; p.x < screen.size.width; p.x++)
				set_pixel(&p, colour);
	}

	return CBGFX_SUCCESS;
}

/*
 * Bi-linear Interpolation
 *
 * It estimates the value of a middle point (tx, ty) using the values from four
 * adjacent points (q00, q01, q10, q11).
 */
static u32 bli(u32 q00, u32 q10, u32 q01, u32 q11, struct fraction *tx,
	       struct fraction *ty)
{
	u32 r0 = (tx->n * q10 + (tx->d - tx->n) * q00) / tx->d;
	u32 r1 = (tx->n * q11 + (tx->d - tx->n) * q01) / tx->d;
	u32 p = (ty->n * r1 + (ty->d - ty->n) * r0) / ty->d;
	return p;
}

static int draw_bitmap_v3(const struct vector *top_left,
			  const struct scale *scale,
			  const struct vector *dim,
			  const struct vector *dim_org,
			  const struct bmp_header *header,
			  const struct bmp_color_table_entry *pal,
			  const u8 *pixel_array,
			  u8 invert)
{
	const int bpp = header->bit_count;
	s32 dir;
	struct vector p;

	if (header->compression) {
		log_err("Compressed bitmaps are not supported\n");
		return CBGFX_ERROR_BITMAP_FORMAT;
	}
	if (bpp >= 16) {
		log_err("Non-palette bitmaps are not supported\n");
		return CBGFX_ERROR_BITMAP_FORMAT;
	}
	if (bpp != 8) {
		log_err("Unsupported bits per pixel: %d\n", bpp);
		return CBGFX_ERROR_BITMAP_FORMAT;
	}
	if (scale->x.n == 0 || scale->y.n == 0) {
		log_err("Scaling out of range\n");
		return CBGFX_ERROR_SCALE_OUT_OF_RANGE;
	}

	const s32 y_stride = ROUNDUP(dim_org->width * bpp / 8, 4);
	/*
	 * header->height can be positive or negative.
	 *
	 * If it's negative, pixel data is stored from top to bottom. We render
	 * image from the lowest row to the highest row.
	 *
	 * If it's positive, pixel data is stored from bottom to top. We render
	 * image from the highest row to the lowest row.
	 */
	p.y = top_left->y;
	if (header->height < 0) {
		dir = 1;
	} else {
		p.y += dim->height - 1;
		dir = -1;
	}
	/*
	 * Plot pixels scaled by the bilinear interpolation. We scan over the
	 * image on canvas (using d) and find the corresponding pixel in the
	 * bitmap data (using s0, s1).
	 *
	 * When d hits the right bottom corner, s0 also hits the right bottom
	 * corner of the pixel array because that's how scale->x and scale->y
	 * have been set. Since the pixel array size is already validated in
	 * parse_bitmap_header_v3, s0 is guranteed not to exceed pixel array
	 * boundary.
	 */
	struct vector s0, s1, d;
	struct fraction tx, ty;

	for (d.y = 0; d.y < dim->height; d.y++, p.y += dir) {
		s0.y = d.y * scale->y.d / scale->y.n;
		s1.y = s0.y;
		if (s1.y + 1 < dim_org->height)
			s1.y++;
		ty.d = scale->y.n;
		ty.n = (d.y * scale->y.d) % scale->y.n;
		const u8 *data0 = pixel_array + s0.y * y_stride;
		const u8 *data1 = pixel_array + s1.y * y_stride;

		p.x = top_left->x;
		for (d.x = 0; d.x < dim->width; d.x++, p.x++) {
			s0.x = d.x * scale->x.d / scale->x.n;
			s1.x = s0.x;
			if (s1.x + 1 < dim_org->width)
				s1.x++;
			tx.d = scale->x.n;
			tx.n = (d.x * scale->x.d) % scale->x.n;
			u8 c00 = data0[s0.x];
			u8 c10 = data0[s1.x];
			u8 c01 = data1[s0.x];
			u8 c11 = data1[s1.x];

			if (c00 >= header->colors_used ||
			    c10 >= header->colors_used ||
			    c01 >= header->colors_used ||
			    c11 >= header->colors_used) {
				log_err("colour index exceeds palette boundary\n");
				return CBGFX_ERROR_BITMAP_DATA;
			}
			const struct rgb_colour rgb = {
				.red = bli(pal[c00].red, pal[c10].red,
					   pal[c01].red, pal[c11].red,
					   &tx, &ty),
				.green = bli(pal[c00].green, pal[c10].green,
					     pal[c01].green, pal[c11].green,
					     &tx, &ty),
				.blue = bli(pal[c00].blue, pal[c10].blue,
					    pal[c01].blue, pal[c11].blue,
					    &tx, &ty),
			};
			set_pixel(&p, calculate_colour(&rgb, invert));
		}
	}

	return CBGFX_SUCCESS;
}

static int get_bitmap_file_header(const void *bitmap, size_t size,
				  struct bmp_header *hdr)
{
	const struct bmp_header *fh;

	if (sizeof(*hdr) > size) {
		log_err("Invalid bitmap data\n");
		return CBGFX_ERROR_BITMAP_DATA;
	}
	fh = (struct bmp_header *)bitmap;
	if (fh->signature[0] != 'B' || fh->signature[1] != 'M') {
		log_err("Bitmap signature mismatch\n");
		return CBGFX_ERROR_BITMAP_SIGNATURE;
	}
	hdr->file_size = le32_to_cpu(fh->file_size);
	if (hdr->file_size != size) {
		log_err("Bitmap file size does not match cbfs file size\n");
		return CBGFX_ERROR_BITMAP_DATA;
	}
	hdr->data_offset = le32_to_cpu(fh->data_offset);

	return CBGFX_SUCCESS;
}

/**
 * parse_header() - Parse a BMP v3 header return
 *
 * @bitmap:	BMP bitmap to parse
 * @size:	Size of BMP file
 * @header:	Returns header
 * @palette:	Returns colour palette
 * @pixel_array: Returns a pointer to the pixel data
 * @dim_ord:	Returns image width/heght in pixels
 * @return 0 if OK, other value on error
 */
static int parse_header(const u8 *bitmap, size_t size,
			struct bmp_header *header,
			const struct bmp_color_table_entry **palette,
			const u8 **pixel_array,
			struct vector *dim_org)
{
	struct bmp_header file_header;
	struct bmp_header *h;
	int rv;

	rv = get_bitmap_file_header(bitmap, size, &file_header);
	if (rv)
		return rv;

	size_t header_size = sizeof(struct bmp_header) - 14;
	size_t palette_offset = sizeof(struct bmp_header);
	size_t file_size = file_header.file_size;

	h = (struct bmp_header *)bitmap;
	header->size = le32_to_cpu(h->size);
	if (header->size != header_size) {
		log_err("Unsupported bitmap format\n");
		return CBGFX_ERROR_BITMAP_FORMAT;
	}

	header->width = le32_to_cpu(h->width);
	header->height = le32_to_cpu(h->height);
	if (header->width == 0 || header->height == 0) {
		log_err("Invalid image width or height\n");
		return CBGFX_ERROR_BITMAP_DATA;
	}
	dim_org->width = header->width;
	dim_org->height = ABS(header->height);

	header->bit_count = le16_to_cpu(h->bit_count);
	header->compression = le32_to_cpu(h->compression);
	header->image_size = le32_to_cpu(h->image_size);
	header->colors_used = le32_to_cpu(h->colors_used);
	size_t palette_size = header->colors_used
			* sizeof(struct bmp_color_table_entry);
	size_t pixel_offset = file_header.data_offset;

	if (pixel_offset > file_size) {
		log_err("Bitmap pixel data exceeds buffer boundary\n");
		return CBGFX_ERROR_BITMAP_DATA;
	}
	if (palette_offset + palette_size > pixel_offset) {
		log_err("Bitmap palette data exceeds palette boundary\n");
		return CBGFX_ERROR_BITMAP_DATA;
	}
	*palette = (struct bmp_color_table_entry *)(bitmap +
			palette_offset);

	size_t pixel_size = header->image_size;

	if (pixel_size != dim_org->height *
		ROUNDUP(dim_org->width * header->bit_count / 8, 4)) {
		log_err("Bitmap pixel array size does not match expected size\n");
		return CBGFX_ERROR_BITMAP_DATA;
	}
	if (pixel_offset + pixel_size > file_size) {
		log_err("Bitmap pixel array exceeds buffer boundary\n");
		return CBGFX_ERROR_BITMAP_DATA;
	}
	*pixel_array = bitmap + pixel_offset;

	return CBGFX_SUCCESS;
}

/*
 * This calculates the dimension of the image projected on the canvas from the
 * dimension relative to the canvas size. If either width or height is zero, it
 * is derived from the other (non-zero) value to keep the aspect ratio.
 */
static int calculate_dimension(const struct vector *dim_org,
			       const struct scale *dim_rel,
			       struct vector *dim)
{
	if (dim_rel->x.n == 0 && dim_rel->y.n == 0)
		return CBGFX_ERROR_INVALID_PARAMETER;

	if (dim_rel->x.n > dim_rel->x.d || dim_rel->y.n > dim_rel->y.d)
		return CBGFX_ERROR_INVALID_PARAMETER;

	if (dim_rel->x.n > 0) {
		if (!is_valid_fraction(&dim_rel->x))
			return CBGFX_ERROR_INVALID_PARAMETER;
		dim->width = canvas.size.width  * dim_rel->x.n / dim_rel->x.d;
	}
	if (dim_rel->y.n > 0) {
		if (!is_valid_fraction(&dim_rel->y))
			return CBGFX_ERROR_INVALID_PARAMETER;
		dim->height = canvas.size.height * dim_rel->y.n / dim_rel->y.d;
	}

	/* Derive height from width using aspect ratio */
	if (dim_rel->y.n == 0)
		dim->height = dim->width * dim_org->height / dim_org->width;
	/* Derive width from height using aspect ratio */
	if (dim_rel->x.n == 0)
		dim->width = dim->height * dim_org->width / dim_org->height;

	return CBGFX_SUCCESS;
}

static int calculate_position(const struct vector *dim,
			      const struct scale *pos_rel, u8 pivot,
			      struct vector *top_left)
{
	int rv;

	rv = transform_vector(top_left, &canvas.size, pos_rel, &canvas.offset);
	if (rv)
		return rv;

	switch (pivot & PIVOT_H_MASK) {
	case PIVOT_H_LEFT:
		break;
	case PIVOT_H_CENTER:
		top_left->x -= dim->width / 2;
		break;
	case PIVOT_H_RIGHT:
		top_left->x -= dim->width;
		break;
	default:
		return CBGFX_ERROR_INVALID_PARAMETER;
	}

	switch (pivot & PIVOT_V_MASK) {
	case PIVOT_V_TOP:
		break;
	case PIVOT_V_CENTER:
		top_left->y -= dim->height / 2;
		break;
	case PIVOT_V_BOTTOM:
		top_left->y -= dim->height;
		break;
	default:
		return CBGFX_ERROR_INVALID_PARAMETER;
	}

	return CBGFX_SUCCESS;
}

static int check_boundary(const struct vector *top_left,
			  const struct vector *dim,
			  const struct rect *bound)
{
	struct vector v;

	add_vectors(&v, dim, top_left);
	if (top_left->x < bound->offset.x || top_left->y < bound->offset.y ||
	    within_box(&v, bound) < 0)
		return CBGFX_ERROR_BOUNDARY;
	return CBGFX_SUCCESS;
}

int cbgfx_draw_bitmap(const void *bitmap, size_t size,
		      const struct scale *pos_rel, const struct scale *dim_rel,
		      u32 flags)
{
	struct bmp_header header;
	const struct bmp_color_table_entry *palette;
	const u8 *pixel_array;
	struct vector top_left, dim, dim_org;
	struct scale scale;
	int rv;
	const u8 pivot = flags & PIVOT_MASK;
	const u8 invert = (flags & INVERT_COLOURS) >> INVERT_SHIFT;

	/* only v3 is supported now */
	rv = parse_header(bitmap, size, &header, &palette, &pixel_array,
			  &dim_org);
	if (rv)
		return rv;

	/* Calculate height and width of the image */
	rv = calculate_dimension(&dim_org, dim_rel, &dim);
	if (rv)
		return rv;

	/* Calculate self scale */
	scale.x.n = dim.width;
	scale.x.d = dim_org.width;
	scale.y.n = dim.height;
	scale.y.d = dim_org.height;

	/* Calculate coordinate */
	rv = calculate_position(&dim, pos_rel, pivot, &top_left);
	if (rv)
		return rv;

	rv = check_boundary(&top_left, &dim, &canvas);
	if (rv) {
		log_err("Bitmap image exceeds canvas boundary\n");
		return rv;
	}

	return draw_bitmap_v3(&top_left, &scale, &dim, &dim_org,
			      &header, palette, pixel_array, invert);
}

int cbgfx_get_bitmap_dimension(const void *bitmap, size_t sz,
			       struct scale *dim_rel)
{
	struct bmp_header header;
	const struct bmp_color_table_entry *palette;
	const u8 *pixel_array;
	struct vector dim, dim_org;
	int rv;

	rv = parse_header(bitmap, sz, &header, &palette, &pixel_array,
			  &dim_org);
	if (rv)
		return rv;

	/* Calculate height and width of the image */
	rv = calculate_dimension(&dim_org, dim_rel, &dim);
	if (rv)
		return rv;

	/* Calculate size relative to the canvas */
	dim_rel->x.n = dim.width;
	dim_rel->x.d = canvas.size.width;
	dim_rel->y.n = dim.height;
	dim_rel->y.d = canvas.size.height;

	return CBGFX_SUCCESS;
}

int cbgfx_init(struct udevice *dev)
{
	struct video_uc_platdata *plat = dev_get_uclass_platdata(dev);
	struct video_priv *priv = dev_get_uclass_priv(dev);

	fbinfo->physical_address = plat->base;
	fbinfo->x_resolution = priv->xsize;
	fbinfo->y_resolution = priv->ysize;
	fbinfo->bytes_per_line = priv->line_length;
	fbinfo->bits_per_pixel = 1 << priv->bpix;
	fbinfo->reserved_mask_pos = 0;
	fbinfo->reserved_mask_size = 0;
	switch (priv->bpix) {
	case VIDEO_BPP32:
		fbinfo->red_mask_pos = 0;
		fbinfo->red_mask_size = 8;
		fbinfo->green_mask_pos = 8;
		fbinfo->green_mask_size = 8;
		fbinfo->blue_mask_pos = 16;
		fbinfo->blue_mask_size = 8;
		break;
	case VIDEO_BPP16:
		fbinfo->red_mask_pos = 11;
		fbinfo->red_mask_size = 5;
		fbinfo->green_mask_pos = 5;
		fbinfo->green_mask_size = 6;
		fbinfo->blue_mask_pos = 0;
		fbinfo->blue_mask_size = 5;
		break;
	default:
		log(UCLASS_VIDEO, LOGL_ERR, "Invalid bpix %d\n", priv->bpix);
		return CBGFX_ERROR_INIT;
	}

	fbaddr = map_sysmem(fbinfo->physical_address, plat->size);
	if (!fbaddr)
		return CBGFX_ERROR_FRAMEBUFFER_ADDR;

	screen.size.width = fbinfo->x_resolution;
	screen.size.height = fbinfo->y_resolution;
	screen.offset.x = 0;
	screen.offset.y = 0;

	/* Calculate canvas size & offset. Canvas is always square */
	if (screen.size.height > screen.size.width) {
		canvas.size.height = screen.size.width;
		canvas.size.width = canvas.size.height;
		canvas.offset.x = 0;
		canvas.offset.y = (screen.size.height - canvas.size.height) / 2;
	} else {
		canvas.size.height = screen.size.height;
		canvas.size.width = canvas.size.height;
		canvas.offset.x = (screen.size.width - canvas.size.width) / 2;
		canvas.offset.y = 0;
	}

	log_info("cbgfx initialised: screen:width=%d, height=%d, offset=%d canvas:width=%d, height=%d, offset=%d\n",
		 screen.size.width, screen.size.height, screen.offset.x,
		 canvas.size.width, canvas.size.height, canvas.offset.x);

	return 0;
}

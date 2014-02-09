/*
 * Copyright (C) 2009
 * Guennadi Liakhovetski, DENX Software Engineering, <lg@denx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 */
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <memory.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <semaphore.h>
#include <iconv.h>
#include <dirent.h>
#include <stdint.h>
#include <glib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/mman.h>
#include <sys/vfs.h>
#include <sys/wait.h>
#include <sys/utsname.h>

#include <asm/byteorder.h>

#include <linux/types.h>
#include <linux/usb/functionfs.h>
#include <linux/usb/ch9.h>

#define cpu_to_le16(x)	htole16(x)
#define cpu_to_le32(x)	htole32(x)
#define le32_to_cpu(x)	le32toh(x)
#define le16_to_cpu(x)	le16toh(x)

#define min(a,b) ({ typeof(a) __a = (a); typeof(b) __b = (b); __a < __b ? __a : __b; })
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

static int verbose;

/* Still Image class-specific requests: */
#define USB_REQ_PTP_CANCEL_REQUEST		0x64
#define USB_REQ_PTP_GET_EXTENDED_EVENT_DATA	0x65
#define USB_REQ_PTP_DEVICE_RESET_REQUEST	0x66
#define USB_REQ_PTP_GET_DEVICE_STATUS_REQUEST	0x67

#define DRIVER_VENDOR_NUM	0x1d6b
#define DRIVER_PRODUCT_NUM	0x0100
#define DRIVER_INTERFACE	"PTP Interface"

/* Will be used for bcdDevice: remember to update on major changes */
#define MAJOR			1
#define MINOR			1
#define DRIVER_VERSION		((MAJOR << 8) | MINOR)
#define VERSION_STRING		__stringify(MAJOR) "." __stringify(MINOR)

#define PTP_MANUFACTURER	"Linux Foundation"
#define PTP_MODEL		"PTP Gadget"
#define PTP_STORAGE_DESC	"SD/MMC"
#define PTP_MODEL_DIR		"100LINUX"

#define THUMB_SUPPORT
#undef THUMB_SUPPORT

/*-------------------------------------------------------------------------*/

/* USB subclass value = the protocol encapsulation */
#define USB_SC_IMAGE_CAPTURE	0x01		/* Still Image Capture Subclass */
#define USB_PR_CB		0x01		/* Control/Bulk w/o interrupt */

#define MAX_PACKET_SIZE_HS 512

/* some devices can handle other status packet sizes */
#define STATUS_MAXPACKET	8

static const struct
{
	struct usb_functionfs_descs_head header;
	struct {
		struct usb_interface_descriptor intf;
		struct usb_endpoint_descriptor_no_audio source;
		struct usb_endpoint_descriptor_no_audio sink;
		struct usb_endpoint_descriptor_no_audio status;
	} __attribute__((packed)) fs_descs, hs_descs;
} __attribute__((packed)) descriptors = {
	.header = {
		.magic = cpu_to_le32(FUNCTIONFS_DESCRIPTORS_MAGIC),
		.length = cpu_to_le32(sizeof(descriptors)),
		.fs_count = 4,
		.hs_count = 4,
	}, .fs_descs = {
		.intf = {
			.bLength		= sizeof(descriptors.fs_descs.intf),
			.bDescriptorType	= USB_DT_INTERFACE,
			.bNumEndpoints		= 3,
			.bInterfaceClass	= USB_CLASS_STILL_IMAGE,
			.bInterfaceSubClass	= USB_SC_IMAGE_CAPTURE,
			.bInterfaceProtocol	= USB_PR_CB,
			.iInterface		= 1,
		}, .source = {
			.bLength		= sizeof(descriptors.fs_descs.source),
			.bDescriptorType	= USB_DT_ENDPOINT,
			.bEndpointAddress	= 1 | USB_DIR_IN,
			.bmAttributes		= USB_ENDPOINT_XFER_BULK,
		}, .sink = {
			.bLength		= sizeof(descriptors.fs_descs.sink),
			.bDescriptorType	= USB_DT_ENDPOINT,
			.bEndpointAddress	= 2 | USB_DIR_OUT,
			.bmAttributes		= USB_ENDPOINT_XFER_BULK,
		}, .status = {
			.bLength		= sizeof(descriptors.fs_descs.status),
			.bDescriptorType	= USB_DT_ENDPOINT,
			.bmAttributes		= USB_ENDPOINT_XFER_INT,
			.bEndpointAddress	= 3 | USB_DIR_IN,
			.wMaxPacketSize		= __constant_cpu_to_le16(STATUS_MAXPACKET),
			.bInterval		= 10,
		},
	}, .hs_descs = {
		.intf = {
			.bLength		= sizeof(descriptors.hs_descs.intf),
			.bDescriptorType	= USB_DT_INTERFACE,
			.bNumEndpoints		= 3,
			.bInterfaceClass	= USB_CLASS_STILL_IMAGE,
			.bInterfaceSubClass	= USB_SC_IMAGE_CAPTURE,
			.bInterfaceProtocol	= USB_PR_CB,
			.iInterface		= 1,
		}, .source = {
			.bLength		= sizeof(descriptors.hs_descs.source),
			.bDescriptorType	= USB_DT_ENDPOINT,
			.bEndpointAddress	= 1 | USB_DIR_IN,
			.bmAttributes		= USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize		= __constant_cpu_to_le16(MAX_PACKET_SIZE_HS),
		}, .sink = {
			.bLength		= sizeof(descriptors.hs_descs.sink),
			.bDescriptorType	= USB_DT_ENDPOINT,
			.bEndpointAddress	= 2 | USB_DIR_OUT,
			.bmAttributes		= USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize		= __constant_cpu_to_le16(MAX_PACKET_SIZE_HS),
		}, .status = {
			.bLength		= sizeof(descriptors.hs_descs.status),
			.bDescriptorType	= USB_DT_ENDPOINT,
			.bEndpointAddress	= 3 | USB_DIR_IN,
			.bmAttributes		= USB_ENDPOINT_XFER_INT,
			.wMaxPacketSize		= __constant_cpu_to_le16(STATUS_MAXPACKET),
			.bInterval		= 10,
		},
	},
};

/*-------------------------------------------------------------------------*/

static const struct
{
	struct usb_functionfs_strings_head header;
	struct {
		__le16 code;
		const char str1[sizeof(DRIVER_INTERFACE)];
	} __attribute__((packed)) lang0;
} __attribute__((packed)) strings = {
	.header = {
		.magic = cpu_to_le32(FUNCTIONFS_STRINGS_MAGIC),
		.length = cpu_to_le32(sizeof(strings)),
		.str_count = cpu_to_le32(1),
		.lang_count = cpu_to_le32(1),
	}, .lang0 = {
		cpu_to_le16(0x0409), /* en-us */
		DRIVER_INTERFACE,
	},
};

/*-------------------------------------------------------------------------*/

/* kernel drivers could autoconfigure like this too ... if
 * they were willing to waste the relevant code/data space.
 */

#define FFS_PREFIX	"/dev/ptp/"
#define FFS_PTP_EP0	FFS_PREFIX"ep0"
#define FFS_PTP_IN	FFS_PREFIX"ep1"
#define FFS_PTP_OUT	FFS_PREFIX"ep2"
#define FFS_PTP_INT	FFS_PREFIX"ep3"

/* gadgetfs currently has no chunking (or O_DIRECT/zerocopy) support
 * to turn big requests into lots of smaller ones; so this is "small".
 */
#define	USB_BUFSIZE	(7 * 1024)

#define CHECK_COUNT(cnt, min, max, op) do {			\
	if (cnt & 3 || cnt < min || cnt > max) {		\
		fprintf(stderr, "Wrong " op " size: %u\n",	\
			(unsigned int)cnt);					\
		errno = EPIPE;					\
		return -1;					\
	}							\
} while (0)

#define CHECK_SESSION(s_container, r_container, cnt, ret) do {	\
	if (session <= 0) {					\
		make_response(s_container, r_container,		\
			PIMA15740_RESP_SESSION_NOT_OPEN,	\
			sizeof(*s_container));			\
		*cnt = 0;					\
		*ret = 0;					\
		break;						\
	}							\
} while (0)

#define STORE_ID		0x00010001

#define PTP_PARAM_UNUSED	0
#define PTP_PARAM_ANY		0xffffffff

/* All little endian */
struct ptp_container {
	uint32_t	length;
	uint16_t	type;
	uint16_t	code;
	uint32_t	id;
	uint8_t		payload[];
} __attribute__ ((packed));

enum ptp_container_type {
	PTP_CONTAINER_TYPE_UNDEFINED		= 0,
	PTP_CONTAINER_TYPE_COMMAND_BLOCK	= 1,
	PTP_CONTAINER_TYPE_DATA_BLOCK		= 2,
	PTP_CONTAINER_TYPE_RESPONSE_BLOCK	= 3,
	PTP_CONTAINER_TYPE_EVENT_BLOCK		= 4,
};

enum pima15740_operation_code {
	PIMA15740_OP_UNDEFINED			= 0x1000,
	PIMA15740_OP_GET_DEVICE_INFO		= 0x1001,
	PIMA15740_OP_OPEN_SESSION		= 0x1002,
	PIMA15740_OP_CLOSE_SESSION		= 0x1003,
	PIMA15740_OP_GET_STORAGE_IDS		= 0x1004,
	PIMA15740_OP_GET_STORAGE_INFO		= 0x1005,
	PIMA15740_OP_GET_NUM_OBJECTS		= 0x1006,
	PIMA15740_OP_GET_OBJECT_HANDLES		= 0x1007,
	PIMA15740_OP_GET_OBJECT_INFO		= 0x1008,
	PIMA15740_OP_GET_OBJECT			= 0x1009,
	PIMA15740_OP_GET_THUMB			= 0x100a,
	PIMA15740_OP_DELETE_OBJECT		= 0x100b,
	PIMA15740_OP_SEND_OBJECT_INFO		= 0x100c,
	PIMA15740_OP_SEND_OBJECT		= 0x100d,
	PIMA15740_OP_INITIATE_CAPTURE		= 0x100e,
	PIMA15740_OP_FORMAT_STORE		= 0x100f,
	PIMA15740_OP_RESET_DEVICE		= 0x1010,
	PIMA15740_OP_SELF_TEST			= 0x1011,
	PIMA15740_OP_SET_OBJECT_PROTECTION	= 0x1012,
	PIMA15740_OP_POWER_DOWN			= 0x1013,
	PIMA15740_OP_GET_DEVICE_PROP_DESC	= 0x1014,
	PIMA15740_OP_GET_DEVICE_PROP_VALUE	= 0x1015,
	PIMA15740_OP_SET_DEVICE_PROP_VALUE	= 0x1016,
	PIMA15740_OP_RESET_DEVICE_PROP_VALUE	= 0x1017,
	PIMA15740_OP_TERMINATE_OPEN_CAPTURE	= 0x1018,
	PIMA15740_OP_MOVE_OBJECT		= 0x1009,
	PIMA15740_OP_COPY_OBJECT		= 0x100a,
	PIMA15740_OP_GET_PARTIAL_OBJECT		= 0x100b,
	PIMA15740_OP_INITIATE_OPEN_CAPTURE	= 0x100c,
};

enum pima15740_response_code {
	PIMA15740_RESP_UNDEFINED				= 0x2000,
	PIMA15740_RESP_OK					= 0x2001,
	PIMA15740_RESP_GENERAL_ERROR				= 0x2002,
	PIMA15740_RESP_SESSION_NOT_OPEN				= 0x2003,
	PIMA15740_RESP_INVALID_TRANSACTION_ID			= 0x2004,
	PIMA15740_RESP_OPERATION_NOT_SUPPORTED			= 0x2005,
	PIMA15740_RESP_PARAMETER_NOT_SUPPORTED			= 0x2006,
	PIMA15740_RESP_INCOMPLETE_TRANSFER			= 0x2007,
	PIMA15740_RESP_INVALID_STORAGE_ID			= 0x2008,
	PIMA15740_RESP_INVALID_OBJECT_HANDLE			= 0x2009,
	PIMA15740_RESP_DEVICE_PROP_NOT_SUPPORTED		= 0x200a,
	PIMA15740_RESP_INVALID_OBJECT_FORMAT_CODE		= 0x200b,
	PIMA15740_RESP_STORE_FULL				= 0x200c,
	PIMA15740_RESP_OBJECT_WRITE_PROTECTED			= 0x200d,
	PIMA15740_RESP_STORE_READ_ONLY				= 0x200e,
	PIMA15740_RESP_ACCESS_DENIED				= 0x200f,
	PIMA15740_RESP_NO_THUMBNAIL_PRESENT			= 0x2010,
	PIMA15740_RESP_SELFTEST_FAILED				= 0x2011,
	PIMA15740_RESP_PARTIAL_DELETION				= 0x2012,
	PIMA15740_RESP_STORE_NOT_AVAILABLE			= 0x2013,
	PIMA15740_RESP_SPECIFICATION_BY_FORMAT_NOT_SUPPORTED	= 0x2014,
	PIMA15740_RESP_NO_VALID_OBJECT_INFO			= 0x2015,
	PIMA15740_RESP_INVALID_CODE_FORMAT			= 0x2016,
	PIMA15740_RESP_UNKNOWN_VENDOR_CODE			= 0x2017,
	PIMA15740_RESP_CAPTURE_ALREADY_TERMINATED		= 0x2018,
	PIMA15740_RESP_DEVICE_BUSY				= 0x2019,
	PIMA15740_RESP_INVALID_PARENT_OBJECT			= 0x201a,
	PIMA15740_RESP_INVALID_DEVICE_PROP_FORMAT		= 0x201b,
	PIMA15740_RESP_INVALID_DEVICE_PROP_VALUE		= 0x201c,
	PIMA15740_RESP_INVALID_PARAMETER			= 0x201d,
	PIMA15740_RESP_SESSION_ALREADY_OPEN			= 0x201e,
	PIMA15740_RESP_TRANSACTION_CANCELLED			= 0x201f,
	PIMA15740_RESP_SPECIFICATION_OF_DESTINATION_UNSUPPORTED	= 0x2020,
};

enum pima15740_data_format {
	PIMA15740_FMT_A_UNDEFINED		= 0x3000,
	PIMA15740_FMT_A_ASSOCIATION		= 0x3001,
	PIMA15740_FMT_A_TEXT			= 0x3004,
	PIMA15740_FMT_I_UNDEFINED		= 0x3800,
	PIMA15740_FMT_I_EXIF_JPEG		= 0x3801,
	PIMA15740_FMT_I_TIFF_EP			= 0x3802,
	PIMA15740_FMT_I_JFIF			= 0x3808,
	PIMA15740_FMT_I_PNG			= 0x380b,
	PIMA15740_FMT_I_TIFF			= 0x380d,
	PIMA15740_FMT_I_TIFF_IT			= 0x380e,
};

enum pima15740_storage_type {
	PIMA15740_STORAGE_UNDEFINED		= 0,
	PIMA15740_STORAGE_FIXED_ROM		= 0x0001,
	PIMA15740_STORAGE_REMOVABLE_ROM		= 0x0002,
	PIMA15740_STORAGE_FIXED_RAM		= 0x0003,
	PIMA15740_STORAGE_REMOVABLE_RAM		= 0x0004,
};

enum pima15740_filesystem_type {
	PIMA15740_FILESYSTEM_UNDEFINED		= 0,
	PIMA15740_FILESYSTEM_GENERIC_FLAT	= 0x0001,
	PIMA15740_FILESYSTEM_GENERIC_HIERARCH	= 0x0002,
	PIMA15740_FILESYSTEM_DCF		= 0x0003,
};

enum pima15740_access_capability {
	PIMA15740_ACCESS_CAP_RW			= 0,
	PIMA15740_ACCESS_CAP_RO_WITHOUT_DEL	= 0x0001,
	PIMA15740_ACCESS_CAP_RO_WITH_DEL	= 0x0002,
};

static const char manuf[] = PTP_MANUFACTURER;
static const char model[] = PTP_MODEL;
static const char storage_desc[] = PTP_STORAGE_DESC;

#define SUPPORTED_OPERATIONS					\
	__constant_cpu_to_le16(PIMA15740_OP_GET_DEVICE_INFO),	\
	__constant_cpu_to_le16(PIMA15740_OP_OPEN_SESSION),	\
	__constant_cpu_to_le16(PIMA15740_OP_CLOSE_SESSION),	\
	__constant_cpu_to_le16(PIMA15740_OP_GET_STORAGE_IDS),	\
	__constant_cpu_to_le16(PIMA15740_OP_GET_STORAGE_INFO),	\
	__constant_cpu_to_le16(PIMA15740_OP_GET_NUM_OBJECTS),	\
	__constant_cpu_to_le16(PIMA15740_OP_GET_OBJECT_HANDLES),\
	__constant_cpu_to_le16(PIMA15740_OP_GET_OBJECT_INFO),	\
	__constant_cpu_to_le16(PIMA15740_OP_GET_OBJECT),	\
	__constant_cpu_to_le16(PIMA15740_OP_GET_THUMB),		\
	__constant_cpu_to_le16(PIMA15740_OP_DELETE_OBJECT),	\
	__constant_cpu_to_le16(PIMA15740_OP_SEND_OBJECT_INFO),	\
	__constant_cpu_to_le16(PIMA15740_OP_SEND_OBJECT),

static uint16_t dummy_supported_operations[] = {
	SUPPORTED_OPERATIONS
};

#define SUPPORTED_FORMATS					\
	__constant_cpu_to_le16(PIMA15740_FMT_A_UNDEFINED),	\
	__constant_cpu_to_le16(PIMA15740_FMT_A_TEXT),		\
	__constant_cpu_to_le16(PIMA15740_FMT_I_EXIF_JPEG),	\
	__constant_cpu_to_le16(PIMA15740_FMT_I_TIFF_EP),	\
	__constant_cpu_to_le16(PIMA15740_FMT_I_PNG),		\
	__constant_cpu_to_le16(PIMA15740_FMT_I_TIFF),		\
	__constant_cpu_to_le16(PIMA15740_FMT_I_TIFF_IT),	\
	__constant_cpu_to_le16(PIMA15740_FMT_I_JFIF),

static uint16_t dummy_supported_formats[] = {
	SUPPORTED_FORMATS
};

struct my_device_info {
	uint16_t	std_ver;
	uint32_t	vendor_ext_id;
	uint16_t	vendor_ext_ver;
	uint8_t		vendor_ext_desc_len;
	uint16_t	func_mode;
	uint32_t	operations_n;
	uint16_t	operations[ARRAY_SIZE(dummy_supported_operations)];
	uint32_t	events_n;
	uint32_t	device_properties_n;
	uint32_t	capture_formats_n;
	uint32_t	image_formats_n;
	uint16_t	image_formats[ARRAY_SIZE(dummy_supported_formats)];
	uint8_t		manuf_len;
	uint8_t		manuf[sizeof(manuf) * 2];
	uint8_t		model_len;
	uint8_t		model[sizeof(model) * 2];
	uint8_t		dev_version_len;
	uint8_t		serial_num_len;
} __attribute__ ((packed));

struct my_device_info dev_info = {
	.std_ver		= __constant_cpu_to_le16(100),	/* Standard version 1.00 */
	.vendor_ext_id		= __constant_cpu_to_le32(0),
	.vendor_ext_ver		= __constant_cpu_to_le16(0),
	.vendor_ext_desc_len	= __constant_cpu_to_le16(0),
	.func_mode		= __constant_cpu_to_le16(0),
	.operations_n		= __constant_cpu_to_le32(ARRAY_SIZE(dummy_supported_operations)),
	.operations = {
		SUPPORTED_OPERATIONS
	},
	.events_n		= __constant_cpu_to_le32(0),
	.device_properties_n	= __constant_cpu_to_le32(0),
	.capture_formats_n	= __constant_cpu_to_le32(0),
	.image_formats_n	= __constant_cpu_to_le32(ARRAY_SIZE(dummy_supported_formats)),
	.image_formats = {
		SUPPORTED_FORMATS
	},
	.manuf_len = sizeof(manuf),
	.model_len = sizeof(model),
	.dev_version_len = 0,
	.serial_num_len	= 0,
};

struct my_storage_info {
	uint16_t	storage_type;
	uint16_t	filesystem_type;
	uint16_t	access_capability;
	uint64_t	max_capacity;
	uint64_t	free_space_in_bytes;
	uint32_t	free_space_in_images;
	uint8_t		desc_len;
	uint8_t		desc[sizeof(storage_desc) * 2];
	uint8_t		volume_label_len;
} __attribute__ ((packed));

static struct my_storage_info storage_info = {
	.storage_type		= __constant_cpu_to_le16(PIMA15740_STORAGE_REMOVABLE_RAM),
	.filesystem_type	= __constant_cpu_to_le16(PIMA15740_FILESYSTEM_DCF),
	.access_capability	= __constant_cpu_to_le16(PIMA15740_ACCESS_CAP_RW),
	.desc_len		= sizeof(storage_desc),
	.volume_label_len	= 0,
};

/* full duplex data, with at least three threads: ep0, sink, and source */

static int bulk_in = -ENXIO;
static int bulk_out = -ENXIO;
static int control = -ENXIO;
static int interrupt = -ENXIO;
static int session = -EINVAL;
static sem_t reset;

static iconv_t ic, uc;
static char *root;

#define	NEVENT		5

enum ptp_status {
	PTP_WAITCONFIG,	/* Waiting to be configured */
	PTP_IDLE,	/* Waiting for control / bulk-out */
	PTP_DATA_OUT,	/* Data arrival on bulk-out expected */
	PTP_DATA_READY,	/* Finished receive, have to process and send (Data and) Response */
	PTP_DATA_IN,	/* Waiting for bulk-in to become free for more data */
};

static enum ptp_status status = PTP_WAITCONFIG;

static pthread_t bulk_pthread;

#define __stringify_1(x)	#x
#define __stringify(x)		__stringify_1(x)

#define BUF_SIZE	4096
#ifdef THUMB_SUPPORT
#define THUMB_WIDTH	160
#define THUMB_HEIGHT	120
#define THUMB_SIZE	__stringify(THUMB_WIDTH) "x" __stringify(THUMB_HEIGHT)
#define THUMB_LOCATION    "/var/cache/ptp/thumb/"
#endif

struct ptp_object_info {
	uint32_t	storage_id;
	uint16_t	object_format;
	uint16_t	protection_status;
	uint32_t	object_compressed_size;
	uint16_t	thumb_format;
	uint32_t	thumb_compressed_size;
	uint32_t	thumb_pix_width;
	uint32_t	thumb_pix_height;
	uint32_t	image_pix_width;
	uint32_t	image_pix_height;
	uint32_t	image_bit_depth;
	uint32_t	parent_object;
	uint16_t	association_type;
	uint32_t	association_desc;
	uint32_t	sequence_number;
	uint8_t		strings[];
} __attribute__ ((packed));

static struct ptp_object_info association = {
	.storage_id		= __constant_cpu_to_le32(STORE_ID),
	.object_format		= __constant_cpu_to_le16(PIMA15740_FMT_A_ASSOCIATION),
	.protection_status	= __constant_cpu_to_le16(0),	/* Read-only */
	.object_compressed_size	= __constant_cpu_to_le32(4096),
	.thumb_format		= __constant_cpu_to_le16(0),
	.thumb_compressed_size	= __constant_cpu_to_le32(0),
	.thumb_pix_width	= __constant_cpu_to_le32(0),
	.thumb_pix_height	= __constant_cpu_to_le32(0),
	.image_pix_width	= __constant_cpu_to_le32(0),
	.image_pix_height	= __constant_cpu_to_le32(0),
	.image_bit_depth	= __constant_cpu_to_le32(0),
	.parent_object		= __constant_cpu_to_le32(0),	/* Will be overwritten */
	.association_type	= __constant_cpu_to_le16(1),	/* Generic Folder */
	.association_desc	= __constant_cpu_to_le32(0),
	.sequence_number	= __constant_cpu_to_le32(0),
};

struct obj_list {
	struct obj_list		*next;
	uint32_t		handle;
	size_t			info_size;
	char			name[256];
	struct ptp_object_info	info;
};

#define GFOREACH(item, list) for(iterator = list; (item = NULL, 1) && iterator && (item = iterator->data, 1); iterator = g_slist_next(iterator))

static GSList *images;
/* number of objects, including associations - decrement when deleting */
static int object_number;
static int last_object_number;

static struct obj_list *object_info_p;

static size_t put_string(iconv_t ic, char *buf, const char *s, size_t len);
static size_t get_string(iconv_t ic, char *buf, const char *s, size_t len);

static int object_handle_valid(unsigned int h)
{
	struct obj_list *obj;
	GSList *iterator;

	/* First two handles: dcim and PTP_MODEL_DIR */
	if (h == 1 || h == 2)
		return 1;

	GFOREACH(obj, images) {
		if (obj->handle == h)
			return 1;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static void make_response(struct ptp_container *s_cntn, struct ptp_container *r_cntn,
			  enum pima15740_response_code code, size_t len)
{
	s_cntn->id = r_cntn->id;
	s_cntn->type = __cpu_to_le16(PTP_CONTAINER_TYPE_RESPONSE_BLOCK);
	s_cntn->code = __cpu_to_le16(code);
	s_cntn->length = __cpu_to_le32(len);
}

static int bulk_write(void *buf, size_t length)
{
	size_t count = 0;
	int ret;

	do {
		ret = write(bulk_in, buf + count, length - count);
		if (ret < 0) {
			if (errno != EINTR)
				return ret;

			/* Need to wait for control thread to finish reset */
			sem_wait(&reset);
		} else
			count += ret;
	} while (count < length);

	if (verbose)
		fprintf(stderr, "BULK-IN Sent %u bytes\n", (unsigned int)count);

	return count;
}

static int bulk_read(void *buf, size_t length)
{
	size_t count = 0;
	int ret;

	do {
		ret = read(bulk_out, buf + count, length - count);
		if (ret < 0) {
			if (errno != EINTR)
				return ret;

			/* Need to wait for control thread to finish reset */
			sem_wait(&reset);
		} else
			count += ret;
	} while (count < length);

	if (verbose)
		fprintf(stderr, "BULK-OUT Read %u bytes\n", (unsigned int)count);

	return count;
}

static int send_association_handle(int n, struct ptp_container *s)
{
	uint32_t *handle;

	s->type = __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
	/* One array element */
	*(uint32_t *)s->payload = __cpu_to_le32(1);
	s->length = __cpu_to_le32(2 * sizeof(uint32_t) + sizeof(*s));

	handle = (uint32_t *)s->payload + 1;
	/* The next directory */
	*handle = __cpu_to_le32(n);
	return bulk_write(s, (void *)(handle + 1) - (void *)s);
}

static int send_object_handles(void *recv_buf, void *send_buf, size_t send_len)
{
	struct ptp_container *r_container = recv_buf;
	struct ptp_container *s_container = send_buf;
	unsigned long length;
	uint32_t *param;
	uint32_t store_id;
	struct obj_list *obj;
	GSList *iterator = NULL;
	int ret;
	uint32_t *handle;
	uint32_t format, association;
	int obj_to_send = g_slist_length(images) + 2;

	length	= __le32_to_cpu(r_container->length);

	param = (uint32_t *)r_container->payload;
	store_id = __le32_to_cpu(*param);

	/* supported storage IDs: 0x00010001 - our single storage, 0xffffffff - all stores */
	if (store_id != STORE_ID && store_id != PTP_PARAM_ANY) {
		make_response(s_container, r_container,
			      PIMA15740_RESP_INVALID_STORAGE_ID, sizeof(*s_container));
		return 0;
	}

	format = __le32_to_cpu(*(param + 1));
	if (length > 16 && format != PTP_PARAM_UNUSED && format != PTP_PARAM_ANY) {
		make_response(s_container, r_container,
			      PIMA15740_RESP_SPECIFICATION_BY_FORMAT_NOT_SUPPORTED,
			      sizeof(*s_container));
		return 0;
	}

	association = __le32_to_cpu(*(param + 2));
	if (length > 20 && association != PTP_PARAM_UNUSED && association != 2) {
		enum pima15740_response_code code;
		if (!object_handle_valid(association) && association != PTP_PARAM_ANY)
			code = PIMA15740_RESP_INVALID_OBJECT_HANDLE;
		else if (association == PTP_PARAM_ANY) {
			/* "/" is requested */
			ret = send_association_handle(1, s_container);
			if (ret < 0) {
				errno = EPIPE;
				return ret;
			} else
				code = PIMA15740_RESP_OK;
		} else if (association == 1) {
			/* The subdirectory of "/DCIM" is requested */
			ret = send_association_handle(2, s_container);
			if (ret < 0) {
				errno = EPIPE;
				return ret;
			} else
				code = PIMA15740_RESP_OK;
		} else
			code = PIMA15740_RESP_INVALID_PARENT_OBJECT;

		make_response(s_container, r_container, code, sizeof(*s_container));
		return 0;
	}

	if (association == 2)
		/* Only send contents of /DCIM/100LINUX */
		obj_to_send -= 2;

	s_container->type = __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
	*(uint32_t *)s_container->payload = __cpu_to_le32(obj_to_send);
	s_container->length = __cpu_to_le32((obj_to_send + 1) * sizeof(uint32_t) +
					    sizeof(*s_container));

	handle = (uint32_t *)s_container->payload + 1;

	if (association != 2) {
		/* The two directories */
		*handle++ = __cpu_to_le32(1);
		*handle++ = __cpu_to_le32(2);
	}

	GFOREACH(obj, images) {
		if ((void *)handle == send_buf + send_len) {
			ret = bulk_write(send_buf, send_len);
			if (ret < 0) {
				errno = EPIPE;
				return ret;
			}
			handle = send_buf;
		}

		*handle++ = __cpu_to_le32(obj->handle);
	}
	if ((void *)handle > send_buf) {
		ret = bulk_write(send_buf, (void *)handle - send_buf);
		if (ret < 0) {
			errno = EPIPE;
			return ret;
		}
	}

	/* Prepare response */
	make_response(s_container, r_container, PIMA15740_RESP_OK, sizeof(*s_container));

	return 0;
}

static int send_association(int n, struct ptp_container *s, size_t size)
{
	struct ptp_object_info *objinfo = (struct ptp_object_info *)s->payload;
	size_t len, total;
	int ret = -1;

	s->type = __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
	memcpy(objinfo, &association, sizeof(association));
	objinfo->object_compressed_size = __cpu_to_le32(size);
	switch (n) {
	case 1:
		len = strlen("DCIM") + 1;
		ret = put_string(ic, (char *)objinfo->strings + 1, "DCIM", len);
		objinfo->parent_object = __cpu_to_le32(0);
		break;
	case 2:
		len = strlen(PTP_MODEL_DIR) + 1;
		ret = put_string(ic, (char *)objinfo->strings + 1, PTP_MODEL_DIR, len);
		objinfo->parent_object = __cpu_to_le32(1);
		break;
	}
	if (ret < 0)
		return ret;
	objinfo->strings[0] = len;
	objinfo->strings[2 * len + 1] = 0;	/* Empty Capture Date */
	objinfo->strings[2 * len + 2] = 0;	/* Empty Modification Date */
	objinfo->strings[2 * len + 3] = 0;	/* Empty Keywords */
	total = 2 * len + 4 + sizeof(*s) + sizeof(*objinfo);
	s->length = __cpu_to_le32(total);

	return bulk_write(s, total);
}

static int send_object_info(void *recv_buf, void *send_buf, size_t send_len)
{
	struct ptp_container *r_container = recv_buf;
	struct ptp_container *s_container = send_buf;
	uint32_t *param;
	struct obj_list *obj = NULL;
	GSList *iterator;
	int ret;
	uint32_t handle;
	size_t count, total, offset;
	void *info;
	enum pima15740_response_code code = PIMA15740_RESP_OK;

	param = (uint32_t *)r_container->payload;
	handle = __le32_to_cpu(*param);

	if (handle == 1 || handle == 2) {
		struct stat dstat;
		size_t size;

		/* Directory information requested */
		if (handle == 2) {
			ret = stat(root, &dstat);
			if (ret < 0) {
				errno = EPIPE;
				return ret;
			}
			size = dstat.st_size;
			if (verbose > 1)
				fprintf(stderr, "%s size %u\n", root, (unsigned int)size);
		} else
			size = 4096;
		ret = send_association(handle, s_container, size);
		if (ret < 0) {
			errno = EPIPE;
			return ret;
		}

		goto send_resp;
	}

	GFOREACH(obj, images) {
		if (obj->handle == handle)
			break;
	}

	if (!obj) {
		code = PIMA15740_RESP_INVALID_OBJECT_HANDLE;
		goto send_resp;
	}

	s_container->type = __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
	total = obj->info_size + sizeof(*s_container);
	s_container->length = __cpu_to_le32(total);
	offset = sizeof(*s_container);
	info = &obj->info;

	/* Object Info cannot get > 4096 bytes - four strings make a maximum of 2048
	 * bytes plus a fixed-size block, but we play safe for the case someone
	 * makes the buffers smaller */

	while (total) {
		count = min(total, send_len);
		memcpy(send_buf + offset, info, count - offset);
		info += count - offset;
		ret = bulk_write(send_buf, count);
		if (ret < 0) {
			errno = EPIPE;
			return ret;
		}
		offset = 0;
		total -= count;
	}

send_resp:
	/* Prepare response */
	make_response(s_container, r_container, code, sizeof(*s_container));

	return 0;
}

static int send_object_or_thumb(void *recv_buf, void *send_buf, size_t send_len, int thumb)
{
	struct ptp_container *r_container = recv_buf;
	struct ptp_container *s_container = send_buf;
	uint32_t *param;
	struct obj_list *obj = NULL;
	GSList *iterator;
	int ret;
	uint32_t handle;
	size_t count, total, offset, file_size;
	void *data, *map;
	int fd = -1;
	char name[256];

	param = (uint32_t *)r_container->payload;
	handle = __le32_to_cpu(*param);

	GFOREACH(obj, images) {
		if (obj->handle == handle)
			break;
	}

	if (!obj) {
		make_response(s_container, r_container, PIMA15740_RESP_INVALID_OBJECT_HANDLE,
			      sizeof(*s_container));
		return 0;
	}

	s_container->type = __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
	offset = sizeof(*s_container);

#ifdef THUMB_SUPPORT
	if (!thumb) {
		strncpy(name, obj->name, sizeof(name) - 1);
		name[sizeof(name) - 1] = '\0';
		ret = chdir(root);
		file_size = __le32_to_cpu(obj->info.object_compressed_size);
	} else {
		char *dot = strrchr(obj->name, '.');
		*dot = '\0';			/* We know there is a dot in the name */
		snprintf(name, sizeof(name) - 1, "%s.thumb.jpeg", obj->name);
		*dot = '.';
		name[sizeof(name) - 1] = '\0';
		ret = chdir(THUMB_LOCATION);
		file_size = __le32_to_cpu(obj->info.thumb_compressed_size);
	}
#else
	(void)thumb;
	strncpy(name, obj->name, sizeof(name) - 1);
	name[sizeof(name) - 1] = '\0';
	ret = chdir(root);
	file_size = __le32_to_cpu(obj->info.object_compressed_size);
#endif

	total = file_size + sizeof(*s_container);
	if (verbose)
		fprintf(stderr, "%s(): total %d\n", __func__, total);
	s_container->length = __cpu_to_le32(total);

	if (!ret)
		fd = open(name, O_RDONLY);
	if (ret < 0 || fd < 0) {
		make_response(s_container, r_container, PIMA15740_RESP_INCOMPLETE_TRANSFER,
			      sizeof(*s_container));
		return 0;
	}

	map = mmap(NULL, file_size, PROT_READ, MAP_SHARED, fd, 0);
	if (map == MAP_FAILED) {
		close(fd);
		make_response(s_container, r_container, PIMA15740_RESP_INCOMPLETE_TRANSFER,
			      sizeof(*s_container));
		return 0;
	}

	count = min(total, send_len);
	memcpy(send_buf + offset, map, count - offset);
	ret = bulk_write(send_buf, count);
	if (ret < 0) {
		errno = EPIPE;
		goto out;
	}
	total -= count;
	data = map + count - offset;
	send_len = 8 * 1024;

	while (total) {
		count = min(total, send_len);
		ret = bulk_write(data, count);
		if (ret < 0) {
			errno = EPIPE;
			goto out;
		}
		total -= count;
		data += count;
	}
	ret = 0;

out:
	munmap(map, file_size);
	close(fd);

	if (!ret)
		/* Prepare response */
		make_response(s_container, r_container, PIMA15740_RESP_OK, sizeof(*s_container));

	return ret;
}

static int send_storage_ids(void *recv_buf, void *send_buf, size_t send_len)
{
	struct ptp_container *s_container = send_buf;
	uint32_t *param;
	int ret;
	(void) send_len;

	s_container->type = __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
	s_container->length = __cpu_to_le32(20);
	param = (uint32_t *)s_container->payload;

	*param = __cpu_to_le32(1);
	*(param + 1) = __cpu_to_le32(STORE_ID);
	ret = bulk_write(send_buf, 20);
	if (ret < 0) {
		errno = EPIPE;
		return ret;
	}

	/* Prepare response */
	memcpy(send_buf, recv_buf, sizeof(*s_container));
	s_container->type = __cpu_to_le16(PTP_CONTAINER_TYPE_RESPONSE_BLOCK);
	s_container->code = __cpu_to_le16(PIMA15740_RESP_OK);
	s_container->length = __cpu_to_le32(sizeof(*s_container));

	return 0;
}

static int send_storage_info(void *recv_buf, void *send_buf, size_t send_len)
{
	struct ptp_container *r_container = recv_buf;
	struct ptp_container *s_container = send_buf;
	uint32_t *param;
	uint32_t store_id;
	unsigned long long bytes;
	int ret;
	size_t count;
	struct statfs fs;
	(void) send_len;

	param = (uint32_t *)r_container->payload;
	store_id = __le32_to_cpu(*param);

	if (verbose)
		fprintf(stderr, "%u bytes storage info\n", sizeof(storage_info));

	if (store_id != STORE_ID) {
		make_response(s_container, r_container,
			      PIMA15740_RESP_INVALID_STORAGE_ID, sizeof(*s_container));
		return 0;
	}

	ret = statfs(root, &fs);
	if (ret < 0) {
		make_response(s_container, r_container,
			      PIMA15740_RESP_ACCESS_DENIED, sizeof(*s_container));
		return 0;
	}

	if (verbose > 1)
		fprintf(stderr, "Block-size %d, total 0x%lx, free 0x%lx\n",
			fs.f_bsize, fs.f_blocks, fs.f_bfree);

	count = sizeof(storage_info) + sizeof(*s_container);

	s_container->type	= __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
	s_container->length	= __cpu_to_le32(count);

	bytes					= (unsigned long long)fs.f_bsize * fs.f_blocks;
	storage_info.max_capacity		= __cpu_to_le64(bytes);
	bytes					= (unsigned long long)fs.f_bsize * fs.f_bfree;
	storage_info.free_space_in_bytes	= __cpu_to_le64(bytes);
	storage_info.free_space_in_images	= __cpu_to_le32(PTP_PARAM_ANY);

	memcpy(send_buf + sizeof(*s_container), &storage_info, sizeof(storage_info));
	ret = bulk_write(s_container, count);
	if (ret < 0) {
		errno = EPIPE;
		return ret;
	}

	/* Prepare response */
	make_response(s_container, r_container, PIMA15740_RESP_OK, sizeof(*s_container));

	return 0;
}

#ifdef DEBUG
const char *oid[] =
{	"StorageID             ",
	"ObjectFormat          ",
	"ProtectionStatus      ",
	"ObjectCommpressedSize ",
	"ThumbFormat           ",
	"ThumbCommpressedSize  ",
	"ThumbPixWidth         ",
	"ThumbPixHeight        ",
	"ImagePixWidth         ",
	"ImagePixHeight        ",
	"ImageBitDepth         ",
	"ParentObject          ",
	"AssotiationType       ",
	"AssotiationDesc       ",
	"SequenceNumber        ",
	"Filename              ",
	"CaptureDate           ",
	"ModificationDate      ",
	"Keywords              ",
};

static void dump_object_info(const struct ptp_object_info *i)
{
	char buf[256];
	int idx;

	fprintf(stdout, "%s\n", __func__);
	fprintf(stdout, "-------------------------------\n");
	fprintf(stdout, "%s: 0x%.8x\n", oid[0], i->storage_id);
	fprintf(stdout, "%s: 0x%.4x\n", oid[1], i->object_format);
	fprintf(stdout, "%s: 0x%.4x\n", oid[2], i->protection_status);
	fprintf(stdout, "%s: %d\n",     oid[3], i->object_compressed_size);
	fprintf(stdout, "%s: 0x%.4x\n", oid[4], i->thumb_format);
	fprintf(stdout, "%s: %d\n",     oid[5], i->thumb_compressed_size);
	fprintf(stdout, "%s: %d\n",     oid[6], i->thumb_pix_width);
	fprintf(stdout, "%s: %d\n",     oid[7], i->thumb_pix_height);
	fprintf(stdout, "%s: %d\n",	oid[8], i->image_pix_width);
	fprintf(stdout, "%s: %d\n",	oid[9], i->image_pix_height);
	fprintf(stdout, "%s: %d\n",	oid[10], i->image_bit_depth);
	fprintf(stdout, "%s: 0x%.8x\n", oid[11], i->parent_object);
	fprintf(stdout, "%s: 0x%.4x\n", oid[12], i->association_type);
	fprintf(stdout, "%s: 0x%.8x\n", oid[13], i->association_desc);
	fprintf(stdout, "%s: %d\n",	oid[14], i->sequence_number);
	get_string(uc, (char *)buf, (const char *)&i->strings[1],
		   i->strings[0]);
	fprintf(stdout, "%s: %s, len: %d\n", oid[15], buf, i->strings[0]);
	idx = i->strings[0] * 2 + 1;
	if (i->strings[idx]) {
		get_string(uc, (char *)buf, (const char *)&i->strings[idx + 1],
			   i->strings[idx]);
		fprintf(stdout, "%s: %s, len: %d\n",
			oid[16], buf, i->strings[idx]);
	}
	fprintf(stdout, "-------------------------------\n");
}

#ifdef DEBUG
static void dump_obj(const char *s)
{
	struct obj_list *obj;
	GSList *iterator;

	printf("%s, object_number %d\n", s, object_number);

	GFOREACH(obj, images) {
		printf("obj: 0x%p, next 0x%p, handle %u, name %s\n",
			obj, obj->next, obj->handle, obj->name);
	}
	printf("\n");
}
#endif

static void delete_thumb(struct obj_list *obj)
{
#ifdef THUMB_SUPPORT
	char thumb[256];
	char *dot;

	if (__le16_to_cpu(obj->info.thumb_format) != PIMA15740_FMT_I_JFIF)
		return;

	dot = strrchr(obj->name, '.');
	if (!dot || dot == obj->name)
		return;

	*dot = 0;
	snprintf(thumb, sizeof(thumb), THUMB_LOCATION "%s.thumb.jpeg",
		 obj->name);
	*dot = '.';

	if (unlink(thumb))
		fprintf(stderr, "Cannot delete %s: %s\n",
			thumb, strerror(errno));
#else
	(void)obj;
#endif
}

static enum pima15740_response_code delete_file(const char *name)
{
	struct stat st;
	int ret;
	uid_t euid;
	gid_t egid;

	/* access() is unreliable on NFS, we use stat() instead */
	ret = stat(name, &st);
	if (ret < 0) {
		fprintf(stderr, "Cannot stat %s: %s\n", name, strerror(errno));
		return PIMA15740_RESP_GENERAL_ERROR;
	}

	euid = geteuid();
	if (euid == st.st_uid) {
		if (!(st.st_mode & S_IWUSR))
			return PIMA15740_RESP_OBJECT_WRITE_PROTECTED;
		goto del;
	}

	egid = getegid();
	if (egid == st.st_gid) {
		if (!(st.st_mode & S_IWGRP))
			return PIMA15740_RESP_OBJECT_WRITE_PROTECTED;
		goto del;
	}

	if (!(st.st_mode & S_IWOTH))
		return PIMA15740_RESP_OBJECT_WRITE_PROTECTED;

del:
	ret = unlink(name);
	if (ret) {
		fprintf(stderr, "Cannot delete %s: %s\n",
			name, strerror(errno));
		return PIMA15740_RESP_GENERAL_ERROR;
	}

	return PIMA15740_RESP_OK;
}

static int update_free_space(void)
{
	unsigned long long bytes;
	struct statfs fs;
	int ret;

	ret = statfs(root, &fs);
	if (ret < 0) {
		fprintf(stderr, "statfs %s: %s\n", root, strerror(errno));
		return ret;
	}

	if (verbose > 1)
		fprintf(stdout, "Block-size %d, total %d, free %d\n",
			fs.f_bsize, (int)fs.f_blocks, (int)fs.f_bfree);

	bytes = (unsigned long long)fs.f_bsize * fs.f_bfree;
	storage_info.free_space_in_bytes = __cpu_to_le64(bytes);
	return 0;
}

static void delete_object(void *recv_buf, void *send_buf)
{
	struct ptp_container *r_container = recv_buf;
	struct ptp_container *s_container = send_buf;
	enum pima15740_response_code code = PIMA15740_RESP_OK;
	uint32_t format, handle;
	struct obj_list *obj = NULL;
	GSList *iterator;
	uint32_t *param;
	unsigned long length;
	int ret = 0;

	length = __le32_to_cpu(r_container->length);

	param = (uint32_t *)r_container->payload;
	handle = __le32_to_cpu(*param);
	format = __le32_to_cpu(*(param + 1));

	if (length > 16 && format != PTP_PARAM_UNUSED) {
		/* ObjectFormatCode not supported */
		code = PIMA15740_RESP_SPECIFICATION_BY_FORMAT_NOT_SUPPORTED;
		goto resp;
	} else if (handle == 1 || handle == 2) {
		/* read-only /DCIM and /DCIM/100LINUX */
		code = PIMA15740_RESP_OBJECT_WRITE_PROTECTED;
		goto resp;
	}

	ret = chdir(root);
	if (ret) {
		fprintf(stderr, "chdir %s: %s\n", root, strerror(errno));
		code = PIMA15740_RESP_GENERAL_ERROR;
		goto resp;
	}

	if (handle == PTP_PARAM_ANY) {
		int partial = 0;

		if (!g_slist_length(images) + 2) {
			code = PIMA15740_RESP_OK;
			goto resp;
		}

		GFOREACH(obj, images) {
			code = delete_file(obj->name);
			if (code == PIMA15740_RESP_OK) {
				delete_thumb(obj);
				images = g_slist_remove(images, obj);
				free(obj);
			} else {
				partial++;
			}
		}

		if (partial)
			code = PIMA15740_RESP_PARTIAL_DELETION;
	} else {
		GFOREACH(obj, images) {
			if (obj->handle == handle)
				break;
		}

		if (obj) {
			code = delete_file(obj->name);
			if (code == PIMA15740_RESP_OK) {
				delete_thumb(obj);
				images = g_slist_remove(images, obj);
				free(obj);
			}
		} else {
			code = PIMA15740_RESP_INVALID_OBJECT_HANDLE;
		}
	}

	ret = update_free_space();
	if (ret < 0)
		code = PIMA15740_RESP_STORE_NOT_AVAILABLE;

resp:
	make_response(s_container, r_container, code, sizeof(*s_container));
}

static int read_container(void *recv_buf, size_t recv_size)
{
	size_t count = 0;
	int ret;

	do {
		ret = read(bulk_out, recv_buf + count, recv_size - count);
		if (ret < 0) {
			if (errno != EINTR)
				return ret;

			/* Need to wait for control thread to finish reset */
			sem_wait(&reset);
		} else {
			count += ret;
		}
	} while (count < sizeof(struct ptp_container));

	if (verbose > 1)
		fprintf(stderr, "%s: read %u bytes, count %d length %u\n",
			__func__, ret, (unsigned int)count, (unsigned int)recv_size);

	return count;
}

static int process_send_object_info(void *recv_buf, void *send_buf)
{
	struct ptp_container *r_container = recv_buf;
	struct ptp_container *s_container = send_buf;
	enum pima15740_response_code code = PIMA15740_RESP_OK;
	struct ptp_object_info *info;
	uint32_t *param, p1, p2;
	size_t new_info_size, alloc_size;
	char lock_file[256];
	char new_file[256];
	mode_t mode;
	int fd, fd_new;
	int ret = 0, len;
	char fs_buf[32];

	param = (uint32_t *)r_container->payload;
	p1 = __le32_to_cpu(*param);
	p2 = __le32_to_cpu(*(param + 1));

	if (verbose) {
		fprintf(stderr, "store_id 0x%lx, parent handle 0x%lx\n",
			(unsigned long)p1, (unsigned long)p2);
	}

	/* Read ObjectInfo comming in the data phase */
	ret = read_container(recv_buf, BUF_SIZE);
	if (ret < 0) {
		code = PIMA15740_RESP_INCOMPLETE_TRANSFER;
		goto resp;
	}

	if (p1 != STORE_ID) {
		code = PIMA15740_RESP_INVALID_STORAGE_ID;
		goto resp;
	}
	if (p2 != 2) {
		code = PIMA15740_RESP_SPECIFICATION_OF_DESTINATION_UNSUPPORTED;
		goto resp;
	}

	new_info_size = ret - sizeof(*r_container);
	alloc_size = (new_info_size - sizeof(struct ptp_object_info)) +
		     sizeof(struct obj_list);
	if (verbose) {
		fprintf(stdout, "new object_info size %d\n",
			(unsigned int)new_info_size);
	}

	info = (struct ptp_object_info *)r_container->payload;
#ifdef DEBUG
	dump_object_info(info);
#endif

	switch (info->object_format) {
	case PIMA15740_FMT_A_UNDEFINED:
	case PIMA15740_FMT_A_TEXT:
	case PIMA15740_FMT_I_EXIF_JPEG:
	case PIMA15740_FMT_I_TIFF:
		code = PIMA15740_RESP_OK;
		break;
	default:
		code = PIMA15740_RESP_INVALID_OBJECT_FORMAT_CODE;
		goto resp;
		break;
	}

	if (((uint64_t)__le32_to_cpu(info->object_compressed_size)) >
	    __le64_to_cpu(storage_info.free_space_in_bytes)) {
		code = PIMA15740_RESP_STORE_FULL;
		if (verbose) {
			fprintf(stdout, "no space: free %lld, req. %d\n",
				storage_info.free_space_in_bytes,
				info->object_compressed_size);
		}
		goto resp;
	}

	ret = chdir(root);
	if (ret) {
		fprintf(stderr, "chdir: %s: %s\n", root, strerror(errno));
		code = PIMA15740_RESP_STORE_NOT_AVAILABLE;
		goto resp;
	}

	if (object_info_p) {
		/* replace previously allocated info, free resources */
		int len;
		char c;

		len = strlen(object_info_p->name);
		if (len > 250) {
			c = object_info_p->name[250];
			object_info_p->name[250] = '\0';
		}
		snprintf(lock_file, 256, "%s.lock", object_info_p->name);
		if (len > 250)
			object_info_p->name[250] = c;

		ret = unlink(lock_file);
		if (ret < 0)
			fprintf(stderr, "can't remove %s: %s",
				lock_file, strerror(errno));

		ret = unlink(object_info_p->name);
		if (ret < 0)
			fprintf(stderr, "can't remove %s: %s",
				lock_file, strerror(errno));

		free(object_info_p);
		last_object_number--;
	}

	object_info_p = malloc(alloc_size);
	if (!object_info_p) {
		perror("object info allocation failed");
		code = PIMA15740_RESP_GENERAL_ERROR;
		goto resp;
	}

	ret = get_string(uc, (char *)new_file, (const char *)&info->strings[1],
			 info->strings[0]);
	if (ret < 0) {
		fprintf(stderr, "Filename conversion failed: %d\n", ret);
		code = PIMA15740_RESP_GENERAL_ERROR;
		goto err;
	}

	snprintf(lock_file, 255, "%s.lock", new_file);

	mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
	if (info->protection_status & 0x0001)
		mode &= ~S_IWUSR;

	if (verbose > 1) {
		fprintf(stdout, "Lock Filename: %s\n", lock_file);
		fprintf(stdout, "New Filename: %s, size %d\n",
			new_file, info->object_compressed_size);
	}

	fd = open(lock_file, O_CREAT | O_EXCL | O_WRONLY, mode);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", lock_file, strerror(errno));
		if (errno == EEXIST)
			code = PIMA15740_RESP_STORE_NOT_AVAILABLE;
		else
			code = PIMA15740_RESP_GENERAL_ERROR;

		goto err;
	}

	fd_new = open(new_file, O_CREAT | O_EXCL | O_WRONLY, mode);
	if (fd_new < 0) {
		fprintf(stderr, "open: lock file %s, new file %s: %s\n",
			lock_file, new_file, strerror(errno));
		if (errno == EEXIST)
			code = PIMA15740_RESP_STORE_NOT_AVAILABLE;
		else
			code = PIMA15740_RESP_GENERAL_ERROR;

		close(fd);
		ret = unlink(lock_file);
		if (ret < 0)
			perror ("can't remove lock file");

		goto err;
	}

	len = snprintf(fs_buf, sizeof(fs_buf), "%d",
		       info->object_compressed_size);
	ret = ftruncate(fd, len);
	if (ret < 0) {
		fprintf(stderr, "ftruncate for lock file: %s: %s\n",
			lock_file, strerror(errno));
		goto err_del;
	}

	ret = write(fd, fs_buf, len);
	if (ret < 0) {
		fprintf(stderr, "write new file size to %s: %s\n",
			lock_file, strerror(errno));
		goto err_del;
	}

	ret = ftruncate(fd_new, info->object_compressed_size);
	if (ret < 0) {
		fprintf(stderr, "ftruncate: %s: %s\n",
			new_file, strerror(errno));
		goto err_del;
	}

	memcpy(&object_info_p->info, info, new_info_size);
	snprintf(object_info_p->name, 256, "%s", new_file);
	object_info_p->handle			= ++last_object_number;
	object_info_p->info_size		= new_info_size;
	object_info_p->info.storage_id		= __cpu_to_le32(STORE_ID);
	object_info_p->info.parent_object	= __cpu_to_le32(2);	/* Fixed /dcim/xxx/ */
	object_info_p->info.association_type	= __cpu_to_le16(0);
	object_info_p->info.association_desc	= __cpu_to_le32(0);
	object_info_p->info.sequence_number	= __cpu_to_le32(0);

	param = (uint32_t *)&s_container->payload[0];
	param[0] = __cpu_to_le32(STORE_ID);
	param[1] = __cpu_to_le32(2);
	param[2] = __cpu_to_le32(last_object_number);

	close(fd);
	close(fd_new);
resp:
	make_response(s_container, r_container, code, sizeof(*s_container) + 12);
	return 0;

err_del:
	code = PIMA15740_RESP_STORE_FULL;
	close(fd);
	close(fd_new);

	ret = unlink(new_file);
	if (ret < 0)
		fprintf(stderr, "can't remove %s: %s\n",
			new_file, strerror(errno));

	ret = unlink(lock_file);
	if (ret < 0)
		fprintf(stderr, "can't remove %s: %s\n",
			lock_file, strerror(errno));
err:
	free(object_info_p);
	object_info_p = 0;
	make_response(s_container, r_container, code, sizeof(*s_container));
	return -1;
}

#ifdef THUMB_SUPPORT
static int generate_thumb(const char *);
#endif
static int process_send_object(void *recv_buf, void *send_buf)
{
	struct ptp_container *r_container = (struct ptp_container *)recv_buf;
	struct ptp_container *s_container = send_buf;
	enum pima15740_response_code code = PIMA15740_RESP_OK;
	struct obj_list *oi;
	int length;
	void *map;
	int offset = sizeof(*r_container);
	int fd, cnt = 0, obj_size, ret;
	char lock_file[256];
	int len;
	char c;

	/* start reading data phase */
	ret = read_container(recv_buf, BUF_SIZE);
	if (ret < 0) {
		code = PIMA15740_RESP_INCOMPLETE_TRANSFER;
		goto resp;
	}

	cnt = ret - offset;
	length = __le32_to_cpu(r_container->length) - offset;

	if (!object_info_p) {
		/* get remaining data, end data phase */
		while (cnt < length) {
			ret = read(bulk_out, recv_buf, BUF_SIZE);
			if (ret < 0) {
				errno = EPIPE;
				return -1;
			}
			cnt += ret;
		}
		code = PIMA15740_RESP_NO_VALID_OBJECT_INFO;
		goto resp;
	}

	oi = object_info_p;
	obj_size = oi->info.object_compressed_size;

	if (length != obj_size) {
		/* less or more data as at SendObjectInfo */
		while (cnt < length) {
			ret = read(bulk_out, recv_buf, BUF_SIZE);
			if (ret < 0) {
				errno = EPIPE;
				return -1;
			}
			cnt += ret;
		}

		if (length < obj_size)
			code = PIMA15740_RESP_INCOMPLETE_TRANSFER;
		else
			code = PIMA15740_RESP_STORE_FULL;

		goto resp;
	}

	/* empty file was send, don't need to write something */
	if (!obj_size) {
		code = PIMA15740_RESP_OK;
		goto link;
	}

	fd = open(oi->name, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "%s: open %s: %s\n", __func__,
			oi->name, strerror(errno));
		code = PIMA15740_RESP_STORE_FULL;
		goto resp;
	}

	map = mmap(NULL, obj_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (map == MAP_FAILED) {
		fprintf(stderr, "%s: mmap %s: %s\n", __func__,
			oi->name, strerror(errno));
		code = PIMA15740_RESP_STORE_FULL;
		close(fd);
		goto resp;
	}

	/* store first data block */
	memcpy(map, recv_buf + offset, cnt);

	/* more data? */
	if (obj_size > cnt) {
		unsigned int rest, recv_len;
		void *data = map + cnt;

		if (verbose) {
			fprintf(stderr, "Reading rest %d of %d\n",
				obj_size - cnt, obj_size);
		}
		rest = obj_size - cnt;
		recv_len = 8192;
		while (rest) {
			cnt = min(rest, recv_len);
			ret = bulk_read(data, cnt);
			if (ret < 0) {
				fprintf(stderr, "%s: reading data for %s failed: %s\n",
					__func__, object_info_p->name, strerror(errno));
				code = PIMA15740_RESP_INCOMPLETE_TRANSFER;
				munmap(map, obj_size);
				close(fd);
				errno = EPIPE;
				return ret;
			}
			rest -= ret;
			data += ret;
		}
	}

	munmap(map, obj_size);
	close(fd);

#ifdef THUMB_SUPPORT
	if (oi->info.object_format != PIMA15740_FMT_A_UNDEFINED &&
	    oi->info.object_format != PIMA15740_FMT_A_TEXT) {
		ret = generate_thumb(object_info_p->name);
		if (ret > 0) {
			oi->info.thumb_format = __cpu_to_le16(PIMA15740_FMT_I_JFIF);
			oi->info.thumb_compressed_size = __cpu_to_le32(ret);
			oi->info.thumb_pix_width = __cpu_to_le32(THUMB_WIDTH);
			oi->info.thumb_pix_height = __cpu_to_le32(THUMB_HEIGHT);
		}
	}
#endif

link:
	object_info_p->next = 0;
	images = g_slist_append(images, object_info_p);

	len = strlen(object_info_p->name);
	if (len > 250) {
		c = object_info_p->name[250];
		object_info_p->name[250] = '\0';
	}
	snprintf(lock_file, 250, "%s.lock", object_info_p->name);
	if (len > 250)
		object_info_p->name[250] = c;

	ret = unlink(lock_file);
	if (ret < 0)
		fprintf(stderr, "can't remove %s: %s",
			lock_file, strerror(errno));

	object_number++;
	object_info_p = 0;
#ifdef DEBUG
	dump_obj("after link");
#endif

	ret = update_free_space();
	if (ret < 0)
		code = PIMA15740_RESP_STORE_NOT_AVAILABLE;

resp:
	make_response(s_container, r_container, code, sizeof(*s_container));

	return 0;
}

static int process_one_request(void *recv_buf, size_t *recv_size, void *send_buf, size_t *send_size)
{
	struct ptp_container *r_container = recv_buf;
	struct ptp_container *s_container = send_buf;
	uint32_t *param, p1, p2, p3;
	unsigned long length = *recv_size, type = 0, code = 0, id = 0;
	size_t count = 0;
	int ret;

	do {
		ret = read(bulk_out, recv_buf + count, *recv_size - count);
		if (ret < 0) {
			if (errno != EINTR)
				return ret;

			/* Need to wait for control thread to finish reset */
			sem_wait(&reset);
		} else {
			count += ret;
			if (count >= sizeof(*s_container)) {
				length	= __le32_to_cpu(r_container->length);
				type	= __le16_to_cpu(r_container->type);
				code	= __le16_to_cpu(r_container->code);
				id	= __le32_to_cpu(r_container->id);
			}
		}
	} while (count < length);

	if (count > length) {
		/* TODO: have to stall according to Figure 7.2-1? */
		fprintf(stderr, "BULK-OUT ERROR: received %u byte, expected %lu\n",
			(unsigned int)count, length);
		errno = EPIPE;
		return -1;
	}

	memcpy(send_buf, recv_buf, sizeof(*s_container));

	if (verbose)
		fprintf(stderr, "BULK-OUT Received %lu byte, type %lu, code 0x%lx, id %lu\n",
			length, type, code, id);

	ret = -1;

	switch (type) {
	case PTP_CONTAINER_TYPE_COMMAND_BLOCK:
		switch (code) {
		case PIMA15740_OP_GET_DEVICE_INFO:
			CHECK_COUNT(count, 12, 12, "GET_DEVICE_INFO");

			if (verbose)
				fprintf(stderr, "%u bytes device info\n", sizeof(dev_info));
			count = sizeof(dev_info) + sizeof(*s_container);

			/* First part: data block */
			s_container->type = __cpu_to_le16(PTP_CONTAINER_TYPE_DATA_BLOCK);
			s_container->length = __cpu_to_le32(count);
			memcpy(send_buf + sizeof(*s_container), &dev_info, sizeof(dev_info));
			ret = bulk_write(s_container, count);
			if (ret < 0)
				return ret;

			/* Second part: response block */
			s_container = send_buf + count;
			make_response(s_container, r_container, PIMA15740_RESP_OK, sizeof(*s_container));

			break;
		case PIMA15740_OP_OPEN_SESSION:
			CHECK_COUNT(count, 16, 16, "OPEN_SESSION");

			ret = sizeof(*s_container);
			param = (uint32_t *)r_container->payload;
			p1 = __le32_to_cpu(*param);
			if (verbose)
				fprintf(stderr, "OpenSession %d\n", p1);
			/* No multiple sessions. */
			if (session > 0) {
				/* already open */
				code = PIMA15740_RESP_SESSION_ALREADY_OPEN;
				*param = __cpu_to_le32(session);
				ret += sizeof(*param);
			} else if (!p1) {
				code = PIMA15740_RESP_INVALID_PARAMETER;
			} else {
				code = PIMA15740_RESP_OK;
				session = p1;
			}
			make_response(s_container, r_container, code, ret);
			count = 0;
			break;
		case PIMA15740_OP_CLOSE_SESSION:
			CHECK_COUNT(count, 12, 12, "CLOSE_SESSION");

			if (session > 0) {
				code = PIMA15740_RESP_OK;
				session = -EINVAL;
			} else {
				code = PIMA15740_RESP_SESSION_NOT_OPEN;
			}
			make_response(s_container, r_container, code, sizeof(*s_container));
			ret = 0;
			count = 0;
			break;
		case PIMA15740_OP_GET_OBJECT_HANDLES:
			CHECK_COUNT(count, 16, 24, "GET_OBJECT_HANDLES");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = send_object_handles(recv_buf, send_buf, *send_size);
			count = ret; /* even if ret is negative, handled below */
			break;
		case PIMA15740_OP_GET_OBJECT_INFO:
			CHECK_COUNT(count, 16, 16, "GET_OBJECT_INFO");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = send_object_info(recv_buf, send_buf, *send_size);
			count = ret; /* even if ret is negative, handled below */
			break;
		case PIMA15740_OP_GET_STORAGE_IDS:
			CHECK_COUNT(count, 12, 12, "GET_STORAGE_IDS");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = send_storage_ids(recv_buf, send_buf, *send_size);
			count = ret; /* even if ret is negative, handled below */
			break;
		case PIMA15740_OP_GET_STORAGE_INFO:
			CHECK_COUNT(count, 16, 16, "GET_STORAGE_INFO");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = send_storage_info(recv_buf, send_buf, *send_size);
			count = ret; /* even if ret is negative, handled below */
			break;
		case PIMA15740_OP_GET_OBJECT:
			CHECK_COUNT(count, 16, 16, "GET_OBJECT");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = send_object_or_thumb(recv_buf, send_buf, *send_size, 0);
			count = ret; /* even if ret is negative, handled below */
			break;
		case PIMA15740_OP_GET_NUM_OBJECTS:
			CHECK_COUNT(count, 16, 24, "GET_NUM_OBJECTS");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = 12;
			param = (uint32_t *)r_container->payload;
			p1 = __le32_to_cpu(*param);
			p2 = __le32_to_cpu(*(param + 1));
			p3 = __le32_to_cpu(*(param + 2));
			if (p1 != PTP_PARAM_ANY && p1 != STORE_ID)
				code = PIMA15740_RESP_INVALID_STORAGE_ID;
			else if (count > 16 && p2 != PTP_PARAM_UNUSED && p2 != PTP_PARAM_ANY)
				code = PIMA15740_RESP_SPECIFICATION_BY_FORMAT_NOT_SUPPORTED;
			else if (count > 20 && p3 != PTP_PARAM_UNUSED) {
				if (!object_handle_valid(p3))
					code = PIMA15740_RESP_INVALID_OBJECT_HANDLE;
				else if (p3 == PTP_PARAM_ANY || p3 == 1) {
					/* root or DCIM - report one handle */
					code = PIMA15740_RESP_OK;
					ret += sizeof(*param);
					*param = __cpu_to_le32(1);
				} else if (p3 == 2) {
					/* Contents of 100LINUX */
					code = PIMA15740_RESP_OK;
					ret += sizeof(*param);
					*param = __cpu_to_le32(g_slist_length(images));
				} else
					code = PIMA15740_RESP_INVALID_PARENT_OBJECT;
			} else {
				/* No parent Association specified or 0 */
				code = PIMA15740_RESP_OK;
				ret += sizeof(*param);
				*param = __cpu_to_le32(g_slist_length(images) + 2);
			}
			make_response(s_container, r_container, code, ret);
			count = 0;
			break;
		case PIMA15740_OP_GET_THUMB:
			CHECK_COUNT(count, 16, 16, "GET_THUMB");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = send_object_or_thumb(recv_buf, send_buf, *send_size, 1);
			count = ret; /* even if ret is negative, handled below */
			break;
		case PIMA15740_OP_DELETE_OBJECT:
			CHECK_COUNT(count, 16, 20, "DELETE_OBJECT");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			delete_object(recv_buf, send_buf);
			count = 0;
			ret = 0;
			break;
		case PIMA15740_OP_SEND_OBJECT_INFO:
			CHECK_COUNT(count, 12, 20, "SEND_OBJECT_INFO");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			process_send_object_info(recv_buf, send_buf);
			count = 0;
			ret = 0;
			break;
		case PIMA15740_OP_SEND_OBJECT:
			CHECK_COUNT(count, 12, 12, "SEND_OBJECT");
			CHECK_SESSION(s_container, r_container, &count, &ret);

			ret = process_send_object(recv_buf, send_buf);
			count = 0;
			break;
		}
		break;
	}

	if (ret < 0) {
		if (errno == EPIPE)
			return -1;

		if (verbose)
			fprintf(stderr, "Unsupported type %lu code 0x%lx\n", type, code);
		errno = EOPNOTSUPP;
		make_response(s_container, r_container,
			      PIMA15740_RESP_OPERATION_NOT_SUPPORTED, sizeof(*s_container));
		count = 0;
	}

	/* send out response at send_buf + count */
	s_container = send_buf + count;
	length = __le32_to_cpu(s_container->length);
	return bulk_write(s_container, length);
}

static void cleanup_endpoint(int ep_fd, char *ep_name)
{
	int ret;

	if(ep_fd < 0)
		return;

	ret = ioctl(ep_fd, FUNCTIONFS_FIFO_STATUS);
	if(ret < 0)
	{
		//ENODEV reported after disconnect
		if(errno != ENODEV)
			fprintf(stderr, "%s, %s: get fifo status(%s): %s \n", __FILE__, __FUNCTION__, ep_name, strerror(errno));
	}
	else if(ret)
	{
		if (verbose)
			fprintf(stderr, "%s, %s: %s: unclaimed = %d \n", __FILE__, __FUNCTION__, ep_name, ret);
		if(ioctl(ep_fd, FUNCTIONFS_FIFO_FLUSH) < 0)
			fprintf(stderr, "%s, %s: %s: fifo flush \n", __FILE__, __FUNCTION__, ep_name);
	}

	if(close(ep_fd) < 0)
		fprintf(stderr, "%s, %s: %s: close \n", __FILE__, __FUNCTION__, ep_name);
}

/*
 * communication thread cleanup actions
 */
static void cleanup_bulk_thread(void *arg)
{
	(void) arg;

	cleanup_endpoint(bulk_out, "out");
	cleanup_endpoint(bulk_in, "in");
	cleanup_endpoint(interrupt, "interrupt");
}

static void *bulk_thread(void *param)
{
	void *recv_buf, *send_buf;
	int ret;
	size_t s_size = BUF_SIZE, r_size = BUF_SIZE;
	(void) param;

	pthread_cleanup_push(cleanup_bulk_thread, NULL);

	recv_buf = malloc(BUF_SIZE);
	send_buf = malloc(BUF_SIZE);
	if (!recv_buf || !send_buf) {
		if (verbose)
			fprintf(stderr, "No memory!\n");
		goto done;
	}

	do {
		ret = process_one_request(recv_buf, &r_size, send_buf, &s_size);
		if (ret < 0 && errno == EPIPE) {
			/* TODO: Have to stall and wait to be unstalled / exit
			 * thread to be restarted */
			fprintf(stderr, "Protocol error!\n");
			break;
		}

		pthread_testcancel();
	} while (ret >= 0);

	pthread_cleanup_pop(1);
done:
	free(recv_buf);
	free(send_buf);
	pthread_exit(NULL);
}

static int start_io(void)
{
	int ret;

	if (verbose)
		fprintf(stderr, "Start bulk EPs\n");

	if (bulk_in >= 0 && bulk_out >= 0)
		return 0;

	bulk_in = open(FFS_PTP_IN, O_RDWR);
	if (bulk_in < 0)
		return bulk_in;

	bulk_out = open(FFS_PTP_OUT, O_RDWR);
	if (bulk_out < 0)
		return bulk_out;

	interrupt = open(FFS_PTP_INT, O_RDWR);
	if (interrupt < 0)
		return interrupt;

	status = PTP_IDLE;

	ret = pthread_create(&bulk_pthread, NULL, bulk_thread, NULL);
	if (ret < 0) {
		perror ("can't create bulk thread");
		return ret;
	}

	return 0;
}

static void stop_io(void)
{
	fprintf(stderr, "Stop bulk EPs\n");

	if (bulk_in < 0 || bulk_out < 0)
		return;

	pthread_cancel(bulk_pthread);
	pthread_join(bulk_pthread, NULL);

	status = PTP_WAITCONFIG;

	close(bulk_out);
	bulk_out = -EINVAL;
	close(bulk_in);
	bulk_in = -EINVAL;
	close(interrupt);
	interrupt = -EINVAL;
}

static void init_device(void)
{
	int		ret;

	control = open(FFS_PTP_EP0, O_RDWR);
	if (control < 0) {
		perror(FFS_PTP_EP0);
		control = -errno;
		return;
	}

	ret = write(control, &descriptors, sizeof(descriptors));
	if (ret < 0) {
		perror("write dev descriptors");
		close(control);
		control = -errno;
		return;
	}
	ret = write(control, &strings, sizeof(strings));
	if (ret < 0) {
		perror("write dev strings");
		close(control);
		control = -errno;
		return;
	}

	return;
}

/*-------------------------------------------------------------------------*/

static int reset_interface(void)
{
	/* just reset toggle/halt for the interface's endpoints */
	int err;

	if (status == PTP_WAITCONFIG)
		return 0;

	sem_init(&reset, 0, 0);

	pthread_kill(bulk_pthread, SIGINT);

	err = ioctl(bulk_in, FUNCTIONFS_CLEAR_HALT);
	if (err < 0)
		perror("reset source fd");

	err = ioctl(bulk_out, FUNCTIONFS_CLEAR_HALT);
	if (err < 0)
		perror("reset sink fd");

	sem_post(&reset);

	/* FIXME eventually reset the status endpoint too */

	/* Always return "success"... */
	return 0;
}

static void handle_control(struct usb_ctrlrequest *setup)
{
	int		err;
	uint8_t		buf[256];
	uint16_t	value, index, length;

	value = __le16_to_cpu(setup->wValue);
	index = __le16_to_cpu(setup->wIndex);
	length = __le16_to_cpu(setup->wLength);

	if (verbose)
		fprintf(stderr, "SETUP %02x.%02x "
				"v%04x i%04x %d\n",
			setup->bRequestType, setup->bRequest,
			value, index, length);

	/*
	if ((setup->bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		goto special;
	*/

	switch (setup->bRequest) {
	/* Still Image class-specific requests */
	case USB_REQ_PTP_CANCEL_REQUEST:
		return;
	case USB_REQ_PTP_GET_EXTENDED_EVENT_DATA:
		/* Optional, may stall */
		goto stall;
	case USB_REQ_PTP_DEVICE_RESET_REQUEST:
		if (setup->bRequestType != 0x21
				|| index != 0
				|| value != 0)
			goto stall;

		err = reset_interface();
		if (err)
			goto stall;

		/* ... and ack (a write would stall) */
		err = read(control, &err, 0);
		if (err)
			perror("ack DEVICE_RESET_REQUEST");
		return;
	case USB_REQ_PTP_GET_DEVICE_STATUS_REQUEST:
		if (setup->bRequestType != 0xa1
				|| index != 0
				|| value != 0)
			goto stall;
		else {
			uint16_t resp_ok[] = {
				__constant_cpu_to_le16(4),
				__constant_cpu_to_le16(PIMA15740_RESP_OK),
			};
			memcpy(buf, resp_ok, 4);
			err = write(control, buf, 4);
			if (err != 4)
				fprintf(stderr, "DEVICE_STATUS_REQUEST %d\n", err);
		}

		return;
	default:
		goto stall;
	}

stall:
	if (verbose)
		fprintf(stderr, "... protocol stall %02x.%02x\n",
			setup->bRequestType, setup->bRequest);

	/* non-iso endpoints are stalled by issuing an i/o request
	 * in the "wrong" direction.  ep0 is special only because
	 * the direction isn't fixed.
	 */
	if (setup->bRequestType & USB_DIR_IN)
		err = read(control, &err, 0);
	else
		err = write(control, &err, 0);
	if (err != -1)
		fprintf(stderr, "can't stall ep0 for %02x.%02x\n",
			setup->bRequestType, setup->bRequest);
	else if (errno != EL2HLT)
		perror("ep0 stall");
}

static int read_control(void)
{
	struct usb_functionfs_event event[NEVENT];
	int i, nevent, ret;

	static const char * const names[] = {
		[FUNCTIONFS_BIND] = "BIND",
		[FUNCTIONFS_UNBIND] = "UNBIND",
		[FUNCTIONFS_ENABLE] = "ENABLE",
		[FUNCTIONFS_DISABLE] = "DISABLE",
		[FUNCTIONFS_SETUP] = "SETUP",
		[FUNCTIONFS_SUSPEND] = "SUSPEND",
		[FUNCTIONFS_RESUME] = "RESUME",
	};

	ret = read(control, &event, sizeof(event));
	if (ret < 0) {
		if (errno == EAGAIN) {
			sleep(1);
			return ret;
		}
		perror("ep0 read after poll");
		return ret;
	}
	nevent = ret / sizeof event[0];

	for (i = 0; i < nevent; i++) {
		if (verbose)
			fprintf(stderr,
				"Event %s,%d", names[event->type], event[i].type);

		switch (event[i].type) {
		case FUNCTIONFS_BIND:
			break;
		case FUNCTIONFS_UNBIND:
			break;
		case FUNCTIONFS_ENABLE:
			start_io();
			break;
		case FUNCTIONFS_DISABLE:
			stop_io();
			break;
		case FUNCTIONFS_SETUP:
			handle_control(&event[i].u.setup);
			break;
		case FUNCTIONFS_SUSPEND:
			break;
		case FUNCTIONFS_RESUME:
			break;
			break;
		default:
			fprintf(stderr,
				"* unhandled event %d\n",
				event[i].type);
		}
	}

	return ret;
}

static int main_loop(void)
{
	struct pollfd ep_poll[1];
	int ret;

	do {
		/* Always listen on control */
		ep_poll[0].fd = control;
		ep_poll[0].events = POLLIN | POLLHUP;

		ret = poll(ep_poll, 1, -1);
		if (ret < 0) {
			perror("poll");
			break;
		}

		/* TODO: What to do with HUP? */
		if (ep_poll[0].revents & POLLIN) {
			ret = read_control();
			if (ret < 0) {
				if (errno == EAGAIN)
					continue;
				goto done;
			}
		}
	} while (1);

	return 0;

done:
	switch (status) {
	case PTP_IDLE:
	case PTP_DATA_OUT:
	case PTP_DATA_IN:
	case PTP_DATA_READY:
		stop_io();
	case PTP_WAITCONFIG:
		break;
	}

	return ret;
}

/*-------------------------------------------------------------------------*/

static size_t put_string(iconv_t ic, char *buf, const char *s, size_t len)
{
	char *in = (char *)s;
	size_t ret, inl = len, outl = len * 2;

	ret = iconv(ic, &in, &inl, &buf, &outl);
	if (inl || outl)
		fprintf(stdout, "iconv() error %d: %u input / %u output bytes left!\n",
			errno, (unsigned int)inl, (unsigned int)outl);

	return ret;
}

static size_t get_string(iconv_t ic, char *buf, const char *s, size_t len)
{
	char *in = (char *)s;
	size_t ret, inl = len * 2, outl = len;

	ret = iconv(ic, &in, &inl, &buf, &outl);
	if (inl || outl)
		fprintf(stdout, "iconv() error %d: %u input / %u output bytes left!\n",
			errno, (unsigned int)inl, (unsigned int)outl);

	return ret;
}

static void clean_up(const char *path)
{
	struct dirent *dentry;
	DIR *d;
	char file_name[256];
	int fd, ret;
	char *endptr;
	char fs_buf[32];

	ret = chdir(path);
	if (ret < 0)
		return;

	d = opendir(".");

	while ((dentry = readdir(d))) {
		struct stat fstat;
		char *dot;
		unsigned long lsize, fsize;

		dot = strrchr(dentry->d_name, '.');

		if (!dot || dot == dentry->d_name)
			continue;

		if (strcasecmp(dot, ".lock"))
			continue;

		*dot = '\0';
		snprintf(file_name, sizeof(file_name), "%s", dentry->d_name);
		*dot = '.';

		fd = open(dentry->d_name, O_RDONLY);
		if (fd < 0) {
			fprintf(stderr, "%s: open %s: %s\n",
				__func__, dentry->d_name, strerror(errno));
			continue;
		}

		memset(fs_buf, 0, sizeof(fs_buf));
		ret = read(fd, fs_buf, sizeof(fs_buf));
		if (ret < 0) {
			fprintf(stderr, "%s: read %s: %s\n",
				__func__, dentry->d_name, strerror(errno));
			close(fd);
			continue;
		}
		close(fd);

		if (ret) {
			lsize = strtoul(fs_buf, &endptr, 10);
			fprintf(stdout, "%s: *endptr %x\n", __func__, *endptr);
			/*
			 * if not entire string is valid, then this file was not
			 * created by ptp, so skip it.
			 */
			if (fs_buf == endptr) {
				fprintf(stderr, "%s: can't get size of locked "
					"file %s\n", __func__, dentry->d_name);
				continue;
			}
			if (*endptr != 0)
				continue;
		} else {
			/* delete empty lock */
			ret = unlink(dentry->d_name);
			if (ret < 0)
				fprintf(stderr, "%s: %s: %s\n",
					__func__, dentry->d_name,
					strerror(errno));
			continue;
		}

		ret = stat(file_name, &fstat);
		if (ret < 0) {
			/* no corresponding object file, so delete lock file */
			fprintf(stderr, "%s: stat %s: %s\n",
				__func__, file_name, strerror(errno));
			ret = unlink(dentry->d_name);
			if (ret < 0)
				fprintf(stderr, "%s: %s: %s\n",
					__func__, dentry->d_name,
					strerror(errno));
			continue;
		}

		fsize = fstat.st_size;
		if (lsize == fsize) {
			/* locked file size is the same as the reserved size,
			 * but lock file was not deleted. This means transaction
			 * was not completed, so delete both, lock file and
			 * appropriate object file.
			 */
			if (verbose)
				printf("remove %s, %s\n",
					dentry->d_name, file_name);
			ret = unlink(dentry->d_name);
			if (ret < 0)
				fprintf(stderr, "%s: %s: %s\n",
					__func__, dentry->d_name, strerror(errno));
			ret = unlink(file_name);
			if (ret < 0)
				fprintf(stderr, "%s: %s: %s\n",
					__func__, file_name, strerror(errno));
		} else {
			/* no corresponding object file, so delete lock file */
			if (verbose)
				printf("remove %s\n", dentry->d_name);
			ret = unlink(dentry->d_name);
			if (ret < 0)
				fprintf(stderr, "%s: %s: %s\n",
					__func__, dentry->d_name, strerror(errno));
		}
	}
	closedir(d);
}

static int enum_objects(const char *path)
{
	struct dirent *dentry;
	char /*creat[32], creat_ucs2[64], */mod[32], mod_ucs2[64], fname_ucs2[512];
	DIR *d;
	int ret;
	struct obj_list *obj;
	/* First two handles used for /DCIM/PTP_MODEL_DIR */
	uint32_t handle = 2;

	ret = chdir(path);
	if (ret < 0)
		return ret;

	d = opendir(".");

	while ((dentry = readdir(d))) {
		struct stat fstat;
		char *dot;
		size_t namelen, datelen, osize;
		enum pima15740_data_format format, thumb_format;
		struct tm mod_tm;
		int thumb_size = 0, thumb_width, thumb_height;

		dot = strrchr(dentry->d_name, '.');

		if (!dot || dot == dentry->d_name || !strncmp(dentry->d_name, "..", 2))
			continue;

		/* TODO: use identify from ImageMagick and parse its output */
		switch (dot[1]) {
		case 't':
		case 'T':
			if (dot[2] == 'x' || dot[2] == 'X')
				format = PIMA15740_FMT_A_TEXT;
			else
				format = PIMA15740_FMT_I_TIFF;
			break;
		case 'j':
		case 'J':
			format = PIMA15740_FMT_I_EXIF_JPEG;
			break;
		default:
			format = PIMA15740_FMT_A_UNDEFINED;
		}

		ret = stat(dentry->d_name, &fstat);
		if (ret < 0)
			break;

		namelen = strlen(dentry->d_name) + 1;

		ret = put_string(ic, fname_ucs2, dentry->d_name, namelen);
		if (ret)
			break;

		gmtime_r(&fstat.st_mtime, &mod_tm);
		snprintf(mod, sizeof(mod),"%04u%02u%02uT%02u%02u%02u.0Z",
			 mod_tm.tm_year + 1900, mod_tm.tm_mon + 1,
			 mod_tm.tm_mday, mod_tm.tm_hour,
			 mod_tm.tm_min, mod_tm.tm_sec);

		/* String length including the trailing '\0' */
		datelen = strlen(mod) + 1;
		ret = put_string(ic, mod_ucs2, mod, datelen);
		if (ret) {
			mod[0] = '\0';
			datelen = 0;
		}

#ifdef THUMB_SUPPORT
		if (format != PIMA15740_FMT_A_TEXT) {
			thumb_size = generate_thumb(dentry->d_name);
			if (thumb_size < 0) {
				thumb_size = 0;
				continue;
			}
		}
#endif

		/* namelen and datelen include terminating '\0', plus 4 string-size bytes */
		osize = sizeof(*obj) + 2 * (datelen + namelen) + 4;

		if (verbose)
			fprintf(stderr, "Listing image %s, modified %s, info-size %u\n",
				dentry->d_name, mod, (unsigned int)osize);

		obj = malloc(osize);
		if (!obj) {
			ret = -1;
			break;
		}

#ifdef THUMB_SUPPORT
		if (format == PIMA15740_FMT_A_TEXT ||
		    format == PIMA15740_FMT_A_UNDEFINED) {
			thumb_format = PIMA15740_FMT_A_UNDEFINED;
			thumb_width = 0;
			thumb_height = 0;
			thumb_size = 0;
		} else {
			thumb_format = PIMA15740_FMT_I_JFIF;
			thumb_width = THUMB_WIDTH;
			thumb_height = THUMB_HEIGHT;
		}
#else
		thumb_format = PIMA15740_FMT_A_UNDEFINED;
		thumb_width = 0;
		thumb_height = 0;
		thumb_size = 0;
#endif

		obj->handle = ++handle;

		/* Fixed size object info, filename, capture date, and two empty strings */
		obj->info_size = sizeof(obj->info) + 2 * (datelen + namelen) + 4;

		obj->info.storage_id			= __cpu_to_le32(STORE_ID);
		obj->info.object_format			= __cpu_to_le16(format);
		obj->info.protection_status		= __cpu_to_le16(fstat.st_mode & S_IWUSR ? 0 : 1);
		obj->info.object_compressed_size	= __cpu_to_le32(fstat.st_size);
		obj->info.thumb_format			= __cpu_to_le16(thumb_format);
		obj->info.thumb_compressed_size		= __cpu_to_le32(thumb_size);
		obj->info.thumb_pix_width		= __cpu_to_le32(thumb_width);
		obj->info.thumb_pix_height		= __cpu_to_le32(thumb_height);
		obj->info.image_pix_width		= __cpu_to_le32(0);	/* 0 == */
		obj->info.image_pix_height		= __cpu_to_le32(0);	/* not */
		obj->info.image_bit_depth		= __cpu_to_le32(0);	/* supported */
		obj->info.parent_object			= __cpu_to_le32(2);	/* Fixed /dcim/xxx/ */
		obj->info.association_type		= __cpu_to_le16(0);
		obj->info.association_desc		= __cpu_to_le32(0);
		obj->info.sequence_number		= __cpu_to_le32(0);
		strncpy(obj->name, dentry->d_name, sizeof(obj->name));

		obj->info.strings[0]					= namelen;
		memcpy(obj->info.strings + 1, fname_ucs2, namelen * 2);
		/* We use file modification date as Capture Date */
		obj->info.strings[1 + namelen * 2]			= datelen;
		memcpy(obj->info.strings + 2 + namelen * 2, mod_ucs2, datelen * 2);
		/* Empty Modification Date */
		obj->info.strings[2 + (namelen + datelen) * 2]	= 0;
		/* Empty Keywords */
		obj->info.strings[3 + (namelen + datelen) * 2]	= 0;

		images = g_slist_append(images, obj);
	}

	object_number = handle;
	last_object_number = handle;

	closedir(d);
	return ret;
}

static void init_strings(iconv_t ic)
{
	put_string(ic, (char *)dev_info.manuf, manuf, sizeof(manuf));
	put_string(ic, (char *)dev_info.model, model, sizeof(model));
	put_string(ic, (char *)storage_info.desc,
		   storage_desc, sizeof(storage_desc));
}

static void signothing(int sig, siginfo_t *info, void *ptr)
{
	(void) ptr;
	(void) info;

	/* NOP */
	if (verbose > 2)
		fprintf(stderr, "%s %d\n", __func__, sig);
}

static int init_signal(void)
{
	struct sigaction sa = {
		.sa_sigaction = signothing,
		.sa_flags = SA_SIGINFO,
	};

	sigfillset(&sa.sa_mask);
	/* We will use SIGINT to wake up the bulk thread from read() */
	if (sigaction(SIGINT, &sa, NULL) < 0) {
		perror("SIGINT");
		return -1;
	}
	return 0;
}

int main(int argc, char *argv[])
{
	int c, ret;
	struct stat root_stat;
	images = NULL;

	puts("Linux PTP Gadget v" VERSION_STRING);

	ic = iconv_open("UCS-2LE", "ISO8859-1");
	if (ic == (iconv_t)-1) {
		perror("iconv_open");
		return -1;
	}

	uc = iconv_open("ISO8859-1", "UCS-2LE");
	if (uc == (iconv_t)-1) {
		perror("iconv_open 2");
		return -1;
	}

	init_strings(ic);

	if (init_signal() < 0)
		exit(EXIT_FAILURE);

	if (sem_init(&reset, 0, 0) < 0)
		exit(EXIT_FAILURE);

	while ((c = getopt(argc, argv, "v")) != EOF) {
		switch (c) {
		case 'v':
			verbose++;
			break;
		default:
			fprintf(stderr, "Unsupported option %c\n", c);
			exit(EXIT_FAILURE);
		}
	}

	root = argv[argc - 1];

	clean_up(root);

	/*
	 * if a client doesn't ask for storage info (as seen with some
	 * older SW versions, e.g. on Ubuntu 8.04), then the free space
	 * in storage_info will not be updated. This might result in non
	 * working upload because before upload the free space will be
	 * checked. Prevent this by running update_free_space() early.
	 */
	update_free_space();

	enum_objects(root);

	if (chdir("/dev/ptp") < 0) {
		perror("can't chdir /dev/ptp");
		exit(EXIT_FAILURE);
	}

	ret = stat(root, &root_stat);
	if (ret < 0 || !S_ISDIR(root_stat.st_mode) || access(root, R_OK | W_OK) < 0) {
		fprintf(stderr, "Invalid base directory %s\n", root);
		exit(EXIT_FAILURE);
	}

	init_device();
	if (control < 0)
		exit(EXIT_FAILURE);

	fflush(stderr);

	ret = main_loop();

	iconv_close(uc);
	iconv_close(ic);

	exit(ret ? EXIT_FAILURE : EXIT_SUCCESS);
}

#ifdef THUMB_SUPPORT
static int generate_thumb(const char *file_name)
{
	struct stat fstat, tstat;
	char thumb[256];
	char *dot;

	if (!file_name)
		return -1;

	dot = strrchr(file_name, '.');
	if (!dot || dot == file_name)
		return -1;

	/* Put thumbnails under /var/cache/ptp/thumb/
	 * and call them <filename>.thumb.<extension> */
	*dot = '\0';
	snprintf(thumb, sizeof(thumb), THUMB_LOCATION "%s.thumb.jpeg",
		 file_name);
	*dot = '.';

	if (stat(thumb, &tstat) < 0 || tstat.st_mtime < fstat.st_mtime) {
		pid_t converter;
		if (verbose)
			fprintf(stderr, "No or old thumbnail for %s\n", file_name);
		converter = fork();
		if (converter < 0) {
			if (verbose)
				fprintf(stderr, "Cannot generate thumbnail for %s\n",
					file_name);
			return -1;
		} else if (converter) {
			int status;
			waitpid(converter, &status, 0);
			if (!WIFEXITED(status) || WEXITSTATUS(status) ||
			    stat(thumb, &tstat) < 0) {
				if (verbose)
					fprintf(stderr,
						"Generate thumbnail for %s failed\n",
						file_name);
				return -1;
			}
		} else
			execlp("convert", "convert", "-thumbnail", THUMB_SIZE,
				       file_name, thumb, NULL);
	}
	return tstat.st_size;
}
#endif

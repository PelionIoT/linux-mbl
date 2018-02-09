// SPDX-License-Identifier: GPL-2.0
/*
 * Video Capture dev for Freescale i.MX7 SoC
 *
 * Copyright (C) 2018 Linaro Ltd
 * Copyright (C) 2014-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 */

#include <asm/dma.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gcd.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/media-bus-format.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#define IMX7_CAM_DRV_NAME "imx7-csi"
#define IMX7_CAM_VERSION "0.0.1"
#define IMX7_CAM_DRIVER_DESCRIPTION "i.IMX7_CSI"

#define MAX_VIDEO_MEM 64

/* reset values */
#define CSICR1_RESET_VAL	0x40000800
#define CSICR2_RESET_VAL	0x0
#define CSICR3_RESET_VAL	0x0

/* csi control reg 1 */
#define BIT_SWAP16_EN		BIT(31)
#define BIT_EXT_VSYNC		BIT(30)
#define BIT_EOF_INT_EN		BIT(29)
#define BIT_PRP_IF_EN		BIT(28)
#define BIT_CCIR_MODE		BIT(27)
#define BIT_COF_INT_EN		BIT(26)
#define BIT_SF_OR_INTEN		BIT(25)
#define BIT_RF_OR_INTEN		BIT(24)
#define BIT_SFF_DMA_DONE_INTEN  BIT(22)
#define BIT_STATFF_INTEN	BIT(21)
#define BIT_FB2_DMA_DONE_INTEN  BIT(20)
#define BIT_FB1_DMA_DONE_INTEN  BIT(19)
#define BIT_RXFF_INTEN		BIT(18)
#define BIT_SOF_POL		BIT(17)
#define BIT_SOF_INTEN		BIT(16)
#define BIT_MCLKDIV		(0xF << 12)
#define BIT_HSYNC_POL		BIT(11)
#define BIT_CCIR_EN		BIT(10)
#define BIT_MCLKEN		BIT(9)
#define BIT_FCC			BIT(8)
#define BIT_PACK_DIR		BIT(7)
#define BIT_CLR_STATFIFO	BIT(6)
#define BIT_CLR_RXFIFO		BIT(5)
#define BIT_GCLK_MODE		BIT(4)
#define BIT_INV_DATA		BIT(3)
#define BIT_INV_PCLK		BIT(2)
#define BIT_REDGE		BIT(1)
#define BIT_PIXEL_BIT		BIT(0)

#define SHIFT_MCLKDIV		12

/* control reg 3 */
#define BIT_FRMCNT		(0xFFFF << 16)
#define BIT_FRMCNT_RST		BIT(15)
#define BIT_DMA_REFLASH_RFF	BIT(14)
#define BIT_DMA_REFLASH_SFF	BIT(13)
#define BIT_DMA_REQ_EN_RFF	BIT(12)
#define BIT_DMA_REQ_EN_SFF	BIT(11)
#define BIT_STATFF_LEVEL	(0x7 << 8)
#define BIT_HRESP_ERR_EN	BIT(7)
#define BIT_RXFF_LEVEL		(0x7 << 4)
#define BIT_TWO_8BIT_SENSOR	BIT(3)
#define BIT_ZERO_PACK_EN	BIT(2)
#define BIT_ECC_INT_EN		BIT(1)
#define BIT_ECC_AUTO_EN		BIT(0)

#define SHIFT_FRMCNT		16
#define SHIFT_RXFIFO_LEVEL	4

/* csi status reg */
#define BIT_ADDR_CH_ERR_INT	BIT(28)
#define BIT_FIELD0_INT		BIT(27)
#define BIT_FIELD1_INT		BIT(26)
#define BIT_SFF_OR_INT		BIT(25)
#define BIT_RFF_OR_INT		BIT(24)
#define BIT_DMA_TSF_DONE_SFF	BIT(22)
#define BIT_STATFF_INT		BIT(21)
#define BIT_DMA_TSF_DONE_FB2	BIT(20)
#define BIT_DMA_TSF_DONE_FB1	BIT(19)
#define BIT_RXFF_INT		BIT(18)
#define BIT_EOF_INT		BIT(17)
#define BIT_SOF_INT		BIT(16)
#define BIT_F2_INT		BIT(15)
#define BIT_F1_INT		BIT(14)
#define BIT_COF_INT		BIT(13)
#define BIT_HRESP_ERR_INT	BIT(7)
#define BIT_ECC_INT		BIT(1)
#define BIT_DRDY		BIT(0)

/* csi control reg 18 */
#define BIT_CSI_ENABLE			BIT(31)
#define BIT_MIPI_DATA_FORMAT_RAW8	(0x2a << 25)
#define BIT_MIPI_DATA_FORMAT_RAW10	(0x2b << 25)
#define BIT_MIPI_DATA_FORMAT_YUV422_8B	(0x1e << 25)
#define BIT_MIPI_DATA_FORMAT_MASK	(0x3F << 25)
#define BIT_MIPI_DATA_FORMAT_OFFSET	25
#define BIT_DATA_FROM_MIPI		BIT(22)
#define BIT_MIPI_YU_SWAP		BIT(21)
#define BIT_MIPI_DOUBLE_CMPNT		BIT(20)
#define BIT_BASEADDR_CHG_ERR_EN		BIT(9)
#define BIT_BASEADDR_SWITCH_SEL		BIT(5)
#define BIT_BASEADDR_SWITCH_EN		BIT(4)
#define BIT_PARALLEL24_EN		BIT(3)
#define BIT_DEINTERLACE_EN		BIT(2)
#define BIT_TVDECODER_IN_EN		BIT(1)
#define BIT_NTSC_EN			BIT(0)

#define CSI_MCLK_VF		1
#define CSI_MCLK_ENC		2
#define CSI_MCLK_RAW		4
#define CSI_MCLK_I2C		8

#define CSI_CSICR1		0x0
#define CSI_CSICR2		0x4
#define CSI_CSICR3		0x8
#define CSI_STATFIFO		0xC
#define CSI_CSIRXFIFO		0x10
#define CSI_CSIRXCNT		0x14
#define CSI_CSISR		0x18

#define CSI_CSIDBG		0x1C
#define CSI_CSIDMASA_STATFIFO	0x20
#define CSI_CSIDMATS_STATFIFO	0x24
#define CSI_CSIDMASA_FB1	0x28
#define CSI_CSIDMASA_FB2	0x2C
#define CSI_CSIFBUF_PARA	0x30
#define CSI_CSIIMAG_PARA	0x34

#define CSI_CSICR18		0x48
#define CSI_CSICR19		0x4c

#define NUM_FORMATS ARRAY_SIZE(formats)
#define IMX7X_MAX_SENSORS    1

/*
 * Basic structures
 */
struct imx7_fmt {
	char  name[32];
	u32   fourcc;		/* v4l2 format id */
	u32   pixelformat;
	u32   mbus_code;
	int   bpp;
};

static struct imx7_fmt formats[] = {
	{
		.name		= "UYVY-16",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.bpp		= 2,
	}, {
		.name		= "YUYV-16",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.bpp		= 2,
	}, {
		.name		= "YUV32 (X-Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.pixelformat	= V4L2_PIX_FMT_YUV32,
		.mbus_code	= MEDIA_BUS_FMT_AYUV8_1X32,
		.bpp		= 4,
	}, {
		.name		= "RAWRGB8 (SBGGR8)",
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.bpp		= 1,
	}, {
		.name		= "RAWRGB10 (SBGGR10)",
		.fourcc		= V4L2_PIX_FMT_SBGGR10,
		.pixelformat	= V4L2_PIX_FMT_SBGGR10,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp		= 2,
	}
};

struct imx7_buf_internal {
	struct list_head	queue;
	int			bufnum;
	bool			discard;
};

/* buffer for one video frame */
struct imx7_buffer {
	struct vb2_v4l2_buffer		vb;
	struct imx7_buf_internal	internal;
};

struct imx7_csi_mux {
	struct regmap	*gpr;
	u8		req_gpr;
	u8		req_bit;
};

struct imx7_csi_dev {
	struct device		*dev;
	struct video_device	*vdev;
	struct v4l2_subdev	*sd;
	struct v4l2_device	v4l2_dev;
	struct media_device	media_dev;

	struct vb2_queue	vb2_vidq;
	struct v4l2_ctrl_handler ctrl_handler;

	struct mutex		lock;
	spinlock_t		slock;

	/* clock */
	struct clk	*clk_disp_axi;
	struct clk	*clk_disp_dcic;
	struct clk	*clk_csi_mclk;

	void __iomem	*regbase;
	int		irq;

	u32		type;
	u32		bytesperline;
	v4l2_std_id	std;
	struct imx7_fmt	*fmt;
	struct v4l2_pix_format pix;
	u32		 mbus_code;

	unsigned int	frame_count;

	struct list_head	capture;
	struct list_head	active_bufs;
	struct list_head	discard;

	void			*discard_buffer;
	dma_addr_t		discard_buffer_dma;
	size_t			discard_size;
	struct imx7_buf_internal	buf_discard[2];

	struct v4l2_async_subdev	asd;
	struct v4l2_async_notifier	subdev_notifier;
	struct v4l2_async_subdev	*async_subdevs[2];

	bool		csi_mux_mipi;
	const bool	*rx_fifo_rst;
	struct imx7_csi_mux csi_mux;
};

static const struct of_device_id imx7_csi_dt_ids[];

static int csi_read(struct imx7_csi_dev *csi, unsigned int offset)
{
	return __raw_readl(csi->regbase + offset);
}

static void csi_write(struct imx7_csi_dev *csi, unsigned int value,
		      unsigned int offset)
{
	__raw_writel(value, csi->regbase + offset);
}

static struct imx7_csi_dev *notifier_to_imx7_dev(struct v4l2_async_notifier *n)
{
	return container_of(n, struct imx7_csi_dev, subdev_notifier);
}

static struct imx7_fmt *format_by_fourcc(int fourcc)
{
	int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].pixelformat == fourcc)
			return formats + i;
	}

	return NULL;
}

static struct imx7_fmt *format_by_mbus(u32 code)
{
	int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].mbus_code == code)
			return formats + i;
	}

	return NULL;
}

static struct imx7_buffer *imx7_ibuf_to_buf(struct imx7_buf_internal *int_buf)
{
	return container_of(int_buf, struct imx7_buffer, internal);
}

static void csi_clk_enable(struct imx7_csi_dev *csi_dev)
{
	clk_prepare_enable(csi_dev->clk_disp_axi);
	clk_prepare_enable(csi_dev->clk_disp_dcic);
	clk_prepare_enable(csi_dev->clk_csi_mclk);
}

static void csi_clk_disable(struct imx7_csi_dev *csi_dev)
{
	clk_disable_unprepare(csi_dev->clk_csi_mclk);
	clk_disable_unprepare(csi_dev->clk_disp_dcic);
	clk_disable_unprepare(csi_dev->clk_disp_axi);
}

static void csihw_reset(struct imx7_csi_dev *csi_dev)
{
	__raw_writel(__raw_readl(csi_dev->regbase + CSI_CSICR3)
		     | BIT_FRMCNT_RST,
		     csi_dev->regbase + CSI_CSICR3);

	__raw_writel(CSICR1_RESET_VAL, csi_dev->regbase + CSI_CSICR1);
	__raw_writel(CSICR2_RESET_VAL, csi_dev->regbase + CSI_CSICR2);
	__raw_writel(CSICR3_RESET_VAL, csi_dev->regbase + CSI_CSICR3);
}

static void csisw_reset(struct imx7_csi_dev *csi_dev)
{
	int cr1, cr3, cr18, isr;

	/* Disable csi  */
	cr18 = csi_read(csi_dev, CSI_CSICR18);
	cr18 &= ~BIT_CSI_ENABLE;
	csi_write(csi_dev, cr18, CSI_CSICR18);

	/* Clear RX FIFO */
	cr1 = csi_read(csi_dev, CSI_CSICR1);
	csi_write(csi_dev, cr1 & ~BIT_FCC, CSI_CSICR1);
	cr1 = csi_read(csi_dev, CSI_CSICR1);
	csi_write(csi_dev, cr1 | BIT_CLR_RXFIFO, CSI_CSICR1);

	/* DMA reflash */
	cr3 = csi_read(csi_dev, CSI_CSICR3);
	cr3 |= BIT_DMA_REFLASH_RFF | BIT_FRMCNT_RST;
	csi_write(csi_dev, cr3, CSI_CSICR3);

	usleep_range(2000, 3000);

	cr1 = csi_read(csi_dev, CSI_CSICR1);
	csi_write(csi_dev, cr1 | BIT_FCC, CSI_CSICR1);

	isr = csi_read(csi_dev, CSI_CSISR);
	csi_write(csi_dev, isr, CSI_CSISR);

	/* Ensable csi  */
	cr18 |= BIT_CSI_ENABLE;
	csi_write(csi_dev, cr18, CSI_CSICR18);
}

static void csi_init_interface(struct imx7_csi_dev *csi_dev)
{
	unsigned int val = 0;
	unsigned int imag_para;

	val = BIT_SOF_POL | BIT_REDGE | BIT_GCLK_MODE | BIT_HSYNC_POL |
		BIT_FCC | 1 << SHIFT_MCLKDIV | BIT_MCLKEN;
	__raw_writel(val, csi_dev->regbase + CSI_CSICR1);

	imag_para = (640 << 16) | 960;
	__raw_writel(imag_para, csi_dev->regbase + CSI_CSIIMAG_PARA);

	val = BIT_DMA_REFLASH_RFF;
	__raw_writel(val, csi_dev->regbase + CSI_CSICR3);
}

static void csi_enable_int(struct imx7_csi_dev *csi_dev, int arg)
{
	unsigned long cr1 = __raw_readl(csi_dev->regbase + CSI_CSICR1);

	cr1 |= BIT_SOF_INTEN;
	cr1 |= BIT_RFF_OR_INT;
	if (arg == 1) {
		/* still capture needs DMA interrupt */
		cr1 |= BIT_FB1_DMA_DONE_INTEN;
		cr1 |= BIT_FB2_DMA_DONE_INTEN;
	}
	__raw_writel(cr1, csi_dev->regbase + CSI_CSICR1);
}

static void csi_disable_int(struct imx7_csi_dev *csi_dev)
{
	unsigned long cr1 = __raw_readl(csi_dev->regbase + CSI_CSICR1);

	cr1 &= ~BIT_SOF_INTEN;
	cr1 &= ~BIT_RFF_OR_INT;
	cr1 &= ~BIT_FB1_DMA_DONE_INTEN;
	cr1 &= ~BIT_FB2_DMA_DONE_INTEN;
	__raw_writel(cr1, csi_dev->regbase + CSI_CSICR1);
}

static void csi_enable(struct imx7_csi_dev *csi_dev, int arg)
{
	unsigned long cr = __raw_readl(csi_dev->regbase + CSI_CSICR18);

	if (arg == 1)
		cr |= BIT_CSI_ENABLE;
	else
		cr &= ~BIT_CSI_ENABLE;
	__raw_writel(cr, csi_dev->regbase + CSI_CSICR18);
}

static void csi_buf_stride_set(struct imx7_csi_dev *csi_dev, u32 stride)
{
	__raw_writel(stride, csi_dev->regbase + CSI_CSIFBUF_PARA);
}

static void csi_deinterlace_enable(struct imx7_csi_dev *csi_dev, bool enable)
{
	unsigned long cr18 = __raw_readl(csi_dev->regbase + CSI_CSICR18);

	if (enable)
		cr18 |= BIT_DEINTERLACE_EN;
	else
		cr18 &= ~BIT_DEINTERLACE_EN;

	__raw_writel(cr18, csi_dev->regbase + CSI_CSICR18);
}

static void csi_deinterlace_mode(struct imx7_csi_dev *csi_dev, int mode)
{
	unsigned long cr18 = __raw_readl(csi_dev->regbase + CSI_CSICR18);

	if (mode == V4L2_STD_NTSC)
		cr18 |= BIT_NTSC_EN;
	else
		cr18 &= ~BIT_NTSC_EN;

	__raw_writel(cr18, csi_dev->regbase + CSI_CSICR18);
}

static void csi_tvdec_enable(struct imx7_csi_dev *csi_dev, bool enable)
{
	unsigned long cr18 = __raw_readl(csi_dev->regbase + CSI_CSICR18);
	unsigned long cr1 = __raw_readl(csi_dev->regbase + CSI_CSICR1);

	if (enable) {
		cr18 |= (BIT_TVDECODER_IN_EN |
				BIT_BASEADDR_SWITCH_EN |
				BIT_BASEADDR_SWITCH_SEL |
				BIT_BASEADDR_CHG_ERR_EN);
		cr1 |= BIT_CCIR_MODE;
		cr1 &= ~(BIT_SOF_POL | BIT_REDGE);
	} else {
		cr18 &= ~(BIT_TVDECODER_IN_EN |
				BIT_BASEADDR_SWITCH_EN |
				BIT_BASEADDR_SWITCH_SEL |
				BIT_BASEADDR_CHG_ERR_EN);
		cr1 &= ~BIT_CCIR_MODE;
		cr1 |= BIT_SOF_POL | BIT_REDGE;
	}

	__raw_writel(cr18, csi_dev->regbase + CSI_CSICR18);
	__raw_writel(cr1, csi_dev->regbase + CSI_CSICR1);
}

static void csi_dmareq_rff_enable(struct imx7_csi_dev *csi_dev)
{
	unsigned long cr3 = __raw_readl(csi_dev->regbase + CSI_CSICR3);
	unsigned long cr2 = __raw_readl(csi_dev->regbase + CSI_CSICR2);

	/* Burst Type of DMA Transfer from RxFIFO. INCR16 */
	cr2 |= 0xC0000000;

	cr3 |= BIT_DMA_REQ_EN_RFF;
	cr3 |= BIT_HRESP_ERR_EN;
	cr3 &= ~BIT_RXFF_LEVEL;
	cr3 |= 0x2 << 4;

	__raw_writel(cr3, csi_dev->regbase + CSI_CSICR3);
	__raw_writel(cr2, csi_dev->regbase + CSI_CSICR2);
}

static void csi_dmareq_rff_disable(struct imx7_csi_dev *csi_dev)
{
	unsigned long cr3 = __raw_readl(csi_dev->regbase + CSI_CSICR3);

	cr3 &= ~BIT_DMA_REQ_EN_RFF;
	cr3 &= ~BIT_HRESP_ERR_EN;
	__raw_writel(cr3, csi_dev->regbase + CSI_CSICR3);
}

static void csi_set_imagpara(struct imx7_csi_dev *csi, int width, int height)
{
	int imag_para = 0;
	unsigned long cr3 = __raw_readl(csi->regbase + CSI_CSICR3);

	imag_para = (width << 16) | height;
	__raw_writel(imag_para, csi->regbase + CSI_CSIIMAG_PARA);

	/* reflash the embedded DMA controller */
	__raw_writel(cr3 | BIT_DMA_REFLASH_RFF, csi->regbase + CSI_CSICR3);
}

static void csi_error_recovery(struct imx7_csi_dev *csi_dev)
{
	u32 cr1, cr3, cr18;
	/* software reset */

	/* Disable csi  */
	cr18 = csi_read(csi_dev, CSI_CSICR18);
	cr18 &= ~BIT_CSI_ENABLE;
	csi_write(csi_dev, cr18, CSI_CSICR18);

	/* Clear RX FIFO */
	cr1 = csi_read(csi_dev, CSI_CSICR1);
	csi_write(csi_dev, cr1 & ~BIT_FCC, CSI_CSICR1);
	cr1 = csi_read(csi_dev, CSI_CSICR1);
	csi_write(csi_dev, cr1 | BIT_CLR_RXFIFO, CSI_CSICR1);

	cr1 = csi_read(csi_dev, CSI_CSICR1);
	csi_write(csi_dev, cr1 | BIT_FCC, CSI_CSICR1);

	/* DMA reflash */
	cr3 = csi_read(csi_dev, CSI_CSICR3);
	cr3 |= BIT_DMA_REFLASH_RFF;
	csi_write(csi_dev, cr3, CSI_CSICR3);

	/* Ensable csi  */
	cr18 |= BIT_CSI_ENABLE;
	csi_write(csi_dev, cr18, CSI_CSICR18);
}

/*
 *  Videobuf operations
 */
static int imx7_videobuf_setup(struct vb2_queue *vq, unsigned int *count,
			       unsigned int *num_planes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct imx7_csi_dev *csi_dev = vb2_get_drv_priv(vq);

	alloc_devs[0] = csi_dev->dev;

	sizes[0] = csi_dev->pix.sizeimage;

	dev_dbg(csi_dev->dev, "count=%d, size=%d\n", *count, sizes[0]);

	if (*count == 0)
		*count = 32;
	if (!*num_planes &&
	    sizes[0] * *count > MAX_VIDEO_MEM * 1024 * 1024)
		*count = (MAX_VIDEO_MEM * 1024 * 1024) / sizes[0];

	*num_planes = 1;

	return 0;
}

static int imx7_videobuf_prepare(struct vb2_buffer *vb)
{
	struct imx7_csi_dev *csi_dev = vb2_get_drv_priv(vb->vb2_queue);

	dev_dbg(csi_dev->dev, "%s (vb=0x%p) 0x%p %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

#ifdef DEBUG
	/*
	 * This can be useful if you want to see if we actually fill
	 * the buffer with something
	 */
	if (vb2_plane_vaddr(vb, 0))
		memset((void *)vb2_plane_vaddr(vb, 0),
		       0xaa, vb2_get_plane_payload(vb, 0));
#endif
	if (vb2_plane_size(vb, 0) < csi_dev->pix.sizeimage) {
		dev_info(csi_dev->dev,
			 "data will not fit into plane (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), (long)csi_dev->pix.sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, csi_dev->pix.sizeimage);
	if (vb2_plane_vaddr(vb, 0) &&
	    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0))
		return -EINVAL;

	return 0;
}

static void imx7_videobuf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct imx7_csi_dev *csi_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imx7_buffer *buf = container_of(vbuf, struct imx7_buffer, vb);
	unsigned long flags;

	dev_dbg(csi_dev->dev, "%s (vb=0x%p) 0x%p %lu\n", __func__, vb,
		vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	spin_lock_irqsave(&csi_dev->slock, flags);

	list_add_tail(&buf->internal.queue, &csi_dev->capture);

	spin_unlock_irqrestore(&csi_dev->slock, flags);
}

static void imx7_update_csi_buf(struct imx7_csi_dev *csi_dev,
				unsigned long phys, int bufnum)
{
	if (bufnum == 1)
		csi_write(csi_dev, phys, CSI_CSIDMASA_FB2);
	else
		csi_write(csi_dev, phys, CSI_CSIDMASA_FB1);
}

static void imx7_csi_init(struct imx7_csi_dev *csi_dev)
{
	csi_clk_enable(csi_dev);
	csihw_reset(csi_dev);
	csi_init_interface(csi_dev);
	csi_dmareq_rff_disable(csi_dev);
}

static void imx7_csi_deinit(struct imx7_csi_dev *csi_dev)
{
	csihw_reset(csi_dev);
	csi_init_interface(csi_dev);
	csi_dmareq_rff_disable(csi_dev);
	csi_clk_disable(csi_dev);
}

static int imx7_csi_enable(struct imx7_csi_dev *csi_dev)
{
	struct v4l2_pix_format *pix = &csi_dev->pix;
	unsigned long flags;
	unsigned long val;
	int timeout, timeout2;

	csisw_reset(csi_dev);

	if (pix->field == V4L2_FIELD_INTERLACED)
		csi_tvdec_enable(csi_dev, true);

	/* For mipi csi input only */
	if (csi_dev->csi_mux_mipi) {
		csi_dmareq_rff_enable(csi_dev);
		csi_enable_int(csi_dev, 1);
		csi_enable(csi_dev, 1);
		return 0;
	}

	local_irq_save(flags);
	for (timeout = 10000000; timeout > 0; timeout--) {
		if (!(csi_read(csi_dev, CSI_CSISR) & BIT_SOF_INT)) {
			cpu_relax();
			continue;
		}

		val = csi_read(csi_dev, CSI_CSICR3);
		csi_write(csi_dev, val | BIT_DMA_REFLASH_RFF,
			  CSI_CSICR3);
		/* Wait DMA reflash done */
		for (timeout2 = 1000000; timeout2 > 0; timeout2--) {
			if (csi_read(csi_dev, CSI_CSICR3) & BIT_DMA_REFLASH_RFF)
				cpu_relax();
			else
				break;
		}

		if (timeout2 <= 0) {
			pr_err("timeout when wait for reflash done.\n");
			local_irq_restore(flags);
			return -ETIME;
		}

		/*
		 * For iimx7l csi, DMA FIFO will auto start when sensor
		 * ready to work,so DMA should enable right after FIFO
		 * reset, otherwise dma will lost data and image will split.
		 */

		csi_dmareq_rff_enable(csi_dev);
		csi_enable_int(csi_dev, 1);
		csi_enable(csi_dev, 1);
		break;
	}
	if (timeout <= 0) {
		pr_err("timeout when wait for SOF\n");
		local_irq_restore(flags);
		return -ETIME;
	}
	local_irq_restore(flags);

	return 0;
}

static void imx7_csi_disable(struct imx7_csi_dev *csi_dev)
{
	struct v4l2_pix_format *pix = &csi_dev->pix;

	csi_dmareq_rff_disable(csi_dev);
	csi_disable_int(csi_dev);

	/* set CSI_CSIDMASA_FB1 and CSI_CSIDMASA_FB2 to default value */
	csi_write(csi_dev, 0, CSI_CSIDMASA_FB1);
	csi_write(csi_dev, 0, CSI_CSIDMASA_FB2);

	csi_buf_stride_set(csi_dev, 0);

	if (pix->field == V4L2_FIELD_INTERLACED) {
		csi_deinterlace_enable(csi_dev, false);
		csi_tvdec_enable(csi_dev, false);
	}

	csi_enable(csi_dev, 0);
}

static int imx7_configure_csi(struct imx7_csi_dev *csi_dev)
{
	struct v4l2_pix_format *pix = &csi_dev->pix;
	u32 cr1, cr18;
	u32 width;

	if (pix->field == V4L2_FIELD_INTERLACED) {
		csi_deinterlace_enable(csi_dev, true);
		csi_buf_stride_set(csi_dev, csi_dev->pix.width);
		csi_deinterlace_mode(csi_dev, csi_dev->std);
	} else {
		csi_deinterlace_enable(csi_dev, false);
		csi_buf_stride_set(csi_dev, 0);
	}

	switch (csi_dev->fmt->pixelformat) {
	case V4L2_PIX_FMT_YUV32:
	case V4L2_PIX_FMT_SBGGR8:
		width = pix->width;
		break;
	case V4L2_PIX_FMT_SBGGR10:
		width = pix->width * 2;
		break;
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUYV:
		if (csi_dev->csi_mux_mipi)
			width = pix->width;
		else
			width = pix->width * 2;
		break;
	default:
		return -EINVAL;
	}

	csi_set_imagpara(csi_dev, width, pix->height);

	if (!csi_dev->csi_mux_mipi)
		return 0;

	cr1 = csi_read(csi_dev, CSI_CSICR1);
	cr1 &= ~BIT_GCLK_MODE;

	cr18 = csi_read(csi_dev, CSI_CSICR18);
	cr18 &= BIT_MIPI_DATA_FORMAT_MASK;
	cr18 |= BIT_DATA_FROM_MIPI;

	switch (csi_dev->fmt->pixelformat) {
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUYV:
		cr18 |= BIT_MIPI_DATA_FORMAT_YUV422_8B;
		break;
	case V4L2_PIX_FMT_SBGGR8:
		cr18 |= BIT_MIPI_DATA_FORMAT_RAW8;
		break;
	case V4L2_PIX_FMT_SBGGR10:
		cr18 |= BIT_MIPI_DATA_FORMAT_RAW10;
		cr1 |= BIT_PIXEL_BIT;
		break;
	default:
		return -EINVAL;
	}

	csi_write(csi_dev, cr1, CSI_CSICR1);
	csi_write(csi_dev, cr18, CSI_CSICR18);

	return 0;
}

static int imx7_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct imx7_csi_dev *csi_dev = vb2_get_drv_priv(vq);
	struct vb2_buffer *vb;
	struct imx7_buffer *buf;
	unsigned long phys;
	unsigned long flags;

	if (count < 2)
		return -ENOBUFS;

	csi_dev->discard_size = csi_dev->pix.sizeimage;
	csi_dev->discard_buffer = dma_alloc_coherent(csi_dev->v4l2_dev.dev,
					PAGE_ALIGN(csi_dev->discard_size),
					&csi_dev->discard_buffer_dma,
					GFP_DMA | GFP_KERNEL);
	if (!csi_dev->discard_buffer)
		return -ENOMEM;

	spin_lock_irqsave(&csi_dev->slock, flags);

	csi_dev->buf_discard[0].discard = true;
	list_add_tail(&csi_dev->buf_discard[0].queue,
		      &csi_dev->discard);

	csi_dev->buf_discard[1].discard = true;
	list_add_tail(&csi_dev->buf_discard[1].queue,
		      &csi_dev->discard);

	/* csi buf 0 */
	buf = list_first_entry(&csi_dev->capture, struct imx7_buffer,
			       internal.queue);
	buf->internal.bufnum = 0;
	vb = &buf->vb.vb2_buf;
	vb->state = VB2_BUF_STATE_ACTIVE;

	phys = vb2_dma_contig_plane_dma_addr(vb, 0);

	imx7_update_csi_buf(csi_dev, phys, buf->internal.bufnum);
	list_move_tail(csi_dev->capture.next, &csi_dev->active_bufs);

	/* csi buf 1 */
	buf = list_first_entry(&csi_dev->capture, struct imx7_buffer,
			       internal.queue);
	buf->internal.bufnum = 1;
	vb = &buf->vb.vb2_buf;
	vb->state = VB2_BUF_STATE_ACTIVE;

	phys = vb2_dma_contig_plane_dma_addr(vb, 0);
	imx7_update_csi_buf(csi_dev, phys, buf->internal.bufnum);
	list_move_tail(csi_dev->capture.next, &csi_dev->active_bufs);

	spin_unlock_irqrestore(&csi_dev->slock, flags);

	return imx7_csi_enable(csi_dev);
}

static void imx7_stop_streaming(struct vb2_queue *vq)
{
	struct imx7_csi_dev *csi_dev = vb2_get_drv_priv(vq);
	unsigned long flags;
	struct imx7_buffer *buf, *tmp;
	void *b;

	imx7_csi_disable(csi_dev);

	spin_lock_irqsave(&csi_dev->slock, flags);

	list_for_each_entry_safe(buf, tmp, &csi_dev->capture, internal.queue) {
		list_del_init(&buf->internal.queue);
		if (buf->vb.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	list_for_each_entry_safe(buf, tmp, &csi_dev->active_bufs,
				 internal.queue) {
		list_del_init(&buf->internal.queue);

		if (buf->vb.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&csi_dev->capture);
	INIT_LIST_HEAD(&csi_dev->active_bufs);
	INIT_LIST_HEAD(&csi_dev->discard);

	b = csi_dev->discard_buffer;
	csi_dev->discard_buffer = NULL;

	spin_unlock_irqrestore(&csi_dev->slock, flags);

	dma_free_coherent(csi_dev->v4l2_dev.dev, csi_dev->discard_size, b,
			  csi_dev->discard_buffer_dma);
}

static struct vb2_ops imx7_videobuf_ops = {
	.queue_setup     = imx7_videobuf_setup,
	.buf_prepare     = imx7_videobuf_prepare,
	.buf_queue       = imx7_videobuf_queue,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
	.start_streaming = imx7_start_streaming,
	.stop_streaming	 = imx7_stop_streaming,
};

static void imx7_csi_frame_done(struct imx7_csi_dev *csi_dev, int bufnum,
				bool err)
{
	struct imx7_buf_internal *ibuf;
	struct imx7_buffer *buf;
	struct vb2_buffer *vb;
	unsigned long phys;

	ibuf = list_first_entry(&csi_dev->active_bufs, struct imx7_buf_internal,
				queue);

	if (ibuf->discard) {
		/*
		 * Discard buffer must not be returned to user space.
		 * Just return it to the discard queue.
		 */
		list_move_tail(csi_dev->active_bufs.next, &csi_dev->discard);
	} else {
		buf = imx7_ibuf_to_buf(ibuf);

		vb = &buf->vb.vb2_buf;
		phys = vb2_dma_contig_plane_dma_addr(vb, 0);
		if (bufnum == 1) {
			if (csi_read(csi_dev, CSI_CSIDMASA_FB2) != phys) {
				dev_err(csi_dev->dev, "%lx != %x\n", phys,
					csi_read(csi_dev, CSI_CSIDMASA_FB2));
			}
		} else {
			if (csi_read(csi_dev, CSI_CSIDMASA_FB1) != phys) {
				dev_err(csi_dev->dev, "%lx != %x\n", phys,
					csi_read(csi_dev, CSI_CSIDMASA_FB1));
			}
		}
		dev_dbg(csi_dev->dev, "%s (vb=0x%p) 0x%p %lu\n", __func__, vb,
			vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

		list_del_init(&buf->internal.queue);
		vb->timestamp = ktime_get_ns();
		to_vb2_v4l2_buffer(vb)->sequence = csi_dev->frame_count;
		if (err)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		else
			vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}

	csi_dev->frame_count++;

	/* Config discard buffer to active_bufs */
	if (list_empty(&csi_dev->capture)) {
		if (list_empty(&csi_dev->discard)) {
			dev_warn(csi_dev->dev, "%s: trying to access empty discard list\n",
				 __func__);
			return;
		}

		ibuf = list_first_entry(&csi_dev->discard,
					struct imx7_buf_internal, queue);
		ibuf->bufnum = bufnum;

		list_move_tail(csi_dev->discard.next, &csi_dev->active_bufs);

		imx7_update_csi_buf(csi_dev, csi_dev->discard_buffer_dma,
				    bufnum);
		return;
	}

	buf = list_first_entry(&csi_dev->capture, struct imx7_buffer,
			       internal.queue);

	buf->internal.bufnum = bufnum;

	list_move_tail(csi_dev->capture.next, &csi_dev->active_bufs);

	vb = &buf->vb.vb2_buf;
	vb->state = VB2_BUF_STATE_ACTIVE;

	phys = vb2_dma_contig_plane_dma_addr(vb, 0);
	imx7_update_csi_buf(csi_dev, phys, bufnum);
}

static irqreturn_t imx7_csi_irq_handler(int irq, void *data)
{
	struct imx7_csi_dev *csi_dev =  data;
	unsigned long status;
	u32 cr3, cr18;

	spin_lock(&csi_dev->slock);

	status = csi_read(csi_dev, CSI_CSISR);
	csi_write(csi_dev, status, CSI_CSISR);

	if (list_empty(&csi_dev->active_bufs)) {
		dev_warn(csi_dev->dev, "called while active list is empty\n");
		spin_unlock(&csi_dev->slock);
		return IRQ_HANDLED;
	}

	if (status & BIT_RFF_OR_INT) {
		dev_warn(csi_dev->dev, "Rx fifo overflow\n");
		if (*csi_dev->rx_fifo_rst)
			csi_error_recovery(csi_dev);
	}

	if (status & BIT_HRESP_ERR_INT) {
		dev_warn(csi_dev->dev, "Hresponse error detected\n");
		csi_error_recovery(csi_dev);
	}

	if (status & BIT_ADDR_CH_ERR_INT) {
		/* Disable csi  */
		cr18 = csi_read(csi_dev, CSI_CSICR18);
		cr18 &= ~BIT_CSI_ENABLE;
		csi_write(csi_dev, cr18, CSI_CSICR18);

		/* DMA reflash */
		cr3 = csi_read(csi_dev, CSI_CSICR3);
		cr3 |= BIT_DMA_REFLASH_RFF;
		csi_write(csi_dev, cr3, CSI_CSICR3);

		/* Ensable csi  */
		cr18 |= BIT_CSI_ENABLE;
		csi_write(csi_dev, cr18, CSI_CSICR18);
	}

	if ((status & BIT_DMA_TSF_DONE_FB1) &&
	    (status & BIT_DMA_TSF_DONE_FB2)) {
		/*
		 * For both FB1 and FB2 interrupter bits set case,
		 * CSI DMA is work in one of FB1 and FB2 buffer,
		 * but software can not know the state.
		 * Skip it to avoid base address updated
		 * when csi work in field0 and field1 will write to
		 * new base address.
		 */
	} else if (status & BIT_DMA_TSF_DONE_FB1) {
		imx7_csi_frame_done(csi_dev, 0, false);
	} else if (status & BIT_DMA_TSF_DONE_FB2) {
		imx7_csi_frame_done(csi_dev, 1, false);
	}

	spin_unlock(&csi_dev->slock);

	return IRQ_HANDLED;
}

/*
 * File operations for the device
 */
static int imx7_csi_open(struct file *file)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;
	int ret = 0;

	file->private_data = csi_dev;

	if (mutex_lock_interruptible(&csi_dev->lock))
		return -ERESTARTSYS;

	ret = v4l2_fh_open(file);
	if (ret < 0)
		goto unlock;

	pm_runtime_get_sync(csi_dev->dev);

	v4l2_subdev_call(sd, core, s_power, 1);

	imx7_csi_init(csi_dev);

unlock:
	mutex_unlock(&csi_dev->lock);

	return ret;
}

static int imx7_csi_release(struct file *file)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;
	struct vb2_queue *vq = &csi_dev->vb2_vidq;

	mutex_lock(&csi_dev->lock);

	imx7_stop_streaming(vq);
	vb2_queue_release(vq);

	imx7_csi_deinit(csi_dev);
	v4l2_subdev_call(sd, core, s_power, 0);

	mutex_unlock(&csi_dev->lock);

	pm_runtime_put_sync_suspend(csi_dev->dev);
	v4l2_fh_release(file);

	return 0;
}

static struct v4l2_file_operations imx7_csi_fops = {
	.owner		= THIS_MODULE,
	.open		= imx7_csi_open,
	.release	= imx7_csi_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2, /* V4L2 ioctl handler */
	.mmap		= vb2_fop_mmap,
};

/*
 * Video node IOCTLs
 */
static int imx7_vidioc_enum_input(struct file *file, void *priv,
				  struct v4l2_input *inp)
{
	if (inp->index != 0)
		return -EINVAL;

	/* default is camera */
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(inp->name, "Camera", sizeof(inp->name));

	return 0;
}

static int imx7_vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int imx7_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static int imx7_vidioc_querystd(struct file *file, void *priv, v4l2_std_id *a)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;

	return v4l2_subdev_call(sd, video, querystd, a);
}

static int imx7_vidioc_s_std(struct file *file, void *priv, v4l2_std_id a)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;

	return v4l2_subdev_call(sd, video, s_std, a);
}

static int imx7_vidioc_g_std(struct file *file, void *priv, v4l2_std_id *a)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;

	return v4l2_subdev_call(sd, video, g_std, a);
}

static int imx7_vidioc_reqbufs(struct file *file, void *priv,
			       struct v4l2_requestbuffers *p)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);

	return vb2_reqbufs(&csi_dev->vb2_vidq, p);
}

static int imx7_vidioc_querybuf(struct file *file, void *priv,
				struct v4l2_buffer *p)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct vb2_buffer *vb;
	int ret;

	ret = vb2_querybuf(&csi_dev->vb2_vidq, p);

	if (!ret) {
		/* return physical address */
		vb = csi_dev->vb2_vidq.bufs[p->index];
		if (p->flags & V4L2_BUF_FLAG_MAPPED)
			p->m.offset = vb2_dma_contig_plane_dma_addr(vb, 0);
	}
	return ret;
}

static int imx7_vidioc_qbuf(struct file *file, void *priv,
			    struct v4l2_buffer *p)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);

	return vb2_qbuf(&csi_dev->vb2_vidq, p);
}

static int imx7_vidioc_dqbuf(struct file *file, void *priv,
			     struct v4l2_buffer *p)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);

	return vb2_dqbuf(&csi_dev->vb2_vidq, p, file->f_flags & O_NONBLOCK);
}

static int imx7_vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.index = f->index,
	};
	struct imx7_fmt *fmt;
	int ret;

	ret = v4l2_subdev_call(sd, pad, enum_mbus_code, NULL, &code);
	if (ret < 0) {
		/* no more formats */
		dev_info(csi_dev->dev, "No more fmt\n");
		return -EINVAL;
	}

	fmt = format_by_mbus(code.code);
	if (!fmt) {
		dev_err(csi_dev->dev, "mbus (0x%08x) invalid.\n", code.code);
		return -EINVAL;
	}

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->pixelformat;

	return 0;
}

static int imx7_vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				       struct v4l2_format *f)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct imx7_fmt *fmt;
	int ret;

	fmt = format_by_fourcc(f->fmt.pix.pixelformat);
	if (!fmt) {
		dev_err(csi_dev->dev, "Fourcc format (0x%08x) invalid.",
			f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	v4l2_fill_mbus_format(&format.format, pix, fmt->mbus_code);
	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;

	v4l2_fill_pix_format(pix, &format.format);

	if (pix->field != V4L2_FIELD_INTERLACED)
		pix->field = V4L2_FIELD_NONE;

	pix->sizeimage = fmt->bpp * pix->height * pix->width;
	pix->bytesperline = fmt->bpp * pix->width;

	return 0;
}

/*
 * The real work of figuring out a workable format.
 */
static int imx7_vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	int ret;

	ret = imx7_vidioc_try_fmt_vid_cap(file, csi_dev, f);
	if (ret < 0)
		return ret;

	csi_dev->fmt = format_by_fourcc(f->fmt.pix.pixelformat);
	csi_dev->mbus_code = csi_dev->fmt->mbus_code;
	csi_dev->pix = f->fmt.pix;
	csi_dev->type = f->type;

	dev_dbg(csi_dev->dev, "set to pixelformat '%4.6s' colorspace: %d\n",
		(char *)&csi_dev->fmt->name, csi_dev->pix.colorspace);

	/* Config csi */
	imx7_configure_csi(csi_dev);

	return 0;
}

static int imx7_vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);

	f->fmt.pix = csi_dev->pix;
	f->fmt.pix.pixelformat = csi_dev->fmt->pixelformat;

	return 0;
}

static int imx7_vidioc_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);

	WARN_ON(priv != file->private_data);

	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->driver, IMX7_CAM_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->card, IMX7_CAM_DRIVER_DESCRIPTION, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(csi_dev->dev));

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int imx7_vidioc_streamon(struct file *file, void *priv,
				enum v4l2_buf_type i)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;
	int ret;

	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	ret = vb2_streamon(&csi_dev->vb2_vidq, i);
	if (!ret)
		v4l2_subdev_call(sd, video, s_stream, 1);

	return ret;
}

static int imx7_vidioc_streamoff(struct file *file, void *priv,
				 enum v4l2_buf_type i)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;

	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/*
	 * This calls buf_release from host driver's videobuf_queue_ops for all
	 * remaining buffers. When the last buffer is freed, stop capture
	 */
	vb2_streamoff(&csi_dev->vb2_vidq, i);

	v4l2_subdev_call(sd, video, s_stream, 0);

	return 0;
}

static int imx7_vidioc_cropcap(struct file *file, void *fh,
			       struct v4l2_cropcap *a)
{
	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return -ENODATA;
}

static int imx7_vidioc_g_crop(struct file *file, void *priv,
			      struct v4l2_crop *a)
{
	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return -ENODATA;
}

static int imx7_vidioc_s_crop(struct file *file, void *priv,
			      const struct v4l2_crop *a)
{
	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return -ENODATA;
}

static int imx7_vidioc_g_parm(struct file *file, void *priv,
			      struct v4l2_streamparm *a)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;

	return v4l2_subdev_call(sd, video, g_parm, a);
}

static int imx7_vidioc_s_parm(struct file *file, void *priv,
			      struct v4l2_streamparm *a)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;

	return v4l2_subdev_call(sd, video, s_parm, a);
}

static int imx7_vidioc_enum_framesizes(struct file *file, void *priv,
				       struct v4l2_frmsizeenum *fsize)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;
	struct imx7_fmt *fmt;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	fmt = format_by_fourcc(fsize->pixel_format);
	if (!fmt)
		return -EINVAL;

	if (fmt->pixelformat != fsize->pixel_format)
		return -EINVAL;

	fse.code = fmt->mbus_code;

	ret = v4l2_subdev_call(sd, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	if (fse.min_width == fse.max_width &&
	    fse.min_height == fse.max_height) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.min_width;
		fsize->discrete.height = fse.min_height;
		return 0;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->stepwise.min_width = fse.min_width;
	fsize->stepwise.max_width = fse.max_width;
	fsize->stepwise.min_height = fse.min_height;
	fsize->stepwise.max_height = fse.max_height;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int imx7_vidioc_enum_frameintervals(struct file *file, void *priv,
					   struct v4l2_frmivalenum *interval)
{
	struct imx7_csi_dev *csi_dev = video_drvdata(file);
	struct v4l2_subdev *sd = csi_dev->sd;
	struct imx7_fmt *fmt;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = interval->index,
		.width = interval->width,
		.height = interval->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	fmt = format_by_fourcc(interval->pixel_format);
	if (!fmt)
		return -EINVAL;

	if (fmt->pixelformat != interval->pixel_format)
		return -EINVAL;

	fie.code = fmt->mbus_code;

	ret = v4l2_subdev_call(sd, pad, enum_frame_interval, NULL, &fie);
	if (ret)
		return ret;

	interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	interval->discrete = fie.interval;

	return 0;
}

static const struct v4l2_ioctl_ops imx7_csi_ioctl_ops = {
	.vidioc_querycap		= imx7_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap	= imx7_vidioc_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= imx7_vidioc_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= imx7_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= imx7_vidioc_s_fmt_vid_cap,
	.vidioc_cropcap			= imx7_vidioc_cropcap,
	.vidioc_s_crop			= imx7_vidioc_s_crop,
	.vidioc_g_crop			= imx7_vidioc_g_crop,
	.vidioc_reqbufs			= imx7_vidioc_reqbufs,
	.vidioc_querybuf		= imx7_vidioc_querybuf,
	.vidioc_qbuf			= imx7_vidioc_qbuf,
	.vidioc_dqbuf			= imx7_vidioc_dqbuf,
	.vidioc_g_std			= imx7_vidioc_g_std,
	.vidioc_s_std			= imx7_vidioc_s_std,
	.vidioc_querystd		= imx7_vidioc_querystd,
	.vidioc_enum_input		= imx7_vidioc_enum_input,
	.vidioc_g_input			= imx7_vidioc_g_input,
	.vidioc_s_input			= imx7_vidioc_s_input,
	.vidioc_streamon		= imx7_vidioc_streamon,
	.vidioc_streamoff		= imx7_vidioc_streamoff,
	.vidioc_g_parm			= imx7_vidioc_g_parm,
	.vidioc_s_parm			= imx7_vidioc_s_parm,
	.vidioc_enum_framesizes		= imx7_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals	= imx7_vidioc_enum_frameintervals,
};

static int imx7_subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				      struct v4l2_subdev *subdev,
				      struct v4l2_async_subdev *asd)
{
	struct imx7_csi_dev *csi_dev = notifier_to_imx7_dev(notifier);
	struct v4l2_async_subdev *casd = &csi_dev->asd;
	int ret;

	/* Find platform data for this sensor subdev */
	if (casd->match.fwnode.fwnode == of_fwnode_handle(subdev->dev->of_node))
		csi_dev->sd = subdev;

	if (!csi_dev->sd)
		return -EINVAL;

	v4l2_info(&csi_dev->v4l2_dev, "Registered subdevice: %s\n",
		  subdev->name);

	ret = v4l2_ctrl_add_handler(&csi_dev->ctrl_handler,
				    subdev->ctrl_handler,
				    NULL);
	if (ret < 0)
		return ret;

	return 0;
}

static void imx7_subdev_notifier_unbind(struct v4l2_async_notifier *notifier,
					struct v4l2_subdev *subdev,
					struct v4l2_async_subdev *asd)
{
	struct imx7_csi_dev *csi_dev = notifier_to_imx7_dev(notifier);

	vb2_queue_release(&csi_dev->vb2_vidq);
}

static int imx7_probe_complete(struct v4l2_async_notifier *notifier)
{
	struct imx7_csi_dev *csi_dev = notifier_to_imx7_dev(notifier);

	return v4l2_device_register_subdev_nodes(&csi_dev->v4l2_dev);
}

static int imx7_csi_mux_sel(struct imx7_csi_dev *csi_dev)
{
	struct device_node *np = csi_dev->dev->of_node;
	struct device_node *node;
	phandle phandle;
	u32 out_val[3];
	int ret;

	ret = of_property_read_u32_array(np, "csi-mux-mipi", out_val, 3);
	if (ret) {
		dev_err(csi_dev->dev, "no csi-mux-mipi property found: %d\n",
			ret);
		csi_dev->csi_mux_mipi = false;
		return ret;
	}

	phandle = *out_val;

	node = of_find_node_by_phandle(phandle);
	if (!node) {
		ret = PTR_ERR(node);
		dev_err(csi_dev->dev, "not find gpr node by phandle: %d\n",
			ret);
		return ret;
	}

	csi_dev->csi_mux.gpr = syscon_node_to_regmap(node);
	if (IS_ERR(csi_dev->csi_mux.gpr)) {
		ret = PTR_ERR(csi_dev->csi_mux.gpr);
		dev_err(csi_dev->dev, "failed to get gpr regmap: %d\n", ret);
	}
	if (ret < 0)
		goto node_put;

	csi_dev->csi_mux.req_gpr = out_val[1];
	csi_dev->csi_mux.req_bit = out_val[2];

	regmap_update_bits(csi_dev->csi_mux.gpr, csi_dev->csi_mux.req_gpr,
			   1 << csi_dev->csi_mux.req_bit,
			   1 << csi_dev->csi_mux.req_bit);

	csi_dev->csi_mux_mipi = true;

node_put:
	of_node_put(node);

	return ret;
}

static int imx7x_register_subdevs(struct imx7_csi_dev *csi_dev)
{
	struct device_node *parent = csi_dev->dev->of_node;
	struct device_node *node, *port, *rem;
	int ret;

	/* Attach sensors linked to csi receivers */
	for_each_available_child_of_node(parent, node) {
		if (of_node_cmp(node->name, "port"))
			continue;

		/* The csi node can have only port subnode. */
		port = of_get_next_child(node, NULL);
		if (!port)
			continue;
		rem = of_graph_get_remote_port_parent(port);
		of_node_put(port);

		if (!rem) {
			v4l2_info(&csi_dev->v4l2_dev, "Remote device at %s not found\n",
				  port->full_name);
			return -EINVAL;
		}

		v4l2_info(&csi_dev->v4l2_dev, "Remote device at %s found\n",
			  rem->name);

		csi_dev->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		csi_dev->asd.match.fwnode.fwnode = of_fwnode_handle(rem);
		csi_dev->async_subdevs[0] = &csi_dev->asd;

		of_node_put(rem);
		break;
	}

	csi_dev->subdev_notifier.subdevs = csi_dev->async_subdevs;
	csi_dev->subdev_notifier.num_subdevs = 1;
	csi_dev->subdev_notifier.bound = imx7_subdev_notifier_bound;
	csi_dev->subdev_notifier.unbind = imx7_subdev_notifier_unbind;
	csi_dev->subdev_notifier.complete = imx7_probe_complete;

	ret = v4l2_async_notifier_register(&csi_dev->v4l2_dev,
					   &csi_dev->subdev_notifier);
	if (ret)
		dev_err(csi_dev->dev, "Error register async notifier\n");

	return ret;
}

static int imx7_csi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id;
	struct imx7_csi_dev *csi_dev;
	struct video_device *vdev;
	struct resource *res;
	struct vb2_queue *q;
	int ret = 0;

	dev_info(dev, "initialising\n");

	/* Prepare our private structure */
	csi_dev = devm_kzalloc(dev, sizeof(*csi_dev), GFP_KERNEL);
	if (!csi_dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi_dev->irq = platform_get_irq(pdev, 0);
	if (!res || csi_dev->irq < 0) {
		dev_err(dev, "Missing platform resources data\n");
		return -ENODEV;
	}

	csi_dev->regbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(csi_dev->regbase)) {
		dev_err(dev, "Failed platform resources map\n");
		return -ENODEV;
	}

	/* init video dma queues */
	INIT_LIST_HEAD(&csi_dev->capture);
	INIT_LIST_HEAD(&csi_dev->active_bufs);
	INIT_LIST_HEAD(&csi_dev->discard);

	csi_dev->clk_disp_axi = devm_clk_get(dev, "disp-axi");
	if (IS_ERR(csi_dev->clk_disp_axi)) {
		dev_err(dev, "Could not get csi axi clock\n");
		return -ENODEV;
	}

	csi_dev->clk_disp_dcic = devm_clk_get(dev, "disp_dcic");
	if (IS_ERR(csi_dev->clk_disp_dcic)) {
		dev_err(dev, "Could not get disp dcic clock\n");
		return -ENODEV;
	}

	csi_dev->clk_csi_mclk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(csi_dev->clk_csi_mclk)) {
		dev_err(dev, "Could not get csi mclk clock\n");
		return -ENODEV;
	}

	csi_dev->dev = dev;

	imx7_csi_mux_sel(csi_dev);

	of_id = of_match_node(imx7_csi_dt_ids, csi_dev->dev->of_node);
	if (!of_id)
		return -EINVAL;
	csi_dev->rx_fifo_rst = of_id->data;

	snprintf(csi_dev->v4l2_dev.name, sizeof(csi_dev->v4l2_dev.name), "CSI");

	/*
	 * this is to make sure the sensor already added its controls and that
	 * we can add them in the bound of the mipi subdev. it will be removed
	 *when moving to a media controller approach
	 *
	 */
	msleep(1000);

	ret = v4l2_device_register(dev, &csi_dev->v4l2_dev);
	if (ret < 0) {
		dev_err(dev, "v4l2_device_register() failed: %d\n", ret);
		return -ENODEV;
	}

	/* initialize locks */
	mutex_init(&csi_dev->lock);
	spin_lock_init(&csi_dev->slock);

	/* Allocate memory for video device */
	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto unregister_dev;
	}

	snprintf(vdev->name, sizeof(vdev->name), "imx7-csi");

	vdev->v4l2_dev		= &csi_dev->v4l2_dev;
	vdev->fops		= &imx7_csi_fops;
	vdev->ioctl_ops		= &imx7_csi_ioctl_ops;
	vdev->release		= video_device_release;
	vdev->lock		= &csi_dev->lock;

	vdev->queue = &csi_dev->vb2_vidq;

	csi_dev->vdev = vdev;

	video_set_drvdata(csi_dev->vdev, csi_dev);

	v4l2_ctrl_handler_init(&csi_dev->ctrl_handler, 0);
	vdev->ctrl_handler = &csi_dev->ctrl_handler;

	mutex_lock(&csi_dev->lock);

	ret = video_register_device(csi_dev->vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		video_device_release(csi_dev->vdev);
		mutex_unlock(&csi_dev->lock);
		goto unregister_dev;
	}
	dev_info(dev, "video device registered\n");

	/* install interrupt handler */
	if (devm_request_irq(dev, csi_dev->irq, imx7_csi_irq_handler, 0, "csi",
			     (void *)csi_dev)) {
		mutex_unlock(&csi_dev->lock);
		dev_err(dev, "Request CSI IRQ failed.\n");
		ret = -ENODEV;
		goto unregister_video;
	}

	q = &csi_dev->vb2_vidq;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = csi_dev;
	q->ops = &imx7_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct imx7_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->lock = &csi_dev->lock;
	q->dev = csi_dev->dev;

	ret = vb2_queue_init(q);
	if (ret < 0)
		return -ENOMEM;

	mutex_unlock(&csi_dev->lock);

	ret = imx7x_register_subdevs(csi_dev);
	if (ret < 0)
		goto unregister_video;

	csi_dev->fmt = format_by_fourcc(V4L2_PIX_FMT_SBGGR10);

	pm_runtime_enable(csi_dev->dev);

	return 0;

unregister_video:
	video_unregister_device(csi_dev->vdev);

unregister_dev:
	v4l2_device_unregister(&csi_dev->v4l2_dev);
	v4l2_ctrl_handler_free(&csi_dev->ctrl_handler);
	mutex_destroy(&csi_dev->lock);

	return ret;
}

static int imx7_csi_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = dev_get_drvdata(&pdev->dev);
	struct imx7_csi_dev *csi_dev;

	csi_dev = container_of(v4l2_dev, struct imx7_csi_dev, v4l2_dev);

	v4l2_async_notifier_unregister(&csi_dev->subdev_notifier);

	video_unregister_device(csi_dev->vdev);
	v4l2_device_unregister(&csi_dev->v4l2_dev);
	v4l2_ctrl_handler_free(&csi_dev->ctrl_handler);
	mutex_destroy(&csi_dev->lock);

	pm_runtime_disable(csi_dev->dev);
	return 0;
}

static int imx7_csi_runtime_suspend(struct device *dev)
{
	return 0;
}

static int imx7_csi_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops imx7_csi_pm_ops = {
	SET_RUNTIME_PM_OPS(imx7_csi_runtime_suspend, imx7_csi_runtime_resume,
			   NULL)
};

static const u8 imx7_fifo_rst = true;
static const u8 imx7l_fifo_rst;

static const struct of_device_id imx7_csi_dt_ids[] = {
	{ .compatible = "fsl,imx7-csi",
	  .data = &imx7_fifo_rst,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx7_csi_dt_ids);

static struct platform_driver imx7_csi_driver = {
	.driver		= {
		.name	= IMX7_CAM_DRV_NAME,
		.of_match_table = of_match_ptr(imx7_csi_dt_ids),
		.pm = &imx7_csi_pm_ops,
	},
	.probe	= imx7_csi_probe,
	.remove	= imx7_csi_remove,
};
module_platform_driver(imx7_csi_driver);

MODULE_DESCRIPTION("i.IMX7x SoC Camera Host driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION(IMX7_CAM_VERSION);

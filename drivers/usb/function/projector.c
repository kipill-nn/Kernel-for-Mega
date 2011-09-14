
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>

#include <linux/wait.h>
#include <linux/list.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/msm_fb.h>
#include "usb_function.h"

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_INFO x)
#endif

/*16KB*/
#define TXN_MAX 16384
#define RXN_MAX 4096

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 30

#define BITSPIXEL 16

#define PROJECTOR_FUNCTION_NAME "projector"

//extern char *fbaddr;
//extern int msmfb_area;

struct device prj_dev;
static int enabled = 0;

struct projector_context
{
	int online;
	int error;
	int registered;

	char *ok;
	u32 bitsPixel;
	u32 framesize;
	u32 width;
	u32 heigh;

	int multiple;
	int remainder;

	spinlock_t lock;

	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;
	struct list_head rx_idle;

	struct platform_device *pdev;

	char *fbaddr;

};

static struct projector_context _context;

/* add a request to the tail of a list */
static void put_req(struct projector_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;
	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *get_req(struct projector_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}

static void projector_queue_out(struct projector_context *ctxt)
{
	int ret;
	struct usb_request *req;

	/* if we have idle read requests, get them queued */
	while ((req = get_req(ctxt, &ctxt->rx_idle))) {
		req->length = RXN_MAX;
		ret = usb_ept_queue_xfer(ctxt->out, req);
		if (ret < 0) {
			DBG("projector_queue_out: failed to queue req %p (%d)\n", req, ret);
			ctxt->error = 1;
			put_req(ctxt, &ctxt->rx_idle, req);
		}
	}
}

static void send_fb(struct projector_context *ctxt)
{

	struct usb_request *req;
	int ret;
	char *frame;
	unsigned int fblen = 0;
	//int count = 0;


	if(!list_empty(&ctxt->tx_idle))
	{
		//spin_lock_irqsave(&ctxt->lock, flags);
		if (msmfb_get_fb_area())
			frame = (ctxt->fbaddr + ctxt->framesize);
		else
			frame = ctxt->fbaddr;
		//spin_unlock_irqrestore(&ctxt->lock, flags);
		while(fblen < ctxt->framesize) {
			req = get_req(ctxt, &ctxt->tx_idle);
			if (req) {
				req->length = fblen < ctxt->multiple? TXN_MAX : ctxt->remainder;
				//req->length = TXN_MAX;
				memcpy(req->buf, frame + fblen, req->length);
				//req->buf = frame + fblen;
				//spin_unlock_irqrestore(&ctxt->lock, flags);
				if ((ret=usb_ept_queue_xfer(ctxt->in, req))<0) {
					printk(KERN_WARNING "send_fb: cannot queue bulk in request, ret=%d\n",ret);
					//spin_lock_irqsave(&ctxt->lock, flags);
					break;
				}
				//spin_lock_irqsave(&ctxt->lock, flags);
				fblen += (req->length);
			}
			else{
				DBG("send_fb: There are no req to send\n");
				break;
			}
		}
		//DBG("send_fb:write to PC Length:%d\n", fblen);
		//spin_unlock_irqrestore(&ctxt->lock, flags);
	}
	else
		printk(KERN_ERR "send_fb: ctxt->tx_idle list is empty\n");
}

static void send_info(struct projector_context *ctxt)
{
	struct usb_request *req;

	if(!list_empty(&ctxt->tx_idle))
	{
		req = get_req(ctxt, &ctxt->tx_idle);
		if (req) {
			req->length = 20;
			memcpy(req->buf, ctxt->ok, 4);
			memcpy(req->buf + 4, &ctxt->bitsPixel, 4);
			memcpy(req->buf + 8, &ctxt->framesize, 4);
			memcpy(req->buf + 12, &ctxt->width, 4);
			memcpy(req->buf + 16, &ctxt->heigh, 4);
			if (usb_ept_queue_xfer(ctxt->in, req)<0)
				printk(KERN_WARNING "send_info: Can't queue bulk in request\n");
		}
		else
			DBG("send_info: There are no req to send\n");
	}
	else
		printk(KERN_ERR "send_info: ctxt->tx_idle list is empty\n");
}

static void projector_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct projector_context *ctxt = req->context;
	//DBG("BULK IN Complete Length %d\n", req->actual);

	switch(req->status) {
	case 0:
		/* normal completion */
requeue:
		put_req(ctxt, &ctxt->tx_idle, req);
		break;

	case -ESHUTDOWN:
		/* disconnect */
		DBG("projector_complete_in: shutdown\n");
		//usb_ept_free_req(ep, req);
		goto requeue;
		break;

	case -ENODEV:
		DBG("projector_complete_in: nodev\n");
		put_req(ctxt, &ctxt->tx_idle, req);
		break;

	default:
		/* unexpected */
		printk(KERN_ERR "projector_complete_in: unexpected status error, status=%d\n",
			req->status);
		goto requeue;
		break;
	}

}

static void projector_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct projector_context *ctxt = req->context;
	unsigned char *data = req->buf;
	//DBG("Bulk Out Complete: data %02x , length %d\n", *((char *)(req->buf)), req->actual);
	if (req->status != 0) {
		ctxt->error = 1;
		put_req(ctxt, &ctxt->rx_idle, req);
		DBG("projector_complete_out status %d\n", req->status);
		return ;
	}

	if(!strncmp("init", data, 4))
	{
		send_info(ctxt);
	}
	else if(*data == ' ')
	{
		send_fb(ctxt);
	}
	else
	{
		printk(KERN_ERR "USB projector function:Unknow command\n");
	}

	put_req(ctxt, &ctxt->rx_idle, req);
	projector_queue_out(ctxt);

}

static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long ul;
	if (count > 2 || (buf[0] != '0' && buf[0] != '1'))
	{
		printk(KERN_WARNING "Can't enable/disable projector %s\n", buf);
		return -EINVAL;;
	}
	ul = simple_strtoul(buf, NULL, 10);
	enabled = usb_function_enable(PROJECTOR_FUNCTION_NAME, ul);
	return count;
}

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	if (enabled)
	{
		buf[0] = '1';
		buf[1] = '\n';
	}
	else
	{
		buf[0] = '0';
		buf[1] = '\n';
	}
	return 2;

}
/* in shipmode, adb only has user privilege, not root*/
static DEVICE_ATTR(enable, 0666, show_enable, store_enable);

static void prj_dev_release (struct device *dev) {}

static void projector_unbind(void *_ctxt)
{
	struct projector_context *ctxt = _ctxt;
	struct usb_request *req;

	while ((req = get_req(ctxt, &ctxt->rx_idle))) {
		usb_ept_free_req(ctxt->out, req);
	}
	while ((req = get_req(ctxt, &ctxt->tx_idle))) {
		usb_ept_free_req(ctxt->in, req);
	}

	if (ctxt->registered == 1)
	{
		device_remove_file(&prj_dev, &dev_attr_enable);
		device_unregister(&prj_dev);
		ctxt->registered = 0;
	}

	ctxt->online = 0;
	ctxt->error = 1;
}

static void projector_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct projector_context *ctxt = _ctxt;
	struct usb_request *req;
	struct msm_fb_info fb_info;
	int n, rc;

	ctxt->out = ept[0];
	ctxt->in = ept[1];

	ctxt->registered = 0;

	for (n = 0; n < RX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->out, RXN_MAX);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = projector_complete_out;
		put_req(ctxt, &ctxt->rx_idle, req);
	}

	for (n = 0; n < TX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->in, TXN_MAX);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = projector_complete_in;
		//for (i = 0; i<TXN_MAX; i++)
		//	*((char *)(req->buf + i)) = 0xfb;
		put_req(ctxt, &ctxt->tx_idle, req);
	}

	msmfb_get_var(&fb_info);

	ctxt->ok = "okay";
	ctxt->bitsPixel = BITSPIXEL;
	ctxt->width = fb_info.xres;
	ctxt->heigh = fb_info.yres;
	ctxt->fbaddr = fb_info.fb_addr;

	ctxt->framesize = (ctxt->width)*(ctxt->heigh)*2;
	ctxt->remainder = ctxt->framesize % TXN_MAX;
	ctxt->multiple = ctxt->framesize - ctxt->remainder;

	prj_dev.release = prj_dev_release;
	prj_dev.parent = &ctxt->pdev->dev;
	strcpy(prj_dev.bus_id, "interface");

	rc = device_register(&prj_dev);
	if (rc != 0) {
		DBG("projector failed to register device: %d\n", rc);
		goto fail;
	}
	rc = device_create_file(&prj_dev, &dev_attr_enable);
	if (rc != 0) {
		DBG("projector device_create_file failed: %d\n", rc);
		device_unregister(&prj_dev);
		goto fail;
	}
	ctxt->registered = 1;
	return;

fail:
	printk(KERN_WARNING "projector_bind() could not allocate requests\n");
	projector_unbind(ctxt);
}

static void projector_configure(int configured, void *_ctxt)
{
	struct projector_context *ctxt = _ctxt;

	if (configured) {
		ctxt->online = 1;
    	projector_queue_out(ctxt);
	} else {
		ctxt->online = 0;
		ctxt->error = 1;
	}

}

static struct usb_function usb_func_projector = {
	.bind = projector_bind,
	.unbind = projector_unbind,
	.configure = projector_configure,

	.name = PROJECTOR_FUNCTION_NAME,
	.context = &_context,

	.ifc_class = 0xff,
	.ifc_subclass = 0xff,
	.ifc_protocol = 0xff,

	.ifc_name = "projector",

	.ifc_ept_count = 2,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },

	.disabled = 1,
	.position_bit = USB_FUNCTION_PROJECTOR_NUM,
	.cdc_desc = NULL,
	.ifc_num = 1,
	.ifc_index = STRING_PROJECTOR,
};

static int pjr_probe (struct platform_device *pdev)
{
	struct projector_context *ctxt = &_context;
	ctxt->pdev = pdev;
	return 0;
}

static struct platform_driver pjr_driver = {
	.probe = pjr_probe,
	.driver = { .name = PROJECTOR_FUNCTION_NAME, },
};

static void pjr_release (struct device *dev) {}


static struct platform_device pjr_device = {
	.name		= PROJECTOR_FUNCTION_NAME,
	.id		= -1,
	.dev		= {
		.release	= pjr_release,
	},
};
static int __init projector_init(void)
{
	int r;
	struct projector_context *ctxt = &_context;

	spin_lock_init(&ctxt->lock);

	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->tx_idle);

	r = platform_driver_register (&pjr_driver);
	if (r < 0)
		return r;
	r = platform_device_register (&pjr_device);
	if (r < 0)
		goto err_register_device;
	r = usb_function_register(&usb_func_projector);
	if (r <0)
		goto err_register_function;
	return r;

err_register_function:
	platform_device_unregister(&pjr_device);
err_register_device:
	platform_driver_unregister(&pjr_driver);
	return r;

}

module_init(projector_init);

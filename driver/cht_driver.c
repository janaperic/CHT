#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/device.h>

#include <linux/io.h>	//iowrite ioread
#include <linux/slab.h>	//kmalloc kfree
#include <linux/platform_device.h>	//platform driver
#include <linux/of.h>	//of_match_table
#include <linux/ioport.h>	//ioremap

#include <linux/dma-mapping.h>  //dma access
#include <linux/mm.h>  //dma access
#include <linux/interrupt.h>  //interrupt handlers

MODULE_AUTHOR ("FTN");
MODULE_DESCRIPTION("Test Driver for Circle Hough Transform IP.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("custom:cht ip core driver");

#define DEVICE_NAME "cht"
#define DRIVER_NAME "cht_driver"

unsigned int TX_PKT_LEN; //size of transmit data
unsigned int RX_PKT_LEN; // size of receive data
char FINISHED[5] = "0";

//*******************FUNCTION PROTOTYPES************************************
static int cht_probe(struct platform_device *pdev);
static int cht_open(struct inode *i, struct file *f);
static int cht_close(struct inode *i, struct file *f);
static ssize_t cht_read(struct file *f, char __user *buf, size_t len, loff_t *off);
static ssize_t cht_write(struct file *f, const char __user *buf, size_t length, loff_t *off);
static ssize_t cht_mmap(struct file *f, struct vm_area_struct *vma_s);
static int __init cht_init(void);
static void __exit cht_exit(void);
static int cht_remove(struct platform_device *pdev);

static irqreturn_t dma_isr(int irq,void *dev_id);
int dma_init(void __iomem *base_address);
u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address); 
u32 dma_simple_read(dma_addr_t RxBufferPtr, u32 max_pkt_len, void __iomem *base_address); 

//*********************GLOBAL VARIABLES*************************************
struct cht_info {
  unsigned long mem_start;
  unsigned long mem_end;
  void __iomem *base_addr;
  int irq_num;
};

static struct cdev *my_cdev;
static dev_t my_dev_id;
static struct class *my_class;
static struct device *my_device;
static struct cht_info *vp = NULL;

static struct file_operations my_fops =
{
	.owner = THIS_MODULE,
	.open = cht_open,
	.release = cht_close,
	.read = cht_read,
	.write = cht_write,
	.mmap = cht_mmap
};

//Struct used for matching a device
static struct of_device_id cht_of_match[] = {
	{ .compatible = "cht_dma", },
	{ .compatible = "xlnx,axi-dma-mm2s-channel", },
	{ .compatible = "xlnx,axi-dma-s2mm-channel", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, cht_of_match);

static struct platform_driver cht_driver = {
	.driver = {
		.name 			= DRIVER_NAME,
		.owner 			= THIS_MODULE,
		.of_match_table	= cht_of_match,
	},
	.probe	= cht_probe,
	.remove	= cht_remove,
};


dma_addr_t tx_phy_buffer, rx_phy_buffer;
u32 *tx_vir_buffer, *rx_vir_buffer;

//***************************************************************************
// PROBE AND REMOVE
// Function probe() should in general verify that the specified device hardware actually exists
// This function gets called as soon as we insmod the module
static int cht_probe(struct platform_device *pdev)
{
	struct resource *r_mem;
	int rc = 0;

	printk(KERN_INFO "cht_probe: Probing\n");
	// Get physical register address space from device tree
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		printk(KERN_ALERT "cht_probe: Failed to get reg resource\n");
		return -ENODEV;
	}
	// Allocate memory for structure cht_info
	vp = (struct cht_info *) kmalloc(sizeof(struct cht_info), GFP_KERNEL);
	if (!vp) {
		printk(KERN_ALERT "cht_probe: Could not allocate memory for structure cht_info\n");
		return -ENOMEM;
	}
	// Put physical adresses in timer_info structure
	vp->mem_start = r_mem->start;
	vp->mem_end = r_mem->end;

	// Reserve that memory space for this driver
	if (!request_mem_region(vp->mem_start,vp->mem_end - vp->mem_start + 1, DRIVER_NAME))
	{
		printk(KERN_ALERT "cht_probe: Could not lock memory region at %p\n",(void *)vp->mem_start);
		rc = -EBUSY;
		goto error1;
	}    
	// Remap physical to virtual adresses
	vp->base_addr = ioremap(vp->mem_start, vp->mem_end - vp->mem_start + 1);
	if (!vp->base_addr) {
		printk(KERN_ALERT "cht_probe: Could not allocate memory for remapping\n");
		rc = -EIO;
		goto error2;
	}

	// Get irq number
	vp->irq_num = platform_get_irq(pdev, 0);
	if(!vp->irq_num)
	{
		printk(KERN_ERR "cht_probe: Could not get IRQ resource\n");
		rc = -ENODEV;
		goto error2;
	}

	if (request_irq(vp->irq_num, dma_isr, 0, DEVICE_NAME, NULL)) {
		printk(KERN_ERR "cht_probe: Could not register IRQ %d\n", vp->irq_num);
		return -EIO;
		goto error3;
	}
	else {
		printk(KERN_INFO "cht_probe: Registered IRQ %d\n", vp->irq_num);
	}

	/* INIT DMA */
	dma_init(vp->base_addr);

	printk(KERN_NOTICE "cht_probe: CHT platform driver registered\n");
	return 0;

error3:
	iounmap(vp->base_addr);
error2:
	release_mem_region(vp->mem_start, vp->mem_end - vp->mem_start + 1);
	kfree(vp);
error1:
	return rc;

}

static int cht_remove(struct platform_device *pdev)
{
	u32 reset = 0x00000004;
	// writing to MM2S_DMACR and SS2M_DMACR registers
	printk(KERN_INFO "cht_remove: reseting");
	iowrite32(reset, vp->base_addr); 
	iowrite32(0x1, vp->base_addr + 52);

	free_irq(vp->irq_num, NULL);
	iounmap(vp->base_addr);
	release_mem_region(vp->mem_start, vp->mem_end - vp->mem_start + 1);
	kfree(vp);
	printk(KERN_INFO "cht_remove: CHT platform removed");
	return 0;
}

//***************************************************
// IMPLEMENTATION OF FILE OPERATION FUNCTIONS
static int cht_open(struct inode *i, struct file *f)
{
	//printk("cht opened\n");
	return 0;
}

static int cht_close(struct inode *i, struct file *f)
{
	printk("cht_closed: Driver closed.\n");
	dma_free_coherent(NULL, TX_PKT_LEN, tx_vir_buffer, tx_phy_buffer);
	dma_free_coherent(NULL, RX_PKT_LEN, rx_vir_buffer, rx_phy_buffer);

	return 0;
}

static ssize_t cht_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	int ret = 0;
	printk(KERN_NOTICE "cht_read: Cheking if driver is ready to send pixels.\n");
	ret = copy_to_user(buf, FINISHED, len);
	if(ret){
		printk("Copy to user failed \n");
		return -EFAULT;
	}  

	return 0;
}

static ssize_t cht_write(struct file *f, const char __user *buf, size_t length, loff_t *off)
{	
	char buff[100] = "";
	int ret = 0;
	int numw = 0;
	int i = 0;
	ret = copy_from_user(buff, buf, length);  
	if(ret){
		printk("Copy from user failed \n");
		return -EFAULT;
	}  
	sscanf(buff, "%d", &numw);
	if(numw != 1) //If numw is not 1, app is sending the number of white pixels
	{
		printk("cht_write: Number of white pixels = %d\n", numw);

		TX_PKT_LEN = (numw + 1) * 4;
		RX_PKT_LEN = numw * 360 * 4;

		printk("cht_write: TX_PKT_LEN = %d\n", TX_PKT_LEN);
		printk("cht_write: RX_PKT_LEN = %d\n", RX_PKT_LEN);

		//Allocating memory for TX channel
		tx_vir_buffer = dma_alloc_coherent(NULL, TX_PKT_LEN, &tx_phy_buffer, GFP_DMA | GFP_KERNEL);
		if(!tx_vir_buffer)
		{
			printk(KERN_ALERT "cht_write: Could not allocate memory for TX buffer");
			return -1;
		}
		else
			printk("cht_write: Successfully allocated memory for DMA TX buffer\n");

		//Allocating memory for RX channel
		rx_vir_buffer = dma_alloc_coherent(NULL, RX_PKT_LEN, &rx_phy_buffer, GFP_DMA | GFP_KERNEL);
		if(!rx_vir_buffer)
		{
			printk(KERN_ALERT "cht_write: Could not allocate memory for RX buffer");
			return -1;
		}
		else
			printk("cht_write: Successfully allocated memory for DMA RX buffer\n");

		//Resetting buffers
		for (i = 0; i < TX_PKT_LEN/4;i++)
			tx_vir_buffer[i] = 0x00000000;
		for (i = 0; i < RX_PKT_LEN/4;i++)
			rx_vir_buffer[i] = 0x00000000;
		printk(KERN_INFO "cht_write: DMA memory reset.\n");
	}
	else //When numw is 1, it's the signal to start the first transaction
	{
		printk("cht_write: Start the transaction\n");
		//printk("tx_vir_buffer[31] = %d\n", tx_vir_buffer[31]);
		dma_simple_write(tx_phy_buffer, TX_PKT_LEN, vp->base_addr);
	}

	return 0;
}

static ssize_t cht_mmap(struct file *f, struct vm_area_struct *vma_s)
{
	int ret, val;
	long length = vma_s->vm_end - vma_s->vm_start;
	printk(KERN_NOTICE "cht_mmap: Memory mapping\n");
	printk(KERN_NOTICE "length: %ld\n", length);

	ret = 0;
	val = 0;

	if(length < 500000)
	{
		//Memory mapping of the TX buffer
		ret = dma_mmap_coherent(NULL, vma_s, tx_vir_buffer, tx_phy_buffer, length);
		if(ret<0)
		{
			printk(KERN_ERR "TX memory map failed\n");
			return ret;
		}
		
	}
	else
	{	
		//Memory mapping od the RX buffer
		val = dma_mmap_coherent(NULL, vma_s, rx_vir_buffer, rx_phy_buffer, length);
		if(val<0)
		{
			printk(KERN_ERR "RX memory map failed\n");
			return val;

		}
	
	}

	return 0;
}

/****************************************************/
// IMPLEMENTATION OF DMA related functions

static irqreturn_t dma_isr(int irq,void*dev_id)
{
	u32 IrqStatus;  
	printk(KERN_NOTICE "dma_isr: An interrupt has occured\n");

	/* Read pending interrupts */
	IrqStatus = ioread32(vp->base_addr + 52);//Read irq status from S2MM_DMASR register
	iowrite32(IrqStatus | 0x00007000, vp->base_addr + 52);//Clear irq status in S2MM_DMASR register

	//Send the signal to the app that IP has finished
	strcpy(FINISHED, "1");
	printk(KERN_NOTICE "dma_simple_read: Driver is ready to send pixels\n");
	return IRQ_HANDLED;
}

int dma_init(void __iomem *base_address)
{
	u32 reset = 0x00000004;
	u32 IOC_IRQ_EN_S2MM; 
	u32 ERR_IRQ_EN_S2MM;
	u32 en_interrupt;
	u32 S2MM_DMACR_reg;
	printk(KERN_NOTICE "dma_init: Initialising DMA\n");

	//Configuring the reset bit in MM2S channel
	iowrite32(reset, base_address); // Writing to MM2S_DMACR register. Seting reset bit (3. bit)  

	//Configuring the reset bit and interrupt in S2MM channel
	S2MM_DMACR_reg = ioread32(base_address + 52);
	iowrite32(0x1 | S2MM_DMACR_reg, base_address + 52);

	IOC_IRQ_EN_S2MM = 1 << 12; //This is IOC_IrqEn bit in S2MM_DMACR register
	ERR_IRQ_EN_S2MM = 1 << 14; //This is Err_IrqEn bit in S2MM_DMACR register
	en_interrupt = S2MM_DMACR_reg | IOC_IRQ_EN_S2MM | ERR_IRQ_EN_S2MM;
	iowrite32(en_interrupt, base_address + 48);

	return 0;
}

//Confguration of MM2S channel
u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address) 
{
	u32 MM2S_DMACR_reg;
	printk(KERN_NOTICE "dma_simple_write: Writing pixels\n");
	strcpy(FINISHED, "0"); // Reset finish signal

	//Reading the current configuration from MM2S_DMACR register
	MM2S_DMACR_reg = ioread32(base_address); 

	//Set RS bit in MM2S_DMACR register (this bit starts the DMA)
	iowrite32(0x1 |  MM2S_DMACR_reg, base_address); 

	//Write into MM2S_SA register the value of TxBufferPtr.
	//With this, the DMA knows from where to start.
	iowrite32((u32)TxBufferPtr, base_address + 24); 

	//Write into MM2S_LENGTH register. This is the length of a tranaction.
	iowrite32(max_pkt_len, base_address + 40); 
	
	//Start reading data from IP
	dma_simple_read(rx_phy_buffer, RX_PKT_LEN, vp->base_addr);
	return 0;
}

//Configuration of S2MM channel
u32 dma_simple_read(dma_addr_t RxBufferPtr, u32 max_pkt_len, void __iomem *base_address)
{
	u32 S2MM_DMACR_reg;

	//Reading the current configuration from S2MM_DMACR register
	S2MM_DMACR_reg = ioread32(base_address + 48);

	//Setting the RS bit of S2MM_DMACR register
	iowrite32(0x1 | S2MM_DMACR_reg, base_address + 48); 

	//Setting the S2MM_DA register
	iowrite32((u32)RxBufferPtr, base_address + 72); 

	//Setting the S2MM_LENGTH register
	iowrite32(max_pkt_len, base_address + 88); 

	return 0;
}


//***************************************************
// INIT AND EXIT FUNCTIONS OF THE DRIVER
static int __init cht_init(void)
{

	int ret = 0;
	//int i = 0;

	printk(KERN_INFO "cht_init: Initialize Module \"%s\"\n", DEVICE_NAME);
	ret = alloc_chrdev_region(&my_dev_id, 0, 1, "CHT_region");
	if (ret)
	{
		printk(KERN_ALERT "cht_init: Failed CHRDEV!\n");
		return -1;
	}
	printk(KERN_INFO "cht_init: Successful CHRDEV!\n");
	my_class = class_create(THIS_MODULE, "CHT_drv");
	if (my_class == NULL)
	{
		printk(KERN_ALERT "cht_init: Failed class create!\n");
		goto fail_0;
	}
	printk(KERN_INFO "cht_init: Successful class chardev1 create!\n");
	my_device = device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),0), NULL, "cht");
	if (my_device == NULL)
	{
		goto fail_1;
	}

	printk(KERN_INFO "cht_init: Device created\n");

	my_cdev = cdev_alloc();	
	my_cdev->ops = &my_fops;
	my_cdev->owner = THIS_MODULE;
	ret = cdev_add(my_cdev, my_dev_id, 1);
	if (ret)
	{
		printk(KERN_ERR "cht_init: Failed to add cdev\n");
		goto fail_2;
	}
	printk(KERN_INFO "cht_init: Module init done\n");

	return platform_driver_register(&cht_driver);

fail_2:
	device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
fail_1:
	class_destroy(my_class);
fail_0:
	unregister_chrdev_region(my_dev_id, 1);
	return -1;

} 

// Exit Device Module
static void __exit cht_exit(void)  		
{
	platform_driver_unregister(&cht_driver);
	cdev_del(my_cdev);
	device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
	class_destroy(my_class);
	unregister_chrdev_region(my_dev_id, 1);
	printk(KERN_INFO "cht_exit: Exit device module finished\"%s\".\n", DEVICE_NAME);
}

module_init(cht_init);
module_exit(cht_exit);


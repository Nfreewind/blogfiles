#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/timer.h>  /*timer*/

#include <asm/uaccess.h>

//#define CONFIG_RALINK_MT7628

#include <asm/rt2880/rt_mmap.h>
#include <asm/rt2880/surfboardint.h>

//#define SURFBOARDINT_GPIO        6      /* GPIO */

#define RALINK_SYSCTL_ADDR		RALINK_SYSCTL_BASE	// system control
#define RALINK_REG_GPIOMODE		(RALINK_SYSCTL_ADDR + 0x60)
#define RALINK_REG_GPIOMODE2		(RALINK_SYSCTL_ADDR + 0x64)

#define RALINK_REG_AGPIO_CFG    (RALINK_SYSCTL_ADDR + 0x3c)

#define RALINK_IRQ_ADDR			RALINK_INTCL_BASE
#define RALINK_PRGIO_ADDR		RALINK_PIO_BASE // Programmable I/O

#define RALINK_REG_INTENA		(RALINK_IRQ_ADDR   + 0x80)
#define RALINK_REG_INTDIS		(RALINK_IRQ_ADDR   + 0x78)

//GPIO0 to GPIO31 interrupt status register
#define RALINK_REG_PIOINT		(RALINK_PRGIO_ADDR + 0x90)
//GPIO0 to GPIO31 edge status register
#define RALINK_REG_PIOEDGE		(RALINK_PRGIO_ADDR + 0xA0)
#define RALINK_REG_PIORENA		(RALINK_PRGIO_ADDR + 0x50)
#define RALINK_REG_PIOFENA		(RALINK_PRGIO_ADDR + 0x60)
#define RALINK_REG_PIODATA		(RALINK_PRGIO_ADDR + 0x20)
#define RALINK_REG_PIODIR		(RALINK_PRGIO_ADDR + 0x00)
#define RALINK_REG_PIOSET		(RALINK_PRGIO_ADDR + 0x30)
#define RALINK_REG_PIORESET		(RALINK_PRGIO_ADDR + 0x40)

#define RALINK_REG_PIO6332INT		(RALINK_PRGIO_ADDR + 0x94)
#define RALINK_REG_PIO6332EDGE		(RALINK_PRGIO_ADDR + 0xA4)
#define RALINK_REG_PIO6332RENA		(RALINK_PRGIO_ADDR + 0x54)
#define RALINK_REG_PIO6332FENA		(RALINK_PRGIO_ADDR + 0x64)
#define RALINK_REG_PIO6332DATA		(RALINK_PRGIO_ADDR + 0x24)
#define RALINK_REG_PIO6332DIR		(RALINK_PRGIO_ADDR + 0x04)
#define RALINK_REG_PIO6332SET		(RALINK_PRGIO_ADDR + 0x34)
#define RALINK_REG_PIO6332RESET		(RALINK_PRGIO_ADDR + 0x44)

#define RALINK_REG_PIO9564INT		(RALINK_PRGIO_ADDR + 0x98)
#define RALINK_REG_PIO9564EDGE		(RALINK_PRGIO_ADDR + 0xA8)
#define RALINK_REG_PIO9564RENA		(RALINK_PRGIO_ADDR + 0x58)
#define RALINK_REG_PIO9564FENA		(RALINK_PRGIO_ADDR + 0x68)
#define RALINK_REG_PIO9564DATA		(RALINK_PRGIO_ADDR + 0x28)
#define RALINK_REG_PIO9564DIR		(RALINK_PRGIO_ADDR + 0x08)
#define RALINK_REG_PIO9564SET		(RALINK_PRGIO_ADDR + 0x38)
#define RALINK_REG_PIO9564RESET		(RALINK_PRGIO_ADDR + 0x48)

#define RALINK_GPIOMODE_GPIO		0x1
#define RALINK_GPIOMODE_SPI_SLAVE	0x4
#define RALINK_GPIOMODE_SPI_CS1		0x10
#define RALINK_GPIOMODE_I2S		0x40
#define RALINK_GPIOMODE_UART1		0x100
#define RALINK_GPIOMODE_SDXC		0x400
#define RALINK_GPIOMODE_SPI		0x1000
#define RALINK_GPIOMODE_WDT		0x4000
#define RALINK_GPIOMODE_PERST		0x10000
#define RALINK_GPIOMODE_REFCLK		0x40000
#define RALINK_GPIOMODE_I2C		0x100000
#define RALINK_GPIOMODE_EPHY		0x40000
#define RALINK_GPIOMODE_P0LED		0x100000
#define RALINK_GPIOMODE_WLED		0x400000
#define RALINK_GPIOMODE_UART2		0x1000000
#define RALINK_GPIOMODE_UART3		0x4000000
#define RALINK_GPIOMODE_PWM0		0x10000000
#define RALINK_GPIOMODE_PWM1		0x40000000

#define RALINK_GPIOMODE_DFT		( RALINK_GPIOMODE_PWM0 | RALINK_GPIOMODE_PWM1 | RALINK_GPIOMODE_WLED | RALINK_GPIOMODE_UART3 | RALINK_GPIOMODE_SPI_CS1 | RALINK_GPIOMODE_WDT | RALINK_GPIOMODE_I2S | RALINK_GPIOMODE_REFCLK)

#define DEVICE_NAME "cgpio"

struct led_blink_args {
	unsigned int gpio;
	unsigned int enable;
	unsigned int timer;
};

struct led_blink {
	int mode; //0 默认,1 设置  2 定时
	int value;

	unsigned long timer; //定时间隔时间
	unsigned long timer_tmp; //上次切换的时间
};

struct led_blink _led_blink[50];

struct timer_list _my_led_timer;

static DECLARE_WAIT_QUEUE_HEAD(my_waitq);
static volatile int ev_press = 0;                //中断事件标志, 中断服务程序将它置1
volatile int _irq_count = 0;

static int get_gpio_value(int gpio) {
	unsigned int v = 0;
	u32 tmp;
	if (gpio < 32) {
		v = 1 << gpio;
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA)); //
	} else if (gpio < 64) {
		v = 1 << (gpio - 32);
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DATA)); //
	} else {
		v = 1 << (gpio - 64);
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DATA)); //
	}
	v = tmp & v;
	return v == 0 ? 0 : 1;
}

static void set_gpio_value(int gpio, int value) {
	int v;
	u32 tmp = 0;

	if (gpio < 32) {
		v = 0x01 << gpio;
		if (value) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOSET));
			tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIOSET) = cpu_to_le32( v); //GPIO0 to GPIO31 data set register
		} else
			*(volatile u32 *) (RALINK_REG_PIORESET) = cpu_to_le32(v); //GPIO0 to GPIO31 data clear register
	} else if (gpio < 64) {
		v = 0x01 << (gpio - 32);
		if (value) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332SET));
			tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIO6332SET) = cpu_to_le32(v); //GPIO32 to GPIO63 data set register
		} else {
			*(volatile u32 *) (RALINK_REG_PIO6332RESET) = cpu_to_le32(v);
		}
	} else {
		v = 0x01 << (gpio - 64);
		if (value) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564SET));
			tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIO9564SET) = cpu_to_le32(v); //GPIO64 to GPIO95 data set register
		} else {
			*(volatile u32 *) (RALINK_REG_PIO9564RESET) = cpu_to_le32(v);
		}
	}
}

//reg s 0
//reg r
static irqreturn_t my_irq_handler(int irq, void *dev_id) {
	//ev_press = 1;
	//wake_up_interruptible(&my_waitq);
	unsigned long now;
	int i;
	u32 _gpio_intp;
	u32 _gpio_edge;

	_gpio_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOINT));
	_gpio_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOEDGE));

	//1: Interrupt is detected
	*(volatile u32 *) (RALINK_REG_PIOINT) = cpu_to_le32( 0xFFFFFFFF);
	*(volatile u32 *) (RALINK_REG_PIOEDGE) = cpu_to_le32(0xFFFFFFFF);
	now = jiffies;

	//printk(KERN_EMERG"irq state 0x%08x\n", _gpio_intp);
	//printk(KERN_EMERG"irq edge 0x%08x\n", _gpio_edge);
	//那个按键产生中断
//	for (i = 0; i < 32; i++) {
//		if (!(_gpio_intp & (1 << i)))
//			continue;
//		printk(KERN_EMERG"interrupt -> GPIO-%d \n", i);
//	}
	if (_gpio_intp & (1)) { 		//GPIO0
		ev_press = 1;
		wake_up_interruptible(&my_waitq);
	}
	if (_gpio_intp & (1 << 2)) { //GPIO2
		ev_press = 1;
		wake_up_interruptible(&my_waitq);
	}
	if (_gpio_intp & (1 << 3)) { //GPIO3
		ev_press = 1;
		wake_up_interruptible(&my_waitq);
	}
	return IRQ_RETVAL(IRQ_HANDLED);
}

#define MTIOCTL_SET_DIR_IN 			0
#define MTIOCTL_SET_DIR_OUT 		1
#define MTIOCTL_SET_VALUE        	2
#define MTIOCTL_GET_VALUE			3
#define MTIOCTL_ENABLE_IRQ		4
#define MTIOCTL_SET_INTERRUPT 5
#define MTIOCTL_SET_LED_BLINK  6
#define MTIOCTL_GET_SW_POA				7  //return Port Ability Offset:mt7688_datasheet_v1_4.pdf

struct arg_type {
	unsigned int gpio;
	unsigned int value;
};

volatile unsigned char key_value[4];

//static DEFINE_MUTEX(_my_mutex);

//int _devfops_ioctl(struct inode *inode, struct file *file, unsigned int req,	unsigned long arg) {
static long _devfops_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	unsigned long tmp;

	switch (cmd) {
	case MTIOCTL_SET_DIR_IN:
		//0: GPIO input mode
		//1: GPIO output mode
		if (arg < 32) { //GPIO0 to GPIO31
			arg = 1 << arg;
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp &= ~cpu_to_le32(arg);
			*(volatile u32 *) (RALINK_REG_PIODIR) = tmp;
			printk(KERN_EMERG"IN GPIO0 to GPIO31(0x%08X)=0x%08X\n", RALINK_REG_PIODIR, tmp);
		} else if (arg < 64) { //GPIO32 to GPIO63
			arg = 1 << (arg - 32);
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
			tmp &= ~cpu_to_le32(arg);
			*(volatile u32 *) (RALINK_REG_PIO6332DIR) = tmp;
			printk(KERN_EMERG"IN GPIO32 to GPIO63(0x%08X)=0x%08X\n", RALINK_REG_PIO6332DIR, tmp);
		} else { //GPIO64 to GPIO95
			arg = 1 << (arg - 64);
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
			tmp &= ~cpu_to_le32(arg);
			*(volatile u32 *) (RALINK_REG_PIO9564DIR) = tmp;
			printk(KERN_EMERG"IN GPIO64 to GPIO95(0x%08X)=0x%08X\n", RALINK_REG_PIO9564DIR, tmp);
		}
		break;
	case MTIOCTL_SET_DIR_OUT:
		if (arg < 32) { //GPIO0 to GPIO31
			arg = 1 << arg;
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp |= cpu_to_le32(arg);
			*(volatile u32 *) (RALINK_REG_PIODIR) = tmp;
			printk(KERN_EMERG"OUT GPIO0 to GPIO31(0x%08X)=0x%08X\n", RALINK_REG_PIODIR, tmp);
		} else if (arg < 64) { //GPIO32 to GPIO63
			arg = 1 << (arg - 32);
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
			tmp |= cpu_to_le32(arg);
			*(volatile u32 *) (RALINK_REG_PIO6332DIR) = tmp;
			printk(KERN_EMERG"OUT GPIO32 to GPIO63(0x%08X)=0x%08X\n", RALINK_REG_PIO6332DIR, tmp);
		} else { //GPIO64 to GPIO95
			arg = 1 << (arg - 64);
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
			tmp |= cpu_to_le32(arg);
			*(volatile u32 *) (RALINK_REG_PIO9564DIR) = tmp;
			printk(KERN_EMERG"OUT GPIO64 to GPIO95(0x%08X)=0x%08X\n", RALINK_REG_PIO9564DIR, tmp);
		}
		break;
	case MTIOCTL_SET_VALUE: {
		struct arg_type value;
		unsigned int v = 0;
		if (copy_from_user(&value, (int __user* ) arg, sizeof(value)))
			return -ENOTTY;
		printk(KERN_EMERG"GPIO-%d, value=%d\n", value.gpio, value.value);
		if (value.gpio < sizeof(_led_blink) / sizeof(_led_blink[0])) {
			_led_blink[value.gpio].mode = 1;
			_led_blink[value.gpio].value = value.value;
			_led_blink[value.gpio].timer = 0;
			_led_blink[value.gpio].timer_tmp = 0;
		}
		set_gpio_value(value.gpio, value.value);
	}
		break;
	case MTIOCTL_GET_VALUE: {
		//RALINK_REG_PIODATA
		unsigned int v = 0; //32Bit
		unsigned int gpio = 0;

		if (copy_from_user(&gpio, (int __user* ) arg, sizeof(gpio)))
			return -ENOTTY;
		v = get_gpio_value(gpio);
		copy_to_user((int __user * )arg, &v, sizeof(v));
	}
		break;
	case MTIOCTL_ENABLE_IRQ:
		*(volatile u32 *) (RALINK_REG_INTENA) = cpu_to_le32(RALINK_INTCTL_PIO);
		break;
	case MTIOCTL_SET_INTERRUPT: {
		struct arg_type value;
		unsigned int v = 0;
		if (copy_from_user(&value, (int __user* ) arg, sizeof(value)))
			return -ENOTTY;
		if (value.gpio < 32) {
			v = 0x01 << value.gpio;

			//GPIO0 to GPIO31 rising edge interrupt enable register
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA)); //
			tmp = value.value == 0 ? (tmp & (~v)) : (tmp | v); //tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIORENA) = cpu_to_le32(tmp);

			printk(KERN_EMERG"RALINK_REG_PIORENA (0x%08X)=0x%08X\n", RALINK_REG_PIORENA, tmp);
			//GPIO0 to GPIO31 falling edge interrupt enable register
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
			tmp = value.value == 0 ? (tmp & (~v)) : (tmp | v); //tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIOFENA) = cpu_to_le32(tmp);

			printk(KERN_EMERG"RALINK_REG_PIOFENA (0x%08X)=0x%08X\n", RALINK_REG_PIOFENA, tmp);
		} else if (value.gpio < 64) {
			v = 0x01 << (value.gpio - 32);

			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332RENA));
			tmp = value.value == 0 ? (tmp & (~v)) : (tmp | v); //tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIO6332RENA) = cpu_to_le32(tmp);

			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332FENA));
			tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIO6332FENA) = cpu_to_le32(tmp);
		} else {
			v = 0x01 << (value.gpio - 64);

			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564RENA));
			tmp = value.value == 0 ? (tmp & (~v)) : (tmp | v); //tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIO9564RENA) = cpu_to_le32(tmp);

			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564FENA));
			tmp = value.value == 0 ? (tmp & (~v)) : (tmp | v); //tmp |= v;
			*(volatile u32 *) (RALINK_REG_PIO9564FENA) = cpu_to_le32(tmp);
		}
	}
		break;
	case MTIOCTL_SET_LED_BLINK: {
		struct led_blink_args value;
		if (copy_from_user(&value, (int __user* ) arg, sizeof(value)))
			return -ENOTTY;
		if (value.gpio > sizeof(_led_blink) / sizeof(_led_blink[0])) {
			printk(KERN_WARNING"gpio max is %d>%d\n", value.gpio,
					sizeof(_led_blink) / sizeof(_led_blink[0]));
			return -ENOTTY;
		}
		_led_blink[value.gpio].timer = value.timer;
		_led_blink[value.gpio].timer_tmp = 0;
		if (value.enable)
			_led_blink[value.gpio].mode = 2;
		else
			_led_blink[value.gpio].mode = 0;
		_led_blink[value.gpio].value = 0;

		printk(KERN_WARNING"BLINK, GPIO-%d, timer=%d, enable=%d\n", value.gpio, value.timer,
				value.enable);
	}
		break;
	case MTIOCTL_GET_SW_POA: {
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_ETH_SW_BASE+0x80));
		copy_to_user((u32 __user * )arg, &tmp, sizeof(u32));
	}
		break;
	}
	return 0;
}

static ssize_t _devfops_write(struct file *filp, const char __user *data, size_t len, loff_t *offp) {
	char args[50];
	unsigned int gpio = 0;
	unsigned int value = 0;
	unsigned int v = 0;

	if (len < sizeof(args)) {
		memset(args, 0, sizeof(args));
		copy_from_user(args, data, len);

		if (2 == sscanf(args, "%d %d", &gpio, &value)) {
			set_gpio_value(gpio, value);
			return len;
		}
	} else {
		len = 0;
		return 0;
	}
	//ev_press = 1;
	//wake_up_interruptible(&my_waitq);
	return len;
}

static ssize_t _devfops_read(struct file *filp, char __user *data, size_t len, loff_t *offp) {
	if (ev_press == 0) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			wait_event_interruptible(my_waitq, ev_press);
	}
	ev_press = 0;

	key_value[0] = get_gpio_value(0);
	key_value[1] = 0;
	key_value[2] = get_gpio_value(2);
	key_value[3] = get_gpio_value(3);
	copy_to_user(data, key_value, sizeof(key_value));                //返回键值
	return sizeof(key_value);
}

unsigned int _devfops_poll(struct file *filp, struct poll_table_struct *wait) {
	unsigned int mask = 0;
	poll_wait(filp, &my_waitq, wait);
	if (ev_press)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static struct irqaction _my_irqaction = {
///////////////////////////
		.name = DEVICE_NAME, ///////////////////////////////
		.flags = IRQF_SHARED, ///////////////////////////
		.handler = my_irq_handler ///////////////////////////
		};

volatile int _open_flag = 0; //只能打开一次哦

int _devfops_open(struct inode *inode, struct file *file) {
	int ret = 0;
	if (_open_flag) {
		printk(KERN_EMERG"_devfops_open bus err\n");
		return -EBUSY;
	}
	ret = setup_irq(SURFBOARDINT_GPIO, &_my_irqaction); //对应remove_irq
	if (ret) {
		printk(KERN_EMERG"setup_irq err\n");
		return ret;
	}
	_open_flag = 1;
	return ret;
}

int _devfops_release(struct inode *inode, struct file *file) {
	printk(KERN_EMERG"CGPIO _devfops_release \n");
	_open_flag = 0;
	remove_irq(SURFBOARDINT_GPIO, &_my_irqaction);
	return 0;
}

/*
 include/fs.h
 老版本中是ioctl, 3.0后版本修改unlocked_ioctl和compat_ioctl（支持64bit的driver必须要实现的ioctl,64位专有,un是通用）
 */
static struct file_operations device_fops = {
////////////////////////////////////////
		owner: THIS_MODULE, ////////////////////////////////////////
		unlocked_ioctl: _devfops_ioctl, ////////////////////////////////////////
		open: _devfops_open, ////////////////////////////////////////
		release: _devfops_release, ////////////////////////////////////////
		read: _devfops_read, ////////////////////////////////////////
		write: _devfops_write, ////////////////////////////////////////
		.poll = _devfops_poll, ///////////////////////////////////////
		};

struct miscdevice misc_dev = {
////////////////////////////////////
		.minor = MISC_DYNAMIC_MINOR, ////////////////////////////////////
		.name = DEVICE_NAME, ////////////////////////////////////
		.fops = &device_fops, ////////////////////////////////////
		};

//定时器任务
static void _my_led_timer_handler(unsigned long unused) {
	int i = 0;
	int value = 0;

	for (i = 0; i < sizeof(_led_blink) / sizeof(_led_blink[0]); i++) {
		switch (_led_blink[i].mode) //0 默认,1 设置  2 定时
		{
		case 0:
			break;
		case 1:
			set_gpio_value(i, _led_blink[i].value);
			break;
		case 2:
			if (jiffies - _led_blink[i].timer_tmp > _led_blink[i].timer) {
				value = get_gpio_value(i);
				set_gpio_value(i, value > 0 ? 0 : 1);
				///_led_blink[i].value = _led_blink[i].value>0?0:1;
				_led_blink[i].timer_tmp = jiffies;
			}
			break;
		}
	}

	init_timer(&_my_led_timer);
	_my_led_timer.function = _my_led_timer_handler;
	_my_led_timer.expires = jiffies + (HZ / 10);
	add_timer(&_my_led_timer);
}
/*
 static struct class *my_class;
 static struct device *my_device;
 */

int __init my_init(void) {
	int ret = 0;
	u32 gpiomode;

	printk(KERN_EMERG"================ mt7688 cgpio init 20180718 ================\n");

	ret = misc_register(&misc_dev); //字符杂项设备注册
	if (ret) {
		printk(KERN_EMERG"misc_register err\n");
		return ret;
	}
	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_REG_GPIOMODE));
	gpiomode &= ~0x1C;  //clear bit[2:4]UARTF_SHARE_MODE
	gpiomode |= RALINK_GPIOMODE_DFT;
	*(volatile u32 *) (RALINK_REG_GPIOMODE) = cpu_to_le32(gpiomode);
	*(volatile u32 *) (RALINK_REG_GPIOMODE2) = cpu_to_le32(0x551);

	printk(KERN_EMERG"GPIO1MODE=%08X\n", gpiomode);
	printk(KERN_EMERG"GPIO2MODE=%08X\n", *(volatile u32 *) (RALINK_REG_GPIOMODE2));

	memset(_led_blink,0,sizeof(_led_blink));

#define WLED_AN_MODE_GPIO  1
#define WLED_AN_MODE_WLED 0

#define P3_LED_AN_MODE_GPIO (0x01<<8)
#define P4_LED_AN_MODE_GPIO (0x01<<10)
#define WLED_KN_MODE_GPIO   (0x01<<16)

	//0x551
	//0101  0101 0001

	init_timer(&_my_led_timer);
_my_led_timer.function = _my_led_timer_handler;
	_my_led_timer.expires = jiffies + (HZ / 10);
	add_timer(&_my_led_timer);
	/***** 普通注册
	 ret = register_chrdev(major, DEVICE_NAME, &device_fops);
	 if (ret < 0) {
	 printk(KERN_EMERG ": unable to register character device\n");
	 return ret;
	 }
	 my_class=class_create(THIS_MODULE, DEVICE_NAME);

	 if (IS_ERR(my_class))
	 return PTR_ERR(my_class);

	 my_device=device_create(my_class, NULL, MKDEV(major, 0),NULL, DEVICE_NAME);
	 **/
	return ret;
}

void __exit my_exit(void) {
	printk(KERN_EMERG"exit\n");
	del_timer(&_my_led_timer);
	misc_deregister(&misc_dev);
	/***
	 device_unregister(my_device);
	 class_destroy(my_class);
	 unregister_chrdev(major, DEVICE_NAME);
	 ***/
}

module_init(my_init);
module_exit(my_exit);
MODULE_LICENSE("GPL");

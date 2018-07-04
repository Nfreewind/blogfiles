#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>

#define DEVICE_NAME "cjtest"

//可以参考 https://blog.csdn.net/cjsycyl/article/details/20923001

/***************************
定义一个等待队列，这个等待队列实际上是由中断驱动的，当中断发生时，会令挂接到这个等待队列的休眠进程唤醒

	poll_wait(filp, &my_waitq, wait); //第二个参数就是 wait_queue_head_t *
**********************/
static DECLARE_WAIT_QUEUE_HEAD(my_waitq);
static volatile int ev_press = 0;                //中断事件标志, 中断服务程序将它置1

static char message[100];
int major = 133;

//#define __init
//#define __exit

//int _devfops_ioctl(struct inode *inode, struct file *file, unsigned int req,	unsigned long arg) {
static long _devfops_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {

	return 0;
}

int _devfops_open(struct inode *inode, struct file *file) {
	return 0;
}

static ssize_t _devfops_write(struct file *filp, const char __user *data, size_t len, loff_t *offp) {
	if (len < sizeof(message)) {
		copy_from_user(message, data, len);
	} else {
		len=0;
	}

	ev_press = 1;
	wake_up_interruptible(&my_waitq);
	return len;
}

static ssize_t _devfops_read(struct file *filp, char __user *data, size_t len, loff_t *offp) {
	wait_event_interruptible(my_waitq, ev_press);
	ev_press = 0;
	copy_to_user(data, message, strlen(message));                //返回键值
	return strlen(message);
}

int _devfops_release(struct inode *inode, struct file *file) {
	return 0;
}

unsigned int _devfops_poll(struct file *filp, struct poll_table_struct *wait) {
	unsigned int mask = 0;
	poll_wait(filp, &my_waitq, wait);
	if (ev_press)
		mask |= POLLIN | POLLRDNORM;
	return mask;
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

struct miscdevice misc_dev = { .minor = MISC_DYNAMIC_MINOR, ////////////////////////////////////
		.name = DEVICE_NAME, ////////////////////////////////////
		.fops = &device_fops, ////////////////////////////////////
		};

/*
 static struct class *my_class;
 static struct device *my_device;
 */

int __init  my_init(void) {
	int ret=0;
	printk(KERN_EMERG"gpio_init\n");
	memset(message,0,sizeof(message));

	ret=misc_register(&misc_dev); //字符杂项设备注册
	return ret;
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

void __exit my_exit(void)
{
	printk(KERN_EMERG"gpio_exit\n");
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

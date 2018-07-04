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
#include <linux/timer.h>  /*timer*/

#define DEVICE_NAME "cjtest"

/**
 这是定时器
 当一个定时器函数即将要被运行前，内核会把相应的定时器从内核链表中删除（相当于注销）
初始化
 方法一：
DEFINE_TIMER(timer_name, function_name, expires_value, data);
该宏会静态创建一个名叫 timer_name 内核定时器，并初始化其 function, expires, name 和 base 字段。
 方法二：
struct timer_list mytimer;
setup_timer(&mytimer, (*function)(unsigned long), unsigned long data);
mytimer.expires = jiffies + 5*HZ;
 方法3：
	struct timer_list mytimer;
	init_timer(&mytimer);
	mytimer ->timer.expires = jiffies + 5*HZ;
	mytimer ->timer.data = (unsigned long) dev;
	mytimer ->timer.function = &corkscrew_timer;
 注册
add_timer(struct timer_list *timer) ;
注销
del_timer(struct timer_list *timer) 或 del_timer_sync(struct timer_list *timer)
 */

static char message[100];
static volatile int setop=0;

//定义一个定时器链表
struct timer_list timer;

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
		len = 0;
	}
	return len;
}

static ssize_t _devfops_read(struct file *filp, char __user *data, size_t len, loff_t *offp) {
	copy_to_user(data, message, strlen(message));                //返回键值
	return strlen(message);
}

int _devfops_release(struct inode *inode, struct file *file) {
	return 0;
}

unsigned int _devfops_poll(struct file *filp, struct poll_table_struct *wait) {
	unsigned int mask = 0;
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

//#define __init
//#define __exit

void _timer_fun(unsigned long v) {
	printk(KERN_EMERG"_timer_fun:%d\n", v);
	setop++;
	sprintf(setop,"%d", setop);
	mod_timer(&timer, jiffies+100); //修改
}

int __init my_init(void) {
	int ret = 0;
	printk(KERN_EMERG"gpio_init\n");
	memset(message, 0, sizeof(message));

	setup_timer(&timer, _timer_fun, 333);

	timer.expires = jiffies + 100; ///(5 * HZ); 设定超时时间 100代表1秒

	//
	add_timer(&timer);

	ret = misc_register(&misc_dev); //字符杂项设备注册
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

void __exit my_exit(void) {
	printk(KERN_EMERG"gpio_exit\n");
	del_timer(&timer);

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

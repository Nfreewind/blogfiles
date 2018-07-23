/**
 * Linux 三大设备之网络设备
 */
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

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/miscdevice.h>
#include <linux/ethtool.h>
#include <linux/if.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/if_tun.h>

struct net_device *dev_cjether;

int cjether_init(struct net_device *dev) {
	return 0;
}

void cjether_uninit(struct net_device *dev) {

}
int ei_open(struct net_device *dev) {
	return 0;
}
int ei_close(struct net_device *dev) {
	return 0;
}

netdev_tx_t ei_start_xmit_fake(struct sk_buff *skb, struct net_device *dev) {
	//kmalloc(40, GFP_KERNEL);
	//skb->data, skb->data_len;
	unsigned int i = 0;
	printk("ei_start_xmit_fake:(%d)", skb->len);
	for (i = 0; i < skb->len; i++) {
		printk("%02X ", skb->data[i]);
	}
	printk("\n");
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}
struct net_device_stats* ra_get_stats(struct net_device *dev) {
	return NULL;
}

int eth_mac_addr(struct net_device *dev, void *addr) {
	return 0;
}

int ei_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd) {
	return 0;
}

static const struct net_device_ops cc_netdev_ops = {
//////////////////////////////
		.ndo_init = cjether_init, //注册网络设备时调用，驱动程序一般不使用，虚拟网络设备可能使用
		.ndo_uninit = cjether_uninit, //网络设备卸载时调用，驱动程序一般使用
		.ndo_open = ei_open, //https://blog.csdn.net/u011955950/article/details/19041089
		.ndo_stop = ei_close, //////////////////////////////
		.ndo_start_xmit = ei_start_xmit_fake, //////////////////////////////
		//.ndo_get_stats = ra_get_stats, //////////////////////////////
		//.ndo_set_mac_address = eth_mac_addr, //////////////////////////////
		//.ndo_change_mtu = ei_change_mtu, //////////////////////////////
		//.ndo_do_ioctl = ei_ioctl, //////////////////////////////
		//.ndo_validate_addr = eth_validate_addr, //////////////////////////////
		};

void tun_setup(struct net_device *dev) {

}

int my_init(void) {
	int ret = 0;
	unsigned int queues = 30;
	struct net_device *dev = NULL;

	printk(KERN_EMERG"init\n");

	dev = alloc_netdev_mqs(0, "ccj%d", tun_setup, queues, queues);

	netdev_priv(dev);

	if (!dev) {
		printk(KERN_WARNING " " __FILE__ ": alloc_etherdev.\n");
		return -ENOMEM;
	}

	ether_setup(dev);
	eth_hw_addr_random(dev);
	dev->netdev_ops = &cc_netdev_ops;

	dev->dev_addr[0] = 0x11;
	dev->dev_addr[1] = 0x22;
	dev->dev_addr[2] = 0x33;
	dev->dev_addr[3] = 0x44;
	dev->dev_addr[4] = 0x55;
	dev->dev_addr[5] = 0x66;
#if 0
	dev_net_set(dev, net);

	dev->hw_features = NETIF_F_SG | NETIF_F_FRAGLIST;
	dev->features = dev->hw_features;
	dev->vlan_features = dev->features;

	dev->netdev_ops = &cc_netdev_ops;
	/* Ethernet TAP Device */
	ether_setup(dev);
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;

	dev->tx_queue_len = TUN_READQ_SIZE; /* We prefer our own queue length */
#endif

	printk(KERN_EMERG"start register\n");

	if (register_netdev(dev) != 0) {
		printk(KERN_WARNING " " __FILE__ ": No ethernet port found.\n");
		free_netdev(dev);
		return -ENXIO;
	}
	dev_cjether = dev;
	return ret;
}

void my_exit(void) {
	printk(KERN_EMERG"exit\n");
	unregister_netdev(dev_cjether);
	free_netdev(dev_cjether);
}

module_init(my_init);
module_exit(my_exit);
MODULE_LICENSE("GPL");

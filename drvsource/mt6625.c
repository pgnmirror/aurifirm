// mt6625.c first beta driver

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/firmware.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>

#define DRV_NAME "mt6625"

struct mt6625_net {
    struct device *dev;
    struct net_device *ndev;
};

struct mt6625_priv {
    struct device *dev;
    struct regulator *v18_wbt;
    struct regulator *v28_fm;
    struct regulator *v33_wbt;
    struct gpio_desc *gpiod_hrst;
    struct mt6625_net *mnet;
};

static int mt6625_power_on(struct mt6625_priv *mp)
{
    int ret;

    dev_info(mp->dev, "%s: enabling regulators\n", DRV_NAME);

    if (mp->v18_wbt) {
        ret = regulator_enable(mp->v18_wbt);
        if (ret) { dev_err(mp->dev, "%s: v18_wbt enable failed: %d\n", DRV_NAME, ret); return ret; }
    }
    usleep_range(200, 500);

    if (mp->v33_wbt) {
        ret = regulator_enable(mp->v33_wbt);
        if (ret) { dev_err(mp->dev, "%s: v33_wbt enable failed: %d\n", DRV_NAME, ret); return ret; }
    }
    usleep_range(200, 500);

    if (mp->v28_fm) {
        ret = regulator_enable(mp->v28_fm);
        if (ret) { dev_err(mp->dev, "%s: v28_fm enable failed: %d\n", DRV_NAME, ret); return ret; }
    }

    msleep(5);

    if (mp->gpiod_hrst) {
        /* HRST_B active low — on boot keep low then release */
        gpiod_set_value_cansleep(mp->gpiod_hrst, 0);
        msleep(2);
        gpiod_set_value_cansleep(mp->gpiod_hrst, 1);
        msleep(5);
    }

    dev_info(mp->dev, "%s: regulators enabled + reset released\n", DRV_NAME);
    return 0;
}

static void mt6625_power_off(struct mt6625_priv *mp)
{
    dev_info(mp->dev, "%s: disabling regulators\n", DRV_NAME);
    if (mp->gpiod_hrst)
        gpiod_set_value_cansleep(mp->gpiod_hrst, 0);

    if (mp->v28_fm) regulator_disable(mp->v28_fm);
    if (mp->v33_wbt) regulator_disable(mp->v33_wbt);
    if (mp->v18_wbt) regulator_disable(mp->v18_wbt);

    msleep(2);
}

/* netdev private */
struct mt6625_net_priv {
    struct mt6625_priv *mp;
};

static int mt6625_ndev_open(struct net_device *dev)
{
    netif_start_queue(dev);
    pr_info("%s: netdev open\n", DRV_NAME);
    return 0;
}

static int mt6625_ndev_stop(struct net_device *dev)
{
    netif_stop_queue(dev);
    pr_info("%s: netdev stop\n", DRV_NAME);
    return 0;
}

static netdev_tx_t mt6625_ndev_xmit(struct sk_buff *skb, struct net_device *dev)
{

    /* debug */
    pr_info("%s: xmit len=%u\n", DRV_NAME, skb->len);

    skb->dev = dev;
    skb->protocol = eth_type_trans(skb, dev);
    netif_rx(skb);
    /* update stats */
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;

    return NETDEV_TX_OK;
}

static const struct net_device_ops mt6625_netdev_ops = {
    .ndo_open = mt6625_ndev_open,
    .ndo_stop = mt6625_ndev_stop,
    .ndo_start_xmit = mt6625_ndev_xmit,
};

static int mt6625_request_fw(struct mt6625_priv *mp)
{
    const struct firmware *fw;
    int ret;

    dev_info(mp->dev, "%s: requesting firmware 'mt6625/mt6625_fw.bin'\n", DRV_NAME);
    ret = request_firmware(&fw, "mt6625/mt6625_fw.bin", mp->dev);
    if (ret) {
        dev_warn(mp->dev, "%s: firmware not found (%d). Put mt6625/mt6625_fw.bin into /lib/firmware\n", DRV_NAME, ret);
        return ret;
    }

    dev_info(mp->dev, "%s: firmware loaded size=%zu\n", DRV_NAME, fw->size);
    /* TODO: transfer or parse fw*/
    release_firmware(fw);
    return 0;
}

static int mt6625_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct mt6625_priv *mp;
    struct net_device *ndev;
    struct mt6625_net_priv *npriv;
    int ret;

    mp = devm_kzalloc(dev, sizeof(*mp), GFP_KERNEL);
    if (!mp) return -ENOMEM;
    mp->dev = dev;

    mp->v18_wbt = devm_regulator_get_optional(dev, "v18-wbt");
    mp->v28_fm  = devm_regulator_get_optional(dev, "v28-fm");
    mp->v33_wbt = devm_regulator_get_optional(dev, "v33-wbt");

    mp->gpiod_hrst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(mp->gpiod_hrst)) {
        dev_err(dev, "%s: failed to get reset gpio\n", DRV_NAME);
        return PTR_ERR(mp->gpiod_hrst);
    }

    platform_set_drvdata(pdev, mp);

    dev_info(dev, "%s: probe start\n", DRV_NAME);

    ret = mt6625_power_on(mp);
    if (ret) {
        dev_err(dev, "%s: power_on failed %d\n", DRV_NAME, ret);
        return ret;
    }

    mt6625_request_fw(mp);

    ndev = alloc_netdev(sizeof(struct mt6625_net_priv), "mtw%d", NET_NAME_ENUM, ether_setup);
    if (!ndev) {
        ret = -ENOMEM;
        goto err_power;
    }

    npriv = netdev_priv(ndev);
    npriv->mp = mp;

    ndev->netdev_ops = &mt6625_netdev_ops;

    if (register_netdev(ndev)) {
        dev_err(dev, "%s: register_netdev failed\n", DRV_NAME);
        ret = -ENODEV;
        goto err_free;
    }

    /* save handles */
    mp->mnet = devm_kzalloc(dev, sizeof(*mp->mnet), GFP_KERNEL);
    if (!mp->mnet) { ret = -ENOMEM; goto err_unregister; }
    mp->mnet->dev = dev;
    mp->mnet->ndev = ndev;
    dev_info(dev, "%s: netdev %s registered\n", DRV_NAME, ndev->name);

    return 0;

err_unregister:
    unregister_netdev(ndev);
err_free:
    free_netdev(ndev);
err_power:
    mt6625_power_off(mp);
    return ret;
}

static void mt6625_remove(struct platform_device *pdev)
{
    struct mt6625_priv *mp = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "%s: remove\n", DRV_NAME);

    if (mp && mp->mnet && mp->mnet->ndev) {
        unregister_netdev(mp->mnet->ndev);
        free_netdev(mp->mnet->ndev);
    }

    mt6625_power_off(mp);
}

static const struct of_device_id mt6625_of_match[] = {
    { .compatible = "mediatek,mt6625l", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6625_of_match);

static struct platform_driver mt6625_platform_driver = {
    .probe  = mt6625_probe,
    .remove = mt6625_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = mt6625_of_match,
    },
};

module_platform_driver(mt6625_platform_driver);

MODULE_AUTHOR("LICGX Team, ne5link");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6625L driver");

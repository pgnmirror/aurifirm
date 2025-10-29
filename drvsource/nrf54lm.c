/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Nordic Semiconductor nRF54LM20A Radio Driver
 *
 * Driver: nrf54_radio.c
 * Author: LICGX Team, ne5link
 * Description: Nordic Semiconductor nRF54LM20A Radio Driver
 *
 * Datasheet used:
 * https://cdn.everythingrf.com/live/nRF54LM20A_Preliminary_Datasheet_v0_7_638948117935012299.pdf
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

MODULE_AUTHOR("LICGX Team, ne5link");
MODULE_DESCRIPTION("Nordic Semiconductor nRF54LM20A Radio Driver");
MODULE_LICENSE("GPL");

#define NRF_RADIO_DRV_NAME "nrf54_radio"

#define TASKS_TXEN            0x000
#define TASKS_RXEN            0x004
#define TASKS_START           0x008
#define TASKS_STOP            0x00C
#define TASKS_DISABLE         0x010
#define TASKS_RSSISTART       0x014
#define TASKS_BCSTART         0x018
#define TASKS_BCSTOP          0x01C
#define TASKS_EDSTART         0x020
#define TASKS_EDSTOP          0x024
#define TASKS_CCASTART        0x028
#define TASKS_CCASTOP         0x02C
#define TASKS_AUXDATADMASTART 0x038
#define TASKS_AUXDATADMASTOP  0x03C
#define TASKS_PLLEN           0x06C
#define TASKS_SOFTRESET       0x0A4

/* EVENTS */
#define EVENTS_READY          0x200
#define EVENTS_TXREADY        0x204
#define EVENTS_RXREADY        0x208
#define EVENTS_ADDRESS        0x20C
#define EVENTS_FRAMESTART     0x210
#define EVENTS_PAYLOAD        0x214
#define EVENTS_END            0x218
#define EVENTS_PHYEND         0x21C
#define EVENTS_DISABLED       0x220
#define EVENTS_DEVMATCH       0x224
#define EVENTS_DEVMISS        0x228
#define EVENTS_CRCOK          0x22C
#define EVENTS_CRCERROR       0x230
#define EVENTS_BCMATCH        0x238
#define EVENTS_EDEND          0x23C
#define EVENTS_EDSTOPPED      0x240
#define EVENTS_CCAIDLE        0x244
#define EVENTS_CCABUSY        0x248
#define EVENTS_CCASTOPPED     0x24C
#define EVENTS_SYNC           0x258
#define EVENTS_PLLREADY       0x2B0
#define EVENTS_RXADDRESS      0x2BC
#define EVENTS_AUXDATADMAEND  0x2C0
#define EVENTS_CSTONESEND     0x2C8

/* PUBLISH offsets start at 0x300 .. */
#define PUBLISH_BASE          0x300
#define PUBLISH_READY         (0x300 + 0x00)
#define PUBLISH_TXREADY       (0x300 + 0x04)
#define PUBLISH_RXREADY       (0x300 + 0x08)
#define PUBLISH_ADDRESS       (0x300 + 0x0C)
#define PUBLISH_FRAMESTART    (0x300 + 0x10)
#define PUBLISH_PAYLOAD       (0x300 + 0x14)
#define PUBLISH_END           (0x300 + 0x18)
#define PUBLISH_PHYEND        (0x300 + 0x1C)
#define PUBLISH_PLLREADY      (0x3B0)
#define PUBLISH_RXADDRESS     (0x3BC)
#define PUBLISH_AUXDATADMAEND (0x3C0)

/* SHORTS */
#define SHORTS                0x400

/* INTENSET / INTENCLR */
#define INTENSET00            0x488
#define INTENCLR00            0x490
#define INTENSET10            0x4A8
#define INTENCLR10            0x4B0

/* RADIO configuration */
#define MODE_REG              0x500
#define FREQUENCY_REG         0x708
#define TXPOWER_REG           0x710
#define TIMING_REG            0x704

/* DFE / Direction finding */
#define CLEARPATTERN_REG      0xD2C
#define PSEL_DFEGPIO(n)       (0xD30 + ((n) * 0x4))
#define DFEPACKET_PTR         0xD50
#define DFEPACKET_MAXCNT      0xD54

/* PACKET pointer (for regular RX/TX) */
#define PACKETPTR_REG         0xED0

/* Useful macros for bit ops */
#define BITFIELD_SET(val, mask, shift) (((val) << (shift)) & (mask))

#define DEFAULT_FREQUENCY     2   /* FREQUENCY field -> 2400 + 2 = 2402 MHz */
#define DEFAULT_TXPOWER_DBM   0

/* Driver runtime context */
struct nrf54_radio {
    void __iomem *reg_base;
    int irq;
    struct device *dev;

    /* workqueue for deferred event processing */
    struct workqueue_struct *wq;
    struct work_struct irq_work;

    /* protect state changes */
    struct mutex lock;
    bool radio_enabled;
};

static void nrf54_handle_events_work(struct work_struct *work);

/* helpers: read/write registers */
static inline u32 reg_read(struct nrf54_radio *r, u32 off)
{
    return ioread32(r->reg_base + off);
}

static inline void reg_write(struct nrf54_radio *r, u32 off, u32 v)
{
    iowrite32(v, r->reg_base + off);
    /* read-back to ensure posted write completes on some buses */
    (void)ioread32(r->reg_base + off);
}

/* IRQ handler: quick ack + schedule work */
static irqreturn_t nrf54_irq_handler(int irq, void *dev_id)
{
    struct nrf54_radio *r = dev_id;
    u32 ev_ready = reg_read(r, EVENTS_READY);
    u32 ev_end = reg_read(r, EVENTS_END);
    u32 ev_crcok = reg_read(r, EVENTS_CRCOK);
    u32 ev_crcerr = reg_read(r, EVENTS_CRCERROR);
    u32 ev_rxready = reg_read(r, EVENTS_RXREADY);
    u32 ev_txready = reg_read(r, EVENTS_TXREADY);

    /* Clear the events we've observed (write 0 to event reg) */
    if (ev_ready)
        reg_write(r, EVENTS_READY, 0);
    if (ev_end)
        reg_write(r, EVENTS_END, 0);
    if (ev_crcok)
        reg_write(r, EVENTS_CRCOK, 0);
    if (ev_crcerr)
        reg_write(r, EVENTS_CRCERROR, 0);
    if (ev_rxready)
        reg_write(r, EVENTS_RXREADY, 0);
    if (ev_txready)
        reg_write(r, EVENTS_TXREADY, 0);

    /* store any necessary state in device context if required (not done here) */
    /* schedule deferred worker to do heavier processing outside IRQ */
    queue_work(r->wq, &r->irq_work);

    return IRQ_HANDLED;
}

/* Deferred work: read event flags again and process them safely */
static void nrf54_handle_events_work(struct work_struct *work)
{
    struct nrf54_radio *r = container_of(work, struct nrf54_radio, irq_work);
    u32 state;

    mutex_lock(&r->lock);

    /* Example handling: check for END and CRCOK/CRCERROR and log
     * Real driver would pull packet data from RAM via PACKETPTR and hand off
     * to upper stack (net/ieee802154 or custom).
     */
    state = reg_read(r, EVENTS_END);
    if (state) {
        dev_info(r->dev, "EVENTS_END was signalled (deferred). Clearing.\n");
        reg_write(r, EVENTS_END, 0);
    }

    if (reg_read(r, EVENTS_CRCOK)) {
        dev_info(r->dev, "Packet CRC OK\n");
        reg_write(r, EVENTS_CRCOK, 0);
    }
    if (reg_read(r, EVENTS_CRCERROR)) {
        dev_warn(r->dev, "Packet CRC ERROR\n");
        reg_write(r, EVENTS_CRCERROR, 0);
    }

    /* Example: check ready -> maybe start radio if needed */
    if (reg_read(r, EVENTS_READY)) {
        dev_info(r->dev, "Radio reports READY (deferred). Clearing and enabling start.\n");
        reg_write(r, EVENTS_READY, 0);
        /* Optionally auto-start RX (if shorts configured, this may be redundant) */
        reg_write(r, TASKS_RXEN, 1);
        reg_write(r, TASKS_START, 1);
    }

    mutex_unlock(&r->lock);
}

/* Utility: enable a set of interrupts using INTENSET semantics (W1S) */
static void nrf54_enable_irqs(struct nrf54_radio *r)
{
    u32 mask = 0;

    /* enable key radio events:
     * bit positions correspond to INTENSET mapping in datasheet excerpt.
     * We must use the correct bit positions (datasheet: A=READY, B=TXREADY, C=RXREADY, ...).
     *
     * The datasheet lists individual bits; here we construct mask by bit index.
     * Using names for readability:
     */
    /* bit A -> READY (bit 0) */
    mask |= (1U << 0);   /* READY */
    mask |= (1U << 1);   /* TXREADY */
    mask |= (1U << 2);   /* RXREADY */
    mask |= (1U << 6);   /* END (example: bit G in the excerpt mapped to 6) */
    mask |= (1U << 11);  /* CRCOK (bit L) */
    mask |= (1U << 12);  /* CRCERROR (bit M) */
    mask |= (1U << 21);  /* SYNC (bit V) */

    /* Write mask to INTENSET00 (W1S semantics) */
    reg_write(r, INTENSET00, mask);
}

/* Utility: disable the IRQs we enabled earlier */
static void nrf54_disable_irqs(struct nrf54_radio *r)
{
    u32 mask = 0;
    mask |= (1U << 0);   /* READY */
    mask |= (1U << 1);   /* TXREADY */
    mask |= (1U << 2);   /* RXREADY */
    mask |= (1U << 6);   /* END */
    mask |= (1U << 11);  /* CRCOK */
    mask |= (1U << 12);  /* CRCERROR */
    mask |= (1U << 21);  /* SYNC */

    reg_write(r, INTENCLR00, mask); /* clear via W1C semantics (register name INTENCLR) */
}

/* Setup basic radio defaults: clearing patterns, attach DFE pins if needed */
static void nrf54_radio_config_defaults(struct nrf54_radio *r)
{
    /* Clear antenna gpio pattern array before configuring DFE pins */
    reg_write(r, CLEARPATTERN_REG, 1);

    {
        u32 val;
        int i;
        for (i = 0; i < 3; i++) {
            /* build PIN and PORT in register value; datasheet gives field layout:
             * assume PIN in bits [7:0] and PORT in [23:16] as typical Nordic style.
             * Confirm mapping in full datasheet for exact bit positions.
             */
            u32 pin = i;        /* example pin number */
            u32 port = 1;       /* example port number */
            /* Compose value: PORT<<16 | PIN (example composition) */
            val = (port << 16) | (pin & 0xFF);
            reg_write(r, PSEL_DFEGPIO(i), val);
        }
    }

    /* Set default frequency and TX power */
    reg_write(r, FREQUENCY_REG, DEFAULT_FREQUENCY);
    reg_write(r, TXPOWER_REG, DEFAULT_TXPOWER_DBM);

    /* Configure timing for fast ramp-up (example: set RU bit to 1) */
    reg_write(r, TIMING_REG, 1);
}

/* Bring radio up: PLL enable, wait for PLLREADY, then enable RX */
static int nrf54_radio_power_on(struct nrf54_radio *r)
{
    int timeout = 1000; /* milliseconds */
    unsigned long start = jiffies;
    u32 evt;

    /* Enable PLL (TASKS_PLLEN) */
    reg_write(r, TASKS_PLLEN, 1);

    /* Wait for PLLREADY event (EVENTS_PLLREADY) */
    while (time_before(jiffies, start + msecs_to_jiffies(timeout))) {
        evt = reg_read(r, EVENTS_PLLREADY);
        if (evt) {
            /* clear it */
            reg_write(r, EVENTS_PLLREADY, 0);
            dev_info(r->dev, "PLL ready\n");
            /* Optionally subscribe/publish PLLREADY to DPPI here */
            return 0;
        }
        msleep(1);
    }

    dev_err(r->dev, "PLL ready timeout\n");
    return -ETIMEDOUT;
}

/* Start RX path with minimal config (packet ptr etc must be setup by caller) */
static int nrf54_radio_start_rx(struct nrf54_radio *r)
{
    mutex_lock(&r->lock);

    if (!r->radio_enabled) {
        /* Enable RADIO in RX mode and start */
        reg_write(r, TASKS_RXEN, 1);
        reg_write(r, TASKS_START, 1);
        r->radio_enabled = true;
        dev_info(r->dev, "Radio RX started\n");
    }

    mutex_unlock(&r->lock);
    return 0;
}

/* Stop radio */
static void nrf54_radio_stop(struct nrf54_radio *r)
{
    mutex_lock(&r->lock);
    if (r->radio_enabled) {
        reg_write(r, TASKS_STOP, 1);
        reg_write(r, TASKS_DISABLE, 1); /* disable radio */
        r->radio_enabled = false;
        dev_info(r->dev, "Radio stopped/disabled\n");
    }
    mutex_unlock(&r->lock);
}

/* platform probe/remove */
static int nrf54_probe(struct platform_device *pdev)
{
    struct nrf54_radio *r;
    struct resource *res;
    int irq;
    int ret;

    r = devm_kzalloc(&pdev->dev, sizeof(*r), GFP_KERNEL);
    if (!r)
        return -ENOMEM;

    /* map mmio */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "No MMIO resource\n");
        return -ENODEV;
    }
    r->reg_base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(r->reg_base))
        return PTR_ERR(r->reg_base);

    r->dev = &pdev->dev;
    mutex_init(&r->lock);
    INIT_WORK(&r->irq_work, nrf54_handle_events_work);
    r->wq = create_singlethread_workqueue("nrf54_wq");
    if (!r->wq) {
        dev_err(&pdev->dev, "Failed to create workqueue\n");
        return -ENOMEM;
    }

    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        dev_err(&pdev->dev, "No IRQ resource\n");
        ret = -ENODEV;
        goto err_wq;
    }
    r->irq = irq;

    /* Request IRQ - edge/level and flags are platform/DT dependent */
    ret = devm_request_irq(&pdev->dev, r->irq, nrf54_irq_handler, 0,
                           dev_name(&pdev->dev), r);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n", r->irq, ret);
        goto err_wq;
    }

    /* Basic radio config (pins, freq, txpower). Caller must set PACKETPTR and DFEPACKET if used. */
    nrf54_radio_config_defaults(r);

    /* Turn on PLL and wait for readiness */
    ret = nrf54_radio_power_on(r);
    if (ret) {
        dev_err(&pdev->dev, "Radio PLL failed to come up\n");
        goto err_wq;
    }

    /* Enable interrupts for relevant EVENTS */
    nrf54_enable_irqs(r);

    /* Optionally set shorts to automate behavior (example: READY -> START) */
    /* Enable READY->START and RXREADY->START so radio auto-starts after ready */
    reg_write(r, SHORTS, (1U << 0) /* READY_START */ | (1U << 14) /* RXREADY_START */);

    /* publish example: disable by default (writes zeros). To enable, write CHIDX|EN */
    reg_write(r, PUBLISH_RXADDRESS, 0); /* keep publishing off until configured */

    nrf54_radio_start_rx(r);

    platform_set_drvdata(pdev, r);

    dev_info(&pdev->dev, "nRF54 radio probe OK (MMIO at %p, IRQ %d)\n", r->reg_base, r->irq);
    return 0;

err_wq:
    if (r->wq)
        destroy_workqueue(r->wq);
    return ret;
}

/* Device tree match */
static const struct of_device_id nrf54_of_match[] = {
    { .compatible = "nordic,nrf54-radio", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nrf54_of_match);

/* Platform driver remove */
static void nrf54_remove(struct platform_device *pdev)
{
    struct nrf54_radio *r = platform_get_drvdata(pdev);

    if (!r)
        return;

    /* Disable interrupts */
    nrf54_disable_irqs(r);

    /* Stop radio cleanly */
    nrf54_radio_stop(r);

    /* flush and destroy workqueue */
    if (r->wq) {
        flush_workqueue(r->wq);
        destroy_workqueue(r->wq);
    }

    dev_info(&pdev->dev, "nRF54 radio removed\n");
}

/* Platform driver */
static struct platform_driver nrf54_driver = {
    .probe  = nrf54_probe,
    .remove = (void (*)(struct platform_device *))nrf54_remove, /* каст для правильного типа */
    .driver = {
        .name = NRF_RADIO_DRV_NAME,
        .of_match_table = nrf54_of_match,
        .owner = THIS_MODULE,
    },
};


module_platform_driver(nrf54_driver);

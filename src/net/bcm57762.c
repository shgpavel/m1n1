/* SPDX-License-Identifier: MIT */

#include "bcm57762.h"
#include "string.h"
#include "utils.h"

#define PCI_VENDOR_DEVICE             0x00
#define PCI_COMMAND                   0x04
#define PCI_COMMAND_MEMORY_SPACE      BIT(1)
#define PCI_COMMAND_BUS_MASTER        BIT(2)
#define PCI_COMMAND_INTERRUPT_DISABLE BIT(10)
#define PCI_CLASS_REVISION            0x08
#define PCI_PMCSR                     0x4c
#define PCI_PMCSR_POWER_STATE         GENMASK(1, 0)
#define PCI_MISC_HOST_CTRL            0x68
#define MHC_MASK_INTERRUPT            BIT(1)
#define MHC_ENABLE_ENDIAN_WORD_SWAP   BIT(3)
#define MHC_ENABLE_PCI_STATE_RW       BIT(4)
#define MHC_ENABLE_INDIRECT_ACCESS    BIT(7)
#define PCI_DMA_RW_CTRL               0x6c
#define DMA_RW_CTRL_DMA_WRITE_WM      GENMASK(21, 19)
#define PCI_MEMORY_BASE               0x7c
#define PCI_DEVICE_STATUS_CTRL        0xb4
#define DSC_ENABLE_RELAXED_ORDERING   BIT(4)
#define DSC_MAX_PAYLOAD_SIZE          GENMASK(7, 5)
#define DSC_ENABLE_NO_SNOOP           BIT(11)
#define DSC_ERRORS_DETECTED           GENMASK(19, 16)

#define MHC_INIT                                                                                   \
    (MHC_MASK_INTERRUPT | MHC_ENABLE_ENDIAN_WORD_SWAP | MHC_ENABLE_PCI_STATE_RW |                  \
     MHC_ENABLE_INDIRECT_ACCESS)

#define MBOX_RCV_BD_STD_PROD_IDX   0x26c
#define MBOX_RCV_BD_RR0_CONS_IDX   0x284
#define MBOX_SEND_BD_HOST_PROD_IDX 0x304

#define MODE_ENABLE      BIT(1)
#define MODE_ATTN_ENABLE BIT(2)

#define EMAC_MODE                        0x400
#define EMAC_MODE_HALF_DUPLEX            BIT(1)
#define EMAC_MODE_PORT_MODE              GENMASK(3, 2)
#define EMAC_MODE_PORT_MODE_MII          (1 << 2)
#define EMAC_MODE_PORT_MODE_GMII         (2 << 2)
#define EMAC_MODE_ENABLE_RX_STATS        BIT(11)
#define EMAC_MODE_CLEAR_RX_STATS         BIT(12)
#define EMAC_MODE_ENABLE_TX_STATS        BIT(14)
#define EMAC_MODE_CLEAR_TX_STATS         BIT(15)
#define EMAC_MODE_ENABLE_TDE             BIT(21)
#define EMAC_MODE_ENABLE_RDE             BIT(22)
#define EMAC_MODE_ENABLE_FHDE            BIT(23)
#define EMAC_STATUS                      0x404
#define LED_CONTROL                      0x40c
#define LED_CONTROL_INIT                 0x800
#define EMAC_MAC_ADDR_HIGH(n)            (0x410 + 8 * (n))
#define EMAC_MAC_ADDR_LOW(n)             (0x414 + 8 * (n))
#define TX_RANDOM_BACKOFF                0x438
#define TX_RANDOM_BACKOFF_SEED           GENMASK(9, 0)
#define RX_MTU_SIZE                      0x43c
#define MII_COMMUNICATION                0x44c
#define MII_COMM_DATA                    GENMASK(15, 0)
#define MII_COMM_REG_ADDR(x)             ((x) << 16)
#define MII_COMM_PHY_ADDR(x)             ((x) << 21)
#define MII_COMM_CMD_WRITE               (1 << 26)
#define MII_COMM_CMD_READ                (2 << 26)
#define MII_COMM_READ_FAILED             BIT(28)
#define MII_COMM_START_BUSY              BIT(29)
#define MII_STATUS                       0x450
#define MII_STATUS_LINK_STATUS           BIT(0)
#define MII_MODE                         0x454
#define MII_MODE_PORT_POLLING            BIT(4)
#define TX_MAC_MODE                      0x45c
#define TX_MAC_MODE_ENABLE               BIT(1)
#define TX_MAC_MODE_BAD_TXMBUF_FIX       BIT(8)
#define TX_MAC_STATUS                    0x460
#define TX_MAC_LENGTHS                   0x464
#define TX_MAC_LENGTHS_INIT              0x2620
#define RX_MAC_MODE                      0x468
#define RX_MAC_MODE_ENABLE               BIT(1)
#define RX_MAC_MODE_PROMISCUOUS          BIT(8)
#define RX_MAC_STATUS                    0x46c
#define MAC_HASH(n)                      (0x470 + 4 * (n))
#define RX_RULES_CONFIG                  0x500
#define RX_RULES_CONFIG_DEFAULT_CLASS(x) ((x) << 3)
#define LOW_WM_MAX_RX_FRAMES             0x504
#define LOW_WM_MAX_RX_FRAMES_COUNT       GENMASK(15, 0)
#define SDI_MODE                         0xc00
#define SDI_STATUS                       0xc04
#define SDI_STATS_CTRL                   0xc08
#define SDI_STATS_CTRL_ENABLE            BIT(0)
#define SDI_STATS_CTRL_FASTER_UPDATE     BIT(1)
#define SDI_STATS_MASK                   0xc0c
#define SDC_MODE                         0x1000
#define SBDS_MODE                        0x1400
#define SBDS_STATUS                      0x1404
#define SBDI_MODE                        0x1800
#define SBDI_STATUS                      0x1804
#define SBDC_MODE                        0x1c00
#define RLP_MODE                         0x2000
#define RLP_STATUS                       0x2004
#define RLP_CONFIG                       0x2010
#define RLP_CONFIG_INIT                  0x181
#define RLP_STATS_CTRL                   0x2014
#define RLP_STATS_CTRL_ENABLE            BIT(0)
#define RLP_STATS_ENABLE_MASK            0x2018
#define RDI_MODE                         0x2400
#define RDI_MODE_ILLEGAL_RR_SIZE         BIT(4)
#define RDI_STATUS                       0x2404
#define JUMBO_RCB_MAXLEN_FLAGS           0x2448
#define STD_RCB_HOST_ADDR_HI             0x2450
#define STD_RCB_HOST_ADDR_LO             0x2454
#define STD_RCB_MAXLEN_FLAGS             0x2458
#define STD_RCB_NIC_ADDR                 0x245c
#define RDC_MODE                         0x2800
#define RBDI_MODE                        0x2c00
#define RBDI_MODE_BDS_ON_DISABLED        BIT(2)
#define RBDI_STD_REPLENISH_THRESH        0x2c18
#define STD_RING_REPLENISH_WM            0x2d00
#define RBDC_MODE                        0x3000
#define RBDC_STATUS                      0x3004
#define NIC_STD_RCV_BD_PROD_IDX          0x300c
#define CHIP_ID                          0x3658
#define EEE_MODE                         0x36b0
#define EEE_MODE_USER_LPI_ENABLE         BIT(7)
#define EEE_MODE_EEE_ENABLE              BIT(20)
#define HC_MODE                          0x3c00
#define HC_MODE_COALESCE_NOW             BIT(3)
#define HC_MODE_STATUS_BLOCK_32          (2 << 7)
#define HC_STATUS                        0x3c04
#define HC_RX_COAL_TICKS                 0x3c08
#define HC_TX_COAL_TICKS                 0x3c0c
#define HC_RX_MAX_COAL_BDS               0x3c10
#define HC_TX_MAX_COAL_BDS               0x3c14
#define HC_RX_MAX_COAL_BDS_INT           0x3c20
#define HC_TX_MAX_COAL_BDS_INT           0x3c24
#define HC_STATUS_BLOCK_ADDR_HI          0x3c38
#define HC_STATUS_BLOCK_ADDR_LO          0x3c3c
#define FLOW_ATTENTION                   0x3c48
#define NIC_DIAG_RR0_PROD_IDX            0x3c80
#define NIC_DIAG_SEND_BD_CONS_IDX        0x3cc0
#define MA_MODE                          0x4000
#define MA_STATUS                        0x4004
#define BM_MODE                          0x4400
#define BM_STATUS                        0x4404
#define BM_DMA_MBUF_LOW_WM               0x4414
#define BM_MBUF_HIGH_WM                  0x4418
#define RDMA_MODE                        0x4800
#define RDMA_STATUS                      0x4804
#define WDMA_MODE                        0x4c00
#define WDMA_MODE_STATUS_TAG_FIX         BIT(29)
#define WDMA_STATUS                      0x4c04
#define DMA_MODE_ATTN_ALL                GENMASK(9, 2)
#define MSI_MODE                         0x6000
#define MSI_MODE_SINGLE_SHOT_DISABLE     BIT(5)
#define MSI_STATUS                       0x6004
#define MODE_CONTROL                     0x6800
#define MODE_CONTROL_BYTE_SWAP_BD        BIT(1)
#define MODE_CONTROL_WORD_SWAP_BD        BIT(2)
#define MODE_CONTROL_BYTE_SWAP_DATA      BIT(4)
#define MODE_CONTROL_WORD_SWAP_DATA      BIT(5)
#define MODE_CONTROL_HOST_STACK_UP       BIT(16)
#define MODE_CONTROL_HOST_SEND_BDS       BIT(17)
#define MISC_CONFIG                      0x6804
#define MISC_CONFIG_GRC_RESET            BIT(0)
#define MISC_CONFIG_GPHY_IDDQ            BIT(21)
#define MISC_CONFIG_BIAS_IDDQ            BIT(22)
#define MISC_CONFIG_RAM_POWER_DOWN       BIT(24)
#define MISC_CONFIG_GPHY_PD_OVERRIDE     BIT(26)
#define MISC_CONFIG_NO_GRC_RESET_PCIE    BIT(29)
#define FAST_BOOT_PC                     0x6894
#define SW_ARBITRATION                   0x7020
#define SW_ARB_REQ_SET1                  BIT(1)
#define SW_ARB_REQ_CLR1                  BIT(5)
#define SW_ARB_ARB_WON1                  BIT(9)
#define MEMORY_WINDOW                    0x8000

#define MODE_CONTROL_SWAP                                                                          \
    (MODE_CONTROL_WORD_SWAP_BD | MODE_CONTROL_BYTE_SWAP_DATA | MODE_CONTROL_WORD_SWAP_DATA)

#define NIC_SEND_RCB           0x100
#define NIC_RETURN_RCB         0x200
#define NIC_RCB_SIZE           0x10
#define NIC_FW_MAILBOX         0xb50
#define NIC_SEND_BD_CACHE      0x4000
#define NIC_STD_BD_CACHE       0x6000
#define RCB_HOST_ADDR_HI       0x0
#define RCB_HOST_ADDR_LO       0x4
#define RCB_MAXLEN_FLAGS       0x8
#define RCB_NIC_ADDR           0xc
#define RCB_MAX_LEN(n)         ((n) << 16)
#define RCB_FLAG_RING_DISABLED BIT(1)
#define T3_MAGIC_NUMBER        0x4b657654u

#define PHY_ADDR                   1
#define PHY_MII_CONTROL            0x00
#define PHY_MII_CONTROL_RESTART_AN BIT(9)
#define PHY_MII_CONTROL_AN_ENABLE  BIT(12)
#define PHY_MII_CONTROL_RESET      BIT(15)
#define PHY_MII_STATUS             0x01
#define PHY_ID_MSB                 0x02
#define PHY_ID_LSB                 0x03
#define PHY_AN_ADV                 0x04
#define PHY_AN_ADV_802_3           0x0001
#define PHY_AN_ADV_10_HALF         BIT(5)
#define PHY_AN_ADV_10_FULL         BIT(6)
#define PHY_AN_ADV_100_HALF        BIT(7)
#define PHY_AN_ADV_100_FULL        BIT(8)
#define PHY_AN_LINK_PARTNER        0x05
#define PHY_1000BASE_T_CONTROL     0x09
#define PHY_1000BASE_T_ADV_HALF    BIT(8)
#define PHY_1000BASE_T_ADV_FULL    BIT(9)
#define PHY_1000BASE_T_STATUS      0x0a
#define PHY_MMD_CONTROL            0x0d
#define PHY_MMD_CONTROL_DATA       0x4000
#define PHY_MMD_DATA               0x0e
#define PHY_DSP_RW_PORT            0x15
#define PHY_DSP_ADDRESS            0x17
#define PHY_AUX_CONTROL            0x18
#define PHY_AUX_CTRL_TX_6DB_CODING BIT(10)
#define PHY_AUX_CTRL_SM_DSP_CLOCK  BIT(11)
#define PHY_AUX_STATUS             0x19
#define PHY_AUX_STATUS_LINK        BIT(2)
#define PHY_AUX_STATUS_HCD         GENMASK(10, 8)
#define PHY_DSP_EEE_ADDR           0x4022
#define PHY_DSP_EEE_VAL            0x017b
#define MMD_AN                     7
#define MMD_AN_EEE_ADV             0x3c

#define SB_STATUS              0
#define SB_STATUS_UPDATED      BIT(0)
#define SB_STATUS_UPDATED_BSWP BIT(24)
#define SB_TAG                 1
#define SB_IDX                 4
#define SB_IDX_RR0_PROD        GENMASK(15, 0)
#define SB_IDX_SEND_CONS       GENMASK(31, 16)
#define SB_WORDS               8

#define RXBD_ADDR_HI          0
#define RXBD_ADDR_LO          1
#define RXBD_IDX_LEN          2
#define RXBD_TYPE_FLAGS       3
#define RXBD_WORDS            8
#define RXBD_FLAG_PACKET_END  BIT(2)
#define RXBD_FLAG_FRAME_ERROR BIT(10)
#define TXBD_ADDR_HI          0
#define TXBD_ADDR_LO          1
#define TXBD_LEN_FLAGS        2
#define TXBD_VLAN             3
#define TXBD_WORDS            4
#define TXBD_FLAG_PACKET_END  BIT(2)

#define RX_RING BCM57762_RX_RING_SIZE
#define TX_RING BCM57762_TX_RING_SIZE
#define BUF_SZ  BCM57762_BUF_SIZE

#define DMA_STATUS_BLOCK 0x0
#define DMA_RX_STD_RING  0x1000
#define DMA_RX_RET_RING  (DMA_RX_STD_RING + RX_RING * RXBD_WORDS * 4)
#define DMA_TX_RING      (DMA_RX_RET_RING + RX_RING * RXBD_WORDS * 4)
#define DMA_RX_BUF       ALIGN_UP(DMA_TX_RING + TX_RING * TXBD_WORDS * 4, 0x1000)
#define DMA_TX_BUF       (DMA_RX_BUF + RX_RING * BUF_SZ)
#define DMA_TOTAL        (DMA_TX_BUF + TX_RING * BUF_SZ)

#define ETH_MIN_FRAME 60
#define ETH_MAX_FRAME 1518
#define ETH_FCS_LEN   4
#define RX_MTU        1522

#define HC_TICKS      1
#define HC_MAX_BDS    1
#define MBUF_LOW_WM   0x2a
#define MBUF_HIGH_WM  0xa0
#define STD_REPL_THR  0x19
#define STD_REPL_WM   0x20
#define RLP_STATS_ALL 0x7bffff
#define SDI_STATS_ALL 0xffffff

#define RESET_DELAY_MS      100
#define GMII_DELAY_MS       40
#define BOOTCODE_TIMEOUT    1000000
#define NVRAM_TIMEOUT       20000
#define BLOCK_TIMEOUT       20000
#define MDIO_TIMEOUT        5000
#define PHY_RESET_TIMEOUT   500000
#define SB_SELFTEST_TIMEOUT 10000

static inline u32 rd(struct bcm57762 *nic, u32 off)
{
    return read32(nic->regs + off);
}

static inline void wr(struct bcm57762 *nic, u32 off, u32 val)
{
    write32(nic->regs + off, val);
}

static inline u32 cfg_rd(struct bcm57762 *nic, u16 off)
{
    return nic->pci.cfg_read32(nic->pci.ctx, off);
}

static inline void cfg_wr(struct bcm57762 *nic, u16 off, u32 val)
{
    nic->pci.cfg_write32(nic->pci.ctx, off, val);
}

static inline u32 mem_rd(struct bcm57762 *nic, u32 addr)
{
    return rd(nic, MEMORY_WINDOW + addr);
}

static inline void mem_wr(struct bcm57762 *nic, u32 addr, u32 val)
{
    wr(nic, MEMORY_WINDOW + addr, val);
}

static int wait_reg(struct bcm57762 *nic, u32 off, u32 mask, u32 val, u32 usec)
{
    u64 timeout = timeout_calculate(usec);

    while ((rd(nic, off) & mask) != val) {
        if (timeout_expired(timeout))
            return (rd(nic, off) & mask) == val ? 0 : -1;
        udelay(10);
    }
    return 0;
}

static int phy_cmd(struct bcm57762 *nic, u32 cmd, u16 *val)
{
    u64 timeout;
    u32 v;

    wr(nic, MII_COMMUNICATION, cmd | MII_COMM_PHY_ADDR(PHY_ADDR) | MII_COMM_START_BUSY);
    timeout = timeout_calculate(MDIO_TIMEOUT);
    while ((v = rd(nic, MII_COMMUNICATION)) & MII_COMM_START_BUSY) {
        if (timeout_expired(timeout)) {
            printf("bcm57762: MDIO timeout (cmd 0x%08x)\n", cmd);
            return -1;
        }
        udelay(5);
    }
    if (!val)
        return 0;
    if (v & MII_COMM_READ_FAILED) {
        printf("bcm57762: MDIO read of reg 0x%02x failed\n", (cmd >> 16) & 0x1f);
        return -1;
    }
    *val = v & MII_COMM_DATA;
    return 0;
}

static int phy_read(struct bcm57762 *nic, u32 reg, u16 *val)
{
    return phy_cmd(nic, MII_COMM_CMD_READ | MII_COMM_REG_ADDR(reg), val);
}

static int phy_write(struct bcm57762 *nic, u32 reg, u16 val)
{
    return phy_cmd(nic, MII_COMM_CMD_WRITE | MII_COMM_REG_ADDR(reg) | val, NULL);
}

static int phy_eee_disable(struct bcm57762 *nic)
{
    u16 val;

    clear32(nic->regs + EEE_MODE, EEE_MODE_USER_LPI_ENABLE);
    if (phy_write(nic, PHY_AUX_CONTROL, PHY_AUX_CTRL_SM_DSP_CLOCK | PHY_AUX_CTRL_TX_6DB_CODING) ||
        phy_write(nic, PHY_DSP_ADDRESS, PHY_DSP_EEE_ADDR) ||
        phy_write(nic, PHY_DSP_RW_PORT, PHY_DSP_EEE_VAL) ||
        phy_write(nic, PHY_AUX_CONTROL, PHY_AUX_CTRL_TX_6DB_CODING) ||
        phy_write(nic, PHY_MMD_CONTROL, MMD_AN) || phy_write(nic, PHY_MMD_DATA, MMD_AN_EEE_ADV) ||
        phy_write(nic, PHY_MMD_CONTROL, PHY_MMD_CONTROL_DATA | MMD_AN) ||
        phy_write(nic, PHY_MMD_DATA, 0) || phy_read(nic, PHY_MMD_DATA, &val))
        return -1;
    wr(nic, EEE_MODE, rd(nic, EEE_MODE) & EEE_MODE_EEE_ENABLE);
    return 0;
}

static int phy_setup(struct bcm57762 *nic)
{
    u16 id_msb, id_lsb, ctrl;
    u64 timeout;

    if (phy_read(nic, PHY_ID_MSB, &id_msb) || phy_read(nic, PHY_ID_LSB, &id_lsb))
        return -1;
    printf("bcm57762: PHY id %04x:%04x\n", id_msb, id_lsb);

    if (phy_write(nic, PHY_MII_CONTROL, PHY_MII_CONTROL_RESET))
        return -1;
    timeout = timeout_calculate(PHY_RESET_TIMEOUT);
    do {
        mdelay(1);
        if (phy_read(nic, PHY_MII_CONTROL, &ctrl))
            return -1;
        if (timeout_expired(timeout) && (ctrl & PHY_MII_CONTROL_RESET)) {
            printf("bcm57762: PHY reset timeout\n");
            return -1;
        }
    } while (ctrl & PHY_MII_CONTROL_RESET);

    if (phy_eee_disable(nic))
        return -1;

    if (phy_write(nic, PHY_AN_ADV,
                  PHY_AN_ADV_802_3 | PHY_AN_ADV_10_HALF | PHY_AN_ADV_10_FULL | PHY_AN_ADV_100_HALF |
                      PHY_AN_ADV_100_FULL) ||
        phy_write(nic, PHY_1000BASE_T_CONTROL, PHY_1000BASE_T_ADV_HALF | PHY_1000BASE_T_ADV_FULL) ||
        phy_write(nic, PHY_MII_CONTROL, PHY_MII_CONTROL_AN_ENABLE | PHY_MII_CONTROL_RESTART_AN))
        return -1;
    return 0;
}

static void mac_hash_set(struct bcm57762 *nic, const u8 *addr)
{
    u32 crc = 0xffffffff;

    for (int i = 0; i < 6; i++) {
        crc ^= addr[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
    set32(nic->regs + MAC_HASH((crc >> 5) & 3), BIT(crc & 0x1f));
}

static void rx_post(struct bcm57762 *nic, u32 buf)
{
    u32 *bd = nic->rx_std + nic->rx_prod * RXBD_WORDS;
    u64 addr = nic->iova + DMA_RX_BUF + (u64)buf * BUF_SZ;

    bd[RXBD_ADDR_HI] = addr >> 32;
    bd[RXBD_ADDR_LO] = addr;
    bd[RXBD_IDX_LEN] = (buf << 16) | BUF_SZ;
    bd[RXBD_TYPE_FLAGS] = RXBD_FLAG_PACKET_END;
    for (int i = RXBD_TYPE_FLAGS + 1; i < RXBD_WORDS; i++)
        bd[i] = 0;
    nic->rx_prod = (nic->rx_prod + 1) % RX_RING;
}

static void rcb_write(struct bcm57762 *nic, u32 rcb, u64 host, u32 maxlen_flags, u32 nic_addr)
{
    mem_wr(nic, rcb + RCB_HOST_ADDR_HI, host >> 32);
    mem_wr(nic, rcb + RCB_HOST_ADDR_LO, host);
    mem_wr(nic, rcb + RCB_MAXLEN_FLAGS, maxlen_flags);
    mem_wr(nic, rcb + RCB_NIC_ADDR, nic_addr);
}

static void pci_setup(struct bcm57762 *nic)
{
    u32 dsc = cfg_rd(nic, PCI_DEVICE_STATUS_CTRL) & 0xffff;

    cfg_wr(nic, PCI_COMMAND,
           (cfg_rd(nic, PCI_COMMAND) & 0xffff) | PCI_COMMAND_MEMORY_SPACE | PCI_COMMAND_BUS_MASTER |
               PCI_COMMAND_INTERRUPT_DISABLE);
    dsc &= ~(DSC_ENABLE_RELAXED_ORDERING | DSC_ENABLE_NO_SNOOP);
    cfg_wr(nic, PCI_DEVICE_STATUS_CTRL, dsc | DSC_ERRORS_DETECTED);
    cfg_wr(nic, PCI_MISC_HOST_CTRL, MHC_INIT);
    cfg_wr(nic, PCI_MEMORY_BASE, 0);
}

static void core_reset(struct bcm57762 *nic)
{
    u32 cfg;

    mem_wr(nic, NIC_FW_MAILBOX, T3_MAGIC_NUMBER);

    wr(nic, SW_ARBITRATION, SW_ARB_REQ_SET1);
    if (wait_reg(nic, SW_ARBITRATION, SW_ARB_ARB_WON1, SW_ARB_ARB_WON1, NVRAM_TIMEOUT))
        printf("bcm57762: NVRAM arbitration not won, continuing\n");

    wr(nic, FAST_BOOT_PC, 0);
    set32(nic->regs + MA_MODE, MODE_ENABLE);

    set32(nic->regs + MISC_CONFIG, MISC_CONFIG_NO_GRC_RESET_PCIE);
    cfg = rd(nic, MISC_CONFIG);
    cfg &= ~(MISC_CONFIG_GPHY_IDDQ | MISC_CONFIG_BIAS_IDDQ | MISC_CONFIG_RAM_POWER_DOWN);
    cfg |= MISC_CONFIG_GRC_RESET | MISC_CONFIG_GPHY_PD_OVERRIDE | MISC_CONFIG_NO_GRC_RESET_PCIE;
    wr(nic, MISC_CONFIG, cfg);
    mdelay(RESET_DELAY_MS);

    pci_setup(nic);
    wr(nic, SW_ARBITRATION, SW_ARB_REQ_CLR1);
    set32(nic->regs + MA_MODE, MODE_ENABLE);
    wr(nic, MODE_CONTROL, MODE_CONTROL_SWAP);
    wr(nic, EMAC_MODE, EMAC_MODE_PORT_MODE_GMII);
    mdelay(GMII_DELAY_MS);
}

static int bootcode_wait(struct bcm57762 *nic)
{
    u64 timeout = timeout_calculate(BOOTCODE_TIMEOUT);
    u32 val;

    while ((val = mem_rd(nic, NIC_FW_MAILBOX)) != ~T3_MAGIC_NUMBER) {
        if (timeout_expired(timeout)) {
            printf("bcm57762: no bootcode handshake (0xb50 = 0x%08x), continuing\n", val);
            return -1;
        }
        udelay(100);
    }
    return 0;
}

static int status_block_check(struct bcm57762 *nic)
{
    static const u32 fix[4] = {0, MODE_CONTROL_BYTE_SWAP_BD, MODE_CONTROL_WORD_SWAP_BD,
                               MODE_CONTROL_BYTE_SWAP_BD | MODE_CONTROL_WORD_SWAP_BD};

    for (int attempt = 0; attempt < 2; attempt++) {
        u64 timeout = timeout_calculate(SB_SELFTEST_TIMEOUT);
        u32 w0 = 0, w1 = 0;
        int k;

        for (int i = 0; i < SB_WORDS; i++)
            nic->status[i] = 0;
        dma_mb();
        set32(nic->regs + HC_MODE, HC_MODE_COALESCE_NOW);
        while (!timeout_expired(timeout)) {
            dma_rmb();
            w0 = nic->status[SB_STATUS];
            w1 = nic->status[SB_TAG];
            if (w0 || w1)
                break;
            udelay(10);
        }

        if (w0 & SB_STATUS_UPDATED)
            k = 0;
        else if (w0 & SB_STATUS_UPDATED_BSWP)
            k = 1;
        else if (w1 & SB_STATUS_UPDATED)
            k = 2;
        else if (w1 & SB_STATUS_UPDATED_BSWP)
            k = 3;
        else
            k = -1;

        if (k == 0)
            return 0;
        if (k < 0) {
            printf("bcm57762: status block not written by DMA (%08x %08x)\n", w0, w1);
            return -1;
        }
        printf("bcm57762: status block arrives swapped (%08x %08x), toggling mode ctrl 0x%x\n", w0,
               w1, fix[k]);
        wr(nic, MODE_CONTROL, rd(nic, MODE_CONTROL) ^ fix[k]);
    }
    return -1;
}

static void stop_block(struct bcm57762 *nic, u32 off)
{
    clear32(nic->regs + off, MODE_ENABLE);
    if (wait_reg(nic, off, MODE_ENABLE, 0, BLOCK_TIMEOUT))
        printf("bcm57762: block at 0x%04x did not stop\n", off);
}

static int hw_init(struct bcm57762 *nic)
{
    u32 val = cfg_rd(nic, PCI_PMCSR);

    if (val & PCI_PMCSR_POWER_STATE) {
        cfg_wr(nic, PCI_PMCSR, val & ~PCI_PMCSR_POWER_STATE);
        mdelay(10);
    }

    pci_setup(nic);
    core_reset(nic);
    bootcode_wait(nic);

    memset(nic->cpu, 0, DMA_RX_BUF);

    val = cfg_rd(nic, PCI_DMA_RW_CTRL) & ~DMA_RW_CTRL_DMA_WRITE_WM;
    if (FIELD_GET(DSC_MAX_PAYLOAD_SIZE, cfg_rd(nic, PCI_DEVICE_STATUS_CTRL)))
        val |= FIELD_PREP(DMA_RW_CTRL_DMA_WRITE_WM, 7);
    else
        val |= FIELD_PREP(DMA_RW_CTRL_DMA_WRITE_WM, 3);
    cfg_wr(nic, PCI_DMA_RW_CTRL, val);

    wr(nic, MODE_CONTROL,
       MODE_CONTROL_SWAP | MODE_CONTROL_HOST_SEND_BDS | MODE_CONTROL_HOST_STACK_UP);
    set32(nic->regs + MSI_MODE, MSI_MODE_SINGLE_SHOT_DISABLE);

    wr(nic, BM_DMA_MBUF_LOW_WM, MBUF_LOW_WM);
    wr(nic, BM_MBUF_HIGH_WM, MBUF_HIGH_WM);
    mask32(nic->regs + LOW_WM_MAX_RX_FRAMES, LOW_WM_MAX_RX_FRAMES_COUNT, 1);
    set32(nic->regs + BM_MODE, MODE_ENABLE | MODE_ATTN_ENABLE);
    if (wait_reg(nic, BM_MODE, MODE_ENABLE, MODE_ENABLE, BLOCK_TIMEOUT)) {
        printf("bcm57762: buffer manager did not start\n");
        return -1;
    }

    wr(nic, RBDI_STD_REPLENISH_THRESH, STD_REPL_THR);
    wr(nic, STD_RCB_HOST_ADDR_HI, (nic->iova + DMA_RX_STD_RING) >> 32);
    wr(nic, STD_RCB_HOST_ADDR_LO, nic->iova + DMA_RX_STD_RING);
    wr(nic, STD_RCB_MAXLEN_FLAGS, RCB_MAX_LEN(RX_RING) | (BUF_SZ << 2));
    wr(nic, STD_RCB_NIC_ADDR, NIC_STD_BD_CACHE);
    wr(nic, JUMBO_RCB_MAXLEN_FLAGS, RCB_FLAG_RING_DISABLED);
    wr(nic, MBOX_RCV_BD_STD_PROD_IDX, 0);
    wr(nic, STD_RING_REPLENISH_WM, STD_REPL_WM);

    wr(nic, MBOX_SEND_BD_HOST_PROD_IDX, 0);
    rcb_write(nic, NIC_SEND_RCB, nic->iova + DMA_TX_RING, RCB_MAX_LEN(TX_RING), NIC_SEND_BD_CACHE);
    mem_wr(nic, NIC_SEND_RCB + NIC_RCB_SIZE + RCB_MAXLEN_FLAGS, RCB_FLAG_RING_DISABLED);

    for (int i = 1; i < 4; i++)
        mem_wr(nic, NIC_RETURN_RCB + i * NIC_RCB_SIZE + RCB_MAXLEN_FLAGS, RCB_FLAG_RING_DISABLED);
    rcb_write(nic, NIC_RETURN_RCB, nic->iova + DMA_RX_RET_RING, RCB_MAX_LEN(RX_RING),
              NIC_STD_BD_CACHE);
    wr(nic, MBOX_RCV_BD_RR0_CONS_IDX, 0);

    for (int i = 0; i < 4; i++) {
        wr(nic, EMAC_MAC_ADDR_HIGH(i), (nic->mac[0] << 8) | nic->mac[1]);
        wr(nic, EMAC_MAC_ADDR_LOW(i),
           (nic->mac[2] << 24) | (nic->mac[3] << 16) | (nic->mac[4] << 8) | nic->mac[5]);
    }
    val = 0;
    for (int i = 0; i < 6; i++)
        val += nic->mac[i];
    wr(nic, TX_RANDOM_BACKOFF, val & TX_RANDOM_BACKOFF_SEED);
    wr(nic, RX_MTU_SIZE, RX_MTU);
    wr(nic, TX_MAC_LENGTHS, TX_MAC_LENGTHS_INIT);

    wr(nic, RX_RULES_CONFIG, RX_RULES_CONFIG_DEFAULT_CLASS(1));
    wr(nic, RLP_CONFIG, RLP_CONFIG_INIT);
    wr(nic, RLP_STATS_ENABLE_MASK, RLP_STATS_ALL);
    set32(nic->regs + RLP_STATS_CTRL, RLP_STATS_CTRL_ENABLE);
    wr(nic, SDI_STATS_MASK, SDI_STATS_ALL);
    set32(nic->regs + SDI_STATS_CTRL, SDI_STATS_CTRL_ENABLE | SDI_STATS_CTRL_FASTER_UPDATE);

    wr(nic, HC_MODE, 0);
    if (wait_reg(nic, HC_MODE, ~0u, 0, BLOCK_TIMEOUT)) {
        printf("bcm57762: host coalescing did not stop\n");
        return -1;
    }
    wr(nic, HC_RX_COAL_TICKS, HC_TICKS);
    wr(nic, HC_TX_COAL_TICKS, HC_TICKS);
    wr(nic, HC_RX_MAX_COAL_BDS, HC_MAX_BDS);
    wr(nic, HC_TX_MAX_COAL_BDS, HC_MAX_BDS);
    wr(nic, HC_RX_MAX_COAL_BDS_INT, HC_MAX_BDS);
    wr(nic, HC_TX_MAX_COAL_BDS_INT, HC_MAX_BDS);
    wr(nic, HC_STATUS_BLOCK_ADDR_HI, (nic->iova + DMA_STATUS_BLOCK) >> 32);
    wr(nic, HC_STATUS_BLOCK_ADDR_LO, nic->iova + DMA_STATUS_BLOCK);
    wr(nic, HC_MODE, MODE_ENABLE | HC_MODE_STATUS_BLOCK_32);

    wr(nic, RBDC_MODE, MODE_ENABLE | MODE_ATTN_ENABLE);
    wr(nic, RLP_MODE, MODE_ENABLE);
    set32(nic->regs + EMAC_MODE,
          EMAC_MODE_ENABLE_FHDE | EMAC_MODE_ENABLE_RDE | EMAC_MODE_ENABLE_TDE);
    set32(nic->regs + EMAC_MODE, EMAC_MODE_CLEAR_TX_STATS | EMAC_MODE_ENABLE_TX_STATS |
                                     EMAC_MODE_CLEAR_RX_STATS | EMAC_MODE_ENABLE_RX_STATS);
    udelay(140);

    set32(nic->regs + WDMA_MODE, MODE_ENABLE | DMA_MODE_ATTN_ALL | WDMA_MODE_STATUS_TAG_FIX);
    udelay(40);
    set32(nic->regs + RDMA_MODE, MODE_ENABLE | DMA_MODE_ATTN_ALL);
    udelay(40);

    wr(nic, RDC_MODE, MODE_ENABLE | MODE_ATTN_ENABLE);
    wr(nic, SDC_MODE, MODE_ENABLE);
    wr(nic, SBDC_MODE, MODE_ENABLE | MODE_ATTN_ENABLE);
    wr(nic, RBDI_MODE, MODE_ENABLE | RBDI_MODE_BDS_ON_DISABLED);
    wr(nic, RDI_MODE, MODE_ENABLE | RDI_MODE_ILLEGAL_RR_SIZE);
    wr(nic, SDI_MODE, MODE_ENABLE);
    wr(nic, SBDI_MODE, MODE_ENABLE | MODE_ATTN_ENABLE);
    wr(nic, SBDS_MODE, MODE_ENABLE | MODE_ATTN_ENABLE);

    if (status_block_check(nic))
        return -1;

    for (u32 i = 0; i < RX_RING - 1; i++)
        rx_post(nic, i);
    dma_wmb();
    wr(nic, MBOX_RCV_BD_STD_PROD_IDX, nic->rx_prod);

    wr(nic, TX_MAC_MODE, TX_MAC_MODE_ENABLE | TX_MAC_MODE_BAD_TXMBUF_FIX);
    udelay(100);
    set32(nic->regs + RX_MAC_MODE, RX_MAC_MODE_ENABLE);
    udelay(10);

    wr(nic, LED_CONTROL, LED_CONTROL_INIT);
    set32(nic->regs + MII_STATUS, MII_STATUS_LINK_STATUS);
    clear32(nic->regs + MII_MODE, MII_MODE_PORT_POLLING);
    udelay(40);
    mask32(nic->regs + LOW_WM_MAX_RX_FRAMES, LOW_WM_MAX_RX_FRAMES_COUNT, 1);

    if (phy_setup(nic))
        return -1;

    for (int i = 0; i < 4; i++)
        wr(nic, MAC_HASH(i), 0);
    mac_hash_set(nic, (const u8[6]){0xff, 0xff, 0xff, 0xff, 0xff, 0xff});

    printf("bcm57762: chip %08x up, MAC %02x:%02x:%02x:%02x:%02x:%02x, DMA at 0x%lx\n",
           rd(nic, CHIP_ID), nic->mac[0], nic->mac[1], nic->mac[2], nic->mac[3], nic->mac[4],
           nic->mac[5], nic->iova);
    return 0;
}

int bcm57762_init(struct bcm57762 *nic, u64 bar0, const struct bcm57762_pci *pci,
                  const struct bcm57762_dma *dma, const u8 mac[6])
{
    memset(nic, 0, sizeof(*nic));
    nic->regs = bar0;
    nic->pci = *pci;
    memcpy(nic->mac, mac, sizeof(nic->mac));

    if (dma->size < DMA_TOTAL || (dma->iova >> 32) != ((dma->iova + DMA_TOTAL - 1) >> 32)) {
        printf("bcm57762: DMA region 0x%lx+0x%lx unusable (need 0x%x, no 4 GiB crossing)\n",
               dma->iova, (u64)dma->size, DMA_TOTAL);
        return -1;
    }
    nic->cpu = dma->cpu;
    nic->iova = dma->iova;
    nic->status = (volatile u32 *)(void *)(nic->cpu + DMA_STATUS_BLOCK);
    nic->rx_std = (u32 *)(void *)(nic->cpu + DMA_RX_STD_RING);
    nic->rx_ret = (u32 *)(void *)(nic->cpu + DMA_RX_RET_RING);
    nic->tx_ring = (u32 *)(void *)(nic->cpu + DMA_TX_RING);
    nic->rx_buf = nic->cpu + DMA_RX_BUF;
    nic->tx_buf = nic->cpu + DMA_TX_BUF;

    if (cfg_rd(nic, PCI_VENDOR_DEVICE) == 0xffffffff) {
        printf("bcm57762: device not responding to config reads\n");
        return -1;
    }
    if (hw_init(nic)) {
        bcm57762_shutdown(nic);
        return -1;
    }
    nic->up = true;
    return 0;
}

bool bcm57762_link_poll(struct bcm57762 *nic)
{
    static const u16 hcd_speed[8] = {0, 10, 10, 100, 100, 100, 1000, 1000};
    u16 aux;
    u32 hcd, speed;
    bool up, full;

    if (!nic->up || phy_read(nic, PHY_AUX_STATUS, &aux))
        return false;

    hcd = FIELD_GET(PHY_AUX_STATUS_HCD, aux);
    up = (aux & PHY_AUX_STATUS_LINK) && hcd;
    speed = up ? hcd_speed[hcd] : 0;
    full = up && (hcd == 2 || hcd == 5 || hcd == 7);

    if (up == nic->link && speed == nic->speed && full == nic->full_duplex)
        return up;

    if (up) {
        u32 port = speed == 1000 ? EMAC_MODE_PORT_MODE_GMII : EMAC_MODE_PORT_MODE_MII;
        mask32(nic->regs + EMAC_MODE, EMAC_MODE_PORT_MODE | EMAC_MODE_HALF_DUPLEX,
               port | (full ? 0 : EMAC_MODE_HALF_DUPLEX));
        printf("bcm57762: link up, %u Mb/s %s duplex\n", speed, full ? "full" : "half");
    } else {
        printf("bcm57762: link down\n");
    }
    nic->link = up;
    nic->speed = speed;
    nic->full_duplex = full;
    return up;
}

int bcm57762_tx(void *opaque, const void *frame, size_t len)
{
    struct bcm57762 *nic = opaque;
    u32 next, cons, *bd;
    u8 *buf;
    u64 addr;

    if (!nic->up || !len || len > ETH_MAX_FRAME)
        return -1;

    next = (nic->tx_prod + 1) % TX_RING;
    cons = FIELD_GET(SB_IDX_SEND_CONS, nic->status[SB_IDX]) % TX_RING;
    if (next == cons) {
        nic->tx_busy++;
        return -1;
    }

    buf = nic->tx_buf + nic->tx_prod * BUF_SZ;
    memcpy(buf, frame, len);
    if (len < ETH_MIN_FRAME) {
        memset(buf + len, 0, ETH_MIN_FRAME - len);
        len = ETH_MIN_FRAME;
    }

    addr = nic->iova + DMA_TX_BUF + (u64)nic->tx_prod * BUF_SZ;
    bd = nic->tx_ring + nic->tx_prod * TXBD_WORDS;
    bd[TXBD_ADDR_HI] = addr >> 32;
    bd[TXBD_ADDR_LO] = addr;
    bd[TXBD_LEN_FLAGS] = (len << 16) | TXBD_FLAG_PACKET_END;
    bd[TXBD_VLAN] = 0;
    nic->tx_prod = next;

    dma_wmb();
    wr(nic, MBOX_SEND_BD_HOST_PROD_IDX, nic->tx_prod);
    return 0;
}

ssize_t bcm57762_rx(void *opaque, void *frame, size_t maxlen)
{
    struct bcm57762 *nic = opaque;

    if (!nic->up)
        return 0;

    for (;;) {
        u32 prod = FIELD_GET(SB_IDX_RR0_PROD, nic->status[SB_IDX]) % RX_RING;
        u32 *bd, buf, len, flags;
        ssize_t ret = 0;

        if (prod == nic->rx_cons)
            return 0;
        dma_rmb();

        bd = nic->rx_ret + nic->rx_cons * RXBD_WORDS;
        buf = bd[RXBD_IDX_LEN] >> 16;
        len = bd[RXBD_IDX_LEN] & 0xffff;
        flags = bd[RXBD_TYPE_FLAGS] & 0xffff;
        nic->rx_cons = (nic->rx_cons + 1) % RX_RING;

        if (buf >= RX_RING) {
            printf("bcm57762: bogus RX buffer index %u\n", buf);
            nic->rx_errors++;
        } else {
            if ((flags & RXBD_FLAG_FRAME_ERROR) || len <= ETH_FCS_LEN) {
                nic->rx_errors++;
            } else if (len - ETH_FCS_LEN > maxlen) {
                nic->rx_dropped++;
            } else {
                ret = len - ETH_FCS_LEN;
                memcpy(frame, nic->rx_buf + buf * BUF_SZ, ret);
            }
            rx_post(nic, buf);
        }

        dma_mb();
        wr(nic, MBOX_RCV_BD_RR0_CONS_IDX, nic->rx_cons);
        wr(nic, MBOX_RCV_BD_STD_PROD_IDX, nic->rx_prod);
        if (ret)
            return ret;
    }
}

void bcm57762_set_promisc(struct bcm57762 *nic, bool on)
{
    if (nic->regs)
        mask32(nic->regs + RX_MAC_MODE, RX_MAC_MODE_PROMISCUOUS, on ? RX_MAC_MODE_PROMISCUOUS : 0);
}

void bcm57762_mcast_add(struct bcm57762 *nic, const u8 addr[6])
{
    if (nic->regs)
        mac_hash_set(nic, addr);
}

void bcm57762_shutdown(struct bcm57762 *nic)
{
    static const u32 blocks[] = {RX_MAC_MODE, RBDI_MODE, RLP_MODE,  RDI_MODE, RDC_MODE,
                                 RBDC_MODE,   SBDS_MODE, SBDI_MODE, SDI_MODE, RDMA_MODE,
                                 SDC_MODE,    SBDC_MODE, HC_MODE,   WDMA_MODE};

    if (!nic->regs || !nic->pci.cfg_read32)
        return;

    for (u32 i = 0; i < ARRAY_SIZE(blocks); i++)
        stop_block(nic, blocks[i]);
    clear32(nic->regs + EMAC_MODE,
            EMAC_MODE_ENABLE_FHDE | EMAC_MODE_ENABLE_RDE | EMAC_MODE_ENABLE_TDE);
    clear32(nic->regs + TX_MAC_MODE, TX_MAC_MODE_ENABLE);
    cfg_wr(nic, PCI_COMMAND, cfg_rd(nic, PCI_COMMAND) & 0xffff & ~PCI_COMMAND_BUS_MASTER);
    nic->up = false;
}

void bcm57762_dump(struct bcm57762 *nic)
{
    u16 ctrl = 0, stat = 0, aux = 0, lpa = 0, gstat = 0;

    if (!nic->regs || !nic->pci.cfg_read32 || cfg_rd(nic, PCI_VENDOR_DEVICE) == 0xffffffff) {
        printf("bcm57762: not initialized or not responding\n");
        return;
    }

    printf("bcm57762: pci id %08x rev %08x cmd %08x pmcsr %08x mhc %08x dmarw %08x devctl %08x\n",
           cfg_rd(nic, PCI_VENDOR_DEVICE), cfg_rd(nic, PCI_CLASS_REVISION),
           cfg_rd(nic, PCI_COMMAND), cfg_rd(nic, PCI_PMCSR), cfg_rd(nic, PCI_MISC_HOST_CTRL),
           cfg_rd(nic, PCI_DMA_RW_CTRL), cfg_rd(nic, PCI_DEVICE_STATUS_CTRL));
    printf("bcm57762: chip %08x mode_ctrl %08x misc_cfg %08x fw_mbox %08x eee %08x\n",
           rd(nic, CHIP_ID), rd(nic, MODE_CONTROL), rd(nic, MISC_CONFIG),
           mem_rd(nic, NIC_FW_MAILBOX), rd(nic, EEE_MODE));
    printf("bcm57762: emac mode %08x status %08x | txmac mode %08x status %08x | rxmac mode %08x "
           "status %08x | mi mode %08x status %08x\n",
           rd(nic, EMAC_MODE), rd(nic, EMAC_STATUS), rd(nic, TX_MAC_MODE), rd(nic, TX_MAC_STATUS),
           rd(nic, RX_MAC_MODE), rd(nic, RX_MAC_STATUS), rd(nic, MII_MODE), rd(nic, MII_STATUS));

    if (!phy_read(nic, PHY_MII_CONTROL, &ctrl) && !phy_read(nic, PHY_MII_STATUS, &stat) &&
        !phy_read(nic, PHY_AN_LINK_PARTNER, &lpa) &&
        !phy_read(nic, PHY_1000BASE_T_STATUS, &gstat) && !phy_read(nic, PHY_AUX_STATUS, &aux))
        printf("bcm57762: phy ctrl %04x status %04x lpa %04x 1000t %04x aux %04x\n", ctrl, stat,
               lpa, gstat, aux);
    printf("bcm57762: link %s %u Mb/s %s, up %d\n", nic->link ? "up" : "down", nic->speed,
           nic->full_duplex ? "full" : "half", nic->up);

    printf("bcm57762: sw rx_prod %u rx_cons %u tx_prod %u | nic std_prod %u rr0_prod %u "
           "send_cons %u\n",
           nic->rx_prod, nic->rx_cons, nic->tx_prod, rd(nic, NIC_STD_RCV_BD_PROD_IDX),
           rd(nic, NIC_DIAG_RR0_PROD_IDX), rd(nic, NIC_DIAG_SEND_BD_CONS_IDX));
    if (nic->status)
        printf("bcm57762: status block %08x %08x %08x %08x %08x %08x\n", nic->status[0],
               nic->status[1], nic->status[2], nic->status[3], nic->status[4], nic->status[5]);
    printf("bcm57762: flow_attn %08x hc %08x/%08x rdma %08x/%08x wdma %08x/%08x bm %08x/%08x "
           "ma %08x\n",
           rd(nic, FLOW_ATTENTION), rd(nic, HC_MODE), rd(nic, HC_STATUS), rd(nic, RDMA_MODE),
           rd(nic, RDMA_STATUS), rd(nic, WDMA_MODE), rd(nic, WDMA_STATUS), rd(nic, BM_MODE),
           rd(nic, BM_STATUS), rd(nic, MA_STATUS));
    printf("bcm57762: rbdc %08x rdi %08x rlp %08x sbds %08x sbdi %08x sdi %08x msi %08x\n",
           rd(nic, RBDC_STATUS), rd(nic, RDI_STATUS), rd(nic, RLP_STATUS), rd(nic, SBDS_STATUS),
           rd(nic, SBDI_STATUS), rd(nic, SDI_STATUS), rd(nic, MSI_STATUS));
    printf("bcm57762: rx_errors %u rx_dropped %u tx_busy %u\n", nic->rx_errors, nic->rx_dropped,
           nic->tx_busy);
}

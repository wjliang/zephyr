/*
 * Copyright (c) 2018 Marvell
 * Copyright (c) 2018 Lexmark International, Inc.
 */

#include <device.h>
#include <irq_nextlevel.h>
#include <sw_isr_table.h>

#define CPU_ID			CONFIG_CPU0_ID
#define XSCUGIC_MAX_IRQS	CONFIG_NUM_IRQS

/*
 * First Interrupt Id for SPI interrupts.
 */
#define XSCUGIC_SPI_INT_ID_START	0x20

/*
 * The maximum priority value that can be used in the GIC.
 */
#define XSCUGIC_MAX_INTR_PRIO_VAL	248U
#define XSCUGIC_INTR_PRIO_MASK		0x000000F8U

#define XSCUGIC_BASEADDR	CONFIG_XSCUGIC_BASEADDR
#define XSCUGIC_DIST_EN		(XSCUGIC_BASEADDR + 0x000)
#define XSCUGIC_IC_TYPE		(XSCUGIC_BASEADDR + 0x004)
#define XSCUGIC_DIST_IDENT	(XSCUGIC_BASEADDR + 0x008)
#define XSCUGIC_SECURITY	(XSCUGIC_BASEADDR + 0x080)
#define XSCUGIC_ENABLE_SET	(XSCUGIC_BASEADDR + 0x100)
#define XSCUGIC_DISABLE		(XSCUGIC_BASEADDR + 0x180)
#define XSCUGIC_PENDING_SET	(XSCUGIC_BASEADDR + 0x200)
#define XSCUGIC_PENDING_CLR	(XSCUGIC_BASEADDR + 0x280)
#define XSCUGIC_ACTIVE		(XSCUGIC_BASEADDR + 0x300)
#define XSCUGIC_PRIORITY	(XSCUGIC_BASEADDR + 0x400)
#define XSCUGIC_SPI_TARGET	(XSCUGIC_BASEADDR + 0x800)
#define XSCUGIC_INT_CFG		(XSCUGIC_BASEADDR + 0xC00)
#define XSCUGIC_PPI_STAT	(XSCUGIC_BASEADDR + 0xD00)
#define XSCUGIC_SPI_STAT	(XSCUGIC_BASEADDR + 0xD04)
#define XSCUGIC_AHB_CONFIG	(XSCUGIC_BASEADDR + 0xD80)
#define XSCUGIC_SFI_TRIG	(XSCUGIC_BASEADDR + 0xF00)
#define XSCUGIC_PERPHID		(XSCUGIC_BASEADDR + 0xFD0)
#define XSCUGIC_PCELLID		(XSCUGIC_BASEADDR + 0xFF0)

/* CPU Interface Registers */
#define XSCUGIC_CPU_BASEADDR	CONFIG_XSCUGIC_CPU_BASEADDR
#define XSCUGIC_CPU_CONTROL	(XSCUGIC_CPU_BASEADDR + 0x000)
#define XSCUGIC_CPU_PRIOR	(XSCUGIC_CPU_BASEADDR + 0x004)
#define XSCUGIC_BIN_PT		(XSCUGIC_CPU_BASEADDR + 0x008)
#define XSCUGIC_INT_ACK		(XSCUGIC_CPU_BASEADDR + 0x00C)
#define XSCUGIC_EOI		(XSCUGIC_CPU_BASEADDR + 0x010)
#define XSCUGIC_RUN_PRIOR	(XSCUGIC_CPU_BASEADDR + 0x014)
#define XSCUGIC_HI_PEND		(XSCUGIC_CPU_BASEADDR + 0x018)

#define XSCUGIC_SPI_TARGET_OFFSET_CALC(irq_id) (((irq_id)/4U) * 4U)
#define XSCUGIC_EN_DIS_OFFSET_CALC(irq_id) (((irq_id)/32U) * 4U)
#define XSCUGIC_INT_CFG_OFFSET_CALC(irq_id) (((irq_id)/16U) *4U)
#define XSCUGIC_PRIORITY_OFFSET_CALC(irq_id) (((irq_id)/4U) *4U)

#define XSCUGIC_EN_INT_MASK	0x00000001U

#define XSCUGIC_LSPI_MASK	0x0000F800U
#define XSCUGIC_DOMAIN_MASK	0x00000400U
#define XSCUGIC_CPU_NUM_MASK	0x000000E0U
#define XSCUGIC_NUM_INT_MASK	0x0000001FU
#define XSCUGIC_INT_NS_MASK	0x00000001U
#define XSCUGIC_INT_EN_MASK	0x00000001U

#define XSCUGIC_REV_MASK	0x00FFF000U
#define XSCUGIC_IMPL_MASK	0x00000FFFU
#define XSCUGIC_PRIORITY_MASK	0x000000FFU
#define XSCUGIC_PRIORITY_MAX	0x000000FFU
#define XSCUGIC_INT_CFG_MASK	0x00000003U
#define XSCUGIC_ACK_INTID_MASK	0x000003FFU

#define XSCUGIC_CPU_ID_CALC(cpu_id) \
	((1 << (cpu_id)) | ((1 << (cpu_id)) << 8) | ((1 << (cpu_id)) << 16))

struct gic_ictl_config {
	u32_t isr_table_offset;
};

static int is_gic_dist_enabled()
{
	u32_t val;

	val = sys_read32(XSCUGIC_DIST_EN);
	return (val & XSCUGIC_EN_INT_MASK) != 0 ? 1 : 0;
}

static void gic_cpu_stop (u32_t cpu_id)
{
	u32_t val;
	u32_t i;
	u32_t lcpu_id;
	int gic_dist_disable; /* Track to see if distributor can be disabled. */

	gic_dist_disable = 1;
	lcpu_id = XSCUGIC_CPU_ID_CALC(cpu_id);
	for (i = 32; i < XSCUGIC_MAX_IRQS; i += 4) {
		val = sys_read32(XSCUGIC_SPI_TARGET +
				 XSCUGIC_SPI_TARGET_OFFSET_CALC(i));
		if ((val != lcpu_id) && (val != 0)) {
			/* if another CPU is programmed to target register,
			 * do not disable gic distributor.
			 */
			gic_dist_disable = 0;
		}
		/* Remove current CPU from interrupt target register */
		val &= (~lcpu_id);
		sys_write32(val, XSCUGIC_SPI_TARGET +
			    XSCUGIC_SPI_TARGET_OFFSET_CALC(i));
	}
	if (is_gic_dist_enabled() == 0) {
		gic_dist_disable = 0;
	}

	if (gic_dist_disable == 1) {
		for (i = 0; i < XSCUGIC_MAX_IRQS; i += 32) {
			sys_write32(0xFFFFFFFFU, XSCUGIC_DISABLE +
				    XSCUGIC_EN_DIS_OFFSET_CALC(i));
		}
		sys_write32(0, XSCUGIC_DIST_EN);
	}
}

static void gic_cpu_init(uint32_t cpu_id)
{
	(void)cpu_id;
	/* program priorty mask of the CPU using the priority mask register */
	sys_write32(0xF0U, XSCUGIC_CPU_PRIOR);

	sys_write32(0x07, XSCUGIC_CPU_CONTROL);
}

static void gic_dist_init(void)
{
	unsigned int i;
	u32_t val, cpu_id, lcpu_id;

	cpu_id = CPU_ID;
	gic_cpu_stop(cpu_id);

	if (is_gic_dist_enabled() != 0) {
		sys_write32(0, XSCUGIC_DIST_EN);

		/* initialize interrupt config registers */
		for (i = 32; i < XSCUGIC_MAX_IRQS; i += 16) {
			sys_write32(0, XSCUGIC_INT_CFG +
				    XSCUGIC_INT_CFG_OFFSET_CALC(i));
		}

		/* set default priority */
		for (i = 0; i < XSCUGIC_MAX_IRQS; i += 4) {
			sys_write32(0xa0a0a0a0U, XSCUGIC_PRIORITY +
				    XSCUGIC_PRIORITY_OFFSET_CALC(i));
		}

		/* Disable SPIs */
		for (i = 0; i < XSCUGIC_MAX_IRQS; i += 32) {
			sys_write32(0xFFFFFFFFU, XSCUGIC_DISABLE +
				    XSCUGIC_EN_DIS_OFFSET_CALC(i));
		}
	}

	lcpu_id  = XSCUGIC_CPU_ID_CALC(cpu_id);
	for (i = 32; i < XSCUGIC_MAX_IRQS; i += 4) {
		val = sys_read32(XSCUGIC_SPI_TARGET +
				 XSCUGIC_SPI_TARGET_OFFSET_CALC(i));
		val |= lcpu_id;
		sys_write32(val, XSCUGIC_SPI_TARGET +
			    XSCUGIC_SPI_TARGET_OFFSET_CALC(i));
	}

	/* Enable Distributor */
	sys_write32(XSCUGIC_EN_INT_MASK, XSCUGIC_DIST_EN);

	gic_cpu_init(cpu_id);
}

static void gic_irq_map_to_cpu(u8_t cpu_id, u32_t irq_id)
{
	u32_t regval, offset;

	regval = sys_read32(XSCUGIC_SPI_TARGET +
			    XSCUGIC_SPI_TARGET_OFFSET_CALC(irq_id));
	offset = (irq_id & 0x3U);
	cpu_id = (0x1U << cpu_id);

	regval = (regval & (~(0xFFU << (offset * 8U))));
	regval |= ((cpu_id) << (offset * 8U));

	sys_write32(regval,
		    XSCUGIC_SPI_TARGET +
		    XSCUGIC_SPI_TARGET_OFFSET_CALC(irq_id));
}

static void gic_irq_enable(struct device *dev, unsigned int irq)
{
	int int_off;

	irq += XSCUGIC_SPI_INT_ID_START;
	int_off = irq % 32;

	gic_irq_map_to_cpu((u8_t)CPU_ID, (u32_t)irq);
	sys_write32((1 << int_off), XSCUGIC_ENABLE_SET +
		    XSCUGIC_EN_DIS_OFFSET_CALC(irq));
}

static void gic_irq_disable(struct device *dev, unsigned int irq)
{
	int int_off;

	irq += XSCUGIC_SPI_INT_ID_START;
	int_off = irq % 32;

	sys_write32((1 << int_off), XSCUGIC_DISABLE +
		    XSCUGIC_EN_DIS_OFFSET_CALC(irq));
}

static unsigned int gic_irq_get_state(struct device *dev)
{
	return 1;
}

static void gic_irq_set_priority(struct device *dev,
		unsigned int irq, unsigned int prio, u32_t flags)
{
	u32_t val;
	u8_t prio8;
	u8_t flags8;

	irq += XSCUGIC_SPI_INT_ID_START;

	/* Set priority */
	val = sys_read32(XSCUGIC_PRIORITY + XSCUGIC_PRIORITY_OFFSET_CALC(irq));
	prio8 =(u8_t)(prio & (u8_t)XSCUGIC_INTR_PRIO_MASK);
	val &= ~(XSCUGIC_PRIORITY_MASK) << ((irq % 4U) * 8U);
	val |= (u32_t)prio8 << ((irq %4) * 8);

	sys_write32(val, XSCUGIC_PRIORITY +
		    XSCUGIC_PRIORITY_OFFSET_CALC(irq));

	/* Set interrupt type */
	flags8 = (u8_t)(flags);
	val = sys_read32(XSCUGIC_INT_CFG + XSCUGIC_INT_CFG_OFFSET_CALC(irq));
	val &= ~(XSCUGIC_INT_CFG_MASK << ((irq % 16U) * 2U));
	val |= (u32_t)flags8 << ((irq % 16U) *2U);

	sys_write32(val, XSCUGIC_INT_CFG + XSCUGIC_INT_CFG_OFFSET_CALC(irq));
}

static void gic_isr(void *arg)
{
	struct device *dev = arg;
	const struct gic_ictl_config *cfg = dev->config->config_info;
	void (*gic_isr_handle)(void *);
	u32_t irq, isr_offset, irq_id_full;

	irq_id_full = sys_read32(XSCUGIC_INT_ACK);
	irq = irq_id_full & XSCUGIC_ACK_INTID_MASK;

	if (irq < XSCUGIC_MAX_IRQS) {
		isr_offset = cfg->isr_table_offset + irq -
			     XSCUGIC_SPI_INT_ID_START;

		gic_isr_handle = _sw_isr_table[isr_offset].isr;
		if (gic_isr_handle)
			gic_isr_handle(_sw_isr_table[isr_offset].arg);
		else
			printk("gic: no handler found for int %d\n", irq);
	} else {
		printk("Invalid irq %d\n", irq);
	}

	/* set to inactive */
	sys_write32(irq_id_full, XSCUGIC_EOI);
}

static int gic_init(struct device *unused);
static const struct irq_next_level_api gic_apis = {
	.intr_enable = gic_irq_enable,
	.intr_disable = gic_irq_disable,
	.intr_get_state = gic_irq_get_state,
	.intr_set_priority = gic_irq_set_priority,
};

#define XSCUGIC_ISR_TBL_OFFSET 0
static const struct gic_ictl_config gic_config = {
	.isr_table_offset = XSCUGIC_ISR_TBL_OFFSET,
};

DEVICE_AND_API_INIT(xlnx_scugic, "Xilinx SCUGIC",
		gic_init, NULL, &gic_config,
		PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &gic_apis);

/**
 *
 * @brief Initialize the GIC device driver
 *
 *
 * @return N/A
 */
#define GIC_PARENT_IRQ 0
#define GIC_PARENT_IRQ_PRI 0
#define GIC_PARENT_IRQ_FLAGS 0
static int gic_init(struct device *unused)
{
	IRQ_CONNECT(GIC_PARENT_IRQ, GIC_PARENT_IRQ_PRI, gic_isr,
		    DEVICE_GET(xlnx_scugic), GIC_PARENT_IRQ_FLAGS);

	/* Init of Distributor interface registers */
	gic_dist_init();

	return 0;
}

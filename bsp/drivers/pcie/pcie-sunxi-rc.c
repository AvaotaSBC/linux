/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
// SPDX_License-Identifier: GPL-2.0
/*
 * allwinner PCIe host controller driver
 *
 * Copyright (c) 2007-2022 Allwinnertech Co., Ltd.
 *
 * Author: songjundong <songjundong@allwinnertech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "pci.h"
#include "pcie-sunxi.h"
#include "pcie-sunxi-dma.h"

static int sunxi_pcie_host_link_up(struct sunxi_pcie_port *pp)
{
	if (pp->ops->link_up)
		return pp->ops->link_up(pp);
	else
		return 0;
}

static int sunxi_pcie_host_rd_own_conf(struct sunxi_pcie_port *pp, int where, int size, u32 *val)
{
	int ret;

	if (pp->ops->rd_own_conf)
		ret = pp->ops->rd_own_conf(pp, where, size, val);
	else
		ret = sunxi_pcie_cfg_read(pp->dbi_base + where, size, val);

	return ret;
}

int sunxi_pcie_host_wr_own_conf(struct sunxi_pcie_port *pp, int where, int size, u32 val)
{
	int ret;

	if (pp->ops->wr_own_conf)
		ret = pp->ops->wr_own_conf(pp, where, size, val);
	else
		ret = sunxi_pcie_cfg_write(pp->dbi_base + where, size, val);

	return ret;
}

static void sunxi_msi_top_irq_ack(struct irq_data *d)
{
	/* NULL */
}

static struct irq_chip sunxi_msi_top_chip = {
	.name	     = "SUNXI-PCIe-MSI",
	.irq_ack     = sunxi_msi_top_irq_ack,
	.irq_mask    = pci_msi_mask_irq,
	.irq_unmask  = pci_msi_unmask_irq,
};

static int sunxi_msi_set_affinity(struct irq_data *d, const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void sunxi_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct sunxi_pcie_port *pcie = irq_data_get_irq_chip_data(data);
	phys_addr_t pa = ALIGN_DOWN(virt_to_phys(pcie), SZ_4K);

	msg->address_lo = lower_32_bits(pa);
	msg->address_hi = upper_32_bits(pa);
	msg->data = data->hwirq;
}

/*
 * whether the following interface needs to be added on the driver:
 * .irq_ack, .irq_mask, .irq_unmask and the xxx_bottom_irq_chip.
 */
static struct irq_chip sunxi_msi_bottom_chip = {
	.name			= "SUNXI MSI",
	.irq_set_affinity 	= sunxi_msi_set_affinity,
	.irq_compose_msi_msg	= sunxi_compose_msi_msg,
};

static int sunxi_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs, void *args)
{
	struct sunxi_pcie_port *pp = domain->host_data;
	int hwirq, i;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	hwirq = bitmap_find_free_region(pp->msi_map, INT_PCI_MSI_NR, order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);

	if (unlikely(hwirq < 0)) {
		dev_err(pp->dev, "failed to alloc hwirq\n");
		return -ENOSPC;
	}

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    &sunxi_msi_bottom_chip, pp,
				    handle_edge_irq, NULL, NULL);

	return 0;
}

static void sunxi_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct sunxi_pcie_port *pp = domain->host_data;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bitmap_release_region(pp->msi_map, d->hwirq, order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static const struct irq_domain_ops sunxi_msi_domain_ops = {
	.alloc	= sunxi_msi_domain_alloc,
	.free	= sunxi_msi_domain_free,
};

static struct msi_domain_info sunxi_msi_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS),
	.chip	= &sunxi_msi_top_chip,
};

static int sunxi_allocate_msi_domains(struct sunxi_pcie_port *pp)
{
	struct fwnode_handle *fwnode = dev_fwnode(pp->dev);

	pp->irq_domain = irq_domain_create_linear(fwnode, INT_PCI_MSI_NR,
							  &sunxi_msi_domain_ops, pp);
	if (!pp->irq_domain) {
		dev_err(pp->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}
	irq_domain_update_bus_token(pp->irq_domain, DOMAIN_BUS_NEXUS);

	pp->msi_domain = pci_msi_create_irq_domain(fwnode, &sunxi_msi_info, pp->irq_domain);
	if (!pp->msi_domain) {
		dev_err(pp->dev, "failed to create MSI domain\n");
		irq_domain_remove(pp->irq_domain);
		return -ENOMEM;
	}

	return 0;
}

static void sunxi_free_msi_domains(struct sunxi_pcie_port *pp)
{
	irq_domain_remove(pp->msi_domain);
	irq_domain_remove(pp->irq_domain);
}

static void sunxi_pcie_prog_outbound_atu(struct sunxi_pcie_port *pp, int index, int type,
					u64 cpu_addr, u64 pci_addr, u32 size)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_pp(pp);

	sunxi_pcie_writel_dbi(pci, PCIE_ATU_LOWER_BASE_OUTBOUND(index), lower_32_bits(cpu_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_UPPER_BASE_OUTBOUND(index), upper_32_bits(cpu_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_LIMIT_OUTBOUND(index), lower_32_bits(cpu_addr + size - 1));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_LOWER_TARGET_OUTBOUND(index), lower_32_bits(pci_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_UPPER_TARGET_OUTBOUND(index), upper_32_bits(pci_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR1_OUTBOUND(index), type);
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR2_OUTBOUND(index), PCIE_ATU_ENABLE);
}

static int sunxi_pcie_rd_other_conf(struct sunxi_pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val)
{
	int ret = PCIBIOS_SUCCESSFUL, type;
	u64 busdev;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (pci_is_root_bus(bus->parent))
		type = PCIE_ATU_TYPE_CFG0;
	else
		type = PCIE_ATU_TYPE_CFG1;

	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_INDEX0, type, pp->cfg0_base, busdev, pp->cfg0_size);

	ret = sunxi_pcie_cfg_read(pp->va_cfg0_base + where, size, val);

	return ret;
}

static int sunxi_pcie_wr_other_conf(struct sunxi_pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 val)
{
	int ret = PCIBIOS_SUCCESSFUL, type;
	u64 busdev;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (pci_is_root_bus(bus->parent))
		type = PCIE_ATU_TYPE_CFG0;
	else
		type = PCIE_ATU_TYPE_CFG1;

	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_INDEX0, type, pp->cfg0_base, busdev, pp->cfg0_size);

	ret = sunxi_pcie_cfg_write(pp->va_cfg0_base + where, size, val);

	return ret;
}

static int sunxi_pcie_valid_config(struct sunxi_pcie_port *pp,
				struct pci_bus *bus, int dev)
{
	/* If there is no link, then there is no device */
	if (!pci_is_root_bus(bus)) {
		if (!sunxi_pcie_host_link_up(pp))
			return 0;
	} else if (dev > 0)
		/* Access only one slot on each root port */
		return 0;

	return 1;
}

static int sunxi_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct sunxi_pcie_port *pp = (bus->sysdata);
	int ret;

	if (!pp)
		BUG();

	if (!sunxi_pcie_valid_config(pp, bus, PCI_SLOT(devfn))) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (!pci_is_root_bus(bus))
		ret = sunxi_pcie_rd_other_conf(pp, bus, devfn,
						where, size, val);
	else
		ret = sunxi_pcie_host_rd_own_conf(pp, where, size, val);

	return ret;
}

static int sunxi_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct sunxi_pcie_port *pp = (bus->sysdata);
	int ret;

	if (!pp)
		BUG();

	if (sunxi_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (!pci_is_root_bus(bus))
		ret = sunxi_pcie_wr_other_conf(pp, bus, devfn,
						where, size, val);
	else
		ret = sunxi_pcie_host_wr_own_conf(pp, where, size, val);

	return ret;
}

static struct pci_ops sunxi_pcie_ops = {
	.read = sunxi_pcie_rd_conf,
	.write = sunxi_pcie_wr_conf,
};

int sunxi_pcie_host_init(struct sunxi_pcie_port *pp)
{
	struct device *dev = pp->dev;
	struct resource_entry *win;
	struct pci_host_bridge *bridge;
	int ret;

	bridge = devm_pci_alloc_host_bridge(dev, 0);
	if (!bridge)
		return -ENOMEM;

	pp->bridge = bridge;

	/* Get the I/O and memory ranges from DT */
	resource_list_for_each_entry(win, &bridge->windows) {
		switch (resource_type(win->res)) {
		case IORESOURCE_IO:
			pp->io_size = resource_size(win->res);
			pp->io_bus_addr = win->res->start - win->offset;
			pp->io_base = pci_pio_to_address(win->res->start);
			break;
		case 0:
			pp->cfg0_size = resource_size(win->res);
			pp->cfg0_base = win->res->start;
			break;
		}
	}

	if (!pp->va_cfg0_base) {
		pp->va_cfg0_base = devm_pci_remap_cfgspace(dev,
					pp->cfg0_base, pp->cfg0_size);
		if (!pp->va_cfg0_base) {
			dev_err(dev, "Error with ioremap in function\n");
			return -ENOMEM;
		}
	}

	if (pp->cpu_pcie_addr_quirk) {
		pp->cfg0_base -= PCIE_CPU_BASE;
		pp->io_base   -= PCIE_CPU_BASE;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->has_its) {

		phys_addr_t pa = ALIGN_DOWN(virt_to_phys(pp), SZ_4K);

		ret = sunxi_allocate_msi_domains(pp);
		if (ret)
			return ret;

		sunxi_pcie_host_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4, lower_32_bits(pa));
		sunxi_pcie_host_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4, upper_32_bits(pa));
	}

	if (pp->ops->host_init)
		pp->ops->host_init(pp);

	bridge->sysdata = pp;
	bridge->ops = &sunxi_pcie_ops;

	ret = pci_host_probe(bridge);

	if (ret) {
		if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->has_its)
			sunxi_free_msi_domains(pp);

		dev_err(pp->dev, "Failed to probe host bridge\n");

		return ret;
	}

	return 0;
}

void sunxi_pcie_host_setup_rc(struct sunxi_pcie_port *pp)
{
	u32 val, i;
	int atu_idx = 0;
	struct resource_entry *entry;
	phys_addr_t mem_base;
	struct sunxi_pcie *pci = to_sunxi_pcie_from_pp(pp);

	sunxi_pcie_plat_set_rate(pci);

	/* setup RC BARs */
	sunxi_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0x4);
	sunxi_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0x0);

	/* setup interrupt pins */
	val = sunxi_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
	val &= PCIE_INTERRUPT_LINE_MASK;
	val |= PCIE_INTERRUPT_LINE_ENABLE;
	sunxi_pcie_writel_dbi(pci, PCI_INTERRUPT_LINE, val);

	/* setup bus numbers */
	val = sunxi_pcie_readl_dbi(pci, PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	sunxi_pcie_writel_dbi(pci, PCI_PRIMARY_BUS, val);

	/* setup command register */
	val = sunxi_pcie_readl_dbi(pci, PCI_COMMAND);

	val &= PCIE_HIGH16_MASK;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;

	sunxi_pcie_writel_dbi(pci, PCI_COMMAND, val);

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->has_its) {
		for (i = 0; i < 8; i++) {
			sunxi_pcie_host_wr_own_conf(pp, PCIE_MSI_INTR_ENABLE(i), 4, ~0);
		}
	}

	resource_list_for_each_entry(entry, &pp->bridge->windows) {
		if (resource_type(entry->res) != IORESOURCE_MEM)
			continue;

		if (pp->num_ob_windows <= ++atu_idx)
			break;

		if (pp->cpu_pcie_addr_quirk)
			mem_base = entry->res->start - PCIE_CPU_BASE;
		else
			mem_base = entry->res->start;

		sunxi_pcie_prog_outbound_atu(pp, atu_idx, PCIE_ATU_TYPE_MEM, mem_base,
						  entry->res->start - entry->offset,
						  resource_size(entry->res));
	}

	if (pp->io_size) {
		if (pp->num_ob_windows > ++atu_idx)
			sunxi_pcie_prog_outbound_atu(pp, atu_idx, PCIE_ATU_TYPE_IO, pp->io_base,
							pp->io_bus_addr, pp->io_size);
		else
			dev_err(pp->dev, "Resources exceed number of ATU entries (%d)",
							pp->num_ob_windows);
	}

	sunxi_pcie_host_wr_own_conf(pp, PCI_BASE_ADDRESS_0, 4, 0);

	sunxi_pcie_dbi_ro_wr_en(pci);

	sunxi_pcie_host_wr_own_conf(pp, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);

	sunxi_pcie_dbi_ro_wr_dis(pci);

	sunxi_pcie_host_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val |= PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_host_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_host_setup_rc);

static int sunxi_pcie_host_wait_for_speed_change(struct sunxi_pcie *pci)
{
	u32 tmp;
	unsigned int retries;

	for (retries = 0; retries < LINK_WAIT_MAX_RETRIE; retries++) {
		tmp = sunxi_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			return 0;
		usleep_range(SPEED_CHANGE_USLEEP_MIN, SPEED_CHANGE_USLEEP_MAX);
	}

	dev_err(pci->dev, "Speed change timeout\n");
	return -ETIMEDOUT;
}

int sunxi_pcie_host_speed_change(struct sunxi_pcie *pci, int gen)
{
	int val;
	int ret;

	sunxi_pcie_dbi_ro_wr_en(pci);
	val = sunxi_pcie_readl_dbi(pci, LINK_CONTROL2_LINK_STATUS2);
	val &= ~0xf;
	val |= gen;
	sunxi_pcie_writel_dbi(pci, LINK_CONTROL2_LINK_STATUS2, val);

	val = sunxi_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	val = sunxi_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	ret = sunxi_pcie_host_wait_for_speed_change(pci);
	if (!ret)
		dev_info(pci->dev, "PCIe speed of Gen%d\n", gen);
	else
		dev_info(pci->dev, "PCIe speed of Gen1\n");

	sunxi_pcie_dbi_ro_wr_dis(pci);
	return 0;
}

static void __sunxi_pcie_host_init(struct sunxi_pcie_port *pp)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_pp(pp);

	sunxi_pcie_plat_ltssm_disable(pci);

	if (!IS_ERR(pci->rst_gpio))
		gpiod_set_value(pci->rst_gpio, 0);
	msleep(100);
	if (!IS_ERR(pci->rst_gpio))
		gpiod_set_value(pci->rst_gpio, 1);

	sunxi_pcie_host_setup_rc(pp);

	sunxi_pcie_host_establish_link(pci);

	sunxi_pcie_host_speed_change(pci, pci->link_gen);
}

static int sunxi_pcie_host_link_up_status(struct sunxi_pcie_port *pp)
{
	u32 val;
	int ret;
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_pp(pp);

	val = sunxi_pcie_readl(pcie, PCIE_LINK_STAT);

	if ((val & RDLH_LINK_UP) && (val & SMLH_LINK_UP))
		ret = 1;
	else
		ret = 0;

	return ret;
}

static struct sunxi_pcie_host_ops sunxi_pcie_host_ops = {
	.link_up = sunxi_pcie_host_link_up_status,
	.host_init = __sunxi_pcie_host_init,
};

static int sunxi_pcie_host_wait_for_link(struct sunxi_pcie_port *pp)
{
	int retries;

	for (retries = 0; retries < LINK_WAIT_MAX_RETRIE; retries++) {
		if (sunxi_pcie_host_link_up(pp)) {
			dev_info(pp->dev, "pcie link up success\n");
			return 0;
		}
		usleep_range(LINK_WAIT_USLEEP_MIN, LINK_WAIT_USLEEP_MAX);
	}

	return -ETIMEDOUT;
}

int sunxi_pcie_host_establish_link(struct sunxi_pcie *pci)
{
	struct sunxi_pcie_port *pp = &pci->pp;

	if (sunxi_pcie_host_link_up(pp)) {
		dev_info(pci->dev, "pcie is already link up\n");
		return 0;
	}

	sunxi_pcie_plat_ltssm_enable(pci);

	return sunxi_pcie_host_wait_for_link(pp);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_host_establish_link);

static irqreturn_t sunxi_pcie_host_msi_irq_handler(int irq, void *arg)
{
	struct sunxi_pcie_port *pp = (struct sunxi_pcie_port *)arg;
	struct sunxi_pcie *pci = to_sunxi_pcie_from_pp(pp);
	unsigned long val;
	int i, pos;
	u32 status;
	irqreturn_t ret = IRQ_NONE;

	for (i = 0; i < MAX_MSI_CTRLS; i++) {
		status = sunxi_pcie_readl_dbi(pci, PCIE_MSI_INTR_STATUS + (i * MSI_REG_CTRL_BLOCK_SIZE));

		if (!status)
			continue;

		ret = IRQ_HANDLED;
		pos = 0;
		val = status;
		while ((pos = find_next_bit(&val, MAX_MSI_IRQS_PER_CTRL, pos)) != MAX_MSI_IRQS_PER_CTRL) {

			generic_handle_domain_irq(pp->irq_domain, (i * MAX_MSI_IRQS_PER_CTRL) + pos);

			sunxi_pcie_writel_dbi(pci,
					PCIE_MSI_INTR_STATUS + (i * MSI_REG_CTRL_BLOCK_SIZE), 1 << pos);
			pos++;
		}
	}

	return ret;
}

int sunxi_pcie_host_add_port(struct sunxi_pcie *pci, struct platform_device *pdev)
{
	struct sunxi_pcie_port *pp = &pci->pp;
	int ret;

	ret = of_property_read_u32(pp->dev->of_node, "num-ob-windows", &pp->num_ob_windows);
	if (ret) {
		dev_err(&pdev->dev, "failed to parse num-ob-windows\n");
		return -EINVAL;
	}

	pp->has_its = device_property_read_bool(&pdev->dev, "msi-map");

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->has_its) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq < 0)
			return pp->msi_irq;

		ret = devm_request_irq(&pdev->dev, pp->msi_irq, sunxi_pcie_host_msi_irq_handler,
					IRQF_SHARED, "pcie-msi", pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request MSI IRQ\n");
			return ret;
		}
	}

	pp->ops = &sunxi_pcie_host_ops;
	raw_spin_lock_init(&pp->lock);

	ret = sunxi_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_host_add_port);

void sunxi_pcie_host_remove_port(struct sunxi_pcie *pci)
{
	struct sunxi_pcie_port *pp = &pci->pp;

	if (pp->bridge->bus) {
		pci_stop_root_bus(pp->bridge->bus);
		pci_remove_root_bus(pp->bridge->bus);
	}

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->has_its) {
		irq_domain_remove(pp->msi_domain);
		irq_domain_remove(pp->irq_domain);
	}
}
EXPORT_SYMBOL_GPL(sunxi_pcie_host_remove_port);

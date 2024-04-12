// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * SUNXI PCIe Endpoint controller driver
 *
 * Copyright (c) 2007-2022 Allwinnertech Co., Ltd.
 * Author: songjundong <songjundong@allwinnertech.com>
 */

#include <linux/configfs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pci-epc.h>
#include <linux/platform_device.h>
#include <linux/pci-epf.h>
#include <linux/sizes.h>

#include "pci.h"
#include "pcie-sunxi.h"


static void sunxi_pcie_setup_ep(struct sunxi_pcie *pci)
{
	sunxi_pcie_plat_set_rate(pci);
}

static unsigned int sunxi_pcie_ep_func_select(struct sunxi_pcie_ep *ep, u8 func_no)
{
	unsigned int func_offset = 0;

	if (ep->ops->func_conf_select)
		func_offset = ep->ops->func_conf_select(ep, func_no);

	return func_offset;
}

static u8 __sunxi_pcie_ep_find_next_cap(struct sunxi_pcie_ep *ep, u8 func_no,
						u8 cap_ptr, u8 cap)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	unsigned int func_offset = 0;
	u8 cap_id, next_cap_ptr;
	u16 reg;

	if (!cap_ptr)
		return 0;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	reg = sunxi_pcie_readw_dbi(pci, func_offset + cap_ptr);
	cap_id = (reg & 0x00ff);

	if (cap_id > PCI_CAP_ID_MAX)
		return 0;

	if (cap_id == cap)
		return cap_ptr;

	next_cap_ptr = (reg & 0xff00) >> 8;
	return __sunxi_pcie_ep_find_next_cap(ep, func_no, next_cap_ptr, cap);
}

static u8 sunxi_pcie_ep_find_capability(struct sunxi_pcie_ep *ep, u8 func_no, u8 cap)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	unsigned int func_offset = 0;
	u8 next_cap_ptr;
	u16 reg;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	reg = sunxi_pcie_readw_dbi(pci, func_offset + PCI_CAPABILITY_LIST);
	next_cap_ptr = (reg & 0x00ff);

	return __sunxi_pcie_ep_find_next_cap(ep, func_no, next_cap_ptr, cap);
}

struct sunxi_pcie_ep_func *sunxi_pcie_ep_get_func_from_ep(struct sunxi_pcie_ep *ep, u8 func_no)
{
	struct sunxi_pcie_ep_func *ep_func;

	list_for_each_entry(ep_func, &ep->func_list, list) {
		if (ep_func->func_no == func_no)
			return ep_func;
	}

	return NULL;
}

static void __sunxi_pcie_ep_reset_bar(struct sunxi_pcie *pci, u8 func_no,
					enum pci_barno bar, int flags)
{
	u32 reg;
	unsigned int func_offset = 0;
	struct sunxi_pcie_ep *ep = &pci->ep;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	reg = func_offset + PCI_BASE_ADDRESS_0 + (4 * bar);

	sunxi_pcie_dbi_ro_wr_en(pci);
	sunxi_pcie_writel_dbi(pci, reg, 0x0);

	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64) {
		sunxi_pcie_writel_dbi(pci, reg + 4, 0x0);
	}
	sunxi_pcie_dbi_ro_wr_dis(pci);
}

static void sunxi_pcie_ep_reset_bar(struct sunxi_pcie *pci, enum pci_barno bar)
{
	u8 func_no, funcs;

	funcs = pci->ep.epc->max_functions;

	for (func_no = 0; func_no < funcs; func_no++)
		__sunxi_pcie_ep_reset_bar(pci, func_no, bar, 0);
}

static void sunxi_pcie_prog_inbound_atu(struct sunxi_pcie *pci, u8 func_no, int index,
						int type, u64 cpu_addr, u8 bar)
{
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_LOWER_TARGET_INBOUND(index),
					lower_32_bits(cpu_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_UPPER_TARGET_INBOUND(index),
					upper_32_bits(cpu_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR1_INBOUND(index),
					type | PCIE_ATU_FUNC_NUM(func_no));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR2_INBOUND(index),
					PCIE_ATU_ENABLE | PCIE_ATU_FUNC_NUM_MATCH_EN |
					PCIE_ATU_BAR_MODE_ENABLE | (bar << 8));
}

static void sunxi_pcie_prog_ep_outbound_atu(struct sunxi_pcie *pci, u8 func_no, int index,
						int type, u64 cpu_addr, u64 pci_addr,
						u64 size)
{
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_LOWER_BASE_OUTBOUND(index),
					lower_32_bits(cpu_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_UPPER_BASE_OUTBOUND(index),
					upper_32_bits(cpu_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_LIMIT_OUTBOUND(index),
					lower_32_bits(cpu_addr + size - 1));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_LOWER_TARGET_OUTBOUND(index),
					lower_32_bits(pci_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_UPPER_TARGET_OUTBOUND(index), upper_32_bits(pci_addr));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR1_OUTBOUND(index),
					type | PCIE_ATU_FUNC_NUM(func_no));
	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR2_OUTBOUND(index),
					PCIE_ATU_ENABLE);
}

static int sunxi_pcie_ep_inbound_atu(struct sunxi_pcie_ep *ep, u8 func_no, int type,
						dma_addr_t cpu_addr, enum pci_barno bar)
{
	u32 free_win;
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);

	if (!ep->bar_to_atu[bar])
		free_win = find_first_zero_bit(ep->ib_window_map, ep->num_ib_windows);
	else
		free_win = ep->bar_to_atu[bar];

	if (free_win >= ep->num_ib_windows) {
		dev_err(&pci->dev, "No free inbound window\n");
		return -EINVAL;
	}

	sunxi_pcie_prog_inbound_atu(pci, func_no, free_win, type,
						cpu_addr, bar);

	ep->bar_to_atu[bar] = free_win;
	set_bit(free_win, ep->ib_window_map);

	return 0;
}

static int sunxi_pcie_ep_outbound_atu(struct sunxi_pcie_ep *ep, u8 func_no,
					phys_addr_t phys_addr,
					u64 pci_addr, size_t size)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	u32 free_win;

	free_win = find_first_zero_bit(ep->ob_window_map, ep->num_ob_windows);
	if (free_win >= ep->num_ob_windows) {
		dev_err(&pci->dev, "No free outbound window\n");
		return -EINVAL;
	}

	sunxi_pcie_prog_ep_outbound_atu(pci, func_no, free_win, PCIE_ATU_TYPE_MEM,
					   phys_addr, pci_addr, size);

	set_bit(free_win, ep->ob_window_map);
	ep->outbound_addr[free_win] = phys_addr;

	return 0;
}

static int sunxi_pcie_find_index(struct sunxi_pcie_ep *ep, phys_addr_t addr,
				u32 *atu_index)
{
	u32 index;

	for (index = 0; index < ep->num_ob_windows; index++) {
		if (ep->outbound_addr[index] != addr)
			continue;
		*atu_index = index;
		return 0;
	}

	return -EINVAL;
}

static void sunxi_ep_init_bar(struct sunxi_pcie_ep *ep)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	struct sunxi_pcie_ep_func *ep_func;
	enum pci_barno bar;

	ep_func = sunxi_pcie_ep_get_func_from_ep(ep, 0);
	if (!ep_func)
		return;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
		sunxi_pcie_ep_reset_bar(pci, bar);
}

static int sunxi_pcie_start_link(struct sunxi_pcie *pci)
{
	sunxi_pcie_plat_ltssm_enable(pci);

	return 0;
}

static void sunxi_pcie_stop_link(struct sunxi_pcie *pci)
{
	sunxi_pcie_plat_ltssm_disable(pci);
}

static const struct of_device_id sunxi_pcie_ep_of_match[] = {
	{ .compatible = "sunxi,sun55iw3-pcie-ep"},
	{},
};

static int sunxi_pcie_ep_write_header(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
					 struct pci_epf_header *hdr)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	unsigned int func_offset = 0;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	sunxi_pcie_dbi_ro_wr_en(pci);
	sunxi_pcie_writew_dbi(pci, func_offset + PCI_VENDOR_ID, hdr->vendorid);
	sunxi_pcie_writew_dbi(pci, func_offset + PCI_DEVICE_ID, hdr->deviceid);
	sunxi_pcie_writeb_dbi(pci, func_offset + PCI_REVISION_ID, hdr->revid);
	sunxi_pcie_writeb_dbi(pci, func_offset + PCI_CLASS_PROG, hdr->progif_code);
	sunxi_pcie_writew_dbi(pci, func_offset + PCI_CLASS_DEVICE,
					hdr->subclass_code | hdr->baseclass_code << 8);
	sunxi_pcie_writeb_dbi(pci, func_offset + PCI_CACHE_LINE_SIZE,
					hdr->cache_line_size);
	sunxi_pcie_writew_dbi(pci, func_offset + PCI_SUBSYSTEM_VENDOR_ID,
					hdr->subsys_vendor_id);
	sunxi_pcie_writew_dbi(pci, func_offset + PCI_SUBSYSTEM_ID, hdr->subsys_id);
	sunxi_pcie_writeb_dbi(pci, func_offset + PCI_INTERRUPT_PIN,
					hdr->interrupt_pin);
	sunxi_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int sunxi_pcie_ep_set_bar(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
					struct pci_epf_bar *epf_bar)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	enum pci_barno bar = epf_bar->barno;
	size_t size = epf_bar->size;
	int flags = epf_bar->flags;
	unsigned int func_offset = 0;
	int ret, type;
	u32 reg;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	reg = PCI_BASE_ADDRESS_0 + (4 * bar) + func_offset;

	if (!(flags & PCI_BASE_ADDRESS_SPACE))
		type = PCIE_ATU_TYPE_MEM;
	else
		type = PCIE_ATU_TYPE_IO;

	ret = sunxi_pcie_ep_inbound_atu(ep, func_no, type, epf_bar->phys_addr, bar);
	if (ret)
		return ret;

	if (ep->epf_bar[bar])
		return 0;

	sunxi_pcie_dbi_ro_wr_en(pci);

	sunxi_pcie_writel_dbi(pci, reg, flags | lower_32_bits(size - 1));

	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64) {
		sunxi_pcie_writel_dbi(pci, reg + 4, upper_32_bits(size - 1));
	}

	ep->epf_bar[bar] = epf_bar;
	sunxi_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static void sunxi_pcie_ep_clear_bar(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
				       struct pci_epf_bar *epf_bar)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	enum pci_barno bar = epf_bar->barno;
	u32 index = ep->bar_to_atu[bar];

	__sunxi_pcie_ep_reset_bar(pci, func_no, bar, epf_bar->flags);

	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR2_INBOUND(index), 0);
	clear_bit(index, ep->ib_window_map);
	ep->epf_bar[bar] = NULL;
	ep->bar_to_atu[bar] = 0;
}

static void sunxi_pcie_ep_unmap_addr(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
				  phys_addr_t addr)
{
	int ret;
	u32 atu_index;
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);

	ret = sunxi_pcie_find_index(ep, addr, &atu_index);
	if (ret < 0)
		return;

	sunxi_pcie_writel_dbi(pci, PCIE_ATU_CR2_OUTBOUND(atu_index), 0);
	clear_bit(atu_index, ep->ob_window_map);
}

static int sunxi_pcie_ep_map_addr(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
			       phys_addr_t cpu_addr, u64 pci_addr, size_t size)
{
	int ret;
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);

	ret = sunxi_pcie_ep_outbound_atu(ep, func_no, cpu_addr, pci_addr, size);
	if (ret) {
		dev_err(&pci->dev, "Failed to enable address\n");
		return ret;
	}

	return 0;
}

static int sunxi_pcie_ep_set_msi(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
				    u8 interrupts)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	u32 val, reg;
	unsigned int func_offset = 0;
	struct sunxi_pcie_ep_func *ep_func;

	ep_func = sunxi_pcie_ep_get_func_from_ep(ep, func_no);
	if (!ep_func || !ep_func->msi_cap)
		return -EINVAL;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	reg = ep_func->msi_cap + func_offset + PCI_MSI_FLAGS;
	val = sunxi_pcie_readw_dbi(pci, reg);
	val &= ~PCI_MSI_FLAGS_QMASK;
	val |= (interrupts << 1) & PCI_MSI_FLAGS_QMASK;
	sunxi_pcie_dbi_ro_wr_en(pci);
	sunxi_pcie_writew_dbi(pci, reg, val);
	sunxi_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int sunxi_pcie_ep_get_msi(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	u32 val, reg;
	unsigned int func_offset = 0;
	struct sunxi_pcie_ep_func *ep_func;

	ep_func = sunxi_pcie_ep_get_func_from_ep(ep, func_no);
	if (!ep_func || !ep_func->msi_cap)
		return -EINVAL;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	reg = ep_func->msi_cap + func_offset + PCI_MSI_FLAGS;
	val = sunxi_pcie_readw_dbi(pci, reg);
	if (!(val & PCI_MSI_FLAGS_ENABLE))
		return -EINVAL;

	val = (val & PCI_MSI_FLAGS_QSIZE) >> 4;

	return val;
}

static int sunxi_pcie_ep_send_msi_irq(struct sunxi_pcie_ep *ep, u8 func_no,
					u8 interrupt_num)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	struct sunxi_pcie_ep_func *ep_func;
	struct pci_epc *epc = ep->epc;
	unsigned int aligned_offset;
	unsigned int func_offset = 0;
	u16 msg_ctrl, msg_data;
	u32 msg_addr_lower, msg_addr_upper, reg;
	u64 msg_addr;
	bool has_upper;
	int ret;

	ep_func = sunxi_pcie_ep_get_func_from_ep(ep, func_no);
	if (!ep_func || !ep_func->msi_cap)
		return -EINVAL;

	func_offset = sunxi_pcie_ep_func_select(ep, func_no);

	/* Raise MSI per the PCI Local Bus Specification Revision 3.0, 6.8.1. */
	reg = ep_func->msi_cap + func_offset + PCI_MSI_FLAGS;
	msg_ctrl = sunxi_pcie_readw_dbi(pci, reg);
	has_upper = !!(msg_ctrl & PCI_MSI_FLAGS_64BIT);
	reg = ep_func->msi_cap + func_offset + PCI_MSI_ADDRESS_LO;
	msg_addr_lower = sunxi_pcie_readl_dbi(pci, reg);
	if (has_upper) {
		reg = ep_func->msi_cap + func_offset + PCI_MSI_ADDRESS_HI;
		msg_addr_upper = sunxi_pcie_readl_dbi(pci, reg);
		reg = ep_func->msi_cap + func_offset + PCI_MSI_DATA_64;
		msg_data = sunxi_pcie_readw_dbi(pci, reg);
	} else {
		msg_addr_upper = 0;
		reg = ep_func->msi_cap + func_offset + PCI_MSI_DATA_32;
		msg_data = sunxi_pcie_readw_dbi(pci, reg);
	}
	aligned_offset = msg_addr_lower & (epc->mem->window.page_size - 1);
	msg_addr = ((u64)msg_addr_upper) << 32 |
			(msg_addr_lower & ~aligned_offset);
	ret = sunxi_pcie_ep_map_addr(epc, func_no, 0, ep->msi_mem_phys, msg_addr,
				  epc->mem->window.page_size);
	if (ret)
		return ret;

	writel(msg_data | (interrupt_num - 1), ep->msi_mem + aligned_offset);

	sunxi_pcie_ep_unmap_addr(epc, func_no, 0, ep->msi_mem_phys);

	return 0;
}

static int sunxi_pcie_ep_raise_irq(struct pci_epc *epc, u8 fn, u8 vfn,
				      enum pci_epc_irq_type type,
				      u16 interrupt_num)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);

	switch (type) {
	case PCI_EPC_IRQ_MSI:
		return sunxi_pcie_ep_send_msi_irq(ep, fn, interrupt_num);
	default:
		return -EINVAL;
	}
}

static int sunxi_pcie_ep_start(struct pci_epc *epc)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);

	/* Whether to enable a bit. Need to pay more attention to it. */
	sunxi_pcie_start_link(pci);

	return 0;
}

static void sunxi_pcie_ep_stop(struct pci_epc *epc)
{
	struct sunxi_pcie_ep *ep = epc_get_drvdata(epc);
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);

	/* Whether to disable a bit. Need to pay more attention to it. */
	sunxi_pcie_stop_link(pci);
}

static const struct pci_epc_features sunxi_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable     = true,
	.msix_capable    = false,
};

static const struct pci_epc_features *sunxi_pcie_ep_get_features(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	return &sunxi_pcie_epc_features;
}


static const struct pci_epc_ops sunxi_pcie_epc_ops = {
	.write_header	= sunxi_pcie_ep_write_header,
	.set_bar	= sunxi_pcie_ep_set_bar,
	.clear_bar	= sunxi_pcie_ep_clear_bar,
	.map_addr	= sunxi_pcie_ep_map_addr,
	.unmap_addr	= sunxi_pcie_ep_unmap_addr,
	.set_msi	= sunxi_pcie_ep_set_msi,
	.get_msi	= sunxi_pcie_ep_get_msi,
	.raise_irq	= sunxi_pcie_ep_raise_irq,
	.start		= sunxi_pcie_ep_start,
	.stop		= sunxi_pcie_ep_stop,
	.get_features	= sunxi_pcie_ep_get_features,
};

static int sunxi_pcie_parse_ep_dts(struct sunxi_pcie_ep *ep)
{
	int ret;
	void *addr;
	struct resource *res;
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	struct device *dev = &pci->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res) {
		dev_err(dev, "unable to read *addr_space* property\n");
		return -EINVAL;
	}

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);
	ep->page_size = SZ_16K;

	ret = of_property_read_u32(np, "num-ib-windows",
					&ep->num_ib_windows);
	if (ret < 0) {
		dev_err(dev, "unable to read *num-ib-windows* property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "num-ob-windows",
					&ep->num_ob_windows);
	if (ret < 0) {
		dev_err(dev, "unable to read *num-ob-windows* property\n");
		return ret;
	}

	ep->ib_window_map = devm_bitmap_zalloc(dev, ep->num_ib_windows,
							GFP_KERNEL);
	if (!ep->ib_window_map)
		return -ENOMEM;

	ep->ob_window_map = devm_bitmap_zalloc(dev, ep->num_ob_windows,
							GFP_KERNEL);
	if (!ep->ob_window_map)
		return -ENOMEM;

	addr = devm_kcalloc(dev, ep->num_ob_windows, sizeof(phys_addr_t),
							GFP_KERNEL);
	if (!addr)
		return -ENOMEM;
	ep->outbound_addr = addr;

	return 0;
}

static unsigned int sunxi_pcie_ep_find_ext_capability(struct sunxi_pcie *pci, int cap)
{
	u32 header;
	int pos = PCI_CFG_SPACE_SIZE;

	while (pos) {
		header = sunxi_pcie_readl_dbi(pci, pos);
		if (PCI_EXT_CAP_ID(header) == cap)
			return pos;

		pos = PCI_EXT_CAP_NEXT(header);
		if (!pos)
			break;
	}

	return 0;
}

int sunxi_plat_ep_init_end(struct sunxi_pcie_ep *ep)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_ep(ep);
	unsigned int offset;
	unsigned int nbars;
	u8 hdr_type;
	u32 reg;
	int i;

	hdr_type = sunxi_pcie_readb_dbi(pci, PCI_HEADER_TYPE) &
							PCI_HEADER_TYPE_MASK;
	if (hdr_type != PCI_HEADER_TYPE_NORMAL) {
		dev_err(&pci->dev,
			"PCIe controller is not set to EP mode (hdr_type:0x%x)!\n",
			hdr_type);
		return -EIO;
	}

	offset = sunxi_pcie_ep_find_ext_capability(pci, PCI_EXT_CAP_ID_REBAR);

	sunxi_pcie_dbi_ro_wr_en(pci);

	if (offset) {
		reg = sunxi_pcie_readl_dbi(pci, offset + PCI_REBAR_CTRL);
		nbars = (reg & PCI_REBAR_CTRL_NBAR_MASK) >>
			PCI_REBAR_CTRL_NBAR_SHIFT;

		for (i = 0; i < nbars; i++, offset += PCI_REBAR_CTRL)
			sunxi_pcie_writel_dbi(pci, offset + PCI_REBAR_CAP, PCIE_EP_REBAR_SIZE_32M);
	}

	sunxi_pcie_setup_ep(pci);
	sunxi_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

int sunxi_pcie_ep_init(struct sunxi_pcie *pci)
{
	int ret;
	u8 func_no;
	struct pci_epc *epc;
	struct sunxi_pcie_ep *ep = &pci->ep;
	struct device *dev = &pci->dev;
	struct device_node *np = dev->of_node;
	struct sunxi_pcie_ep_func *ep_func;

	INIT_LIST_HEAD(&ep->func_list);

	ret = sunxi_pcie_parse_ep_dts(ep);
	if (ret) {
		dev_err(dev, "failed to parse ep dts\n");
		return ret;
	}

	epc = devm_pci_epc_create(dev, &sunxi_pcie_epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		return PTR_ERR(epc);
	}

	ep->epc = epc;
	epc_set_drvdata(epc, ep);

	ret = of_property_read_u8(np, "max-functions", &epc->max_functions);
	if (ret < 0)
		epc->max_functions = 1;

	for (func_no = 0; func_no < epc->max_functions; func_no++) {
		ep_func = devm_kzalloc(dev, sizeof(*ep_func), GFP_KERNEL);
		if (!ep_func)
			return -ENOMEM;

		ep_func->func_no = func_no;
		ep_func->msi_cap = sunxi_pcie_ep_find_capability(ep, func_no,
							      PCI_CAP_ID_MSI);
		ep_func->msix_cap = sunxi_pcie_ep_find_capability(ep, func_no,
							       PCI_CAP_ID_MSIX);

		list_add_tail(&ep_func->list, &ep->func_list);
	}

	sunxi_ep_init_bar(ep);

	ret = pci_epc_mem_init(epc, ep->phys_base, ep->addr_size,
			       ep->page_size);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize address space\n");
		return ret;
	}

	ep->msi_mem = pci_epc_mem_alloc_addr(epc, &ep->msi_mem_phys,
					     epc->mem->window.page_size);
	if (!ep->msi_mem) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to reserve memory for MSI\n");
		goto err_exit_epc_mem;
	}

	ret = sunxi_plat_ep_init_end(ep);
	if (ret)
		goto err_free_epc_mem;

	return 0;

err_free_epc_mem:
	pci_epc_mem_free_addr(epc, ep->msi_mem_phys, ep->msi_mem, epc->mem->window.page_size);

err_exit_epc_mem:
	pci_epc_mem_exit(epc);

	return ret;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_ep_init);

void sunxi_pcie_ep_deinit(struct sunxi_pcie *pci)
{
	struct pci_epc *epc = &pci->ep->epc;

	pci_epc_mem_exit(epc);
	pci_epc_mem_free_addr(epc, ep->msi_mem_phys, ep->msi_mem, epc->mem->window.page_size);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_ep_deinit);

# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_USB_PHYTIUM) += phytium-usb.o

phytium-usb-y := core.o dma.o platform.o

phytium-usb-$(CONFIG_USB_PHYTIUM_HOST)	+= host.o
phytium-usb-$(CONFIG_USB_PHYTIUM_GADGET) += gadget.o

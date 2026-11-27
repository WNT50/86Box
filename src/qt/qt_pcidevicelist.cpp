#include "qt_pcidevicelist.hpp"
#include "ui_qt_pcidevicelist.h"

extern "C" {
#include <86box/86box.h>
#include <86box/video.h>
#include <86box/pci.h>
#include <86box/plat.h>
}

static QString
pci_type_name(uint8_t type)
{
    switch (type) {
        case PCI_CARD_NORTHBRIDGE:
            return QObject::tr("Host bridge");
        case PCI_CARD_NORTHBRIDGE_SEC:
            return QObject::tr("Second host bridge");
        case PCI_CARD_AGPBRIDGE:
            return QObject::tr("AGP bridge");
        case PCI_CARD_SOUTHBRIDGE:
            return QObject::tr("ISA/LPC bridge");
        case PCI_CARD_SOUTHBRIDGE_IDE:
            return QObject::tr("IDE controller");
        case PCI_CARD_SOUTHBRIDGE_PMU:
            return QObject::tr("Power management");
        case PCI_CARD_SOUTHBRIDGE_USB:
            return QObject::tr("USB controller");
        case PCI_CARD_AGP:
            return QObject::tr("AGP card");
        case PCI_CARD_NORMAL:
            return QObject::tr("Add-in card");
        case PCI_CARD_VIDEO:
            return QObject::tr("Video card");
        case PCI_CARD_HANGUL:
            return QObject::tr("Hangul card");
        case PCI_CARD_IDE:
            return QObject::tr("IDE card");
        case PCI_CARD_SCSI:
            return QObject::tr("SCSI card");
        case PCI_CARD_SOUND:
            return QObject::tr("Sound card");
        case PCI_CARD_MODEM:
            return QObject::tr("Modem");
        case PCI_CARD_NETWORK:
            return QObject::tr("Network card");
        case PCI_CARD_UART:
            return QObject::tr("Serial card");
        case PCI_CARD_USB:
            return QObject::tr("USB card");
        case PCI_CARD_BRIDGE:
            return QObject::tr("PCI bridge");
        default:
            return QObject::tr("Unknown");
    }
}

PCIDeviceList::PCIDeviceList(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::PCIDeviceList)
{
    ui->setupUi(this);

    startblit();
    int shown = 0;
    for (int i = 0; i < pci_get_nr_slots(); i++) {
        uint8_t  bus;
        uint8_t  dev;
        uint8_t  type;
        uint8_t  irq_routing[PCI_INT_PINS_NUM];
        uint16_t vendor;
        uint16_t device;

        if (!pci_get_slot_info(i, &bus, &dev, &type, irq_routing, &vendor, &device))
            continue;

        /* The device number is what the guest sees at 0xCF8; bus + dev identify
           the card unambiguously even when two slots share a device number. */
        /* The device may know its own name; fall back to the slot type. */
        const char   *device_name = pci_get_slot_device_name(i);
        const QString kind = device_name ? QString::fromUtf8(device_name) : pci_type_name(type);
        const uint8_t slot_number = pci_get_slot_number(i);

        /* Add-in slots are numbered in the order the machine registers them,
           which is the order they are printed on the board; on-board devices
           carry no number and are marked as such. */
        const QString card = slot_number ?
                                 QString::asprintf("Slot %u: Bus %u, device %u, %04X:%04X  (%s)",
                                                   slot_number, bus, dev, vendor, device,
                                                   kind.toUtf8().constData()) :
                                 QString::asprintf("Onboard: Bus %u, device %u, %04X:%04X  (%s)",
                                                   bus, dev, vendor, device,
                                                   kind.toUtf8().constData());

        /* One line per wired interrupt pin, each repeating the card it belongs
           to; a pin the slot does not wire up is left out entirely. A card with
           no wired pin at all keeps a single line of its own. Cards are
           separated by a divider so the groups stay readable. */
        QStringList lines;

        for (int pin = 0; pin < PCI_INT_PINS_NUM; pin++) {
            const uint8_t routing = irq_routing[pin];
            QString       target;

            if (routing == 0x00)
                continue;

            if (routing >= PCI_DIRQ_BASE) {
                target = QString::asprintf("IRQ %u (direct)", routing & 0x0f);
            } else if (routing >= PCI_IIRQ_BASE) {
                target = QStringLiteral("chipset internal PIRQ");
            } else if (routing <= PCI_IRQS_NUM) {
                const QChar   pirq('A' + (routing - 1));
                const uint8_t irq = pci_get_irq_line(i, PCI_INTA + pin);
                if (irq == 0xff)
                    target = QStringLiteral("PIRQ %1 (not routed)").arg(pirq);
                else
                    target = QStringLiteral("PIRQ %1 → IRQ %2").arg(pirq).arg(irq);
            } else {
                continue;
            }

            lines.append(
                QStringLiteral("%1    INT%2 → %3").arg(card).arg(QChar('A' + pin)).arg(target));
        }

        if (lines.isEmpty())
            lines.append(card);

        if (shown > 0) {
            auto *separator = new QListWidgetItem(QStringLiteral("────────────────────────"));
            separator->setFlags(Qt::NoItemFlags);
            ui->listWidget->addItem(separator);
        }

        for (const QString &line : lines)
            ui->listWidget->addItem(line);

        shown++;
    }

    if (shown == 0) {
        ui->listWidget->addItem(QObject::tr("No PCI devices."));
        ui->listWidget->setDisabled(true);
    }

    endblit();
}

PCIDeviceList::~PCIDeviceList()
{
    delete ui;
}

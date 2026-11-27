#ifndef QT_PCIDEVICELIST_HPP
#define QT_PCIDEVICELIST_HPP

#include <QDialog>

namespace Ui {
class PCIDeviceList;
}

class PCIDeviceList : public QDialog {
    Q_OBJECT

public:
    explicit PCIDeviceList(QWidget *parent = nullptr);
    ~PCIDeviceList();

private:
    Ui::PCIDeviceList *ui;
};

#endif // QT_PCIDEVICELIST_HPP

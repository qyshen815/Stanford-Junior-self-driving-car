/********************************************************************************
** Form generated from reading UI file 'trajectory2Lane_qtgui.ui'
**
** Created: Sun Nov 15 16:33:13 2009
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef TRAJECTORY2LANE_QTGUI_H
#define TRAJECTORY2LANE_QTGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSlider>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_traceDialog
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSlider *horizontalSlider;
    QDoubleSpinBox *doubleSpinBox;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *traceDialog)
    {
        if (traceDialog->objectName().isEmpty())
            traceDialog->setObjectName(QString::fromUtf8("traceDialog"));
        traceDialog->resize(400, 300);
        verticalLayout = new QVBoxLayout(traceDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(traceDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        horizontalSlider = new QSlider(groupBox);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(horizontalSlider);

        doubleSpinBox = new QDoubleSpinBox(groupBox);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setMaximum(1);

        horizontalLayout->addWidget(doubleSpinBox);


        gridLayout->addLayout(horizontalLayout, 0, 0, 1, 1);


        verticalLayout->addWidget(groupBox);

        buttonBox = new QDialogButtonBox(traceDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(traceDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), traceDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), traceDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(traceDialog);
    } // setupUi

    void retranslateUi(QDialog *traceDialog)
    {
        traceDialog->setWindowTitle(QApplication::translate("traceDialog", "Convert Trajectory to Lane", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("traceDialog", "Point Removal Parameters", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("traceDialog", "Distance Threshold / cm", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class traceDialog: public Ui_traceDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // TRAJECTORY2LANE_QTGUI_H
